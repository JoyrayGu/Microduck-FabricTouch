"""@file protocol.py
@brief 电子触觉皮肤的协议帧编解码与流式解析。
@details
    该模块实现设备帧结构、校验逻辑、下行命令构造、协议流式扫描以及
    USB ASCII 明文压力矩阵解析，保证上层 Session 和 Controller 能以统一方式处理数据。

采集设备（电子触觉皮肤）蓝牙/USB 通讯协议实现。

协议摘自《采集设备使用说明-蓝牙_V5.0》：

    帧格式
    +--------+--------+--------+--------+----------+--------+--------+
    |  帧头  |  长度  | 指令位 | 状态字 |  数据包  | 校验字 |  帧尾  |
    |  0x98  | 2 Byte | 1 Byte | 1 Byte |  n Byte  | 1 Byte |  0x81  |
    +--------+--------+--------+--------+----------+--------+--------+

    * 长度   : 2 字节，高位在前低位在后，等于「数据包」的字节数
    * 指令位 : 见 Command
    * 状态字 : 0x00 = 上位机（主设备）下发；0x01 = 设备（从设备）应答
    * 校验字 : 帧头 ~ 数据包 所有字节之和，高位舍弃（即 & 0xFF）
    * 帧尾   : 0x81

两种上行通道（用 0x0D 指令切换，参数见 TransportMode）：

**蓝牙 BLE（设备出厂默认）**
    Service        0xFFE0
    Characteristics 0xFFE1  Write Without Response  （下行 APP -> UART）
    Characteristics 0xFFE2  Notify                  （上行 UART -> APP）

**USB（CDC / 虚拟串口，本程序的默认通道）**
    * 与蓝牙共用同一套帧格式与指令，只是不再有那些无效前导字节；
    * 文档 3.2 注 2：USB 模式下 **单次采集(0x02)** 的返回是「计算后的压强
      值（单位 kPa）」的 **ASCII 明文**，每行 32 个点、空格分隔、保留 3 位
      小数、共 32 行 —— 既不是二进制帧也不是「帧号 + 原始 ADC 值」；
    * 文档 3.7 注：设备默认蓝牙模式，**掉电重启后会自动恢复蓝牙模式**，
      所以走 USB 时每次连接都必须重新下发 0x0D = 0x00。

注意：默认蓝牙模式下由于数据量大，设备发出的通知包前 N 个字节可能是
无效数据，因此接收端必须做「流式扫描 + 帧头帧尾匹配」而不是按包定长解析，
本模块的 StreamParser 已完成该处理；USB 通道上明文与帧可能混在一起，
由 ``AsciiTelemetryParser`` 负责把明文压力块还原成与二进制帧等价的网格。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Iterable, List, Optional, Sequence, Tuple

# --------------------------------------------------------------------------- #
# 常量
# --------------------------------------------------------------------------- #

FRAME_HEAD = 0x98
FRAME_TAIL = 0x81

DEV_HOST = 0x00  # 上位机 / 主设备
DEV_SLAVE = 0x01  # 采集设备 / 从设备

# 单帧数据包允许的最大字节数。32x32 阵列 = 2 + 1024*2 = 2050 字节，
# 所以必须留到 4KB，否则大阵列的帧会被当成「非法长度」而整帧丢弃。
MAX_PAYLOAD_LEN = 4096
# 最长帧 = 帧头1 + 长度2 + 指令1 + 状态1 + 数据 + 校验1 + 帧尾1
MAX_FRAME_LEN = MAX_PAYLOAD_LEN + 7

BLE_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
BLE_WRITE_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
BLE_NOTIFY_UUID = "0000ffe2-0000-1000-8000-00805f9b34fb"

# 默认采集阵列（原始通道数）
DEFAULT_ROWS = 32
DEFAULT_COLS = 32
DEFAULT_THRESHOLD = 100  # 文档示例 98 00 01 0C 00 64 09 81（0x64 = 100）
DEFAULT_FRAME_RATE_HZ = 50.0  # 设备未文档化帧率，仅用于估算超时

#: USB 模式下 ASCII 明文压力块的原生尺寸（文档 3.2 注 2：32 行 x 32 点）
ASCII_GRID_SIZE = 32

#: USB 虚拟串口的常见波特率（文档未给出，连接时按顺序试探）
USB_BAUD_CANDIDATES = (115200, 921600, 460800, 230400, 57600, 38400, 9600)


class Command:
    """设备指令一览表（指令位）。"""

    CONTINUOUS = 0x01  # 连续采集，数据保存至 flash
    SINGLE = 0x02  # 单次采集，数据不保存
    STOP = 0x03  # 停止采集
    READ_BATTERY = 0x07  # 读取电量
    SET_ARRAY = 0x08  # 设置采集阵列（数据 = 行数, 列数）
    DEVICE_INFO = 0x0B  # 设备信息打印（ASCII 明码）
    SET_THRESHOLD = 0x0C  # 设置采集阈值（数据 = 阈值，最大 255）
    SET_MODE = 0x0D  # 设置数据传输模式（0x01=蓝牙，0x00=USB）


class TransportMode:
    """设备的数据上行通道（0x0D 指令的参数帧）。"""

    USB = 0x00
    BLE = 0x01

    NAMES = {USB: "USB", BLE: "蓝牙"}

    @classmethod
    def name_of(cls, mode: int) -> str:
        """0 -> 'USB'，1 -> '蓝牙'，其它返回十六进制。"""
        return cls.NAMES.get(mode, f"未知(0x{mode:02X})")

    @classmethod
    def valid(cls, mode: int) -> bool:
        return mode in cls.NAMES


#: 本程序默认使用 USB 通道：无需蓝牙配对、带宽更大，且不受丢包影响。
#: （设备自身出厂默认是蓝牙，连接后由命令行自动下发 0x0D=0x00 切换。）
DEFAULT_TRANSPORT_MODE = TransportMode.USB


class ProtocolError(Exception):
    """协议解析错误。"""


# --------------------------------------------------------------------------- #
# 校验与打包
# --------------------------------------------------------------------------- #


def checksum(payload: bytes) -> int:
    """校验字 = 校验字之前所有字节之和，高位舍弃。

    文档示例：``98 00 00 01 00`` -> 0x98+0x01 = 0x99
    """
    return sum(payload) & 0xFF


def build_frame(cmd: int, dev: int = DEV_HOST, data: bytes = b"") -> bytes:
    """按帧格式打包一条指令帧。"""
    if not 0 <= cmd <= 0xFF:
        raise ProtocolError(f"指令位超出范围: {cmd}")
    if not 0 <= dev <= 0xFF:
        raise ProtocolError(f"状态字超出范围: {dev}")
    data = bytes(data)
    if len(data) > MAX_PAYLOAD_LEN:
        raise ProtocolError(f"数据包过长: {len(data)} > {MAX_PAYLOAD_LEN}")

    body = bytes([FRAME_HEAD, (len(data) >> 8) & 0xFF, len(data) & 0xFF, cmd, dev]) + data
    return body + bytes([checksum(body), FRAME_TAIL])


# --------------------------------------------------------------------------- #
# 下行指令（上位机 -> 设备）
# --------------------------------------------------------------------------- #


def cmd_start_continuous() -> bytes:
    """连续采集：98 00 00 01 00 99 81"""
    return build_frame(Command.CONTINUOUS)


def cmd_single_acquire() -> bytes:
    """单次采集：98 00 00 02 00 9A 81"""
    return build_frame(Command.SINGLE)


def cmd_stop() -> bytes:
    """停止采集：98 00 00 03 00 9B 81"""
    return build_frame(Command.STOP)


def cmd_read_battery() -> bytes:
    """读取电量：98 00 00 07 00 9F 81"""
    return build_frame(Command.READ_BATTERY)


def cmd_device_info() -> bytes:
    """查看设备信息：98 00 00 0B 00 A3 81"""
    return build_frame(Command.DEVICE_INFO)


def cmd_set_array(rows: int = DEFAULT_ROWS, cols: int = DEFAULT_COLS) -> bytes:
    """设置采集阵列：rows=32 cols=32 -> 98 00 02 08 00 20 20 E2 81"""
    if not (1 <= rows <= 255 and 1 <= cols <= 255):
        raise ProtocolError("阵列尺寸必须为 1~255")
    return build_frame(Command.SET_ARRAY, data=bytes([rows, cols]))


def cmd_set_threshold(value: int = DEFAULT_THRESHOLD) -> bytes:
    """设置采集阈值（0~255）：值 100 -> 98 00 01 0C 00 64 09 81"""
    if not 0 <= value <= 255:
        raise ProtocolError("阈值必须为 0~255")
    return build_frame(Command.SET_THRESHOLD, data=bytes([value]))


def cmd_set_transport_mode(mode: int = DEFAULT_TRANSPORT_MODE) -> bytes:
    """设置数据传输模式。

    蓝牙: 98 00 01 0D 00 01 A7 81（文档 3.7 示例，逐字节一致）
    USB : 98 00 01 0D 00 00 A6 81

    注意：设备出厂默认蓝牙，且**掉电重启后自动恢复蓝牙**，因此每次用 USB
    连接都要重新下发一次本指令。
    """
    if not TransportMode.valid(mode):
        raise ProtocolError("传输模式只能为 0x00(USB) 或 0x01(蓝牙)")
    return build_frame(Command.SET_MODE, data=bytes([mode]))


def cmd_set_usb_mode() -> bytes:
    """切到 USB 上行：98 00 01 0D 00 00 A6 81"""
    return cmd_set_transport_mode(TransportMode.USB)


def cmd_set_ble_mode() -> bytes:
    """切到蓝牙上行：98 00 01 0D 00 01 A7 81"""
    return cmd_set_transport_mode(TransportMode.BLE)


# --------------------------------------------------------------------------- #
# 帧对象与流式解析
# --------------------------------------------------------------------------- #


@dataclass
class Frame:
    """一条完整、校验通过的数据帧。"""

    cmd: int
    dev: int
    data: bytes
    raw: bytes = field(repr=False, default=b"")

    @property
    def is_from_device(self) -> bool:
        """设备应答帧状态字为 0x01，或数据帧本身来自设备（连续/单次采集）。"""
        return self.dev == DEV_SLAVE or self.cmd in (Command.CONTINUOUS, Command.SINGLE)

    def hex(self) -> str:
        return " ".join(f"{b:02X}" for b in self.raw)


SCAN_OK = "ok"
SCAN_NEED_MORE = "need_more"
SCAN_SKIP = "skip"


def scan_frame(
    buf,
    start: int = 0,
    stats: Optional[dict] = None,
    max_payload: int = MAX_PAYLOAD_LEN,
) -> Tuple[Optional[Frame], int, str]:
    """在 ``buf[start:]`` 中寻找第一条「完整且校验通过」的帧。

    返回 ``(frame, next_pos, status)``：

    * ``status == SCAN_OK``        —— 找到帧，``next_pos`` 指向帧尾之后；
    * ``status == SCAN_SKIP``      —— ``buf[start]`` 是个假帧头（长度非法 /
      帧尾不对 / 校验错），``next_pos == start + 1``，调用方只需跳过这 1 个
      字节再继续（**不要在此时丢弃后面的字节**：USB 通道上紧跟其后的很
      可能是需要保留的明文）；
    * ``status == SCAN_NEED_MORE`` —— 有候选帧但数据未收全，``next_pos`` 之前
      的字节可以安全丢弃，从 ``next_pos`` 起保留等待后续数据。

    难点：蓝牙通知的前 N 字节是无效数据，其中可能恰好出现 0x98，并伪造出
    一个很大的长度把后面的真帧卡死。因此遇到「长度合法但数据未收全」的
    候选帧时不能立即停下，要继续向后寻找可完整解码的帧；只有在后面确实
    没有可解码帧时，才回到最早的未完成候选处等待。
    """
    n = len(buf)
    pos = start
    first_incomplete = -1
    head_rejected = False
    while True:
        cand = buf.find(FRAME_HEAD, pos)
        if cand < 0:
            break
        if n - cand < 3:
            if first_incomplete < 0:
                first_incomplete = cand
            break
        length = (buf[cand + 1] << 8) | buf[cand + 2]
        if length > max_payload:
            head_rejected = head_rejected or cand == start
            pos = cand + 1  # 长度非法 -> 噪声，跳过
            continue
        total = length + 7
        if n - cand < total:
            if first_incomplete < 0:
                first_incomplete = cand
            pos = cand + 1  # 先别停，继续找后面有没有完整帧
            continue
        if buf[cand + total - 1] != FRAME_TAIL:
            if stats is not None:
                stats["bad_tail"] = stats.get("bad_tail", 0) + 1
            head_rejected = head_rejected or cand == start
            pos = cand + 1
            continue
        body = bytes(buf[cand : cand + total - 2])
        if checksum(body) != buf[cand + total - 2]:
            if stats is not None:
                stats["bad_checksum"] = stats.get("bad_checksum", 0) + 1
            head_rejected = head_rejected or cand == start
            pos = cand + 1
            continue
        raw = bytes(buf[cand : cand + total])
        return (
            Frame(cmd=raw[3], dev=raw[4], data=raw[5 : 5 + length], raw=raw),
            cand + total,
            SCAN_OK,
        )

    if first_incomplete >= 0:
        return None, first_incomplete, SCAN_NEED_MORE
    if head_rejected:
        return None, start + 1, SCAN_SKIP
    return None, n, SCAN_NEED_MORE


class StreamParser:
    """把字节流（可能含无效前导数据、跨包分片、粘包）还原成完整帧序列。

    用法::

        parser = StreamParser()
        for frame in parser.feed(chunk):
            ...
    """

    def __init__(self, max_buffer: int = MAX_FRAME_LEN * 8) -> None:
        self._buf = bytearray()
        self.stats = {
            "bytes": 0,
            "frames": 0,
            "bad_checksum": 0,
            "bad_tail": 0,
        }
        self._max_buffer = max_buffer

    def reset(self) -> None:
        self._buf.clear()

    @property
    def pending_bytes(self) -> int:
        return len(self._buf)

    def feed(self, chunk: bytes) -> List[Frame]:
        """喂入一段原始字节，返回本次能完整解析出的所有帧。"""
        if chunk:
            self._buf.extend(chunk)
            self.stats["bytes"] += len(chunk)
        return self._drain()

    def _drain(self) -> List[Frame]:
        frames: List[Frame] = []
        buf = self._buf
        i = 0
        n = len(buf)
        while i < n:
            head = buf.find(FRAME_HEAD, i)
            if head < 0:
                i = n
                break
            frame, end, status = scan_frame(buf, head, self.stats)
            if status == SCAN_OK and frame is not None:
                frames.append(frame)
                self.stats["frames"] += 1
                i = end
                continue
            if status == SCAN_SKIP:
                i = end  # 假帧头，跳过它继续找后面的真帧
                continue
            i = end  # 数据不足：丢掉已确认的噪声，保留候选帧等待后续数据
            break

        if i:
            del buf[:i]
        if len(buf) > self._max_buffer:  # 兜底：防止异常流把内存吃满
            del buf[: len(buf) - self._max_buffer]
        return frames


# --------------------------------------------------------------------------- #
# 上行数据解析
# --------------------------------------------------------------------------- #


def parse_acquire_payload(
    data: bytes,
    rows: int = DEFAULT_ROWS,
    cols: int = DEFAULT_COLS,
) -> Tuple[int, List[int]]:
    """解析连续/单次采集的数据包。

    数据帧结构（见文档 3.1 / 3.2）::

        前 2 字节 : 当前帧数（高位在前，清除 flash 或 >65535 时归零）
        其后 n 字节: 采集点数据，每 2 字节一个点，高位在前
                   值 = DATAH * 256 + DATAL

    返回 ``(帧号, 采集点列表)``，列表按「行优先」排列（行 x 列）。
    """
    if len(data) < 2:
        raise ProtocolError(f"采集数据包过短: {len(data)} 字节，至少需要 2 字节帧号")
    frame_no = (data[0] << 8) | data[1]
    body = data[2:]
    if len(body) % 2:
        body = body[:-1]  # 容忍奇数长度的尾字节
    points = [(body[i] << 8) | body[i + 1] for i in range(0, len(body), 2)]
    expect = rows * cols
    if len(points) < expect:
        # 帧可能被截断（蓝牙丢包），补零以保证网格完整
        points = points + [0] * (expect - len(points))
    elif len(points) > expect:
        points = points[:expect]
    return frame_no, points


# --------------------------------------------------------------------------- #
# USB 明码压力块（文档 3.2 注 2）
# --------------------------------------------------------------------------- #

#: 一个「数字」token 必须是能被 float() 接受、且不含下划线的裸数字。
#: 这样设备信息里的 device_id:1、hw_version:1.0 不会被误判成压力数据。
def looks_numeric_token(token: str) -> bool:
    if not token or "_" in token:
        return False
    try:
        value = float(token)
    except ValueError:
        return False
    return value == value and value not in (float("inf"), float("-inf"))


def numeric_tail(
    tokens: Sequence[str],
    min_width: int,
    max_prefix_tokens: int = 2,
) -> Optional[List[float]]:
    """取「行尾连续的纯数字段」，容忍行首少量噪声；不符合则返回 None。

    为什么要容忍行首噪声：串口在切换到明文模式前可能残留半截二进制数据，
    这些字节没有换行分隔，会和紧随其后的第一行数据粘在一起。若因为行首几
    个噪声 token 就丢弃整行，每块都会少一行，**后面所有压力块都会垂直错位
    一行** —— 这比丢数据更难排查。因此这里只要求：噪声不超过
    ``max_prefix_tokens`` 个 token，且其后**全部**是数字、且个数 >= ``min_width``。

    device_id:1 / hw_version:1.0 这类设备信息只有一个（非数字）token，
    数字段为空，因此仍然会被正确地透传出去。
    """
    if len(tokens) < min_width:
        return None
    start = 0
    while start < len(tokens) and not looks_numeric_token(tokens[start]):
        start += 1
    if start > max_prefix_tokens:
        return None
    if len(tokens) - start < min_width:
        return None
    values: List[float] = []
    for tok in tokens[start:]:
        try:
            values.append(float(tok))
        except ValueError:
            return None  # 中间夹杂非数字 -> 不是数据行
    return values


def resample_grid(
    values: Sequence[float],
    src_rows: int,
    src_cols: int,
    dst_rows: int,
    dst_cols: int,
) -> List[float]:
    """把 ``src_rows x src_cols`` 的压力场按面积平均重采样到目标尺寸。

    用途：USB 模式下设备固定输出 32x32 的明文压力块（文档 3.2），而映射
    参数是按 ``rows x cols``（默认 7x7）整定的。这里做「盒式**面积平均**」：
    每格的值仍是「该格位置上的压力」，因此 ``activate_abs`` 这类**单格**
    阈值与阵列尺寸无关，缩放前后含义一致。

    ⚠ 因为取的是平均而不是求和，网格尺寸变化时**整场求和**的量级会随格数
    变化（例如 32x32 降到 7x7 后总和约为原来的 1/21），所以 ``heavy_total``
    这类「按总和比较」的阈值仍应按实测调整；相对判据 ``heavy_auto`` 不受影响。

    列表按行优先排列。
    """
    if src_rows <= 0 or src_cols <= 0:
        return [0.0] * (dst_rows * dst_cols)
    if (src_rows, src_cols) == (dst_rows, dst_cols):
        return [float(v) for v in values]

    out: List[float] = []
    scale_r = src_rows / float(dst_rows)
    scale_c = src_cols / float(dst_cols)
    n = len(values)

    for dr in range(dst_rows):
        y0, y1 = dr * scale_r, (dr + 1) * scale_r
        r_lo = max(0, int(y0))
        r_hi = min(src_rows, int(y1) + 1)
        for dc in range(dst_cols):
            x0, x1 = dc * scale_c, (dc + 1) * scale_c
            c_lo = max(0, int(x0))
            c_hi = min(src_cols, int(x1) + 1)
            total = 0.0
            area = 0.0
            for r in range(r_lo, r_hi):
                oy = min(y1, r + 1) - max(y0, r)
                if oy <= 0:
                    continue
                base = r * src_cols
                for c in range(c_lo, c_hi):
                    ox = min(x1, c + 1) - max(x0, c)
                    if ox <= 0:
                        continue
                    w = oy * ox
                    idx = base + c
                    if 0 <= idx < n:
                        total += float(values[idx]) * w
                    area += w
            out.append(total / area if area > 0 else 0.0)
    return out


@dataclass
class AsciiBlock:
    """USB 明码压力块（已重采样到目标阵列尺寸）。"""

    values: List[float]
    rows: int
    cols: int
    #: 设备实际输出的行数 / 每行点数（如 32x32）
    src_rows: int = 0
    src_cols: int = 0
    #: 是否为残缺块（行数不足，缺的行按 0 补）
    partial: bool = False
    timestamp: float = 0.0

    @property
    def native_shape(self) -> Tuple[int, int]:
        return self.src_rows or self.rows, self.src_cols or self.cols


class AsciiTelemetryParser:
    """把 USB 通道上的 ASCII 明文还原成 ``rows x cols`` 的压力块。

    文档 3.2 注 2 规定 USB 单次采集的返回形如::

        0.123 1.004 2.500 ... （每行 32 个点，空格分隔，3 位小数）
        ...                                        （共 32 行）

    解析要点：

    * 行宽自适应 —— 以第一行的数字个数为宽度，凑满同样行数即视为一块；
      行宽变化说明换了新的一块，上一块作废重来；
    * 非数字行（设备信息、调试打印等）原样透传，不会丢；
    * 容忍行首少量噪声 token（见 ``numeric_tail``），避免「每块少一行」
      造成的整场压力数据垂直错位；
    * 只有「凑满」的块才会被交出：明文块的传输耗时可能很长（32x32 在
      115200 波特率下要 0.6 秒），所以**默认不做残缺块补零**，宁可丢掉
      半截数据也不给出一块形状错误的压力场；确有需要时用
      ``allow_partial=True`` 打开；
    * 超时的作用是把「永远凑不满」的残留清掉，并通过 ``stats['abandoned']``
      与状态回调告诉使用者原因（例如设备只发 20 行）；
    * 数值统一乘 ``scale`` 换算到内部压力单位，使既有的激活/重压阈值
      （默认以原始 ADC 计数为量纲）在 kPa 明码模式下仍然可用。
    """

    def __init__(
        self,
        rows: int = DEFAULT_ROWS,
        cols: int = DEFAULT_COLS,
        scale: float = 100.0,
        min_width: int = 4,
        min_lines: int = 2,
        timeout: float = 0.5,
        allow_partial: bool = False,
        max_prefix_tokens: int = 2,
    ) -> None:
        self.rows = int(rows)
        self.cols = int(cols)
        self.scale = float(scale)
        self.min_width = int(min_width)
        self.min_lines = int(min_lines)
        self.timeout = float(timeout)
        self.allow_partial = bool(allow_partial)
        self.max_prefix_tokens = int(max_prefix_tokens)

        self._tail = ""  # 尚未收到换行的残留
        self._lines: List[List[float]] = []
        self._width = 0
        self._last_rx = 0.0
        self.stats = {
            "lines": 0,
            "blocks": 0,
            "partial_blocks": 0,
            "skipped_lines": 0,
            "resampled": 0,
            "abandoned": 0,
        }

    # -- 状态 -------------------------------------------------------------- #
    @property
    def pending_lines(self) -> int:
        return len(self._lines)

    @property
    def native_width(self) -> int:
        """当前块每行的点数（0 = 还没有数据）。"""
        return self._width

    def reset(self) -> None:
        self._lines.clear()
        self._width = 0

    # -- 输入 -------------------------------------------------------------- #
    def feed_text(self, text: str, now: Optional[float] = None) -> Tuple[List[AsciiBlock], str]:
        """喂入一段文本，返回 ``(凑成的压力块, 需要原样透传的文本)``。"""
        now = time.perf_counter() if now is None else now
        self._last_rx = now
        blocks: List[AsciiBlock] = []
        passthrough: List[str] = []

        chunk = self._tail + text
        parts = chunk.split("\n")
        self._tail = parts.pop()  # 最后一段可能还没结束，留在缓冲区

        for raw in parts:
            line = raw.rstrip("\r")
            tokens = line.split()
            if not tokens:
                continue
            values = numeric_tail(tokens, self.min_width, self.max_prefix_tokens)
            if values is None:
                self.stats["skipped_lines"] += 1
                passthrough.append(line)
                continue
            block = self._push(values, now)
            if block is not None:
                blocks.append(block)

        return blocks, ("\n".join(passthrough) + "\n" if passthrough else "")

    def feed_bytes(self, data: bytes, now: Optional[float] = None) -> Tuple[List[AsciiBlock], bytes]:
        """``feed_text`` 的字节版，方便直接接在串口读取循环里。"""
        blocks, keep = self.feed_text(data.decode("ascii", errors="replace"), now)
        return blocks, keep.encode("ascii", errors="replace")

    def flush(self, now: Optional[float] = None) -> Tuple[Optional[AsciiBlock], str]:
        """空闲超时结算。

        先尝试把「没有换行结尾」的残留行折算进来（很多设备最后一行不带
        换行），再决定是交出一块残缺数据还是干脆丢弃。
        """
        now = time.perf_counter() if now is None else now
        if not self._lines and not self._tail.strip():
            return None, ""
        if now - self._last_rx < self.timeout:
            return None, ""  # 还在收，继续等

        keep = ""
        if self._tail.strip():
            blocks, keep = self.feed_text("\n", now)
            if blocks:
                return blocks[-1], keep
        if not self._lines:
            return None, keep
        if self.allow_partial and len(self._lines) >= self.min_lines:
            return self._build(now), keep
        self.stats["abandoned"] += 1
        self.reset()
        return None, keep

    # -- 内部 -------------------------------------------------------------- #
    def _lines_needed(self) -> int:
        """凑满多少行算一块。"""
        if self._width == self.cols:
            return max(self.min_lines, self.rows)
        return self._width

    def _push(self, values: List[float], now: float) -> Optional[AsciiBlock]:
        if self._width and len(values) != self._width:
            # 行宽变了 -> 上一块作废，从这里重新开始
            self._lines.clear()
            self._width = 0
        if not self._width:
            self._width = len(values)
        self._lines.append(values)
        self.stats["lines"] += 1
        if len(self._lines) >= self._lines_needed():
            return self._build(now)
        return None

    def _build(self, now: float) -> Optional[AsciiBlock]:
        width = self._width
        lines = self._lines
        rows_seen = len(lines)
        if width <= 0 or rows_seen < self.min_lines:
            self.reset()
            return None
        if rows_seen > width:  # 多出来的行不属于这一块
            lines = lines[:width]
            rows_seen = width

        partial = rows_seen != width
        flat = [v * self.scale for row in lines for v in row]
        src_rows, src_cols = rows_seen, width
        if partial:  # 缺的行按 0 补，保持几何形状
            flat.extend([0.0] * (width * (width - rows_seen)))
            src_rows = width

        if (src_rows, src_cols) != (self.rows, self.cols):
            self.stats["resampled"] += 1
        values = resample_grid(flat, src_rows, src_cols, self.rows, self.cols)

        self.reset()
        self.stats["blocks"] += 1
        if partial:
            self.stats["partial_blocks"] += 1
        return AsciiBlock(
            values=values,
            rows=self.rows,
            cols=self.cols,
            src_rows=src_rows,
            src_cols=src_cols,
            partial=partial,
            timestamp=now,
        )


def parse_ascii_telemetry(
    text: str,
    rows: int = DEFAULT_ROWS,
    cols: int = DEFAULT_COLS,
    scale: float = 100.0,
) -> Optional[AsciiBlock]:
    """一次性把完整的 ASCII 文本解析成压力块（不留状态，便于测试与抓包分析）。"""
    parser = AsciiTelemetryParser(
        rows=rows, cols=cols, scale=scale, min_lines=1, allow_partial=True
    )
    blocks, _ = parser.feed_text(text)
    if blocks:
        return blocks[-1]
    block, _ = parser.flush(now=time.perf_counter() + 1e6)  # 强制结算残留
    return block


def parse_battery(data: bytes) -> int:
    """电量百分比（0~100）。文档示例: 98 00 01 07 01 34 .. 81 -> 0x34 = 52%"""
    if not data:
        raise ProtocolError("电量应答没有数据")
    return int(data[0])


def parse_ascii_info(data: bytes) -> str:
    """设备信息（0x0B 应答）为 ASCII 明码。"""
    return data.decode("ascii", errors="replace").strip("\x00\r\n ")


def parse_array_reply(data: bytes) -> Tuple[int, int]:
    """设置采集阵列应答 -> (rows, cols)。"""
    if len(data) < 2:
        raise ProtocolError("阵列应答数据不足")
    return int(data[0]), int(data[1])


def parse_threshold_reply(data: bytes) -> int:
    """设置阈值应答 -> 阈值。"""
    if not data:
        raise ProtocolError("阈值应答数据不足")
    return int(data[0])


def to_hex(data: Sequence[int] | bytes) -> str:
    """字节序列 -> '98 00 00 01 00 99 81' 形式的可读字符串。"""
    return " ".join(f"{b:02X}" for b in data)


def from_hex(text: str) -> bytes:
    """'98 00 00 01 00 99 81' / '98000001009981' -> bytes。"""
    cleaned = text.replace("0x", "").replace("0X", "")
    cleaned = "".join(ch for ch in cleaned if ch in "0123456789abcdefABCDEF")
    if len(cleaned) % 2:
        raise ProtocolError("十六进制串长度必须为偶数")
    return bytes.fromhex(cleaned)


def iter_frames(data: bytes) -> Iterable[Frame]:
    """一次性解析一段（或一帧）字节。"""
    parser = StreamParser()
    yield from parser.feed(data)
