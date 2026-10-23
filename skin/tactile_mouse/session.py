"""@file session.py
@brief 设备会话编排与生命周期管理。
@details
    Session 将传输层、协议解析器与时间控制器联系起来，负责发送配置指令、
    处理上下行帧、在后台线程驱动控制器，并为 GUI 提供状态和调试回调。

会话层：把「传输 -> 协议解析 -> 控制器」串起来，供 CLI / GUI 复用。

线程模型::

    传输线程(BLE 通知回调) --原始字节--> StreamParser --> 帧队列(maxsize 小)
                                                        |
                                    工作线程 <-----------+
                                      |-- 控制指令应答 -> 状态回调
                                      |-- 采集数据帧   -> Controller.feed()
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

from . import protocol as P
from .config import AppConfig
from .controller import Controller
from .mouse_backend import MouseBackend, create_backend
from .transport import (
    BleTransport,
    MockTransport,
    Transport,
    TransportError,
    UsbTransport,
    list_serial_ports,
    wrap_usb_channel,
)

StatusCallback = Callable[[str], None]


@dataclass
class DeviceInfo:
    battery: Optional[int] = None
    array: Optional[tuple] = None
    threshold: Optional[int] = None
    mode: Optional[int] = None
    ascii_info: str = ""
    dropped_frames: int = 0
    raw_frames: int = 0
    data_frames: int = 0
    extra: Dict[str, str] = field(default_factory=dict)


class Session:
    """一次完整的「设备 <-> 鼠标」会话。"""

    #: 打开串口后等待设备就绪的秒数。见 ``start()`` 里的说明与实测数据。
    SETTLE_S = 0.5

    def __init__(
        self,
        cfg: Optional[AppConfig] = None,
        transport: Optional[Transport] = None,
        backend: Optional[MouseBackend] = None,
        backend_kind: str = "auto",
        dry_run: bool = False,
        unaccelerated: bool = False,
        on_status: Optional[StatusCallback] = None,
        queue_size: int = 8,
    ):
        self.cfg = cfg or AppConfig()
        self.on_status = on_status or (lambda msg: print(msg, flush=True))
        self.on_debug = lambda direction, data: None
        self.info = DeviceInfo()

        self.parser = P.StreamParser()
        self._queue: "queue.Queue[tuple]" = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._poller: Optional[threading.Thread] = None

        if backend is None:
            backend = create_backend(backend_kind, dry_run=dry_run, unaccelerated=unaccelerated)
        self.controller = Controller(self.cfg, backend=backend)

        if transport is None:
            transport = MockTransport(rows=self.cfg.filter.rows, cols=self.cfg.filter.cols)
        self.transport = transport
        self.transport._on_data = self._on_raw  # 复用同一个传输实例时确保回调正确

    # ------------------------------------------------------------------ #
    # 生命周期
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        self._stop.clear()
        self.transport.open()
        self._report_channel()
        self._worker = threading.Thread(target=self._worker_loop, name="tm-worker", daemon=True)
        self._worker.start()
        # 打开串口会使 USB CDC 设备重新枚举，期间下发的指令会被丢弃却仍有
        # 应答，造成"配置成功但收不到数据"的假象。实测：open 后立即下发
        # = 0 帧；等 0.3s = 99 帧。详见 robot_session.RobotSession.SETTLE_S。
        time.sleep(self.SETTLE_S)
        self._configure_device()
        if self.cfg.auto_calibrate_on_start:
            self.recalibrate()
        if (self.cfg.acquire or "continuous").lower() == "single":
            self._poller = threading.Thread(target=self._poll_loop, name="tm-poll", daemon=True)
            self._poller.start()
        self.status(f"鼠标后端: {self.controller.backend.name}")

    def _ensure_stopped_before_start(self) -> None:
        """**已停用**。保留函数只为兼容旧调用点。

        本机固件收到 0x03（STOP）后无法再被 START 恢复推流：实测只吐 1 帧
        就彻底停止，且任何后续指令序列都救不回来，只能重新上电。设备上电
        本来就不推流，所以「先停」这一步是多余且有害的。
        """
        return None

    def _report_channel(self) -> None:
        """把实际使用的通道参数告诉使用者，方便排查「连上了但没数据」。"""
        transport = self.transport
        if isinstance(transport, UsbTransport):
            self.status(
                f"传输通道: USB {transport.port} @ {transport.baudrate} 8N1"
                f"（形态自动识别，见 UsbConfig.format）"
            )
        elif isinstance(transport, BleTransport):
            target = transport.address or transport.device_name or "任意设备"
            self.status(f"传输通道: 蓝牙 {target}")

    def stop(self) -> None:
        self._stop.set()
        if self._poller is not None:
            self._poller.join(timeout=2.0)
            self._poller = None
        # 不发 cmd_stop()：固件收到 0x03 后无法用 START 恢复推流，会让下一次
        # 启动彻底收不到数据，只能给设备重新上电。直接关串口即可。
        try:
            self.transport.close()
        except Exception:  # noqa: BLE001
            pass
        if self._worker is not None:
            self._worker.join(timeout=2.0)
            self._worker = None
        self.controller.close()
        self.status("会话已停止")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()
        return False

    # ------------------------------------------------------------------ #
    def status(self, msg: str) -> None:
        self.on_status(msg)

    def set_debug_hook(self, callback) -> None:
        self.on_debug = callback or (lambda direction, data: None)

    def _debug_packet(self, direction: str, data: bytes) -> None:
        if not data:
            return
        self.on_debug(direction, data)

    def _configure_device(self) -> None:
        """连接后按协议下发初始参数。

        顺序有讲究：**0x0D 必须最先发**。文档 3.7 明确说明设备出厂默认
        蓝牙、且掉电重启后自动恢复蓝牙，因此每次用 USB 连接都要先把上行
        通道切到 USB（0x00），否则后面的指令虽然能通，但数据仍会走蓝牙。
        """
        rows, cols = self.cfg.device_array
        self.send(P.cmd_set_transport_mode(int(self.cfg.device_mode)))
        self.status(
            f"已下发：上行通道 = {P.TransportMode.name_of(int(self.cfg.device_mode))} (0x0D)"
        )
        self.send(P.cmd_set_array(int(rows), int(cols)))
        self.send(P.cmd_set_threshold(int(self.cfg.device_threshold)))
        self.send(P.cmd_read_battery())
        if (self.cfg.acquire or "continuous").lower() == "continuous":
            self.send(P.cmd_start_continuous())
            self.status("已下发：连续采集 (0x01)")
        else:
            self.status(f"已下发：单次采集轮询 (0x02) @ {self.cfg.single_poll_hz:g}Hz")
            if isinstance(self.transport, UsbTransport):
                self.status(
                    "提示：USB 模式下单次采集(0x02)返回的是 ASCII 明文压力块"
                    "（文档 3.2 注 2，单位 kPa），已自动识别并换算。"
                )

    def _poll_loop(self) -> None:
        period = 1.0 / max(1.0, self.cfg.single_poll_hz)
        next_t = time.perf_counter()
        while not self._stop.is_set():
            try:
                self.send(P.cmd_single_acquire())
            except Exception as exc:  # noqa: BLE001
                self.status(f"单次采集指令发送失败: {exc}")
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    # ------------------------------------------------------------------ #
    # 接收路径
    # ------------------------------------------------------------------ #
    def _on_raw(self, data: bytes) -> None:
        """在传输层线程里做轻量解析，把数据帧丢进队列（满了丢最旧的）。"""
        for frame in self.parser.feed(data):
            self._debug_packet("RX", frame.raw)
            self.info.raw_frames += 1
            if frame.cmd in (P.Command.CONTINUOUS, P.Command.SINGLE) and len(frame.data) >= 2:
                # 采集数据帧至少要有 2 字节帧号；0x01/0x02 的空载荷帧只是设备
                # 「指令已执行」的应答（文档 3.1/3.2 的设备应答），不是数据。
                self.info.data_frames += 1
                try:
                    frame_no, points = P.parse_acquire_payload(
                        frame.data, self.cfg.filter.rows, self.cfg.filter.cols
                    )
                except P.ProtocolError as exc:
                    self.info.extra["parse_error"] = str(exc)
                    continue
                self._put((frame_no, points))
            else:
                self._handle_reply(frame)

    def _put(self, item) -> None:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            # 队列满说明上位机处理不过来，丢弃最旧的一帧并累计丢帧数
            try:
                self._queue.get_nowait()
                self.info.dropped_frames += 1
                self._queue.put_nowait(item)
            except queue.Empty:
                pass

    def _handle_reply(self, frame: P.Frame) -> None:
        try:
            if frame.cmd == P.Command.READ_BATTERY:
                self.info.battery = P.parse_battery(frame.data)
                self.status(f"设备电量 {self.info.battery}%")
            elif frame.cmd == P.Command.SET_ARRAY:
                self.info.array = P.parse_array_reply(frame.data)
                self.status(f"采集阵列已设为 {self.info.array[0]}x{self.info.array[1]}")
            elif frame.cmd == P.Command.SET_THRESHOLD:
                self.info.threshold = P.parse_threshold_reply(frame.data)
                self.status(f"采集阈值已设为 {self.info.threshold}")
            elif frame.cmd == P.Command.SET_MODE:
                self.info.mode = frame.data[0] if frame.data else None
                if self.info.mode is not None:
                    self.status(f"设备上行通道已切换为 {P.TransportMode.name_of(self.info.mode)}")
            elif frame.cmd == P.Command.DEVICE_INFO:
                self.info.ascii_info = P.parse_ascii_info(frame.data)
                self.status("设备信息:\n" + self.info.ascii_info)
            else:
                self.info.extra.setdefault("other", frame.hex())
        except P.ProtocolError as exc:
            self.info.extra["reply_error"] = str(exc)

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame_no, points = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.controller.feed(points, frame_no=frame_no)
            except Exception as exc:  # noqa: BLE001
                self.status(f"处理帧出错: {exc}")

    # ------------------------------------------------------------------ #
    # 便捷操作（供 GUI 按钮调用）
    # ------------------------------------------------------------------ #
    def send(self, data: bytes) -> None:
        self._debug_packet("TX", data)
        self.transport.send(data)

    def read_battery(self) -> None:
        self.send(P.cmd_read_battery())

    def set_threshold(self, value: int) -> None:
        self.cfg.device_threshold = int(value)
        self.send(P.cmd_set_threshold(int(value)))

    def set_device_info(self) -> None:
        self.send(P.cmd_device_info())

    def emergency_stop(self) -> None:
        self.controller.emergency_stop()

    def set_enabled(self, enabled: bool) -> None:
        self.controller.set_enabled(enabled)
        self.status("已启用" if enabled else "已暂停（不再注入鼠标动作）")

    def recalibrate(self) -> None:
        self.controller.recalibrate()
        self.status("正在重新校准零点，请在此期间不要触碰皮肤 ...")


# --------------------------------------------------------------------------- #
def normalize_kind(kind: Optional[str], mock: bool = False) -> str:
    """把各种写法归一到 usb / ble / mock。"""
    if mock:
        return "mock"
    text = (kind or "").strip().lower()
    if text in ("", "auto", "default"):
        return "usb"
    if text in ("usb", "serial", "com", "uart", "cdc"):
        return "usb"
    if text in ("ble", "bluetooth", "bt", "gatt"):
        return "ble"
    if text in ("mock", "sim", "fake", "demo"):
        return "mock"
    raise TransportError(f"未知的传输类型: {kind}（可用 usb / ble / mock）")


def make_transport(
    cfg: AppConfig,
    kind: Optional[str] = None,
    address: Optional[str] = None,
    name: Optional[str] = None,
    port: Optional[str] = None,
    mock: bool = False,
    pattern: str = "auto",
    on_status: Optional[StatusCallback] = None,
    mock_mode: str = "ble",
) -> Transport:
    """按配置/命令行参数创建传输实例。

    默认走 **USB**（``cfg.transport_kind`` 默认 "usb"）：无需蓝牙配对，
    插上即用；用 ``kind="ble"`` / ``--ble`` 可切回蓝牙。

    ``mock_mode`` 决定模拟设备扮演的角色：

    * ``"ble"``      —— 二进制帧 + 无效前导字节 + 随机分片（默认，最像蓝牙）
    * ``"usb"``      —— 二进制帧、无前导无分片（USB 连续采集）
    * ``"usb-ascii"``—— 明文压力块（USB 单次采集），并套上 ``UsbChannelTransport``
                        走与真机相同的解析路径
    """
    resolved = normalize_kind(kind if kind is not None else cfg.transport_kind, mock)

    if resolved == "mock":
        role = (mock_mode or "ble").lower()
        ascii_mode = role in ("usb-ascii", "usb_ascii", "ascii")
        inner = MockTransport(
            rows=cfg.filter.rows,
            cols=cfg.filter.cols,
            pattern=pattern,
            on_status=on_status,
            usb_ascii=ascii_mode,
            ascii_scale=cfg.usb.ascii_scale,
            raw_usb=role in ("usb", "usb-ascii", "usb_ascii"),
        )
        if ascii_mode:
            return wrap_usb_channel(inner, **usb_kwargs(cfg))
        return inner
    if resolved == "ble":
        return BleTransport(
            address=address,
            device_name=name,
            on_data=lambda b: None,
            on_status=on_status,
        )
    return UsbTransport(
        port=port or cfg.usb.port,
        baudrate=cfg.usb.baudrate,
        on_data=lambda b: None,
        on_status=on_status,
        read_timeout=cfg.usb.read_timeout,
        reconnect=cfg.usb.reconnect,
        port_hint=cfg.usb.port_hint,
        autobaud=cfg.usb.autobaud,
        dtr=cfg.usb.dtr,
        exclusive=cfg.usb.exclusive,
        **usb_kwargs(cfg),
    )


def usb_kwargs(cfg: AppConfig) -> Dict[str, object]:
    """把 ``cfg.usb`` 摊平成 ``UsbTransport`` / ``UsbChannelTransport`` 的公共参数。"""
    return {
        "rows": cfg.filter.rows,
        "cols": cfg.filter.cols,
        "scale": cfg.usb.ascii_scale,
        "stream_format": cfg.usb.format,
        "ascii_timeout": cfg.usb.ascii_timeout,
        "allow_partial": cfg.usb.allow_partial,
    }


def list_usb_ports():
    """列出本机串口（转发 ``transport.list_serial_ports``，方便上层统一引入）。"""
    return list_serial_ports()
