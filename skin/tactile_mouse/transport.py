"""@file transport.py
@brief USB / BLE / Mock 传输实现与统一接口。
@details
    该模块封装底层通讯细节，确保上层 Session 无需关心设备使用何种连接介质。
    它支持串口枚举、蓝牙扫描、帧流解码以及模拟数据源，统一输出字节流给协议层。

传输层：USB（默认）/ 蓝牙(BLE) / 模拟数据源。

三者对外暴露统一接口，上层控制器只依赖 ``Transport``，因此：

  * 没有硬件时用 ``MockTransport`` 即可完整跑通「协议 → 特征 → 手势 → 鼠标」链路；
  * 换通信方式（USB ↔ 蓝牙）不需要改任何上层代码。

**默认走 USB**：设备自带 USB 通道（CDC / 虚拟串口），无需配对、带宽更大、
不受蓝牙丢包影响，代价是文档 3.7 明确说明「设备出厂默认蓝牙，且掉电重启后
自动恢复蓝牙」，所以每次连接都必须重新下发 0x0D = 0x00（由 ``Session`` 负责）。

USB 通道上的数据有两种形态，``UsbStreamDecoder`` 会把它们统一成
「协议帧 + 字节流」的形式交给上层：

  1. 二进制协议帧（连续采集数据、各类指令应答）—— 帧头 0x98 不是可打印
     ASCII，绝不会与明文混淆，因此可以直接扫描；
  2. 0x02 单次采集返回的 ASCII 明文压力块（文档 3.2 注 2）—— 被还原成与
     二进制帧等价的采集帧，必要时还会重采样到配置的阵列尺寸。

线程模型：USB 与蓝牙的收数据都在后台线程里进行，通过 ``on_data(bytes)``
回调把原始字节丢出来（通常是丢进 ``queue.Queue``）；``send()`` 是线程安全的。
"""

from __future__ import annotations

import math
import random
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from . import protocol as P

DataCallback = Callable[[bytes], None]
StatusCallback = Callable[[str], None]


class TransportError(Exception):
    pass


class Transport:
    """传输层基类。"""

    name = "transport"

    def __init__(self, on_data: DataCallback, on_status: Optional[StatusCallback] = None):
        self._on_data = on_data
        self._on_status = on_status or (lambda msg: None)
        self._open = False

    # -- 生命周期 ---------------------------------------------------------- #
    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    # -- 收发 -------------------------------------------------------------- #
    def send(self, data: bytes) -> None:
        raise NotImplementedError

    @property
    def is_open(self) -> bool:
        return self._open

    # -- 工具 -------------------------------------------------------------- #
    def _emit(self, data: bytes) -> None:
        if data:
            self._on_data(data)

    def _status(self, msg: str) -> None:
        self._on_status(msg)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# --------------------------------------------------------------------------- #
# 1) 蓝牙 BLE
# --------------------------------------------------------------------------- #


def list_ble_devices(timeout: float = 5.0, name_filter: Optional[str] = None) -> List[Tuple[str, str, int]]:
    """扫描 BLE 设备，返回 [(name, address, rssi), ...]。

    不依赖具体设备名，但可用 ``name_filter`` 做子串过滤。
    """
    try:
        import asyncio

        from bleak import BleakScanner
    except ImportError as exc:  # pragma: no cover
        raise TransportError("未安装 bleak，请执行: pip install bleak") from exc

    async def _scan():
        found = await BleakScanner.discover(timeout=timeout, return_adv=True)
        out = []
        items = found.items() if isinstance(found, dict) else ((d.address, (d, None)) for d in found)
        for addr, pair in items:
            dev, adv = pair
            nm = getattr(dev, "name", None) or getattr(adv, "local_name", None) or ""
            if name_filter and name_filter.lower() not in nm.lower():
                continue
            rssi = getattr(adv, "rssi", None)
            out.append((nm or "(无名)", addr, rssi if rssi is not None else 0))
        out.sort(key=lambda x: -x[2])
        return out

    return asyncio.run(_scan())


class BleTransport(Transport):
    """BLE GATT 传输。

    默认 UUID（协议文档）::

        Service          0xFFE0
        Write (无响应)    0xFFE1   APP -> UART
        Notify           0xFFE2   UART -> APP
    """

    name = "ble"

    def __init__(
        self,
        address: Optional[str] = None,
        device_name: Optional[str] = None,
        on_data: DataCallback = lambda b: None,
        on_status: Optional[StatusCallback] = None,
        service_uuid: str = P.BLE_SERVICE_UUID,
        write_uuid: str = P.BLE_WRITE_UUID,
        notify_uuid: str = P.BLE_NOTIFY_UUID,
        scan_timeout: float = 8.0,
        reconnect: bool = True,
    ):
        super().__init__(on_data, on_status)
        self.address = address
        self.device_name = device_name
        self.service_uuid = service_uuid
        self.write_uuid = write_uuid
        self.notify_uuid = notify_uuid
        self.scan_timeout = scan_timeout
        self.reconnect = reconnect

        self._loop = None
        self._thread: Optional[threading.Thread] = None
        self._client = None
        self._stop = threading.Event()
        self._connected = threading.Event()
        self._pending: List[bytes] = []
        self._lock = threading.Lock()

    # -- 生命周期 ---------------------------------------------------------- #
    def open(self) -> None:
        try:
            import asyncio  # noqa: F401

            from bleak import BleakClient  # noqa: F401
        except ImportError as exc:  # pragma: no cover
            raise TransportError("未安装 bleak，请执行: pip install bleak") from exc

        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ble-loop", daemon=True)
        self._thread.start()
        # 等待连接结果（最多 30 秒）
        if not self._connected.wait(timeout=30.0):
            self.close()
            raise TransportError(f"连接超时: {self.device_name or self.address}")
        self._open = True

    def close(self) -> None:
        self._open = False
        self._stop.set()
        loop = self._loop
        if loop is not None:
            try:
                loop.call_soon_threadsafe(lambda: None)
            except RuntimeError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        self._connected.clear()

    def send(self, data: bytes) -> None:
        """线程安全地写下行数据；未连接时先缓存。"""
        with self._lock:
            if self._client is None or not self._connected.is_set():
                self._pending.append(data)
                return
        self._write(data)

    # -- 内部实现 ---------------------------------------------------------- #
    def _write(self, data: bytes) -> None:
        loop = self._loop
        if loop is None:
            return
        try:
            import asyncio

            fut = asyncio.run_coroutine_threadsafe(self._async_write(data), loop)
            fut.result(timeout=2.0)
        except Exception as exc:  # noqa: BLE001
            self._status(f"写入失败: {exc}")

    async def _async_write(self, data: bytes) -> None:
        client = self._client
        if client is None:
            return
        try:
            await client.write_gatt_char(self.write_uuid, data, response=False)
        except Exception:
            # 部分模块要求按特征句柄/短 UUID 写，退化为带响应写
            await client.write_gatt_char(self.write_uuid, data, response=True)

    def _run(self) -> None:
        import asyncio

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._main())
        except Exception as exc:  # noqa: BLE001
            self._status(f"蓝牙线程退出: {exc}")
        finally:
            try:
                self._loop.close()
            except Exception:  # noqa: BLE001
                pass
            self._loop = None

    async def _main(self) -> None:
        from bleak import BleakClient, BleakScanner

        while not self._stop.is_set():
            address = self.address
            if address is None:
                self._status(f"扫描设备 (name={self.device_name or '任意'}) ...")
                device = await BleakScanner.find_device_by_name(
                    self.device_name or "", timeout=self.scan_timeout
                ) if self.device_name else await BleakScanner.find_device_by_filter(
                    lambda d, ad: True, timeout=self.scan_timeout
                )
                if device is None:
                    self._status("未找到设备，5 秒后重试")
                    await self._sleep(5.0)
                    continue
                address = device.address
                self.address = address
                self._status(f"找到设备 {device.name or ''} [{address}]")

            self._status(f"连接 {address} ...")

            def _on_disconnect(_client):
                self._connected.clear()
                self._client = None
                self._status("连接已断开")

            try:
                async with BleakClient(address, disconnected_callback=_on_disconnect) as client:
                    self._client = client
                    await client.start_notify(self.notify_uuid, self._on_notify)
                    self._status(f"已连接 {address}，订阅 {self.notify_uuid[-8:]}")
                    self._connected.set()
                    with self._lock:
                        pending, self._pending = self._pending, []
                    for item in pending:  # 补发连接期间排队的指令
                        try:
                            await self._async_write(item)
                        except Exception:  # noqa: BLE001
                            pass
                    while not self._stop.is_set() and client.is_connected:
                        await self._sleep(0.2)
                    try:
                        await client.stop_notify(self.notify_uuid)
                    except Exception:  # noqa: BLE001
                        pass
            except Exception as exc:  # noqa: BLE001
                self._status(f"连接异常: {exc}")
            finally:
                self._connected.clear()
                self._client = None

            if not self.reconnect or self._stop.is_set():
                break
            self._status("3 秒后重连 ...")
            await self._sleep(3.0)

    async def _sleep(self, seconds: float) -> None:
        import asyncio

        try:
            await asyncio.sleep(seconds)
        except asyncio.CancelledError:  # pragma: no cover
            pass

    def _on_notify(self, _sender, data: bytearray) -> None:
        self._emit(bytes(data))


# --------------------------------------------------------------------------- #
# 2) USB（默认通道）
# --------------------------------------------------------------------------- #

#: 常见 USB 转串口 / CDC 芯片的 VID。设备插上后大概率会以这些芯片枚举成
#: 一个虚拟串口，自动选口时优先挑它们，避免误选主板自带串口或虚拟串口软件。
USB_SERIAL_VIDS: Dict[int, str] = {
    0x1A86: "WCH CH34x/CH91xx",
    0x10C4: "Silicon Labs CP210x",
    0x0403: "FTDI FT232",
    0x0483: "STMicroelectronics CDC",
    0x067B: "Prolific PL2303",
    0x2341: "Arduino",
    0x2A03: "Arduino (org)",
    0x239A: "Adafruit",
    0x2E8A: "Raspberry Pi Pico",
    0x303A: "Espressif",
    0x1546: "u-blox",
    0x1915: "Nordic",
}


@dataclass
class SerialPortInfo:
    """一个串口（可能是 USB 虚拟串口）。"""

    device: str
    description: str = ""
    hwid: str = ""
    vid: Optional[int] = None
    pid: Optional[int] = None

    @property
    def is_usb(self) -> bool:
        """是否 USB 设备（而不是主板自带 COM / 虚拟串口软件）。"""
        if self.vid is not None:
            return True
        return "USB" in self.hwid.upper()

    @property
    def bridge(self) -> str:
        if self.vid in USB_SERIAL_VIDS:
            return USB_SERIAL_VIDS[self.vid]
        return "USB 串口" if self.is_usb else "串口"

    def label(self) -> str:
        desc = self.description or self.bridge
        return f"{self.device} ({desc})"


def list_serial_ports() -> List[SerialPortInfo]:
    """列出本机所有串口，USB 虚拟串口排在前面。"""
    try:
        from serial.tools import list_ports  # type: ignore
    except ImportError as exc:  # pragma: no cover
        raise TransportError("未安装 pyserial，请执行: pip install pyserial") from exc

    out: List[SerialPortInfo] = []
    for p in list_ports.comports():
        vid = getattr(p, "vid", None)
        out.append(
            SerialPortInfo(
                device=p.device,
                description=p.description or "",
                hwid=p.hwid or "",
                vid=vid,
                pid=getattr(p, "pid", None),
            )
        )

    def _rank(info: SerialPortInfo):
        return (
            0 if info.vid in USB_SERIAL_VIDS else (1 if info.is_usb else 2),
            info.device,
        )

    out.sort(key=_rank)
    return out


#: 判定「这是一条二进制采集通道」所需的最小载荷：2 字节帧号 + 至少 1 个点。
#: 空载荷的 0x01/0x02 帧只是指令应答，不能用来判定通道形态。
_MIN_BINARY_ACQUIRE_BYTES = 4


class UsbStreamDecoder:
    """把 USB 通道上的「二进制帧 + ASCII 明文」混合流解成统一的字节流。

    设计要点：

    * 0x98 属于不可打印字节，**不可能出现在 ASCII 明文里**，所以「先在缓冲区
      里找完整帧、帧之前的字节按文本处理」这条规则不会误判；
    * 文本行若是整行数字（例如 ``0.123 1.004 ...``）则计入 ASCII 压力块，
      否则（设备信息、调试打印、JSON）原样透出，不丢内容；
    * ASCII 压力块最终被合成一条与二进制连续的采集帧完全等价的帧，因此
      上层（Session / Controller）不需要区分通道；
    * 一旦确凿地收到二进制采集帧，就锁定为 binary，不再做明文累积，
      避免二进制载荷里的可打印字节被当成压力数据。
    """

    def __init__(
        self,
        emit: Callable[[bytes], None],
        rows: int = P.DEFAULT_ROWS,
        cols: int = P.DEFAULT_COLS,
        scale: float = 100.0,
        stream_format: str = "auto",
        ascii_timeout: float = 0.5,
        allow_partial: bool = False,
        min_width: int = 4,
        on_status: Optional[StatusCallback] = None,
    ) -> None:
        self._emit = emit
        self._on_status = on_status or (lambda msg: None)
        self.stream_format = (stream_format or "auto").lower()
        self.ascii = P.AsciiTelemetryParser(
            rows=rows,
            cols=cols,
            scale=scale,
            min_width=min_width,
            timeout=ascii_timeout,
            allow_partial=allow_partial,
        )
        self._buf = bytearray()
        self._mode: Optional[str] = None  # 锁定的通道形态：binary / ascii
        self._block_no = 0
        self._announced_abandon = 0
        self.stats = {
            "bytes": 0,
            "frames": 0,
            "text_bytes": 0,
            "ascii_blocks": 0,
            "bytes_dropped": 0,
            "bad_tail": 0,
            "bad_checksum": 0,
        }

    # -- 对外接口 ---------------------------------------------------------- #
    @property
    def mode(self) -> Optional[str]:
        """已识别的通道形态（None = 还没收到足够证据）。"""
        return self._mode

    @property
    def ascii_stats(self) -> Dict[str, int]:
        return self.ascii.stats

    def feed(self, chunk: bytes, now: Optional[float] = None) -> None:
        if not chunk:
            return
        self.stats["bytes"] += len(chunk)
        self._buf.extend(chunk)
        self._drain(now)

    def poll(self, now: Optional[float] = None) -> None:
        """空闲时调用：结算超时的明文块，并汇报「凑不满」的原因。"""
        if self._mode == "binary" or self.stream_format == "binary":
            return
        block, keep = self.ascii.flush(now)
        if keep:
            self._emit(keep)
        if block is not None:
            self._emit_block(block)
        abandoned = self.ascii.stats["abandoned"]
        if abandoned != self._announced_abandon:
            self._announced_abandon = abandoned
            width = self.ascii.native_width or 0
            self._on_status(
                "USB 明文压力块凑不满（已丢弃 %d 块，当前行宽 %d 列）。"
                "若设备确实只发部分行，可用 --usb-allow-partial 允许残缺块（缺行按 0 补）。"
                % (abandoned, width)
            )

    # -- 内部 -------------------------------------------------------------- #
    def _drain(self, now: Optional[float]) -> None:
        buf = self._buf
        while buf:
            idx = buf.find(P.FRAME_HEAD)
            if idx < 0:
                self._handle_text(bytes(buf), now)
                buf.clear()
                return
            if idx:
                self._handle_text(bytes(buf[:idx]), now)
                del buf[:idx]
            # 此时 buf[0] 一定是帧头
            frame, next_pos, status = P.scan_frame(buf, 0, self.stats)
            if status == P.SCAN_OK and frame is not None:
                self._on_frame(frame)
                del buf[:next_pos]
                continue
            if status == P.SCAN_SKIP:
                # 假帧头：只跳过这 1 个字节，后面的字节重新按「文本/帧」判断，
                # 否则紧跟在假帧头后面的明文会被整段丢掉。
                self.stats["bytes_dropped"] += next_pos
                del buf[:next_pos]
                continue
            # 数据不足：丢弃已确认的噪声，保留候选帧等待后续数据
            if next_pos > 0:
                self.stats["bytes_dropped"] += next_pos
                del buf[:next_pos]
            return

    def _handle_text(self, raw: bytes, now: Optional[float]) -> None:
        if not raw:
            return
        use_ascii = self.stream_format != "binary" and self._mode != "binary"
        if not use_ascii:
            self.stats["text_bytes"] += len(raw)
            self._emit(raw)
            return
        blocks, keep = self.ascii.feed_bytes(raw, now)
        if keep:
            self.stats["text_bytes"] += len(keep)
            self._emit(keep)
        for block in blocks:
            self._emit_block(block)

    def _on_frame(self, frame: P.Frame) -> None:
        # 只有「载荷足够长」的采集帧才能证明这是一条二进制数据通道：
        # 设备对 0x01/0x02 的空载荷应答（98 00 00 01 01 xx 81）不能算数，
        # 否则 USB 明文模式会在开头的应答帧上被误判成二进制。
        if (
            self.stream_format != "ascii"
            and frame.cmd in (P.Command.CONTINUOUS, P.Command.SINGLE)
            and len(frame.data) >= _MIN_BINARY_ACQUIRE_BYTES
        ):
            if self._mode is None:
                self._on_status("USB 通道：二进制协议帧（连续/单次采集）")
            self._mode = "binary"
        self.stats["frames"] += 1
        self._emit(frame.raw)

    def _emit_block(self, block: P.AsciiBlock) -> None:
        if self._mode is None:
            self._on_status(
                "USB 通道：ASCII 明文压力块 %dx%d（已换算并重采样到 %dx%d）"
                % (*block.native_shape, block.rows, block.cols)
            )
        self._mode = "ascii"
        payload = bytearray([(self._block_no >> 8) & 0xFF, self._block_no & 0xFF])
        self._block_no = (self._block_no + 1) & 0xFFFF
        for value in block.values:
            v = int(round(value))
            v = 0 if v < 0 else min(0xFFFF, v)
            payload += bytes([(v >> 8) & 0xFF, v & 0xFF])
        self.stats["ascii_blocks"] += 1
        # 合成一条标准采集帧：帧号 + 每点 2 字节，与二进制通道完全同构
        self._emit(P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, bytes(payload)))


class UsbTransport(Transport):
    """USB（CDC / 虚拟串口）传输 —— 本程序的默认通道。

    与蓝牙相比：

    * 不需要配对，插上就能用，带宽足够跑满阵列帧率；
    * 没有蓝牙那个「通知包前 N 字节无效数据」的问题（但解析仍然容错）；
    * 0x0D 必须每次下发（设备掉电重启会自己切回蓝牙）。

    串口参数由协议文档未给出，因此默认 **自动选口 + 自动试探波特率**：
    依次用常见波特率发送一次 0x0B 设备信息指令，谁有应答就用谁。
    若已知具体参数，用 ``port`` / ``baudrate`` 直接指定可跳过试探。
    """

    name = "usb"

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 0,
        on_data: DataCallback = lambda b: None,
        on_status: Optional[StatusCallback] = None,
        rows: int = P.DEFAULT_ROWS,
        cols: int = P.DEFAULT_COLS,
        scale: float = 100.0,
        stream_format: str = "auto",
        ascii_timeout: float = 0.5,
        allow_partial: bool = False,
        read_timeout: float = 0.05,
        read_size: int = 4096,
        reconnect: bool = True,
        port_hint: Optional[str] = None,
        autobaud: bool = True,
        probe_timeout: float = 1.0,
        dtr: Optional[bool] = True,
        exclusive: bool = True,
    ):
        super().__init__(on_data, on_status)
        self.port = port
        self.baudrate = int(baudrate or 0)
        self.read_timeout = float(read_timeout)
        self.read_size = int(read_size)
        self.reconnect = bool(reconnect)
        self.port_hint = port_hint
        self.autobaud = bool(autobaud)
        self.probe_timeout = float(probe_timeout)
        self.dtr = dtr
        self.exclusive = bool(exclusive)

        self._serial_mod = None
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._decoder = UsbStreamDecoder(
            emit=self._emit,
            rows=rows,
            cols=cols,
            scale=scale,
            stream_format=stream_format,
            ascii_timeout=ascii_timeout,
            allow_partial=allow_partial,
            on_status=on_status,
        )
        self.stats = self._decoder.stats

    # -- 只读信息 ---------------------------------------------------------- #
    @property
    def detected_baudrate(self) -> int:
        return self.baudrate

    @property
    def channel_mode(self) -> Optional[str]:
        return self._decoder.mode

    # -- 生命周期 ---------------------------------------------------------- #
    def open(self) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise TransportError(
                "未安装 pyserial（USB 通道需要）。请执行: pip install pyserial\n"
                "或者改用蓝牙: python run.py --ble --name <设备名>"
            ) from exc
        self._serial_mod = serial

        if not self.port:
            self.port = self._autodetect_port()
        if not self.baudrate:
            self.baudrate = self._probe_baudrate(self.port) or 115200
            self._status(f"自动试探到波特率 {self.baudrate}")

        try:
            self._ser = self._open_port(self.port, self.baudrate)
        except Exception as exc:  # noqa: BLE001
            raise TransportError(
                f"打开串口 {self.port} 失败: {exc}\n"
                f"提示：确认端口未被其它程序占用（串口助手 / 微信小程序等），"
                f"或用 --port 指定其它端口。"
            ) from exc

        self._open = True
        self._thread = threading.Thread(target=self._reader, name="usb-rx", daemon=True)
        self._thread.start()
        self._status(f"USB 已打开 {self.port} @ {self.baudrate} 8N1")

    def close(self) -> None:
        was_open = self._open
        self._open = False
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if was_open:
            self._status("USB 已关闭")

    def send(self, data: bytes) -> None:
        ser = self._ser
        if ser is None:
            return
        with self._tx_lock:
            try:
                ser.write(data)
                ser.flush()
            except Exception as exc:  # noqa: BLE001
                self._status(f"USB 写入失败: {exc}")

    # -- 选口 / 试探波特率 -------------------------------------------------- #
    def _autodetect_port(self) -> str:
        try:
            ports = list_serial_ports()
        except TransportError:
            raise
        except Exception as exc:  # noqa: BLE001
            raise TransportError(f"枚举串口失败: {exc}") from exc

        if not ports:
            raise TransportError(
                "没有找到任何串口。请依次确认：\n"
                "  1) 皮肤已用 USB 线连上电脑（换根线/换个 USB 口也试试，有些线只能充电）；\n"
                "  2) 已安装 pyserial：pip install pyserial；\n"
                "  3) 设备管理器里能看到端口，然后显式指定：--port COM5。"
            )
        if self.port_hint:
            hint = self.port_hint.lower()
            matched = [p for p in ports if hint in (p.device + " " + p.description).lower()]
            if matched:
                self._status(f"按提示「{self.port_hint}」选中串口 {matched[0].label()}")
                return matched[0].device
            self._status(f"没有串口匹配「{self.port_hint}」，回退到自动选择")
        self._status(f"自动选中串口 {ports[0].label()}（共 {len(ports)} 个）")
        return ports[0].device

    def _open_port(self, port: str, baudrate: int):
        ser = self._serial_mod.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.read_timeout,
            write_timeout=2.0,
            rtscts=False,
            dsrdtr=False,
            exclusive=self.exclusive,
        )
        # 有些 USB 转串口板把 DTR 接到了 MCU 复位脚，拉低会一直复位设备；
        # 但标准 CDC 设备又常常要求 DTR 有效才开始发数据。默认置位，
        # 遇到「一连上就重启 / 收不到数据」时把 UsbConfig.dtr 设成 false。
        if self.dtr is not None:
            try:
                ser.dtr = bool(self.dtr)
                ser.rts = False
            except Exception:  # noqa: BLE001
                pass
        return ser

    def _probe_baudrate(self, port: str) -> Optional[int]:
        """用 0x0B 设备信息指令试探波特率，返回第一个有应答的波特率。"""
        if not self.autobaud:
            return None
        for baudrate in P.USB_BAUD_CANDIDATES:
            self._status(f"试探波特率 {baudrate} ...")
            try:
                ser = self._open_port(port, baudrate)
            except Exception as exc:  # noqa: BLE001
                self._status(f"打开失败（{baudrate}）: {exc}")
                continue
            try:
                ser.reset_input_buffer()
                ser.write(P.cmd_device_info())
                ser.flush()
                if self._wait_reply(ser, self.probe_timeout):
                    return baudrate
            except Exception as exc:  # noqa: BLE001
                self._status(f"试探失败（{baudrate}）: {exc}")
            finally:
                try:
                    ser.close()
                except Exception:  # noqa: BLE001
                    pass
        self._status("所有候选波特率都没有应答，回退到 115200（可用 --baud 显式指定）")
        return None

    def _wait_reply(self, ser, timeout: float) -> bool:
        """在 ``timeout`` 内等待任何可识别的设备应答。"""
        parser = P.StreamParser()
        buf = bytearray()
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            try:
                chunk = ser.read(512)
            except Exception:  # noqa: BLE001
                return False
            if not chunk:
                continue
            buf.extend(chunk)
            for frame in parser.feed(chunk):
                if frame.cmd in (P.Command.DEVICE_INFO, P.Command.READ_BATTERY,
                                 P.Command.SET_MODE, P.Command.SET_ARRAY,
                                 P.Command.SET_THRESHOLD, P.Command.STOP,
                                 P.Command.CONTINUOUS, P.Command.SINGLE):
                    return True
            # 设备信息也可能直接以明文打印（0x0B 应答是 ASCII 明码）
            low = bytes(buf).lower()
            if b"device_id" in low or b"hw_version" in low or b"out_mode" in low:
                return True
            if len(buf) > P.MAX_FRAME_LEN * 4:
                del buf[: -P.MAX_FRAME_LEN]
        return False

    # -- 收数据 ------------------------------------------------------------ #
    def _reader(self) -> None:
        while self._open:
            ser = self._ser
            if ser is None:
                break
            try:
                chunk = ser.read(self.read_size)
            except Exception as exc:  # noqa: BLE001
                if not self._open:
                    break
                self._status(f"USB 读取失败: {exc}")
                if not self._reopen():
                    break
                continue
            if chunk:
                self._decoder.feed(chunk)
            else:
                self._decoder.poll()

    def _reopen(self) -> bool:
        """读失败后尝试重开端口。返回是否成功。"""
        if not self.reconnect:
            return False
        for attempt in range(1, 4):
            if not self._open:
                return False
            self._status(f"USB 断开，{attempt}/3 次尝试重连 ...")
            time.sleep(1.0)
            try:
                if self._ser is not None:
                    try:
                        self._ser.close()
                    except Exception:  # noqa: BLE001
                        pass
                self._ser = self._open_port(self.port, self.baudrate)
                return True
            except Exception as exc:  # noqa: BLE001
                self._status(f"重连失败: {exc}")
        return False


#: 兼容旧名字（1.0 里 USB 通道叫 SerialTransport）
class SerialTransport(UsbTransport):
    """``UsbTransport`` 的旧名，保留以便兼容既有代码。"""

    name = "serial"

    def __init__(self, port: str, baudrate: int = 115200, on_data: DataCallback = lambda b: None,
                 on_status: Optional[StatusCallback] = None, **kwargs):
        kwargs.setdefault("autobaud", False)
        super().__init__(port=port, baudrate=baudrate, on_data=on_data,
                         on_status=on_status, **kwargs)


class UsbChannelTransport(Transport):
    """把任意「原始字节流传输」装饰成 USB 通道：字节先过 ``UsbStreamDecoder``。

    用途：让设备模拟器（``MockTransport``）或自研字节源也能走**与真机完全
    相同**的 USB 解析路径（明文块识别 / 网格重采样 / 二进制帧扫描），
    这样「无硬件验证」才真正有意义。
    """

    name = "usb-loopback"

    def __init__(
        self,
        inner: Transport,
        rows: int = P.DEFAULT_ROWS,
        cols: int = P.DEFAULT_COLS,
        scale: float = 100.0,
        stream_format: str = "auto",
        ascii_timeout: float = 0.5,
        allow_partial: bool = False,
        on_status: Optional[StatusCallback] = None,
    ):
        super().__init__(lambda b: None, on_status)
        self.inner = inner
        self.decoder = UsbStreamDecoder(
            emit=self._emit,
            rows=rows,
            cols=cols,
            scale=scale,
            stream_format=stream_format,
            ascii_timeout=ascii_timeout,
            allow_partial=allow_partial,
            on_status=on_status,
        )
        inner._on_data = self.decoder.feed

    # -- 生命周期（转发给内层） -------------------------------------------- #
    def open(self) -> None:
        self.inner.open()
        self._open = True

    def close(self) -> None:
        self.inner.close()
        self._open = False

    def send(self, data: bytes) -> None:
        self.inner.send(data)

    # -- 诊断信息 ---------------------------------------------------------- #
    @property
    def channel_mode(self) -> Optional[str]:
        return self.decoder.mode

    @property
    def stats(self) -> Dict[str, int]:
        return self.decoder.stats


def wrap_usb_channel(inner: Transport, **kwargs) -> UsbChannelTransport:
    """便捷函数：``UsbChannelTransport(inner, **kwargs)``。"""
    return UsbChannelTransport(inner, **kwargs)


# --------------------------------------------------------------------------- #
# 3) 模拟数据源（无硬件调试）
# --------------------------------------------------------------------------- #


class MockTransport(Transport):
    """用合成数据生成符合协议的采集帧，用于无硬件开发与演示。

    刻意复刻真实蓝牙的两个坑：
      * 每包前面塞入随机长度的无效字节；
      * 把一个帧随机切成 1~3 段发送。

    ``usb_ascii=True`` 时改为扮演「USB 模式的设备」：不再有无效前导字节，
    采集数据按文档 3.2 注 2 以 **ASCII 明文**（32x32、kPa、3 位小数）吐出，
    指令应答仍然是二进制帧 —— 用来在无硬件时验证 USB 解析链路。
    """

    name = "mock"

    def __init__(
        self,
        on_data: DataCallback = lambda b: None,
        on_status: Optional[StatusCallback] = None,
        rows: int = P.DEFAULT_ROWS,
        cols: int = P.DEFAULT_COLS,
        rate_hz: float = 50.0,
        pattern: str = "auto",
        garbage_bytes: int = 6,
        split: bool = True,
        seed: Optional[int] = None,
        usb_ascii: bool = False,
        ascii_size: int = P.ASCII_GRID_SIZE,
        ascii_scale: float = 100.0,
        raw_usb: bool = False,
    ):
        super().__init__(on_data, on_status)
        self.rows, self.cols = rows, cols
        self.rate_hz = rate_hz
        self.pattern = pattern
        self.garbage_bytes = garbage_bytes
        self.split = split
        self.usb_ascii = bool(usb_ascii)
        self.ascii_size = int(ascii_size)
        self.ascii_scale = float(ascii_scale or 100.0)
        if self.usb_ascii or raw_usb:  # USB 通道没有无效前导字节，也不必刻意分片
            self.garbage_bytes = 0
            self.split = False
        if self.usb_ascii and self.rate_hz > 20.0:
            # 32x32 的明文块体积很大（每块约 8KB 文本），真实链路也刷不了太快
            self.rate_hz = 20.0
        self._rng = random.Random(seed)
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._t0 = 0.0
        self._frame_no = 0
        self.sent_commands: List[bytes] = []
        self._state = {
            "array": (rows, cols),
            "threshold": P.DEFAULT_THRESHOLD,
            "mode": P.DEFAULT_TRANSPORT_MODE,
            "running": False,
        }

    # -- 生命周期 ---------------------------------------------------------- #
    def open(self) -> None:
        self._open = True
        self._stop.clear()
        self._t0 = time.perf_counter()
        self._thread = threading.Thread(target=self._loop, name="mock-tx", daemon=True)
        self._thread.start()
        self._status(f"模拟设备已启动（{self.rows}x{self.cols}, {self.rate_hz:g}Hz）")

    def close(self) -> None:
        self._open = False
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._status("模拟设备已停止")

    # -- 下行指令：模拟设备按协议文档的格式应答 ---------------------------- #
    def send(self, data: bytes) -> None:
        self.sent_commands.append(bytes(data))
        parser = P.StreamParser()
        for frame in parser.feed(bytes(data)):
            self._handle_command(frame)

    def _handle_command(self, frame: P.Frame) -> None:
        cmd = frame.cmd
        if cmd == P.Command.CONTINUOUS:
            self._state["running"] = True
            self._reply(cmd, b"")
        elif cmd == P.Command.SINGLE:
            self._state["running"] = True
            self._reply(cmd, b"")  # 立刻会推一帧数据
        elif cmd == P.Command.STOP:
            self._state["running"] = False
            self._reply(cmd, b"")
        elif cmd == P.Command.READ_BATTERY:
            self._reply(cmd, bytes([87]))  # 87%
        elif cmd == P.Command.SET_ARRAY:
            rows, cols = P.parse_array_reply(frame.data)
            self.rows, self.cols = rows, cols
            self._state["array"] = (rows, cols)
            self._reply(cmd, bytes([rows, cols]))
        elif cmd == P.Command.SET_THRESHOLD:
            value = P.parse_threshold_reply(frame.data)
            self._state["threshold"] = value
            self._reply(cmd, bytes([value]))
        elif cmd == P.Command.SET_MODE:
            self._state["mode"] = frame.data[0] if frame.data else 1
            self._reply(cmd, frame.data)
        elif cmd == P.Command.DEVICE_INFO:
            text = (
                "device_id:1\r\n"
                "hw_version:1.0\r\n"
                "sw_version:1.0\r\n"
                f"line_x:{self.rows}\r\n"
                f"line_y:{self.cols}\r\n"
                f"cj_value:{self._state['threshold']}\r\n"
                f"out_mode:{self._state['mode']}\r\n"
            )
            self._emit(text.encode("ascii"))

    def _reply(self, cmd: int, data: bytes) -> None:
        self._emit(P.build_frame(cmd, P.DEV_SLAVE, data))

    # -- 数据生成 ---------------------------------------------------------- #
    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        next_t = time.perf_counter()
        while not self._stop.is_set():
            next_t += period
            if self._state["running"]:
                if self.usb_ascii:
                    self._transmit(self._make_ascii_block())
                else:
                    grid = self._make_grid()
                    payload = self._encode_frame(self._frame_no, grid)
                    self._frame_no = (self._frame_no + 1) & 0xFFFF
                    self._transmit(payload)
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    def _make_ascii_block(self) -> bytes:
        """按文档 3.2 注 2 生成 USB 单次采集的 ASSCI 明文块。

        真实设备固定输出 32x32、单位 kPa、每行 32 个点、保留 3 位小数；
        这里把 7x7 的合成压力场按最近邻放大到 32x32，并按 ascii_scale
        换算成 kPa（与解析端的 scale 互逆，因此往返后数值一致）。
        """
        grid = self._make_grid()
        size = self.ascii_size
        lines = []
        for r in range(size):
            sr = min(self.rows - 1, int(r * self.rows / size))
            row = []
            for c in range(size):
                sc = min(self.cols - 1, int(c * self.cols / size))
                value = grid[sr * self.cols + sc] / self.ascii_scale
                row.append(f"{value:7.3f}")
            lines.append(" ".join(row))
        return ("\n".join(lines) + "\n").encode("ascii")

    def _encode_frame(self, frame_no: int, grid: Sequence[int]) -> bytes:
        body = bytes([(frame_no >> 8) & 0xFF, frame_no & 0xFF])
        for value in grid:
            v = max(0, min(0xFFFF, int(value)))
            body += bytes([(v >> 8) & 0xFF, v & 0xFF])
        return P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, body)

    def _transmit(self, frame: bytes) -> None:
        """按「无效前导 + 随机分片」的方式发送，模拟真实蓝牙。"""
        prefix = bytes(self._rng.randrange(256) for _ in range(self.garbage_bytes))
        stream = prefix + frame
        if not self.split:
            self._emit(stream)
            return
        pos = 0
        while pos < len(stream):
            size = self._rng.randint(8, 24)
            self._emit(stream[pos : pos + size])
            pos += size
            time.sleep(0.0008)

    def _make_grid(self) -> List[int]:
        rows, cols = self.rows, self.cols
        t = time.perf_counter() - self._t0
        contacts = self._contacts(t)
        grid = []
        for r in range(rows):
            for c in range(cols):
                value = 0.0
                for cx, cy, amp in contacts:
                    d2 = (c - cx) ** 2 + (r - cy) ** 2
                    value += amp * math.exp(-d2 / (2 * 1.15 ** 2))
                grid.append(int(min(value, 65535.0)))
        return grid

    def _contacts(self, t: float) -> List[Tuple[float, float, float]]:
        """返回当前时刻的接触点列表 [(cx, cy, 压力幅值), ...]，支持双指。

        幅值与 GestureConfig.heavy_total 的默认值(7000)配套：
        普通接触的单点压力总和约 5600，重压约 18000，便于开箱即用地
        演示「重压拖拽」。
        """
        rows, cols = self.rows, self.cols
        mid_r, mid_c = (rows - 1) / 2.0, (cols - 1) / 2.0
        pattern = self.pattern
        if pattern == "auto":  # 每 12 秒轮换一种动作
            pattern = ["circle", "tap", "scroll", "drag"][int(t // 12) % 4]
        phase = t % 12.0

        if pattern == "circle":  # 画圈：光标应绕圈移动
            return [(mid_c + 2.0 * math.cos(t * 1.8), mid_r + 2.0 * math.sin(t * 1.8), 800.0)]
        if pattern == "tap":  # 轻点：0.6 秒一个点按
            on = (phase % 0.6) < 0.15
            return [(mid_c, mid_r, 900.0 if on else 0.0)]
        if pattern == "scroll":  # 双指：两指同向平移 -> 滚轮
            dy = 1.5 * math.sin(t * 2.2)
            return [
                (mid_c - 1.5, mid_r + dy, 700.0),
                (mid_c + 1.5, mid_r + dy, 700.0),
            ]
        if pattern == "drag":  # 重压拖拽：压力大 -> 按住左键移动
            return [(mid_c + 1.8 * math.sin(t * 1.2), mid_r + 1.8 * math.cos(t * 1.2), 2600.0)]
        if pattern == "center":  # 静止在中心
            return [(mid_c, mid_r, 800.0)]
        # 随机游走
        rnd = self._rng
        return [(
            min(cols - 1.0, max(0.0, mid_c + 2.0 * math.sin(t * 0.7) + rnd.uniform(-0.2, 0.2))),
            min(rows - 1.0, max(0.0, mid_r + 2.0 * math.cos(t * 0.5) + rnd.uniform(-0.2, 0.2))),
            max(0.0, 850.0 + rnd.uniform(-200.0, 200.0)),
        )]
