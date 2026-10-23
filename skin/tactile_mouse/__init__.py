"""@file __init__.py
@brief 包入口，声明公开 API 与版本信息。
@details
    该模块导出控制器、协议解析、传输层与会话对象，方便外部直接
    使用 tactile_mouse 包中的核心能力。

电子触觉皮肤（压力阵列）→ 鼠标。

按《采集设备使用说明-蓝牙_V5.0》实现通讯协议，并把触觉压力映射为鼠标动作：
移动光标、单击/双击/右键、按住拖拽、双指滚轮。

**默认走 USB 通道**（文档 3.7 的 0x0D = 0x00）：无需蓝牙配对、带宽更大、
不受丢包影响，代价是设备出厂默认蓝牙且掉电后会自己切回蓝牙，因此每次连接
程序都会重发 0x0D 指令。

最常用的入口::

    from tactile_mouse.config import AppConfig
    from tactile_mouse.session import Session, make_transport

    cfg = AppConfig()                        # 默认 transport_kind == "usb"
    session = Session(cfg, transport=make_transport(cfg))   # 自动选串口 + 自动试探波特率
    session.start()          # 下发 0x0D/0x08/0x0C/0x07/0x01 并开始采集
    ...
    session.stop()

切回蓝牙::

    make_transport(cfg, kind="ble", name="TT-Skin")

无硬件调试::

    make_transport(cfg, kind="mock", mock_mode="usb-ascii")   # 模拟 USB 明文设备

或者直接跑命令行：``python run.py``（USB）／``python run.py --mock --gui``
"""

__version__ = "1.1.0"

from .config import AppConfig, FilterConfig, GestureConfig, MappingConfig, UsbConfig  # noqa: F401
from .controller import Controller  # noqa: F401
from .features import Contact, TactileProcessor, TactileState  # noqa: F401
from .gestures import Gesture, GestureEvent, GestureRecognizer  # noqa: F401
from .mouse_backend import MouseBackend, create_backend  # noqa: F401
from .protocol import AsciiTelemetryParser, Command, Frame, StreamParser, TransportMode  # noqa: F401
from .session import Session, list_usb_ports, make_transport  # noqa: F401
from .transport import (  # noqa: F401
    BleTransport,
    MockTransport,
    SerialPortInfo,
    Transport,
    UsbChannelTransport,
    UsbStreamDecoder,
    UsbTransport,
    list_serial_ports,
)

__all__ = [
    "AppConfig",
    "FilterConfig",
    "GestureConfig",
    "MappingConfig",
    "UsbConfig",
    "Controller",
    "TactileProcessor",
    "TactileState",
    "Contact",
    "Gesture",
    "GestureEvent",
    "GestureRecognizer",
    "MouseBackend",
    "create_backend",
    "AsciiTelemetryParser",
    "Command",
    "Frame",
    "StreamParser",
    "TransportMode",
    "Session",
    "make_transport",
    "list_usb_ports",
    "Transport",
    "UsbTransport",
    "UsbChannelTransport",
    "UsbStreamDecoder",
    "SerialPortInfo",
    "list_serial_ports",
    "BleTransport",
    "MockTransport",
]
