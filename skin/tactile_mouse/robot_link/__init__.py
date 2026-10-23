"""@file __init__.py
@brief robot_link：skin -> brain 的手势事件发送链路。

与鼠标链路（``controller`` / ``mouse_backend``）完全独立：
本包只把已经识别完成的机器人手势事件，通过 TCP 发给 RK3566 上的
``microduck_brain`` 命令服务器。
"""

from .client import RobotLink, RobotLinkError
from .events import PROTOCOL_VERSION, to_wire  # noqa: F401

__all__ = ["RobotLink", "RobotLinkError", "PROTOCOL_VERSION", "to_wire"]
