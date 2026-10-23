"""@file events.py
@brief skin 侧的线上事件格式（与 brain 的 command_schema.py 对应）。

协议版本必须与 brain 端一致（当前为 1）。这里只做**序列化**，不做安全校验
——最终校验在 brain 端，因为 brain 才是硬件安全边界。
"""

from __future__ import annotations

import json
import time
import uuid
from typing import Any, Dict, Optional

from ..robot_gestures import RobotGestureEvent

#: 与 brain ``command_schema.PROTOCOL_VERSION`` 保持一致。
PROTOCOL_VERSION: int = 1


def new_event_id() -> str:
    """生成一个唯一事件 ID。"""
    return uuid.uuid4().hex


def to_wire(
    event: RobotGestureEvent,
    event_id: Optional[str] = None,
    timestamp_ms: Optional[int] = None,
) -> Dict[str, Any]:
    """把识别到的手势事件转成线上 JSON 对象。

    时间戳必须用**墙钟**（``time.time()``），不能用 ``event.timestamp``。
    ------------------------------------------------------------------
    ``event.timestamp`` 最终来自 ``time.perf_counter()``，是「系统启动后的
    单调秒数」（实测机器上约为 606929 秒）。而 brain 端用
    ``int(time.time() * 1000)`` 做新鲜度校验：

        age = now_ms - event.timestamp_ms      # ≈ 1.79e12 - 6.07e8
        if age > MAX_EVENT_AGE_MS:             # 2000 ms
            raise SchemaError("expired", ...)

    两者差 9 个数量级，导致**每一条**事件都被 brain 判为 expired 拒收
    （应答 {"accepted": false, "reason": "expired"}）。这一缺陷只在跨进程
    真实发送时暴露，本仓库内的 dry-run 测试不会经过这条序列化边界。

    因此这里显式使用墙钟时间；``event.timestamp`` 仍保留为识别内部使用的
    相对时间，只用于本地日志，不上线路。
    """
    if timestamp_ms is None:
        timestamp_ms = int(time.time() * 1000)
    payload: Dict[str, Any] = {
        "version": PROTOCOL_VERSION,
        "event_id": event_id or new_event_id(),
        "gesture": event.gesture.value,
        "timestamp_ms": int(timestamp_ms),
        "confidence": round(float(event.confidence), 3),
    }
    if event.direction is not None:
        payload["direction"] = event.direction
    if event.start is not None:
        payload["start"] = [round(float(event.start[0]), 2), round(float(event.start[1]), 2)]
    if event.end is not None:
        payload["end"] = [round(float(event.end[0]), 2), round(float(event.end[1]), 2)]
    return payload


def encode_line(payload: Dict[str, Any]) -> bytes:
    """编码为一行 JSON（含结尾换行），newline-delimited JSON。"""
    return (json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
