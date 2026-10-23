# SPDX-License-Identifier: GPL-3.0-or-later
"""@file command_schema.py
@brief 触觉手势事件的线上协议与校验。

该模块定义 ``microduck_skin``（laptop）与 ``microduck_brain``（RK3566）
之间的**唯一**线上契约，并且是 brain 侧的第一道安全边界。

设计原则
--------
* 外部只允许发送**手势名**，绝不允许发送舵机 ID、目标 count、速度等底层参数。
* 手势名必须属于 :data:`ALLOWED_GESTURES` 白名单。
* 一帧一条 JSON，以换行分隔（newline-delimited JSON）。
* brain 是最终安全边界：即使 skin 端校验通过，这里仍要重新校验。

事件示例::

    {"version": 1, "event_id": "8f2c...", "gesture": "tap",
     "timestamp_ms": 1791500000000, "confidence": 0.92}

brain 的应答示例::

    {"event_id": "8f2c...", "accepted": true, "action": "tilt_head", "reason": "ok"}
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

#: 协议版本。skin 与 brain 必须一致，否则拒收。
PROTOCOL_VERSION: int = 1

#: 允许的手势名。任何不在此集合中的手势一律拒绝。
ALLOWED_GESTURES: Tuple[str, ...] = (
    "tap",
    "single_stroke",
    "back_forth_stroke",
)

#: 单个事件 JSON 的最大字节数（防御性限制，避免超大报文）。
MAX_EVENT_BYTES: int = 4096

#: 事件允许的最大年龄（毫秒）。超过则视为过期，拒绝执行。
MAX_EVENT_AGE_MS: int = 2000


class SchemaError(ValueError):
    """事件不合法时抛出。``reason`` 是可以回给客户端的简短原因码。"""

    def __init__(self, reason: str, detail: str = ""):
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail or reason


@dataclass
class GestureEvent:
    """一条已经通过校验的手势事件。"""

    gesture: str
    event_id: str
    timestamp_ms: int
    confidence: float = 1.0
    version: int = PROTOCOL_VERSION
    direction: Optional[str] = None
    start: Optional[Tuple[float, float]] = None
    end: Optional[Tuple[float, float]] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {
            "version": self.version,
            "event_id": self.event_id,
            "gesture": self.gesture,
            "timestamp_ms": self.timestamp_ms,
            "confidence": self.confidence,
        }
        if self.direction is not None:
            data["direction"] = self.direction
        if self.start is not None:
            data["start"] = list(self.start)
        if self.end is not None:
            data["end"] = list(self.end)
        return data


def _coerce_pair(value: Any, name: str) -> Optional[Tuple[float, float]]:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise SchemaError("bad_field", f"{name} 必须是长度 2 的数组")
    try:
        return float(value[0]), float(value[1])
    except (TypeError, ValueError) as exc:
        raise SchemaError("bad_field", f"{name} 不是数字") from exc


def parse_event(payload: Any) -> GestureEvent:
    """把已解析的 JSON 对象转成 :class:`GestureEvent`，或抛出 :class:`SchemaError`。

    ``payload`` 应该是 ``json.loads`` 的结果（dict）。这里**不做**时间校验，
    时间校验在 :func:`check_freshness` 中单独进行，方便测试。
    """
    if not isinstance(payload, dict):
        raise SchemaError("not_object", "事件必须是 JSON 对象")

    version = payload.get("version", PROTOCOL_VERSION)
    try:
        version = int(version)
    except (TypeError, ValueError) as exc:
        raise SchemaError("bad_version", "version 必须是整数") from exc
    if version != PROTOCOL_VERSION:
        raise SchemaError("bad_version", f"不支持的协议版本 {version}")

    gesture = payload.get("gesture")
    if not isinstance(gesture, str) or gesture not in ALLOWED_GESTURES:
        raise SchemaError("bad_gesture", f"未知或不允许的手势 {gesture!r}")

    event_id = payload.get("event_id")
    if not isinstance(event_id, str) or not event_id.strip():
        raise SchemaError("bad_event_id", "event_id 必须是非空字符串")
    if len(event_id) > 128:
        raise SchemaError("bad_event_id", "event_id 过长")

    timestamp_ms = payload.get("timestamp_ms")
    if not isinstance(timestamp_ms, (int, float)):
        raise SchemaError("bad_timestamp", "timestamp_ms 必须是数字")
    timestamp_ms = int(timestamp_ms)

    confidence = payload.get("confidence", 1.0)
    if not isinstance(confidence, (int, float)):
        raise SchemaError("bad_confidence", "confidence 必须是数字")
    confidence = float(confidence)
    if not 0.0 <= confidence <= 1.0:
        raise SchemaError("bad_confidence", "confidence 必须在 0..1")

    direction = payload.get("direction")
    if direction is not None and not isinstance(direction, str):
        raise SchemaError("bad_field", "direction 必须是字符串")

    start = _coerce_pair(payload.get("start"), "start")
    end = _coerce_pair(payload.get("end"), "end")

    # 保留未知字段以便日志排查，但绝不据此执行任何动作。
    known = {
        "version",
        "event_id",
        "gesture",
        "timestamp_ms",
        "confidence",
        "direction",
        "start",
        "end",
    }
    extra = {k: v for k, v in payload.items() if k not in known}

    return GestureEvent(
        gesture=gesture,
        event_id=event_id,
        timestamp_ms=timestamp_ms,
        confidence=confidence,
        version=version,
        direction=direction,
        start=start,
        end=end,
        extra=extra,
    )


def parse_line(line: str) -> GestureEvent:
    """解析一行（不含结尾换行）JSON，失败时抛出 :class:`SchemaError`。"""
    if len(line.encode("utf-8", errors="ignore")) > MAX_EVENT_BYTES:
        raise SchemaError("too_large", "事件超过最大长度")
    text = line.strip()
    if not text:
        raise SchemaError("empty", "空行")
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise SchemaError("bad_json", "无法解析 JSON") from exc
    return parse_event(payload)


def check_freshness(
    event: GestureEvent,
    now_ms: int,
    max_age_ms: int = MAX_EVENT_AGE_MS,
    future_tolerance_ms: int = 500,
) -> None:
    """检查事件是否过期或时间戳在未来。

    * 事件比 ``now_ms`` 早超过 ``max_age_ms`` -> 过期（网络延迟 / 断线恢复）。
    * 事件比 ``now_ms`` 晚超过 ``future_tolerance_ms`` -> 拒绝（时钟明显不对）。
    """
    age = now_ms - event.timestamp_ms
    if age > max_age_ms:
        raise SchemaError("expired", f"事件已过期 {age} ms")
    if age < -future_tolerance_ms:
        raise SchemaError("from_future", f"事件时间戳在未来 {-age} ms")


def make_reply(
    event_id: Optional[str],
    accepted: bool,
    action: Optional[str] = None,
    reason: str = "ok",
) -> Dict[str, Any]:
    """构造 brain -> skin 的应答对象。"""
    reply: Dict[str, Any] = {
        "event_id": event_id,
        "accepted": bool(accepted),
        "reason": reason,
    }
    if action is not None:
        reply["action"] = action
    return reply


def encode_line(obj: Dict[str, Any]) -> bytes:
    """把应答对象编码为一行 JSON（含结尾换行）。"""
    return (json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
