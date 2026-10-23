"""@file gestures.py
@brief 时序手势识别器与手势事件定义。
@details
    将逐帧基于压力场的 TactileState 转换成 TAP / DOUBLE_TAP / LONG_PRESS /
    DRAG_START / DRAG_END / SCROLL 等离散事件，供 Controller 进一步驱动鼠标。

时序手势识别：把逐帧的 ``TactileState`` 变成离散的鼠标语义事件。

识别出的手势
------------
======== ==========================================================
TAP      轻点：按下后很快抬起且几乎没移动            -> 左键单击
DOUBLE  双击：两次 TAP 间隔小于 double_tap_ms        -> 左键双击
LONG    长按：按住超过 long_press_ms 且没移动        -> 右键单击
DRAG_START / DRAG_END  重压（总压力超过阈值）        -> 按住左键拖拽
SCROLL  双指同向平移                               -> 滚轮（可横可纵）
======== ==========================================================

设计要点
--------
* 单击**不延迟**发出，双击交由操作系统的双击间隔识别（两次左键单击
  在系统看来就是双击），这样点击延迟最低。若确实需要显式双击事件，
  把 ``GestureConfig`` 的 ``double_click_as_event`` 打开即可。
* 长按一旦触发，本次按住就不再产生 TAP，避免误点。
* 重压拖拽期间不产生 TAP。
* 双指滚动期间会通知上层「暂停光标移动」，避免滚动时手指带着光标乱跑。
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Dict, List, Optional, Tuple

from .config import GestureConfig
from .features import TactileState, euclid


class Gesture(str, Enum):
    TAP = "tap"
    DOUBLE_TAP = "double_tap"
    LONG_PRESS = "long_press"
    DRAG_START = "drag_start"
    DRAG_END = "drag_end"
    SCROLL = "scroll"
    SCROLL_BEGIN = "scroll_begin"
    SCROLL_END = "scroll_end"
    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"


@dataclass
class GestureEvent:
    kind: Gesture
    timestamp: float
    data: Dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        extra = " ".join(f"{k}={v}" for k, v in self.data.items())
        return f"{self.kind.value}{'(' + extra + ')' if extra else ''}"


class GestureRecognizer:
    """有状态的时序手势识别器（每个设备/会话一个实例）。"""

    def __init__(self, cfg: Optional[GestureConfig] = None):
        self.cfg = cfg or GestureConfig()
        self.reset()

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self._down = False
        self._down_time = 0.0
        self._down_xy: Optional[Tuple[float, float]] = None
        self._last_xy: Optional[Tuple[float, float]] = None
        self._max_move = 0.0
        self._long_fired = False  # 本次按住不再产生 TAP
        self._freeze_move = False  # 长按后锁住光标移动
        self._dragging = False
        self._scroll_mode = False
        self._scroll_acc = 0.0
        self._scroll_axis = "v"
        self._scroll_ref: Optional[Tuple[float, float]] = None
        self._last_tap_time = -1e9
        self._pending_tap: Optional[GestureEvent] = None
        # 每次接触的峰值压力，用于估计「什么是轻触」，从而与设备量纲解耦
        self._contact_peak = 0.0
        self._contact_start = 0.0
        self._contact_force = 0.0
        self._peaks: Deque[float] = deque(maxlen=20)
        self.tap_count = 0

    # ------------------------------------------------------------------ #
    @property
    def dragging(self) -> bool:
        return self._dragging

    @property
    def in_scroll_mode(self) -> bool:
        return self._scroll_mode

    @property
    def suppress_move(self) -> bool:
        """是否应暂停光标移动（滚动中 / 长按后锁定）。"""
        return self._scroll_mode or self._freeze_move

    @property
    def reference_force(self) -> float:
        """历史「轻触」压力的估计值（取最近若干次接触峰值的低分位）。"""
        if not self._peaks:
            return 0.0
        ordered = sorted(self._peaks)
        return ordered[max(0, len(ordered) // 3)]

    def heavy_level(self) -> float:
        """判定「重压」的压力门限。"""
        level = self.cfg.heavy_total
        if self.cfg.heavy_auto:
            ref = self.reference_force
            if ref > 0:
                level = max(level, self.cfg.heavy_auto_ratio * ref)
        return level

    @property
    def peak_total(self) -> float:
        return self._contact_peak

    # ------------------------------------------------------------------ #
    def update(self, state: TactileState) -> List[GestureEvent]:
        """喂入一帧处理结果，返回本帧产生的（可能是多个）手势事件。"""
        events: List[GestureEvent] = []
        # 注意：不能用 `state.timestamp or ...`，时间戳 0.0 是合法值
        t = state.timestamp if state.timestamp is not None else time.perf_counter()
        events.extend(self._flush_pending_tap(t))

        # 接触点数量变化 -> 滚动模式进出
        n_contacts = state.n_contacts
        want_scroll = state.active and n_contacts >= 2
        if want_scroll and not self._scroll_mode:
            self._scroll_mode = True
            self._scroll_acc = 0.0
            self._scroll_ref = self._mean_xy(state)
            events.append(GestureEvent(Gesture.SCROLL_BEGIN, t, {"contacts": n_contacts}))
        elif not want_scroll and self._scroll_mode:
            self._scroll_mode = False
            self._scroll_acc = 0.0
            self._scroll_ref = None
            events.append(GestureEvent(Gesture.SCROLL_END, t))

        if self._scroll_mode:
            events.extend(self._update_scroll(state, t))
            # 滚动期间不参与点按/拖拽判定，但仍要跟踪按下状态
            self._down = True
            self._down_xy = None
            self._max_move = 0.0
            self._long_fired = True
            self._last_xy = self._mean_xy(state)
            return events

        if state.active:
            events.extend(self._update_active(state, t))
        else:
            events.extend(self._update_release(t))
        return events

    # ------------------------------------------------------------------ #
    def _force_of(self, state: TactileState) -> float:
        """用于重压判定的压力值：取最强那个接触点的压力总和。

        用「单点压力」而不是全阵列总和，这样双指（滚轮）时不会因为
        压力翻倍而误触发重压。
        """
        contact = state.primary
        if contact is not None:
            return contact.weight
        return state.total

    def _update_active(self, state: TactileState, t: float) -> List[GestureEvent]:
        events: List[GestureEvent] = []
        xy = state.centroid
        if xy is None:
            return events
        force = self._force_of(state)

        if not self._down:
            # 新的按下
            self._down = True
            self._down_time = t
            self._contact_start = t
            self._contact_peak = force
            self._down_xy = xy
            self._max_move = 0.0
            self._long_fired = False
            self._freeze_move = False
        else:
            self._contact_peak = max(self._contact_peak, force)
            if self._down_xy is not None:
                self._max_move = max(self._max_move, euclid(xy, self._down_xy))

        # 重压 -> 拖拽。需要先经过稳定期，避免落指冲击被误判
        settled = (t - self._contact_start) * 1000.0 >= self.cfg.heavy_settle_ms
        heavy = self.heavy_level()
        if not self._dragging and settled and force >= heavy and heavy > 0:
            self._dragging = True
            self._long_fired = True  # 拖拽抬手后不再补一次点击
            events.append(GestureEvent(Gesture.DRAG_START, t,
                                       {"force": round(force), "level": round(heavy)}))
        elif self._dragging and force < heavy * 0.6:
            self._dragging = False
            events.append(GestureEvent(Gesture.DRAG_END, t, {"reason": "force_released"}))

        # 长按 -> 右键
        if (
            not self._long_fired
            and not self._dragging
            and self._down_xy is not None
            and (t - self._down_time) * 1000.0 >= self.cfg.long_press_ms
            and self._max_move <= self.cfg.tap_move_tol
        ):
            self._long_fired = True
            self._freeze_move = True  # 触发右键后锁住光标，避免右键点歪
            events.append(GestureEvent(Gesture.LONG_PRESS, t,
                                       {"held_ms": round((t - self._down_time) * 1000)}))

        self._contact_force = force
        self._last_xy = xy
        return events

    def _update_release(self, t: float) -> List[GestureEvent]:
        events: List[GestureEvent] = []
        if self._dragging:
            self._dragging = False
            events.append(GestureEvent(Gesture.DRAG_END, t, {"reason": "lift"}))

        if self._down:
            # 记录本次接触的峰值，供「轻触压力」参考值估计使用
            if self._contact_peak > 0:
                self._peaks.append(self._contact_peak)
            held_ms = (t - self._down_time) * 1000.0
            was_tap = (
                not self._long_fired
                and held_ms <= self.cfg.tap_max_ms
                and self._max_move <= self.cfg.tap_move_tol
            )
            if was_tap:
                self.tap_count += 1
                window = self.cfg.double_tap_ms / 1000.0
                within = (t - self._last_tap_time) <= window
                payload = {"held_ms": round(held_ms)}
                if self.cfg.double_click_as_event:
                    # 需要显式双击：先押住第一次单击，等过了双击窗口再放出去
                    if self._pending_tap is not None and within:
                        self._pending_tap = None
                        self._last_tap_time = -1e9
                        events.append(GestureEvent(Gesture.DOUBLE_TAP, t, {"taps": 2}))
                    else:
                        self._pending_tap = GestureEvent(Gesture.TAP, t, dict(payload, deferred=True))
                        self._last_tap_time = t
                else:
                    events.append(GestureEvent(Gesture.TAP, t, dict(payload)))
                    if within:
                        events.append(GestureEvent(Gesture.DOUBLE_TAP, t, {"taps": 2}))
                        self._last_tap_time = -1e9  # 双击后重新计数
                    else:
                        self._last_tap_time = t
        # 复位按下状态
        self._down = False
        self._down_xy = None
        self._max_move = 0.0
        self._long_fired = False
        self._freeze_move = False
        self._contact_peak = 0.0
        self._last_xy = None
        return events

    # ------------------------------------------------------------------ #
    def _mean_xy(self, state: TactileState) -> Optional[Tuple[float, float]]:
        if not state.contacts:
            return None
        sx = sum(c.cx for c in state.contacts)
        sy = sum(c.cy for c in state.contacts)
        k = float(len(state.contacts))
        return sx / k, sy / k

    def _update_scroll(self, state: TactileState, t: float) -> List[GestureEvent]:
        events: List[GestureEvent] = []
        xy = self._mean_xy(state)
        if xy is None:
            return events
        if self._scroll_ref is None:
            self._scroll_ref = xy
            return events

        dx = xy[0] - self._scroll_ref[0]
        dy = xy[1] - self._scroll_ref[1]
        self._scroll_ref = xy

        # 判定主轴：纵向滚动要求横向位移足够小
        if abs(dy) >= abs(dx) or abs(dx) <= self.cfg.scroll_axis_ratio * abs(dy):
            axis, delta = "v", dy
        else:
            axis, delta = "h", dx

        if axis != self._scroll_axis:
            self._scroll_axis = axis
            self._scroll_acc = 0.0
        self._scroll_acc += delta

        dz = max(1e-6, self.cfg.scroll_deadzone)
        notches = 0
        if abs(self._scroll_acc) >= dz:
            notches = int(self._scroll_acc / dz)
            self._scroll_acc -= notches * dz
        if notches:
            events.append(
                GestureEvent(Gesture.SCROLL, t, {"axis": axis, "notches": notches})
            )
        return events

    # ------------------------------------------------------------------ #
    def _flush_pending_tap(self, t: float) -> List[GestureEvent]:
        """双击窗口过期后，把押着的第一次单击放出去。"""
        pending = self._pending_tap
        if pending is None:
            return []
        if (t - pending.timestamp) * 1000.0 > self.cfg.double_tap_ms:
            self._pending_tap = None
            return [GestureEvent(Gesture.TAP, pending.timestamp, dict(pending.data, deferred=False))]
        return []

    def force_stop(self) -> List[GestureEvent]:
        """急停时调用：把按下的键都放掉。"""
        events: List[GestureEvent] = []
        now = time.perf_counter()
        if self._pending_tap is not None:  # 别把用户那次点击吞掉
            events.append(GestureEvent(Gesture.TAP, self._pending_tap.timestamp, dict(self._pending_tap.data)))
            self._pending_tap = None
        if self._dragging:
            self._dragging = False
            events.append(GestureEvent(Gesture.DRAG_END, now, {"reason": "estop"}))
        self._down = False
        self._long_fired = False
        self._freeze_move = False
        self._scroll_mode = False
        self._scroll_acc = 0.0
        self._scroll_ref = None
        self._contact_peak = 0.0
        return events


def apply_scroll_direction(notches: int, axis: str, invert: bool = False) -> int:
    """把「阵列上向下移动」换算成系统滚轮方向。

    阵列坐标 y 向下增大；Windows 滚轮正值向上滚动，因此取反。
    """
    return notches if invert else -notches
