# SPDX-License-Identifier: GPL-3.0-or-later
"""@file action_dispatcher.py
@brief 事件 -> 动作的状态机：白名单校验、去重、过期、单动作互斥、超时。

这是 brain 侧的**第二道安全边界**：即使事件已通过 :mod:`command_schema`，
这里仍要判断「当前是否允许执行」。

策略（Milestone 1）
-------------------
* 一次只执行一个动作；动作执行期间到来的新事件被**拒绝**，不排队、不打断。
* 同一个 ``event_id`` 只处理一次（去重窗口有限，避免内存无限增长）。
* 事件过期（见 :func:`command_schema.check_freshness`）直接拒绝。
* 手势必须能映射到一个在白名单内存在的动作。
* 执行以固定频率逐帧插值；每帧回调 ``on_step``，便于 dry-run 记录或真机写舵机。
"""

from __future__ import annotations

import inspect
import math
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Mapping, Optional

from . import command_schema as S
from .action_library import Action, ActionLibrary, Keyframe

#: 默认插值频率（Hz），与现有 pose_loop.py 一致。
DEFAULT_RATE_HZ = 40.0

#: 去重窗口内保留的最大事件数。
MAX_SEEN_EVENTS = 512


def _ease(u: float) -> float:
    """余弦缓动，与现有 pose_loop / squat_stand 一致。"""
    u = max(0.0, min(1.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


def _callback_arity(cb, default: int = 2) -> int:
    """返回回调声明的**位置参数**个数，无法判断时返回 ``default``。

    用于兼容 ``on_action_start`` 的两种签名：旧的 ``(action)`` 与新的
    ``(action, event)``。用 inspect 而不是 try/except TypeError，
    因为后者会把回调内部抛出的真实 TypeError 一并吞掉。
    """
    if cb is None:
        return default
    try:
        sig = inspect.signature(cb)
    except (TypeError, ValueError):
        return default
    params = [
        p for p in sig.parameters.values()
        if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)
    ]
    if any(p.kind == p.VAR_POSITIONAL for p in sig.parameters.values()):
        return default
    return len(params)


@dataclass
class Decision:
    """一次事件的处理结果。"""

    accepted: bool
    action: Optional[str] = None
    reason: str = "ok"


@dataclass
class DispatchStats:
    received: int = 0
    accepted: int = 0
    rejected: int = 0
    last_reason: str = ""
    running_action: Optional[str] = None
    last_action_name: Optional[str] = None


class ActionDispatcher:
    """把已校验的事件转成动作执行。

    线程模型：``submit`` 可从网络线程调用；实际执行默认在**调用线程**同步完成，
    或由外层 (server) 决定在何处调用。若动作较长，可设 ``rate_hz`` 并在真实
    执行器中保证每帧耗时可控。
    """

    def __init__(
        self,
        library: Optional[ActionLibrary] = None,
        on_step: Optional[Callable[[Mapping[int, int]], None]] = None,
        on_action_start: Optional[Callable[[Action, Optional[S.GestureEvent]], None]] = None,
        on_action_end: Optional[Callable[[Action, str], None]] = None,
        rate_hz: float = DEFAULT_RATE_HZ,
        now_ms: Optional[Callable[[], int]] = None,
        max_event_age_ms: int = S.MAX_EVENT_AGE_MS,
        sleep: Callable[[float], None] = time.sleep,
        perf: Callable[[], float] = time.perf_counter,
    ):
        self.library = library or ActionLibrary()
        self.on_step = on_step
        self.on_action_start = on_action_start
        self.on_action_end = on_action_end
        self.rate_hz = rate_hz
        self._now_ms = now_ms or (lambda: int(time.time() * 1000))
        self.max_event_age_ms = max_event_age_ms
        self._sleep = sleep
        self._perf = perf

        self.stats = DispatchStats()
        self._lock = threading.RLock()
        self._seen: "OrderedDict[str, float]" = OrderedDict()
        self._running = False
        self._stop_requested = False

    # ------------------------------------------------------------------ #
    @property
    def busy(self) -> bool:
        with self._lock:
            return self._running

    def request_stop(self) -> None:
        """请求当前动作尽快结束（用于急停）。"""
        with self._lock:
            self._stop_requested = True

    # ------------------------------------------------------------------ #
    def _remember(self, event_id: str) -> bool:
        """记录 event_id。返回 True 表示这是新事件。"""
        with self._lock:
            if event_id in self._seen:
                return False
            self._seen[event_id] = self._perf()
            while len(self._seen) > MAX_SEEN_EVENTS:
                self._seen.popitem(last=False)
            return True

    def submit(self, event: S.GestureEvent) -> Decision:
        """处理一条已通过 schema 校验的事件，返回决策。"""
        self.stats.received += 1

        # 1) 过期 / 未来
        try:
            S.check_freshness(event, self._now_ms(), max_age_ms=self.max_event_age_ms)
        except S.SchemaError as exc:
            return self._reject(exc.reason)

        # 2) 去重
        if not self._remember(event.event_id):
            return self._reject("duplicate")

        # 3) 互斥
        with self._lock:
            if self._running:
                return self._reject("busy")

        # 4) 手势 -> 动作（白名单）
        action = self.library.resolve(event.gesture)
        if action is None:
            return self._reject("no_action")

        # 5) 执行
        self._execute(action, event)
        self.stats.accepted += 1
        self.stats.last_reason = "ok"
        return Decision(accepted=True, action=action.name, reason="ok")

    def _reject(self, reason: str) -> Decision:
        self.stats.rejected += 1
        self.stats.last_reason = reason
        return Decision(accepted=False, reason=reason)

    # ------------------------------------------------------------------ #
    def _execute(self, action: Action, event: Optional[S.GestureEvent] = None) -> None:
        with self._lock:
            self._running = True
            self._stop_requested = False
            self.stats.running_action = action.name
        started = self._perf()
        outcome = "done"
        try:
            if self.on_action_start is not None:
                # 把触发本次动作的事件一并传出，便于日志把 ACTION_START 与
                # skin 侧的 event_id / 手势名对上（多个手势可能共用一个动作，
                # 只看动作名无法区分是哪一个手势触发的）。
                #
                # 兼容性：旧回调只接收 (action)。这里按回调**当前**声明的参数
                # 个数决定调用形态（回调可能在构造后被替换，例如测试里
                # 直接赋值 dispatcher.on_action_start = ...），避免用
                # try/except TypeError 捕获——那会把回调内部真实的 TypeError
                # 一并吞掉，掩盖真正的 bug。
                if _callback_arity(self.on_action_start) == 2:
                    self.on_action_start(action, event)
                else:
                    self.on_action_start(action)
            self._run_frames(action, started)
        except Exception:  # noqa: BLE001
            outcome = "error"
            raise
        finally:
            if self._stop_requested:
                outcome = "stopped"
            with self._lock:
                self._running = False
                self.stats.last_action_name = action.name
                self.stats.running_action = None
            if self.on_action_end is not None:
                self.on_action_end(action, outcome)

    def _run_frames(self, action: Action, started: float) -> None:
        """按关键帧逐段插值。未列出的 ID 保持上一帧目标。"""
        targets: Dict[int, int] = {}
        prev: Dict[int, int] = {}
        period = 1.0 / max(1.0, self.rate_hz)

        for frame in action.frames:
            for sid, value in frame.pose.items():
                targets[int(sid)] = int(value)
            begin = {sid: prev.get(sid, targets[sid]) for sid in targets}

            duration = max(0.0, float(frame.duration))
            if duration <= 0.0:
                self._emit(targets, started, action)
                prev = dict(targets)
                continue

            steps = max(1, int(round(duration * self.rate_hz)))
            for i in range(1, steps + 1):
                if self._stop_requested:
                    return
                if self._perf() - started > action.timeout_s:
                    return
                u = _ease(i / steps)
                frame_targets = {
                    sid: int(round(begin[sid] + u * (targets[sid] - begin[sid])))
                    for sid in targets
                }
                self._emit(frame_targets, started, action)
                self._sleep(period)
            prev = dict(targets)

        # 动作结束后停在最后一个关键帧的目标值。
        self._emit(targets, started, action)

    def _emit(self, targets: Mapping[int, int], started: float, action: Action) -> None:
        if self._perf() - started > action.timeout_s:
            return
        if self.on_step is not None:
            self.on_step(targets)
