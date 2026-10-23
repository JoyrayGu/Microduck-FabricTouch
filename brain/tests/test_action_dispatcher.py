# SPDX-License-Identifier: GPL-3.0-or-later
"""@file test_action_dispatcher.py
@brief 第二道安全边界（状态机）的测试。

对应 ``microduck/src/action_dispatcher.py``。核心不变量：
* 过期 / 重复 / 忙碌 / 未映射的事件一律拒绝，绝不排队更不打断。
* 接受的事件会逐帧插值，并且只触及该动作声明的舵机。
* 始终只执行一个动作（单动作互斥）。

为了让测试确定、飞快，这里注入假的时钟与 ``sleep``（不真的睡）。
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from microduck.src import action_dispatcher as D  # noqa: E402
from microduck.src import command_schema as S  # noqa: E402
from microduck.src.action_library import Action, ActionLibrary, Keyframe  # noqa: E402

NOW = 1_791_500_000_000


def make_event(**overrides) -> S.GestureEvent:
    payload = {
        "version": S.PROTOCOL_VERSION,
        "event_id": overrides.pop("event_id", "e1"),
        "gesture": overrides.pop("gesture", "tap"),
        "timestamp_ms": overrides.pop("timestamp_ms", NOW),
        "confidence": 0.9,
    }
    payload.update(overrides)
    return S.parse_event(payload)


def make_library() -> ActionLibrary:
    roll = Action(
        name="head_roll",
        frames=[Keyframe(pose={14: 2050}, duration=0.1)],
        timeout_s=2.0,
    )
    mouth = Action(
        name="open_mouth",
        frames=[
            Keyframe(pose={15: 1760}, duration=0.05),
            Keyframe(pose={15: 2174}, duration=0.05),
        ],
        timeout_s=2.0,
    )
    return ActionLibrary(
        actions={"head_roll": roll, "open_mouth": mouth},
        gesture_map={"tap": "head_roll", "single_stroke": "open_mouth",
                     "back_forth_stroke": "open_mouth"},
    )


class DispatcherHarness:
    """把 dispatcher 的副作用（每帧目标、开始/结束）都记下来。"""

    def __init__(self):
        self.steps = []
        self.started = []
        self.ended = []
        self.dispatcher = D.ActionDispatcher(
            library=make_library(),
            on_step=self.steps.append,
            on_action_start=self.started.append,
            on_action_end=lambda action, outcome: self.ended.append((action.name, outcome)),
            now_ms=lambda: NOW,
            sleep=lambda _: None,
        )


class AcceptTests(unittest.TestCase):
    def test_tap_accepted_and_runs_head_roll(self):
        h = DispatcherHarness()
        decision = h.dispatcher.submit(make_event(gesture="tap"))
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.action, "head_roll")
        self.assertEqual(h.started[0].name, "head_roll")
        self.assertEqual(h.ended[-1], ("head_roll", "done"))
        self.assertTrue(h.steps, "应产生至少一帧输出")

    def test_back_forth_stroke_runs_open_mouth(self):
        h = DispatcherHarness()
        decision = h.dispatcher.submit(make_event(gesture="back_forth_stroke", event_id="w1"))
        self.assertTrue(decision.accepted)
        self.assertEqual(decision.action, "open_mouth")

    def test_steps_only_touch_declared_servos(self):
        h = DispatcherHarness()
        h.dispatcher.submit(make_event(gesture="tap"))
        touched = set()
        for step in h.steps:
            touched.update(step.keys())
        self.assertTrue(touched.issubset({13, 14}))


class RejectTests(unittest.TestCase):
    def test_expired_event_rejected(self):
        h = DispatcherHarness()
        event = make_event(timestamp_ms=NOW - S.MAX_EVENT_AGE_MS - 5)
        decision = h.dispatcher.submit(event)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "expired")
        self.assertFalse(h.steps)

    def test_duplicate_event_rejected(self):
        h = DispatcherHarness()
        first = h.dispatcher.submit(make_event(event_id="dup"))
        second = h.dispatcher.submit(make_event(event_id="dup"))
        self.assertTrue(first.accepted)
        self.assertFalse(second.accepted)
        self.assertEqual(second.reason, "duplicate")

    def test_unmapped_gesture_rejected(self):
        h = DispatcherHarness()
        lib = ActionLibrary(actions={}, gesture_map={})
        d = D.ActionDispatcher(library=lib, now_ms=lambda: NOW, sleep=lambda _: None)
        event = make_event(gesture="tap")
        decision = d.submit(event)
        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "no_action")

    def test_busy_rejected_while_running(self):
        h = DispatcherHarness()
        reentrant = {}

        def on_start(action):
            # 动作正在跑：此时再提交必须被 busy 拒绝。
            reentrant["decision"] = h.dispatcher.submit(make_event(event_id="during"))

        h.dispatcher.on_action_start = on_start
        h.dispatcher.submit(make_event(event_id="first"))
        self.assertIn("decision", reentrant)
        self.assertFalse(reentrant["decision"].accepted)
        self.assertEqual(reentrant["decision"].reason, "busy")


class StatsTests(unittest.TestCase):
    def test_stats_track_accept_and_reject(self):
        h = DispatcherHarness()
        h.dispatcher.submit(make_event(event_id="a"))
        h.dispatcher.submit(make_event(event_id="a"))  # duplicate
        self.assertEqual(h.dispatcher.stats.received, 2)
        self.assertEqual(h.dispatcher.stats.accepted, 1)
        self.assertEqual(h.dispatcher.stats.rejected, 1)
        self.assertEqual(h.dispatcher.stats.last_action_name, "head_roll")

    def test_not_busy_after_completion(self):
        h = DispatcherHarness()
        h.dispatcher.submit(make_event())
        self.assertFalse(h.dispatcher.busy)


class ActionStartEventTests(unittest.TestCase):
    """``on_action_start`` 必须把触发本次动作的事件一并传出。

    动机：``tap`` 与 ``single_stroke`` 都映射到 ``head_roll``，只记录动作名
    无法判断是哪个手势触发的；带上 ``event_id`` 后可与 skin 侧日志精确对账。
    """

    def test_callback_receives_both_action_and_event(self):
        seen = []

        def on_start(action, event):
            seen.append((action.name, event.gesture, event.event_id))

        d = D.ActionDispatcher(
            library=make_library(),
            on_action_start=on_start,
            now_ms=lambda: NOW,
            sleep=lambda _: None,
        )
        d.submit(make_event(gesture="back_forth_stroke", event_id="abc123"))
        self.assertEqual(seen, [("open_mouth", "back_forth_stroke", "abc123")])

    def test_legacy_single_argument_callback_still_works(self):
        seen = []

        def on_start(action):  # 旧签名：只接收 action
            seen.append(action.name)

        d = D.ActionDispatcher(
            library=make_library(),
            on_action_start=on_start,
            now_ms=lambda: NOW,
            sleep=lambda _: None,
        )
        d.submit(make_event(gesture="tap"))
        self.assertEqual(seen, ["head_roll"])

    def test_callback_reassigned_after_construction_is_detected(self):
        """回调可在构造后替换（测试里常这么做），参数个数需实时判定。"""
        seen = []

        def on_start(action, event):
            seen.append((action.name, event.event_id))

        d = D.ActionDispatcher(
            library=make_library(),
            now_ms=lambda: NOW,
            sleep=lambda _: None,
        )
        d.on_action_start = on_start
        d.submit(make_event(event_id="late"))
        self.assertEqual(seen, [("head_roll", "late")])

    def test_error_inside_two_arg_callback_is_not_swallowed(self):
        """不能用 try/except TypeError 做兼容：那会吞掉回调内部的真实错误。"""
        def on_start(action, event):
            raise TypeError("内部真实错误")

        d = D.ActionDispatcher(
            library=make_library(),
            on_action_start=on_start,
            now_ms=lambda: NOW,
            sleep=lambda _: None,
        )
        with self.assertRaises(TypeError):
            d.submit(make_event())


if __name__ == "__main__":
    unittest.main()
