"""机器人手势识别器的行为测试。

固定「什么样的接触轨迹应产生什么手势」，以后调参改代码时能立刻发现回归。

这里直接构造 ``TactileState``（带 active/centroid/timestamp），绕开特征处理器，
让时序完全可控、结果确定。

运行::

    python -m unittest discover -s tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse.config import RobotGestureConfig
from tactile_mouse.features import Contact, TactileState
from tactile_mouse.robot_gestures import (
    ContactMode,
    MotionMode,
    RobotGesture,
    RobotGestureRecognizer,
)


def state(x, y, t, active=True, contacts=None):
    """构造一个带质心和可选接触点的状态帧。"""
    s = TactileState(raw=[], grid=[], peak=1.0, frame_no=0, timestamp=t)
    s.active = active
    s.centroid = (x, y) if active else None
    s.contacts = contacts or []
    return s


def release(rec, t, cfg=None, dt=0.05):
    """模拟抬手：连续喂入不活跃帧直到消抖窗口结束，返回产生的事件。

    释放消抖（release_debounce_ms）意味着单帧不活跃不足以判定抬手，必须
    持续不活跃超过该时长。测试里统一用这个辅助函数，避免每处手写循环。
    """
    window = (cfg.release_debounce_ms if cfg else RobotGestureConfig().release_debounce_ms) / 1000.0
    events = []
    end = t + window + dt * 2
    while t < end:
        events.extend(rec.update(state(0, 0, t, active=False)))
        t += dt
    return events


class TapTests(unittest.TestCase):
    def test_short_still_press_is_static_touch(self):
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        rec.update(state(20.0, 20.0, 0.00))
        rec.update(state(20.0, 20.0, 0.10))
        events = release(rec, 0.15, cfg)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].gesture, RobotGesture.STATIC_TOUCH)

    def test_long_press_is_not_static_touch(self):
        """按住超过 press_max_ms、且几乎没有位移，不算 static_touch。

        注意阈值是 press_max_ms（1200ms），不是早期的 static_touch_max_ms（320ms）：
        实测最长的一次**有意轻点**持续 750ms，用 320ms 当门槛会把真实点击
        全部丢掉。press_max_ms 专门用来排除「手掌一直搭着」这类长时间贴合。
        """
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        t = 0.0
        while t < 1.5:
            rec.update(state(20.0, 20.0, t))
            t += 0.05
        events = release(rec, t, cfg)
        self.assertTrue(all(e.gesture is not RobotGesture.STATIC_TOUCH for e in events))

    def test_medium_press_with_drift_is_static_touch(self):
        """有轻微漂移、按住 500ms —— 真实轻点的典型形态，必须判为 static_touch。

        回归用例：旧实现要求 max_move <= 0.9 且 held_ms <= 320 才给 static_touch，
        两者之间是一大片静默丢弃区间，导致实测 4 次点击只偶尔报出 1 次。
        """
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        t = 0.0
        for i in range(11):
            rec.update(state(20.0 + 0.15 * i, 20.0, t))  # 总漂移 1.5 格
            t += 0.05
        events = release(rec, t, cfg)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.STATIC_TOUCH)

    def test_brief_pressure_dropout_merges_into_one_static_touch(self):
        """压力短暂回落不应把一次点击拆成两次。

        实测一次轻点会被切成 4~5 个活跃片段；做 8 个手势曾报出 37 个事件。
        这里在接触中途插入一段 150ms 的不活跃（短于消抖窗口 350ms），
        期望最终只得到 1 个 static_touch。
        """
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        t = 0.0
        for i in range(6):
            rec.update(state(20.0, 20.0, t))
            t += 0.05
        for _ in range(3):                      # 150ms 空档 < 350ms 消抖
            rec.update(state(0, 0, t, active=False))
            t += 0.05
        for _ in range(6):                      # 压力恢复
            rec.update(state(20.0, 20.0, t))
            t += 0.05
        events = release(rec, t, cfg)
        self.assertEqual(len(events), 1, "消抖失败：一次点击被上报成 %d 个事件" % len(events))
        self.assertIs(events[0].gesture, RobotGesture.STATIC_TOUCH)

    def test_static_touch_with_clear_motion_is_not_static_touch(self):
        # 有明确、连续的移动（路径长度足够）且抬手时，应判定为 stroke
        # 而不是 static_touch —— 即使按下的总时长仍在 static_touch_max_ms 附近。
        # 路径需 >= stroke_min_path（3.0 格）才构成 stroke，故走到 6 格。
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        t = 0.0
        for i in range(7):
            rec.update(state(10.0 + 1.0 * i, 20.0, t))
            t += 0.04
        events = release(rec, t, cfg)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.DYNAMIC_TOUCH)


class StrokeTests(unittest.TestCase):
    def _stroke(self, xs, y=20.0, dt=0.04, rec=None):
        rec = rec or RobotGestureRecognizer(RobotGestureConfig())
        t = 0.0
        for x in xs:
            rec.update(state(x, y, t))
            t += dt
        return rec, t

    def test_monotonic_stroke_is_dynamic_touch(self):
        rec, t = self._stroke([10, 12, 14, 16, 18, 20, 22])
        events = release(rec, t)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].gesture, RobotGesture.DYNAMIC_TOUCH)
        self.assertEqual(events[0].direction, "left_to_right")

    def test_right_to_left_direction(self):
        rec, t = self._stroke([24, 22, 20, 18, 16, 14, 12])
        events = release(rec, t)
        self.assertEqual(events[0].gesture, RobotGesture.DYNAMIC_TOUCH)
        self.assertEqual(events[0].direction, "right_to_left")

    def test_back_and_forth_is_dynamic_touch(self):
        # 向右再向左，形成来回。
        # 分类依据已从 "reversals 计数" 改为 "net/path 直线度"：
        # 单程接近 1.0，来回显著小于 1.0。
        xs = [10, 13, 16, 19, 22, 19, 16, 14]
        rec, t = self._stroke(xs)
        events = release(rec, t)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].gesture, RobotGesture.DYNAMIC_TOUCH)
        self.assertLess(events[0].data.get("straightness", 1.0), 0.55)

    def test_returns_to_start_is_back_forth(self):
        """走回起点：净位移≈0 —— 这是最典型的「来回抚摸」。

        旧实现要求 net >= stroke_min_net 才判 stroke，于是来回抚摸（net≈0）
        被整个丢弃。实测 run8-backforth3 有一段 path=42.2 / net=0.5，
        直线度仅 0.012，是标准来回，却被静默丢掉，导致 head_wag 无法触发。
        """
        xs = [10, 13, 16, 19, 22, 19, 16, 13, 10]
        rec, t = self._stroke(xs)
        events = release(rec, t)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.DYNAMIC_TOUCH)
        self.assertLess(events[0].data.get("straightness", 1.0), 0.55)

    def test_dynamic_touch_requires_release(self):
        rec, _ = self._stroke([10, 12, 14, 16])
        # 尚未抬手：不应产生事件
        events = rec.update(state(16.0, 20.0, 1.0))
        self.assertEqual(events, [])


class MultiContactSemanticTests(unittest.TestCase):
    @staticmethod
    def _contact(x, y=10.0):
        return Contact(cx=x, cy=y, weight=10.0, peak=5.0, cells=3)

    def test_static_multiple_contact_marks_multi_and_stationary(self):
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        contacts = [self._contact(8.0), self._contact(20.0)]
        for i in range(15):
            rec.update(state(14.0, 10.0, i * 0.05, contacts=contacts))
        events = release(rec, 0.75, cfg)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.STATIC_TOUCH)
        self.assertIs(events[0].contact_mode, ContactMode.MULTI)
        self.assertIs(events[0].motion_mode, MotionMode.STATIONARY)
        self.assertEqual(events[0].stationary_contacts, 2)

    def test_static_multi_contact_centroid_jitter_does_not_accumulate_as_slip(self):
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        for i in range(18):
            x = 14.0 + (0.1 if i % 2 else -0.1)
            contacts = [self._contact(8.0 + x - 14), self._contact(20.0 + x - 14)]
            rec.update(state(x, 10.0, i * 0.05, contacts=contacts))
        events = release(rec, 0.90, cfg)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.STATIC_TOUCH)

    def test_centroid_tracking_jump_is_not_counted_as_dynamic_path(self):
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        points = [(10.0, 10.0), (10.1, 10.0), (17.0, 10.0), (17.1, 10.0)]
        for i, (x, y) in enumerate(points):
            rec.update(state(x, y, i * 0.05))
        events = release(rec, 0.90, cfg)
        self.assertEqual(len(events), 1)
        self.assertIs(events[0].gesture, RobotGesture.STATIC_TOUCH)

    def test_partial_slip_counts_one_moving_one_stationary(self):
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        for i in range(15):
            contacts = [self._contact(8.0 + i * 0.5), self._contact(22.0)]
            rec.update(state(15.0 + i * 0.2, 10.0, i * 0.05, contacts=contacts))
        events = release(rec, 0.75, cfg)
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertIs(event.contact_mode, ContactMode.MULTI)
        self.assertIs(event.motion_mode, MotionMode.PARTIAL_SLIP)
        self.assertEqual(event.moving_contacts, 1)
        self.assertEqual(event.stationary_contacts, 1)

    def test_partial_slip_survives_pressure_order_change(self):
        """Contact identity must not follow pressure-weight sorting."""
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        for i in range(12):
            moving = self._contact(8.0 + i * 0.4)
            stationary = self._contact(22.0)
            if i % 2:
                contacts = [stationary, moving]
            else:
                contacts = [moving, stationary]
            rec.update(state(15.0, 10.0, i * 0.05, contacts=contacts))
        events = release(rec, 0.60, cfg)
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertIs(event.motion_mode, MotionMode.PARTIAL_SLIP)
        self.assertEqual(event.moving_contacts, 1)
        self.assertEqual(event.stationary_contacts, 1)


class ContactStateTests(unittest.TestCase):
    def test_in_contact_reflects_active(self):
        """in_contact 应反映「是否处于接触中」，含消抖挂起期。

        抬手瞬间压力落到不活跃时，识别器仍保留本次接触的状态（等消抖窗口
        确认），因此 in_contact 此时仍为 True；窗口过后才变 False。
        """
        cfg = RobotGestureConfig()
        rec = RobotGestureRecognizer(cfg)
        rec.update(state(20.0, 20.0, 0.0))
        self.assertTrue(rec.in_contact)
        rec.update(state(0, 0, 0.1, active=False))
        self.assertTrue(rec.in_contact, "消抖挂起期内仍应视为接触中")
        release(rec, 0.1, cfg)
        self.assertFalse(rec.in_contact)

    def test_force_stop_clears_state(self):
        rec = RobotGestureRecognizer(RobotGestureConfig())
        rec.update(state(20.0, 20.0, 0.0))
        rec.force_stop()
        self.assertFalse(rec.in_contact)

    def test_no_event_without_contact(self):
        rec = RobotGestureRecognizer(RobotGestureConfig())
        events = rec.update(state(0, 0, 0.0, active=False))
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()
