# SPDX-License-Identifier: GPL-3.0-or-later
"""@file test_action_library.py
@brief 动作白名单的测试。

对应 ``microduck/src/action_library.py``。核心不变量：
* 白名单只包含**纯头部**动作（第一批不改变重心）。
* 手势 -> 动作的映射不会把外部输入引到未知动作。
"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from microduck.src import action_library as L  # noqa: E402


class DefaultLibraryTests(unittest.TestCase):
    def setUp(self):
        self.lib = L.ActionLibrary()

    def test_expected_actions_present(self):
        # 动作已全部改为实测常量，不再依赖姿态 JSON 文件。
        self.assertIn("head_roll", self.lib.names)
        self.assertIn("open_mouth", self.lib.names)
        self.assertIn("head_wag", self.lib.names)  # 备用，无手势映射

    def test_head_roll_only_touches_roll_servo(self):
        action = self.lib.get("head_roll")
        self.assertIsNotNone(action)
        self.assertEqual(action.ids, [L.HEAD_ROLL_ID])

    def test_head_roll_swings_to_both_calibrated_extremes(self):
        action = self.lib.get("head_roll")
        values = {f.pose.get(L.HEAD_ROLL_ID) for f in action.frames
                  if L.HEAD_ROLL_ID in f.pose}
        self.assertEqual(
            values,
            {L.HEAD_ROLL_CENTER, L.HEAD_ROLL_RIGHT, L.HEAD_ROLL_LEFT},
        )

    def test_head_roll_has_no_mid_swing_stop(self):
        """右→左必须是一段连续运动，中间不能插回正帧。

        插值器在每个关键帧边界速度归零（余弦缓动）。若在中间插入中位，
        头会在中途完全停住再重新加速，观感是「摆到一半卡一下」——
        2026-10-22 实机手感验证确认了这一点，去掉后用户评价「观感足够好」。
        """
        action = self.lib.get("head_roll")
        values = [f.pose.get(L.HEAD_ROLL_ID) for f in action.frames
                  if L.HEAD_ROLL_ID in f.pose]
        right_idx = values.index(L.HEAD_ROLL_RIGHT)
        left_idx = values.index(L.HEAD_ROLL_LEFT)
        between = values[right_idx:left_idx]
        self.assertNotIn(
            L.HEAD_ROLL_CENTER, between,
            "右→左之间不应出现中位帧（会造成中途急停）",
        )

    def test_open_mouth_only_touches_mouth_servo(self):
        action = self.lib.get("open_mouth")
        self.assertIsNotNone(action)
        self.assertEqual(action.ids, [L.MOUTH_ID])

    def test_open_mouth_returns_to_closed(self):
        action = self.lib.get("open_mouth")
        values = [f.pose.get(L.MOUTH_ID) for f in action.frames
                  if L.MOUTH_ID in f.pose]
        self.assertEqual(values[0], L.MOUTH_CLOSED, "必须从闭合开始")
        self.assertEqual(values[-1], L.MOUTH_CLOSED, "必须回到闭合")
        self.assertIn(L.MOUTH_OPEN, values)

    def test_open_mouth_keeps_margin_from_hard_limit(self):
        """张开工作位不能等于实测硬限位 1711，必须留余量。"""
        self.assertGreater(L.MOUTH_OPEN, 1711,
                           "张开工作位应大于硬限位，留出机械余量")
        self.assertGreaterEqual(L.MOUTH_OPEN - 1711, 30, "余量过小")

    def test_closed_is_not_the_hard_limit(self):
        """闭合目标不能等于实测硬限位 2188。

        主动驱动到 2188 时实测只能到 2174（ERR=-14），弹簧挡住了。若把目标
        写成硬限位，每次闭合舵机都会顶到最大扭矩去够一个到不了的位置。
        """
        self.assertLess(L.MOUTH_CLOSED, 2188, "闭合工作位应低于硬限位")

    def test_head_wag_only_touches_head(self):
        action = self.lib.get("head_wag")
        self.assertIsNotNone(action)
        self.assertTrue(set(action.ids).issubset(L.HEAD_IDS))

    def test_head_wag_sways_both_directions(self):
        action = self.lib.get("head_wag")
        yaws = [frame.pose.get(13) for frame in action.frames if 13 in frame.pose]
        # ID13 的视觉中位是实测的 1854，不是 2048。必须相对真实中位判断方向。
        self.assertTrue(
            any(y is not None and y < L.HEAD_YAW_CENTER for y in yaws), "应有向左偏"
        )
        self.assertTrue(
            any(y is not None and y > L.HEAD_YAW_CENTER for y in yaws), "应有向右偏"
        )

    def test_head_wag_uses_symmetric_calibrated_span(self):
        """两端相对中位对称，且幅度等于实测的 ±169 tick。

        旧的镜像公式算出 2360，比对称值多 337 tick，会撞线缆限位。
        """
        action = self.lib.get("head_wag")
        yaws = {frame.pose.get(13) for frame in action.frames if 13 in frame.pose}
        yaws.discard(None)
        self.assertEqual(yaws, {L.HEAD_YAW_LEFT, L.HEAD_YAW_CENTER, L.HEAD_YAW_RIGHT})
        self.assertEqual(L.HEAD_YAW_CENTER - L.HEAD_YAW_LEFT, 169)
        self.assertEqual(L.HEAD_YAW_RIGHT - L.HEAD_YAW_CENTER, 169)

    def test_head_wag_leaves_coupled_roll_servo_alone(self):
        """ID14 与 ID13 机械耦合，不应被同时给定目标（否则两舵机互相顶住）。"""
        action = self.lib.get("head_wag")
        for frame in action.frames:
            self.assertNotIn(L.HEAD_ROLL_ID, frame.pose)

    def test_unknown_action_not_found(self):
        self.assertIsNone(self.lib.get("small_bow"))
        self.assertFalse(self.lib.has("lift_left"))


class ResolveTests(unittest.TestCase):
    def setUp(self):
        self.lib = L.ActionLibrary()

    def test_tap_maps_to_head_roll(self):
        """短促接触 -> 歪头（只动 ID14）。"""
        action = self.lib.resolve("tap")
        self.assertIsNotNone(action)
        self.assertEqual(action.name, "head_roll")

    def test_single_stroke_maps_to_open_mouth(self):
        """单向滑动 -> 张嘴再闭合（只动 ID15）。"""
        action = self.lib.resolve("single_stroke")
        self.assertEqual(action.name, "open_mouth")

    def test_back_forth_stroke_maps_to_open_mouth(self):
        """来回抚摸 -> 同一个张嘴动作。"""
        action = self.lib.resolve("back_forth_stroke")
        self.assertEqual(action.name, "open_mouth")

    def test_head_wag_has_no_gesture_mapping(self):
        """head_wag 已完成标定但当前无手势触发它（备用动作）。"""
        mapped = set(L.DEFAULT_GESTURE_TO_ACTION.values())
        self.assertNotIn("head_wag", mapped)
        self.assertIn("head_wag", self.lib.names)

    def test_unknown_gesture_resolves_to_none(self):
        self.assertIsNone(self.lib.resolve("dance"))

    def test_mapping_to_missing_action_resolves_none(self):
        lib = L.ActionLibrary(actions={}, gesture_map={"tap": "head_roll"})
        self.assertIsNone(lib.resolve("tap"))


class CustomLibraryTests(unittest.TestCase):
    def test_explicit_action_and_map(self):
        action = L.Action(
            name="nod",
            frames=[L.Keyframe(pose={11: 2100}, duration=0.1)],
        )
        lib = L.ActionLibrary(actions={"nod": action}, gesture_map={"tap": "nod"})
        self.assertEqual(lib.names, ["nod"])
        self.assertIs(lib.resolve("tap"), action)


if __name__ == "__main__":
    unittest.main()
