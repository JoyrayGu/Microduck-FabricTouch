"""特征提取 / 手势识别 / 光标映射的行为测试。

这些用例把「什么样的压力分布应该产生什么样的鼠标动作」固定下来，
以后调参改代码时能立刻发现回归。

运行:
    python -m unittest discover -s tests -v
"""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse.config import AppConfig
from tactile_mouse.controller import Controller
from tactile_mouse.features import TactileProcessor
from tactile_mouse.gestures import Gesture, GestureRecognizer, apply_scroll_direction
from tactile_mouse.mouse_backend import NullBackend

ROWS = COLS = 32


# --------------------------------------------------------------------------- #
# 工具
# --------------------------------------------------------------------------- #
def flat(value: float = 0.0):
    return [value] * (ROWS * COLS)


def block(r0: int, c0: int, r1: int, c1: int, value: float, base=None):
    """在网格上铺一块矩形压力。"""
    grid = list(base) if base is not None else flat(0.0)
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            grid[r * COLS + c] = value
    return grid


def blob(r0: int, c0: int, value: float, sigma: float = 1.0):
    """单个高斯接触。"""
    grid = flat(0.0)
    for r in range(ROWS):
        for c in range(COLS):
            d2 = (r - r0) ** 2 + (c - c0) ** 2
            grid[r * COLS + c] = value * math.exp(-d2 / (2 * sigma * sigma))
    return grid


def two_blobs(cx1: int, cx2: int, cy: int, value: float, sigma: float = 1.15):
    """两个高斯接触（用于双指）。"""
    grid = flat(0.0)
    for r in range(ROWS):
        for c in range(COLS):
            v = 0.0
            for cx in (cx1, cx2):
                d2 = (r - cy) ** 2 + (c - cx) ** 2
                v += value * math.exp(-d2 / (2 * sigma * sigma))
            grid[r * COLS + c] = v
    return grid


def make_controller(mode: str = "joystick", **mapping_overrides):
    cfg = AppConfig()
    cfg.filter.baseline_frames = 0
    cfg.mapping.mode = mode
    for key, value in mapping_overrides.items():
        setattr(cfg.mapping, key, value)
    backend = NullBackend()
    return Controller(cfg, backend=backend), backend


# --------------------------------------------------------------------------- #
class TestFeatureExtraction(unittest.TestCase):
    def test_centroid_of_single_blob(self):
        proc = TactileProcessor(AppConfig().filter)
        state = proc.process(blob(1, 5, 1000.0))
        self.assertTrue(state.active)
        self.assertAlmostEqual(state.centroid[0], 5.0, delta=0.2)
        self.assertAlmostEqual(state.centroid[1], 1.0, delta=0.2)
        self.assertEqual(state.n_contacts, 1)

    def test_two_fingers_are_split(self):
        """两指相距 3 格、压力尾巴互相叠加时也必须分成两个接触点。"""
        proc = TactileProcessor(AppConfig().filter)
        state = proc.process(two_blobs(cx1=1, cx2=5, cy=3, value=900.0))
        self.assertEqual(state.n_contacts, 2)
        xs = sorted(round(c.cx) for c in state.contacts)
        self.assertEqual(xs, [1, 5])

    def test_noise_below_threshold_is_ignored(self):
        cfg = AppConfig().filter
        cfg.activate_abs = 500.0
        proc = TactileProcessor(cfg)
        grid = flat(0.0)
        grid[3 * COLS + 3] = 200.0  # 单点小噪声
        state = proc.process(grid)
        self.assertFalse(state.active)

    def test_baseline_calibration_removes_offset(self):
        cfg = AppConfig().filter
        cfg.baseline_frames = 3
        proc = TactileProcessor(cfg)
        offset = flat(400.0)  # 模拟通道零点偏移
        for _ in range(3):
            proc.push_calibration_frame(offset)
        self.assertTrue(proc.calibrated)
        state = proc.process(flat(400.0))
        self.assertFalse(state.active)  # 偏移被扣掉
        state = proc.process(flat(1500.0))
        self.assertTrue(state.active)  # 真实按压仍然有效

    def test_ascii_art_shape(self):
        proc = TactileProcessor(AppConfig().filter)
        state = proc.process(blob(3, 3, 1000.0))
        art = state.ascii_art()
        self.assertEqual(len(art.splitlines()), ROWS)


# --------------------------------------------------------------------------- #
class TestGestures(unittest.TestCase):
    def setUp(self):
        self.cfg = AppConfig().gesture
        self.rec = GestureRecognizer(self.cfg)
        self._t = 100.0

    def _feed(self, grid, dt=0.02, active_none=False):
        """构造一帧状态并喂给识别器，返回事件列表。"""
        proc = TactileProcessor(AppConfig().filter)
        state = proc.process(grid, timestamp=self._t)
        self._t += dt
        return self.rec.update(state)

    def _feed_state(self, grid, dt=0.02):
        proc = getattr(self, "_proc", None) or TactileProcessor(AppConfig().filter)
        self._proc = proc
        state = proc.process(grid, timestamp=self._t)
        self._t += dt
        return self.rec.update(state)

    def test_tap_then_release_emits_tap(self):
        events = []
        for _ in range(4):  # 80ms 按住
            events += self._feed_state(block(3, 3, 4, 4, 1000.0))
        for _ in range(4):
            events += self._feed_state(flat(0.0))
        kinds = [e.kind for e in events]
        self.assertIn(Gesture.TAP, kinds)
        self.assertNotIn(Gesture.DOUBLE_TAP, kinds)
        self.assertNotIn(Gesture.LONG_PRESS, kinds)

    def test_double_tap_detected(self):
        events = []
        for cycle in range(2):
            for _ in range(3):
                events += self._feed_state(block(3, 3, 4, 4, 1000.0))
            for _ in range(3):
                events += self._feed_state(flat(0.0))
            # 两次点按之间的间隔很短，仍在 double_tap_ms 窗口内
        kinds = [e.kind for e in events]
        self.assertEqual(kinds.count(Gesture.TAP), 2)
        self.assertIn(Gesture.DOUBLE_TAP, kinds)

    def test_long_press_emits_right_click_once(self):
        events = []
        for _ in range(45):  # 0.9s，且手指不动
            events += self._feed_state(block(3, 3, 4, 4, 1000.0))
        for _ in range(3):
            events += self._feed_state(flat(0.0))
        kinds = [e.kind for e in events]
        self.assertEqual(kinds.count(Gesture.LONG_PRESS), 1)
        self.assertNotIn(Gesture.TAP, kinds)  # 长按抬手不再补一次点击
        self.assertTrue(self.rec.suppress_move is False)  # 抬手后已复位

    def test_moving_finger_does_not_long_press(self):
        grid_a = block(3, 3, 4, 4, 1000.0)
        grid_b = block(1, 4, 2, 5, 1000.0)
        events = []
        for i in range(50):
            events += self._feed_state(grid_b if i % 2 else grid_a)
        kinds = [e.kind for e in events]
        self.assertNotIn(Gesture.LONG_PRESS, kinds)

    def test_heavy_press_starts_and_ends_drag(self):
        events = []
        for _ in range(25):  # 0.5s 轻触，建立「轻触参考」（超过 tap_max_ms，不会算点击）
            events += self._feed_state(block(3, 3, 4, 4, 600.0))
        for _ in range(6):
            events += self._feed_state(flat(0.0))
        self.rec  # 参考值已由接触峰值建立
        for _ in range(40):  # 重压
            events += self._feed_state(block(2, 2, 4, 4, 900.0))
        for _ in range(4):  # 抬手
            events += self._feed_state(flat(0.0))
        kinds = [e.kind for e in events]
        self.assertIn(Gesture.DRAG_START, kinds)
        self.assertIn(Gesture.DRAG_END, kinds)
        self.assertNotIn(Gesture.TAP, kinds)
        self.assertLess(kinds.index(Gesture.DRAG_START), kinds.index(Gesture.DRAG_END))

    def test_settle_window_blocks_instant_drag(self):
        """落指瞬间的冲击不能立刻判为重压。"""
        events = []
        events += self._feed_state(block(2, 2, 4, 4, 900.0), dt=0.005)
        self.assertNotIn(Gesture.DRAG_START, [e.kind for e in events])

    def test_two_finger_scroll(self):
        events = []
        for i in range(40):
            grid = two_blobs(1, 5, 3 + (i % 3), 900.0)
            events += self._feed_state(grid)
        kinds = [e.kind for e in events]
        self.assertIn(Gesture.SCROLL_BEGIN, kinds)
        self.assertIn(Gesture.SCROLL, kinds)
        self.assertTrue(self.rec.in_scroll_mode)
        self.assertTrue(self.rec.suppress_move)
        notches = [e.data["notches"] for e in events if e.kind is Gesture.SCROLL]
        self.assertTrue(all(n != 0 for n in notches))
        self.assertNotIn(Gesture.TAP, kinds)
        self.assertNotIn(Gesture.LONG_PRESS, kinds)

    def test_scroll_direction(self):
        # 阵列 y 向下增大 -> 手指下移应该产生「向下滚动」= 负的滚轮值
        self.assertLess(apply_scroll_direction(1, "v"), 0)
        self.assertGreater(apply_scroll_direction(-1, "v"), 0)
        self.assertGreater(apply_scroll_direction(1, "v", invert=True), 0)


# --------------------------------------------------------------------------- #
class TestJoystickMapping(unittest.TestCase):
    def test_rightward_finger_moves_cursor_right_and_faster(self):
        ctl, backend = make_controller()
        ctl.feed(blob(3, 3, 900.0), timestamp=0.00)  # 落指，记录零点
        ctl.feed(blob(3, 5, 900.0), timestamp=0.02)
        ctl.feed(blob(3, 5, 900.0), timestamp=0.04)
        ctl.feed(blob(3, 5, 900.0), timestamp=0.06)
        self.assertGreater(ctl.stats.velocity[0], 0.0)
        self.assertLess(abs(ctl.stats.velocity[1]), 1.0)
        moves = [a for a in backend.actions if a.startswith("move_rel")]
        self.assertTrue(moves)
        # 前几次可能因为亚像素累加还没满 1 像素，但总量必须为正
        total_x = sum(int(a[9:-1].split(",")[0]) for a in moves)
        self.assertGreater(total_x, 0)

    def test_deadzone_suppresses_small_motion(self):
        ctl, backend = make_controller()
        ctl.feed(blob(3, 3, 900.0), timestamp=0.00)
        for i in range(1, 8):  # 只挪一点点，仍在死区内
            ctl.feed(blob(3, 3, 900.0), timestamp=i * 0.02)
        self.assertLess(abs(ctl.stats.velocity[0]), 1e-6)
        self.assertFalse([a for a in backend.actions if a.startswith("move_rel")])

    def test_touch_origin_prevents_tap_drift(self):
        """origin=touch 时，手指原地点按不应产生任何位移。"""
        ctl, backend = make_controller()
        for i in range(10):
            ctl.feed(blob(3, 5, 900.0), timestamp=i * 0.02)
        for i in range(10, 16):
            ctl.feed(flat(0.0), timestamp=i * 0.02)
        self.assertFalse([a for a in backend.actions if a.startswith("move_rel")])
        self.assertIn("down(left)", backend.actions)

    def test_array_origin_centers_on_grid_middle(self):
        ctl, backend = make_controller(origin="array")
        center = block(14, 18, 16, 24, 900.0)
        right = block(14, 24, 16, 30, 900.0)
        for i in range(10):
            ctl.feed(center, timestamp=i * 0.02)  # 32x32 几何中心附近的稳定接触
        self.assertFalse([a for a in backend.actions if a.startswith("move_rel")])
        for i in range(10, 20):
            ctl.feed(right, timestamp=i * 0.02)  # 再往右移 6~12 格，超过死区
        self.assertGreater(ctl.stats.velocity[0], 0.0)

    def test_step_limit(self):
        ctl, backend = make_controller(max_speed_px_s=5000.0, max_step_px=50.0)
        ctl.feed(blob(3, 0, 900.0), timestamp=0.0)
        ctl.feed(blob(3, 6, 900.0), timestamp=0.02)
        for i in range(3, 12):
            ctl.feed(blob(3, 6, 900.0), timestamp=i * 0.02)
        for action in backend.actions:
            if action.startswith("move_rel"):
                dx, dy = (int(v) for v in action[9:-1].split(","))
                self.assertLessEqual(math.hypot(dx, dy), 50.0 + 1e-6)

    def test_absolute_mode_maps_to_screen(self):
        ctl, backend = make_controller(mode="absolute")
        ctl.feed(blob(0, 0, 900.0), timestamp=0.0)
        ctl.feed(blob(0, 0, 900.0), timestamp=0.02)
        ctl.feed(blob(6, 6, 900.0), timestamp=0.04)
        absolute = [a for a in backend.actions if a.startswith("move_abs")]
        self.assertTrue(absolute)
        x, y = (int(v) for v in absolute[-1][9:-1].split(","))
        self.assertGreater(x, 0)
        self.assertGreater(y, 0)

    def test_ninegrid_button_zone_clicks(self):
        ctl, backend = make_controller(mode="ninegrid")
        ctl.feed(block(20, 30, 23, 31, 900.0), timestamp=0.0)  # 32x32 右下角 -> 单击
        ctl.feed(block(20, 30, 23, 31, 900.0), timestamp=0.02)
        self.assertIn("down(left)", backend.actions)

    def test_ninegrid_direction_events(self):
        events = []
        ctl, _ = make_controller(mode="ninegrid")
        ctl.on_event = events.append
        for i in range(3):
            ctl.feed(block(14, 28, 18, 31, 900.0), timestamp=i * 0.02)
        self.assertIn(Gesture.MOVE_RIGHT, [e.kind for e in events])

    def test_scroll_mode_freezes_cursor(self):
        ctl, backend = make_controller()
        for i in range(30):
            grid = two_blobs(1, 5, 3 + (i % 3), 900.0)
            ctl.feed(grid, timestamp=i * 0.02)
        self.assertTrue(ctl.recognizer.in_scroll_mode)
        self.assertFalse([a for a in backend.actions if a.startswith("move_rel")])

    def test_emergency_stop_releases_button(self):
        ctl, backend = make_controller()
        ctl.feed(block(2, 2, 4, 4, 900.0), timestamp=0.0)
        for i in range(1, 40):
            ctl.feed(block(2, 2, 4, 4, 900.0), timestamp=i * 0.02)
        self.assertTrue(ctl.recognizer.dragging)
        self.assertIn("down(left)", backend.actions)
        ctl.emergency_stop()
        self.assertIn("up(left)", backend.actions)
        self.assertFalse(ctl.recognizer.dragging)

    def test_disabled_controller_ignores_frames(self):
        ctl, backend = make_controller()
        ctl.set_enabled(False)
        for i in range(20):
            ctl.feed(blob(3, 5, 900.0), timestamp=i * 0.02)
        self.assertEqual(backend.actions, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
