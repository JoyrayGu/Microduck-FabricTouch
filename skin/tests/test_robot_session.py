"""机器人模式会话的集成测试。

验证 ``RobotSession`` 把帧喂进识别器、产生的手势通过 ``RobotLink`` 送出去，
并且默认 **dry-run 不联网**。用一个假的 link 捕获发送内容，避免真实网络。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse.config import AppConfig
from tactile_mouse.features import TactileState
from tactile_mouse.robot_gestures import RobotGesture, RobotGestureEvent, RobotGestureRecognizer
from tactile_mouse.robot_session import RobotSession


class _CapturingLink:
    """假的 RobotLink：记录 send 到的事件，不真的联网。"""

    def __init__(self):
        self.sent = []
        self.started = False

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def send(self, event, event_id=None):
        self.sent.append(event)
        return True


def _session(dry_run=False):
    cfg = AppConfig()
    link = _CapturingLink()
    sess = RobotSession(cfg=cfg, link=link, on_status=lambda m: None, dry_run=dry_run)
    return sess, link


class DryRunTests(unittest.TestCase):
    def test_dry_run_does_not_send(self):
        sess, link = _session(dry_run=True)
        # 绕开传输层，直接喂状态给识别器路径。
        sess._emit(RobotGestureEvent(RobotGesture.TAP, timestamp=1.0))
        self.assertEqual(link.sent, [])

    def test_emit_sends_when_not_dry_run(self):
        sess, link = _session(dry_run=False)
        sess._emit(RobotGestureEvent(RobotGesture.TAP, timestamp=1.0))
        self.assertEqual(len(link.sent), 1)


class FeedTests(unittest.TestCase):
    def _blocks(self, sess, blocks, dt=0.04):
        """把压力帧序列喂进会话，时间轴在多次调用之间**连续**。

        block 为 (r0,c0,r1,c1,value) 矩形；None 表示空帧。

        时间戳必须连续：识别器的释放消抖依赖 ``t - _pending_release_at``。
        早期实现每次调用都从 t=0 重新计时，导致第二次调用传入的时间戳比
        上一次的末尾还小，消抖计时变成负数、永远不触发。
        """
        rows, cols = sess.processor.rows, sess.processor.cols
        events = []
        t = getattr(self, "_t", 0.0)
        for spec in blocks:
            grid = [0.0] * (rows * cols)
            if spec is not None:
                r0, c0, r1, c1, value = spec
                for r in range(r0, r1 + 1):
                    for c in range(c0, c1 + 1):
                        grid[r * cols + c] = value
            sess.feed(grid, frame_no=0, timestamp=t)
            t += dt
        self._t = t
        return events

    def test_tap_reaches_link(self):
        sess, link = _session(dry_run=False)
        # 先用静止帧完成零点校准（baseline_frames 默认 >0 时会自动收集）。
        for _ in range(sess.cfg.filter.baseline_frames or 1):
            self._blocks(sess, [None])
        # 短促按压后松开。
        # 抬手判定要求空档内压力真正归零（release_zero_peak=1.0）：空帧本身
        # 是 0，但时域 EMA 有残留，需要足够多帧才会衰减到 1.0 以下。
        # 因此这里给足帧数（不是仅依赖 release_debounce_ms 的时长）。
        release_frames = 30
        self._blocks(
            sess,
            [
                (10, 10, 12, 12, 800.0),
                (10, 10, 12, 12, 800.0),
                (10, 10, 12, 12, 800.0),
                (10, 10, 12, 12, 800.0),
            ]
            + [None] * release_frames,
        )
        self.assertTrue(any(e.gesture is RobotGesture.TAP for e in link.sent),
                        f"未识别到 tap: {link.sent}")


if __name__ == "__main__":
    unittest.main()
