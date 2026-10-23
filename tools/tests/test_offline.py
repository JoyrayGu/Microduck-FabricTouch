import unittest

import json
from pathlib import Path

from microduck_behavior_control.calibration import candidate_stand_ticks, load_calibration
from microduck_behavior_control.fake_bus import FakeBus
from microduck_behavior_control.joint_model import HARDWARE_NAMES, JOINT_BY_ID, JointConfig
from microduck_behavior_control.pose_math import angle_to_tick, shortest_single_turn_delta, validate_move
from microduck_behavior_control.stand_controller import StandController


class OfflineTests(unittest.TestCase):
    def test_fifteen_joint_mapping_includes_mouth(self):
        self.assertEqual(len(HARDWARE_NAMES), 15)
        self.assertEqual(HARDWARE_NAMES[2], "left_knee")
        self.assertEqual(HARDWARE_NAMES[7], "right_knee")
        self.assertEqual(HARDWARE_NAMES[15], "mouth")
        self.assertFalse(JOINT_BY_ID[15].participates_in_stand)

    def test_angle_conversion_requires_calibration(self):
        joint = JointConfig("test", 1, 0.0, direction=1, zero_tick=2048)
        self.assertEqual(angle_to_tick(joint, 0.0), 2048)
        self.assertGreater(angle_to_tick(joint, 0.1), 2048)
        with self.assertRaises(ValueError):
            angle_to_tick(JointConfig("uncalibrated", 2, 0.0), 0.1)

    def test_wrap_and_large_move_guard(self):
        self.assertEqual(shortest_single_turn_delta(4090, 5), 11)
        with self.assertRaises(ValueError):
            validate_move(4090, 5, max_delta=100)
        with self.assertRaises(ValueError):
            validate_move(0, 1000, max_delta=100)
        with self.assertRaises(ValueError):
            validate_move(-1, 10)

    def test_dry_run_never_writes_and_rejects_mouth(self):
        bus = FakeBus({2: 2000})
        self.assertEqual(StandController(bus, {2: 2050}).dry_run(), [(2, 2000, 2050)])
        self.assertEqual(bus.writes, [])
        with self.assertRaises(ValueError):
            StandController(FakeBus({15: 2000}), {15: 2050})

    def test_template_refuses_unvalidated_stand(self):
        path = Path(__file__).resolve().parents[1] / "config" / "calibration_template.json"
        template = load_calibration(path)
        self.assertEqual(len(template["joints"]), 15)
        with self.assertRaises(ValueError):
            candidate_stand_ticks(template)

    def test_calibration_rejects_reversed_mapping(self):
        path = Path(__file__).resolve().parents[1] / "config" / "calibration_template.json"
        template = json.loads(path.read_text(encoding="utf-8"))
        template["joints"]["7"]["name"] = "left_knee"
        with self.assertRaises(ValueError):
            from microduck_behavior_control.calibration import validate_calibration
            validate_calibration(template)

    def test_lock_precedes_torque_and_motion(self):
        bus = FakeBus({2: 2000})
        StandController(bus, {2: 2050}, max_delta=100).simulate(steps=3)
        self.assertEqual(bus.writes[:2], [("goal", 2, 2000), ("torque", 2, True)])
        self.assertEqual(bus.positions[2], 2050)
        self.assertFalse(bus.torque[2])


if __name__ == "__main__":
    unittest.main()
