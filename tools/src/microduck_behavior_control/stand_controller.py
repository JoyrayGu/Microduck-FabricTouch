"""Offline stand-sequence simulator; never used as a live bus adapter."""
from .fake_bus import FakeBus
from .joint_model import JOINT_BY_ID
from .pose_math import cosine_trajectory, validate_move

class StandController:
    def __init__(self, bus: FakeBus, targets: dict[int, int], max_delta: int = 512):
        if not targets:
            raise ValueError("at least one calibrated target is required")
        for servo_id in targets:
            if servo_id not in JOINT_BY_ID or not JOINT_BY_ID[servo_id].participates_in_stand:
                raise ValueError(f"ID {servo_id} is not enabled for standing")
        self.bus = bus
        self.targets = dict(targets)
        self.max_delta = max_delta

    def dry_run(self) -> list[tuple[int, int, int]]:
        current = self.bus.read_positions()
        plan = []
        for servo_id, target in sorted(self.targets.items()):
            validate_move(current[servo_id], target, self.max_delta)
            plan.append((servo_id, current[servo_id], target))
        return plan

    def simulate(self, steps: int = 100) -> None:
        plan = self.dry_run()
        trajectories = {
            servo_id: cosine_trajectory(start, target, steps)
            for servo_id, start, target in plan
        }
        ids = list(trajectories)
        self.bus.lock_current(ids)
        self.bus.set_torque(ids, True)
        try:
            for index in range(steps):
                for servo_id in ids:
                    self.bus.write_goal(servo_id, trajectories[servo_id][index])
        finally:
            self.bus.set_torque(ids, False)
