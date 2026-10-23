"""Deterministic bus substitute for tests and dry-run planning."""
from dataclasses import dataclass, field

@dataclass
class FakeBus:
    positions: dict[int, int]
    torque: dict[int, bool] = field(default_factory=dict)
    goals: dict[int, int] = field(default_factory=dict)
    writes: list[tuple[str, int, int | bool]] = field(default_factory=list)

    def read_positions(self) -> dict[int, int]:
        return dict(self.positions)

    def lock_current(self, ids: list[int]) -> None:
        for servo_id in ids:
            value = self.positions[servo_id]
            self.goals[servo_id] = value
            self.writes.append(("goal", servo_id, value))

    def set_torque(self, ids: list[int], enabled: bool) -> None:
        for servo_id in ids:
            self.torque[servo_id] = enabled
            self.writes.append(("torque", servo_id, enabled))

    def write_goal(self, servo_id: int, value: int) -> None:
        self.goals[servo_id] = value
        self.positions[servo_id] = value
        self.writes.append(("goal", servo_id, value))
