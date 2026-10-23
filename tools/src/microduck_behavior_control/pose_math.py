"""Pure pose and trajectory calculations; no hardware imports."""
from math import cos, pi
from .joint_model import JointConfig, TICKS_PER_TURN

def angle_to_tick(joint: JointConfig, angle_rad: float) -> int:
    if joint.direction not in (-1, 1) or joint.zero_tick is None:
        raise ValueError(f"{joint.name}: calibration is incomplete")
    raw = round(joint.zero_tick + (joint.direction * angle_rad + joint.offset_rad) * TICKS_PER_TURN / (2 * pi))
    if not joint.min_tick <= raw <= joint.max_tick:
        raise ValueError(f"{joint.name}: target tick {raw} outside {joint.min_tick}..{joint.max_tick}")
    return raw

def shortest_single_turn_delta(start: int, target: int) -> int:
    return (target - start + TICKS_PER_TURN // 2) % TICKS_PER_TURN - TICKS_PER_TURN // 2

def validate_move(start: int, target: int, max_delta: int = 512) -> None:
    if not 0 <= start < TICKS_PER_TURN or not 0 <= target < TICKS_PER_TURN:
        raise ValueError("single-turn positions must be within 0..4095")
    # The Pi executor interpolates numeric ticks directly; never interpret a
    # near-zero wrap as a short physical path without a validated calibration.
    delta = target - start
    if abs(delta) > max_delta:
        raise ValueError(f"move {start}->{target} is {delta} ticks, above limit {max_delta}")

def cosine_trajectory(start: int, target: int, steps: int) -> list[int]:
    if steps < 1:
        raise ValueError("steps must be positive")
    return [round(start + (target - start) * (0.5 - 0.5 * cos(pi * i / steps))) for i in range(1, steps + 1)]
