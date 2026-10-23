"""Load and validate per-servo calibration data without hardware access."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .joint_model import HARDWARE_NAMES, TICKS_PER_TURN


def load_calibration(path: str | Path) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_calibration(data)
    return data


def validate_calibration(data: dict[str, Any], *, require_complete: bool = False) -> None:
    if data.get("schema_version") != 1:
        raise ValueError("unsupported calibration schema_version")
    hardware = data.get("hardware", {})
    if hardware.get("ticks_per_revolution") != TICKS_PER_TURN:
        raise ValueError("ticks_per_revolution must be 4096")
    joints = data.get("joints")
    if not isinstance(joints, dict):
        raise ValueError("joints must be an object keyed by servo ID")
    expected_ids = {str(servo_id) for servo_id in HARDWARE_NAMES}
    if set(joints) != expected_ids:
        raise ValueError("calibration must contain exactly hardware IDs 1..15")
    for key, expected_name in HARDWARE_NAMES.items():
        entry = joints[str(key)]
        if entry.get("name") != expected_name:
            raise ValueError(f"ID {key} must map to {expected_name}")
        if not entry.get("validated", False):
            if require_complete:
                raise ValueError(f"ID {key} calibration is not validated")
            continue
        if entry.get("direction") not in (-1, 1):
            raise ValueError(f"ID {key} direction must be -1 or 1")
        zero = entry.get("zero_tick")
        low, high = entry.get("min_tick"), entry.get("max_tick")
        if not isinstance(zero, int) or not 0 <= zero < TICKS_PER_TURN:
            raise ValueError(f"ID {key} zero_tick must be an integer in 0..4095")
        if not isinstance(low, int) or not isinstance(high, int) or not 0 <= low <= high < TICKS_PER_TURN:
            raise ValueError(f"ID {key} requires valid min_tick/max_tick")
        if not low <= zero <= high:
            raise ValueError(f"ID {key} zero_tick lies outside calibrated limits")
        if not isinstance(entry.get("offset_rad"), (int, float)):
            raise ValueError(f"ID {key} offset_rad must be numeric")


def candidate_stand_ticks(data: dict[str, Any]) -> dict[int, int]:
    """Convert model home angles using only fully validated joint calibrations."""
    validate_calibration(data, require_complete=True)
    from math import pi
    from .joint_model import MODEL_HOME_RAD

    targets: dict[int, int] = {}
    for servo_id, name in HARDWARE_NAMES.items():
        entry = data["joints"][str(servo_id)]
        if not entry.get("participates_in_stand", True):
            continue
        target = round(
            entry["zero_tick"]
            + (entry["direction"] * MODEL_HOME_RAD[name] + entry["offset_rad"])
            * TICKS_PER_TURN
            / (2 * pi)
        )
        if not entry["min_tick"] <= target <= entry["max_tick"]:
            raise ValueError(f"ID {servo_id} target {target} outside calibrated range")
        targets[servo_id] = target
    return targets
