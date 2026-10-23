"""Offline calibration and stand-plan reporting."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .calibration import load_calibration, candidate_stand_ticks
from .joint_model import HARDWARE_NAMES


def report(path: str | Path) -> str:
    data = load_calibration(path)
    lines = [
        f"calibration={Path(path)}",
        f"protocol={data['hardware']['protocol']} baud={data['hardware']['baud']}",
        "ID NAME VALIDATED DIRECTION ZERO MIN MAX STAND",
    ]
    complete = True
    targets: dict[int, int] = {}
    try:
        targets = candidate_stand_ticks(data)
    except ValueError:
        complete = False
    for servo_id, name in HARDWARE_NAMES.items():
        item = data["joints"][str(servo_id)]
        values = [
            str(servo_id), name, str(bool(item.get("validated", False))),
            str(item.get("direction")), str(item.get("zero_tick")),
            str(item.get("min_tick")), str(item.get("max_tick")), "-",
        ]
        if servo_id in targets:
            values[-1] = str(targets[servo_id])
        elif not item.get("participates_in_stand", True):
            values[-1] = "excluded"
        else:
            complete = False
        lines.append(" ".join(values))
    lines.append("STAND_PLAN=READY" if complete else "STAND_PLAN=REFUSED calibration incomplete")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration", type=Path)
    args = parser.parse_args()
    print(report(args.calibration))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
