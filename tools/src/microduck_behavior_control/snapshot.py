"""Capture Pi servo feedback by physical ID into a local JSON snapshot.

Only invokes the existing read-only servo_status.py over SSH. The parsed data
is keyed by numeric servo ID; legacy joint labels are intentionally ignored.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re

from .ssh_readonly import ReadOnlySshClient

_ROW = re.compile(
    r"^ID(?P<id>\d+) mode=(?P<mode>-?\d+) t=(?P<torque>\d+) "
    r"goal=(?P<goal>-?\d+) present=(?P<present>-?\d+)$"
)
_ERROR_ROW = re.compile(r"^ID(?P<id>\d+) COMM_ERROR (?P<error>.*)$")


def parse_status_output(text: str) -> dict[int, dict[str, int]]:
    rows: dict[int, dict[str, int]] = {}
    errors: dict[int, str] = {}
    for line in text.splitlines():
        match = _ROW.fullmatch(line.strip())
        if match:
            sid = int(match.group("id"))
            rows[sid] = {
                "mode": int(match.group("mode")),
                "torque": int(match.group("torque")),
                "goal_tick": int(match.group("goal")),
                "present_tick": int(match.group("present")),
            }
            continue
        error_match = _ERROR_ROW.fullmatch(line.strip())
        if error_match:
            errors[int(error_match.group("id"))] = error_match.group("error")
    if errors:
        raise ValueError(f"servo status contains read errors: {errors}")
    expected = set(range(1, 15))
    if set(rows) != expected:
        raise ValueError(
            f"expected physical IDs 1..14 from current Pi status script; "
            f"missing={sorted(expected - rows.keys())}, extra={sorted(rows.keys() - expected)}"
        )
    return rows


def capture_snapshot(client: ReadOnlySshClient | None = None) -> dict[str, object]:
    ssh = client or ReadOnlySshClient()
    result = ssh.run("status", timeout=25.0)
    if result.returncode != 0:
        raise RuntimeError(
            f"Pi servo status command failed (exit {result.returncode}): {result.stderr.strip()}"
        )
    rows = parse_status_output(result.stdout)
    return {
        "schema_version": 1,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "Pi servo_status.py via read-only SSH",
        "interpretation": "Physical ID keyed; joint names intentionally not inferred.",
        "pi_reported_ids": sorted(rows),
        "unqueried_ids": [15],
        "servos": {str(sid): rows[sid] for sid in sorted(rows)},
        "raw_stdout": result.stdout,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="local JSON path; no upload is performed")
    args = parser.parse_args()
    snapshot = capture_snapshot()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
    print(f"SNAPSHOT_SAVED {args.output}")
    print(f"PI_IDS={snapshot['pi_reported_ids']} UNQUERIED_IDS={snapshot['unqueried_ids']}")
    torque_on = [sid for sid, state in snapshot["servos"].items() if state["torque"] != 0]
    print(f"TORQUE_ON_IDS={torque_on}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
