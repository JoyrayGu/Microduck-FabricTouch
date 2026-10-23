"""Run a fixed set of read-only checks against the prepared Pi."""
from __future__ import annotations

import argparse
from pathlib import Path
from .ssh_readonly import PiConnection, ReadOnlySshClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--identity-file",
        default=str(PiConnection().identity_file),
        help="SSH private key path (private key stays local)",
    )
    args = parser.parse_args()
    client = ReadOnlySshClient(PiConnection(identity_file=Path(args.identity_file)))
    failed = False
    for name in (
        "identity", "files", "processes", "ports", "status",
        "calibration_readme", "constants_source", "guided_calibration_source",
        "direction_probe_source", "move_pose_source",
    ):
        print(f"--- {name} ---")
        try:
            result = client.run(name)
        except (OSError, TimeoutError) as exc:
            print(f"ERROR: {exc}")
            failed = True
            continue
        if result.stdout:
            print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
        if result.stderr:
            print(result.stderr, end="" if result.stderr.endswith("\n") else "\n")
        if result.returncode:
            print(f"exit_code={result.returncode}")
            failed = True
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
