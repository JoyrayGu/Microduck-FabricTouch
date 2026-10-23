#!/usr/bin/env python3
"""Lock the given servos at their present positions (hold the posed value).

Use after hand-posing limp joints (e.g. a forward head/neck lean) so the
posed counts are held for subsequent runs.  Does NOT return the joints to
any stored standing pose.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", default="11,12")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="write the locked counts to this JSON file",
    )
    args = parser.parse_args()
    ids = [int(part) for part in args.ids.split(",") if part.strip()]

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise SystemExit(f"cannot open {args.port} at {args.baud}")

    def check(label: str, comm: int, error: int) -> None:
        if comm != COMM_SUCCESS:
            raise RuntimeError(f"{label}: {packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {packet.getRxPacketError(error)}")

    try:
        present = {}
        for sid in ids:
            value, comm, error = packet.read4ByteTxRx(
                port, sid, ADDR_PRESENT_POSITION
            )
            check(f"ID{sid} read", comm, error)
            present[sid] = value
        # Enable first, then write the goal, per the XL330 sequence used by
        # the other tools: while torque is off the goal register follows
        # present, so enabling causes no jump and the posed value sticks.
        for sid in ids:
            comm, error = packet.write4ByteTxRx(
                port, sid, ADDR_GOAL_POSITION, present[sid]
            )
            check(f"ID{sid} goal", comm, error)
        for sid in ids:
            comm, error = packet.write1ByteTxRx(
                port, sid, ADDR_TORQUE_ENABLE, 1
            )
            check(f"ID{sid} torque on", comm, error)
        time.sleep(0.3)
        ok = True
        for sid in ids:
            torque, comm, error = packet.read1ByteTxRx(
                port, sid, ADDR_TORQUE_ENABLE
            )
            check(f"ID{sid} torque read", comm, error)
            goal, comm, error = packet.read4ByteTxRx(
                port, sid, ADDR_GOAL_POSITION
            )
            check(f"ID{sid} goal read", comm, error)
            held, comm, error = packet.read4ByteTxRx(
                port, sid, ADDR_PRESENT_POSITION
            )
            check(f"ID{sid} present read", comm, error)
            print(
                f"ID{sid} locked={present[sid]} goal={goal} "
                f"present={held} torque={torque} drift={held - present[sid]}",
                flush=True,
            )
            if torque != 1 or goal != present[sid]:
                ok = False
            if abs(held - present[sid]) > 15:
                print(f"ID{sid} WARNING: pose did not hold", flush=True)
                ok = False
        print(f"LOCKED {present}", flush=True)
        if args.save is not None:
            args.save.write_text(
                json.dumps(
                    {str(sid): present[sid] for sid in ids}, indent=2
                )
                + "\n",
                encoding="utf-8",
            )
            print(f"SAVED {args.save}", flush=True)
        raise SystemExit(0 if ok else 1)
    finally:
        port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())