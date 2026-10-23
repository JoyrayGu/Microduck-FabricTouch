#!/usr/bin/env python3
"""Smoothly move the given servos to the counts stored in a pose JSON file.

Pose files written by lock_current.py (sit_pose.json, lift_left_pose.json, ...)
map servo ID -> goal counts.  This script eases every listed servo from its
present position to that stored count and then holds it, so the robot can be
returned to a captured pose after it has been moved elsewhere.

Servos that are not named in the pose file are left untouched.

With the default --duration the servos are eased along a cosine ramp, which
is gentle but slow.  Pass --duration 0 to hand the goal straight to the
servos and let position mode drive there at full speed.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132


def ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="seconds to ease from the present position to the stored pose; "
        "0 hands the goal straight to the servos so position mode moves at "
        "full speed",
    )
    parser.add_argument(
        "--settle-timeout",
        type=float,
        default=2.0,
        help="max seconds to wait for arrival when --duration is 0",
    )
    parser.add_argument(
        "--settle-time",
        type=float,
        default=0.15,
        help="hold at the target this long before the read-back check",
    )
    parser.add_argument(
        "--pose",
        type=Path,
        default=Path(__file__).with_name("sit_pose.json"),
    )
    parser.add_argument(
        "--ids",
        default=None,
        help="comma-separated subset to move; defaults to every ID in the file",
    )
    args = parser.parse_args()

    pose = {
        int(sid): int(count)
        for sid, count in json.loads(
            args.pose.read_text(encoding="utf-8")
        ).items()
    }
    ids = (
        [int(part) for part in args.ids.split(",") if part.strip()]
        if args.ids
        else sorted(pose)
    )
    missing = [sid for sid in ids if sid not in pose]
    if missing:
        raise SystemExit(f"{args.pose.name} has no entry for IDs {missing}")

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise SystemExit(f"cannot open {args.port} at {args.baud}")

    def check(label: str, comm: int, error: int) -> None:
        if comm != COMM_SUCCESS:
            raise RuntimeError(f"{label}: {packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {packet.getRxPacketError(error)}")

    def read1(sid: int, address: int) -> int:
        value, comm, error = packet.read1ByteTxRx(port, sid, address)
        check(f"ID{sid} read1", comm, error)
        return value

    def read4(sid: int, address: int) -> int:
        value, comm, error = packet.read4ByteTxRx(port, sid, address)
        check(f"ID{sid} read4", comm, error)
        return value

    def write1(sid: int, address: int, value: int) -> None:
        comm, error = packet.write1ByteTxRx(port, sid, address, value)
        check(f"ID{sid} write1", comm, error)

    def write4(sid: int, address: int, value: int) -> None:
        comm, error = packet.write4ByteTxRx(port, sid, address, value)
        check(f"ID{sid} write4", comm, error)

    try:
        present = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ids}
        print(f"POSE {args.pose.name}", flush=True)
        print(f"TARGET { {sid: pose[sid] for sid in ids} }", flush=True)
        print(f"PRESENT {present}", flush=True)
        # Aim at the present position first so enabling cannot cause a jump,
        # then ease toward the stored counts.
        for sid in ids:
            write4(sid, ADDR_GOAL_POSITION, present[sid])
        for sid in ids:
            if read1(sid, ADDR_TORQUE_ENABLE) != 1:
                write1(sid, ADDR_TORQUE_ENABLE, 1)
        if args.duration <= 0.0:
            # Max-speed path: hand the goal straight to the servos and let
            # position mode drive there as fast as it can, then poll for
            # arrival instead of waiting a fixed time.
            for sid in ids:
                write4(sid, ADDR_GOAL_POSITION, pose[sid])
            deadline = time.perf_counter() + args.settle_timeout
            arrived = False
            while time.perf_counter() < deadline:
                time.sleep(0.02)
                if all(
                    abs(read4(sid, ADDR_PRESENT_POSITION) - pose[sid]) <= 20
                    for sid in ids
                ):
                    arrived = True
                    break
            print(
                f"FAST duration=0 arrived={arrived} "
                f"waited={time.perf_counter() - (deadline - args.settle_timeout):.2f}s",
                flush=True,
            )
        else:
            frames = max(1, round(args.duration * args.rate))
            start = time.perf_counter()
            for frame in range(1, frames + 1):
                u = ease(frame / frames)
                for sid in ids:
                    write4(
                        sid,
                        ADDR_GOAL_POSITION,
                        round(present[sid] + u * (pose[sid] - present[sid])),
                    )
                remaining = start + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
        time.sleep(args.settle_time)
        final = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ids}
        print(f"FINAL {final}", flush=True)
        off = {
            sid: final[sid] - pose[sid]
            for sid in ids
            if abs(final[sid] - pose[sid]) > 15
        }
        if off:
            print(f"MOVE_INCOMPLETE {off}", flush=True)
            return 2
        print("MOVE_OK", flush=True)
        return 0
    finally:
        port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())