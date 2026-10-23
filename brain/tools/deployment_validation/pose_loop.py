#!/usr/bin/env python3
"""Replay the captured poses in a loop, one full rotation per --cycle seconds.

Rotation: sit -> lift-left -> sit -> lift-right -> sit -> tilt-head -> sit.
Every servo is left in position mode; each transition is a cosine ramp from
the last commanded counts to the stored pose, so the loop is smooth and
repeatable and no two leg poses ever change at the same time.

Started at boot by microduck-pose-loop.service.
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

HERE = Path(__file__).resolve().parent

ROTATION = ("lift_left", "lift_right", "tilt_head")
POSE_FILES = {
    "sit": "sit_pose.json",
    "lift_left": "lift_left_pose.json",
    "lift_right": "lift_right_pose.json",
    "tilt_head": "tilt_head_pose.json",
}


def ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument(
        "--cycle",
        type=float,
        default=10.0,
        help="seconds for one full rotation of all three poses; whatever is "
        "left after the motions and holds is spent resting in sit",
    )
    parser.add_argument(
        "--ease",
        type=float,
        default=0.9,
        help="seconds for each pose transition; 0 drives at max servo speed",
    )
    parser.add_argument("--hold-action", type=float, default=0.6)
    parser.add_argument("--hold-return", type=float, default=0.2)
    parser.add_argument(
        "--cycles",
        type=int,
        default=0,
        help="0 loops forever, otherwise stop after this many rotations",
    )
    args = parser.parse_args()

    poses = {
        name: {
            int(sid): int(count)
            for sid, count in json.loads(
                (HERE / filename).read_text(encoding="utf-8")
            ).items()
        }
        for name, filename in POSE_FILES.items()
    }
    ids = sorted(poses["sit"])

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
        current = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ids}
        # Aim at the present position before enabling, so powering the torque
        # on can never make a servo jump.
        for sid in ids:
            write4(sid, ADDR_GOAL_POSITION, current[sid])
        for sid in ids:
            if read1(sid, ADDR_TORQUE_ENABLE) != 1:
                write1(sid, ADDR_TORQUE_ENABLE, 1)
        print(
            f"POSE_LOOP ready poses={sorted(poses)} cycle={args.cycle:.1f}s "
            f"ease={args.ease:.2f}s ids={len(ids)}",
            flush=True,
        )

        def rest(duration: float) -> None:
            end = time.perf_counter() + duration
            while time.perf_counter() < end:
                time.sleep(0.01)

        def goto(target: dict[int, int], duration: float) -> None:
            if duration <= 0.0:
                for sid in ids:
                    write4(sid, ADDR_GOAL_POSITION, target[sid])
                    current[sid] = target[sid]
                deadline = time.perf_counter() + 2.0
                while time.perf_counter() < deadline:
                    time.sleep(0.02)
                    if all(
                        abs(read4(sid, ADDR_PRESENT_POSITION) - target[sid])
                        <= 20
                        for sid in ids
                    ):
                        break
                return
            begin = {sid: current[sid] for sid in ids}
            frames = max(1, round(duration * args.rate))
            start = time.perf_counter()
            for frame in range(1, frames + 1):
                u = ease(frame / frames)
                for sid in ids:
                    value = round(begin[sid] + u * (target[sid] - begin[sid]))
                    write4(sid, ADDR_GOAL_POSITION, value)
                    current[sid] = value
                remaining = start + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)

        index = 0
        while args.cycles == 0 or index < args.cycles:
            index += 1
            started = time.perf_counter()
            for name in ROTATION:
                goto(poses[name], args.ease)
                rest(args.hold_action)
                goto(poses["sit"], args.ease)
                rest(args.hold_return)
            spent = time.perf_counter() - started
            slack = args.cycle - spent
            print(
                f"CYCLE {index} motions={spent:.2f}s rest={slack:.2f}s",
                flush=True,
            )
            if slack > 0:
                rest(slack)
            else:
                print(f"CYCLE_OVERRUN {spent - args.cycle:.2f}s", flush=True)
        return 0
    finally:
        port.closePort()


if __name__ == "__main__":
    raise SystemExit(main())