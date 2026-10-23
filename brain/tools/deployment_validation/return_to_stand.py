#!/usr/bin/env python3
"""Smoothly return every leg joint from its present position to standing.

Use after a crash/freeze/power event where the robot is left holding an
arbitrary pose with torque on.  Requires all servos torque-enabled.
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

ALL_IDS = tuple(range(1, 15))


def ease(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(__file__).with_name("standing_zero_calibration.json"),
    )
    args = parser.parse_args()

    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    rad_by_id = {
        int(calibration["motor_id"][name]): float(value)
        for name, value in calibration["raw_position_rad"].items()
    }
    stand = {
        sid: round(2048 + rad_by_id[sid] * 4096 / (2 * math.pi))
        for sid in ALL_IDS
    }

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
        torque = {sid: read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
        present = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ALL_IDS}
        print(f"PRESENT {present}", flush=True)
        off = [sid for sid in ALL_IDS if torque[sid] != 1]
        if off:
            # Servos are limp (e.g. after a power cut): lock them at their
            # current positions first so enabling torque does not jerk.
            print(f"TORQUE_OFF_IDS={off} — locking at present positions", flush=True)
            for sid in off:
                write4(sid, ADDR_GOAL_POSITION, present[sid])
            for sid in off:
                write1(sid, ADDR_TORQUE_ENABLE, 1)
            torque = {sid: read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
            if not all(value == 1 for value in torque.values()):
                raise RuntimeError(f"failed to re-enable torque: {torque}")
            present = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ALL_IDS}
            print("TORQUE_RESTORED at current pose", flush=True)
        print("现在开始：2 秒内平滑回到站姿，请扶稳机器人。", flush=True)
        frames = max(1, round(args.duration * args.rate))
        t0 = time.perf_counter()
        for frame in range(1, frames + 1):
            u = ease(frame / frames)
            for sid in ALL_IDS:
                write4(
                    sid,
                    ADDR_GOAL_POSITION,
                    int(round(present[sid] + u * (stand[sid] - present[sid]))),
                )
            remaining = t0 + frame / args.rate - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
        time.sleep(0.3)
        final = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in ALL_IDS}
        print(f"FINAL_STAND {final}", flush=True)
        bad = {
            sid: final[sid]
            for sid in ALL_IDS
            if abs(final[sid] - stand[sid]) > 40
        }
        print("RETURN_OK" if not bad else f"RETURN_UNSETTLED {bad}", flush=True)
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
