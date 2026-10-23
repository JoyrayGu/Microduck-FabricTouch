#!/usr/bin/env python3
"""Disable torque on the pitch-plane leg joints for hand posing.

Default relaxed IDs: 1, 2, 3 (right ankle/knee/hip-pitch) and 6, 7, 8
(left).  Hip rolls/yaws and head servos stay enabled at their current
goals so the legs keep a symmetric stance while you pose them by hand.
"""

from __future__ import annotations

import argparse

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_TORQUE_ENABLE = 64


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--ids",
        default="1,2,3,6,7,8",
        help="comma-separated servo IDs to relax (torque off)",
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
        for sid in ids:
            comm, error = packet.write1ByteTxRx(
                port, sid, ADDR_TORQUE_ENABLE, 0
            )
            check(f"ID{sid} torque off", comm, error)
        state = {}
        for sid in ids:
            value, comm, error = packet.read1ByteTxRx(
                port, sid, ADDR_TORQUE_ENABLE
            )
            check(f"ID{sid} read", comm, error)
            state[sid] = value
        relaxed = [sid for sid in ids if state[sid] == 0]
        print(f"RELAXED_IDS={relaxed}", flush=True)
        if len(relaxed) != len(ids):
            raise SystemExit(f"some servos did not relax: {state}")
        print("腿已放松，可以手动摆姿势。", flush=True)
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
