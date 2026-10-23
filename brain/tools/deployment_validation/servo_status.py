#!/usr/bin/env python3
"""Servo status check: torque/goal/present plus operating mode.

Read-only by default.  With --fix-modes it switches the listed IDs back to
plain position control (mode 3) if they are stuck in another mode, e.g.
after an interrupted current-mode run; the switch is per-joint (torque off,
mode write, torque on) so the robot keeps standing.
"""

from __future__ import annotations

import argparse

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
MODE_POSITION = 3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fix-modes", action="store_true")
    parser.add_argument("--fix-ids", default="1,2,3,6,7,8")
    args = parser.parse_args()

    port = PortHandler("/dev/ttyACM0")
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(1_000_000):
        raise SystemExit("cannot open /dev/ttyACM0 at 1000000")

    def rd1(sid: int, addr: int) -> tuple[int, int, int]:
        return packet.read1ByteTxRx(port, sid, addr)

    try:
        modes: dict[int, int] = {}
        rows: dict[int, str] = {}
        for sid in range(1, 15):
            mode, comm_m, _ = rd1(sid, ADDR_OPERATING_MODE)
            torque, comm_t, _ = rd1(sid, ADDR_TORQUE_ENABLE)
            goal, comm_g, _ = packet.read4ByteTxRx(
                port, sid, ADDR_GOAL_POSITION
            )
            present, comm_p, _ = packet.read4ByteTxRx(
                port, sid, ADDR_PRESENT_POSITION
            )
            bad = [
                name
                for name, comm in (
                    ("m", comm_m),
                    ("t", comm_t),
                    ("g", comm_g),
                    ("p", comm_p),
                )
                if comm != COMM_SUCCESS
            ]
            if bad:
                rows[sid] = f"COMM_ERROR {bad}"
                modes[sid] = -1
                continue
            modes[sid] = mode
            rows[sid] = f"mode={mode} t={torque} goal={goal} present={present}"
        for sid in sorted(rows):
            print(f"ID{sid:02d} {rows[sid]}")
        stuck = {sid: modes[sid] for sid in sorted(modes) if modes[sid] != MODE_POSITION}
        print(
            "MODES_ALL_POSITION"
            if not stuck
            else f"NON_POSITION_MODES={stuck}"
        )
        disabled = [
            sid for sid in sorted(rows) if "t=0" in rows[sid]
        ]
        print(
            "ALL_TORQUE_ON" if not disabled else f"TORQUE_OFF_IDS={disabled}"
        )
        if args.fix_modes:
            fix_ids = [int(x) for x in args.fix_ids.split(",") if x.strip()]
            for sid in fix_ids:
                if modes.get(sid) == MODE_POSITION:
                    print(f"ID{sid} already mode 3")
                    continue
                packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 0)
                packet.write1ByteTxRx(port, sid, ADDR_OPERATING_MODE, MODE_POSITION)
                packet.write1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE, 1)
                after, _, _ = rd1(sid, ADDR_OPERATING_MODE)
                torque, _, _ = rd1(sid, ADDR_TORQUE_ENABLE)
                print(f"ID{sid} restored mode={after} torque={torque}")
            print("FIX_MODES_DONE")
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())