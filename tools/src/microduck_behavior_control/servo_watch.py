"""Continuously print PRESENT/TORQUE for one or more servos. Read-only.

Why this exists
---------------
Calibrating a joint's mechanical limits needs live numbers while a human moves
the part by hand. The existing tools all fall short for that:

  * Pi ``servo_status.py`` hard-codes ``range(1, 15)`` -- it never queries ID15,
    which is the mouth servo we need to calibrate.
  * ``snapshot.py`` wraps that same script, so it inherits the same blind spot.
  * ``servo_goto`` can read any ID but prints one line and exits.

This tool opens the bus, reads the requested IDs in a loop, and prints them.
It never enables torque and never writes a goal, so a hand-held calibration is
safe. ``--seconds 0`` runs until Ctrl+C.

Usage::

    # watch the mouth servo while moving it by hand
    python -m microduck_behavior_control.servo_watch --id 15

    # one-shot read of several IDs
    python -m microduck_behavior_control.servo_watch --ids 13,14,15 --once
"""
from __future__ import annotations

import argparse
import base64
import subprocess
from typing import List

from .ssh_readonly import PiConnection

_SCRIPT_TEMPLATE = '''
from dynamixel_sdk import PortHandler, PacketHandler
import sys
import time

ADDR_TORQUE = 64
ADDR_PRESENT = 132
ids = {ids!r}
seconds = {seconds}
hz = {hz}
once = {once}

port = PortHandler("/dev/ttyACM0")
packet = PacketHandler(2.0)
if not port.openPort() or not port.setBaudRate(1000000):
    raise SystemExit("port open failed")


def read1(sid, addr):
    value, comm, err = packet.read1ByteTxRx(port, sid, addr)
    return (value, comm, err)


def read4(sid, addr):
    value, comm, err = packet.read4ByteTxRx(port, sid, addr)
    return (value, comm, err)


# Header tells the user what the columns mean.
print("ids=" + ",".join(str(i) for i in ids) + " (read-only, no torque, no goal)")
print("   t     " + "  ".join("ID%-2d present torque" % i for i in ids))
if once:
    print()

try:
    period = 1.0 / max(0.5, hz)
    t0 = time.perf_counter()
    last = None
    while True:
        t = time.perf_counter() - t0
        cells = []
        for sid in ids:
            present, c1, e1 = read4(sid, ADDR_PRESENT)
            torque, c2, e2 = read1(sid, ADDR_TORQUE)
            if c1 != 0 or e1 != 0 or c2 != 0 or e2 != 0:
                cells.append("ID%-2d ERROR(c=%d,e=%d/%d,%d)" % (sid, c1, e1, c2, e2))
                continue
            cells.append("ID%-2d %7d   %d" % (sid, present, torque))
        line = "  ".join(cells)
        # In one-shot mode print regardless; in watch mode suppress repeats so a
        # still servo does not flood the terminal, but show a heartbeat.
        if once:
            print("%6.2f  %s" % (t, line))
            break
        if line != last:
            print("%6.2f  %s" % (t, line), flush=True)
            last = line
        if seconds and t >= seconds:
            break
        time.sleep(period)
finally:
    port.closePort()
if not once:
    print("done")
'''


def _run(conn: PiConnection, script: str, timeout: float) -> subprocess.CompletedProcess[str]:
    encoded = base64.b64encode(script.encode("utf-8")).decode("ascii")
    remote = (
        "cd /home/pi4/microduck-replica-next/tools/deployment_validation && "
        f"echo {encoded} | base64 -d | /home/pi4/microduck_venv/bin/python -"
    )
    return subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=15",
         "-i", str(conn.identity_file), f"{conn.user}@{conn.host}", remote],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=timeout,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--id", type=int, help="single servo ID to watch")
    group.add_argument("--ids", help="comma separated servo IDs, e.g. 13,14,15")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after N seconds (0 = until Ctrl+C)")
    parser.add_argument("--hz", type=float, default=5.0, help="poll rate (default 5 Hz)")
    parser.add_argument("--once", action="store_true", help="read one sample and exit")
    args = parser.parse_args()

    if args.id is not None:
        ids: List[int] = [args.id]
    else:
        ids = [int(p.strip()) for p in args.ids.split(",") if p.strip()]
    if not ids:
        raise SystemExit("no servo IDs given")

    script = _SCRIPT_TEMPLATE.format(
        ids=ids,
        seconds=float(args.seconds),
        hz=float(args.hz),
        once=bool(args.once),
    )
    timeout = 30.0 if args.once else (args.seconds + 30.0 if args.seconds else 600.0)
    result = _run(PiConnection(), script, timeout=timeout)
    print(result.stdout.rstrip())
    if result.stderr.strip():
        print(result.stderr.strip())
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
