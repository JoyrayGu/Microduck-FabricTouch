"""Move one servo to an absolute tick and hold it.

Single-servo, absolute-target counterpart to servo_probe (which is relative).

Safety behaviour:
  * refuses if the delta exceeds --max-delta (default 200 ticks)
  * refuses if torque is already on
  * reads present position and writes it as the goal before enabling torque
  * ramps in small steps instead of jumping
  * on any failure, restores the original position and disables torque

Holding is intentional: clean up on the Pi with relax_legs.py.
"""
from __future__ import annotations

import argparse
import base64
import re
import subprocess

from .ssh_readonly import PiConnection

_REMOTE_TEMPLATE = '''
from dynamixel_sdk import PortHandler, PacketHandler
import time

sid = {sid}
target = {target}
steps = {steps}
step_time = {step_time}
allow_torque_on = {allow_torque_on}
ADDR_TORQUE = 64
ADDR_GOAL = 116
ADDR_PRESENT = 132

port = PortHandler("/dev/ttyACM0")
packet = PacketHandler(2.0)
if not port.openPort() or not port.setBaudRate(1000000):
    raise SystemExit("port open failed")


def check(label, comm, err):
    if comm != 0:
        raise RuntimeError(label + " comm=" + str(comm))
    if err != 0:
        raise RuntimeError(label + " error=" + str(err))


present = None
try:
    tq, c0, e0 = packet.read1ByteTxRx(port, sid, ADDR_TORQUE)
    check("read torque", c0, e0)
    if tq != 0 and not allow_torque_on:
        raise SystemExit("REFUSED: torque already on (use --allow-torque-on)")

    present, c1, e1 = packet.read4ByteTxRx(port, sid, ADDR_PRESENT)
    check("read present", c1, e1)
    print("PRESENT=" + str(present) + " TARGET=" + str(target))

    if tq == 0:
        check("lock", *packet.write4ByteTxRx(port, sid, ADDR_GOAL, present))
        check("torque on", *packet.write1ByteTxRx(port, sid, ADDR_TORQUE, 1))
        time.sleep(0.2)
    else:
        print("TORQUE_ALREADY_ON: continuing without a new lock")

    for i in range(1, steps + 1):
        value = int(round(present + (target - present) * i / steps))
        check("ramp", *packet.write4ByteTxRx(port, sid, ADDR_GOAL, value))
        time.sleep(step_time)

    time.sleep(0.4)
    final, c2, e2 = packet.read4ByteTxRx(port, sid, ADDR_PRESENT)
    check("read final", c2, e2)
    print("FINAL=" + str(final) + " ERR=" + str(final - target))
    print("HOLDING target=" + str(target) + " torque=ON")
except SystemExit:
    raise
except Exception as exc:
    print("FAILED: " + str(exc))
    if present is not None:
        try:
            packet.write4ByteTxRx(port, sid, ADDR_GOAL, present)
            time.sleep(0.3)
        except Exception:
            pass
    try:
        packet.write1ByteTxRx(port, sid, ADDR_TORQUE, 0)
        print("TORQUE_OFF (recovered)")
    except Exception:
        print("TORQUE_OFF_FAILED")
    raise
finally:
    port.closePort()
'''

_READ_TEMPLATE = '''
from dynamixel_sdk import PortHandler, PacketHandler
port = PortHandler("/dev/ttyACM0")
packet = PacketHandler(2.0)
print("OPEN", port.openPort())
print("BAUD", port.setBaudRate(1000000))
try:
    print("PRESENT", packet.read4ByteTxRx(port, {sid}, 132))
    print("TORQUE", packet.read1ByteTxRx(port, {sid}, 64))
finally:
    port.closePort()
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--target", type=int, required=True, help="absolute goal tick 0..4095")
    parser.add_argument("--max-delta", type=int, default=200,
                        help="refuse if |target - present| exceeds this (default 200)")
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--step-time", type=float, default=0.04)
    parser.add_argument("--allow-torque-on", action="store_true",
                        help="continue even if the servo's torque is already enabled")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not 0 <= args.target <= 4095:
        raise SystemExit("refuse: target must be 0..4095")
    if args.steps < 1:
        raise SystemExit("refuse: steps must be >= 1")

    conn = PiConnection()
    probe = _run(conn, _READ_TEMPLATE.format(sid=args.id), 40)
    print(probe.stdout.strip())
    if probe.returncode != 0:
        print(probe.stderr.strip())
        return 1

    match = re.search(r"PRESENT \((\d+), (\d+), (\d+)\)", probe.stdout)
    if not match:
        print("ERROR: could not parse present position")
        return 1
    present, comm = int(match.group(1)), int(match.group(2))
    if comm != 0:
        print("ERROR: present read reported a communication failure")
        return 1

    torque_match = re.search(r"TORQUE \((\d+),", probe.stdout)
    if torque_match and int(torque_match.group(1)) != 0 and not args.allow_torque_on:
        print(f"REFUSED: ID {args.id} torque is already ON (use --allow-torque-on)")
        return 1

    delta = args.target - present
    print(f"delta={delta:+d} ticks ({delta * 360 / 4096:+.1f} deg)")
    if abs(delta) > args.max_delta:
        print(f"REFUSED: |delta| {abs(delta)} exceeds limit {args.max_delta}")
        return 1
    if not args.execute:
        print("DRY_RUN: no torque enabled, no goal written")
        return 0

    result = _run(
        conn,
        _REMOTE_TEMPLATE.format(sid=args.id, target=args.target,
                                steps=args.steps, step_time=args.step_time,
                                allow_torque_on=bool(args.allow_torque_on)),
        90,
    )
    print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip())
    return 0 if result.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
