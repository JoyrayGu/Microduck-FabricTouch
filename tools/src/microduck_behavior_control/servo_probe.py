"""Single-servo, bounded hardware probe helper (Windows side).

This module can only run the read-only status command and one deliberately
bounded goal-position test. It never enables torque without first writing the
present position as the goal, and it always disables torque afterwards.
"""
from __future__ import annotations

import argparse
import re
import subprocess
from dataclasses import dataclass

from .ssh_readonly import PiConnection


def _ssh(conn: PiConnection, remote: str, timeout: float = 30.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8",
            "-i", str(conn.identity_file),
            f"{conn.user}@{conn.host}", remote,
        ],
        check=False, capture_output=True, text=True,
        encoding="utf-8", errors="replace", timeout=timeout,
    )


_READ = """
from dynamixel_sdk import PortHandler, PacketHandler
p = PortHandler("/dev/ttyACM0")
s = PacketHandler(2.0)
if not p.openPort() or not p.setBaudRate(1000000):
    raise SystemExit("port open failed")
try:
    pos, comm, err = s.read4ByteTxRx(p, {sid}, 132)
    tq, tc, te = s.read1ByteTxRx(p, {sid}, 64)
    print(f"POS={{pos}} POSC={{comm}} POSE={{err}} TQ={{tq}} TQC={{tc}} TQE={{te}}")
finally:
    p.closePort()
"""


def read_servo(conn: PiConnection, sid: int) -> dict[str, int]:
    result = _ssh(conn, "cd /home/pi4/microduck-replica-next/tools/deployment_validation && "
                        f"/home/pi4/microduck_venv/bin/python -c '{_READ.format(sid=sid)}'")
    if result.returncode != 0:
        raise RuntimeError(f"read failed: {result.stderr.strip()}")
    match = re.search(r"POS=(-?\d+) POSC=(-?\d+) POSE=(-?\d+) TQ=(-?\d+) TQC=(-?\d+) TQE=(-?\d+)",
                      result.stdout)
    if not match:
        raise RuntimeError(f"unexpected read output: {result.stdout!r}")
    return {
        "present": int(match.group(1)),
        "pos_comm": int(match.group(2)),
        "pos_error": int(match.group(3)),
        "torque": int(match.group(4)),
        "tq_comm": int(match.group(5)),
        "tq_error": int(match.group(6)),
    }


_TEST = """
from dynamixel_sdk import PortHandler, PacketHandler
import time

sid = {sid}
target = {target}
p = PortHandler("/dev/ttyACM0")
s = PacketHandler(2.0)
if not p.openPort() or not p.setBaudRate(1000000):
    raise SystemExit("port open failed")

def check(label, comm, err):
    if comm != 0:
        raise RuntimeError(f"{{label}} comm={{comm}}")
    if err != 0:
        raise RuntimeError(f"{{label}} error={{err}}")

try:
    present, comm, err = s.read4ByteTxRx(p, sid, 132)
    check("read present", comm, err)
    print(f"PRESENT={{present}} TARGET={{target}}")
    check("lock", *s.write4ByteTxRx(p, sid, 116, present))
    check("torque on", *s.write1ByteTxRx(p, sid, 64, 1))
    time.sleep(0.2)
    steps = 40
    for i in range(1, steps + 1):
        value = round(present + (target - present) * i / steps)
        check("ramp", *s.write4ByteTxRx(p, sid, 116, value))
        time.sleep(0.05)
    time.sleep(0.3)
    final, c2, e2 = s.read4ByteTxRx(p, sid, 132)
    print(f"FINAL={{final}} ERR={{final - target}}")
finally:
    try:
        s.write4ByteTxRx(p, sid, 116, present)
        time.sleep(0.3)
    except Exception:
        pass
    try:
        s.write1ByteTxRx(p, sid, 64, 0)
        print("TORQUE_OFF")
    except Exception:
        print("TORQUE_OFF_FAILED")
    p.closePort()
"""


@dataclass
class ServoState:
    present: int
    torque: int


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", type=int, required=True)
    parser.add_argument("--delta", type=int, default=100,
                        help="tick delta; magnitude capped at 150")
    parser.add_argument("--execute", action="store_true",
                        help="actually move; without this only reads state")
    args = parser.parse_args()

    if abs(args.delta) > 150:
        raise SystemExit("refuse: |delta| must be <= 150 ticks")
    conn = PiConnection()
    state = read_servo(conn, args.id)
    print(f"ID={args.id} PRESENT={state['present']} TORQUE={state['torque']}")
    if not args.execute:
        print("DRY_RUN: no torque enabled, no goal written")
        return 0
    if state["torque"] != 0:
        raise SystemExit("refuse: servo torque is not off before the test")
    target = state["present"] + args.delta
    if not 0 <= target <= 4095:
        raise SystemExit(f"refuse: target {target} outside 0..4095")
    result = _ssh(conn, "cd /home/pi4/microduck-replica-next/tools/deployment_validation && "
                        f"/home/pi4/microduck_venv/bin/python -c '{_TEST.format(sid=args.id, target=target)}'",
                  timeout=60.0)
    print(result.stdout)
    print(result.stderr)
    after = read_servo(conn, args.id)
    print(f"AFTER PRESENT={after['present']} TORQUE={after['torque']}")
    return 0 if result.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
