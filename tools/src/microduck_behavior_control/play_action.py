"""Play one brain action's keyframe sequence directly on the Pi.

Purpose: feel an action's speed and amplitude on real hardware BEFORE wiring it
into brain. brain stays untouched and in dry-run; this script drives the same
servo IDs and the same tick targets that ``action_library.py`` would use.

Why not just enable brain hardware?
  Enabling brain hardware also arms the TCP path, the gesture recogniser, and the
  dispatcher. When you only want to judge "is 0.25s per swing the right feel?",
  that is a lot of moving parts. This tool plays one fixed sequence and exits.

Safety behaviour
  * dry-run by default; ``--execute`` is required to touch the bus.
  * refuses if the first keyframe is more than ``--max-first-delta`` from the
    present position (the first keyframe has duration 0.0, so it is written
    instantly -- a large gap would be a sudden jump).
  * enables torque only on the IDs the sequence actually names.
  * on any failure, restores the original positions and disables torque.
  * always ends with torque released unless ``--hold`` is given.
"""
from __future__ import annotations

import argparse
import base64
import re
import subprocess
from typing import Dict, List, Sequence, Tuple

from .ssh_readonly import PiConnection

#: One frame: (targets, duration_seconds). duration 0.0 = write immediately.
Frame = Tuple[Dict[int, int], float]

#: tilt_head with the faster timing being evaluated (1.4s total).
#:
#: 1881 = visual centre, 2050 = right end, 1712 = left end. All three were
#: verified on hardware with <=3 tick error (see action_library.HEAD_ROLL_*).
TILT_HEAD_FAST: List[Frame] = [
    ({14: 1881}, 0.0),   # centre
    ({14: 2050}, 0.25),  # swing right
    ({14: 2050}, 0.15),  # hold
    ({14: 1881}, 0.25),  # back to centre
    ({14: 1712}, 0.25),  # swing left
    ({14: 1712}, 0.15),  # hold
    ({14: 1881}, 0.25),  # back to centre
]

#: Original timing (3.2s total) for side-by-side comparison.
TILT_HEAD_SLOW: List[Frame] = [
    ({14: 1881}, 0.0),
    ({14: 2050}, 0.6),
    ({14: 2050}, 0.4),
    ({14: 1881}, 0.6),
    ({14: 1712}, 0.6),
    ({14: 1712}, 0.4),
    ({14: 1881}, 0.6),
]

#: tilt_head with the mid-swing stop REMOVED (1.30s total, same as fast).
#:
#: Difference from TILT_HEAD_FAST: that version returns to the centre tick
#: between the right and left swings, i.e.
#:     centre -> right -> HOLD -> CENTRE -> left -> hold -> centre
#: Because the interpolator uses cosine easing (zero velocity at every keyframe
#: boundary), the explicit centre frame makes the head decelerate to a full stop
#: mid-swing and then accelerate again. Visually that reads as a hitch.
#:
#: This version swings right -> left in ONE segment, so the head only decelerates
#: at the two extremes, like a pendulum:
#:     centre -> right -> hold -> left -> hold -> centre
#: Total duration and peak angular speed are unchanged (338 ticks in 0.50s).
TILT_HEAD_SMOOTH: List[Frame] = [
    ({14: 1881}, 0.0),   # centre (instant seed)
    ({14: 2050}, 0.25),  # swing right
    ({14: 2050}, 0.15),  # hold right
    ({14: 1712}, 0.50),  # swing left THROUGH the centre, no stop
    ({14: 1712}, 0.15),  # hold left
    ({14: 1881}, 0.25),  # settle back to centre
]

#: Mouth (ID15). Calibrated 2026-10-22 on the current duck:
#:   closed limit  2188  (hand-held to the stop, torque off)
#:   open   limit  1711  (hand-held to the stop, torque off)
#:   working open  1760  (actively driven, ~11% margin, verified stable, no noise)
#: Decreasing tick opens the mouth; increasing tick closes it.
#:
#: Working closed is 2174, NOT the 2188 hard limit: actively driving to 2188
#: lands at FINAL=2174 with ERR=-14, i.e. the spring resists and the servo
#: cannot reach it. Commanding the hard limit would leave the servo pushing at
#: full torque on every close, which heats the motor for no visual gain.
MOUTH_ID = 15
MOUTH_CLOSED = 2174
MOUTH_OPEN = 1760

#: Mouth open-then-close, 0.65s total, with a deliberately minimal hold.
#:
#: Hold is kept at 0.05s rather than 0.0: with 0.0 the open and close targets
#: are written on consecutive frames, so the servo receives the reversal before
#: it has travelled, and the motion reads as a twitch instead of a visible open.
MOUTH_OPEN_CLOSE: List[Frame] = [
    ({MOUTH_ID: MOUTH_CLOSED}, 0.00),  # start from closed (instant seed)
    ({MOUTH_ID: MOUTH_OPEN}, 0.30),    # open
    ({MOUTH_ID: MOUTH_OPEN}, 0.05),    # hold, just enough to be seen
    ({MOUTH_ID: MOUTH_CLOSED}, 0.30),  # close
]

ACTIONS = {
    "tilt_head_fast": TILT_HEAD_FAST,
    "tilt_head_slow": TILT_HEAD_SLOW,
    "tilt_head_smooth": TILT_HEAD_SMOOTH,
    "mouth_open_close": MOUTH_OPEN_CLOSE,
}

#: Interpolation rate, matching brain's ActionDispatcher.DEFAULT_RATE_HZ.
RATE_HZ = 40.0

_SCRIPT_TEMPLATE = '''
from dynamixel_sdk import PortHandler, PacketHandler
import math
import time

ADDR_TORQUE = 64
ADDR_GOAL = 116
ADDR_PRESENT = 132
RATE_HZ = {rate_hz}
frames = {frames!r}
ids = sorted({{sid for t, _ in frames for sid in t}})
max_first_delta = {max_first_delta}
execute = {execute}
hold_at_end = {hold_at_end}

port = PortHandler("/dev/ttyACM0")
packet = PacketHandler(2.0)
if not port.openPort() or not port.setBaudRate(1000000):
    raise SystemExit("port open failed")


def check(label, comm, err):
    if comm != 0:
        raise RuntimeError(label + " comm=" + str(comm))
    if err != 0:
        raise RuntimeError(label + " error=" + str(err))


def _retry(fn, label, attempts=3, delay=0.05):
    """Run a dynamixel_sdk call, retrying transient comm failures.

    The OpenRB bus intermittently answers -3001 (COMM_RX_TIMEOUT) when commands
    are sent back-to-back without a gap. A short retry clears it; a genuine
    wiring fault still fails all attempts.
    """
    last = None
    for i in range(attempts):
        comm, err = fn()
        if comm == 0 and err == 0:
            return
        last = (comm, err)
        time.sleep(delay)
    check(label, last[0], last[1])


def _write_goal(sid, value):
    _retry(lambda: packet.write4ByteTxRx(port, sid, ADDR_GOAL, value),
           "write goal " + str(sid))


def _write_torque(sid, value):
    _retry(lambda: packet.write1ByteTxRx(port, sid, ADDR_TORQUE, value),
           "write torque " + str(sid))


originals = {{}}
torqued = []
try:
    # 1) Read present position and torque state for every ID we will touch.
    for sid in ids:
        tq, c, e = packet.read1ByteTxRx(port, sid, ADDR_TORQUE)
        check("read torque " + str(sid), c, e)
        pos, c2, e2 = packet.read4ByteTxRx(port, sid, ADDR_PRESENT)
        check("read present " + str(sid), c2, e2)
        originals[sid] = (pos, tq)
        print("ID" + str(sid) + " present=" + str(pos) + " torque=" + str(tq))

    # 2) Refuse if the instant first keyframe would jump too far.
    first = frames[0][0]
    for sid, target in sorted(first.items()):
        pos = originals[sid][0]
        delta = target - pos
        print("first-frame ID" + str(sid) + " target=" + str(target)
              + " delta=" + str(delta))
        if abs(delta) > max_first_delta:
            raise SystemExit("REFUSED: first keyframe would jump "
                             + str(abs(delta)) + " ticks on ID" + str(sid))

    if not execute:
        print("DRY_RUN: no torque enabled, no goal written")
        raise SystemExit(0)

    # 3) Lock every ID at its present position, then enable torque.
    #    连续读写之间必须留出间隔：实测在同一个 try 块里连发多条指令时，
    #    回包会出现 comm=-3001（通信失败）。servo_goto 每次写完都 sleep 0.2s，
    #    这里沿用同样的节奏，并在失败时重试一次。
    for sid in ids:
        _write_goal(sid, originals[sid][0])
    time.sleep(0.2)
    for sid in ids:
        _write_torque(sid, 1)
        torqued.append(sid)
    time.sleep(0.25)

    # 4) Play the keyframes with cosine easing, like ActionDispatcher.
    current = {{sid: originals[sid][0] for sid in ids}}
    t_start = time.perf_counter()
    for targets, duration in frames:
        begin = dict(current)
        for sid, value in targets.items():
            current[sid] = value
        if duration <= 0.0:
            for sid, value in targets.items():
                _write_goal(sid, value)
            continue
        steps = max(1, int(round(duration * RATE_HZ)))
        for i in range(1, steps + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * i / steps)
            for sid in targets:
                value = int(round(begin[sid] + u * (targets[sid] - begin[sid])))
                _write_goal(sid, value)
            time.sleep(1.0 / RATE_HZ)
    elapsed = time.perf_counter() - t_start
    print("PLAYED in " + str(round(elapsed, 2)) + " s")

    time.sleep(0.3)
    for sid in ids:
        pos, c, e = packet.read4ByteTxRx(port, sid, ADDR_PRESENT)
        if c == 0 and e == 0:
            print("final ID" + str(sid) + "=" + str(pos)
                  + " (target " + str(current[sid]) + ")")
except SystemExit:
    raise
except Exception as exc:
    print("FAILED: " + str(exc))
    for sid, (pos, tq) in originals.items():
        try:
            packet.write4ByteTxRx(port, sid, ADDR_GOAL, pos)
        except Exception:
            pass
    raise
finally:
    if not hold_at_end:
        for sid in torqued:
            try:
                packet.write1ByteTxRx(port, sid, ADDR_TORQUE, 0)
            except Exception:
                pass
        if torqued:
            print("TORQUE_OFF " + str(torqued))
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
    parser.add_argument("--action", default="tilt_head_fast", choices=sorted(ACTIONS),
                        help="which keyframe sequence to play")
    parser.add_argument("--max-first-delta", type=int, default=200,
                        help="refuse if the instant first keyframe is further than this")
    parser.add_argument("--hold", action="store_true",
                        help="leave torque ON at the end (release on the Pi afterwards)")
    parser.add_argument("--execute", action="store_true",
                        help="actually drive the servos; without it this is a dry run")
    args = parser.parse_args()

    frames: Sequence[Frame] = ACTIONS[args.action]
    ids = sorted({sid for targets, _ in frames for sid in targets})
    total = sum(d for _, d in frames)
    print(f"action={args.action} ids={ids} keyframes={len(frames)} "
          f"nominal={total:.2f}s")

    script = _SCRIPT_TEMPLATE.format(
        rate_hz=RATE_HZ,
        frames=[(dict(sorted(t.items())), d) for t, d in frames],
        max_first_delta=args.max_first_delta,
        execute=bool(args.execute),
        hold_at_end=bool(args.hold),
    )
    result = _run(PiConnection(), script, timeout=120)
    print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip())
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
