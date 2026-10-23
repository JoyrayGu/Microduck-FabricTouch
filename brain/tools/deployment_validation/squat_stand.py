#!/usr/bin/env python3
"""Teach and replay a squat-stand cycle with IMU pitch assist.

teach mode:
  Relax the pitch-plane leg joints (right ID1/2/3, left ID6/7/8), let the
  user pose the squat bottom by hand (feet flat, torso upright), capture
  the six joint counts into squat_pose.json, then re-enable and smoothly
  return to standing.  Run it from an interactive terminal.

run mode:
  One bounded squat-stand cycle: stand -> squat (captured pose) -> hold
  -> stand, with IMU pitch feedback applied to both hip pitches and both
  ankles (double support, both legs correct together).  Any instability
  triggers a smooth recovery to standing.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "microduck" / "src"))
from imu_reader import ThreadedIMUReader, quat_apply_inverse  # noqa: E402

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_GOAL_CURRENT = 102
MODE_POSITION = 3
MODE_CURRENT_BASED_POSITION = 5

ALL_IDS = tuple(range(1, 15))
SQUAT_IDS = (1, 2, 3, 6, 7, 8)
RUN_IDS = (1, 2, 3, 4, 6, 7, 8, 9)

# Verified standing gravity in the IMU frame for this robot on this table
# (measured by imu_static_check.py: three sensor restarts agreed to 0.0 deg
# and matched the raw accelerometer within 1.7 deg).  A run whose live
# baseline deviates from this means the robot was tilted/held at the start,
# which silently corrupts every downstream pitch measurement.
STANDING_GRAVITY_REF = (-0.969, 0.116, -0.218)

# Verified operating points, selected with --preset.  Every entry only
# supplies argparse *defaults*, so an explicit flag still wins.
PRESETS: dict[str, dict[str, object]] = {
    # Full-depth squat-stand, pure open loop.  Reproduced 5/5 on 2026-09-30
    # once the pitch/roll balance loop was fully disabled (the loop was what
    # destabilised every earlier run); see the handoff note, section 11.
    "squat-stand": {
        "squat_time": 1.0,
        "hold_time": 1.0,
        "stand_time": 1.0,
        "squat_depth": 1.0,
        "pitch_kp": 0.0,
        "pitch_kd": 0.0,
        "roll_kp": 0.0,
        "ankle_gain": 0.0,
        "pitch_target": 0.0,
        "pitch_target_early": 0.0,
        "cog_ff": 0.0,
        "attitude_tol": 180.0,
    },
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def ease(value: float) -> float:
    return 0.5 - 0.5 * math.cos(math.pi * clamp(value, 0.0, 1.0))


def cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def load_stand(calibration_path: Path) -> dict[int, int]:
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    rad_by_id = {
        int(calibration["motor_id"][name]): float(value)
        for name, value in calibration["raw_position_rad"].items()
    }
    return {
        sid: round(2048 + rad_by_id[sid] * 4096 / (2 * math.pi))
        for sid in ALL_IDS
    }


def main() -> int:
    preset_probe = argparse.ArgumentParser(add_help=False)
    preset_probe.add_argument("--preset", choices=sorted(PRESETS))
    preset = PRESETS.get(preset_probe.parse_known_args()[0].preset or "", {})

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("teach", "capture", "run"), required=True
    )
    parser.add_argument(
        "--preset",
        choices=sorted(PRESETS),
        default=None,
        help="use a verified operating point as the defaults for the run "
        "options below; any flag given explicitly still overrides it",
    )
    parser.add_argument("--countdown", type=float, default=3.0)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument(
        "--trace",
        action="store_true",
        help="print every goal-position frame send() writes, with a running "
        "index and timestamp: the full data-sending stream without pausing "
        "the real-time loop (about 40 lines/s at the default rate)",
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(__file__).with_name("standing_zero_calibration.json"),
    )
    parser.add_argument(
        "--poses",
        type=Path,
        default=Path(__file__).with_name("squat_pose.json"),
    )
    # run options
    parser.add_argument(
        "--squat-time", type=float, default=preset.get("squat_time", 2.0)
    )
    parser.add_argument(
        "--hold-time", type=float, default=preset.get("hold_time", 1.0)
    )
    parser.add_argument(
        "--stand-time", type=float, default=preset.get("stand_time", 2.0)
    )
    parser.add_argument(
        "--squat-depth",
        type=float,
        default=preset.get("squat_depth", 1.0),
        help="fraction of the taught squat to actually reach (0..1); start "
        "shallow and deepen once a depth is proven stable",
    )
    parser.add_argument(
        "--pitch-kp", type=float, default=preset.get("pitch_kp", 3.5)
    )
    parser.add_argument(
        "--pitch-kd", type=float, default=preset.get("pitch_kd", 0.5)
    )
    parser.add_argument("--pitch-clamp", type=float, default=100.0)
    parser.add_argument(
        "--squat-pitch-ff",
        type=float,
        default=0.0,
        help="depth-proportional pitch bias (counts) added while squatting; "
        "negative leans backward and counters the forward CoG drift of "
        "the interpolated descent path",
    )
    parser.add_argument(
        "--cog-ff",
        type=float,
        default=preset.get("cog_ff", 0.0),
        help="depth-proportional centre-of-gravity feedforward (counts) "
        "applied to the hip pitches and ankles while squatting; shifts the "
        "body ahead of the PD loop so the interpolated descent path keeps "
        "the CoG over the feet. 0 disables",
    )
    parser.add_argument(
        "--pitch-target",
        type=float,
        default=preset.get("pitch_target", 0.0),
        help="attitude shaping: desired pitch (deg) ramped with squat depth; "
        "the PD loop actively holds this forward lean instead of fighting "
        "it, keeping the CoG over the feet during the fold",
    )
    parser.add_argument(
        "--pitch-target-early",
        type=float,
        default=preset.get("pitch_target_early", None),
        help="pitch target (deg) held at the START of the descent, blending "
        "to --pitch-target at full depth; trades backward margin early "
        "against forward margin at the bottom. Defaults to --pitch-target.",
    )
    parser.add_argument(
        "--squat-gate-pitch",
        type=float,
        default=6.0,
        help="max |pitch| (deg) the bottom must hold for the cycle to count "
        "as stable; a stricter-than-necessary value aborts runs that squatted "
        "cleanly but were placed a degree or two off level",
    )
    parser.add_argument("--squat-gate-rate", type=float, default=15.0)
    parser.add_argument(
        "--attitude-tol",
        type=float,
        default=preset.get("attitude_tol", 12.0),
        help="max allowed deviation (deg) of the live start attitude from "
        "the stored standing reference; raise it to bypass the check when "
        "the robot is deliberately placed differently",
    )
    parser.add_argument(
        "--ankle-pre",
        type=float,
        default=0.0,
        help="staged descent: counts of ankle-only pre-tilt applied before "
        "the knee/hip fold starts, relative to the squat's ankle direction "
        "(negative leans the other way); 0 disables staging",
    )
    parser.add_argument(
        "--ankle-time",
        type=float,
        default=0.8,
        help="seconds for the ankle-only pre-tilt phases",
    )
    parser.add_argument(
        "--roll-kp",
        type=float,
        default=preset.get("roll_kp", 0.0),
        help="lateral stabilisation gain (counts per unit gravity-Y error) "
        "driving both hip rolls (ID4/ID9) back toward the start baseline; "
        "0 disables",
    )
    parser.add_argument("--roll-clamp", type=float, default=50.0)
    parser.add_argument(
        "--ankle-gain", type=float, default=preset.get("ankle_gain", 0.8)
    )
    parser.add_argument("--ankle-clamp", type=float, default=60.0)
    parser.add_argument(
        "--goal-current",
        type=int,
        default=0,
        help="if >0, leg servos use current-based position mode with this "
        "goal current in mA; restored to position mode on exit",
    )
    args = parser.parse_args()

    stand = load_stand(args.calibration)

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise RuntimeError(f"cannot open {args.port} at {args.baud}")

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

    def write2(sid: int, address: int, value: int) -> None:
        comm, error = packet.write2ByteTxRx(port, sid, address, value)
        check(f"ID{sid} write2", comm, error)

    def write4(sid: int, address: int, value: int) -> None:
        comm, error = packet.write4ByteTxRx(port, sid, address, value)
        check(f"ID{sid} write4", comm, error)

    send_index = 0
    send_t0 = time.perf_counter()

    def send(targets: dict[int, float]) -> None:
        nonlocal send_index
        for sid, value in targets.items():
            write4(sid, ADDR_GOAL_POSITION, int(round(value)))
        if args.trace:
            send_index += 1
            counts = " ".join(
                f"{sid}:{int(round(targets.get(sid, stand[sid]))):4d}"
                for sid in RUN_IDS
            )
            print(
                f"TX #{send_index:05d} "
                f"t={time.perf_counter() - send_t0:7.3f}s {counts}",
                flush=True,
            )

    imu: ThreadedIMUReader | None = None
    try:
        if args.mode == "capture":
            # Non-interactive timed capture for unstable-SSH environments:
            # countdown, sample, print.  Does not change any servo state.
            print(
                "现在开始采集：请摆好蹲姿底部并保持不动。"
                f"{args.countdown:.0f} 秒后开始采样。",
                flush=True,
            )
            remaining = args.countdown
            while remaining > 0:
                print(f"CAPTURE_IN {math.ceil(remaining)}", flush=True)
                step = min(1.0, remaining)
                time.sleep(step)
                remaining -= step
            samples = {sid: [] for sid in SQUAT_IDS}
            end = time.perf_counter() + 1.5
            while time.perf_counter() < end:
                for sid in SQUAT_IDS:
                    samples[sid].append(read4(sid, ADDR_PRESENT_POSITION))
                time.sleep(0.05)
            mean = {
                sid: statistics.fmean(values) for sid, values in samples.items()
            }
            std = {
                sid: statistics.pstdev(values)
                for sid, values in samples.items()
            }
            print(
                f"CAPTURE_STD { {sid: round(v, 2) for sid, v in std.items()} }",
                flush=True,
            )
            if max(std.values()) > 8.0:
                print("CAPTURE_UNSTABLE — 重新运行并拿稳", flush=True)
                return 2
            print(
                "CAPTURE_POSE "
                + json.dumps(
                    {
                        str(sid): int(round(mean[sid])) for sid in SQUAT_IDS
                    }
                ),
                flush=True,
            )
            print(
                "CAPTURE_DELTA "
                + json.dumps(
                    {
                        str(sid): int(round(mean[sid] - stand[sid]))
                        for sid in SQUAT_IDS
                    }
                ),
                flush=True,
            )
            return 0

        if args.mode == "teach":
            torque = {sid: read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
            if not all(value == 1 for value in torque.values()):
                # After a power cut the servos are limp already; lock them
                # at their settled pose first so the standing reference is
                # not lost, then relax the squat joints.
                for sid in ALL_IDS:
                    if torque[sid] != 1:
                        write4(
                            sid,
                            ADDR_GOAL_POSITION,
                            read4(sid, ADDR_PRESENT_POSITION),
                        )
                for sid in ALL_IDS:
                    if torque[sid] != 1:
                        write1(sid, ADDR_TORQUE_ENABLE, 1)
            print(
                f"TEACH_READY squat_ids={SQUAT_IDS} "
                f"stand={ {sid: stand[sid] for sid in SQUAT_IDS} }",
                flush=True,
            )
            print(
                "注意：踝(1/6)、膝(2/7)、髋俯仰(3/8)将在 3 秒后断电变松。"
                "请把机器人摆成蹲姿底部（双脚贴地、躯干竖直），一手扶稳。",
                flush=True,
            )
            for count in range(3, 0, -1):
                print(f"RELEASE_IN {count}", flush=True)
                time.sleep(1.0)
            for sid in SQUAT_IDS:
                write1(sid, ADDR_TORQUE_ENABLE, 0)
                if read1(sid, ADDR_TORQUE_ENABLE) != 0:
                    raise RuntimeError(f"ID{sid} torque did not turn off")
            print(
                "TEACH_HOLD_ACTIVE：腿已放松。请摆好蹲姿底部并保持不动，"
                "稳定后按回车。",
                flush=True,
            )
            while True:
                try:
                    input()
                except EOFError:
                    raise RuntimeError(
                        "stdin is not interactive; run teach mode from an "
                        "interactive SSH terminal"
                    )
                samples: dict[int, list[int]] = {sid: [] for sid in SQUAT_IDS}
                end = time.perf_counter() + 1.5
                while time.perf_counter() < end:
                    for sid in SQUAT_IDS:
                        samples[sid].append(read4(sid, ADDR_PRESENT_POSITION))
                    time.sleep(0.05)
                mean = {
                    sid: statistics.fmean(values)
                    for sid, values in samples.items()
                }
                std = {
                    sid: statistics.pstdev(values)
                    for sid, values in samples.items()
                }
                if max(std.values()) > 8.0:
                    print(
                        f"TEACH_UNSTABLE 抖动过大 std="
                        f"{ {sid: round(v, 2) for sid, v in std.items()} }。"
                        "请拿稳一点，稳定后再按回车。",
                        flush=True,
                    )
                    continue
                print(
                    f"TAUGHT_POSE mean="
                    f"{ {sid: round(v, 1) for sid, v in mean.items()} } "
                    f"delta_from_stand="
                    f"{ {sid: round(v - stand[sid]) for sid, v in mean.items()} }",
                    flush=True,
                )
                try:
                    answer = input("保存该蹲姿并回到站姿？[Y/n] ")
                except EOFError:
                    answer = "y"
                if answer.strip().lower() in ("", "y", "yes"):
                    break
                print("已放弃。请调整姿势后再按回车重新采集。", flush=True)

            poses = {
                "joints": {
                    str(sid): int(round(mean[sid])) for sid in SQUAT_IDS
                },
                "stand_counts": {
                    str(sid): stand[sid] for sid in SQUAT_IDS
                },
                "delta_counts": {
                    str(sid): int(round(mean[sid] - stand[sid]))
                    for sid in SQUAT_IDS
                },
                "sample_std_counts": {
                    str(sid): round(std[sid], 2) for sid in SQUAT_IDS
                },
                "captured_at": datetime.now().isoformat(timespec="seconds"),
                "note": "user-posed squat bottom; feet flat, torso upright",
            }
            args.poses.write_text(
                json.dumps(poses, indent=2) + "\n", encoding="utf-8"
            )
            print(
                f"TEACH_SAVED path={args.poses} joints={poses['joints']}",
                flush=True,
            )
            # Lock at the posed squat, then return to standing over 2s.
            for sid in SQUAT_IDS:
                write4(sid, ADDR_GOAL_POSITION, int(round(mean[sid])))
            for sid in SQUAT_IDS:
                write1(sid, ADDR_TORQUE_ENABLE, 1)
            print(
                "关节已在蹲姿重新锁定，2 秒内平滑回到站姿，请扶稳。",
                flush=True,
            )
            start = {
                sid: read4(sid, ADDR_PRESENT_POSITION) for sid in SQUAT_IDS
            }
            frames = max(1, round(2.0 * args.rate))
            t0 = time.perf_counter()
            for frame in range(1, frames + 1):
                u = ease(frame / frames)
                send(
                    {
                        sid: start[sid] + u * (stand[sid] - start[sid])
                        for sid in SQUAT_IDS
                    }
                )
                remaining = t0 + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
            final = {
                sid: read4(sid, ADDR_PRESENT_POSITION) for sid in SQUAT_IDS
            }
            print(f"FINAL_STAND {final}", flush=True)
            print("TEACH_DONE", flush=True)
            return 0

        # ---- run mode ----
        if not args.poses.is_file():
            raise RuntimeError(f"no squat pose file: {args.poses}")
        poses = json.loads(args.poses.read_text(encoding="utf-8"))
        squat = {
            int(sid): float(count) for sid, count in poses["joints"].items()
        }
        if tuple(sorted(squat)) != tuple(sorted(SQUAT_IDS)):
            raise RuntimeError(
                f"squat joints {sorted(squat)} do not match {sorted(SQUAT_IDS)}"
            )
        torque = {sid: read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
        if not all(value == 1 for value in torque.values()):
            raise RuntimeError(f"not all servos enabled: {torque}")
        if args.goal_current > 0:
            for sid in SQUAT_IDS:
                write1(sid, ADDR_TORQUE_ENABLE, 0)
                write1(sid, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                write2(sid, ADDR_GOAL_CURRENT, args.goal_current)
                write1(sid, ADDR_TORQUE_ENABLE, 1)
            print(
                f"CURRENT_MODE_ON squat_ids goal_current={args.goal_current}mA",
                flush=True,
            )
        send({sid: float(stand[sid]) for sid in RUN_IDS})
        time.sleep(0.3)

        imu = ThreadedIMUReader(i2c_bus=1, frequency_hz=args.rate)
        imu.start()
        deadline = time.perf_counter() + 3.0
        while time.perf_counter() < deadline and not imu.get_latest().valid:
            time.sleep(0.02)
        if not imu.get_latest().valid:
            raise RuntimeError(f"IMU did not become valid: {imu.get_status()}")

        def latest():
            snap = imu.get_latest()
            if not snap.valid or time.perf_counter() - snap.timestamp_s > 0.20:
                raise RuntimeError(f"IMU stale/invalid: {imu.get_status()}")
            return (
                quat_apply_inverse(snap.quat, [0.0, 0.0, 1.0]),
                snap.gyro,
            )

        def average_gravity(duration: float) -> list[float]:
            rows = []
            end = time.perf_counter() + duration
            while time.perf_counter() < end:
                rows.append(latest()[0])
                time.sleep(0.025)
            return [sum(row[i] for row in rows) / len(rows) for i in range(3)]

        # Let the robot settle, then require a clean static start: two
        # consecutive windows must agree.  A hand-held or still-settling
        # start poisons the live baseline zero and made earlier squat runs
        # fall in inconsistent directions.
        time.sleep(0.5)
        first_half = average_gravity(0.4)
        second_half = average_gravity(0.4)
        drift_cross = cross(first_half, second_half)
        drift_deg = math.degrees(
            math.atan2(
                math.sqrt(sum(v * v for v in drift_cross)),
                sum(a * b for a, b in zip(first_half, second_half)),
            )
        )
        baseline = [(first_half[i] + second_half[i]) / 2.0 for i in range(3)]
        print(
            f"BASELINE {[round(v, 6) for v in baseline]} "
            f"start_drift={drift_deg:.2f}deg "
            f"squat={ {sid: int(squat[sid]) for sid in SQUAT_IDS} }",
            flush=True,
        )
        if drift_deg > 0.8:
            print(
                "BAD_START start pose was not static "
                f"(drift={drift_deg:.2f}deg) — release the robot, let it "
                "stand free, and rerun.",
                flush=True,
            )
            return 1
        attitude_cross = cross(list(STANDING_GRAVITY_REF), baseline)
        attitude_deg = math.degrees(
            math.atan2(
                math.sqrt(sum(v * v for v in attitude_cross)),
                sum(a * b for a, b in zip(STANDING_GRAVITY_REF, baseline)),
            )
        )
        print(
            f"START_ATTITUDE deviation_from_standing={attitude_deg:.1f}deg",
            flush=True,
        )
        if attitude_deg > args.attitude_tol:
            print(
                "BAD_START the robot is not in the normal standing attitude "
                f"(off by {attitude_deg:.1f}deg, tol {args.attitude_tol:.0f}) "
                "— it is being held or leaning. Put it down, let it stand "
                "free, and rerun.",
                flush=True,
            )
            return 1
        baseline_path = args.poses.with_name("squat_last_baseline.json")
        if baseline_path.is_file():
            try:
                previous = json.loads(baseline_path.read_text(encoding="utf-8"))
                previous_cross = cross(previous, baseline)
                previous_deg = math.degrees(
                    math.atan2(
                        math.sqrt(sum(v * v for v in previous_cross)),
                        sum(a * b for a, b in zip(previous, baseline)),
                    )
                )
                if previous_deg > 3.0:
                    print(
                        "BASELINE_WARN start pose differs from the previous "
                        f"run by {previous_deg:.1f}deg (placement changed)",
                        flush=True,
                    )
            except Exception:  # noqa: BLE001
                pass
        baseline_path.write_text(
            json.dumps([round(v, 6) for v in baseline]) + "\n",
            encoding="utf-8",
        )

        filtered_gravity = list(baseline)
        last_pitch_deg: float | None = None
        filtered_pitch_rate = 0.0
        last_targets = {sid: float(stand[sid]) for sid in SQUAT_IDS}
        recovery_reason = ""

        def run_phase(
            name: str,
            duration: float,
            frac_from: float,
            frac_to: float,
            ankle_from: float | None = None,
            ankle_to: float | None = None,
        ) -> bool:
            nonlocal filtered_gravity, last_pitch_deg, filtered_pitch_rate
            nonlocal last_targets, recovery_reason
            if ankle_from is None:
                ankle_from = frac_from
            if ankle_to is None:
                ankle_to = frac_to
            frames = max(1, round(duration * args.rate))
            start = time.perf_counter()
            unstable_frames = 0
            for frame in range(1, frames + 1):
                u = ease(frame / frames)
                frac = frac_from + u * (frac_to - frac_from)
                ankle_frac = ankle_from + u * (ankle_to - ankle_from)
                gravity, gyro = latest()
                filtered_gravity = [
                    0.75 * filtered_gravity[i] + 0.25 * gravity[i]
                    for i in range(3)
                ]
                rotation_cross = cross(baseline, filtered_gravity)
                rotation_dot = sum(
                    a * b for a, b in zip(baseline, filtered_gravity)
                )
                pitch_deg = math.degrees(
                    math.atan2(rotation_cross[1], rotation_dot)
                )
                if last_pitch_deg is None:
                    raw_rate = 0.0
                else:
                    raw_rate = (pitch_deg - last_pitch_deg) * args.rate
                last_pitch_deg = pitch_deg
                filtered_pitch_rate = (
                    0.75 * filtered_pitch_rate + 0.25 * raw_rate
                )
                total_deg = math.degrees(
                    math.atan2(
                        math.sqrt(
                            sum(value * value for value in rotation_cross)
                        ),
                        rotation_dot,
                    )
                )
                gyro_norm = math.sqrt(sum(v * v for v in gyro))

                # Attitude shaping: hold a forward lean that grows with
                # depth so the CoG stays over the feet while folding.
                sp = frac / args.squat_depth if args.squat_depth else 0.0
                early = (
                    args.pitch_target_early
                    if args.pitch_target_early is not None
                    else args.pitch_target
                )
                pitch_target_deg = early + (args.pitch_target - early) * sp
                correction = clamp(
                    -args.pitch_kp * (pitch_deg - pitch_target_deg)
                    - args.pitch_kd * filtered_pitch_rate
                    + args.squat_pitch_ff * frac,
                    -args.pitch_clamp,
                    args.pitch_clamp,
                )
                ankle = clamp(
                    args.ankle_gain * correction,
                    -args.ankle_clamp,
                    args.ankle_clamp,
                )
                roll_fb = clamp(
                    args.roll_kp * (filtered_gravity[1] - baseline[1]),
                    -args.roll_clamp,
                    args.roll_clamp,
                )
                targets = {
                    sid: stand[sid]
                    + (ankle_frac if sid in (1, 6) else frac)
                    * (squat[sid] - stand[sid])
                    for sid in SQUAT_IDS
                }
                # Static probe signs: ID1+/ID3+ pitch forward, ID6+/ID8+
                # pitch backward, so the left leg uses negated offsets.
                targets[3] += correction
                targets[1] += ankle
                targets[8] -= correction
                targets[6] -= ankle
                # Depth-proportional CoG feedforward: shift the body ahead of
                # the reactive PD loop so the linearly interpolated descent
                # path does not push the CoG off the feet before the loop has
                # time to react.  The hips carry most of it; the ankles take a
                # smaller share to avoid lifting the toes.
                cog = args.cog_ff * frac
                targets[3] += cog
                targets[8] -= cog
                targets[1] += 0.3 * cog
                targets[6] -= 0.3 * cog
                # Lateral stabilisation: gravity-Y errors (lateral drift)
                # drive both hip rolls back toward the start baseline.
                targets[4] = stand[4] + roll_fb
                targets[9] = stand[9] + roll_fb
                send(targets)
                last_targets = dict(targets)

                unstable = total_deg > 16.0 or gyro_norm > 2.5
                unstable_frames = unstable_frames + 1 if unstable else 0
                if unstable_frames >= 3:
                    recovery_reason = (
                        f"attitude recovery: total={total_deg:.1f}deg "
                        f"gyro={gyro_norm:.2f}rad/s"
                    )
                    return False

                now = time.perf_counter()
                if (
                    frame <= round(args.rate)
                    or frame % max(1, round(args.rate / 4)) == 0
                ):
                    present2 = read4(2, ADDR_PRESENT_POSITION)
                    present1 = read4(1, ADDR_PRESENT_POSITION)
                    print(
                        f"{name} u={frame / frames:.2f} squat_frac={frac:.2f} "
                        f"a_frac={ankle_frac:.2f} "
                        f"pitch={pitch_deg:+.2f}deg "
                        f"pitch_rate={filtered_pitch_rate:+.1f}dps "
                        f"gy={filtered_gravity[1]:+.4f} "
                        f"knee2={present2} ankle1={present1} "
                        f"gyro_norm={gyro_norm:.2f} "
                        f"corr={correction:+.1f} ankle={ankle:+.1f} "
                        f"roll={roll_fb:+.1f}",
                        flush=True,
                    )
                remaining = start + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
            return True

        def gate(label: str, pitch_limit: float, rate_limit: float) -> bool:
            nonlocal recovery_reason
            for wait_index in range(1, 5):
                if (
                    last_pitch_deg is not None
                    and abs(last_pitch_deg) <= pitch_limit
                    and abs(filtered_pitch_rate) <= rate_limit
                ):
                    print(
                        f"{label} stable pitch={last_pitch_deg:+.2f}deg "
                        f"rate={filtered_pitch_rate:+.1f}dps",
                        flush=True,
                    )
                    return True
                print(
                    f"{label} wait={wait_index} pitch={last_pitch_deg} "
                    f"rate={filtered_pitch_rate:+.1f}dps",
                    flush=True,
                )
                if not run_phase(
                    f"WAIT_{label}", 0.30, args.squat_depth, args.squat_depth
                ):
                    return False
            recovery_reason = (
                f"{label} not stable: pitch={last_pitch_deg}deg "
                f"rate={filtered_pitch_rate:.1f}dps"
            )
            return False

        print("现在开始：蹲下-保持-站起。请护住机器人。", flush=True)
        d = args.squat_depth
        ankle_delta = squat[1] - stand[1]
        ankle_pre_frac = (
            args.ankle_pre / abs(ankle_delta) if ankle_delta else 0.0
        )
        staged = abs(args.ankle_pre) > 0.5
        if staged:
            print(
                f"STAGED ankle_pre={args.ankle_pre:+.0f}counts "
                f"ankle_frac_pre={ankle_pre_frac:+.3f} "
                f"ankle_time={args.ankle_time}s",
                flush=True,
            )
        completed = True
        if not staged:
            if not run_phase("SQUAT_DOWN", args.squat_time, 0.0, d):
                completed = False
        else:
            if not run_phase(
                "SQUAT_ANKLE", args.ankle_time, 0.0, 0.0, 0.0, ankle_pre_frac
            ):
                completed = False
            if completed and not run_phase(
                "SQUAT_FOLD", args.squat_time, 0.0, d, ankle_pre_frac, d
            ):
                completed = False
        if completed and not gate(
            "SQUAT_GATE", args.squat_gate_pitch, args.squat_gate_rate
        ):
            completed = False
        if completed and not run_phase("SQUAT_HOLD", args.hold_time, d, d):
            completed = False
        if not staged:
            if completed and not run_phase("STAND_UP", args.stand_time, d, 0.0):
                completed = False
        else:
            if completed and not run_phase(
                "STAND_FOLD", args.stand_time, d, 0.0, d, ankle_pre_frac
            ):
                completed = False
            if completed and not run_phase(
                "STAND_ANKLE", args.ankle_time, 0.0, 0.0, ankle_pre_frac, 0.0
            ):
                completed = False
        if completed and not gate("STAND_GATE", 2.5, 8.0):
            completed = False
        print(
            "SQUAT_COMPLETE" if completed else f"SQUAT_RECOVERY {recovery_reason}",
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        print("INTERRUPTED", flush=True)
        return 1
    finally:
        try:
            # Smoothly return the run joints (including the two hip rolls
            # that carried the lateral stabilisation offsets) to standing
            # from wherever they are, regardless of why we are exiting.
            start = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in RUN_IDS}
            frames = max(1, round(1.5 * args.rate))
            t0 = time.perf_counter()
            for frame in range(1, frames + 1):
                u = ease(frame / frames)
                send(
                    {
                        sid: start[sid] + u * (stand[sid] - start[sid])
                        for sid in RUN_IDS
                    }
                )
                remaining = t0 + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
            final = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in RUN_IDS}
            print(f"FINAL_STAND {final}", flush=True)
            if args.mode == "run" and args.goal_current > 0:
                for sid in SQUAT_IDS:
                    try:
                        write1(sid, ADDR_TORQUE_ENABLE, 0)
                        write1(sid, ADDR_OPERATING_MODE, MODE_POSITION)
                        write1(sid, ADDR_TORQUE_ENABLE, 1)
                    except Exception:  # noqa: BLE001
                        print(f"ID{sid} mode restore failed", flush=True)
                print("CURRENT_MODE_OFF position mode restored", flush=True)
        except Exception as exc:  # noqa: BLE001
            print(f"SAFE_RETURN_FAILED ({exc})", flush=True)
        if imu is not None:
            try:
                imu.stop()
            except Exception:
                pass
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
