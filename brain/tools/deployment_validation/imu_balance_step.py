#!/usr/bin/env python3
"""One bounded IMU-assisted in-place step cycle for the real Microduck.

The controller shifts the trunk over the support foot with both hip-roll
servos, then lifts the other leg along a Cartesian ankle trajectory: the
ankle rises vertically by --clearance and comes straight back down, so no
intermediate frame is lower than the standing pose and the foot cannot scuff
the ground.  The hip and knee come from a sagittal 2R inverse kinematics
solve (leg_geometry.json) and the ankle counter-rotates to keep the sole
level.  ID11 stays locked at its recalibrated standing goal and is excluded
from dynamic control.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "microduck" / "src"))
from imu_reader import ThreadedIMUReader, quat_apply_inverse  # noqa: E402
from leg_geometry_calibration import (  # noqa: E402
    forward_kinematics,
    inverse_kinematics,
    model_to_counts,
)


ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE = 11
ADDR_GOAL_CURRENT = 102
MODE_POSITION = 3
MODE_CURRENT_BASED_POSITION = 5

CONTROL_IDS = (1, 2, 3, 4, 6, 7, 8, 9)


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


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--goal-current",
        type=int,
        default=0,
        help="if >0, switch the ten leg servos to current-based position "
        "mode (mode 5) with this goal current in mA; 0 keeps plain "
        "position mode.  Mode is restored to position control on exit.",
    )
    parser.add_argument("--rate", type=float, default=40.0)
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.09,
        help="legacy scale kept only for the pitch feed-forward terms; the "
        "swing-foot height is set by --clearance and no longer depends on it",
    )
    parser.add_argument("--roll-counts", type=float, default=90.0)
    parser.add_argument("--roll-gravity", type=float, default=0.070)
    parser.add_argument(
        "--roll-feedback-clamp",
        type=float,
        default=35.0,
        help="max magnitude of the roll feedback correction (counts)",
    )
    parser.add_argument("--hold-duration", type=float, default=0.06)
    parser.add_argument(
        "--lift-duration",
        type=float,
        default=0.18,
        help="seconds for the swing-foot lift phase; the 0.18s default was "
        "tuned for tiny amplitudes, use ~0.35s for the full taught lift",
    )
    parser.add_argument("--pitch-kp", type=float, default=3.5)
    parser.add_argument("--pitch-kd", type=float, default=0.5)
    parser.add_argument("--center-pitch-ff", type=float, default=30.0)
    parser.add_argument("--touchdown-pitch-ff", type=float, default=0.0)
    parser.add_argument(
        "--lift-pitch-ff",
        type=float,
        default=0.0,
        help="forward pitch feed-forward (counts) on the support hip while "
        "the swing foot is lifted; counters the backward CoG shift caused "
        "by the folding swing leg.  Scales with the current lift fraction "
        "so it decays automatically during lowering.",
    )
    parser.add_argument(
        "--pre-lean-ff",
        type=float,
        default=0.0,
        help="forward pitch feed-forward (counts) applied to the support "
        "hip as soon as the weight is shifted (before the lift starts), "
        "scaled by the roll fraction so it ramps in during SHIFT, holds "
        "through STABILIZE/LIFT and decays during CENTER",
    )
    parser.add_argument(
        "--pitch-clamp",
        type=float,
        default=70.0,
        help="max magnitude of the combined support-hip pitch offset",
    )
    parser.add_argument(
        "--gyro-gate",
        type=float,
        default=1.2,
        help="recovery threshold on the gyro norm (rad/s); fast stepping "
        "produces short lateral transients that the ankle/hip corrections "
        "can absorb, so allow somewhat larger bursts than 1.2",
    )
    parser.add_argument(
        "--ankle-gain",
        type=float,
        default=0.0,
        help="fraction of the pitch offset also applied to the SUPPORT "
        "ankle (ID1 when the right foot supports, ID6 mirrored when the "
        "left supports); the swing ankle always stays at standing",
    )
    parser.add_argument(
        "--ankle-clamp",
        type=float,
        default=40.0,
        help="max magnitude of the support-ankle pitch offset",
    )
    parser.add_argument(
        "--swing-tempo",
        type=float,
        default=1.0,
        help="time-scale for the single-support phases (lift/hold/lower); "
        "0.5 makes the swing twice as fast for quick small steps",
    )
    parser.add_argument(
        "--lower-tempo",
        type=float,
        default=None,
        help="separate time-scale for the lowering phases; defaults to "
        "--swing-tempo. The unfolding swing leg kicks the trunk backward, "
        "so lowering usually needs to be much slower than lifting",
    )
    parser.add_argument("--settle-pitch-ff", type=float, default=30.0)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument(
        "--leg-geometry",
        type=Path,
        default=Path(__file__).with_name("leg_geometry.json"),
        help="sagittal leg model written by leg_geometry_calibration.py; "
        "supplies the inverse kinematics used for the swing-foot path",
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=15.0,
        help="how far the swing ankle rises above its standing height (mm); "
        "the ankle path is vertical, so this is the ground clearance",
    )
    parser.add_argument("--reference", choices=("live", "saved"), default="live")
    parser.add_argument(
        "--steps", choices=("balance", "left", "right", "both"), default="both"
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(__file__).with_name("standing_zero_calibration.json"),
    )
    args = parser.parse_args()
    if args.cycles < 1:
        parser.error("--cycles must be at least 1")

    calibration = json.loads(args.calibration.read_text(encoding="utf-8"))
    rad_by_id = {
        int(calibration["motor_id"][name]): float(value)
        for name, value in calibration["raw_position_rad"].items()
    }
    stand = {
        sid: round(2048 + rad_by_id[sid] * 4096 / (2 * math.pi))
        for sid in range(1, 15)
    }

    if not args.leg_geometry.is_file():
        raise RuntimeError(
            f"no leg geometry at {args.leg_geometry}; run "
            "leg_geometry_calibration.py --mode measure first"
        )
    geometry = json.loads(args.leg_geometry.read_text(encoding="utf-8"))

    def reachable_clearance(model: dict, requested: float) -> float:
        """Largest clearance the leg can still reach, capped at requested."""
        x0, z0 = forward_kinematics(model, 0.0, 0.0)
        try:
            inverse_kinematics(model, x0, z0 + requested)
            return requested
        except ValueError:
            pass
        low, high = 0.0, requested
        for _ in range(40):
            mid = 0.5 * (low + high)
            try:
                inverse_kinematics(model, x0, z0 + mid)
                low = mid
            except ValueError:
                high = mid
        return low

    clearance_mm = args.clearance
    for side_name in ("left", "right"):
        allowed = reachable_clearance(geometry[side_name], args.clearance)
        if allowed < clearance_mm - 0.05:
            print(
                f"GEOMETRY warning: {side_name} can only reach "
                f"{allowed:.1f}mm of the requested {args.clearance:.1f}mm",
                flush=True,
            )
            clearance_mm = allowed
    print(
        f"GEOMETRY source={args.leg_geometry} clearance={clearance_mm:.1f}mm",
        flush=True,
    )

    def swing_targets(side_name: str, lift: float) -> dict[int, float]:
        """Hip/knee/ankle counts for the swing leg at a lift fraction."""
        model = geometry[side_name]
        x0, z0 = forward_kinematics(model, 0.0, 0.0)
        # smoothstep: zero slope at both ends, so take-off and touchdown are
        # gentle and the ankle never dips below its standing height.
        height = clearance_mm * lift * lift * (3.0 - 2.0 * lift)
        hip_off, knee_off = inverse_kinematics(model, x0, z0 + height)
        hip_count, knee_count = model_to_counts(model, hip_off, knee_off)
        return {
            model["ids"]["hip"]: float(hip_count),
            model["ids"]["knee"]: float(knee_count),
            model["ids"]["ankle"]: float(
                model["stand_counts"]["ankle"]
                + model["ankle_sign"] * (-knee_off)
            ),
        }

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise RuntimeError(f"cannot open {args.port} at {args.baud}")

    stop_requested = False
    last_targets = {sid: float(stand[sid]) for sid in CONTROL_IDS}

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

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

    def send(targets: dict[int, float]) -> None:
        nonlocal last_targets
        for sid in CONTROL_IDS:
            value = int(round(targets[sid]))
            comm, error = packet.write4ByteTxRx(
                port, sid, ADDR_GOAL_POSITION, value
            )
            check(f"ID{sid} goal", comm, error)
        last_targets = dict(targets)

    imu = ThreadedIMUReader(i2c_bus=1, frequency_hz=args.rate)

    def latest() -> tuple[list[float], tuple[float, float, float]]:
        snap = imu.get_latest()
        if not snap.valid or time.perf_counter() - snap.timestamp_s > 0.20:
            raise RuntimeError(f"IMU stale/invalid: {imu.get_status()}")
        return quat_apply_inverse(snap.quat, [0.0, 0.0, 1.0]), snap.gyro

    def average_gravity(duration: float) -> list[float]:
        samples: list[list[float]] = []
        end = time.perf_counter() + duration
        while time.perf_counter() < end:
            gravity, _gyro = latest()
            samples.append(gravity)
            time.sleep(1.0 / args.rate)
        return [sum(row[i] for row in samples) / len(samples) for i in range(3)]

    baseline: list[float] = []
    filtered_gravity: list[float] = []
    filtered_gyro_y = 0.0
    filtered_pitch_rate = 0.0
    last_pitch_deg: float | None = None
    recovery_reason = ""
    last_log = 0.0

    def run_phase(
        name: str,
        duration: float,
        support: int,
        roll_from: float,
        roll_to: float,
        lift_side: str | None = None,
        lift_from: float = 0.0,
        lift_to: float = 0.0,
    ) -> bool:
        """Run one phase.

        support: +1 means right foot supports/left foot lifts; -1 is opposite.
        roll fractions are relative to the configured feed-forward shift.
        """
        nonlocal filtered_gravity, filtered_gyro_y, filtered_pitch_rate
        nonlocal last_pitch_deg, recovery_reason, last_log
        frames = max(1, round(duration * args.rate))
        start = time.perf_counter()
        unstable_frames = 0
        for frame in range(1, frames + 1):
            if stop_requested:
                recovery_reason = "stop requested"
                return False
            u = ease(frame / frames)
            roll_fraction = roll_from + u * (roll_to - roll_from)
            requested_lift = lift_from + u * (lift_to - lift_from)

            gravity, gyro = latest()
            try:
                present3 = read4(3, ADDR_PRESENT_POSITION)
            except Exception:  # noqa: BLE001
                present3 = -1
            try:
                present7 = read4(7, ADDR_PRESENT_POSITION)
            except Exception:  # noqa: BLE001
                present7 = -1
            filtered_gravity = [
                0.75 * filtered_gravity[i] + 0.25 * gravity[i]
                for i in range(3)
            ]
            filtered_gyro_y = 0.70 * filtered_gyro_y + 0.30 * gyro[1]
            rotation_cross = cross(baseline, filtered_gravity)
            rotation_dot = sum(a * b for a, b in zip(baseline, filtered_gravity))
            pitch_deg = math.degrees(math.atan2(rotation_cross[1], rotation_dot))
            if last_pitch_deg is None:
                raw_pitch_rate = 0.0
            else:
                raw_pitch_rate = (pitch_deg - last_pitch_deg) * args.rate
            last_pitch_deg = pitch_deg
            filtered_pitch_rate = (
                0.75 * filtered_pitch_rate + 0.25 * raw_pitch_rate
            )
            total_deg = math.degrees(
                math.atan2(
                    math.sqrt(sum(value * value for value in rotation_cross)),
                    rotation_dot,
                )
            )
            gyro_norm = math.sqrt(sum(value * value for value in gyro))

            target_gy = baseline[1] - support * args.roll_gravity * roll_fraction
            roll_error = filtered_gravity[1] - target_gy
            feedback_counts = clamp(
                600.0 * roll_error,
                -args.roll_feedback_clamp,
                args.roll_feedback_clamp,
            )
            roll_offset = (
                support * args.roll_counts * roll_fraction + feedback_counts
                if support
                else 0.0
            )

            stability_scale = 1.0
            if abs(pitch_deg) > 4.0:
                stability_scale = min(
                    stability_scale, clamp((9.0 - abs(pitch_deg)) / 5.0, 0.0, 1.0)
                )
            if support and abs(roll_error) > 0.035:
                stability_scale = min(
                    stability_scale,
                    clamp((0.090 - abs(roll_error)) / 0.055, 0.0, 1.0),
                )
            # Keep the commanded foot trajectory continuous.  Scaling the
            # already-raised foot from instantaneous attitude made it drop
            # abruptly exactly when the body started to pitch.  The phase
            # gates and recovery path decide whether lifting is safe instead.
            lift = args.amplitude * requested_lift
            # The measured direction probe showed that ID3 +counts together
            # with ID8 -counts produces positive/forward pitch.  Apply the
            # opposite command to bring signed pitch back to the upright IMU
            # reference.
            # In double support the rigid standing pose is already stable.
            # Full delayed feedback there sustained a fore/aft rocking mode,
            # so use only a weak correction while settling and no active
            # correction during the final passive wait.
            if name == "WAIT_STABLE":
                pitch_feedback_scale = 0.0
            elif "TOUCHDOWN" in name:
                # At the pre-touchdown hold the delayed full-strength PD loop
                # sustained a +/-10 dps rocking mode.  Keep a light correction
                # and let the mechanism dissipate the remaining motion.
                pitch_feedback_scale = 0.25
            elif support == 0:
                pitch_feedback_scale = 0.25
            else:
                pitch_feedback_scale = 1.0
            pitch_offset = pitch_feedback_scale * (
                -args.pitch_kp * pitch_deg
                - args.pitch_kd * filtered_pitch_rate
            )
            phase_pitch_ff = 0.0
            if name.startswith("CENTER"):
                lift_ff_scale = clamp(args.amplitude / 0.15, 0.0, 1.5)
                phase_pitch_ff = (
                    -args.center_pitch_ff * math.sin(math.pi * frame / frames)
                    - args.settle_pitch_ff
                    * lift_ff_scale
                    * ease(frame / frames)
                )
            elif name.startswith("SETTLE"):
                lift_ff_scale = clamp(args.amplitude / 0.15, 0.0, 1.5)
                phase_pitch_ff = (
                    -args.settle_pitch_ff
                    * lift_ff_scale
                    * (1.0 - ease(frame / frames))
                )
            elif name.startswith("LOWER"):
                lift_ff_scale = clamp(args.amplitude / 0.15, 0.0, 1.5)
                phase_pitch_ff = (
                    -args.touchdown_pitch_ff
                    * lift_ff_scale
                    * (1.0 - requested_lift)
                )
            elif name.startswith("LAND"):
                lift_ff_scale = clamp(args.amplitude / 0.15, 0.0, 1.5)
                phase_pitch_ff = (
                    -args.touchdown_pitch_ff
                    * lift_ff_scale
                    * (1.0 - ease(frame / frames))
                )
            if support:
                # Pre-lean the support hip forward as the weight shifts,
                # before the swing fold pulls the CoG backward.
                phase_pitch_ff += args.pre_lean_ff * roll_fraction
            if lift_side is not None:
                # The folding swing leg drags the CoG backward; extra lean
                # in proportion to the current lift so the correction leads
                # the motion instead of chasing it.
                phase_pitch_ff += args.lift_pitch_ff * requested_lift
            pitch_offset += phase_pitch_ff
            pitch_offset = clamp(pitch_offset, -args.pitch_clamp, args.pitch_clamp)

            targets = {sid: float(stand[sid]) for sid in CONTROL_IDS}
            # In double support both hip rolls shift the trunk together.
            # In single support the swing hip must relax back toward
            # standing as the foot lifts: rolling the folded swing leg
            # sideways flings leg inertia laterally and drags the trunk
            # toward the swing side (large gyro_x bursts at lift end).
            # The offset decays with the lift fraction and returns during
            # lowering so both hips realign before touchdown.
            if support >= 0:
                support_roll_sid, swing_roll_sid = 4, 9
            else:
                support_roll_sid, swing_roll_sid = 9, 4
            targets[support_roll_sid] += roll_offset
            if lift_side is None:
                if support and name in ("STABILIZE_RIGHT", "STABILIZE_LEFT"):
                    # Pre-lift stabilize: un-splay the swing hip back toward
                    # standing while both feet are still planted, so the
                    # lift starts from the verified clearing configuration
                    # and the airborne leg makes no lateral motion.
                    targets[swing_roll_sid] += roll_offset * (1.0 - u)
                elif support and name == "WAIT_SUPPORT":
                    # After the pre-lift ramp the swing hip stays at standing.
                    pass
                else:
                    targets[swing_roll_sid] += roll_offset
            elif name.startswith("LIFT") or name.startswith("HOLD"):
                # Airborne: swing hip stays at standing for full clearance.
                pass
            else:
                # Lowering/touchdown: return toward the shift so both hips
                # realign before touchdown.
                targets[swing_roll_sid] += roll_offset * (1.0 - requested_lift)
            # Swing-foot path in task space: the ankle rises vertically by the
            # clearance, so no frame drops below the standing height and the
            # foot cannot scuff the ground.  The hip and knee come from the
            # inverse kinematics and the ankle counter-rotates to keep the
            # sole level.
            if lift_side is not None:
                for sid, value in swing_targets(lift_side, requested_lift).items():
                    targets[sid] = value
            # During single support, correct pitch with the loaded leg only.
            # Applying the same correction to the swing leg distorted its
            # touchdown target (especially ID8 during a left step), so the
            # foot could remain partly lifted while the body was falling.
            # Static probe: ID1+ pitches forward (same sign as ID3+), ID6+
            # pitches backward (mirror), so the left-support ankle uses the
            # negated offset.  The ankle has ~1.7x the pitch authority of
            # the hip per count.
            if support > 0:
                targets[3] += pitch_offset
                targets[1] += clamp(
                    args.ankle_gain * pitch_offset,
                    -args.ankle_clamp,
                    args.ankle_clamp,
                )
            elif support < 0:
                targets[8] -= pitch_offset
                targets[6] -= clamp(
                    args.ankle_gain * pitch_offset,
                    -args.ankle_clamp,
                    args.ankle_clamp,
                )
            else:
                targets[3] += pitch_offset
                targets[8] -= pitch_offset
            send(targets)

            unstable = total_deg > 16.0 or gyro_norm > args.gyro_gate
            unstable_frames = unstable_frames + 1 if unstable else 0
            if unstable_frames >= 3:
                recovery_reason = (
                    f"attitude recovery: total={total_deg:.1f}deg "
                    f"gyro={gyro_norm:.2f}rad/s"
                )
                return False

            now = time.perf_counter()
            if now - last_log >= 0.10:
                print(
                    f"{name} u={frame / frames:.2f} gy={filtered_gravity[1]:+.4f} "
                    f"target_gy={target_gy:+.4f} pitch={pitch_deg:+.2f}deg "
                    f"pitch_rate={filtered_pitch_rate:+.1f}dps "
                    f"gyro_y={filtered_gyro_y:+.3f} "
                    f"gyro_xz={gyro[0]:+.2f}/{gyro[2]:+.2f} "
                    f"id3={present3} id7={present7} "
                    f"roll_cmd={roll_offset:+.1f} pitch_cmd={pitch_offset:+.1f} "
                    f"phase_ff={phase_pitch_ff:+.1f} "
                    f"lift={lift:.3f}",
                    flush=True,
                )
                last_log = now

            remaining = start + frame / args.rate - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
        return True

    def return_to_stand(duration: float = 0.40) -> None:
        start_targets = dict(last_targets)
        frames = max(1, round(duration * args.rate))
        start = time.perf_counter()
        for frame in range(1, frames + 1):
            u = ease(frame / frames)
            send(
                {
                    sid: start_targets[sid] + u * (stand[sid] - start_targets[sid])
                    for sid in CONTROL_IDS
                }
            )
            remaining = start + frame / args.rate - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)

    try:
        torque = {sid: read1(sid, ADDR_TORQUE_ENABLE) for sid in range(1, 15)}
        expected = all(torque[sid] == 1 for sid in range(1, 15))
        if not expected:
            raise RuntimeError(f"unexpected torque state: {torque}")

        if args.goal_current > 0:
            # Current-based position control: position goals still apply but
            # the torque is clamped to +/- goal current, making the whole
            # gait compliant (soft ground contact, gentle reactions).
            for sid in range(1, 11):
                write1(sid, ADDR_TORQUE_ENABLE, 0)
                write1(sid, ADDR_OPERATING_MODE, MODE_CURRENT_BASED_POSITION)
                write2(sid, ADDR_GOAL_CURRENT, args.goal_current)
                write1(sid, ADDR_TORQUE_ENABLE, 1)
            print(
                f"CURRENT_MODE_ON leg_servos=1-10 goal_current={args.goal_current}mA",
                flush=True,
            )

        # Reassert the six controlled standing goals before using current pose as
        # the IMU reference for this bounded trial.
        send({sid: float(stand[sid]) for sid in CONTROL_IDS})
        time.sleep(0.3)
        imu.start()
        deadline = time.perf_counter() + 3.0
        while time.perf_counter() < deadline and not imu.get_latest().valid:
            time.sleep(0.02)
        if not imu.get_latest().valid:
            raise RuntimeError(f"IMU did not become valid: {imu.get_status()}")
        time.sleep(0.25)
        current_gravity = average_gravity(0.40)
        startup_tilt_deg = math.degrees(
            math.acos(clamp(-current_gravity[0], -1.0, 1.0))
        )
        # This IMU is mounted with a fixed ~19 degree pitch in the newly
        # calibrated standing pose.  The live/saved baseline below is the
        # actual upright reference; this check only rejects a grossly fallen
        # startup pose rather than imposing the sensor PCB's mechanical zero.
        if startup_tilt_deg > 35.0:
            raise RuntimeError(
                "startup body is not upright: "
                f"tilt={startup_tilt_deg:.1f}deg "
                f"gravity={[round(value, 6) for value in current_gravity]}"
            )
        saved_upright = Path(__file__).with_name("results_imu_tilt") / "upright_latest.json"
        if args.reference == "saved" and saved_upright.is_file():
            upright_samples = json.loads(saved_upright.read_text(encoding="utf-8"))["samples"]
            baseline = [
                sum(row["projected_gravity"][i] for row in upright_samples)
                / len(upright_samples)
                for i in range(3)
            ]
            baseline_source = str(saved_upright)
        else:
            baseline = current_gravity
            baseline_source = "live startup average"
        filtered_gravity = list(current_gravity)
        filtered_gyro_y = 0.0
        filtered_pitch_rate = 0.0
        last_pitch_deg = None
        print(
            f"BASELINE target={[round(v, 6) for v in baseline]} "
            f"current={[round(v, 6) for v in current_gravity]} "
            f"source={baseline_source} amplitude={args.amplitude:.2f}",
            flush=True,
        )

        swing = args.swing_tempo
        lower = args.lower_tempo if args.lower_tempo is not None else swing
        left_phases = [
            ("PRE_BALANCE", 0.80, 0, 0.0, 0.0, None, 0.0, 0.0),
            ("SHIFT_RIGHT", 0.45, +1, 0.0, 1.0, None, 0.0, 0.0),
            ("STABILIZE_RIGHT", 0.50, +1, 1.0, 1.0, None, 0.0, 0.0),
            ("LIFT_LEFT", args.lift_duration * swing, +1, 1.0, 1.0, "left", 0.0, 1.0),
            ("HOLD_LEFT", args.hold_duration * swing, +1, 1.0, 1.0, "left", 1.0, 1.0),
            ("LOWER_LEFT_APPROACH", 0.35 * lower, +1, 1.0, 1.0, "left", 1.0, 0.2),
            ("STABILIZE_LEFT_TOUCHDOWN", 0.25 * lower, +1, 1.0, 1.0, "left", 0.2, 0.2),
            ("LOWER_LEFT_TOUCH", 0.45 * lower, +1, 1.0, 1.0, "left", 0.2, 0.0),
            ("LAND_LEFT", 0.50, +1, 1.0, 1.0, None, 0.0, 0.0),
            ("CENTER_ONE", 0.80, +1, 1.0, 0.0, None, 0.0, 0.0),
            ("SETTLE_ONE", 0.80, 0, 0.0, 0.0, None, 0.0, 0.0),
        ]
        right_phases = [
            ("SHIFT_LEFT", 0.45, -1, 0.0, 1.0, None, 0.0, 0.0),
            ("STABILIZE_LEFT", 0.50, -1, 1.0, 1.0, None, 0.0, 0.0),
            ("LIFT_RIGHT", args.lift_duration * swing, -1, 1.0, 1.0, "right", 0.0, 1.0),
            ("HOLD_RIGHT", args.hold_duration * swing, -1, 1.0, 1.0, "right", 1.0, 1.0),
            ("LOWER_RIGHT_APPROACH", 0.35 * lower, -1, 1.0, 1.0, "right", 1.0, 0.2),
            ("STABILIZE_RIGHT_TOUCHDOWN", 0.25 * lower, -1, 1.0, 1.0, "right", 0.2, 0.2),
            ("LOWER_RIGHT_TOUCH", 0.45 * lower, -1, 1.0, 1.0, "right", 0.2, 0.0),
            ("LAND_RIGHT", 0.50, -1, 1.0, 1.0, None, 0.0, 0.0),
            ("CENTER_TWO", 0.80, -1, 1.0, 0.0, None, 0.0, 0.0),
            ("SETTLE_TWO", 0.80, 0, 0.0, 0.0, None, 0.0, 0.0),
        ]
        if args.steps == "balance":
            phases = [
                ("STATIC_BALANCE", 1.50, 0, 0.0, 0.0, None, 0.0, 0.0),
            ]
        elif args.steps == "right":
            phases = [left_phases[0], *right_phases]
        elif args.steps == "left":
            phases = left_phases
        else:
            phases = (left_phases + right_phases) * args.cycles
        completed = True
        for phase in phases:
            if not run_phase(*phase):
                completed = False
                break
            phase_name = phase[0]
            if phase_name.startswith("STABILIZE"):
                support = phase[2]
                touchdown_gate = "TOUCHDOWN" in phase_name
                support_pitch_limit = 3.0 if touchdown_gate else 2.5
                support_rate_limit = 8.0 if touchdown_gate else 6.0
                for wait_index in range(1, 5):
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= support_pitch_limit
                        and abs(filtered_pitch_rate) <= support_rate_limit
                    ):
                        print(
                            f"SUPPORT_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                        break
                    print(
                        f"SUPPORT_GATE wait={wait_index} pitch={last_pitch_deg} "
                        f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                        flush=True,
                    )
                    if not run_phase(
                        "WAIT_TOUCHDOWN" if touchdown_gate else "WAIT_SUPPORT",
                        0.25,
                        support,
                        1.0,
                        1.0,
                        phase[5],
                        phase[7],
                        phase[7],
                    ):
                        completed = False
                        break
                else:
                    # The final wait updates the filters after the last
                    # pre-wait check, so accept it when that last sample has
                    # entered the gate.
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= support_pitch_limit
                        and abs(filtered_pitch_rate) <= support_rate_limit
                    ):
                        print(
                            f"SUPPORT_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                    else:
                        recovery_reason = (
                            f"support pose not stable: pitch={last_pitch_deg}deg "
                            f"pitch_rate={filtered_pitch_rate:.1f}dps"
                        )
                        completed = False
                if not completed:
                    break
            if phase_name.startswith("LAND"):
                land_support = phase[2]
                for wait_index in range(1, 4):
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= 3.0
                        and abs(filtered_pitch_rate) <= 8.0
                    ):
                        print(
                            f"LAND_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                        break
                    print(
                        f"LAND_GATE wait={wait_index} pitch={last_pitch_deg} "
                        f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                        flush=True,
                    )
                    if not run_phase(
                        "WAIT_LAND",
                        0.30,
                        land_support,
                        1.0,
                        1.0,
                        None,
                        0.0,
                        0.0,
                    ):
                        completed = False
                        break
                else:
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= 3.0
                        and abs(filtered_pitch_rate) <= 8.0
                    ):
                        print(
                            f"LAND_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                    else:
                        recovery_reason = (
                            f"touchdown did not settle: pitch={last_pitch_deg}deg "
                            f"pitch_rate={filtered_pitch_rate:.1f}dps"
                        )
                        completed = False
                if not completed:
                    break
            if phase_name.startswith("SETTLE"):
                for wait_index in range(1, 6):
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= 1.5
                        and abs(filtered_pitch_rate) <= 4.0
                    ):
                        print(
                            f"PHASE_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                        break
                    print(
                        f"PHASE_GATE wait={wait_index} pitch={last_pitch_deg} "
                        f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                        flush=True,
                    )
                    if not run_phase(
                        "WAIT_STABLE", 0.40, 0, 0.0, 0.0, None, 0.0, 0.0
                    ):
                        completed = False
                        break
                else:
                    if (
                        last_pitch_deg is not None
                        and abs(last_pitch_deg) <= 1.5
                        and abs(filtered_pitch_rate) <= 4.0
                    ):
                        print(
                            f"PHASE_GATE stable pitch={last_pitch_deg:+.2f}deg "
                            f"pitch_rate={filtered_pitch_rate:+.1f}dps",
                            flush=True,
                        )
                    else:
                        recovery_reason = (
                            f"double support did not settle: pitch={last_pitch_deg}deg "
                            f"pitch_rate={filtered_pitch_rate:.1f}dps"
                        )
                        completed = False
                if not completed:
                    break
        print(
            "BALANCE_STEP_COMPLETE" if completed else f"BALANCE_STEP_RECOVERY {recovery_reason}",
            flush=True,
        )
    finally:
        try:
            return_to_stand()
            time.sleep(0.2)
            final = {sid: read4(sid, ADDR_PRESENT_POSITION) for sid in CONTROL_IDS}
            print(f"FINAL_STAND {final}", flush=True)
            if args.goal_current > 0:
                # Restore plain position control so the servos leave the
                # trial in their canonical mode.
                for sid in range(1, 11):
                    try:
                        write1(sid, ADDR_TORQUE_ENABLE, 0)
                        write1(sid, ADDR_OPERATING_MODE, MODE_POSITION)
                        write1(sid, ADDR_TORQUE_ENABLE, 1)
                    except Exception:  # noqa: BLE001
                        print(f"ID{sid} mode restore failed", flush=True)
                print("CURRENT_MODE_OFF position mode restored", flush=True)
        finally:
            try:
                imu.stop()
            except Exception:
                pass
            port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
