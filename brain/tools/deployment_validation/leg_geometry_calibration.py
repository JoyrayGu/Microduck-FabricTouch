#!/usr/bin/env python3
"""Calibrate the sagittal leg geometry needed by a 2R inverse-kinematics solve.

Why this exists
---------------
The swing-foot trajectories used so far were built by interpolating *joint
counts*, which gives no control over where the foot actually goes: the sole
ends up scraping the ground somewhere in the middle of the motion.  Planning
the foot path in task space and solving for the joints requires the leg
lengths and the counts<->angle mapping, which is what this script identifies.

What you measure (standing pose, one side, millimetres)
-------------------------------------------------------
  thigh         hip-pitch axis centre  -> knee axis centre
  shin          knee axis centre       -> ankle axis centre
  ankle-height  ankle axis centre      -> sole plane
  hip-height    hip-pitch axis centre  -> sole plane
  hip-to-ankle-x  fore-aft offset of the hip axis from the ankle axis
                  (hip ahead of the ankle is positive)

Five numbers are enough: with the hip and ankle positions known and the two
link lengths known, the knee position follows from the two-circle
intersection (the knee points forward), so the standing joint angles are
determined without any posture assumption.

Modes
-----
  measure  store the measurements (and derive the standing geometry)
  show     print the derived model and the standing joint angles
  ik       solve joint counts for a target ankle position
  fk       print the ankle position for given joint counts
  verify   drive a short sequence of poses and print the prediction so the
           numbers can be checked against a ruler

The output file is written next to the other calibration files and is meant to
be imported directly by the trajectory planner.
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
COUNTS_PER_TURN = 4096.0

# (knee, hip) per side; the ankle stays out of the 2R chain but is needed to
# keep the sole flat.
SIDE_IDS = {
    "right": {"knee": 2, "hip": 3, "ankle": 1},
    "left": {"knee": 7, "hip": 8, "ankle": 6},
}
# Counting direction of each joint relative to the model angles.  On this
# robot the left leg is mirrored, so its counts run opposite to the model.
SIDE_SIGNS = {"right": {"knee": 1.0, "hip": 1.0}, "left": {"knee": -1.0, "hip": -1.0}}


class Bus:
    def __init__(self, port_name: str, baud: int) -> None:
        self.port = PortHandler(port_name)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(baud):
            raise RuntimeError(f"cannot open {port_name} at {baud}")

    def check(self, label: str, comm: int, error: int) -> None:
        if comm != COMM_SUCCESS:
            raise RuntimeError(f"{label}: {self.packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {self.packet.getRxPacketError(error)}")

    def read1(self, sid: int, address: int) -> int:
        value, comm, error = self.packet.read1ByteTxRx(self.port, sid, address)
        self.check(f"ID{sid} read1", comm, error)
        return value

    def read4(self, sid: int, address: int) -> int:
        value, comm, error = self.packet.read4ByteTxRx(self.port, sid, address)
        self.check(f"ID{sid} read4", comm, error)
        return value

    def write4(self, sid: int, address: int, value: int) -> None:
        comm, error = self.packet.write4ByteTxRx(self.port, sid, address, value)
        self.check(f"ID{sid} write4", comm, error)

    def close(self) -> None:
        self.port.closePort()


def load_stand(calibration_path: Path) -> dict[int, int]:
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    rad_by_id = {
        int(calibration["motor_id"][name]): float(value)
        for name, value in calibration["raw_position_rad"].items()
    }
    return {
        sid: round(2048 + rad_by_id[sid] * COUNTS_PER_TURN / (2 * math.pi))
        for sid in ALL_IDS
    }


def two_circle_intersection(
    hip: tuple[float, float],
    ankle: tuple[float, float],
    l1: float,
    l2: float,
    forward: float = 1.0,
) -> tuple[float, float]:
    """Knee position from the hip and ankle positions and the two links.

    Both solutions are mirrored across the hip-ankle line; `forward` picks
    the one on the side the knee really bends towards.
    """
    hx, hz = hip
    ax, az = ankle
    dx, dz = ax - hx, az - hz
    d = math.hypot(dx, dz)
    if d <= 0.0 or d > l1 + l2:
        raise ValueError(f"leg cannot reach: hip-ankle distance {d:.1f}mm")
    a = (l1 * l1 - l2 * l2 + d * d) / (2.0 * d)
    height = math.sqrt(max(0.0, l1 * l1 - a * a))
    ux, uz = dx / d, dz / d
    px, pz = -uz, ux
    kx = hx + a * ux + forward * height * px
    kz = hz + a * uz + forward * height * pz
    return kx, kz


def link_angle(origin: tuple[float, float], tip: tuple[float, float]) -> float:
    """Angle of origin->tip measured from straight down, forward positive."""
    return math.atan2(tip[0] - origin[0], origin[1] - tip[1])


def derive_model(
    side: str,
    measurements: dict[str, float],
    stand: dict[int, int],
) -> dict[str, object]:
    thigh = measurements["thigh"]
    shin = measurements["shin"]
    ankle_height = measurements["ankle_height"]
    hip_height = measurements["hip_height"]
    hip_to_ankle_x = measurements["hip_to_ankle_x"]

    # Standing pose in the sagittal plane: sole plane at z = 0, the ankle
    # axis vertically above the sole reference point.
    ankle = (0.0, ankle_height)
    hip = (hip_to_ankle_x, hip_height)
    knee = two_circle_intersection(hip, ankle, thigh, shin, forward=1.0)

    hip_angle = link_angle(hip, knee)
    knee_angle = link_angle(knee, ankle)
    knee_flexion = knee_angle - hip_angle

    ids = SIDE_IDS[side]
    signs = SIDE_SIGNS[side]
    counts_per_radian = COUNTS_PER_TURN / (2.0 * math.pi)

    return {
        "side": side,
        "measurements_mm": dict(measurements),
        "links_mm": {"thigh": thigh, "shin": shin, "ankle_height": ankle_height},
        "standing": {
            "hip_axis": list(hip),
            "knee_axis": list(knee),
            "ankle_axis": list(ankle),
        },
        "standing_angles_rad": {
            "hip": hip_angle,
            "knee": knee_angle,
            "knee_flexion": knee_flexion,
        },
        "standing_angles_deg": {
            "hip": math.degrees(hip_angle),
            "knee": math.degrees(knee_angle),
            "knee_flexion": math.degrees(knee_flexion),
        },
        "ids": ids,
        "signs": signs,
        "ankle_sign": 1.0,
        "ankle_sign_note": (
            "verified 2026-09-30 by eye: +1 keeps the sole level when the "
            "shank tilts; a negative value tips the toe up by twice the tilt"
        ),
        "stand_counts": {name: stand[sid] for name, sid in ids.items()},
        "counts_per_radian": counts_per_radian,
        "note": (
            "sagittal 2R chain: hip pitch -> thigh -> knee -> shin -> ankle "
            "axis; angles measured from straight down, forward positive"
        ),
    }


def forward_kinematics(
    model: dict[str, object],
    hip_offset: float,
    knee_offset: float,
) -> tuple[float, float]:
    """Ankle position relative to the hip axis for a physical joint offset.

    `hip_offset` / `knee_offset` are MODEL count offsets: the same numbers
    mean the same physical motion on both legs.  Use model_to_counts() to get
    the servo counts to command.
    """
    hip_angle = model["standing_angles_rad"]["hip"] + hip_offset / model["counts_per_radian"]
    knee_angle = model["standing_angles_rad"]["knee"] + knee_offset / model["counts_per_radian"]
    thigh = model["links_mm"]["thigh"]
    shin = model["links_mm"]["shin"]
    hip_axis = model["standing"]["hip_axis"]
    kx = hip_axis[0] + thigh * math.sin(hip_angle)
    kz = hip_axis[1] - thigh * math.cos(hip_angle)
    ax = kx + shin * math.sin(knee_angle)
    az = kz - shin * math.cos(knee_angle)
    return ax - hip_axis[0], az - hip_axis[1]


def inverse_kinematics(
    model: dict[str, object],
    target_x: float,
    target_z: float,
) -> tuple[float, float]:
    """Model count offsets that put the ankle at (target_x, target_z) from the hip.

    Returns MODEL offsets, so the same numbers describe the same physical
    pose on both legs; convert with model_to_counts() before commanding.
    """
    thigh = model["links_mm"]["thigh"]
    shin = model["links_mm"]["shin"]
    knee = two_circle_intersection(
        (0.0, 0.0), (target_x, target_z), thigh, shin, forward=1.0
    )
    hip_angle = link_angle((0.0, 0.0), knee)
    knee_angle = link_angle(knee, (target_x, target_z))
    hip_offset = (
        hip_angle - model["standing_angles_rad"]["hip"]
    ) * model["counts_per_radian"]
    knee_offset = (
        knee_angle - model["standing_angles_rad"]["knee"]
    ) * model["counts_per_radian"]
    return hip_offset, knee_offset


def model_to_counts(
    model: dict[str, object], hip_offset: float, knee_offset: float
) -> tuple[int, int]:
    """Servo counts for a model offset (mirrors the sign on the left leg)."""
    counts = model["stand_counts"]
    return (
        round(counts["hip"] + model["signs"]["hip"] * hip_offset),
        round(counts["knee"] + model["signs"]["knee"] * knee_offset),
    )


def counts_to_model(
    model: dict[str, object], hip_count: float, knee_count: float
) -> tuple[float, float]:
    """Model offset for servo counts (inverse of model_to_counts)."""
    counts = model["stand_counts"]
    return (
        model["signs"]["hip"] * (hip_count - counts["hip"]),
        model["signs"]["knee"] * (knee_count - counts["knee"]),
    )


def load_model(path: Path, side: str) -> dict[str, object]:
    if not path.is_file():
        raise RuntimeError(f"no geometry file {path}; run --mode measure first")
    data = json.loads(path.read_text(encoding="utf-8"))
    if side not in data:
        raise RuntimeError(f"{path} has no '{side}' entry")
    return data, data[side]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("measure", "show", "ik", "fk", "verify", "lift"), required=True
    )
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument(
        "--geometry",
        type=Path,
        default=Path(__file__).with_name("leg_geometry.json"),
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=Path(__file__).with_name("standing_zero_calibration.json"),
    )
    parser.add_argument("--thigh", type=float, help="hip axis -> knee axis (mm)")
    parser.add_argument("--shin", type=float, help="knee axis -> ankle axis (mm)")
    parser.add_argument(
        "--ankle-height", type=float, help="ankle axis -> sole plane (mm)"
    )
    parser.add_argument(
        "--hip-height", type=float, help="hip axis -> sole plane (mm)"
    )
    parser.add_argument(
        "--hip-to-ankle-x",
        type=float,
        help="fore-aft hip offset from the ankle axis (mm, hip forward = +)",
    )
    parser.add_argument("--target-x", type=float, help="ik: ankle x from hip (mm)")
    parser.add_argument("--target-z", type=float, help="ik: ankle z from hip (mm)")
    parser.add_argument("--hip-delta", type=float, default=0.0)
    parser.add_argument("--knee-delta", type=float, default=0.0)
    parser.add_argument(
        "--verify-span",
        type=float,
        default=120.0,
        help="verify: magnitude of the joint offsets to command; the "
        "predicted ankle moves further with a larger span, which makes the "
        "sign check visible with a ruler",
    )
    parser.add_argument(
        "--verify-hold",
        type=float,
        default=1.5,
        help="verify: seconds to hold each pose so it can be measured",
    )
    parser.add_argument(
        "--clearance",
        type=float,
        default=15.0,
        help="lift: how far the ankle rises above its standing height (mm)",
    )
    parser.add_argument("--lift-time", type=float, default=2.0)
    parser.add_argument("--hold-time", type=float, default=2.0)
    parser.add_argument("--lower-time", type=float, default=2.0)
    parser.add_argument(
        "--roll-counts",
        type=float,
        default=0.0,
        help="lift: optional hip-roll shift (ID4/ID9) before lifting; "
        "0 keeps both hips at standing",
    )
    parser.add_argument(
        "--ankle-flat",
        action="store_true",
        help="lift: counter-rotate the ankle so the sole stays parallel to "
        "the ground while the shank tilts; without it the ankle stays at "
        "standing and the foot tilts with the shank",
    )
    parser.add_argument(
        "--ankle-sign",
        type=float,
        default=1.0,
        help="lift: counting direction of the ankle joint (the ankle sign "
        "has not been calibrated yet, so try +1 and -1)",
    )
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--rate", type=float, default=40.0)
    args = parser.parse_args()

    stand = load_stand(args.calibration)

    if args.mode == "measure":
        values = {
            "thigh": args.thigh,
            "shin": args.shin,
            "ankle_height": args.ankle_height,
            "hip_height": args.hip_height,
            "hip_to_ankle_x": args.hip_to_ankle_x,
        }
        missing = [name for name, value in values.items() if value is None]
        if missing:
            raise SystemExit(f"missing measurements: {missing}")
        model = derive_model(args.side, values, stand)
        data = {}
        if args.geometry.is_file():
            data = json.loads(args.geometry.read_text(encoding="utf-8"))
        data[args.side] = model
        args.geometry.write_text(
            json.dumps(data, indent=2) + "\n", encoding="utf-8"
        )
        print(f"GEOMETRY_SAVED path={args.geometry} side={args.side}", flush=True)
        _print_model(model)
        return 0

    _, model = load_model(args.geometry, args.side)

    if args.mode == "show":
        _print_model(model)
        return 0

    if args.mode == "fk":
        x, z = forward_kinematics(model, args.hip_delta, args.knee_delta)
        hip_count, knee_count = model_to_counts(model, args.hip_delta, args.knee_delta)
        print(
            f"ANKLE_FROM_HIP x={x:+.1f}mm z={z:+.1f}mm "
            f"model_offset=({args.hip_delta:+.0f},{args.knee_delta:+.0f}) "
            f"servo_counts=(hip {hip_count}, knee {knee_count})",
            flush=True,
        )
        return 0

    if args.mode == "ik":
        if args.target_x is None or args.target_z is None:
            raise SystemExit("ik needs --target-x and --target-z")
        hip_delta, knee_delta = inverse_kinematics(model, args.target_x, args.target_z)
        x, z = forward_kinematics(model, hip_delta, knee_delta)
        hip_count, knee_count = model_to_counts(model, hip_delta, knee_delta)
        print(
            f"IK target=({args.target_x:+.1f},{args.target_z:+.1f}) "
            f"model_offset=({hip_delta:+.1f},{knee_delta:+.1f}) "
            f"servo_counts=(hip {hip_count}, knee {knee_count}) "
            f"residual=({x - args.target_x:+.2f},{z - args.target_z:+.2f})mm",
            flush=True,
        )
        return 0

    # lift: task-space demo.  The ankle rises vertically by --clearance and
    # comes straight back down, so no intermediate frame is lower than the
    # standing pose and the foot cannot scuff the ground by construction.
    if args.mode == "lift":
        ids = model["ids"]
        x0, z0 = forward_kinematics(model, 0.0, 0.0)
        bus = Bus(args.port, args.baud)
        try:
            torque = {sid: bus.read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
            if not all(value == 1 for value in torque.values()):
                raise RuntimeError(f"not all servos enabled: {torque}")
            base = {sid: stand[sid] for sid in ALL_IDS}
            for sid in ALL_IDS:
                bus.write4(sid, ADDR_GOAL_POSITION, base[sid])
            time.sleep(0.4)

            roll_ids = (4, 9)
            roll_sign = 1.0 if args.side == "left" else -1.0

            def hold_frames(label, duration, writer):
                frames = max(1, round(duration * args.rate))
                t0 = time.perf_counter()
                print(f"{label}_START", flush=True)
                for frame in range(1, frames + 1):
                    u = 0.5 - 0.5 * math.cos(math.pi * frame / frames)
                    writer(u)
                    remaining = t0 + frame / args.rate - time.perf_counter()
                    if remaining > 0:
                        time.sleep(remaining)
                print(f"{label}_DONE", flush=True)

            def ramp_roll(label, offset_from, offset_to):
                hold_frames(
                    label,
                    1.0,
                    lambda u: [
                        bus.write4(
                            sid,
                            ADDR_GOAL_POSITION,
                            int(round(base[sid] + roll_sign * (
                                offset_from + u * (offset_to - offset_from)
                            ))),
                        )
                        for sid in roll_ids
                    ],
                )

            def apply_z(z: float) -> None:
                hip_off, knee_off = inverse_kinematics(model, x0, z)
                hip_count, knee_count = model_to_counts(model, hip_off, knee_off)
                bus.write4(ids["hip"], ADDR_GOAL_POSITION, hip_count)
                bus.write4(ids["knee"], ADDR_GOAL_POSITION, knee_count)
                if args.ankle_flat:
                    # Cancel the shank tilt so the sole stays parallel to the
                    # ground.  The ankle's counting direction has not been
                    # calibrated, hence --ankle-sign.
                    ankle_count = round(
                        base[ids["ankle"]] + args.ankle_sign * (-knee_off)
                    )
                    bus.write4(ids["ankle"], ADDR_GOAL_POSITION, ankle_count)

            def move_ankle(label, duration, z_from, z_to):
                hold_frames(
                    label, duration, lambda u: apply_z(z_from + u * (z_to - z_from))
                )
                hip_off, knee_off = inverse_kinematics(model, x0, z_to)
                print(
                    f"{label}_END ankle_height={z_to - z0:+.1f}mm "
                    f"model_offset=({hip_off:+.0f},{knee_off:+.0f}) "
                    f"shank_tilt="
                    f"{math.degrees(knee_off / model['counts_per_radian']):+.1f}deg",
                    flush=True,
                )

            print(
                f"现在开始：{args.side}脚垂直抬起 {args.clearance:.0f}mm 再落回，"
                "踝关节保持站姿。请扶稳机器人。",
                flush=True,
            )
            top = z0 + args.clearance
            if args.roll_counts:
                ramp_roll("SHIFT", 0.0, args.roll_counts)
            move_ankle("LIFT", args.lift_time, z0, top)
            hip_off, knee_off = inverse_kinematics(model, x0, top)
            ankle_goal = (
                round(base[ids["ankle"]] + args.ankle_sign * (-knee_off))
                if args.ankle_flat
                else base[ids["ankle"]]
            )
            print(
                f"HOLD_START ankle_height={args.clearance:+.1f}mm "
                f"shank_tilt="
                f"{math.degrees(knee_off / model['counts_per_radian']):+.1f}deg "
                f"ankle_goal={ankle_goal} stand_ankle={base[ids['ankle']]} "
                f"flat={args.ankle_flat} sign={args.ankle_sign:+.0f}",
                flush=True,
            )
            end = time.perf_counter() + args.hold_time
            while time.perf_counter() < end:
                apply_z(top)
                time.sleep(1.0 / args.rate)
            print(
                f"HOLD_ANKLE present="
                f"{bus.read4(ids['ankle'], ADDR_PRESENT_POSITION)} "
                f"goal={bus.read4(ids['ankle'], ADDR_GOAL_POSITION)}",
                flush=True,
            )
            print(
                f"HOLD_END ankle_height={args.clearance:+.1f}mm", flush=True
            )
            move_ankle("LOWER", args.lower_time, top, z0)
            if args.roll_counts:
                ramp_roll("RECENTER", args.roll_counts, 0.0)
            print("LIFT_DEMO_COMPLETE", flush=True)
        finally:
            start = {sid: bus.read4(sid, ADDR_PRESENT_POSITION) for sid in ALL_IDS}
            hold_frames(
                "RETURN",
                1.0,
                lambda u: [
                    bus.write4(
                        sid,
                        ADDR_GOAL_POSITION,
                        int(round(start[sid] + u * (stand[sid] - start[sid]))),
                    )
                    for sid in ALL_IDS
                ],
            )
            bus.close()
        return 0

    # verify: command a few offsets and print the predicted ankle so the
    # numbers can be checked with a ruler.
    ids = model["ids"]
    bus = Bus(args.port, args.baud)
    try:
        torque = {sid: bus.read1(sid, ADDR_TORQUE_ENABLE) for sid in ALL_IDS}
        if not all(value == 1 for value in torque.values()):
            raise RuntimeError(f"not all servos enabled: {torque}")
        base = {sid: stand[sid] for sid in ALL_IDS}
        for sid in ALL_IDS:
            bus.write4(sid, ADDR_GOAL_POSITION, base[sid])
        time.sleep(0.4)
        span = args.verify_span
        steps = ((0.0, 0.0), (0.0, span), (span / 2.0, span), (0.0, -span))
        print(
            f"verify: each pose is held for {args.verify_hold:.1f}s — measure the "
            "ankle axis relative to the hip axis with a ruler",
            flush=True,
        )
        for hip_delta, knee_delta in steps:
            goal = dict(base)
            goal[ids["hip"]] = base[ids["hip"]] + round(hip_delta * model["signs"]["hip"])
            goal[ids["knee"]] = base[ids["knee"]] + round(
                knee_delta * model["signs"]["knee"]
            )
            frames = max(1, round(1.0 * args.rate))
            t0 = time.perf_counter()
            for frame in range(1, frames + 1):
                u = 0.5 - 0.5 * math.cos(math.pi * frame / frames)
                for sid in ALL_IDS:
                    bus.write4(
                        sid,
                        ADDR_GOAL_POSITION,
                        int(round(base[sid] + u * (goal[sid] - base[sid]))),
                    )
                remaining = t0 + frame / args.rate - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
            time.sleep(args.verify_hold)
            x, z = forward_kinematics(model, hip_delta, knee_delta)
            print(
                f"VERIFY_POSE hip_delta={hip_delta:+.0f} knee_delta={knee_delta:+.0f} "
                f"predicted_ankle_from_hip=({x:+.1f},{z:+.1f})mm",
                flush=True,
            )
        frames = max(1, round(1.0 * args.rate))
        t0 = time.perf_counter()
        for frame in range(1, frames + 1):
            u = 0.5 - 0.5 * math.cos(math.pi * frame / frames)
            for sid in ALL_IDS:
                bus.write4(sid, ADDR_GOAL_POSITION, base[sid])
            remaining = t0 + frame / args.rate - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
        print("VERIFY_DONE", flush=True)
    finally:
        bus.close()
    return 0


def _print_model(model: dict[str, object]) -> None:
    deg = model["standing_angles_deg"]
    links = model["links_mm"]
    print(
        f"MODEL side={model['side']} thigh={links['thigh']:.1f}mm "
        f"shin={links['shin']:.1f}mm ankle_height={links['ankle_height']:.1f}mm",
        flush=True,
    )
    print(
        f"MODEL standing_angles hip={deg['hip']:+.1f}deg "
        f"knee={deg['knee']:+.1f}deg knee_flexion={deg['knee_flexion']:+.1f}deg",
        flush=True,
    )
    print(
        f"MODEL stand_counts={model['stand_counts']} signs={model['signs']} "
        f"counts_per_radian={model['counts_per_radian']:.1f}",
        flush=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())