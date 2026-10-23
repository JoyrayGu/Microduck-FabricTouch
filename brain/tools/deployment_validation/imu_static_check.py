#!/usr/bin/env python3
"""Decisive IMU trust check.

Runs three consecutive IMU sessions (each re-initialises the sensor the
same way the controllers do) while the robot stands untouched, and logs:
  - the fused quaternion gravity (what the controllers use),
  - the raw accelerometer gravity (direct, unfused),
  - the angle between them (fusion error),
  - the jump between sessions (reproduces the run-to-run frame shift).

Keep the robot completely untouched for the whole run (~35 s).
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "microduck" / "src"))
from imu_reader import ThreadedIMUReader, quat_apply_inverse  # noqa: E402


def norm(vec: list[float]) -> list[float]:
    mag = math.sqrt(sum(v * v for v in vec)) or 1.0
    return [v / mag for v in vec]


def angle_between(a: list[float], b: list[float]) -> float:
    dot = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b))))
    return math.degrees(math.acos(dot))


def main() -> int:
    segment_s = 10.0
    means_q: list[list[float]] = []
    means_a: list[list[float]] = []
    max_mismatch = 0.0
    print(
        "现在开始 IMU 检查（3 段，每段 10 秒，中间重启传感器）。"
        "请完全不要触碰机器人。",
        flush=True,
    )
    for seg in (1, 2, 3):
        imu = ThreadedIMUReader(i2c_bus=1, frequency_hz=20.0)
        imu.start()
        deadline = time.perf_counter() + 3.0
        while time.perf_counter() < deadline and not imu.get_latest().valid:
            time.sleep(0.02)
        if not imu.get_latest().valid:
            raise RuntimeError("IMU did not become valid")
        samples_q: list[list[float]] = []
        samples_a: list[list[float]] = []
        start = time.perf_counter()
        last_report = start
        while time.perf_counter() - start < segment_s:
            snap = imu.get_latest()
            gq = norm(quat_apply_inverse(snap.quat, [0.0, 0.0, 1.0]))
            ga = norm(list(snap.acc))
            samples_q.append(gq)
            samples_a.append(ga)
            mismatch = angle_between(gq, ga)
            max_mismatch = max(max_mismatch, mismatch)
            now = time.perf_counter()
            if now - last_report >= 2.0:
                last_report = now
                gyro_norm = math.sqrt(sum(v * v for v in snap.gyro))
                print(
                    f"SEG{seg} t={now - start:4.1f}s "
                    f"gq={[round(v, 3) for v in gq]} "
                    f"ga={[round(v, 3) for v in ga]} "
                    f"mismatch={mismatch:5.1f}deg gyro={gyro_norm:.3f}",
                    flush=True,
                )
            time.sleep(0.05)
        imu.stop()
        mean_q = [
            sum(row[i] for row in samples_q) / len(samples_q) for i in range(3)
        ]
        mean_a = [
            sum(row[i] for row in samples_a) / len(samples_a) for i in range(3)
        ]
        means_q.append(norm(mean_q))
        means_a.append(norm(mean_a))
        print(
            f"SEG{seg}_MEAN gq={[round(v, 3) for v in means_q[-1]]} "
            f"ga={[round(v, 3) for v in means_a[-1]]}",
            flush=True,
        )
        time.sleep(1.0)
    print(
        f"SEG_JUMP_q_1_2={angle_between(means_q[0], means_q[1]):.1f}deg "
        f"q_2_3={angle_between(means_q[1], means_q[2]):.1f}deg "
        f"a_1_2={angle_between(means_a[0], means_a[1]):.1f}deg "
        f"a_2_3={angle_between(means_a[1], means_a[2]):.1f}deg",
        flush=True,
    )
    print(f"MAX_QUAT_ACC_MISMATCH={max_mismatch:.1f}deg", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())