"""Current hardware mapping and model-space standing reference."""
from dataclasses import dataclass
from math import pi

TICKS_PER_TURN = 4096

@dataclass(frozen=True)
class JointConfig:
    name: str
    servo_id: int
    model_home_rad: float
    participates_in_stand: bool = True
    direction: int | None = None
    zero_tick: int | None = None
    offset_rad: float = 0.0
    min_tick: int = 0
    max_tick: int = 4095

# Hardware mapping supplied by the project owner: 1-5 left, 6-10 right.
MODEL_HOME_RAD = {
    "left_hip_yaw": 0.0,
    "left_hip_roll": -0.0873,
    "left_hip_pitch": -0.4579,
    "left_knee": -0.0049,
    "left_ankle": 0.4530,
    "right_hip_yaw": 0.0,
    "right_hip_roll": 0.0873,
    "right_hip_pitch": 0.4579,
    "right_knee": 0.0049,
    "right_ankle": -0.4530,
    "head_pitch": 0.3491,
    "neck_pitch": 0.3491,
    "head_yaw": 0.0,
    "head_roll": 0.0,
    "mouth": 0.0,
}

HARDWARE_NAMES = {
    1: "left_ankle", 2: "left_knee", 3: "left_hip_pitch", 4: "left_hip_roll", 5: "left_hip_yaw",
    6: "right_ankle", 7: "right_knee", 8: "right_hip_pitch", 9: "right_hip_roll", 10: "right_hip_yaw",
    11: "head_pitch", 12: "neck_pitch", 13: "head_yaw", 14: "head_roll", 15: "mouth",
}

JOINTS = tuple(
    JointConfig(name, servo_id, MODEL_HOME_RAD[name], participates_in_stand=servo_id != 15)
    for servo_id, name in HARDWARE_NAMES.items()
)
JOINT_BY_ID = {joint.servo_id: joint for joint in JOINTS}

def radians_to_degrees(value: float) -> float:
    return value * 180.0 / pi
