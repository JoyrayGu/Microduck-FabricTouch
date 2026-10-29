# SPDX-License-Identifier: GPL-3.0-or-later
"""@file action_library.py
@brief 经过审核的、可被触觉手势触发的动作集合（动作白名单）。

安全约束
--------
* 动作名只能来自这里。外部网络**不能**提交舵机 ID、目标 count、速度。
* 第一批只允许**不改变重心**的头部动作：
    - ``head_roll``：头部横滚（歪头），**只动 ID14**，用实测标定的三个绝对位置。
    - ``open_mouth``：张嘴再闭合，**只动 ID15**，同样用实测标定值。
    - ``head_wag``：头部偏航（左右转头），**只动 ID13**。已完成标定但当前
      没有手势映射，作为备用动作保留。
* 改变重心的动作（鞠躬、抬腿、蹲起）留到真机验证之后再加入。

动作定义
--------
一个动作由若干**姿态关键帧**依次组成。每个关键帧给出目标 count（缺省表示
保持当前值）与到达该帧的时长。执行器按关键帧逐段做余弦插值。

姿态 count 居中 2048，一圈 4096（XL330 位置模式）。
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Mapping, Optional

#: 头部相关舵机（纯头部，不影响重心）。
HEAD_IDS = (11, 12, 13, 14)

#: 头部横滚（歪头）实测标定（2026-10-21，当前机器鸭，扭矩关闭手动摆 + 单舵机主动验证）。
#:
#: ID 14 的绝对 tick：
#:   中位 1881 —— 视觉端正，且等于左右已验证端点的中点
#:   右端 2050 —— 实测正常、未到机械上限，视觉歪头幅度明显
#:   左端 1712 —— 实测正常
#: 对称幅度 ±169 tick ≈ ±14.9°。
#:
#: 重要：不要用 tilt_head_pose.json 里的 2048，那是旧数据推算值，实测已超出
#: 手动可达范围（右极限测得 2040/2050/2070 波动）。
HEAD_ROLL_ID = 14
HEAD_ROLL_CENTER = 1881
HEAD_ROLL_RIGHT = 2050
HEAD_ROLL_LEFT = 1712

#: 头部偏航（转头）实测标定（2026-10-21，与上面同批次验证）。
#:
#: ID 13 的绝对 tick：
#:   中位 1854 —— 零扭矩手动摆到视觉中央后实测
#:   右端 2023 —— servo_goto 实测到位 2020，误差 -3
#:   左端 1685 —— servo_goto 实测到位 1688，误差 +3
#: 对称幅度 ±169 tick ≈ ±14.9°，与 ID14 一致。
#:
#: 重要：中位**不是** 2048。实测偏差 -194 tick。旧的 head_wag 实现用
#: ``2048 - (1736 - 2048) = 2360`` 推右端，比实测对称值多 337 tick（29.6°），
#: 会撞线缆限位。那个镜像假设已被证伪，不要恢复。
HEAD_YAW_ID = 13
HEAD_YAW_CENTER = 1854
HEAD_YAW_RIGHT = 2023
HEAD_YAW_LEFT = 1685

#: 嘴部实测标定（2026-10-22，当前机器鸭）。
#:
#: 零扭矩手动掰到限位的读数：
#:   闭合限位 2188
#:   张开限位 1711
#:
#: 但**主动驱动到 2188 时实测只能到 2174（ERR=-14）** —— 弹簧阻力挡住了。
#: 若把目标写成硬限位 2188，每次闭合舵机都会顶到最大扭矩去够一个到不了的
#: 位置，长期发热而无视觉收益。因此闭合工作位取 2174。
#:
#: 张开工作位取 1760：距张开限位 1711 留 49 tick（约 11%）余量，实机验证
#: 稳定停住、无异响、视觉张开明显。
#:
#: 方向：tick 减小 = 张嘴，tick 增大 = 闭合。
MOUTH_ID = 15
MOUTH_CLOSED = 2174
MOUTH_OPEN = 1760

#: 已实机验证的姿态文件所在目录（与 deployment_validation 共用）。
_DEFAULT_POSE_DIR = (
    Path(__file__).resolve().parents[2] / "tools" / "deployment_validation"
)


@dataclass(frozen=True)
class Keyframe:
    """一个姿态关键帧。

    ``pose`` 是 ``{servo_id: count}``；未列出的舵机保持上一帧目标值。
    ``duration`` 是从**上一帧**插值到本帧的时长（秒）。
    """

    pose: Mapping[int, int]
    duration: float


@dataclass(frozen=True)
class Action:
    """一个可执行动作：名字 + 关键帧序列。"""

    name: str
    frames: List[Keyframe] = field(default_factory=list)
    #: 动作总时长上限（秒），超过则视为异常。
    timeout_s: float = 8.0
    #: 人类可读说明。
    description: str = ""

    @property
    def ids(self) -> List[int]:
        """动作会触及的舵机 ID 集合。"""
        ids: set[int] = set()
        for frame in self.frames:
            ids.update(int(k) for k in frame.pose.keys())
        return sorted(ids)


def load_pose(path: Path) -> Dict[int, int]:
    """读取一个 ``{servo_id: count}`` 姿态 JSON。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {int(k): int(v) for k, v in data.items()}


def build_default_library(
    pose_dir: Optional[Path] = None,
) -> Dict[str, Action]:
    """构建默认动作白名单。

    只使用**已实机验证**的标定常量。所有动作的帧序列都来自真机单点验证，
    不再依赖姿态 JSON 文件——那些是旧映射下的数据，容易把其他关节一起带进来。

    ``pose_dir`` 参数保留仅为向后兼容，当前实现不使用它。
    """
    del pose_dir  # 未使用：动作不再从姿态文件构建
    library: Dict[str, Action] = {}

    # ---- head_roll：向右歪 → 直接摆向左 → 回中（只动 ID14） ---- #
    # 帧序列定稿于 2026-10-22 的实机手感验证（play_action tilt_head_smooth）：
    # 总时长 1.38s，用户评价「观感足够好、时间合适」。
    #
    # 关键设计：右→左是**连续一段**（0.50s 走 338 tick），中间不回正。
    # 早期版本在中间插了一帧中位（1881），因为插值器用余弦缓动
    # （ActionDispatcher._ease 在每个关键帧边界速度归零），那一帧会让头
    # 在中途完全停住再重新加速，观感是「摆到一半卡一下」。
    # 去掉它之后只在两端减速，像钟摆。
    #
    # 只写 ID14：不把 sit_pose.json 里的 ID11/12/13 值带进来，那些是旧映射
    # 下的姿态，会让头同时转头/低头，与"纯横滚"的设计意图不符。
    #
    # 不再依赖 sit_pose.json 是否存在：帧序列是纯实测常量，与姿态文件无关。
    # 早期版本把构建条件写成 ``if sit is not None``，一旦姿态文件缺失，
    # 这个动作就会静默消失，而调用方只会看到「没有这个动作」。
    library["head_roll"] = Action(
        name="head_roll",
        frames=[
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_CENTER}, duration=0.00),
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_RIGHT}, duration=0.25),   # 向右歪
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_RIGHT}, duration=0.15),   # 保持
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_LEFT}, duration=0.50),    # 直接摆向左，途经中位不停
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_LEFT}, duration=0.15),    # 保持
            Keyframe(pose={HEAD_ROLL_ID: HEAD_ROLL_CENTER}, duration=0.25),  # 回中
        ],
        timeout_s=10.0,
        description="头部向右歪再直接摆向左并回正（只动 ID14，分步缓动执行）",
    )

    # ---- head_wag：头部左右摆动（备用，当前无手势映射） ---- #
    # 保留但**不映射到任何手势**：新设计里 tap 走 head_roll、滑动走 open_mouth，
    # 转头没有对应的触觉语义。留着是因为它已完成标定，将来若需要「被摸时转头」
    # 可以直接加一行映射激活，不必重做。
    #
    # 只用实测的对称端点，不再用 2048 镜像推算（见 HEAD_YAW_* 注释）。
    # 刻意**只写 ID13**：
    #   * ID14 与 ID13 机械耦合（摆一个会带动另一个 100+ tick）。若同时给 ID14
    #     一个固定目标并保持扭矩，两个舵机会互相顶住。让 ID14 被动跟随最安全。
    #   * 也不写 ID11/ID12/ID14 的 sit_pose 值 —— 那是旧映射下的姿态，会让头
    #     同时低头/歪头，与"纯转头"的设计意图不符。
    wag_left = {HEAD_YAW_ID: HEAD_YAW_LEFT}
    wag_right = {HEAD_YAW_ID: HEAD_YAW_RIGHT}
    library["head_wag"] = Action(
        name="head_wag",
        frames=[
            Keyframe(pose={HEAD_YAW_ID: HEAD_YAW_CENTER}, duration=0.0),
            Keyframe(pose=wag_left, duration=0.35),
            Keyframe(pose=wag_right, duration=0.7),
            Keyframe(pose=wag_left, duration=0.7),
            Keyframe(pose=wag_right, duration=0.7),
            Keyframe(pose={HEAD_YAW_ID: HEAD_YAW_CENTER}, duration=0.35),
        ],
        timeout_s=6.0,
        description="头部左右摆动 2 个来回并回正（只动 ID13，备用动作）",
    )

    # ---- open_mouth：张嘴再闭合（只动 ID15） ---- #
    # 帧序列定稿于 2026-10-22 的实机手感验证（play_action mouth_open_close）：
    # 总时长 0.65s，用户评价「节奏够自然、保持时间足够、张开幅度足够」。
    #
    # 保持只给 0.05s（约 2 帧）而不是 0：若为 0，张开与闭合的目标会在相邻帧
    # 连续下发，舵机还没走完就被命令反向，看起来像抽搐而不是一次可见的张嘴。
    library["open_mouth"] = Action(
        name="open_mouth",
        frames=[
            Keyframe(pose={MOUTH_ID: MOUTH_CLOSED}, duration=0.00),  # 起点闭合
            Keyframe(pose={MOUTH_ID: MOUTH_OPEN}, duration=0.30),    # 张开
            Keyframe(pose={MOUTH_ID: MOUTH_OPEN}, duration=0.05),    # 短暂保持
            Keyframe(pose={MOUTH_ID: MOUTH_CLOSED}, duration=0.30),  # 闭合
        ],
        timeout_s=3.0,
        description="张嘴再闭合（只动 ID15，约 0.65s）",
    )

    return library


#: 手势 -> 动作名 的默认映射。brain 持有最终控制权。
#:
#: 设计（2026-10-22 用户确认）：最终形态是把织物装到鸭头上，用户持续触摸时
#: 头部不停做动作会明显影响体验。因此「短促接触」给一个可见的歪头，
#: 「持续滑动」给一次张嘴——张嘴是短促的、有明确结束的动作，不会持续干扰。
#:
#:   static_touch  -> head_roll   （静态触摸：只动 ID14）
#:   dynamic_touch -> open_mouth  （单指、多指或部分滑移：只动 ID15）
DEFAULT_GESTURE_TO_ACTION: Dict[str, str] = {
    "static_touch": "head_roll",
    "dynamic_touch": "open_mouth",
}


class ActionLibrary:
    """动作白名单的查询接口。"""

    def __init__(
        self,
        actions: Optional[Mapping[str, Action]] = None,
        gesture_map: Optional[Mapping[str, str]] = None,
    ):
        self._actions: Dict[str, Action] = dict(actions) if actions is not None else build_default_library()
        self._gesture_map: Dict[str, str] = dict(gesture_map) if gesture_map is not None else dict(
            DEFAULT_GESTURE_TO_ACTION
        )

    @property
    def names(self) -> List[str]:
        return sorted(self._actions)

    def get(self, name: str) -> Optional[Action]:
        return self._actions.get(name)

    def has(self, name: str) -> bool:
        return name in self._actions

    def resolve(self, gesture: str) -> Optional[Action]:
        """把允许的手势映射到动作；未映射或不存在的动作返回 None。"""
        name = self._gesture_map.get(gesture)
        if name is None:
            return None
        return self._actions.get(name)
