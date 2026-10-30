"""@file robot_gestures.py
@brief 面向机器人的时序触觉语义识别器（static_touch / dynamic_touch）。

与 ``gestures.py``（鼠标手势）**互不复用状态**，因为两者的语义与阈值不同：

* 鼠标手势关心「点击 / 拖动 / 滚动」；
* 机器人手势关心「接触点如何移动」，用于驱动鸭子做可爱动作。

两种触觉语义
------------
静态触摸（static_touch）：接触期间没有形成足够长的有效滑动轨迹。
动态触摸（dynamic_touch）：接触期间形成了单向、来回或部分滑移轨迹。


重要语义澄清
------------
这里的「方向」是**压力区域质心在皮肤表面的运动方向**，不是法向/切向/
剪切力方向。当前硬件只能给出标量压力场，因此不可能据此测量某个方向的力。
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Deque, Dict, List, Optional, Tuple

from collections import deque

from .config import RobotGestureConfig
from .features import TactileState


class RobotGesture(str, Enum):
    STATIC_TOUCH = "static_touch"
    DYNAMIC_TOUCH = "dynamic_touch"


class ContactMode(str, Enum):
    SINGLE = "single"
    MULTI = "multi"


class MotionMode(str, Enum):
    STATIC = "static"
    FULL_SLIP = "full_slip"
    PARTIAL_SLIP = "partial_slip"
    STATIONARY = "stationary"
    UNKNOWN = "unknown"


@dataclass
class RobotGestureEvent:
    """一条与动作无关的手势事件。"""

    gesture: RobotGesture
    timestamp: float
    confidence: float = 1.0
    direction: Optional[str] = None
    contact_mode: ContactMode = ContactMode.SINGLE
    motion_mode: MotionMode = MotionMode.UNKNOWN
    moving_contacts: int = 0
    stationary_contacts: int = 0
    start: Optional[Tuple[float, float]] = None
    end: Optional[Tuple[float, float]] = None
    data: Dict[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        extra = " ".join(f"{k}={v}" for k, v in self.data.items())
        return f"{self.gesture.value}{'(' + extra + ')' if extra else ''}"


def _euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dominant_direction(
    vx: float,
    vy: float,
    dominance: float = 0.4,
) -> Optional[str]:
    """把位移向量归到 上下左右 四个方向之一；方向不明确时返回 None。

    阵列坐标：x 向右增大，y 向下增大。

    ``dominance`` 是**主导轴占比**门槛：``max(|vx|,|vy|) / (|vx|+|vy|)``。
    低于该值说明是对角移动，方向不明确，返回 None。

    此前这个函数无视配置里的 ``direction_dominance``（直接比较 ``ax >= ay``），
    导致该配置项形同虚设：对角滑动会被强行归到某一轴。
    """
    ax, ay = abs(vx), abs(vy)
    total = ax + ay
    if total < 1e-6:
        return None
    if max(ax, ay) / total < dominance:
        return None
    if ax >= ay:
        return "left_to_right" if vx > 0 else "right_to_left"
    return "top_to_bottom" if vy > 0 else "bottom_to_top"


class RobotGestureRecognizer:
    """有状态的机器人手势识别器（每个会话一个实例）。"""

    def __init__(self, cfg: Optional[RobotGestureConfig] = None):
        self.cfg = cfg or RobotGestureConfig()
        self.reset()

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self._active = False
        self._start_time = 0.0
        self._last_time = 0.0
        self._start_xy: Optional[Tuple[float, float]] = None
        self._last_xy: Optional[Tuple[float, float]] = None
        #: 本次接触的质心轨迹（按时间顺序）。
        self._path: List[Tuple[float, float]] = []
        self._path_len = 0.0
        self._max_move = 0.0
        #: 累计停顿时间（毫秒），用于打断连续性。
        self._pause_ms = 0.0
        #: 连续段的起点索引（停顿会切分连续段）。
        self._seg_start_idx = 0
        #: 累计方向反转次数。
        self._reversals = 0
        #: 上一次的有效运动方向向量（用于检测反转）。
        self._last_dir: Optional[Tuple[float, float]] = None
        #: 单独追踪的累计位移方向。
        self._seg_disp = [0.0, 0.0]
        #: 当前方向段的累计位移，用于判断一次反转是否「有意义的长度」。
        self._run_disp = [0.0, 0.0]
        #: 相对当前主方向的**反向**累计位移，迟滞判据用。
        self._against_disp = [0.0, 0.0]
        #: 自上次确认反转以来的反向行程总长，用于比例判据。
        self._backward_total = 0.0
        #: 释放消抖：第一次观察到不活跃的时刻；None 表示当前没有挂起的释放。
        self._pending_release_at: Optional[float] = None
        #: 挂起期间观察到的最低峰值，用于判断压力是否真正归零。
        self._gap_min_peak = float("inf")
        self._contact_mode = ContactMode.SINGLE
        self._motion_mode = MotionMode.UNKNOWN
        self._moving_contacts = 0
        self._stationary_contacts = 0
        self._previous_contacts = []
        self._active_motion_frames = 0
        self._multi_motion_frames = 0
        self._contact_mode = ContactMode.SINGLE
        self._motion_mode = MotionMode.UNKNOWN
        self._moving_contacts = 0
        self._stationary_contacts = 0

    # ------------------------------------------------------------------ #
    @property
    def in_contact(self) -> bool:
        return self._active

    @property
    def contact_mode(self) -> ContactMode:
        return self._contact_mode

    @property
    def motion_mode(self) -> MotionMode:
        return self._motion_mode

    @property
    def moving_contacts(self) -> int:
        return self._moving_contacts

    @property
    def stationary_contacts(self) -> int:
        return self._stationary_contacts

    # ------------------------------------------------------------------ #
    def update(self, state: TactileState) -> List[RobotGestureEvent]:
        """喂入一帧处理结果，返回本帧产生的手势事件（通常 0 或 1 个）。

        释放消抖
        --------
        真实按压因压力短暂回落会被切成多个「活跃/不活跃」片段（实测一次
        轻点可被切成 4~5 段）。若任何一帧不活跃就立即判定手势，一次动作会
        被上报成多次：实测做 8 个手势，报出 37 个事件。

        因此不活跃时**先挂起**，在 ``release_debounce_ms`` 内若压力恢复，
        就当作同一次接触继续累积；只有连续不活跃超过该时长才算真正抬手。
        """
        t = state.timestamp if state.timestamp is not None else time.perf_counter()
        events: List[RobotGestureEvent] = []

        # 是否「仍在接触」只看 active，不能要求 centroid 非空。
        #
        # centroid 由 _find_contacts() 给出，而它依赖 3x3 局部峰值。快速滑动
        # 时压力摊开、局部峰值消失，会出现 active=True 但 centroid=None 的帧
        # （实测 trace 里 total=31.2 却没有质心）。若把这种帧当成"不活跃"，
        # 快速来回滑动就会被切成多个孤立单程 —— 实测 3 次来回只报出 2 个事件，
        # 中间那次被切成 3 个 single_stroke。
        #
        # 质心缺失时保持上一次位置、继续同一次接触；只有 active 变假才进消抖。
        if (
            self._active
            and state.active
            and state.centroid is None
            and not state.contacts
            and state.peak <= self.cfg.release_contactless_peak
        ):
            # Residual pressure without a contact cluster is release evidence.
            state = TactileState(
                raw=state.raw,
                grid=state.grid,
                total=0.0,
                peak=state.peak,
                active=False,
                frame_no=state.frame_no,
                timestamp=state.timestamp,
            )

        if state.active:
            # 压力恢复：撤销挂起的释放，继续同一次接触。
            #
            # 曾尝试用「质心跳变距离」判断是否为新接触，实测证明不可行：
            #   run4-tap（4 次独立轻点）   跳跃 3.41~5.19 格
            #   run9-backforth（1 次连续来回）跳跃 0.45~10.61 格
            #   run5-stroke（1 次连续单程）  跳跃 1.02~8.45 格
            # 连续滑动内部的帧间位移反而**大于**独立轻点之间的间距（手指
            # 快速移动时每帧就走好几格），所以这个特征无法区分两者。
            # 位置判据已移除；分离多次点击留给活跃段的时长判据处理。
            self._pending_release_at = None
            if state.centroid is not None:
                self._update_contact_modes(state)
                events.extend(self._update_active(state, state.centroid, t))
            else:
                # 无质心：只推进时间戳，不改变轨迹（避免跳变污染 path_len）。
                self._last_time = t
            return events

        if not self._active:
            self._pending_release_at = None
            return events

        # 处于接触中但本帧不活跃 —— 挂起，等待消抖窗口。
        if self._pending_release_at is None:
            self._pending_release_at = t
            self._gap_min_peak = state.peak
            return events

        self._gap_min_peak = min(self._gap_min_peak, state.peak)

        # 判定抬手需要**两个条件同时成立**：
        #
        # 1) 时长超过 release_debounce_ms —— 过滤极短的片段间断；
        # 2) 空档期间压力必须真正归零 —— 手指离开织物时压力必然降到 0。
        #
        # 只用时间判据无法区分「一次接触的内部碎片」与「两次独立点击」：
        # 实测碎片空档最长 275ms，而快速连点之间的空档可以短到 425ms，
        # 两者在时间上重叠，任何单一阈值都会顾此失彼（实测扫描 200~400ms
        # 没有任何取值能同时让 run4-tap 与 run9-backforth 都正确）。
        #
        # 但「空档内最低峰值」把两者分得很干净：
        #   真实抬手 -> 最低峰 0.0~0.5
        #   内部碎片 -> 最低峰 1.7~157.8
        # 因为碎片只是压力回落，织物仍有接触；抬手则是彻底离开。
        zeroed = self._gap_min_peak <= self.cfg.release_zero_peak
        elapsed_ms = (t - self._pending_release_at) * 1000.0
        if not zeroed and elapsed_ms < self.cfg.release_debounce_hard_ms:
            return events
        if zeroed and elapsed_ms < self.cfg.release_debounce_ms:
            return events

        # 确认抬手。判定时刻用最后一次有效接触的时间。
        events.extend(self._update_release(t))
        return events

    # ------------------------------------------------------------------ #
    def _update_contact_modes(self, state: TactileState) -> None:
        """Track coarse multi-contact motion relative to the prior frame.

        Contact centroids are matched to the previous frame with a global
        minimum-distance assignment, preserving identity when pressure sorting
        changes between frames.
        """
        current = state.contacts
        if not current:
            return
        previous = self._previous_contacts
        threshold = self.cfg.stroke_pause_tol
        moving = 0
        stationary = 0
        # Match the small contact sets globally instead of greedily. Greedy
        # matching can swap two nearby contacts when their pressure weights change.
        pairs = []
        if previous:
            match_count = min(len(current), len(previous))
            for current_indices in itertools.combinations(range(len(current)), match_count):
                for previous_indices in itertools.permutations(range(len(previous)), match_count):
                    cost = sum(
                        math.hypot(
                            current[i].cx - previous[j].cx,
                            current[i].cy - previous[j].cy,
                        )
                        for i, j in zip(current_indices, previous_indices)
                    )
                    pairs.append((cost, current_indices, previous_indices))
        if pairs:
            _cost, current_indices, previous_indices = min(pairs, key=lambda item: item[0])
            for i, j in zip(current_indices, previous_indices):
                distance = math.hypot(
                    current[i].cx - previous[j].cx,
                    current[i].cy - previous[j].cy,
                )
                if distance > threshold:
                    moving += 1
                else:
                    stationary += 1
        self._previous_contacts = list(current)
        self._contact_mode = ContactMode.MULTI if len(current) > 1 else ContactMode.SINGLE
        if self._contact_mode is ContactMode.MULTI and moving:
            self._multi_motion_frames += 1
        else:
            self._multi_motion_frames = 0
        self._moving_contacts = moving
        self._stationary_contacts = stationary
        if not previous:
            self._motion_mode = MotionMode.UNKNOWN
        elif len(current) == 1 and moving:
            self._motion_mode = MotionMode.FULL_SLIP
        elif moving and stationary:
            self._motion_mode = MotionMode.PARTIAL_SLIP
        elif stationary:
            self._motion_mode = MotionMode.STATIONARY
        elif moving:
            self._motion_mode = MotionMode.FULL_SLIP
        else:
            self._motion_mode = MotionMode.UNKNOWN

    def _update_active(
        self,
        state: TactileState,
        xy: Tuple[float, float],
        t: float,
    ) -> List[RobotGestureEvent]:
        if not self._active:
            self._begin(xy, t)
            return []

        dt_ms = max(0.0, (t - self._last_time) * 1000.0)
        prev = self._last_xy if self._last_xy is not None else xy
        step = _euclid(xy, prev)

        # 起始稳定期：刚接触的头 settle_ms 毫秒只跟踪位置，不计入轨迹。
        #
        # 手指刚压上织物时压力分布尚未稳定，实测质心在头两帧内从 15.24 跳到
        # 13.26（单帧 1.99 格），随后才稳定在 14.0 附近。这些帧会凭空堆积
        # path_len（实测把一次静止按压算成 path_len=3.52），使一次轻点被判成
        # back_forth_stroke。它们不代表手指的主动移动，必须排除。
        settled_ms = (t - self._start_time) * 1000.0
        if settled_ms < self.cfg.settle_ms:
            self._last_xy = xy
            self._last_time = t
            # 用稳定期结束时的位置重新锚定起点，避免头部抖动影响 max_move。
            self._start_xy = xy
            self._path = [xy]
            return []

        if step > self.cfg.stroke_max_step:
            # Contact-cluster reassignment can jump the global centroid. Treat it
            # as a tracking discontinuity, not as physical sliding distance.
            self._last_xy = xy
            self._last_time = t
            return []

        # A multi-contact stationary frame can move the global centroid solely
        # because pressure shifts between fingers. Only moving contact clusters
        # contribute to the dynamic trajectory.
        multi_motion_confirmed = self._multi_motion_frames >= 3
        if not (
            self._contact_mode is ContactMode.MULTI
            and self._motion_mode is MotionMode.STATIONARY
        ) and (self._contact_mode is not ContactMode.MULTI or multi_motion_confirmed) \
                and step > self.cfg.stroke_pause_tol:
            self._path_len += step

        if step <= self.cfg.stroke_pause_tol:
            self._pause_ms += dt_ms
            if self._pause_ms >= self.cfg.stroke_pause_break_ms:
                # 停顿过长：重置连续段，避免跨越停顿拼接成同一个 stroke。
                self._seg_start_idx = len(self._path)
                self._seg_disp = [0.0, 0.0]
                self._last_dir = None
        else:
            self._pause_ms = 0.0

        # 只有当这一步确实在移动时才累积轨迹与方向统计。
        if step > self.cfg.stroke_pause_tol:
            dx = xy[0] - prev[0]
            dy = xy[1] - prev[1]
            self._path.append(xy)
            self._seg_disp[0] += dx
            self._seg_disp[1] += dy

            self._track_reversal(dx, dy)

        if self._start_xy is not None:
            self._max_move = max(self._max_move, _euclid(xy, self._start_xy))
        self._last_xy = xy
        self._last_time = t
        return []

    def _track_reversal(self, dx: float, dy: float) -> None:
        """用迟滞（Schmitt trigger）检测方向反转。

        为什么不能用「相邻步向量点积 < 0 就算反转」
        --------------------------------------------
        实测一次干净的单程滑动（run5-stroke.bin，57 个活跃点）用那种判法会
        得到 **8~10 次反转**。原因是手指滑动时每帧位移都带抖动。

        为什么基准不能用「第一步位移」
        ------------------------------
        对角线滑动（实测 run5 净位移 dx=+29.4 / dy=-30.8，接近 45°）的
        首步往往偏重某一轴（实测首步 (0.39,-2.43) 几乎纯纵向）。若拿它当
        固定基准，横轴分量的正常前进会被判成「反向」，反向累计持续增长，
        最终误报多次反转。

        正确判法
        --------
        1. 主方向 ``_last_dir`` 用**指数滑动平均**持续跟踪真实走向，
           而不是固定成第一步；
        2. 每帧算位移在主方向上的投影：顺向则衰减反向累计，反向则累加；
        3. 反向累计超过 ``reversal_min_disp`` 才算一次真反转，然后重置。
        """
        # 建立主方向基准：只在首次、或确认反转后重置。
        #
        # 不能每帧用 EMA 跟随 —— 手指真正反向滑动时，EMA 会跟着一起转向，
        # 于是主方向上的投影始终为正，反转永远检测不到（实测 run9 的
        # forward=42.6 / backward=39.9，是强往返，却因 EMA 跟随被判成单程）。
        if self._last_dir is None:
            self._last_dir = (dx, dy)
            self._run_disp = [dx, dy]
            self._against_disp = [0.0, 0.0]
            return

        lx, ly = self._last_dir
        norm = math.hypot(lx, ly)
        if norm < 1e-9:
            self._last_dir = (dx, dy)
            self._against_disp = [0.0, 0.0]
            return

        proj = (dx * lx + dy * ly) / norm

        if proj >= 0.0:
            # 顺向：主方向缓慢跟随（只允许小幅修正，防止整体走向漂移），
            # 反向累计衰减，使横向抖动无法慢慢攒成假反转。
            self._run_disp[0] += dx
            self._run_disp[1] += dy
            decay = self.cfg.reversal_decay
            self._against_disp[0] *= decay
            self._against_disp[1] *= decay
            follow = self.cfg.reversal_dir_follow
            self._last_dir = (lx + follow * dx, ly + follow * dy)
            return

        # 反向：累加反向位移，达到阈值才算真反转。
        self._against_disp[0] += dx
        self._against_disp[1] += dy
        self._backward_total += math.hypot(dx, dy)

        # 反转判据：既要绝对幅度够大，也要占累计行程的比例够高。
        #
        # 只用绝对阈值挡不住单程滑动的横向抖动：实测干净单程(run5)的横向
        # 抖动累计已达 3.1 格，而真往返(run9)的反向是 39.9 格。用固定 1.2
        # 格当门槛时，前者会被切成 6 次假反转。
        # 比例判据能区分两者：单程的 backward/path 约 0.05，真往返约 0.48。
        magnitude = math.hypot(*self._against_disp)
        if magnitude < self.cfg.reversal_min_disp:
            return
        if self._path_len > 1e-6:
            ratio = self._backward_total / self._path_len
            if ratio < self.cfg.reversal_min_ratio:
                return
        self._reversals += 1
        self._last_dir = (self._against_disp[0], self._against_disp[1])
        self._run_disp = list(self._against_disp)
        self._against_disp = [0.0, 0.0]
        self._backward_total = 0.0

    def _begin(self, xy: Tuple[float, float], t: float) -> None:
        self._active = True
        self._start_time = t
        self._last_time = t
        self._start_xy = xy
        self._last_xy = xy
        self._path = [xy]
        self._path_len = 0.0
        self._max_move = 0.0
        self._pause_ms = 0.0
        self._seg_start_idx = 0
        self._reversals = 0
        self._last_dir = None
        self._seg_disp = [0.0, 0.0]
        self._run_disp = [0.0, 0.0]
        self._against_disp = [0.0, 0.0]
        self._backward_total = 0.0
        self._pending_release_at = None

    # ------------------------------------------------------------------ #
    def _update_release(self, t: float) -> List[RobotGestureEvent]:
        if not self._active:
            return []

        # held_ms 只统计到「最后一次有效接触」，不含释放消抖的等待时间，
        # 否则每次点击都会被多算 release_debounce_ms 毫秒。
        end_t = self._last_time if self._last_time else t
        held_ms = (end_t - self._start_time) * 1000.0
        events: List[RobotGestureEvent] = []
        event = self._classify(t, held_ms)
        if event is not None:
            events.append(event)
        self.reset()
        return events

    def _classify(self, t: float, held_ms: float) -> Optional[RobotGestureEvent]:
        """抬手时根据本次接触的轨迹判定手势。

        判定顺序与兜底
        --------------
        旧实现要求 tap 必须同时满足 ``held_ms <= tap_max_ms`` 且
        ``max_move <= tap_move_tol``，否则落入 stroke 分支；而 stroke 又要求
        ``path_len >= stroke_min_path``。两者之间留了一大片**静默丢弃**区间：

            漂移 >0.9 格 但累计轨迹 <3.0 格  -> 什么都不报
            漂移 <=0.9 格 但按住 >320ms       -> 什么都不报

        真实手指点击的质心漂移普遍超过 0.9 格（手指有肉、按压力心会移动），
        所以实测 4 次点击只偶尔报出 1 次。

        现在改为：**只要轨迹不足以构成 stroke，就按 tap 处理**。tap 的语义是
        「短促、没有明显滑动的接触」，不该用一个极紧的格数上限去卡它。
        """
        start = self._start_xy
        end = self._last_xy
        start_xy = start or end
        end_xy = end or start

        net_x = (end_xy[0] - start_xy[0]) if start_xy and end_xy else 0.0
        net_y = (end_xy[1] - start_xy[1]) if start_xy and end_xy else 0.0
        net = math.hypot(net_x, net_y)

        # ---- stroke 是否成立 ---- #
        # 累计轨迹够长就说明手指在皮肤上滑动过，这是 stroke 的本质特征。
        #
        # 不能要求 ``net >= stroke_min_net``：往返滑动的起止点几乎重合，
        # 净位移接近 0，若以此为前提会把 back_forth 误判成 tap。
        # 净位移只在「区分单程滑动方向」时才有意义。
        is_stroke = (
            held_ms >= max(self.cfg.stroke_min_ms, self.cfg.min_contact_ms)
            and self._path_len >= self.cfg.stroke_min_path
        )

        if not is_stroke:
                    # 轨迹不够长 -> 静态触摸。
            # 短于 min_contact_ms 的接触视为噪声：实测输出里大量
            # held_ms=50 drift=0.0 path_len=0.0 的幽灵事件会误触发真实动作。
            if held_ms < self.cfg.min_contact_ms:
                return None
            # 按住过久（比如手掌一直搭着）不算静态事件，避免误触发。
            if held_ms > self.cfg.press_max_ms:
                return None
            return RobotGestureEvent(
                RobotGesture.STATIC_TOUCH,
                t,
                confidence=1.0,
                contact_mode=self._contact_mode,
                motion_mode=self._motion_mode,
                moving_contacts=self._moving_contacts,
                stationary_contacts=self._stationary_contacts,
                start=start_xy,
                end=end_xy,
                data={
                    "held_ms": round(held_ms),
                    "drift": round(self._max_move, 2),
                    "path_len": round(self._path_len, 2),
                },
            )

        # ---- stroke 分类 ---- #
        # 用「净位移 / 累计路程」这个比值判断是否来回，不再维护方向状态机。
        #
        # 实测数据（同一次接触内）：
        #   单程滑动   net/path ≈ 0.90 ~ 1.00   （几乎直线前进）
        #   来回抚摸   net/path ≈ 0.01 ~ 0.25   （走很多路只净移动一点）
        # 这个比值天然免疫横向抖动：抖动同时抬高 path 与降低 net，方向一致。
        # 而之前基于「相邻步点积」或「反向累计阈值」的判法，在单程滑动上会
        # 误报 6~10 次反转（run5 实测 backward=3.1 格，全是横向抖动）。
        straightness = (net / self._path_len) if self._path_len > 1e-6 else 1.0
        # 三角不等式保证 net <= path_len，比值不该超过 1。
        # 越界说明 path_len 漏算了位移（历史 bug 曾产出 1.225/1.247），
        # 这里夹紧以防下游据此做出错误分类。
        straightness = max(0.0, min(1.0, straightness))
        direction = _dominant_direction(net_x, net_y, self.cfg.direction_dominance)

        if straightness < self.cfg.back_forth_max_straightness:
            return RobotGestureEvent(
                RobotGesture.DYNAMIC_TOUCH,
                t,
                confidence=self._stroke_confidence(net),
                direction=direction,
                contact_mode=self._contact_mode,
                motion_mode=self._motion_mode,
                moving_contacts=self._moving_contacts,
                stationary_contacts=self._stationary_contacts,
                start=start_xy,
                end=end_xy,
                data={
                    "path_len": round(self._path_len, 2),
                    "straightness": round(straightness, 3),
                    "held_ms": round(held_ms),
                },
            )

        # 无有效反转、但净位移也很小：原地抖动把 path_len 累加到了阈值以上，
        # 这不是有意的单向滑动。按静态触摸处理。
        if net < self.cfg.stroke_min_net:
            if held_ms > self.cfg.press_max_ms:
                return None
            return RobotGestureEvent(
                RobotGesture.STATIC_TOUCH,
                t,
                confidence=1.0,
                contact_mode=self._contact_mode,
                motion_mode=self._motion_mode,
                moving_contacts=self._moving_contacts,
                stationary_contacts=self._stationary_contacts,
                start=start_xy,
                end=end_xy,
                data={
                    "held_ms": round(held_ms),
                    "drift": round(self._max_move, 2),
                    "path_len": round(self._path_len, 2),
                },
            )

        return RobotGestureEvent(
            RobotGesture.DYNAMIC_TOUCH,
            t,
            confidence=self._stroke_confidence(net),
            direction=direction,
            contact_mode=self._contact_mode,
            motion_mode=self._motion_mode,
            moving_contacts=self._moving_contacts,
            stationary_contacts=self._stationary_contacts,
            start=start_xy,
            end=end_xy,
            data={
                "path_len": round(self._path_len, 2),
                "straightness": round(straightness, 3),
                "held_ms": round(held_ms),
            },
        )

    def _stroke_confidence(self, net: float) -> float:
        """手势置信度 = 本次接触「有多明确地属于它被判定的那一类」。

        之前的实现直接返回 ``net / path_len``，这对 back_forth_stroke 是**反的**：
        来回抚摸本来就该回到起点，net≈0，于是越是标准的来回，得分越接近 0。
        实测出现过 conf=0.01 的完美来回（path 48.79 格、2 次反转），看起来
        像"不可信"，实际恰恰是最可信的。

        现在按类别分别计算：
          * single_stroke：越直越可信 -> ``net / path_len``
          * back_forth_stroke：越弯越可信 -> ``1 - net / path_len``
        两者都落在 0~1，且方向直观（1 = 非常典型）。
        """
        if self._path_len <= 1e-6:
            return 0.0
        straightness = max(0.0, min(1.0, net / self._path_len))
        if straightness < self.cfg.back_forth_max_straightness:
            # 判为来回：越不直越可信。
            span = max(1e-6, self.cfg.back_forth_max_straightness)
            return round(max(0.0, min(1.0, (span - straightness) / span)), 3)
        return round(straightness, 3)

    # ------------------------------------------------------------------ #
    def force_stop(self) -> None:
        """急停：放弃当前接触，不产生事件。"""
        self.reset()
