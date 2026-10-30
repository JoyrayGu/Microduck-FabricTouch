"""@file features.py
@brief 压力阵列特征提取与零点校准。
@details
    输入一帧 rows × cols 的原始压力值，执行去噪、指数平滑、
    激活判定、零点扣除以及多指分割，并输出 TactileState，供手势识别与控制器消费。

触觉阵列特征提取：零点校准 / 去噪 / 平滑 / 接触点分割。

输入：一帧 ``rows*cols`` 的原始压力值（来自协议解析，行优先）。
输出：``TactileState``，包含清洁后的压力场、总压力、峰值、压力质心、
以及按连通域分割出的多个接触点（用于多指手势）。
"""

from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Iterable, List, Optional, Sequence, Tuple

from .config import FilterConfig


@dataclass
class Contact:
    """一个独立接触点（一根手指压出的一块区域）。"""

    cx: float  # 质心列坐标（0 ~ cols-1，可为小数）
    cy: float  # 质心行坐标（0 ~ rows-1，可为小数）
    weight: float  # 该接触点的压力总和
    peak: float  # 该接触点的峰值压力
    cells: int  # 激活格数
    min_r: int = 0
    max_r: int = 0
    min_c: int = 0
    max_c: int = 0

    @property
    def size(self) -> float:
        """接触面积（格数）的粗略度量。"""
        return float(self.cells)


@dataclass
class TactileState:
    """一帧的处理结果，供手势识别与映射使用。"""

    raw: List[float]
    grid: List[float]
    total: float = 0.0
    peak: float = 0.0
    active: bool = False
    centroid: Optional[Tuple[float, float]] = None
    contacts: List[Contact] = field(default_factory=list)
    frame_no: int = 0
    #: 该帧的时间戳（秒）。None 表示由下游自行取当前时间。
    timestamp: Optional[float] = None

    @property
    def n_contacts(self) -> int:
        return len(self.contacts)

    @property
    def primary(self) -> Optional[Contact]:
        """权重最大的接触点。"""
        return self.contacts[0] if self.contacts else None

    def ascii_art(self, cols: int = 32, rows: int = 32) -> str:
        """把压力场画成字符图，方便在终端里快速观察。"""
        if not self.grid:
            return ""
        chars = " .:-=+*#%@"
        peak = max(self.grid) or 1.0
        lines = []
        for r in range(rows):
            row = []
            for c in range(cols):
                v = self.grid[r * cols + c] / peak
                idx = min(len(chars) - 1, int(v * (len(chars) - 1) + 0.5))
                row.append(chars[idx])
            lines.append("".join(row))
        return "\n".join(lines)


class TactileProcessor:
    """状态化的阵列处理器（逐帧调用 ``process``）。"""

    def __init__(
        self,
        cfg: Optional[FilterConfig] = None,
        invert_x: bool = False,
        invert_y: bool = False,
    ):
        self.cfg = cfg or FilterConfig()
        self.rows = self.cfg.rows
        self.cols = self.cfg.cols
        self.invert_x = invert_x
        self.invert_y = invert_y

        n = self.rows * self.cols
        self.baseline: List[float] = [0.0] * n
        self._calib_samples: List[List[float]] = []
        self._ema: Optional[List[float]] = None
        self._published_contact_count = 0
        self._candidate_contact_count: Optional[int] = None
        self._candidate_contact_frames = 0
        # 不需要校准就直接认为已校准
        self._calib_done = self.cfg.baseline_frames <= 0

    # -- 零点校准 ---------------------------------------------------------- #
    @property
    def calibrated(self) -> bool:
        return self._calib_done

    def start_calibration(self) -> None:
        self._calib_samples.clear()
        self._calib_done = self.cfg.baseline_frames <= 0

    def push_calibration_frame(self, raw: Sequence[float]) -> bool:
        """收集一帧校准数据，凑够 ``baseline_frames`` 后算出零点。返回是否完成。"""
        if self.cfg.baseline_frames <= 0:
            self._calib_done = True
            return True
        self._calib_samples.append([float(v) for v in raw])
        if len(self._calib_samples) < self.cfg.baseline_frames:
            return False
        n = len(self._calib_samples[0])
        # A transient touch during calibration must not raise every channel's zero.
        self.baseline = [
            float(statistics.median(s[i] for s in self._calib_samples))
            for i in range(n)
        ]
        self._calib_samples.clear()
        self._calib_done = True
        self._ema = None
        return True

    @property
    def calibration_progress(self) -> float:
        if self.cfg.baseline_frames <= 0:
            return 1.0
        return min(1.0, len(self._calib_samples) / self.cfg.baseline_frames)

    def reset_ema(self) -> None:
        self._ema = None
        self._published_contact_count = 0
        self._candidate_contact_count = None
        self._candidate_contact_frames = 0

    def _stabilize_contacts(self, contacts: List[Contact]) -> List[Contact]:
        """Suppress short-lived contact-count changes caused by split peaks."""
        count = len(contacts)
        required = max(0, int(self.cfg.contact_count_stability_frames))
        if required <= 1:
            self._published_contact_count = count
            return contacts
        if self._published_contact_count == 0 and count > 0:
            self._published_contact_count = count
            self._candidate_contact_count = None
            self._candidate_contact_frames = 0
            return contacts
        if count == self._published_contact_count:
            self._candidate_contact_count = None
            self._candidate_contact_frames = 0
            return contacts
        if count != self._candidate_contact_count:
            self._candidate_contact_count = count
            self._candidate_contact_frames = 1
        else:
            self._candidate_contact_frames += 1
        if self._candidate_contact_frames < required:
            target = self._published_contact_count
            return contacts[:target] if target else []
        self._published_contact_count = count
        self._candidate_contact_count = None
        self._candidate_contact_frames = 0
        return contacts

    # -- 逐帧处理 ---------------------------------------------------------- #
    def process(
        self,
        raw: Sequence[float],
        frame_no: int = 0,
        timestamp: Optional[float] = None,
    ) -> TactileState:
        rows, cols = self.rows, self.cols
        n = rows * cols
        raw = list(raw[:n]) + [0.0] * max(0, n - len(raw))
        t = timestamp if timestamp is not None else time.perf_counter()

        # 1) 扣零点 + 截断负值
        if self.cfg.use_baseline and self._calib_done:
            clean = [max(0.0, raw[i] - self.baseline[i]) for i in range(n)]
        else:
            clean = [max(0.0, float(v)) for v in raw]

        # 2) 空间平滑（3x3 均值核）：阵列小，这一步能明显压掉孤立噪点
        for _ in range(self.cfg.spatial_smooth_passes):
            clean = self._smooth3x3(clean)

        # 3) 时域指数平滑
        alpha = max(0.0, min(1.0, self.cfg.ema_alpha))
        if self._ema is None or len(self._ema) != n:
            self._ema = list(clean)
        else:
            ema = self._ema
            for i in range(n):
                ema[i] += alpha * (clean[i] - ema[i])
            clean = list(ema)

        peak = max(clean) if clean else 0.0
        # 4) 激活判定：绝对阈值与相对阈值取大者
        threshold = max(self.cfg.activate_abs, self.cfg.activate_rel * peak)
        mask = [v >= threshold and v > 0 for v in clean]
        active_cells = sum(1 for m in mask if m)

        state = TactileState(raw=list(raw), grid=clean, peak=peak,
                             frame_no=frame_no, timestamp=t)
        if active_cells < self.cfg.min_active_cells:
            self._published_contact_count = 0
            self._candidate_contact_count = None
            self._candidate_contact_frames = 0
            return state

        state.active = True
        state.total = sum(clean[i] for i in range(n) if mask[i])

        # 5) 连通域分割 -> 多指接触点
        state.contacts = self._stabilize_contacts(self._find_contacts(clean, mask))
        if state.contacts:
            px, py = self._to_grid_xy(state.contacts[0].cx, state.contacts[0].cy)
            state.centroid = (px, py)
        return state

    # -- 内部工具 ---------------------------------------------------------- #
    def _smooth3x3(self, grid: Sequence[float]) -> List[float]:
        rows, cols = self.rows, self.cols
        out = [0.0] * len(grid)
        for r in range(rows):
            for c in range(cols):
                acc = 0.0
                cnt = 0
                for dr in (-1, 0, 1):
                    rr = r + dr
                    if not 0 <= rr < rows:
                        continue
                    for dc in (-1, 0, 1):
                        cc = c + dc
                        if 0 <= cc < cols:
                            acc += grid[rr * cols + cc]
                            cnt += 1
                out[r * cols + c] = acc / cnt if cnt else grid[r * cols + c]
        return out

    def _find_contacts(self, grid: Sequence[float], mask: Sequence[bool]) -> List[Contact]:
        """把激活区域分割成若干接触点。

        纯连通域做法在阵列分辨率低、压力扩散广时会把两根并拢的手指糊成一个
        接触点（两块压力的中间谷值仍高于阈值）。这里改用触觉阵列处理里更
        常见的做法：

        1. 找出所有 3x3 邻域内的局部峰值；
        2. 距离近于 ``merge_radius_cells`` 的峰值合并（同一根手指的多个峰）；
        3. 每个激活格按欧氏距离就近分配给离它最近的峰值；
        4. 每簇算质心/峰值/面积，丢掉太小的簇。

        这样即使两指的中间谷值没降到阈值以下，也能正确分成两个接触点。

        注意：单点自然接触的相邻压力峰可能与真实双点的峰距重叠，
        因此空间分割不擅自扩大合并半径；接触数量的连续稳定由上层迟滞处理。
        """
        rows, cols = self.rows, self.cols
        peaks = self._local_peaks(grid, mask)
        if not peaks:
            return self._find_contacts_by_connectivity(grid, mask)
        if len(peaks) == 1:
            return self._find_contacts_by_connectivity(grid, mask)

        # 分配激活格 -> 最近峰值
        buckets: List[List[int]] = [[] for _ in peaks]
        for r in range(rows):
            for c in range(cols):
                idx = r * cols + c
                if not mask[idx]:
                    continue
                best = 0
                best_d = float("inf")
                for k, (pr, pc, _v) in enumerate(peaks):
                    d = (pr - r) ** 2 + (pc - c) ** 2
                    if d < best_d:
                        best_d = d
                        best = k
                buckets[best].append(idx)

        contacts: List[Contact] = []
        for cells in buckets:
            if len(cells) < self.cfg.min_contact_cells:
                continue
            wsum = sum(grid[i] for i in cells)
            if wsum <= 0:
                continue
            cx = sum(grid[i] * (i % cols) for i in cells) / wsum
            cy = sum(grid[i] * (i // cols) for i in cells) / wsum
            rs = [i // cols for i in cells]
            cs = [i % cols for i in cells]
            contacts.append(
                Contact(
                    cx=cx,
                    cy=cy,
                    weight=wsum,
                    peak=max(grid[i] for i in cells),
                    cells=len(cells),
                    min_r=min(rs),
                    max_r=max(rs),
                    min_c=min(cs),
                    max_c=max(cs),
                )
            )
        if not contacts:
            return self._find_contacts_by_connectivity(grid, mask)
        contacts.sort(key=lambda ct: -ct.weight)
        return contacts[: self.cfg.max_contacts]

    def _local_peaks(self, grid: Sequence[float], mask: Sequence[bool]) -> List[Tuple[int, int, float]]:
        """找局部峰值，返回 [(row, col, value), ...]，按压力从大到小。

        注意「平台」问题：大面积轻触（或传感器饱和）会出现一整片等值的格子，
        若按逐格比较，一片平台会被误判成很多个峰，进而被错分成多根手指。
        所以先把「值相近且相邻」的格子聚成一个平台，一个平台只算一个峰，
        峰值位置取平台的加权质心。
        """
        rows, cols = self.rows, self.cols
        n = rows * cols
        peak_value = max(grid) if grid else 0.0
        eps = max(1.0, 0.02 * peak_value)  # 平台判定的容差
        visited = [False] * n
        plateaus: List[Tuple[float, float, float]] = []  # (value, row, col)

        for start in range(n):
            if not mask[start] or visited[start]:
                continue
            stack = [start]
            visited[start] = True
            cells: List[int] = []
            while stack:
                cur = stack.pop()
                cells.append(cur)
                cr, cc = divmod(cur, cols)
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        rr, ccc = cr + dr, cc + dc
                        if 0 <= rr < rows and 0 <= ccc < cols:
                            ni = rr * cols + ccc
                            if mask[ni] and not visited[ni] and abs(grid[ni] - grid[cur]) <= eps:
                                visited[ni] = True
                                stack.append(ni)

            value = max(grid[i] for i in cells)
            cellset = set(cells)
            is_peak = True
            for i in cells:
                cr, cc = divmod(i, cols)
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, ccc = cr + dr, cc + dc
                        if 0 <= rr < rows and 0 <= ccc < cols:
                            ni = rr * cols + ccc
                            if ni not in cellset and grid[ni] > value + eps:
                                is_peak = False
                                break
                    if not is_peak:
                        break
                if not is_peak:
                    break
            if not is_peak:
                continue
            wsum = sum(grid[i] for i in cells) or 1.0
            row = sum(grid[i] * (i // cols) for i in cells) / wsum
            col = sum(grid[i] * (i % cols) for i in cells) / wsum
            plateaus.append((value, row, col))

        plateaus.sort(key=lambda p: (-p[0], p[1], p[2]))
        step = float(self.cfg.merge_radius_cells)
        kept: List[Tuple[int, int, float]] = []
        for value, row, col in plateaus:
            if all((kr - row) ** 2 + (kc - col) ** 2 > step * step for kr, kc, _v in kept):
                kept.append((int(round(row)), int(round(col)), value))
            if len(kept) >= self.cfg.max_contacts:
                break
        return kept

    def _find_contacts_by_connectivity(
        self, grid: Sequence[float], mask: Sequence[bool]
    ) -> List[Contact]:
        """连通域分割（兜底/单峰场景）。"""
        rows, cols = self.rows, self.cols
        seen = [False] * (rows * cols)
        contacts: List[Contact] = []
        for r in range(rows):
            for c in range(cols):
                idx = r * cols + c
                if not mask[idx] or seen[idx]:
                    continue
                # BFS 8 邻域
                stack: Deque[int] = deque([idx])
                seen[idx] = True
                cells: List[int] = []
                while stack:
                    cur = stack.popleft()
                    cells.append(cur)
                    cr, cc = divmod(cur, cols)
                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            rr, ccc = cr + dr, cc + dc
                            if 0 <= rr < rows and 0 <= ccc < cols:
                                ni = rr * cols + ccc
                                if mask[ni] and not seen[ni]:
                                    seen[ni] = True
                                    stack.append(ni)
                if len(cells) < self.cfg.min_contact_cells:
                    continue
                wsum = sum(grid[i] for i in cells)
                if wsum <= 0:
                    continue
                cx = sum(grid[i] * (i % cols) for i in cells) / wsum
                cy = sum(grid[i] * (i // cols) for i in cells) / wsum
                rs = [i // cols for i in cells]
                cs = [i % cols for i in cells]
                contacts.append(
                    Contact(
                        cx=cx,
                        cy=cy,
                        weight=wsum,
                        peak=max(grid[i] for i in cells),
                        cells=len(cells),
                        min_r=min(rs),
                        max_r=max(rs),
                        min_c=min(cs),
                        max_c=max(cs),
                    )
                )
        contacts.sort(key=lambda ct: -ct.weight)
        return contacts[: self.cfg.max_contacts]

    def _to_grid_xy(self, cx: float, cy: float) -> Tuple[float, float]:
        """把 (质心列, 质心行) 转成外部坐标 (x, y)，并应用方向翻转。"""
        x = (self.cols - 1) - cx if self.invert_x else cx
        y = (self.rows - 1) - cy if self.invert_y else cy
        return x, y

    def contact_xy(self, contact: Contact) -> Tuple[float, float]:
        return self._to_grid_xy(contact.cx, contact.cy)

    # -- 归一化 ------------------------------------------------------------ #
    def normalized(self, x: float, y: float) -> Tuple[float, float]:
        """把格坐标映射到 [-1, 1]，(0,0) 为阵列中心。"""
        half_x = max(1e-6, (self.cols - 1) / 2.0)
        half_y = max(1e-6, (self.rows - 1) / 2.0)
        nx = (x - (self.cols - 1) / 2.0) / half_x
        ny = (y - (self.rows - 1) / 2.0) / half_y
        return max(-1.0, min(1.0, nx)), max(-1.0, min(1.0, ny))


def deserialize_grid(points: Sequence[int], rows: int, cols: int) -> List[float]:
    """协议点列表 -> 浮点网格（长度不足补零）。"""
    n = rows * cols
    out = [float(v) for v in points[:n]]
    out += [0.0] * (n - len(out))
    return out


def centroid_of(grid: Sequence[float], rows: int, cols: int) -> Optional[Tuple[float, float]]:
    """无状态的压力质心计算（给需要一次性计算的场合用）。"""
    total = sum(grid)
    if total <= 0:
        return None
    cx = sum(grid[r * cols + c] * c for r in range(rows) for c in range(cols)) / total
    cy = sum(grid[r * cols + c] * r for r in range(rows) for c in range(cols)) / total
    return cx, cy


def euclid(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
