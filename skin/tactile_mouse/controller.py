"""@file controller.py
@brief 触觉数据到鼠标动作的控制器主循环。
@details
    该模块接收 TactileProcessor 生成的状态，调用 GestureRecognizer 识别手势，
    然后按照 joystick / absolute / ninegrid 三种映射模式把动作转换为实际鼠标输入。

控制器：把触觉阵列的一帧帧数据变成真实的鼠标动作。

数据流::

    原始帧 -> TactileProcessor（校准/去噪/质心/多指分割）
           -> GestureRecognizer（轻点/双击/长按/重压拖拽/双指滚动）
           -> Controller（摇杆解算 + 平滑 + 限幅 + 安全钳制）
           -> MouseBackend（SendInput / pynput / pyautogui）

三种映射模式
------------
``joystick``  手指相对「零点」的偏移决定光标**速度**（方向 + 速度）。
              零点默认取本次落指的起始位置（``origin="touch"``），
              这样轻点时手指几乎不动，光标就不会漂，点击很准。
              拇指覆盖范围小、只需小幅移动即可驱动全屏。

``absolute``  手指在阵列上的位置直接映射到屏幕坐标（绝对定位）。
              适合大屏快速跳转，但精度受阵列分辨率（32x32）限制。

``ninegrid``  把 32x32 划成 3x3 功能键区：上下左右四个边格负责匀速移动，
              四角是单击/右键/双击/中键，中心是「停」。
              落指即触发、按键式操作，适合不习惯摇杆的人。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Tuple

from .config import AppConfig
from .features import TactileProcessor, TactileState
from .gestures import Gesture, GestureEvent, GestureRecognizer, apply_scroll_direction
from .mouse_backend import MouseBackend, create_backend


@dataclass
class ControllerStats:
    frames: int = 0
    gesture_events: int = 0
    fps: float = 0.0
    last_state: Optional[TactileState] = None
    recent_events: Deque[GestureEvent] = field(default_factory=lambda: deque(maxlen=40))
    velocity: Tuple[float, float] = (0.0, 0.0)
    last_error: str = ""


class Controller:
    """触觉皮肤 -> 鼠标。线程安全性：所有方法都加了内部锁，可跨线程调用。"""

    def __init__(
        self,
        cfg: Optional[AppConfig] = None,
        backend: Optional[MouseBackend] = None,
        on_event: Optional[Callable[[GestureEvent], None]] = None,
    ):
        self.cfg = cfg or AppConfig()
        self.backend = backend or create_backend()
        self.on_event = on_event

        self.processor = TactileProcessor(
            self.cfg.filter,
            invert_x=self.cfg.invert_grid_x,
            invert_y=self.cfg.invert_grid_y,
        )
        self.recognizer = GestureRecognizer(self.cfg.gesture)
        self.stats = ControllerStats()

        self.enabled = True
        self._lock = threading.RLock()
        self._vx = 0.0
        self._vy = 0.0
        self._origin: Optional[Tuple[float, float]] = None
        self._last_move_t: Optional[float] = None
        self._last_ninegrid_action: Optional[str] = None
        self._ref_total = 0.0
        self._frame_times: Deque[float] = deque(maxlen=400)

    # ------------------------------------------------------------------ #
    # 主入口
    # ------------------------------------------------------------------ #
    def feed(self, points, frame_no: int = 0, timestamp: Optional[float] = None) -> TactileState:
        """喂入一帧阵列数据（已由协议层解析出来的点列表）。"""
        t = timestamp if timestamp is not None else time.perf_counter()
        with self._lock:
            state = self.processor.process(points, frame_no=frame_no, timestamp=t)

            if not self.processor.calibrated:
                # 零点校准阶段：只收集静置数据，不产生任何鼠标动作
                self.processor.push_calibration_frame(state.raw)
                self.stats.last_state = state
                return state

            events = self.recognizer.update(state)
            self.stats.frames += 1
            self.stats.last_state = state
            self._update_fps(t)
            for event in events:
                self._dispatch(event)

            self._drive_mouse(state, t)
            return state

    # ------------------------------------------------------------------ #
    # 手势 -> 鼠标动作
    # ------------------------------------------------------------------ #
    def _dispatch(self, event: GestureEvent) -> None:
        self.stats.gesture_events += 1
        self.stats.recent_events.append(event)
        if self.enabled:
            try:
                if event.kind is Gesture.TAP:
                    self.backend.click("left", 1)
                elif event.kind is Gesture.DOUBLE_TAP and self.cfg.gesture.double_click_as_event:
                    self.backend.click("left", 2)
                elif event.kind is Gesture.LONG_PRESS:
                    self.backend.click("right", 1)
                elif event.kind is Gesture.DRAG_START:
                    self.backend.press("left")
                elif event.kind is Gesture.DRAG_END:
                    self.backend.release("left")
                elif event.kind is Gesture.SCROLL:
                    notches = apply_scroll_direction(
                        int(event.data.get("notches", 0)),
                        str(event.data.get("axis", "v")),
                        invert=self.cfg.mapping.scroll_invert,
                    )
                    axis = event.data.get("axis", "v")
                    gain = self.cfg.mapping.scroll_gain
                    if gain != 1.0:
                        notches = int(round(notches * gain))
                    self.backend.wheel(notches, axis)
            except Exception as exc:  # noqa: BLE001
                # 鼠标注入失败（例如目标程序以管理员身份运行）不应该中断整个循环
                self.stats.last_error = f"{event.kind.value}: {exc}"
        if self.on_event is not None:
            try:
                self.on_event(event)
            except Exception as exc:  # noqa: BLE001
                self.stats.last_error = f"on_event: {exc}"

    # ------------------------------------------------------------------ #
    # 位移解算
    # ------------------------------------------------------------------ #
    def _drive_mouse(self, state: TactileState, t: float) -> None:
        mapping = self.cfg.mapping

        if not self.enabled or not state.active or state.centroid is None:
            self._reset_motion(state, t)
            return
        if self.recognizer.suppress_move:
            # 滚动 / 长按期间不动光标
            self._reset_motion(state, t, keep_origin=True)
            return

        xy = state.centroid
        mode = (mapping.mode or "joystick").lower()

        if mode == "ninegrid":
            self._drive_ninegrid(state, xy, t)
            return

        self._last_ninegrid_action = None

        if mode == "absolute":
            self._drive_absolute(state, xy, t)
            return

        # ---- joystick ----
        if self._origin is None:
            self._origin = xy
        ox, oy = self._origin
        dx_cells = xy[0] - ox
        dy_cells = xy[1] - oy

        if mapping.origin == "array":
            range_cells = max(1e-6, (self.processor.cols - 1) / 2.0)
        else:
            range_cells = max(0.2, mapping.touch_range_cells)

        nx = max(-1.0, min(1.0, dx_cells / range_cells))
        ny = max(-1.0, min(1.0, dy_cells / range_cells))
        if mapping.invert_x:
            nx = -nx
        if mapping.invert_y:
            ny = -ny

        # 边缘跟随：顶到边界后让零点慢慢追上来，可一直平移下去
        if mapping.origin == "touch" and mapping.edge_recenter and self._last_move_t is not None:
            dt = max(0.0, min(0.2, t - self._last_move_t))
            if abs(dx_cells) >= range_cells * 0.98 or abs(dy_cells) >= range_cells * 0.98:
                step = mapping.edge_recenter_cells_s * dt
                sx = 0.0 if abs(dx_cells) < range_cells * 0.98 else math.copysign(min(step, abs(dx_cells)), dx_cells)
                sy = 0.0 if abs(dy_cells) < range_cells * 0.98 else math.copysign(min(step, abs(dy_cells)), dy_cells)
                self._origin = (ox + sx, oy + sy)

        # 死区 + 响应曲线（径向，保证斜向手感一致）
        radius = math.hypot(nx, ny)
        if radius <= mapping.deadzone or radius <= 1e-9:
            target_vx = 0.0
            target_vy = 0.0
        else:
            norm = min(1.0, (radius - mapping.deadzone) / max(1e-6, 1.0 - mapping.deadzone))
            shaped = norm ** max(0.1, mapping.gamma)
            speed = mapping.max_speed_px_s * self._pressure_factor(state)
            target_vx = nx / radius * shaped * speed
            target_vy = ny / radius * shaped * speed

        self._integrate(target_vx, target_vy, t)

    def _drive_absolute(self, state: TactileState, xy: Tuple[float, float], t: float) -> None:
        mapping = self.cfg.mapping
        cols, rows = self.processor.cols, self.processor.rows
        fx = xy[0] / max(1e-6, cols - 1)
        fy = xy[1] / max(1e-6, rows - 1)
        if mapping.invert_x:
            fx = 1.0 - fx
        if mapping.invert_y:
            fy = 1.0 - fy
        w, h = self.backend.screen_size()
        margin = max(0, mapping.abs_margin_px)
        tx = margin + fx * max(0, w - 1 - 2 * margin)
        ty = margin + fy * max(0, h - 1 - 2 * margin)
        cx, cy = self.backend.position()
        # 平滑靠拢，避免 32x32 分辨率下光标跳变
        alpha = max(0.05, min(1.0, mapping.speed_smoothing))
        nx = cx + (tx - cx) * alpha * 1.6
        ny = cy + (ty - cy) * alpha * 1.6
        self.backend.move_abs(int(round(nx)), int(round(ny)))
        self._vx = self._vy = 0.0
        self._last_move_t = t
        self.stats.velocity = (0.0, 0.0)
        self._track_reference(state)

    # 九宫格：3x3 分区，边格匀速移动，四角点按
    _NINEGRID_ACTIONS: List[List[str]] = [
        ["right_click", "up", "middle_click"],
        ["left", "stop", "right"],
        ["double_click", "down", "left_click"],
    ]

    def _drive_ninegrid(self, state: TactileState, xy: Tuple[float, float], t: float) -> None:
        mapping = self.cfg.mapping
        cols, rows = self.processor.cols, self.processor.rows
        cz = min(2, max(0, int(xy[0] / cols * 3)))
        rz = min(2, max(0, int(xy[1] / rows * 3)))
        if mapping.invert_x:
            cz = 2 - cz
        if mapping.invert_y:
            rz = 2 - rz
        action = self._NINEGRID_ACTIONS[rz][cz]

        # 落指瞬间只触发一次按键类动作
        if self._origin is None:
            self._origin = xy
            if action in ("left_click", "right_click", "double_click", "middle_click"):
                self._fire_ninegrid_button(action)
        ox, oy = self._origin
        range_cells = max(0.5, mapping.touch_range_cells)

        dirs = {"up": (0.0, -1.0), "down": (0.0, 1.0), "left": (-1.0, 0.0), "right": (1.0, 0.0)}
        if action in dirs:
            self._fire_ninegrid_move(action, t)
            ux, uy = dirs[action]
            speed = mapping.max_speed_px_s * self._pressure_factor(state)
            self._integrate(ux * speed, uy * speed, t)
        elif action == "stop":
            self._last_ninegrid_action = "stop"
            self._integrate(0.0, 0.0, t)
        else:
            self._last_ninegrid_action = None
            # 按钮格：允许小幅拖动，超出容忍范围就当作移动
            nx = max(-1.0, min(1.0, (xy[0] - ox) / range_cells))
            ny = max(-1.0, min(1.0, (xy[1] - oy) / range_cells))
            if math.hypot(nx, ny) > 0.55:
                speed = mapping.max_speed_px_s * 0.5 * self._pressure_factor(state)
                self._integrate(nx * speed, ny * speed, t)
            else:
                self._integrate(0.0, 0.0, t)

    def _fire_ninegrid_move(self, action: str, t: float) -> None:
        if self._last_ninegrid_action == action:
            return
        self._last_ninegrid_action = action
        gesture_map = {
            "up": Gesture.MOVE_UP,
            "down": Gesture.MOVE_DOWN,
            "left": Gesture.MOVE_LEFT,
            "right": Gesture.MOVE_RIGHT,
        }
        event = GestureEvent(gesture_map[action], t, {"direction": action})
        self._dispatch(event)

    def _fire_ninegrid_button(self, action: str) -> None:
        try:
            if action == "left_click":
                self.backend.click("left")
            elif action == "right_click":
                self.backend.click("right")
            elif action == "double_click":
                self.backend.click("left", 2)
            elif action == "middle_click":
                self.backend.click("middle")
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    # 速度积分与限幅
    # ------------------------------------------------------------------ #
    def _integrate(self, target_vx: float, target_vy: float, t: float) -> None:
        mapping = self.cfg.mapping
        smoothing = max(0.0, min(1.0, mapping.speed_smoothing))
        self._vx += (target_vx - self._vx) * smoothing
        self._vy += (target_vy - self._vy) * smoothing

        if self._last_move_t is None:
            self._last_move_t = t
            return
        dt = t - self._last_move_t
        self._last_move_t = t
        if dt <= 0:
            return
        dt = min(dt, 0.1)  # 掉帧保护：不让一次超大位移把光标甩飞

        dx = self._vx * dt
        dy = self._vy * dt
        limit = max(1.0, mapping.max_step_px)
        dist = math.hypot(dx, dy)
        if dist > limit:
            dx *= limit / dist
            dy *= limit / dist
        self.backend.move_rel(dx, dy)
        self.stats.velocity = (self._vx, self._vy)

    def _pressure_factor(self, state: TactileState) -> float:
        """按压力度参与调速：轻推慢移、重推快移。"""
        mix = max(0.0, min(1.0, self.cfg.mapping.pressure_speed_mix))
        self._track_reference(state)
        if mix <= 0 or self._ref_total <= 0:
            return 1.0
        ratio = max(0.0, min(2.0, state.total / self._ref_total))
        return (1.0 - mix) + mix * min(2.0, max(0.25, ratio))

    def _track_reference(self, state: TactileState) -> None:
        if self.cfg.mapping.pressure_ref > 0:
            self._ref_total = self.cfg.mapping.pressure_ref
            return
        if state.total > 0:
            if self._ref_total <= 0:
                self._ref_total = state.total
            else:
                self._ref_total += 0.02 * (state.total - self._ref_total)

    def _reset_motion(self, state: TactileState, t: float, keep_origin: bool = False) -> None:
        self._vx *= 0.5
        self._vy *= 0.5
        if abs(self._vx) < 1.0:
            self._vx = 0.0
        if abs(self._vy) < 1.0:
            self._vy = 0.0
        self._last_move_t = t
        if not keep_origin:
            self._origin = None
            self._last_ninegrid_action = None
        self.stats.velocity = (self._vx, self._vy)

    def _update_fps(self, t: float) -> None:
        self._frame_times.append(t)
        cutoff = t - 1.0
        while len(self._frame_times) > 1 and self._frame_times[0] < cutoff:
            self._frame_times.popleft()
        span = t - self._frame_times[0] if len(self._frame_times) > 1 else 0.0
        self.stats.fps = (len(self._frame_times) - 1) / span if span > 0 else 0.0

    # ------------------------------------------------------------------ #
    # 控制接口
    # ------------------------------------------------------------------ #
    def set_enabled(self, enabled: bool) -> None:
        with self._lock:
            if self.enabled and not enabled:
                self.emergency_stop()
            self.enabled = enabled

    def emergency_stop(self) -> None:
        """急停：松开所有按键、清空速度与手势状态。"""
        with self._lock:
            self.recognizer.force_stop()
            try:
                self.backend.release_all()
            except Exception:  # noqa: BLE001
                pass
            self._vx = self._vy = 0.0
            self._origin = None
            self._last_ninegrid_action = None
            self.stats.velocity = (0.0, 0.0)

    def recalibrate(self) -> None:
        """重新做零点校准（设备静置时调用）。"""
        with self._lock:
            self.processor.start_calibration()
            self.processor.reset_ema()

    def close(self) -> None:
        self.emergency_stop()
        try:
            self.backend.close()
        except Exception:  # noqa: BLE001
            pass
