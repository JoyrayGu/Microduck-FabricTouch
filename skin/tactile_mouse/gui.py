"""@file gui.py
@brief tkinter 实时监控与串口调试界面。
@details
    GUI 负责显示 32×32 压力热力图、实时数值、手势事件日志，以及串口调试面板，
    允许用户观察设备状态、打开/关闭串口并手动触发单次采集。

实时可视化界面（tkinter，标准库自带，无需额外依赖）。

界面内容：
  * 左侧：7x7 压力热力图，叠加压力质心/接触点标记；
  * 右侧：总压力/峰值/FPS/按键状态等实时读数 + 手势事件日志；
  * 底部：灵敏度、死区、响应曲线等滑杆，可边拖边试；
  * 顶部：启用/暂停、急停、重新校准、读电量、设备信息。

注意：真正控制鼠标的是后台工作线程；本界面只做展示与调参，
读取的都是不可变快照，不会卡住数据通路。
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Deque, Optional

from . import protocol as P
from .config import AppConfig
from .gestures import Gesture, GestureEvent
from .session import Session
from .transport import UsbTransport, list_serial_ports

CELL = 46  # 每个传感格的像素尺寸
PAD = 14


def _pick_font(root) -> tuple:
    """挑一个能正常显示中文的字体。"""
    try:
        import tkinter.font as tkfont

        families = set(tkfont.families(root))
        for name in ("Microsoft YaHei UI", "微软雅黑", "Microsoft YaHei", "PingFang SC", "Noto Sans CJK SC"):
            if name in families:
                return (name, 9)
    except Exception:  # noqa: BLE001
        pass
    return ("TkDefaultFont", 9)


# 压力热力图配色（浅色主题）：接近白 -> 橙 -> 深红
_RAMP = [
    (0.00, (247, 248, 250)),
    (0.25, (195, 226, 245)),
    (0.45, (255, 226, 168)),
    (0.65, (245, 152, 90)),
    (0.82, (214, 74, 61)),
    (1.00, (140, 26, 30)),
]


def ramp_color(ratio: float) -> str:
    ratio = max(0.0, min(1.0, ratio))
    for i in range(len(_RAMP) - 1):
        r0, c0 = _RAMP[i]
        r1, c1 = _RAMP[i + 1]
        if r0 <= ratio <= r1:
            k = 0.0 if r1 == r0 else (ratio - r0) / (r1 - r0)
            rgb = tuple(int(c0[j] + (c1[j] - c0[j]) * k) for j in range(3))
            return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"
    r, g, b = _RAMP[-1][1]
    return f"#{r:02x}{g:02x}{b:02x}"


class TactileMouseGUI:
    """tkinter 应用。调用 ``run()`` 进入事件循环。"""

    def __init__(
        self,
        session: Session,
        cfg: AppConfig,
        title: str = "电子触觉皮肤 · 鼠标控制器",
        enable_hotkey: bool = True,
        on_close=None,
    ):
        import tkinter as tk

        self.tk = tk
        self.session = session
        self.cfg = cfg
        self.on_close = on_close
        self._estop_flag = threading.Event()
        self._hotkey_listener = None
        self._event_index = 0  # 已渲染的事件总数（用单调计数，避免 deque 淘汰后错位）
        self._manual_debug_grid = None
        self._manual_debug_deadline = 0.0
        self.session.set_debug_hook(self._on_debug)

        self.root = tk.Tk()
        self.root.title(title)
        self.root.configure(bg="#f5f6f8")
        self.font = _pick_font(self.root)
        self.font_bold = (self.font[0], self.font[1], "bold")
        self.root.protocol("WM_DELETE_WINDOW", self._close)

        self._build_layout()
        if enable_hotkey:
            self._install_hotkey()
        self.root.after(40, self._tick)

    # ------------------------------------------------------------------ #
    # 界面搭建
    # ------------------------------------------------------------------ #
    def _build_layout(self) -> None:
        tk = self.tk
        rows, cols = self.cfg.filter.rows, self.cfg.filter.cols
        grid_w = cols * CELL
        grid_h = rows * CELL

        outer = tk.Frame(self.root, bg="#f5f6f8", padx=PAD, pady=PAD)
        outer.pack(fill="both", expand=True)

        # --- 顶部工具条 ---
        bar = tk.Frame(outer, bg="#f5f6f8")
        bar.pack(fill="x", pady=(0, 10))

        self.lbl_status = tk.Label(bar, text="初始中 ...", font=self.font_bold,
                                   bg="#f5f6f8", fg="#1b1d21")
        self.lbl_status.pack(side="left")

        tk.Button(bar, text="急停 (Esc)", font=self.font, bg="#d64a3d", fg="white",
                  activebackground="#b93a2f", activeforeground="white", relief="flat",
                  padx=14, pady=4, command=self.emergency_stop).pack(side="right", padx=(6, 0))
        self.btn_toggle = tk.Button(bar, text="暂停控制", font=self.font, relief="flat",
                                    bg="#e8eaee", padx=14, pady=4, command=self.toggle_enabled)
        self.btn_toggle.pack(side="right", padx=(6, 0))
        tk.Button(bar, text="重新校准", font=self.font, relief="flat", bg="#e8eaee",
                  padx=14, pady=4, command=self.recalibrate).pack(side="right", padx=(6, 0))
        tk.Button(bar, text="读电量", font=self.font, relief="flat", bg="#e8eaee",
                  padx=14, pady=4, command=self.session.read_battery).pack(side="right", padx=(6, 0))
        tk.Button(bar, text="设备信息", font=self.font, relief="flat", bg="#e8eaee",
                  padx=14, pady=4, command=self.session.set_device_info).pack(side="right", padx=(6, 0))

        body = tk.Frame(outer, bg="#f5f6f8")
        body.pack(fill="both", expand=True)

        # --- 左侧热力图 ---
        left = tk.Frame(body, bg="#ffffff", highlightbackground="#e3e6ea", highlightthickness=1)
        left.pack(side="left", fill="y")
        tk.Label(left, text="压力分布 (32x32)", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").pack(anchor="w", padx=10, pady=(8, 0))
        self.canvas = tk.Canvas(left, width=grid_w, height=grid_h, bg="#ffffff",
                                highlightthickness=0)
        self.canvas.pack(padx=10, pady=(6, 10))
        self._cells = []
        for r in range(rows):
            row_items = []
            for c in range(cols):
                x0, y0 = c * CELL + 1, r * CELL + 1
                rect = self.canvas.create_rectangle(x0, y0, x0 + CELL - 2, y0 + CELL - 2,
                                                    fill="#f7f8fa", outline="#e3e6ea")
                text = self.canvas.create_text(x0 + CELL / 2 - 1, y0 + CELL / 2 - 1, text="",
                                               font=(self.font[0], 7), fill="#6b7280")
                row_items.append((rect, text))
            self._cells.append(row_items)
        self.marker = self.canvas.create_oval(0, 0, 0, 0, outline="#1f6feb", width=2, state="hidden")
        self.contact_markers = [
            self.canvas.create_oval(0, 0, 0, 0, outline="#0b8f6a", width=2, state="hidden")
            for _ in range(3)
        ]

        # --- 右侧读数 / 日志 / 调参 ---
        right = tk.Frame(body, bg="#f5f6f8", padx=12)
        right.pack(side="left", fill="both", expand=True)

        stats = tk.Frame(right, bg="#ffffff", highlightbackground="#e3e6ea", highlightthickness=1)
        stats.pack(fill="x")
        self.readouts = {}
        labels = [
            ("total", "总压力"),
            ("peak", "峰值"),
            ("centroid", "质心 (x, y)"),
            ("contacts", "接触点数"),
            ("fps", "帧率"),
            ("button", "按键状态"),
            ("battery", "电量"),
            ("dropped", "丢帧"),
        ]
        for i, (key, text) in enumerate(labels):
            tk.Label(stats, text=text + "：", font=self.font, bg="#ffffff",
                     fg="#6b7280").grid(row=i // 2, column=(i % 2) * 2, sticky="e", padx=(10, 2), pady=3)
            value = tk.Label(stats, text="-", font=(self.font[0], 9, "bold"), bg="#ffffff",
                             fg="#1b1d21", width=16, anchor="w")
            value.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=(0, 16), pady=3)
            self.readouts[key] = value

        tk.Label(right, text="手势事件", font=self.font_bold, bg="#f5f6f8",
                 fg="#1b1d21").pack(anchor="w", pady=(12, 4))
        self.log = tk.Text(right, height=9, font=(self.font[0], 8), bg="#ffffff",
                           fg="#1b1d21", relief="flat", highlightbackground="#e3e6ea",
                           highlightthickness=1, state="disabled", wrap="word")
        self.log.pack(fill="both", expand=True)

        serial = tk.Frame(right, bg="#ffffff", highlightbackground="#e3e6ea", highlightthickness=1)
        serial.pack(fill="x", pady=(12, 0))
        tk.Label(serial, text="串口调试", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").grid(row=0, column=0, sticky="w", padx=10, pady=(8, 6), columnspan=3)

        ports = []
        try:
            ports = [p.device for p in list_serial_ports()]
        except Exception:  # noqa: BLE001
            ports = []
        current_port = getattr(self.session.transport, "port", None) or (ports[0] if ports else "")
        self.port_var = self.tk.StringVar(value=current_port)
        self.port_menu = tk.OptionMenu(serial, self.port_var, *ports) if ports else tk.Entry(serial, textvariable=self.port_var, width=12)
        self.port_menu.grid(row=1, column=0, sticky="ew", padx=(10, 6), pady=(0, 8))
        self.port_menu.configure(width=12)
        self.serial_open_btn = tk.Button(serial, text="打开串口", font=self.font, relief="flat", bg="#e8eaee",
                                         padx=8, pady=3, command=self._open_serial_port)
        self.serial_open_btn.grid(row=1, column=1, sticky="w", padx=6, pady=(0, 8))
        self.serial_close_btn = tk.Button(serial, text="关闭串口", font=self.font, relief="flat", bg="#e8eaee",
                                          padx=8, pady=3, command=self._close_serial_port)
        self.serial_close_btn.grid(row=1, column=2, sticky="w", padx=6, pady=(0, 8))
        self.single_capture_btn = tk.Button(serial, text="单次采集", font=self.font, relief="flat", bg="#dbeafe",
                                            padx=8, pady=3, command=self._single_capture_debug)
        self.single_capture_btn.grid(row=1, column=3, sticky="w", padx=6, pady=(0, 8))
        serial.grid_columnconfigure(0, weight=1)

        tk.Label(serial, text="收发日志", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").grid(row=2, column=0, sticky="w", padx=10, pady=(0, 4), columnspan=3)
        self.debug_log = tk.Text(serial, height=7, font=(self.font[0], 8), bg="#ffffff",
                                fg="#1b1d21", relief="flat", highlightbackground="#e3e6ea",
                                highlightthickness=1, state="disabled", wrap="word")
        self.debug_log.grid(row=3, column=0, columnspan=3, sticky="ew", padx=10, pady=(0, 10))

        # --- 右侧调参（从阵列下方挪到右侧）---
        ctl = tk.Frame(right, bg="#ffffff", highlightbackground="#e3e6ea", highlightthickness=1)
        ctl.pack(fill="x", pady=(12, 0))

        m = self.cfg.mapping
        self.var_speed = self._slider(ctl, "最大速度 (像素/秒)", 100, 2500, m.max_speed_px_s,
                                      lambda v: setattr(m, "max_speed_px_s", float(v)), row=0)
        self.var_dead = self._slider(ctl, "死区", 0.0, 0.5, m.deadzone,
                                     lambda v: setattr(m, "deadzone", float(v)), row=1)
        self.var_gamma = self._slider(ctl, "响应曲线 γ", 0.5, 3.0, m.gamma,
                                      lambda v: setattr(m, "gamma", float(v)), row=2)
        self.var_press = self._slider(ctl, "按压力度调速", 0.0, 1.0, m.pressure_speed_mix,
                                      lambda v: setattr(m, "pressure_speed_mix", float(v)), row=3)
        self.var_smooth = self._slider(ctl, "平滑", 0.05, 1.0, m.speed_smoothing,
                                       lambda v: setattr(m, "speed_smoothing", float(v)), row=4)

        opts = tk.Frame(ctl, bg="#ffffff")
        opts.grid(row=0, column=2, rowspan=5, sticky="nw", padx=20, pady=8)
        tk.Label(opts, text="映射模式", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").pack(anchor="w")
        self.var_mode = self.tk.StringVar(value=m.mode)
        for value, text in (("joystick", "摇杆（推荐）"), ("absolute", "绝对定位"), ("ninegrid", "九宫格按钮")):
            self.tk.Radiobutton(opts, text=text, value=value, variable=self.var_mode,
                                font=self.font, bg="#ffffff", fg="#1b1d21",
                                activebackground="#ffffff", anchor="w",
                                command=self._apply_mode).pack(anchor="w")

        tk.Label(opts, text="零点来源", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").pack(anchor="w", pady=(8, 0))
        self.var_origin = self.tk.StringVar(value=m.origin)
        for value, text in (("touch", "落指位置"), ("array", "阵列中心")):
            self.tk.Radiobutton(opts, text=text, value=value, variable=self.var_origin,
                                font=self.font, bg="#ffffff", fg="#1b1d21",
                                activebackground="#ffffff", anchor="w",
                                command=self._apply_mode).pack(anchor="w")

        self.var_recenter = self.tk.BooleanVar(value=m.edge_recenter)
        self.tk.Checkbutton(opts, text="边缘自动跟随（长距离移动）", variable=self.var_recenter,
                            font=self.font, bg="#ffffff", fg="#1b1d21",
                            activebackground="#ffffff", anchor="w",
                            command=lambda: setattr(m, "edge_recenter", bool(self.var_recenter.get()))
                            ).pack(anchor="w", pady=(8, 0))

        dev = tk.Frame(ctl, bg="#ffffff")
        dev.grid(row=0, column=3, rowspan=5, sticky="nw", padx=10, pady=8)
        tk.Label(dev, text="设备参数", font=self.font_bold, bg="#ffffff",
                 fg="#1b1d21").pack(anchor="w")
        self.var_threshold = self.tk.IntVar(value=int(self.cfg.device_threshold))
        row = self.tk.Frame(dev, bg="#ffffff")
        row.pack(anchor="w", pady=4)
        self.tk.Label(row, text="采集阈值", font=self.font, bg="#ffffff",
                      fg="#1b1d21").pack(side="left")
        self.tk.Spinbox(row, from_=0, to=255, width=5, font=self.font,
                        textvariable=self.var_threshold,
                        command=self._apply_threshold).pack(side="left", padx=6)
        self.tk.Button(dev, text="下发阈值", font=self.font, relief="flat", bg="#e8eaee",
                       padx=10, pady=3, command=self._apply_threshold).pack(anchor="w", pady=2)
        self.btn_reset_base = self.tk.Button(dev, text="清除零点校准", font=self.font, relief="flat",
                                             bg="#e8eaee", padx=10, pady=3,
                                             command=self._clear_baseline)
        self.btn_reset_base.pack(anchor="w", pady=2)

    def _slider(self, parent, label, lo, hi, init, setter, row: int):
        frame = self.tk.Frame(parent, bg="#ffffff")
        frame.grid(row=row, column=0, sticky="w", padx=(12, 0), pady=3)
        self.tk.Label(frame, text=label, font=self.font, bg="#ffffff", fg="#1b1d21",
                      width=18, anchor="w").pack(side="left")
        value_label = self.tk.Label(frame, text=f"{init:g}", font=self.font, bg="#ffffff",
                                    fg="#1b1d21", width=7)
        scale = self.tk.Scale(parent, from_=lo, to=hi, resolution=(hi - lo) / 200.0,
                              orient="horizontal", length=260, bg="#ffffff",
                              highlightthickness=0, showvalue=False, font=self.font)
        scale.set(init)
        scale.grid(row=row, column=1, sticky="w", pady=3)

        def on_change(v):
            value_label.config(text=f"{float(v):g}")
            try:
                setter(float(v))
            except Exception:  # noqa: BLE001
                pass

        scale.config(command=on_change)
        return scale

    # ------------------------------------------------------------------ #
    # 交互
    # ------------------------------------------------------------------ #
    def _apply_mode(self) -> None:
        self.cfg.mapping.mode = self.var_mode.get()
        self.cfg.mapping.origin = self.var_origin.get()
        self.session.status(f"映射模式: {self.cfg.mapping.mode} / 零点: {self.cfg.mapping.origin}")

    def _apply_threshold(self) -> None:
        try:
            value = int(self.var_threshold.get())
        except Exception:  # noqa: BLE001
            return
        self.cfg.filter.activate_abs = max(0.0, value * 0.6)
        self.session.set_threshold(value)

    def _clear_baseline(self) -> None:
        proc = self.session.controller.processor
        proc.baseline = [0.0] * (self.cfg.filter.rows * self.cfg.filter.cols)
        proc.start_calibration()
        proc.reset_ema()
        self.session.status("零点已清除，正在重新采集基线（请勿触碰）")

    def _describe_debug_command(self, data: bytes) -> str:
        if len(data) < 7 or data[0] != 0x98 or data[-1] != 0x81:
            return "" if not data else data.hex(" ").upper()
        cmd = data[3]
        names = {
            0x01: "连续采集(0x01)",
            0x02: "单次采集(0x02)",
            0x03: "停止采集(0x03)",
            0x07: "读电量(0x07)",
            0x08: "设阵列(0x08)",
            0x0B: "设备信息(0x0B)",
            0x0C: "设阈值(0x0C)",
            0x0D: "设上行通道(0x0D)",
        }
        return names.get(cmd, f"cmd=0x{cmd:02X}")

    def _append_debug(self, line: str) -> None:
        self.debug_log.config(state="normal")
        self.debug_log.insert("end", line + "\n")
        if int(self.debug_log.index("end-1c").split(".")[0]) > 120:
            self.debug_log.delete("1.0", "30.0")
        self.debug_log.see("end")
        self.debug_log.config(state="disabled")

    def _apply_debug_grid(self, values) -> None:
        if values is None:
            self._manual_debug_grid = None
            self._manual_debug_deadline = 0.0
            return
        self._manual_debug_grid = list(values)
        self._manual_debug_deadline = time.time() + 3.0

    def _on_debug(self, direction: str, data: bytes) -> None:
        try:
            payload = data.hex(" ").upper()
            label = "主机->设备" if direction == "TX" else "设备->主机"
            cmd_name = self._describe_debug_command(data)
            line = f"[{time.strftime('%H:%M:%S')}] {label} {cmd_name} {payload}"
            self._append_debug(line)

            if len(data) >= 7 and data[0] == P.FRAME_HEAD and data[-1] == P.FRAME_TAIL:
                frame, _, status = P.scan_frame(data, 0, {})
                if status == P.SCAN_OK and frame is not None:
                    if frame.cmd in (P.Command.CONTINUOUS, P.Command.SINGLE):
                        _, points = P.parse_acquire_payload(frame.data, self.cfg.filter.rows, self.cfg.filter.cols)
                        self._apply_debug_grid(points)
                        self._append_debug(f"[{time.strftime('%H:%M:%S')}] 采集数据已回填到 32x32 阵列，点数={len(points)}")
        except Exception:  # noqa: BLE001
            pass

    def _single_capture_debug(self) -> None:
        try:
            self._apply_debug_grid(None)
            self.session.send(P.cmd_single_acquire())
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 手动单次采集已发送：{P.cmd_single_acquire().hex(' ').upper()}")
        except Exception as exc:  # noqa: BLE001
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 手动单次采集失败：{exc}")

    def _open_serial_port(self) -> None:
        transport = self.session.transport
        if not isinstance(transport, UsbTransport):
            self.session.status("当前正在使用的不是 USB 串口通道，无法手动开关串口")
            return
        port = self.port_var.get().strip()
        if not port:
            self.session.status("请选择串口或手动输入 COM 端口")
            return
        try:
            transport.port = port
            self.cfg.usb.port = port
            if transport.is_open:
                transport.close()
            transport.open()
            self.session.status(f"串口已打开: {port}")
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 串口操作 打开 {port}")
        except Exception as exc:  # noqa: BLE001
            self.session.status(f"打开串口失败: {exc}")
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 串口操作 打开失败 {port}: {exc}")

    def _close_serial_port(self) -> None:
        transport = self.session.transport
        if not isinstance(transport, UsbTransport):
            self.session.status("当前正在使用的不是 USB 串口通道，无法手动开关串口")
            return
        try:
            if transport.is_open:
                transport.close()
            self.session.status("串口已关闭")
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 串口操作 关闭")
        except Exception as exc:  # noqa: BLE001
            self.session.status(f"关闭串口失败: {exc}")
            self._append_debug(f"[{time.strftime('%H:%M:%S')}] 串口操作 关闭失败: {exc}")

    def toggle_enabled(self) -> None:
        enabled = not self.session.controller.enabled
        self.session.set_enabled(enabled)
        self.btn_toggle.config(text="暂停控制" if enabled else "恢复控制")
        self._refresh()  # 立刻反馈，不等下一次定时刷新

    def emergency_stop(self) -> None:
        self.session.emergency_stop()
        self.session.set_enabled(False)
        self.btn_toggle.config(text="恢复控制")
        self._append_log("!! 急停：已松开所有按键并暂停控制")
        self._refresh()

    def recalibrate(self) -> None:
        self.session.recalibrate()

    def _install_hotkey(self) -> None:
        """全局 Esc 急停（需要 pynput；没有就退化为窗口内快捷键）。"""
        def _on_press(key):
            try:
                from pynput import keyboard  # type: ignore

                if key == keyboard.Key.esc:
                    self._estop_flag.set()
            except Exception:  # noqa: BLE001
                pass
            # 返回 None = 继续监听，这样 Esc 随时可用
            return None

        try:
            from pynput import keyboard  # type: ignore

            self._hotkey_listener = keyboard.Listener(on_press=_on_press)
            self._hotkey_listener.daemon = True
            self._hotkey_listener.start()
            self.session.status("全局 Esc 急停已启用")
        except Exception:  # noqa: BLE001
            self.root.bind("<Escape>", lambda _e: self.emergency_stop())
            self.session.status("未安装 pynput，Esc 急停仅在窗口聚焦时生效")

    # ------------------------------------------------------------------ #
    # 周期刷新
    # ------------------------------------------------------------------ #
    def _tick(self) -> None:
        try:
            if self._estop_flag.is_set():
                self._estop_flag.clear()
                self.emergency_stop()
            self._refresh()
        except Exception as exc:  # noqa: BLE001
            self.session.status(f"界面刷新异常: {exc}")
        self.root.after(40, self._tick)

    def _refresh(self) -> None:
        stats = self.session.controller.stats
        state = stats.last_state
        proc = self.session.controller.processor

        if not proc.calibrated:
            pct = int(proc.calibration_progress * 100)
            self.lbl_status.config(text=f"零点校准中 ... {pct}%（请勿触碰皮肤）", fg="#b7791f")
        else:
            color = "#0b8f6a" if self.session.controller.enabled else "#6b7280"
            text = "运行中" if self.session.controller.enabled else "已暂停"
            self.lbl_status.config(text=f"{text} · {self.session.transport.name} · "
                                        f"{self.session.controller.backend.name} · {stats.fps:.1f} fps",
                                   fg=color)

        if time.time() > self._manual_debug_deadline:
            self._manual_debug_grid = None
            self._manual_debug_deadline = 0.0

        grid_values = self._manual_debug_grid if self._manual_debug_grid is not None else (state.grid if state is not None else None)
        if grid_values is None:
            return
        peak = max(grid_values) if grid_values else 0.0
        for r in range(self.cfg.filter.rows):
            for c in range(self.cfg.filter.cols):
                value = grid_values[r * self.cfg.filter.cols + c]
                ratio = value / peak if peak > 0 else 0.0
                rect, text = self._cells[r][c]
                self.canvas.itemconfig(rect, fill=ramp_color(ratio))
                self.canvas.itemconfig(text, text=f"{int(value)}" if value >= 1 else "",
                                       fill="#ffffff" if ratio > 0.55 else "#6b7280")

        if state is None:
            return

        if state.centroid is not None:
            cx = (state.centroid[0] + 0.5) * CELL
            cy = (state.centroid[1] + 0.5) * CELL
            r = 9
            self.canvas.coords(self.marker, cx - r, cy - r, cx + r, cy + r)
            self.canvas.itemconfig(self.marker, state="normal")
        else:
            self.canvas.itemconfig(self.marker, state="hidden")

        for i, item in enumerate(self.contact_markers):
            if i < len(state.contacts):
                contact = state.contacts[i]
                x, y = proc.contact_xy(contact)
                cx, cy = (x + 0.5) * CELL, (y + 0.5) * CELL
                self.canvas.coords(item, cx - 5, cy - 5, cx + 5, cy + 5)
                self.canvas.itemconfig(item, state="normal")
            else:
                self.canvas.itemconfig(item, state="hidden")

        self.readouts["total"].config(text=f"{state.total:.0f}")
        self.readouts["peak"].config(text=f"{state.peak:.0f}")
        if state.centroid is not None:
            self.readouts["centroid"].config(text="%.2f, %.2f" % state.centroid)
        else:
            self.readouts["centroid"].config(text="-")
        self.readouts["contacts"].config(text=str(state.n_contacts))
        self.readouts["fps"].config(text=f"{stats.fps:.1f}")
        dragging = self.session.controller.recognizer.dragging
        scroll = self.session.controller.recognizer.in_scroll_mode
        if dragging:
            button = "左键按住 (拖拽)"
        elif scroll:
            button = "滚动"
        else:
            button = "—"
        self.readouts["button"].config(text=button)
        self.readouts["battery"].config(
            text=f"{self.session.info.battery}%" if self.session.info.battery is not None else "-"
        )
        self.readouts["dropped"].config(text=str(self.session.info.dropped_frames))

        # 新的事件追加到日志（用单调计数判断新增数量，避免 deque 淘汰后错位）
        total = stats.gesture_events
        if total > self._event_index:
            recent = list(stats.recent_events)
            new_count = min(len(recent), total - self._event_index)
            self._event_index = total
            for event in recent[-new_count:]:
                if event.kind is Gesture.SCROLL and abs(int(event.data.get("notches", 0))) == 0:
                    continue
                self._append_log(f"{time.strftime('%H:%M:%S')}  {event}")

    def _append_log(self, line: str) -> None:
        self.log.config(state="normal")
        self.log.insert("end", line + "\n")
        # 只保留最近 200 行
        if int(self.log.index("end-1c").split(".")[0]) > 200:
            self.log.delete("1.0", "50.0")
        self.log.see("end")
        self.log.config(state="disabled")

    # ------------------------------------------------------------------ #
    def _close(self) -> None:
        try:
            if self._hotkey_listener is not None:
                self._hotkey_listener.stop()
        except Exception:  # noqa: BLE001
            pass
        if self.on_close is not None:
            try:
                self.on_close()
            except Exception:  # noqa: BLE001
                pass
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()
