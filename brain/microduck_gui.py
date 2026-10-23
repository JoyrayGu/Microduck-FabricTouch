#!/usr/bin/env python3
"""Microduck 动作控制台：通过 SSH 远程控制树莓派上的鸭子动作。

依赖：
    pip install paramiko

运行：
    python microduck_gui.py

在目标电脑上填树莓派 IP，点「测试连接」，连上后即可切换动作。
"""

from __future__ import annotations

import queue
import socket
import threading
import time
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

try:
    import paramiko
except ImportError:  # pragma: no cover - GUI shows a hint instead of crashing
    raise SystemExit("缺少 paramiko，请先执行：pip install paramiko")

DEFAULT_HOST = "172.20.10.2"
DEFAULT_USER = "pi4"
DEFAULT_PASSWORD = "123456"

REMOTE_DIR = "/home/pi4/microduck-replica-next/tools/deployment_validation"
REMOTE_PYTHON = "/home/pi4/microduck_venv/bin/python"
REMOTE_SRC = "/home/pi4/microduck-replica-next/microduck/src"

POSES = (
    ("坐立", "sit_pose.json"),
    ("抬左脚", "lift_left_pose.json"),
    ("抬右脚", "lift_right_pose.json"),
    ("歪头", "tilt_head_pose.json"),
)

CONNECT_OK = "CONNECT_OK"
CONNECT_FAIL = "CONNECT_FAIL"


def discover_pi_hosts(timeout: float = 0.25, port: int = 22) -> list[str]:
    """Search the local LAN for likely Raspberry Pi hosts by checking SSH port 22."""
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        local_ip = probe.getsockname()[0]
        probe.close()
    except OSError:
        return []

    if not local_ip or local_ip.startswith("127."):
        return []

    prefix = ".".join(local_ip.split(".")[:-1])
    candidates = [f"{prefix}.{i}" for i in range(1, 255)]
    found: list[str] = []
    for candidate in candidates:
        if candidate == local_ip:
            continue
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                sock.settimeout(timeout)
                result = sock.connect_ex((candidate, port))
            if result == 0:
                found.append(candidate)
        except OSError:
            continue
    return found


def wrapped(script: str, arguments: str = "") -> str:
    """Build the remote command with the right cwd/python/PYTHONPATH."""
    return (
        f"cd {REMOTE_DIR} && export PYTHONPATH={REMOTE_SRC} && "
        f"{REMOTE_PYTHON} {script} {arguments}".rstrip()
    )


class MicroduckConsole(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Microduck 动作控制台")
        self.geometry("880x720")
        self.minsize(760, 620)

        self.client: paramiko.SSHClient | None = None
        self.channel = None
        self.worker: threading.Thread | None = None
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()

        self.host_var = tk.StringVar(value=DEFAULT_HOST)
        self.user_var = tk.StringVar(value=DEFAULT_USER)
        self.password_var = tk.StringVar(value=DEFAULT_PASSWORD)
        self.status_var = tk.StringVar(value="● 未连接")
        self.cycle_var = tk.StringVar(value="10")
        self.depth_var = tk.StringVar(value="1.0")
        self.duration_var = tk.StringVar(value="1.0")
        self.gate_var = tk.StringVar(value="9")

        self._build()
        self.after(100, self._pump_events)

    # ---------------------------------------------------------------- layout
    def _build(self) -> None:
        pad = {"padx": 8, "pady": 6}

        conn = ttk.LabelFrame(self, text="① 连接到树莓派")
        conn.pack(fill="x", **pad)

        ttk.Label(conn, text="IP 地址").grid(row=0, column=0, sticky="w", padx=6, pady=6)
        ttk.Entry(conn, textvariable=self.host_var, width=18).grid(row=0, column=1, pady=6)
        ttk.Label(conn, text="用户").grid(row=0, column=2, sticky="w", padx=6)
        ttk.Entry(conn, textvariable=self.user_var, width=10).grid(row=0, column=3, pady=6)
        ttk.Label(conn, text="密码").grid(row=0, column=4, sticky="w", padx=6)
        ttk.Entry(conn, textvariable=self.password_var, width=12, show="*").grid(
            row=0, column=5, pady=6
        )
        self.search_button = ttk.Button(conn, text="搜索 IP", command=self.on_search_host)
        self.search_button.grid(row=0, column=6, padx=6, pady=6)
        self.connect_button = ttk.Button(conn, text="测试连接", command=self.on_connect)
        self.connect_button.grid(row=0, column=7, padx=10, pady=6)
        ttk.Label(conn, textvariable=self.status_var, width=18).grid(
            row=0, column=8, sticky="w", padx=6
        )

        squat = ttk.LabelFrame(self, text="② 蹲下-站起")
        squat.pack(fill="x", **pad)
        self.squat_button = ttk.Button(
            squat, text="1 秒蹲起（满深度）", command=self.on_squat_stand, width=24
        )
        self.squat_button.grid(row=0, column=0, padx=6, pady=6, rowspan=2)
        ttk.Label(squat, text="深度").grid(row=0, column=1, sticky="e")
        ttk.Entry(squat, textvariable=self.depth_var, width=6).grid(row=0, column=2, padx=4)
        ttk.Label(squat, text="蹲/站时长(秒)").grid(row=0, column=3, sticky="e")
        ttk.Entry(squat, textvariable=self.duration_var, width=6).grid(
            row=0, column=4, padx=4
        )
        ttk.Label(squat, text="稳定判据(度)").grid(row=1, column=1, sticky="e")
        ttk.Entry(squat, textvariable=self.gate_var, width=6).grid(row=1, column=2, padx=4)
        ttk.Label(
            squat,
            text="深度 1.0 + 1 秒为实机验证配置；摆放偏斜时会出现底部后仰，判据可放宽到 9",
            foreground="#555",
        ).grid(row=1, column=3, columnspan=2, sticky="w")

        poses = ttk.LabelFrame(self, text="③ 坐立系列（单动作）")
        poses.pack(fill="x", **pad)
        for index, (label, filename) in enumerate(POSES):
            ttk.Button(
                poses,
                text=label,
                width=10,
                command=lambda f=filename, n=label: self.on_pose(f, n),
            ).grid(row=0, column=index, padx=6, pady=6)
        ttk.Label(
            poses,
            text="（自动插入回「坐立」的过渡，避免两条腿同时改变姿态）",
            foreground="#555",
        ).grid(row=1, column=0, columnspan=4, sticky="w", padx=6)

        loop = ttk.LabelFrame(self, text="④ 坐立系列循环播放")
        loop.pack(fill="x", **pad)
        ttk.Label(loop, text="一圈秒数").grid(row=0, column=0, sticky="e", padx=6, pady=6)
        ttk.Entry(loop, textvariable=self.cycle_var, width=6).grid(row=0, column=1, padx=4)
        ttk.Label(loop, text="（10 = 原速，5 = 加快一倍）").grid(
            row=0, column=2, sticky="w"
        )
        self.loop_button = ttk.Button(
            loop, text="开始循环", command=lambda: self.on_loop(0), width=14
        )
        self.loop_button.grid(row=0, column=3, padx=6)
        self.once_button = ttk.Button(
            loop, text="只跑一圈", command=lambda: self.on_loop(1), width=12
        )
        self.once_button.grid(row=0, column=4, padx=6)
        self.stop_button = ttk.Button(
            loop, text="停止", command=self.on_stop, width=10, state="disabled"
        )
        self.stop_button.grid(row=0, column=5, padx=6)

        misc = ttk.LabelFrame(self, text="⑤ 其他")
        misc.pack(fill="x", **pad)
        self.status_button = ttk.Button(
            misc, text="查看舵机状态", command=self.on_servo_status, width=16
        )
        self.status_button.grid(row=0, column=0, padx=6, pady=6)
        self.stand_button = ttk.Button(
            misc, text="回到标定站姿", command=self.on_return_to_stand, width=16
        )
        self.stand_button.grid(row=0, column=1, padx=6, pady=6)

        log_frame = ttk.LabelFrame(self, text="输出日志")
        log_frame.pack(fill="both", expand=True, **pad)
        self.log = ScrolledText(log_frame, height=20, wrap="word", state="disabled")
        self.log.pack(fill="both", expand=True, padx=6, pady=6)

    # ------------------------------------------------------------- utilities
    def log_line(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        for button in (
            self.connect_button,
            self.search_button,
            self.squat_button,
            self.status_button,
            self.stand_button,
        ):
            button.configure(state=state)
        for child in self.winfo_children():
            for sub in child.winfo_children():
                if isinstance(sub, ttk.Button) and sub.cget("text") in {
                    label for label, _ in POSES
                }:
                    sub.configure(state=state)
        if busy:
            self.loop_button.configure(state="disabled")
            self.once_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
        else:
            self.loop_button.configure(state="normal")
            self.once_button.configure(state="normal")
            self.stop_button.configure(state="disabled")

    # ------------------------------------------------------------ connection
    def on_search_host(self) -> None:
        self.status_var.set("● 搜索中…")
        self.log_line("正在扫描局域网中可能的树莓派 IP…")

        def job() -> None:
            hosts = discover_pi_hosts()
            if not hosts:
                self.events.put(("INFO", "未发现可用的 SSH 主机，建议手动填写 IP。"))
                self.events.put(("INFO", f"当前默认 IP：{self.host_var.get().strip() or DEFAULT_HOST}"))
                self.events.put(("INFO", "可尝试先确认本机与树莓派在同一网段，例如 192.168.1.x / 10.0.0.x"))
                return

            self.events.put(("INFO", f"已发现候选 IP：{', '.join(hosts[:8])}"))
            self.host_var.set(hosts[0])
            self.events.put(("INFO", f"已自动填入首个候选 IP：{hosts[0]}"))
            self.status_var.set("● 可用 IP 已填入")

        threading.Thread(target=job, daemon=True).start()

    def on_connect(self) -> None:
        self.status_var.set("● 连接中…")
        self.set_busy(True)
        host = self.host_var.get().strip() or DEFAULT_HOST
        user = self.user_var.get().strip()
        password = self.password_var.get()

        def job() -> None:
            candidate_hosts: list[str] = []
            if host:
                candidate_hosts.append(host)

            discovered = discover_pi_hosts()
            for found in discovered:
                if found not in candidate_hosts:
                    candidate_hosts.append(found)

            last_error: Exception | None = None
            for candidate in candidate_hosts:
                try:
                    self.events.put(("INFO", f"尝试连接 {candidate}…"))
                    client = paramiko.SSHClient()
                    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
                    client.connect(
                        candidate,
                        username=user,
                        password=password,
                        look_for_keys=False,
                        allow_agent=False,
                        timeout=8,
                    )
                    self.client = client
                    self.host_var.set(candidate)
                    probe = (
                        "echo HOST=$(hostname); "
                        "ls -l /dev/ttyACM0 2>&1; "
                        "echo SERVOS; "
                        f"cd {REMOTE_DIR} && export PYTHONPATH={REMOTE_SRC} && "
                        f"{REMOTE_PYTHON} servo_status.py | tail -n 3"
                    )
                    _, stdout, _ = client.exec_command(probe, timeout=30)
                    text = stdout.read().decode("utf-8", "replace")
                    self.events.put((CONNECT_OK, text))
                    return
                except Exception as exc:  # noqa: BLE001
                    last_error = exc
                    self.events.put(("INFO", f"{candidate} 连接失败：{type(exc).__name__}: {exc}"))

            if last_error is not None:
                self.events.put((CONNECT_FAIL, f"{type(last_error).__name__}: {last_error}"))
            else:
                self.events.put((CONNECT_FAIL, "未找到可用的树莓派 IP，先点击“搜索 IP”或手动填入 IP。"))

        threading.Thread(target=job, daemon=True).start()

    # -------------------------------------------------------------- commands
    def run_remote(self, label: str, command: str) -> None:
        if self.client is None:
            self.log_line("请先点「测试连接」建立连接。")
            return
        if self.worker is not None and self.worker.is_alive():
            self.log_line("上一条指令还没结束，请稍等或先点「停止」。")
            return
        self.set_busy(True)
        self.log_line(f"\n>>> {label}")
        self.log_line(f"    {command}")

        def job() -> None:
            try:
                transport = self.client.get_transport()
                channel = transport.open_session()
                channel.set_combine_stderr(True)
                channel.exec_command(command)
                self.channel = channel
                started = time.time()
                while True:
                    while channel.recv_ready():
                        chunk = channel.recv(4096).decode("utf-8", "replace")
                        self.events.put(("OUT", chunk))
                    if channel.exit_status_ready() and not channel.recv_ready():
                        break
                    if time.time() - started > 900:
                        self.events.put(("OUT", "\n[超时 900 秒，已放弃等待]\n"))
                        break
                    time.sleep(0.05)
                code = channel.recv_exit_status() if channel.exit_status_ready() else -1
                self.events.put(("DONE", f"<<< {label} 结束（退出码 {code}）"))
            except Exception as exc:  # noqa: BLE001
                self.events.put(("DONE", f"<<< {label} 失败：{type(exc).__name__}: {exc}"))
            finally:
                self.channel = None

        self.worker = threading.Thread(target=job, daemon=True)
        self.worker.start()

    def _require_connection(self) -> bool:
        if self.client is None:
            self.log_line("请先点「测试连接」建立连接。")
            return False
        return True

    def on_squat_stand(self) -> None:
        if not self._require_connection():
            return
        arguments = (
            f"--mode run --preset squat-stand "
            f"--squat-depth {self.depth_var.get().strip() or '1.0'} "
            f"--squat-time {self.duration_var.get().strip() or '1.0'} "
            f"--stand-time {self.duration_var.get().strip() or '1.0'} "
            f"--squat-gate-pitch {self.gate_var.get().strip() or '9'}"
        )
        self.run_remote("1 秒蹲起", wrapped("squat_stand.py", arguments))

    def on_pose(self, filename: str, label: str) -> None:
        if not self._require_connection():
            return
        # Always go through 坐立 so the two legs never change at the same time.
        if filename == "sit_pose.json":
            command = wrapped("move_to_pose.py", "--pose sit_pose.json --duration 1.0")
        else:
            command = (
                "cd %s && export PYTHONPATH=%s && %s move_to_pose.py "
                "--pose %s --duration 1.0 && sleep 0.8 && "
                "%s move_to_pose.py --pose sit_pose.json --duration 1.0"
                % (
                    REMOTE_DIR,
                    REMOTE_SRC,
                    REMOTE_PYTHON,
                    filename,
                    REMOTE_PYTHON,
                )
            )
        self.run_remote(f"坐立系列：{label}", command)

    def on_loop(self, cycles: int) -> None:
        if not self._require_connection():
            return
        try:
            cycle = float(self.cycle_var.get().strip())
        except ValueError:
            self.log_line("「一圈秒数」要填数字。")
            return
        if cycle < 1.5:
            self.log_line("一圈至少 1.5 秒，否则舵机跟不上。")
            return
        # Scale the internals with the cycle so the shape of the motion is kept.
        ease = round(cycle * 0.09, 3)
        hold_action = round(cycle * 0.06, 3)
        hold_return = round(cycle * 0.02, 3)
        arguments = (
            f"--cycle {cycle} --ease {ease} "
            f"--hold-action {hold_action} --hold-return {hold_return} "
            f"--cycles {cycles}"
        )
        label = "坐立系列循环（%s，一圈 %.1f 秒）" % (
            "无限" if cycles == 0 else f"{cycles} 圈",
            cycle,
        )
        self.run_remote(label, wrapped("pose_loop.py", arguments))

    def on_stop(self) -> None:
        self.log_line("\n>>> 请求停止")
        if self.channel is not None:
            try:
                self.channel.close()
            except Exception:  # noqa: BLE001
                pass
        if self.client is not None:
            transport = self.client.get_transport()
            channel = transport.open_session()
            channel.exec_command("pkill -f pose_loop.py; pkill -f squat_stand.py; echo KILLED")
            channel.recv_exit_status()
        self.log_line("    已发送停止指令")
        self.set_busy(False)

    def on_servo_status(self) -> None:
        if not self._require_connection():
            return
        self.run_remote("舵机状态", wrapped("servo_status.py"))

    def on_return_to_stand(self) -> None:
        if not self._require_connection():
            return
        self.run_remote("回到标定站姿", wrapped("return_to_stand.py"))

    # --------------------------------------------------------------- events
    def _pump_events(self) -> None:
        while True:
            try:
                kind, payload = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "OUT":
                self.log_line(payload.rstrip("\n"))
            elif kind == "INFO":
                self.log_line(payload)
            elif kind == CONNECT_OK:
                self.status_var.set("● 已连接")
                self.set_busy(False)
                self.log_line("连接成功：")
                self.log_line(payload.strip() or "(无输出)")
            elif kind == CONNECT_FAIL:
                self.status_var.set("● 连接失败")
                self.client = None
                self.set_busy(False)
                self.log_line(f"连接失败：{payload}")
            elif kind == "DONE":
                self.set_busy(False)
                self.log_line(payload)
        self.after(100, self._pump_events)


if __name__ == "__main__":
    MicroduckConsole().mainloop()