#!/usr/bin/env python3
"""@file run.py
@brief 项目入口脚本，负责 CLI 参数解析、设备选择与 GUI 启动。
@details
    该模块提供命令行入口，用于选择 USB / BLE / Mock 传输、加载配置、
    连接硬件、启动会话以及打开图形界面；同时也内置自检与设备枚举功能。

用电子触觉皮肤控制鼠标 —— 程序入口。

**默认走 USB**（USB 虚拟串口），因为无需蓝牙配对、带宽更大、不丢包::

    python run.py                 # 自动找串口 + 自动试探波特率，然后接管鼠标
    python run.py --port COM5     # 已知端口时直接指定，跳过自动试探
    python run.py --gui           # 带实时压力热力图界面

扫一下有哪些设备可用::

    python run.py --list

没有硬件，用模拟数据体验完整功能::

    python run.py --mock --gui

切回蓝牙（设备自身出厂默认通道）::

    python run.py --ble --name TT-Skin --gui

只观察、不真的动鼠标（安全调试）::

    python run.py --dry-run --viz

把皮肤当作「机器人的触觉面板」：识别手势后通过 TCP 发给 brain（RK3566）::

    python run.py --robot --brain-host 192.168.1.50 --brain-port 9000
    python run.py --robot --mock --brain-dry-run      # 不联网，只看识别到什么手势

    --robot 模式下**不会**注入任何鼠标事件，纯手势 -> 网络；默认 --brain-dry-run，
    只有显式给出主机且不加 --brain-dry-run 时才会真正发送。

调好参数后存下来::

    python run.py --gui --save-config my.json
    python run.py --config my.json --gui

本程序把「轻点=左键、双击=双击、长按=右键、重压拖拽=按住左键、
双指平移=滚轮」都做了，全部参数都能在界面上或命令行里调。

关于 USB 的两个坑（协议文档里写明的）：
  * 0x0D 指令负责切换上行通道，**设备出厂默认蓝牙，且掉电重启后自动恢复
    蓝牙**，所以每次连接程序都会重发 0x0D=0x00；
  * USB 模式下单次采集(0x02)的返回是 **ASCII 明文**（32x32、单位 kPa、
    3 位小数），不是二进制帧；程序会自动识别、换算并重采样到 --rows/--cols。
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from typing import Any, Dict, List, Optional

# 允许直接 python run.py（不需要 pip install -e .）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tactile_mouse import protocol as P  # noqa: E402
from tactile_mouse.config import AppConfig  # noqa: E402
from tactile_mouse.session import Session, list_usb_ports, make_transport  # noqa: E402
from tactile_mouse.transport import TransportError, list_ble_devices  # noqa: E402

BANNER = r"""
 __  __             _              ___  _   _
|  \/  | ___  _   _| |_ ___        / _ \| | | | ___
| |\/| |/ _ \| | | | __/ __|_____ | | | | |_| |/ _ \
| |  | | (_) | |_| | |_\__ \_____|| |_| |  _  |  __/
|_|  |_|\___/ \__,_|\__|___/       \___/|_| |_|\___|
    电子触觉皮肤 -> 鼠标 (采集设备协议 V5.0 · USB 默认)
"""

#: 自检用的三种通道形态，覆盖真实会遇到的情况
SELFTEST_SCENARIOS = (
    ("蓝牙 二进制(含噪声)", "ble"),
    ("USB  二进制帧", "usb"),
    ("USB  ASCII 明文块", "usb-ascii"),
)


# --------------------------------------------------------------------------- #
# 参数
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="run.py",
        description="用电子触觉皮肤（压力阵列）控制鼠标移动与点击（默认 USB 通道）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    g = p.add_argument_group("设备连接")
    g.add_argument("--transport", choices=["usb", "ble", "mock"], default=None,
                   help="上行通道：usb（默认）| ble | mock")
    g.add_argument("--usb", action="store_true", help="使用 USB 通道（默认，等同于 --transport usb）")
    g.add_argument("--ble", action="store_true", help="改用蓝牙通道（等同于 --transport ble）")
    g.add_argument("--list", action="store_true", help="列出可用的 USB 串口与蓝牙设备后退出")
    g.add_argument("--list-usb", action="store_true", help="只列 USB 串口后退出")
    g.add_argument("--list-ble", action="store_true", help="只扫蓝牙后退出")
    g.add_argument("--port", help="USB 串口，例如 COM5（不填则自动选择）")
    g.add_argument("--baud", type=int, default=None,
                   help="USB 波特率（不填则自动试探；常见的 115200 / 921600）")
    g.add_argument("--usb-port-hint", help="自动选口时的关键字，例如 CH340 / Silicon")
    g.add_argument("--usb-format", choices=["auto", "binary", "ascii"], default=None,
                   help="USB 数据形态（默认 auto 自动识别；设备只发二进制帧时可设 binary）")
    g.add_argument("--usb-scale", type=float, default=None,
                   help="USB 明文(kPa) -> 内部压力单位 的换算倍数（默认 100）")
    g.add_argument("--usb-allow-partial", action="store_true",
                   help="允许残缺的明文块（缺行按 0 补齐）。默认丢弃，宁缺毋滥")
    g.add_argument("--usb-no-dtr", action="store_true",
                   help="不置位 DTR（一连上设备就反复重启时用）")
    g.add_argument("--name", help="设备蓝牙名称（模糊匹配，例如 TT-Skin）")
    g.add_argument("--address", help="设备蓝牙 MAC/UUID，例如 AA:BB:CC:DD:EE:FF")
    g.add_argument("--mock", action="store_true", help="使用模拟数据源（无需硬件）")
    g.add_argument("--mock-usb", action="store_true",
                   help="模拟数据源按 USB 二进制帧输出（无前导噪声、不分片）")
    g.add_argument("--mock-usb-ascii", action="store_true",
                   help="模拟数据源按 USB 明文模式输出（验证 ASCII 解析链路）")
    g.add_argument("--pattern", default="auto",
                   choices=["auto", "circle", "tap", "scroll", "drag", "center", "random"],
                   help="模拟数据源的动作（默认 auto 每 12 秒轮换）")
    g.add_argument("--scan-timeout", type=float, default=8.0, help="蓝牙扫描时长（秒）")

    g2 = p.add_argument_group("采集参数")
    g2.add_argument("--rows", type=int, default=P.DEFAULT_ROWS, help="阵列行数（默认 32）")
    g2.add_argument("--cols", type=int, default=P.DEFAULT_COLS, help="阵列列数（默认 32）")
    g2.add_argument("--threshold", type=int, default=None,
                    help="设备采集阈值 0~255（0x0C 指令，默认沿用配置值 100）")
    g2.add_argument("--acquire", choices=["continuous", "single"], default="continuous",
                    help="continuous=连续采集 0x01（默认）；single=单次采集轮询 0x02"
                         "（USB 下该模式返回 ASCII 明文，单位 kPa）")
    g2.add_argument("--poll-hz", type=float, default=30.0, help="单次采集模式的轮询频率")
    g2.add_argument("--no-calibrate", action="store_true", help="跳过开机零点校准")

    g3 = p.add_argument_group("映射与控制")
    g3.add_argument("--mode", choices=["joystick", "absolute", "ninegrid"], help="映射模式")
    g3.add_argument("--origin", choices=["touch", "array"], help="摇杆零点来源（默认 touch）")
    g3.add_argument("--speed", type=float, help="最大光标速度（像素/秒）")
    g3.add_argument("--deadzone", type=float, help="死区 0~1")
    g3.add_argument("--gamma", type=float, help="响应曲线指数")
    g3.add_argument("--flip-x", action="store_true", help="左右反相")
    g3.add_argument("--flip-y", action="store_true", help="上下反相")
    g3.add_argument("--edge-recenter", action="store_true", help="开启边缘自动跟随")
    g3.add_argument("--backend", default="auto",
                    choices=["auto", "sendinput", "pynput", "pyautogui", "null"],
                    help="鼠标后端（Windows 默认 sendinput）")
    g3.add_argument("--raw-move", action="store_true",
                    help="绕过系统鼠标加速（绝对设置坐标，更线性但略「重」）")
    g3.add_argument("--dry-run", action="store_true",
                    help="不真的注入鼠标事件，只打印动作（安全调试）")

    g4 = p.add_argument_group("机器人模式（skin -> brain）")
    g4.add_argument("--robot", action="store_true",
                    help="机器人模式：手势通过 TCP 发给 brain，注入零次鼠标事件")
    g4.add_argument("--brain-host", default=None,
                    help="brain 的地址（RK3566，默认取配置 robot_link.host，如 192.168.1.50）")
    g4.add_argument("--brain-port", type=int, default=None,
                    help="brain 命令服务器端口（默认 9000）")
    g4.add_argument("--brain-dry-run", action="store_true",
                    help="机器人模式只识别、不发送（安全预览，默认即为该行为）")
    g4.add_argument("--trace", default=None, metavar="CSV",
                    help="把机器人模式每帧的 active/peak/centroid 写入 CSV，"
                         "用于离线复现识别过程（不影响识别本身）")

    g5 = p.add_argument_group("界面与配置")
    g5.add_argument("--gui", action="store_true", help="打开图形界面（需要 tkinter）")
    g5.add_argument("--viz", action="store_true", help="控制台模式下定期打印压力字符图")
    g5.add_argument("--duration", type=float, default=0.0,
                    help="运行指定秒数后自动退出（0 = 一直运行）")
    g5.add_argument("--config", help="从 JSON 文件加载参数")
    g5.add_argument("--save-config", help="退出时把当前参数保存到 JSON 文件")
    g5.add_argument("--no-hotkey", action="store_true", help="关闭全局 Esc 急停")
    g5.add_argument("--selftest", action="store_true",
                    help="自检：用模拟数据跑通整条链路（含 USB 明文解析）并打印统计后退出")
    return p


def resolve_mock_mode(args: argparse.Namespace) -> str:
    """模拟设备扮演的角色：ble（默认，最像蓝牙）| usb | usb-ascii。"""
    if args.mock_usb_ascii:
        return "usb-ascii"
    if args.mock_usb:
        return "usb"
    return "ble"


def resolve_kind(args: argparse.Namespace, cfg: AppConfig) -> str:
    """决定用哪条通道：命令行显式指定 > 配置 > 由蓝牙参数推断 > 默认 USB。"""
    if args.mock:
        return "mock"
    if args.ble:
        return "ble"
    if args.usb:
        return "usb"
    if args.transport:
        return args.transport
    if args.port:
        return "usb"
    if args.name or args.address:
        return "ble"  # 给了蓝牙名/地址，显然是要用蓝牙
    return cfg.transport_kind or "usb"


def apply_overrides(cfg: AppConfig, args: argparse.Namespace) -> AppConfig:
    cfg.filter.rows, cfg.filter.cols = args.rows, args.cols
    cfg.device_array = (args.rows, args.cols)
    if args.threshold is not None:
        cfg.device_threshold = args.threshold
        # 设备阈值与「激活阈值」语义一致（低于它设备直接输出 0），一并同步
        cfg.filter.activate_abs = max(0.0, args.threshold * 0.6)
    cfg.acquire = args.acquire
    cfg.single_poll_hz = args.poll_hz
    if args.no_calibrate:
        cfg.filter.baseline_frames = 0
    # USB 通道
    u = cfg.usb
    if args.port:
        u.port = args.port
    if args.baud:
        u.baudrate = args.baud
        u.autobaud = False
    if args.usb_port_hint:
        u.port_hint = args.usb_port_hint
    if args.usb_format:
        u.format = args.usb_format
    if args.usb_scale is not None:
        u.ascii_scale = args.usb_scale
    if args.usb_allow_partial:
        u.allow_partial = True
    if args.usb_no_dtr:
        u.dtr = False
    m = cfg.mapping
    if args.mode:
        m.mode = args.mode
    if args.origin:
        m.origin = args.origin
    if args.speed is not None:
        m.max_speed_px_s = args.speed
    if args.deadzone is not None:
        m.deadzone = args.deadzone
    if args.gamma is not None:
        m.gamma = args.gamma
    if args.flip_x:
        m.invert_x = True
    if args.flip_y:
        m.invert_y = True
    if args.edge_recenter:
        m.edge_recenter = True
    return cfg


# --------------------------------------------------------------------------- #
# 子流程
# --------------------------------------------------------------------------- #
def do_list_usb() -> int:
    print("=== USB 串口（默认通道）===")
    try:
        ports = list_usb_ports()
    except TransportError as exc:
        print(f"  无法枚举串口: {exc}")
        print("  解决：pip install pyserial")
        return 2
    if not ports:
        print("  没有发现任何串口。请确认皮肤已用 USB 线接好（有些线只能充电）。")
        return 1
    print(f"  {'端口':<10}{'类型':<24}描述")
    print("  " + "-" * 70)
    for p in ports:
        print(f"  {p.device:<10}{p.bridge:<24}{p.description or '-'}")
    print("\n  直接运行 python run.py 即可自动选中第一个（USB 虚拟串口优先）。")
    return 0


def do_list_ble(timeout: float, name_filter: Optional[str]) -> int:
    print(f"=== 蓝牙设备（扫描 {timeout:.0f} 秒，可选通道）===")
    try:
        devices = list_ble_devices(timeout, name_filter)
    except TransportError as exc:
        print(f"  扫描失败: {exc}")
        print("  提示：Windows 需要蓝牙已开启；Linux 需 bluez。")
        return 2
    if not devices:
        print("  没有发现设备。请确认皮肤已上电、蓝牙未被其它程序占用。")
        return 1
    print(f"  {'名称':<28}{'地址':<40}RSSI")
    print("  " + "-" * 74)
    for name, address, rssi in devices:
        print(f"  {name:<28}{address:<40}{rssi} dBm")
    return 0


def do_list(args: argparse.Namespace) -> int:
    codes = []
    if args.list_ble:
        return do_list_ble(args.scan_timeout, args.name)
    if args.list_usb:
        return do_list_usb()
    codes.append(do_list_usb())
    print()
    codes.append(do_list_ble(args.scan_timeout, args.name))
    return 0 if 0 in codes else 1


def _run_scenario(cfg: AppConfig, transport, label: str, seconds: float = 4.0):
    """让一条模拟通道跑满整条链路，返回 (是否通过, 摘要字符串)。"""
    from tactile_mouse.mouse_backend import NullBackend

    backend = NullBackend()
    session = Session(cfg, transport=transport, backend=backend, on_status=lambda m: None)
    session.start()
    start = time.perf_counter()
    while time.perf_counter() - start < seconds:
        time.sleep(0.4)
    session.stop()

    actions = backend.actions
    moves = [a for a in actions if a.startswith("move_rel")]
    clicks = [a for a in actions if a.startswith("down(")]
    charts = [a for a in actions if a.startswith("scroll")]
    frames = session.info.data_frames
    processed = session.controller.stats.frames
    ok = frames > 30 and processed > 10 and (moves or clicks)
    mode = getattr(transport, "channel_mode", None) or "-"
    summary = (
        f"采集帧={frames:5d} 处理={processed:5d} 丢帧={session.info.dropped_frames:3d} "
        f"移动={len(moves):5d} 点击={len(clicks):3d} 滚动={len(charts):3d} "
        f"通道={transport.name} 形态={mode}"
    )
    return ok, summary


def do_selftest(cfg: AppConfig) -> int:
    """用模拟设备跑通「协议 -> 特征 -> 手势 -> 鼠标」，覆盖蓝牙与 USB 两种形态。"""
    print("== 自检：模拟设备 -> 协议解析 -> 手势 -> 鼠标（空后端，不会真的动光标）==")
    cfg.acquire = "continuous"
    cfg.filter.baseline_frames = 0  # 模拟数据没有零点漂移，跳过校准

    all_ok = True
    for label, mock_mode in SELFTEST_SCENARIOS:
        transport = make_transport(
            cfg, kind="mock", pattern="auto", mock_mode=mock_mode,
            on_status=lambda m: None,
        )
        ok, summary = _run_scenario(cfg, transport, label)
        all_ok = all_ok and ok
        print(f"  [{'通过' if ok else '失败'}] {label:<20} {summary}")

    # 单独校验「USB 明文块 -> 合成采集帧」的数值正确性
    from tactile_mouse.transport import UsbStreamDecoder

    size = P.ASCII_GRID_SIZE
    text = "\n".join(
        " ".join(f"{0.5 + 0.01 * (r + c):7.3f}" for c in range(size)) for r in range(size)
    ) + "\n"
    out: List[bytes] = []
    decoder = UsbStreamDecoder(
        emit=out.append, rows=cfg.filter.rows, cols=cfg.filter.cols,
        scale=cfg.usb.ascii_scale, on_status=lambda m: None,
    )
    decoder.feed(text.encode("ascii"))
    frames = list(P.iter_frames(b"".join(out)))
    ascii_ok = False
    detail = "没有解出采集帧"
    if frames:
        _no, points = P.parse_acquire_payload(
            frames[0].data, cfg.filter.rows, cfg.filter.cols
        )
        ascii_ok = len(points) == cfg.filter.rows * cfg.filter.cols and sum(points) > 0
        detail = (f"{frames[0].cmd:#04x} 帧, {len(points)} 点, "
                  f"总压力={sum(points):.0f}")
    all_ok = all_ok and ascii_ok
    print(f"  [{ '通过' if ascii_ok else '失败' }] {'USB ASCII 解码':<18} {detail}")

    print("== 自检", "全部通过 ==" if all_ok else "存在失败 ==")
    return 0 if all_ok else 1


def run_robot_console(session, cfg: AppConfig, args: argparse.Namespace, stop_flag) -> int:
    """机器人模式无界面运行：持续打印触觉状态和识别到的手势。"""
    start = time.perf_counter()

    def _on_gesture(event):
        direction = f" {event.direction}" if getattr(event, "direction", None) else ""
        print(f"\n[手势] {event.gesture.value}{direction} "
              f"contact={event.contact_mode.value} motion={event.motion_mode.value} "
              f"moving={event.moving_contacts} stationary={event.stationary_contacts} "
              f"conf={getattr(event, 'confidence', 1.0):.2f}", flush=True)

    session.on_gesture = _on_gesture
    last_print = 0.0
    target = (
        "dry-run（不发往 brain）"
        if session.dry_run
        else f"{cfg.robot_link.host}:{cfg.robot_link.port}"
    )
    print(f"机器人模式已启动 -> {target}。Ctrl+C 退出。")
    while not stop_flag["stop"]:
        time.sleep(0.1)
        now = time.perf_counter()
        if args.duration and now - start >= args.duration:
            break
        if now - last_print < 0.25:
            continue
        last_print = now
        state = session.last_state
        if state is None:
            continue
        centroid = "%.1f,%.1f" % state.centroid if state.centroid else "--,--"
        recognizer = session.recognizer
        line = (
            f"\r[状态] active={int(state.active)} contacts={state.n_contacts} "
            f"motion={recognizer.motion_mode.value} "
            f"moving={recognizer.moving_contacts} stationary={recognizer.stationary_contacts} "
            f"peak={state.peak:.0f} total={state.total:.0f} centroid={centroid}"
        )
        print(line.ljust(150), end="", flush=True)
    print()
    return 0


def run_console(session: Session, cfg: AppConfig, args: argparse.Namespace, stop_flag) -> int:
    """无界面模式：定期打印状态，Ctrl+C 退出。"""
    last_print = 0.0
    while not stop_flag["stop"]:
        time.sleep(0.1)
        now = time.perf_counter()
        if args.duration and now - start >= args.duration:
            break
        if now - last_print < 0.5:
            continue
        last_print = now
        stats = session.controller.stats
        state = stats.last_state
        if state is None:
            continue
        proc = session.controller.processor
        if not proc.calibrated:
            print(f"\r零点校准中 {proc.calibration_progress * 100:5.1f}%  ", end="", flush=True)
            continue
        centroid = "%.2f,%.2f" % state.centroid if state.centroid else "  --  "
        vx, vy = stats.velocity
        line = (f"\r总压力 {state.total:7.0f} | 峰值 {state.peak:6.0f} | 质心 {centroid} | "
                f"接触 {state.n_contacts} | 速度 {vx:7.0f},{vy:7.0f} | {stats.fps:5.1f} fps | "
                f"丢帧 {session.info.dropped_frames}")
        print(line.ljust(120), end="", flush=True)
        if args.viz:
            art = state.ascii_art(proc.cols, proc.rows)
            print()
            for row in art.splitlines():
                print("    " + " ".join(row))
    print()
    return 0


def run_gui(session: Session, cfg: AppConfig, args: argparse.Namespace) -> int:
    try:
        import tkinter  # noqa: F401
    except ImportError:
        print("当前 Python 没有 tkinter，无法打开图形界面。")
        print("解决办法：改用自带 tkinter 的 Python（如 python.org 官方安装包或 miniconda），")
        print("或者去掉 --gui 使用控制台模式。")
        return 3
    from tactile_mouse.gui import TactileMouseGUI

    gui = TactileMouseGUI(
        session,
        cfg,
        enable_hotkey=not args.no_hotkey,
        title=f"电子触觉皮肤 · 鼠标控制器 [{session.transport.name}]",
    )
    gui.run()
    return 0


# --------------------------------------------------------------------------- #
def _print_reply(reply: Dict[str, Any]) -> None:
    """打印 brain 的应答，让被拒绝的事件当场可见。

    brain 是最终安全边界：它会因过期、重复、忙、未知手势等原因拒收事件。
    没有这个回显时，这些拒绝只出现在 Pi 的日志里，Windows 侧看起来就是
    「什么都没发生」，排查成本很高。

    解析失败（收到半包或非 JSON）时不显示 ``reason=None``——那会误导成
    「brain 返回了空 reason」。此时直接把原始文本打出来。
    """
    if "raw" in reply and "accepted" not in reply:
        print(f"[brain] UNPARSED reply={reply['raw']!r}", flush=True)
        return
    ok = reply.get("accepted")
    eid = reply.get("event_id") or "-"
    if ok:
        print(f"[brain] ACCEPTED action={reply.get('action')} "
              f"reason={reply.get('reason', 'ok')} event_id={eid}", flush=True)
    else:
        print(f"[brain] REJECTED reason={reply.get('reason')} "
              f"event_id={eid}", flush=True)


def _run_robot_mode(cfg: AppConfig, args: argparse.Namespace, kind: str, mock_mode: str) -> int:
    """--robot：识别手势并通过 TCP 发给 brain。绝不注入鼠标事件。"""
    from tactile_mouse.robot_link import RobotLink
    from tactile_mouse.robot_session import RobotSession

    if args.brain_host:
        cfg.robot_link.host = args.brain_host
    if args.brain_port:
        cfg.robot_link.port = args.brain_port

    # 默认 dry-run：只有显式给了主机且未加 --brain-dry-run 才真正联网发送。
    send = bool(args.brain_host) and not args.brain_dry_run

    link = RobotLink(cfg.robot_link, on_reply=_print_reply) if send else None

    trace_writer = None
    trace_cb = None
    if getattr(args, "trace", None):
        import csv

        fh = open(args.trace, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(["t", "active", "peak", "total", "cx", "cy"])

        def trace_cb(state) -> None:
            cx, cy = state.centroid if state.centroid else ("", "")
            writer.writerow([
                f"{state.timestamp:.4f}",
                int(bool(state.active)),
                f"{state.peak:.1f}",
                f"{state.total:.1f}",
                cx, cy,
            ])

        trace_writer = fh
        print(f"逐帧诊断已开启 -> {args.trace}")

    session = RobotSession(
        cfg,
        link=link,
        dry_run=not send,
        on_status=lambda msg: print(f"[系统] {msg}", flush=True),
        trace=trace_cb,
    )

    detail = {
        "usb": f"串口={args.port or cfg.usb.port or '自动'}",
        "ble": f"名称={args.name or '-'} 地址={args.address or '-'}",
        "mock": f"模拟动作={args.pattern} 扮演={mock_mode}",
    }[kind]
    target = (
        f"{cfg.robot_link.host}:{cfg.robot_link.port}" if send else "dry-run（不发往 brain）"
    )
    print(f"机器人模式  通道={kind}  {detail}")
    print(f"  -> brain: {target}")
    print("  语义映射：static_touch -> head_wag (ID13)，dynamic_touch -> open_mouth (ID15)")
    print("  注意：本模式不会注入任何鼠标事件。")

    # 只有在没有 external transport 的情况下才用模拟源；有硬件则复用 make_transport。
    if kind == "mock":
        pass  # RobotSession 默认自带 MockTransport
    else:
        try:
            transport = make_transport(
                cfg,
                kind=kind,
                address=args.address,
                name=args.name,
                port=args.port,
                mock=args.mock,
                pattern=args.pattern,
                mock_mode=mock_mode,
                on_status=lambda msg: print(f"[设备] {msg}", flush=True),
            )
        except TransportError as exc:
            print(f"[错误] {exc}")
            return 2
        # 换用真实设备传输，并让会话接管其数据回调。
        session.transport = transport
        transport._on_data = session._on_raw

    stop_flag = {"stop": False}

    def _on_signal(_signum, _frame):
        stop_flag["stop"] = True

    try:
        signal.signal(signal.SIGINT, _on_signal)
    except Exception:  # noqa: BLE001
        pass

    exit_code = 0
    try:
        session.start()
        exit_code = run_robot_console(session, cfg, args, stop_flag)
    except TransportError as exc:
        print(f"[错误] {exc}")
        exit_code = 2
    except KeyboardInterrupt:
        print("\n收到中断，正在退出 ...")
    finally:
        session.stop()
        if trace_writer is not None:
            try:
                trace_writer.close()
                print(f"[系统] 逐帧诊断已保存 -> {args.trace}")
            except Exception:  # noqa: BLE001
                pass
        if args.save_config:
            cfg.save(args.save_config)
            print(f"[系统] 参数已保存到 {args.save_config}")
    return exit_code


def main(argv: Optional[List[str]] = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list or args.list_usb or args.list_ble:
        return do_list(args)

    cfg = AppConfig.load_optional(args.config)
    cfg = apply_overrides(cfg, args)

    if args.selftest:
        return do_selftest(cfg)

    kind = resolve_kind(args, cfg)
    mock_mode = resolve_mock_mode(args)

    print(BANNER)

    if args.robot:
        return _run_robot_mode(cfg, args, kind, mock_mode)

    detail = {
        "usb": f"串口={args.port or cfg.usb.port or '自动'} "
               f"波特率={args.baud or cfg.usb.baudrate or '自动试探'} "
               f"形态={cfg.usb.format}",
        "ble": f"名称={args.name or '-'} 地址={args.address or '-'}",
        "mock": f"模拟动作={args.pattern} 扮演={mock_mode}",
    }[kind]
    print(f"通道={kind}  {detail}")
    print(f"映射模式={cfg.mapping.mode} 零点={cfg.mapping.origin} "
          f"最大速度={cfg.mapping.max_speed_px_s:.0f}px/s 死区={cfg.mapping.deadzone:.2f} "
          f"γ={cfg.mapping.gamma:.2f} 阵列={cfg.filter.rows}x{cfg.filter.cols}")

    try:
        transport = make_transport(
            cfg,
            kind=kind,
            address=args.address,
            name=args.name,
            port=args.port,
            mock=args.mock,
            pattern=args.pattern,
            mock_mode=mock_mode,
            on_status=lambda msg: print(f"[设备] {msg}", flush=True),
        )
    except TransportError as exc:
        print(f"[错误] {exc}")
        return 2

    session = Session(
        cfg,
        transport=transport,
        backend_kind=args.backend,
        dry_run=args.dry_run,
        unaccelerated=args.raw_move,
        on_status=lambda msg: print(f"[系统] {msg}", flush=True),
    )

    stop_flag = {"stop": False}

    def _on_signal(_signum, _frame):
        stop_flag["stop"] = True

    try:
        signal.signal(signal.SIGINT, _on_signal)
    except Exception:  # noqa: BLE001
        pass

    exit_code = 0
    try:
        session.start()
        if args.dry_run:
            print("[系统] dry-run：只打印动作，不会真的移动鼠标")
        exit_code = run_gui(session, cfg, args) if args.gui else run_console(session, cfg, args, stop_flag)
    except TransportError as exc:
        print(f"[错误] {exc}")
        exit_code = 2
    except KeyboardInterrupt:
        print("\n收到中断，正在退出 ...")
    finally:
        session.stop()
        if args.save_config:
            cfg.save(args.save_config)
            print(f"[系统] 参数已保存到 {args.save_config}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
