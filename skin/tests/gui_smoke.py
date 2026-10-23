"""GUI 冒烟测试：构建界面、跑一段模拟数据、检查渲染，然后自动关闭。

用法（需要带 tkinter 的 Python）：
    python tests/gui_smoke.py            # 默认用 USB 明文模式模拟设备
    python tests/gui_smoke.py ble        # 用蓝牙形态模拟设备
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse.config import AppConfig
from tactile_mouse.mouse_backend import NullBackend
from tactile_mouse.session import Session, make_transport
from tactile_mouse.gui import TactileMouseGUI

MOCK_MODE = sys.argv[1] if len(sys.argv) > 1 else "usb-ascii"

cfg = AppConfig()
cfg.filter.baseline_frames = 10
transport = make_transport(cfg, mock=True, mock_mode=MOCK_MODE, pattern="tap",
                           on_status=lambda m: None)
print("模拟通道:", transport.name, "通道形态:", getattr(transport, "channel_mode", "-"))
# 冒烟测试用空后端，避免真的移动鼠标
session = Session(cfg, transport=transport, backend=NullBackend(),
                  on_status=lambda m: print("[状态]", m))
session.start()

gui = TactileMouseGUI(session, cfg, enable_hotkey=False)
frames = {"n": 0}


def step():
    frames["n"] += 1
    gui.root.update()
    if frames["n"] < 60:
        gui.root.after(30, step)
    else:
        print("渲染帧数:", frames["n"])
        print("日志行数:", gui.log.index("end-1c"))
        print("状态栏:", gui.lbl_status.cget("text"))
        print("总压力读数:", gui.readouts["total"].cget("text"))
        print("质心读数:", gui.readouts["centroid"].cget("text"))
        print("接触点读数:", gui.readouts["contacts"].cget("text"))
        print("按键读数:", gui.readouts["button"].cget("text"))
        print("电量读数:", gui.readouts["battery"].cget("text"))
        gui.toggle_enabled()
        print("切换后状态:", gui.lbl_status.cget("text"))
        gui.recalibrate()
        gui.emergency_stop()
        gui.root.update()
        print("急停后状态:", gui.lbl_status.cget("text"))
        gui._close()


gui.root.after(30, step)
gui.run()
session.stop()
print("GUI 冒烟测试完成")
