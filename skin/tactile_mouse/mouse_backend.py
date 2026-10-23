"""@file mouse_backend.py
@brief 鼠标输入后端与平台封装。
@details
    该模块负责将鼠标相对移动、绝对定位、点击、按键按下/抬起和滚轮事件
    转发到 SendInput / pynput / pyautogui 等目标后端，或者在 dry-run 模式下仅记录事件。

鼠标控制后端。

优先级（自动模式下依次尝试）：
    1. ``SendInputBackend``  Windows 原生 SendInput（纯 ctypes，零第三方依赖，最跟手）
    2. ``PynputBackend``     pynput，跨平台（macOS / Linux 用这个）
    3. ``PyAutoGUIBackend``  pyautogui
    4. ``NullBackend``       空实现，只记录动作，用于 ``--dry-run`` 自测

相对移动做了亚像素累加：输入是浮点像素/秒的位移，累加到整数才发一次
系统事件，避免低速时因为取整而完全不动。
"""

from __future__ import annotations

import ctypes
import sys
import time
from typing import List, Optional, Tuple


class MouseBackend:
    name = "null"

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self._acc_x = 0.0
        self._acc_y = 0.0
        self._pressed: set = set()
        self.actions: List[str] = []  # 仅 NullBackend / dry-run 使用

    # -- 需要子类实现 ------------------------------------------------------ #
    def _do_move_rel(self, dx: int, dy: int) -> None:
        pass

    def move_abs(self, x: int, y: int) -> None:
        pass

    def button_down(self, button: str = "left") -> None:
        pass

    def button_up(self, button: str = "left") -> None:
        pass

    def scroll(self, notches: int, axis: str = "v") -> None:
        pass

    def position(self) -> Tuple[int, int]:
        return (0, 0)

    def screen_size(self) -> Tuple[int, int]:
        """屏幕（虚拟桌面）尺寸，绝对定位模式会用到。

        非 Windows 平台没有统一 API，这里退化为 tkinter 查询；都没有时给个
        常见默认值，只影响 absolute 模式的映射范围。
        """
        try:
            import tkinter as tk

            root = tk.Tk()
            root.withdraw()
            size = (int(root.winfo_screenwidth()), int(root.winfo_screenheight()))
            root.destroy()
            return size
        except Exception:  # noqa: BLE001
            return (1920, 1080)

    # -- 通用逻辑 ---------------------------------------------------------- #
    def move_rel(self, dx: float, dy: float) -> None:
        """按像素移动（可带小数）。"""
        if self.dry_run:
            self._acc_x += dx
            self._acc_y += dy
            ix, iy = int(self._acc_x), int(self._acc_y)
            if ix or iy:
                self._acc_x -= ix
                self._acc_y -= iy
                self.actions.append(f"move_rel({ix},{iy})")
            return
        self._acc_x += dx
        self._acc_y += dy
        ix = int(self._acc_x)
        iy = int(self._acc_y)
        if ix == 0 and iy == 0:
            return
        self._acc_x -= ix
        self._acc_y -= iy
        self._do_move_rel(ix, iy)

    def click(self, button: str = "left", count: int = 1) -> None:
        for _ in range(max(1, count)):
            self.button_down(button)
            time.sleep(0.012)
            self.button_up(button)
            if count > 1:
                time.sleep(0.02)

    def press(self, button: str = "left") -> None:
        if button not in self._pressed:
            self._pressed.add(button)
            self.button_down(button)

    def release(self, button: str = "left") -> None:
        if button in self._pressed:
            self._pressed.discard(button)
        self.button_up(button)

    def release_all(self) -> None:
        for button in list(self._pressed):
            try:
                self.button_up(button)
            except Exception:
                pass
        self._pressed.clear()

    def wheel(self, notches: int, axis: str = "v") -> None:
        if notches:
            self.scroll(notches, axis)

    def close(self) -> None:
        self.release_all()

    # -- 工厂 -------------------------------------------------------------- #
    @staticmethod
    def auto(dry_run: bool = False) -> "MouseBackend":
        if dry_run:
            backend = NullBackend()
            backend.dry_run = True
            return backend
        for cls in (SendInputBackend, PynputBackend, PyAutoGUIBackend):
            if cls.available():
                try:
                    return cls()
                except Exception:  # noqa: BLE001
                    continue
        return NullBackend()


# --------------------------------------------------------------------------- #
# 1) Windows SendInput
# --------------------------------------------------------------------------- #

if sys.platform == "win32":
    from ctypes import wintypes

    ULONG_PTR = ctypes.c_ulonglong if ctypes.sizeof(ctypes.c_void_p) == 8 else ctypes.c_ulong

    class _MOUSEINPUT(ctypes.Structure):
        _fields_ = [
            ("dx", wintypes.LONG),
            ("dy", wintypes.LONG),
            ("mouseData", wintypes.DWORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class _KEYBDINPUT(ctypes.Structure):
        _fields_ = [
            ("wVk", wintypes.WORD),
            ("wScan", wintypes.WORD),
            ("dwFlags", wintypes.DWORD),
            ("time", wintypes.DWORD),
            ("dwExtraInfo", ULONG_PTR),
        ]

    class _HARDWAREINPUT(ctypes.Structure):
        _fields_ = [
            ("uMsg", wintypes.DWORD),
            ("wParamL", wintypes.WORD),
            ("wParamH", wintypes.WORD),
        ]

    class _INPUTUNION(ctypes.Union):
        _fields_ = [("mi", _MOUSEINPUT), ("ki", _KEYBDINPUT), ("hi", _HARDWAREINPUT)]

    class _INPUT(ctypes.Structure):
        _anonymous_ = ("u",)
        _fields_ = [("type", wintypes.DWORD), ("u", _INPUTUNION)]

    INPUT_MOUSE = 0
    MOUSEEVENTF_MOVE = 0x0001
    MOUSEEVENTF_LEFTDOWN = 0x0002
    MOUSEEVENTF_LEFTUP = 0x0004
    MOUSEEVENTF_RIGHTDOWN = 0x0008
    MOUSEEVENTF_RIGHTUP = 0x0010
    MOUSEEVENTF_MIDDLEDOWN = 0x0020
    MOUSEEVENTF_MIDDLEUP = 0x0040
    MOUSEEVENTF_WHEEL = 0x0800
    MOUSEEVENTF_HWHEEL = 0x1000
    MOUSEEVENTF_ABSOLUTE = 0x8000
    MOUSEEVENTF_VIRTUALDESK = 0x4000
    WHEEL_DELTA = 120
else:  # pragma: no cover - 非 Windows 平台不会有这个类
    _INPUT = None  # type: ignore


class SendInputBackend(MouseBackend):
    """Windows 原生鼠标事件（ctypes 调用 user32.SendInput）。

    相比 pyautogui，它不依赖任何第三方库，且直接注入系统输入队列，
    延迟最低、与前台程序的兼容性最好。
    """

    name = "sendinput"

    def __init__(self, dry_run: bool = False, unaccelerated: bool = False):
        super().__init__(dry_run)
        if sys.platform != "win32":
            raise RuntimeError("SendInputBackend 仅支持 Windows")
        self.user32 = ctypes.WinDLL("user32", use_last_error=True)
        self._unaccelerated = unaccelerated
        self._dpi_aware()

    # -- 初始化 ------------------------------------------------------------ #
    @staticmethod
    def available() -> bool:
        return sys.platform == "win32"

    def _dpi_aware(self) -> None:
        """声明 DPI 感知，否则绝对坐标会被系统缩放导致映射偏差。"""
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(2)  # PROCESS_PER_MONITOR_DPI_AWARE
        except Exception:  # noqa: BLE001
            try:
                ctypes.windll.user32.SetProcessDPIAware()
            except Exception:  # noqa: BLE001
                pass

    # -- 发送 -------------------------------------------------------------- #
    def _send(self, *inputs: "_INPUT") -> None:
        n = len(inputs)
        array = (_INPUT * n)(*inputs)
        sent = self.user32.SendInput(n, array, ctypes.sizeof(_INPUT))
        if sent != n:
            err = ctypes.get_last_error()
            # 常见原因：目标进程以更高完整性级别运行（如以管理员身份运行的程序）
            raise OSError(f"SendInput 失败 (err={err})")

    def _mouse_input(self, flags: int, dx: int = 0, dy: int = 0, data: int = 0) -> "_INPUT":
        item = _INPUT()
        item.type = INPUT_MOUSE
        item.mi = _MOUSEINPUT(dx, dy, data & 0xFFFFFFFF, flags, 0, 0)
        return item

    def _do_move_rel(self, dx: int, dy: int) -> None:
        if self._unaccelerated:
            # 绕过「提高指针精确度」的加速曲线：先读当前位置再绝对设置
            x, y = self.position()
            self.move_abs(x + dx, y + dy)
            return
        self._send(self._mouse_input(MOUSEEVENTF_MOVE, dx, dy))

    def move_abs(self, x: int, y: int) -> None:
        if self.dry_run:
            self.actions.append(f"move_abs({x},{y})")
            return
        self.user32.SetCursorPos(int(x), int(y))

    def button_down(self, button: str = "left") -> None:
        flags = {
            "left": MOUSEEVENTF_LEFTDOWN,
            "right": MOUSEEVENTF_RIGHTDOWN,
            "middle": MOUSEEVENTF_MIDDLEDOWN,
        }.get(button)
        if flags is None:
            raise ValueError(f"未知按键: {button}")
        if self.dry_run:
            self.actions.append(f"down({button})")
            return
        self._send(self._mouse_input(flags))

    def button_up(self, button: str = "left") -> None:
        flags = {
            "left": MOUSEEVENTF_LEFTUP,
            "right": MOUSEEVENTF_RIGHTUP,
            "middle": MOUSEEVENTF_MIDDLEUP,
        }.get(button)
        if flags is None:
            raise ValueError(f"未知按键: {button}")
        if self.dry_run:
            self.actions.append(f"up({button})")
            return
        self._send(self._mouse_input(flags))

    def scroll(self, notches: int, axis: str = "v") -> None:
        if self.dry_run:
            self.actions.append(f"scroll({axis},{notches})")
            return
        if axis == "h":
            self._send(self._mouse_input(MOUSEEVENTF_HWHEEL, data=int(notches) * WHEEL_DELTA))
        else:
            self._send(self._mouse_input(MOUSEEVENTF_WHEEL, data=int(notches) * WHEEL_DELTA))

    # -- 查询 -------------------------------------------------------------- #
    def position(self) -> Tuple[int, int]:
        point = wintypes.POINT()
        self.user32.GetCursorPos(ctypes.byref(point))
        return int(point.x), int(point.y)

    def screen_size(self) -> Tuple[int, int]:
        u = self.user32
        vw, vh = int(u.GetSystemMetrics(78)), int(u.GetSystemMetrics(79))  # 虚拟桌面
        if vw > 0 and vh > 0:
            return vw, vh
        return int(u.GetSystemMetrics(0)), int(u.GetSystemMetrics(1))


# --------------------------------------------------------------------------- #
# 2) pynput（跨平台）
# --------------------------------------------------------------------------- #


class PynputBackend(MouseBackend):
    name = "pynput"

    def __init__(self, dry_run: bool = False):
        super().__init__(dry_run)
        from pynput.mouse import Button, Controller  # type: ignore

        self._Button = Button
        self._ctl = Controller()

    @staticmethod
    def available() -> bool:
        try:
            import pynput  # noqa: F401

            return True
        except Exception:  # noqa: BLE001
            return False

    def _btn(self, button: str):
        return {
            "left": self._Button.left,
            "right": self._Button.right,
            "middle": self._Button.middle,
        }[button]

    def _do_move_rel(self, dx: int, dy: int) -> None:
        self._ctl.move(int(dx), int(dy))

    def move_abs(self, x: int, y: int) -> None:
        self._ctl.position = (int(x), int(y))

    def button_down(self, button: str = "left") -> None:
        self._ctl.press(self._btn(button))

    def button_up(self, button: str = "left") -> None:
        self._ctl.release(self._btn(button))

    def scroll(self, notches: int, axis: str = "v") -> None:
        if hasattr(self._ctl, "scroll"):
            self._ctl.scroll(0 if axis == "v" else notches, notches if axis == "v" else 0)

    def position(self) -> Tuple[int, int]:
        x, y = self._ctl.position
        return int(x), int(y)


# --------------------------------------------------------------------------- #
# 3) pyautogui
# --------------------------------------------------------------------------- #


class PyAutoGUIBackend(MouseBackend):
    name = "pyautogui"

    def __init__(self, dry_run: bool = False):
        super().__init__(dry_run)
        import pyautogui  # type: ignore

        self._gui = pyautogui
        self._gui.FAILSAFE = False  # 否则光标撞到屏幕角落会抛异常
        self._gui.PAUSE = 0

    @staticmethod
    def available() -> bool:
        try:
            import pyautogui  # noqa: F401

            return True
        except Exception:  # noqa: BLE001
            return False

    def _do_move_rel(self, dx: int, dy: int) -> None:
        self._gui.moveRel(int(dx), int(dy), duration=0, _pause=False)

    def move_abs(self, x: int, y: int) -> None:
        self._gui.moveTo(int(x), int(y), duration=0, _pause=False)

    def button_down(self, button: str = "left") -> None:
        self._gui.mouseDown(button=button, _pause=False)

    def button_up(self, button: str = "left") -> None:
        self._gui.mouseUp(button=button, _pause=False)

    def scroll(self, notches: int, axis: str = "v") -> None:
        if axis == "h":
            self._gui.hscroll(int(notches), _pause=False)
        else:
            self._gui.scroll(int(notches), _pause=False)

    def position(self) -> Tuple[int, int]:
        x, y = self._gui.position()
        return int(x), int(y)

    def screen_size(self) -> Tuple[int, int]:
        w, h = self._gui.size()
        return int(w), int(h)


# --------------------------------------------------------------------------- #
# 4) 空实现（dry-run / 单元测试）
# --------------------------------------------------------------------------- #


class NullBackend(MouseBackend):
    name = "null"

    def _do_move_rel(self, dx: int, dy: int) -> None:
        self.actions.append(f"move_rel({dx},{dy})")

    def move_abs(self, x: int, y: int) -> None:
        self.actions.append(f"move_abs({x},{y})")

    def button_down(self, button: str = "left") -> None:
        self.actions.append(f"down({button})")

    def button_up(self, button: str = "left") -> None:
        self.actions.append(f"up({button})")

    def scroll(self, notches: int, axis: str = "v") -> None:
        self.actions.append(f"scroll({axis},{notches})")

    def position(self) -> Tuple[int, int]:
        return self._pos

    _pos: Tuple[int, int] = (960, 540)

    def screen_size(self) -> Tuple[int, int]:
        return (1920, 1080)


def create_backend(kind: str = "auto", dry_run: bool = False, unaccelerated: bool = False) -> MouseBackend:
    """按名称创建后端：auto | sendinput | pynput | pyautogui | null。"""
    kind = (kind or "auto").lower()
    if kind == "auto":
        return MouseBackend.auto(dry_run)
    if kind == "sendinput":
        backend = SendInputBackend(dry_run=dry_run, unaccelerated=unaccelerated)
    elif kind == "pynput":
        backend = PynputBackend(dry_run=dry_run)
    elif kind == "pyautogui":
        backend = PyAutoGUIBackend(dry_run=dry_run)
    elif kind == "null":
        backend = NullBackend()
    else:
        raise ValueError(f"未知鼠标后端: {kind}")
    backend.dry_run = dry_run
    return backend
