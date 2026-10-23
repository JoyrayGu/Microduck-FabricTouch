# SPDX-License-Identifier: GPL-3.0-or-later
"""@file servo_runtime.py
@brief 动作执行后端：默认 dry-run，只有显式开启才访问真实舵机总线。

为什么单独一层
--------------
brain 有两类环境：

* 没有真机（开发机 / RK3566 未接线）——只能 dry-run。
* 有真机——才允许打开 ``/dev/tty*`` 并驱动 XL330。

把两者放在同一个接口 ``ServoRuntime`` 后面，dispatcher 就不需要关心硬件是否
在线，也不会因为误配置而在无意中给舵机上电。

安全
----
* :class:`DryRunRuntime` 只记录目标位置，绝不打开串口。
* :class:`DynamixelRuntime` 只有在 ``enable_torque=True`` 时才会调用扭矩开启，
  并且启动时先读取当前位置再写目标，避免上电瞬间跳动（与 ``pose_loop.py``
  的做法保持一致）。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional

# Dynamixel Protocol 2.0 控制表地址（XL330）
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

POSITION_UNIT = 4096.0
CENTER = 2048


def counts_to_rad(counts: float) -> float:
    return (counts - CENTER) * (2.0 * math.pi) / POSITION_UNIT


@dataclass
class FrameLog:
    """dry-run 记录的一帧目标位置。"""

    t: float
    targets: Dict[int, int]


@dataclass
class RuntimeStatus:
    """执行期状态，供服务器 /health 与日志使用。"""

    runtime: str = "dry_run"
    hardware_ready: bool = False
    last_error: str = ""
    frames: int = 0
    enabled_ids: List[int] = field(default_factory=list)


class ServoRuntime:
    """动作执行后端接口。"""

    name = "base"

    def prepare(self, target_ids: List[int]) -> None:
        """执行前准备：检查总线、使能所需舵机、读取当前位置。"""
        raise NotImplementedError

    def step(self, targets: Mapping[int, int]) -> None:
        """写入一帧目标位置。"""
        raise NotImplementedError

    def rest(self) -> None:
        """动作间隙调用（保持当前目标）。"""
        raise NotImplementedError

    def close(self) -> None:
        """释放总线 / 串口。"""
        raise NotImplementedError

    @property
    def status(self) -> RuntimeStatus:
        raise NotImplementedError


class DryRunRuntime(ServoRuntime):
    """只记录目标位置，不接触任何硬件。"""

    name = "dry_run"

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self._status = RuntimeStatus(runtime=self.name, hardware_ready=False)
        self.log: List[FrameLog] = []
        self._start = time.perf_counter()

    def prepare(self, target_ids: List[int]) -> None:
        self._status.enabled_ids = sorted(int(i) for i in target_ids)
        self._status.hardware_ready = False
        self.log.clear()
        self._start = time.perf_counter()
        if self.verbose:
            print(f"[dry-run] prepare ids={self._status.enabled_ids}", flush=True)

    def step(self, targets: Mapping[int, int]) -> None:
        frame = {int(k): int(v) for k, v in targets.items()}
        self.log.append(FrameLog(t=time.perf_counter() - self._start, targets=frame))
        self._status.frames += 1
        if self.verbose:
            rendered = " ".join(f"ID{k}={v}" for k, v in sorted(frame.items()))
            print(f"[dry-run] t={self.log[-1].t:.3f} {rendered}", flush=True)

    def rest(self) -> None:
        return None

    def close(self) -> None:
        return None

    @property
    def status(self) -> RuntimeStatus:
        return self._status


class DynamixelRuntime(ServoRuntime):
    """真实舵机后端（Dynamixel Protocol 2.0）。仅在 ``enable_torque=True`` 时使能扭矩。"""

    name = "dynamixel"

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud: int = 1_000_000,
        enable_torque: bool = True,
    ):
        self.port_name = port
        self.baud = baud
        self.enable_torque = enable_torque
        self._status = RuntimeStatus(runtime=self.name, hardware_ready=False)
        self._port = None
        self._packet = None
        self._current: Dict[int, int] = {}

    # -- 内部：动态导入 dynamixel_sdk，避免无硬件环境导入失败 -- #
    def _ensure_sdk(self):
        if self._packet is not None:
            return
        try:
            from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler  # noqa: PLC0415
        except ImportError as exc:  # pragma: no cover - 依赖真实环境
            raise RuntimeError("未安装 dynamixel_sdk，无法驱动真实舵机") from exc
        self._comm_success = COMM_SUCCESS
        self._PortHandler = PortHandler
        self._PacketHandler = PacketHandler
        self._packet = PacketHandler(2.0)

    def _check(self, label: str, comm: int, error: int) -> None:
        if comm != self._comm_success:
            raise RuntimeError(f"{label}: {self._packet.getTxRxResult(comm)}")
        if error:
            raise RuntimeError(f"{label}: {self._packet.getRxPacketError(error)}")

    def _read4(self, sid: int, address: int) -> int:
        value, comm, error = self._packet.read4ByteTxRx(self._port, sid, address)
        self._check(f"ID{sid} read4@{address}", comm, error)
        return value

    def _write1(self, sid: int, address: int, value: int) -> None:
        comm, error = self._packet.write1ByteTxRx(self._port, sid, address, value)
        self._check(f"ID{sid} write1@{address}", comm, error)

    def _write4(self, sid: int, address: int, value: int) -> None:
        comm, error = self._packet.write4ByteTxRx(self._port, sid, address, value)
        self._check(f"ID{sid} write4@{address}", comm, error)

    def prepare(self, target_ids: List[int]) -> None:
        """为一次动作准备总线：打开串口（仅首次）、读位置、锁定、开扭矩。

        关键：串口**只打开一次**。
        --------------------------
        早期实现每次动作都重新 ``PortHandler(...)`` + ``openPort()``，而
        ``close()`` 从不被调用（动作结束后不关）。结果是每个动作都泄漏一个
        串口句柄：实测第 3 个动作开始出现
        ``STEP error: ID14 write4@116: There is no status packet!``，
        且该舵机的 torque 位仍停在 0 —— 写入落到了一个失效句柄上。
        用独立进程在同一时刻写同一个 ID 却完全正常，证明总线本身没问题。

        现在只在首次准备时打开串口，后续动作复用同一个 PortHandler。
        """
        self._ensure_sdk()
        if self._port is None:
            port = self._PortHandler(self.port_name)
            if not port.openPort() or not port.setBaudRate(self.baud):
                raise RuntimeError(f"无法打开 {self.port_name} @ {self.baud}")
            self._port = port
        self._status.enabled_ids = sorted(int(i) for i in target_ids)
        # 先读取当前位置，并把目标设成当前位置，避免使能瞬间跳动。
        self._current = {}
        for sid in self._status.enabled_ids:
            self._current[sid] = self._read4(sid, ADDR_PRESENT_POSITION)
        for sid in self._status.enabled_ids:
            self._write4(sid, ADDR_GOAL_POSITION, self._current[sid])
        if self.enable_torque:
            for sid in self._status.enabled_ids:
                self._write1(sid, ADDR_TORQUE_ENABLE, 1)
        self._status.hardware_ready = True

    #: 单次写入允许的最大位移（tick）。超过时拆成多次小步写入。
    #:
    #: 实测（同一台设备、同一条总线）单步位移与通信失败的关系：
    #:     169 tick  -> ok=1 fail=3  最终位置 1802（完全没动）
    #:      40 tick  -> ok=5 fail=4
    #:      ~7 tick  -> ok=12 fail=0  最终位置 1880（正常）
    #: 写大位移后舵机全速运转，此时它响应总线的能力下降，后续写入返回
    #: -3001（RX 超时）。间隔 0.025s 与 0.10s 结果相同，说明不是时序问题。
    MAX_STEP_TICKS = 12

    def step(self, targets: Mapping[int, int]) -> None:
        if self._port is None:
            raise RuntimeError("prepare() 未调用或失败")
        for sid, value in targets.items():
            sid = int(sid)
            if self._status.enabled_ids and sid not in self._status.enabled_ids:
                # 只允许写向本动作声明过的舵机。
                continue
            value = int(value)
            # 把大位移拆成多个小步，避免舵机全速运转时丢指令。
            start = self._current.get(sid, value)
            delta = value - start
            if abs(delta) > self.MAX_STEP_TICKS:
                steps = (abs(delta) + self.MAX_STEP_TICKS - 1) // self.MAX_STEP_TICKS
                for i in range(1, steps + 1):
                    inter = start + int(round(delta * i / steps))
                    self._write4(sid, ADDR_GOAL_POSITION, inter)
                    time.sleep(0.002)
            else:
                self._write4(sid, ADDR_GOAL_POSITION, value)
            self._current[sid] = value
        self._status.frames += 1

    def rest(self) -> None:
        return None

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.closePort()
            except Exception:  # noqa: BLE001
                pass
        self._port = None
        self._status.hardware_ready = False

    @property
    def current(self) -> Dict[int, int]:
        return dict(self._current)

    @property
    def status(self) -> RuntimeStatus:
        return self._status


def make_runtime(
    dry_run: bool = True,
    port: str = "/dev/ttyACM0",
    baud: int = 1_000_000,
    verbose: bool = False,
) -> ServoRuntime:
    """根据开关创建后端。**默认 dry-run**，与里程碑约定一致。"""
    if dry_run:
        return DryRunRuntime(verbose=verbose)
    return DynamixelRuntime(port=port, baud=baud, enable_torque=True)
