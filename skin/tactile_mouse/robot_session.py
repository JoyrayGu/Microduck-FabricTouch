"""@file robot_session.py
@brief 机器人模式会话：采集 -> 特征 -> 机器人手势 -> 发送给 brain。

与 ``session.Session``（鼠标模式）并列，复用相同的 ``Transport`` / ``protocol``，
但**不经过鼠标后端**，因此不会注入任何鼠标事件。

数据流::

    原始帧 -> TactileProcessor -> RobotGestureRecognizer
           -> RobotLink (TCP + JSON) -> microduck_brain

这样「鼠标模式」与「机器人模式」互不干扰：即便机器人模式正在运行，鼠标也不会
被意外移动。
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Callable, Optional, Tuple

from . import protocol as P
from .config import AppConfig
from .features import TactileProcessor, TactileState
from .robot_gestures import RobotGestureEvent, RobotGestureRecognizer
from .robot_link import events as E
from .robot_link.client import RobotLink
from .transport import MockTransport, Transport

StatusCallback = Callable[[str], None]


class RobotSession:
    """把触觉阵列的一帧帧数据变成发给 brain 的机器人手势事件。"""

    #: 打开串口后等待设备就绪的秒数。见 ``start()`` 里的说明与实测数据。
    SETTLE_S = 0.5

    def __init__(
        self,
        cfg: Optional[AppConfig] = None,
        transport: Optional[Transport] = None,
        link: Optional[RobotLink] = None,
        on_status: Optional[StatusCallback] = None,
        on_gesture: Optional[Callable[[RobotGestureEvent], None]] = None,
        dry_run: bool = False,
        queue_size: int = 8,
        trace: Optional[Callable[[TactileState], None]] = None,
    ):
        self.cfg = cfg or AppConfig()
        self.on_status = on_status or (lambda msg: print(msg, flush=True))
        self.on_gesture = on_gesture
        self.dry_run = dry_run
        #: 可选的逐帧回调，用于把 active/peak/centroid 落盘做离线诊断。
        self.trace = trace

        self.parser = P.StreamParser()
        self._queue: "queue.Queue[Tuple[int, list]]" = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._poller: Optional[threading.Thread] = None

        self.processor = TactileProcessor(
            self.cfg.filter,
            invert_x=self.cfg.invert_grid_x,
            invert_y=self.cfg.invert_grid_y,
        )
        # 机器人模式把最小激活格数降到 1。
        #
        # 手指快速滑动时压力向前集中、边缘格迅速衰减，实测出现 peak=14~17
        # 却只有 1 个格超过阈值的情况。要求 2 格会把 active 误判为 False，
        # 把一次连续滑动切成碎片，再被消抖连成跨越大片区域的假轨迹。
        # 单点毛刺由 min_contact_ms 过滤，不依赖这个格数。
        # 鼠标模式保持配置默认值（2），因为其既有整定基于该值。
        self.processor.cfg.min_active_cells = 1
        self.recognizer = RobotGestureRecognizer(self.cfg.robot_gesture)
        self.link = link

        if transport is None:
            transport = MockTransport(
                on_data=self._on_raw,
                rows=self.cfg.filter.rows,
                cols=self.cfg.filter.cols,
            )
        self.transport = transport
        self.transport._on_data = self._on_raw  # 复用同一传输实例时确保回调正确

        self.last_state: Optional[TactileState] = None
        self.events_sent = 0

    # ------------------------------------------------------------------ #
    def status(self, msg: str) -> None:
        self.on_status(msg)

    def start(self) -> None:
        self._stop.clear()
        self.transport.open()
        if self.link is not None and not self.dry_run:
            self.link.start()
            self.status(
                f"robot link -> {self.cfg.robot_link.host}:{self.cfg.robot_link.port}"
            )
        else:
            self.status("robot link: dry-run（不发送到 brain）")

        self._worker = threading.Thread(target=self._worker_loop, name="robot-worker", daemon=True)
        self._worker.start()
        # 打开串口时 DTR 会拉低再抬起，USB CDC 设备会因此重新枚举。
        # 这段时间内下发的指令会被设备丢弃——但仍会回应答帧，所以从日志看
        # 像"配置成功"，实际 START 没生效，设备一帧都不推。
        # 实测（同一台设备）：open 后立即下发 = 0 帧；等 0.3s = 99 帧。
        time.sleep(self.SETTLE_S)
        self._configure_device()
        if (self.cfg.acquire or "continuous").lower() == "single":
            self._poller = threading.Thread(target=self._poll_loop, name="robot-poll", daemon=True)
            self._poller.start()

    def stop(self) -> None:
        self._stop.set()
        # 不要发 cmd_stop()：本机固件收到 0x03 后无法用 START 恢复推流，
        # 会让下一次启动彻底收不到数据，只能给设备重新上电。直接关串口即可。
        if self.link is not None and not self.dry_run:
            self.link.stop()
        for thread in (self._worker, self._poller):
            if thread is not None:
                thread.join(timeout=2.0)
        self._worker = None
        self._poller = None
        try:
            self.transport.close()
        except Exception:  # noqa: BLE001
            pass

    # ------------------------------------------------------------------ #
    def feed(self, points, frame_no: int = 0, timestamp: Optional[float] = None) -> TactileState:
        """直接喂入一帧原始采集点（供测试与离线回放使用）。

        ``points`` 为行优先排列的整数列表，与设备上行数据一致。
        """
        t = timestamp if timestamp is not None else time.perf_counter()
        state = self.processor.process(points, frame_no=frame_no, timestamp=t)
        if not self.processor.calibrated:
            self.processor.push_calibration_frame(state.raw)
            self.last_state = state
            return state
        self._handle_state(state)
        return state

    def _handle_state(self, state: TactileState) -> None:
        self.last_state = state
        if self.trace is not None:
            self.trace(state)
        events = self.recognizer.update(state)
        for event in events:
            self._emit(event)

    def _emit(self, event: RobotGestureEvent) -> None:
        if self.on_gesture is not None:
            try:
                self.on_gesture(event)
            except Exception as exc:  # noqa: BLE001
                self.status(f"on_gesture: {exc}")
        if self.dry_run or self.link is None:
            self.status(f"[dry-run] gesture={event}")
            return
        # 显式生成 event_id 并回显，便于与 Pi 日志逐条对账。
        # 不依赖 link.send 内部生成：那样本端日志就无从知道自己的 id，
        # 两端无法精确配对（tap 与 single_stroke 又都映射到 tilt_head）。
        eid = E.new_event_id()
        self.status(f"-> brain event_id={eid} gesture={event.gesture.value}")
        if self.link.send(event, event_id=eid):
            self.events_sent += 1
        else:
            self.status(f"发送失败，事件丢弃: {event} event_id={eid}")

    # ------------------------------------------------------------------ #
    # 与鼠标模式 Session 相同的线程/协议骨架
    # ------------------------------------------------------------------ #
    def _on_raw(self, data: bytes) -> None:
        """在传输层线程里轻量解析，把采集数据帧丢进队列（满了丢最旧的）。"""
        for frame in self.parser.feed(data):
            if frame.cmd in (P.Command.CONTINUOUS, P.Command.SINGLE) and len(frame.data) >= 2:
                try:
                    frame_no, points = P.parse_acquire_payload(
                        frame.data, self.processor.rows, self.processor.cols
                    )
                except P.ProtocolError:
                    continue
                self._put((frame_no, points))

    def _put(self, item: Tuple[int, list]) -> None:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.put_nowait(item)
            except queue.Empty:
                pass

    def _worker_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame_no, points = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            state = self.processor.process(points, frame_no=frame_no)
            if not self.processor.calibrated:
                self.processor.push_calibration_frame(state.raw)
                continue
            self._handle_state(state)

    def _poll_loop(self) -> None:
        period = 1.0 / max(1.0, self.cfg.single_poll_hz)
        next_t = time.perf_counter()
        while not self._stop.is_set():
            try:
                self.send(P.cmd_single_acquire())
            except Exception as exc:  # noqa: BLE001
                self.status(f"单次采集指令发送失败: {exc}")
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    def send(self, data: bytes) -> None:
        self.transport.send(data)

    def _configure_device(self) -> None:
        # 与鼠标模式保持一致：先设通道、设阵列、设阈值，再启动采集。
        #
        # 绝对不要先发 ``cmd_stop()``（0x03）：本机固件实测有缺陷，STOP 之后
        # 再发 START 只会吐 1 帧就彻底停止推流，且无法用任何指令序列恢复，
        # 只能重新上电。实测数据（同一台设备、连续 6 轮）：
        #     只发 STOP -> START       : 1 帧
        #     配置(不含 STOP) -> START : 82~83 帧，稳定
        # 设备上电本来就不推流，所以 STOP 这一步从一开始就是多余的。
        self.send(P.cmd_set_transport_mode(self.cfg.device_mode))
        self.send(P.cmd_set_array(*self.cfg.device_array))
        self.send(P.cmd_set_threshold(int(self.cfg.device_threshold)))
        if (self.cfg.acquire or "continuous").lower() != "single":
            self.send(P.cmd_start_continuous())
        self.status("已下发设备配置指令（通道/阵列/阈值/采集）")
