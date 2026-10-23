"""@file client.py
@brief skin -> brain 的 TCP 客户端（后台发送线程 + 自动重连）。

为什么用 TCP 长连接 + newline-delimited JSON
--------------------------------------------
* 固定两端（laptop -> RK3566），事件量小、需要低延迟。
* 标准库即可实现，没有 HTTP 框架负担。
* TCP 是字节流，不保留消息边界，所以约定「一行一个 JSON」。
* 每个事件带 ``event_id``；brain 的应答回来后可据此判断是否被接受。

注意
----
本客户端**不**注入鼠标，也不直接驱动舵机。发送失败只记录状态，不会阻塞
触觉采集主循环。
"""

from __future__ import annotations

import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from ..config import RobotLinkConfig
from ..robot_gestures import RobotGestureEvent
from . import events as E


class RobotLinkError(RuntimeError):
    """连接或发送失败。"""


@dataclass
class LinkStats:
    sent: int = 0
    dropped: int = 0
    replies: int = 0
    last_reply: Dict[str, Any] = field(default_factory=dict)
    connected: bool = False
    last_error: str = ""


class RobotLink:
    """把机器人手势事件异步送给 brain。

    使用::

        link = RobotLink(RobotLinkConfig(host="192.168.1.50", port=9000))
        link.start()
        link.send(event)      # 非阻塞，进入本地队列
        ...
        link.stop()
    """

    def __init__(
        self,
        cfg: Optional[RobotLinkConfig] = None,
        on_reply: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        self.cfg = cfg or RobotLinkConfig()
        self.on_reply = on_reply
        self.stats = LinkStats()

        self._queue: "queue.Queue[bytes]" = queue.Queue(maxsize=max(1, self.cfg.queue_size))
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        #: 应答接收缓冲：TCP 是字节流，一次 recv 可能是半行或数行。
        self._rx_buffer = b""

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="robot-link", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        self._close_socket()

    # ------------------------------------------------------------------ #
    @property
    def connected(self) -> bool:
        return self.stats.connected

    def send(self, event: RobotGestureEvent, event_id: Optional[str] = None) -> bool:
        """把事件放入发送队列。队列满时丢弃**最旧**的一条并记录。

        返回 True 表示成功入队（不代表 brain 已接受）。
        """
        try:
            payload = E.to_wire(event, event_id=event_id)
            line = E.encode_line(payload)
        except Exception as exc:  # noqa: BLE001
            self.stats.last_error = f"encode: {exc}"
            self.stats.dropped += 1
            return False

        try:
            self._queue.put_nowait(line)
            return True
        except queue.Full:
            try:
                self._queue.get_nowait()  # 丢最旧
                self.stats.dropped += 1
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(line)
                return True
            except queue.Full:
                self.stats.dropped += 1
                return False

    # ------------------------------------------------------------------ #
    def _run(self) -> None:
        while not self._stop.is_set():
            if self._sock is None:
                if not self._connect():
                    self._wait(self.cfg.reconnect_interval)
                    continue
            try:
                line = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if not self._send_line(line):
                # 发送失败：把该行放回队首（尽量），并断开以便重连。
                self._requeue(line)
                self._close_socket()
                if not self.cfg.reconnect:
                    break
                self._wait(self.cfg.reconnect_interval)
        self._close_socket()

    def _connect(self) -> bool:
        try:
            sock = socket.create_connection(
                (self.cfg.host, self.cfg.port), timeout=self.cfg.connect_timeout
            )
            sock.settimeout(self.cfg.timeout)
            with self._lock:
                self._sock = sock
            # 新连接不继承旧连接的半行残留，否则会把旧数据当成新应答解析。
            self._rx_buffer = b""
            self.stats.connected = True
            self.stats.last_error = ""
            return True
        except OSError as exc:
            self.stats.connected = False
            self.stats.last_error = f"connect: {exc}"
            return False

    def _send_line(self, line: bytes) -> bool:
        with self._lock:
            sock = self._sock
        if sock is None:
            return False
        try:
            # 发送前先把上一条事件的应答取走，避免本次触发后读到的是旧应答。
            # brain 执行一个动作要 0.65~1.4s，应答通常在**下一次**发送前才到达，
            # 若不在发送前读取，日志里的应答就会永远滞后一个事件。
            self._try_read_reply(sock)
            sock.sendall(line)
            self.stats.sent += 1
            # 发送后再试一次：brain 也可能立即拒收（expired/busy/duplicate），
            # 这种应答几乎瞬时返回。
            self._try_read_reply(sock)
            return True
        except OSError as exc:
            self.stats.last_error = f"send: {exc}"
            return False

    def _try_read_reply(self, sock: socket.socket) -> None:
        """非阻塞地尝试读取应答（可能一次收到多行，也可能只有半行）。

        为什么需要缓冲
        --------------
        TCP 是字节流，``recv`` 不保证一行就是一次返回。早期实现直接把
        单次 ``recv`` 的结果当成完整 JSON 解析，实测出现过：
          * 输入 ``[brain] REJECTED reason=None event_id=-``（解析到空字典）
          * 应答与事件错位（第 N 条的应答在下一次发送后才被读到）

        另外早期只用 1ms 超时读一次，而 brain 执行一个动作要 0.65~1.4s，
        应答必然在下次发送之后才到 —— 于是每条应答都滞后一个事件。现在改为
        每次发送前与收数据时都把缓冲区里的完整行全部取出。
        """
        try:
            sock.settimeout(0.001)
            data = sock.recv(4096)
            sock.settimeout(self.cfg.timeout)
        except (socket.timeout, OSError):
            try:
                sock.settimeout(self.cfg.timeout)
            except OSError:
                pass
            return
        if not data:
            return

        self._rx_buffer += data
        # 按换行切分；最后一段可能是不完整的行，留在缓冲里等后续数据。
        while b"\n" in self._rx_buffer:
            raw_line, self._rx_buffer = self._rx_buffer.split(b"\n", 1)
            text = raw_line.decode("utf-8", errors="ignore").strip()
            if not text:
                continue  # 空行不是应答，跳过（不要当成 {} 上报）
            self.stats.replies += 1
            try:
                import json

                reply = json.loads(text)
                if not isinstance(reply, dict):
                    reply = {"raw": text}
            except ValueError:
                reply = {"raw": text}
            self.stats.last_reply = reply
            if self.on_reply is not None:
                try:
                    self.on_reply(reply)
                except Exception:  # noqa: BLE001
                    pass

    def _requeue(self, line: bytes) -> None:
        """尽量把未发送成功的行放回队首。标准 queue 无此能力，改用新队列临时承接。"""
        try:
            self._queue.put_nowait(line)
        except queue.Full:
            self.stats.dropped += 1

    def _wait(self, seconds: float) -> None:
        self._stop.wait(timeout=max(0.0, seconds))

    def _close_socket(self) -> None:
        with self._lock:
            sock, self._sock = self._sock, None
        self.stats.connected = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
