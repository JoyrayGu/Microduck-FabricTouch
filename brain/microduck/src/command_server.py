# SPDX-License-Identifier: GPL-3.0-or-later
"""@file command_server.py
@brief 常驻的 TCP 命令服务器：接收 skin 的手势事件并驱动动作。

线上格式
--------
* TCP 长连接。
* 每行一个 JSON（newline-delimited JSON），UTF-8。
* skin -> brain 发送手势事件；brain -> skin 回一行应答。
* 单客户端（Milestone 1）；额外连接被拒绝。

这与现有 ``microduck_gui.py``（SSH 启脚本）不同：SSH 用于部署/运维，
本服务用于**实时**手势事件，长期运行、不每次重启进程。

安全
----
* 只接受白名单手势；不接受舵机 ID / count / 速度。
* 事件过期、重复、动作执行期间的新事件都会被拒绝。
* 默认 dry-run，绝不接触真实舵机总线，除非 ``--enable-hardware``。
"""

from __future__ import annotations

import argparse
import json
import socket
import threading
import time
from typing import Callable, List, Optional

from . import command_schema as S
from .action_dispatcher import ActionDispatcher
from .action_library import ActionLibrary
from .servo_runtime import DryRunRuntime, ServoRuntime, make_runtime

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 9000


class CommandServer:
    """单客户端的 TCP 事件服务器。"""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        library: Optional[ActionLibrary] = None,
        runtime: Optional[ServoRuntime] = None,
        dry_run: bool = True,
        log: Optional[Callable[[str], None]] = None,
    ):
        self.host = host
        self.port = port
        self.library = library or ActionLibrary()
        self.runtime = runtime or make_runtime(dry_run=dry_run)
        self._log = log or (lambda msg: print(msg, flush=True))

        self.dispatcher = ActionDispatcher(
            library=self.library,
            on_step=self._on_step,
            on_action_start=self._on_action_start,
            on_action_end=self._on_action_end,
        )

        self._sock: Optional[socket.socket] = None
        self._client: Optional[socket.socket] = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        #: 本动作内连续写入失败的帧数，用于折叠日志并在结束时汇总。
        self._step_failures = 0

    # ------------------------------------------------------------------ #
    def _on_action_start(self, action, event=None) -> None:
        # 附带 event_id / 手势名 / brain 侧接收时刻。
        #
        # 为什么要带手势名：短促接触(tap)与滑动(single_stroke/back_forth_stroke)
        # 现在映射到不同动作，但将来仍可能出现多个手势共用一个动作的情况，
        # 只看 action 无法判断是哪一个手势触发的。
        # 为什么要带 event_id：skin 侧会打印同一条 event_id，两端据此精确对账，
        # 出现异常时能立刻区分「识别错了」「传输丢了」「硬件没动」。
        if event is not None:
            self._log(
                f"ACTION_START {action.name} ids={action.ids} "
                f"gesture={event.gesture} event_id={event.event_id}"
            )
        else:
            self._log(f"ACTION_START {action.name} ids={action.ids}")
        self._step_failures = 0
        try:
            self.runtime.prepare(action.ids)
        except Exception as exc:  # noqa: BLE001
            self._log(f"ACTION_START failed prepare: {exc}")

    def _on_step(self, targets) -> None:
        # 硬件写入失败必须**向上抛**，不能只记日志。
        #
        # 早期实现把异常吞在这里，结果 dispatcher 认为每帧都写成功，
        # 动作跑完照常报告 ``outcome=done``。实测出现过「舵机一步都没动，
        # 日志却全部成功」的情况（ID14 write4@116: no status packet! ×23，
        # 最后仍打印 ACTION_END outcome=done），排查时极具误导性。
        #
        # 抛出后 _run_frames 会立即中止，剩余帧不再下发——这比继续往一个
        # 已经不响应的总线上刷指令更安全。
        try:
            self.runtime.step(targets)
        except Exception as exc:  # noqa: BLE001
            self._step_failures += 1
            if self._step_failures == 1:
                self._log(f"STEP error: {exc}")
            raise

    def _on_action_end(self, action, outcome: str) -> None:
        if self._step_failures:
            self._log(
                f"ACTION_END {action.name} outcome={outcome} "
                f"step_failures={self._step_failures}"
            )
        else:
            self._log(f"ACTION_END {action.name} outcome={outcome}")
        self._step_failures = 0

    # ------------------------------------------------------------------ #
    def serve_forever(self) -> None:
        """阻塞运行，直到 :meth:`shutdown` 被调用。"""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(1)
        self._sock.settimeout(0.5)
        self._ready.set()
        self._log(f"COMMAND_SERVER listening on {self.host}:{self.port} runtime={self.runtime.name}")
        try:
            while not self._stop.is_set():
                try:
                    conn, addr = self._sock.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                self._log(f"client connected {addr[0]}:{addr[1]}")
                self._handle_client(conn)
        finally:
            self._close_socket()

    def _handle_client(self, conn: socket.socket) -> None:
        self._client = conn
        conn.settimeout(0.5)
        buffer = b""
        try:
            while not self._stop.is_set():
                try:
                    chunk = conn.recv(1024)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    self._handle_line(conn, line)
        finally:
            self._client = None
            try:
                conn.close()
            except OSError:
                pass
            self._log("client disconnected")

    def _handle_line(self, conn: socket.socket, raw: bytes) -> None:
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            self._reply(conn, S.make_reply(None, False, reason="bad_encoding"))
            return
        try:
            event = S.parse_line(text)
        except S.SchemaError as exc:
            self._reply(conn, S.make_reply(None, False, reason=exc.reason))
            return

        # 执行期异常（例如硬件写入失败）不能中断连接。
        #
        # dispatcher._execute 会把异常继续向上抛（这样 on_action_end 才能拿到
        # outcome="error"），若此处不捕获，异常会冒到 _handle_client 的 while
        # 循环外，导致整个客户端连接被关闭，之后所有事件都收不到。
        try:
            decision = self.dispatcher.submit(event)
        except Exception as exc:  # noqa: BLE001
            self._log(f"ACTION failed: {exc}")
            self._reply(conn, S.make_reply(event.event_id, False, reason="action_error"))
            return

        self._reply(
            conn,
            S.make_reply(
                event.event_id,
                decision.accepted,
                action=decision.action,
                reason=decision.reason,
            ),
        )

    def _reply(self, conn: socket.socket, obj) -> None:
        try:
            conn.sendall(S.encode_line(obj))
        except OSError:
            pass

    # ------------------------------------------------------------------ #
    def shutdown(self) -> None:
        self._stop.set()

    @property
    def ready(self) -> bool:
        return self._ready.is_set()

    def _close_socket(self) -> None:
        for sock in (self._client, self._sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        try:
            self.runtime.close()
        except Exception:  # noqa: BLE001
            pass


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="command_server",
        description="Microduck brain：触觉手势 TCP 命令服务器（默认 dry-run）",
    )
    p.add_argument("--host", default=DEFAULT_HOST, help="监听地址（默认 0.0.0.0）")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help="监听端口（默认 9000）")
    p.add_argument(
        "--enable-hardware",
        action="store_true",
        help="允许访问真实舵机总线；不加此选项则永远 dry-run",
    )
    p.add_argument("--port-name", default="/dev/ttyACM0", help="舵机串口")
    p.add_argument("--baud", type=int, default=1_000_000, help="舵机波特率")
    p.add_argument("--verbose", action="store_true", help="打印每一帧目标位置")
    p.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行指定秒数后退出（0 = 一直运行）",
    )
    return p


def main(argv: Optional[List[str]] = None) -> int:
    args = _build_parser().parse_args(argv)

    runtime = make_runtime(
        dry_run=not args.enable_hardware,
        port=args.port_name,
        baud=args.baud,
        verbose=args.verbose,
    )
    server = CommandServer(
        host=args.host,
        port=args.port,
        runtime=runtime,
        dry_run=not args.enable_hardware,
    )

    if args.duration > 0:
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        deadline = time.perf_counter() + args.duration
        while time.perf_counter() < deadline and not server._stop.is_set():
            time.sleep(0.05)
        server.shutdown()
        thread.join(timeout=2.0)
        return 0

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
