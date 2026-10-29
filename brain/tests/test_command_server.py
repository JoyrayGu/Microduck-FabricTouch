# SPDX-License-Identifier: GPL-3.0-or-later
"""@file test_command_server.py
@brief TCP 命令服务器的网络管道测试。

现有测试覆盖「决策内核」（schema / library / dispatcher）；本文件专门覆盖
``microduck/src/command_server.py`` 里**只有服务器才有的**逻辑：

* ndjson 分帧：半包、粘包（一行两事件）、CRLF 结尾；
* 应答回写：合法事件 -> accepted，非法事件 -> 对应 reason 码；
* 错误分支：坏 JSON、未知手势、非法 UTF-8、空行、超长行；
* 连接生命周期：事件确实驱动了 runtime、重复 event_id 拒绝、断开后可重连；
* 进程入口：``main(["--duration", ...])`` 能干净退出并返回 0。

设计说明
--------
* 起一个**真实的** :class:`CommandServer`：回环地址 + 端口 0（OS 分配）+ dry-run，
  既不占用固定端口，也绝不接触舵机总线。
* dispatcher 在**调用线程同步执行**动作；为了不真的等待插值，测试注入一个
  **零时长**动作库——``_run_frames`` 对 ``duration <= 0`` 的关键帧只 emit 一帧、
  不 sleep，动作瞬间完成。
* 事件时间戳取自真实时钟，以满足 dispatcher 的时效校验（默认 2s）。

运行::

    python -m unittest discover -s tests -v
"""

from __future__ import annotations

import contextlib
import io
import json
import os
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from microduck.src import command_schema as S  # noqa: E402
from microduck.src.action_library import (  # noqa: E402
    Action,
    ActionLibrary,
    Keyframe,
)
from microduck.src.command_server import CommandServer  # noqa: E402
from microduck.src.servo_runtime import DryRunRuntime  # noqa: E402


# --------------------------------------------------------------------------- #
# 测试夹具
# --------------------------------------------------------------------------- #
def _instant_library() -> ActionLibrary:
    """一个动作瞬间完成的仓库，避免测试真的等待插值。

    三种手势都映射到同一个零时长头部动作 ``head_roll``；这样测的是**服务器
    管道**，而不是动作本身（动作语义已在 ``test_action_library.py`` 覆盖）。
    """
    action = Action(
        name="head_roll",
        frames=[Keyframe(pose={13: 2200, 14: 2048}, duration=0.0)],
        timeout_s=1.0,
        description="测试用：零时长头部动作（id 13/14）",
    )
    return ActionLibrary(
        actions={"head_roll": action},
        gesture_map={
            "static_touch": "head_roll",
            "dynamic_touch": "head_roll",
            "dynamic_touch": "head_roll",
        },
    )


def _event(gesture: str = "static_touch", event_id: str = "evt-1", **overrides) -> dict:
    payload = {
        "version": S.PROTOCOL_VERSION,
        "event_id": event_id,
        "gesture": gesture,
        "timestamp_ms": int(time.time() * 1000),
        "confidence": 0.9,
    }
    payload.update(overrides)
    return payload


def _line(obj) -> bytes:
    return (json.dumps(obj, ensure_ascii=False) + "\n").encode("utf-8")


class _Client:
    """带缓冲的行读取器：把一行事件发出去、把一行应答读回来。"""

    def __init__(self, conn: socket.socket):
        self.conn = conn
        self._buf = bytearray()

    def send(self, data: bytes) -> None:
        self.conn.sendall(data)

    def send_event(self, **kwargs) -> dict:
        payload = _event(**kwargs)
        self.send(_line(payload))
        return payload

    def read_reply(self) -> dict:
        while b"\n" not in self._buf:
            chunk = self.conn.recv(1024)
            if not chunk:
                raise AssertionError("连接在收到应答前被关闭")
            self._buf += chunk
        raw, _, rest = self._buf.partition(b"\n")
        self._buf = bytearray(rest)
        return json.loads(raw.decode("utf-8"))

    def close(self) -> None:
        try:
            self.conn.close()
        except OSError:
            pass


class _ServerTestCase(unittest.TestCase):
    """起一个真实的 CommandServer（回环 + 随机端口 + dry-run）。"""

    def setUp(self):
        self.runtime = DryRunRuntime()
        self.server = CommandServer(
            host="127.0.0.1",
            port=0,  # 交给 OS 分配空闲端口，避免固定端口冲突
            library=_instant_library(),
            runtime=self.runtime,
            dry_run=True,
            log=lambda _msg: None,  # 测试时静音
        )
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

        deadline = time.perf_counter() + 3.0
        while not self.server.ready and time.perf_counter() < deadline:
            time.sleep(0.01)
        self.assertTrue(self.server.ready, "服务器未能在 3 秒内就绪")

        self.port = self.server._sock.getsockname()[1]
        self._clients: list[_Client] = []

    def tearDown(self):
        for client in self._clients:
            client.close()
        self.server.shutdown()
        self.thread.join(timeout=3.0)
        self.assertFalse(self.thread.is_alive(), "服务器线程未能退出")

    # -- helpers ------------------------------------------------------- #
    def connect(self) -> _Client:
        conn = socket.create_connection(("127.0.0.1", self.port), timeout=2.0)
        conn.settimeout(2.0)
        client = _Client(conn)
        self._clients.append(client)
        return client


# --------------------------------------------------------------------------- #
# 1) 正常应答
# --------------------------------------------------------------------------- #
class ReplyTests(_ServerTestCase):
    def test_valid_event_accepted(self):
        client = self.connect()
        event = client.send_event(gesture="static_touch", event_id="ok-1")
        reply = client.read_reply()
        self.assertTrue(reply["accepted"])
        self.assertEqual(reply["action"], "head_roll")
        self.assertEqual(reply["reason"], "ok")
        self.assertEqual(reply["event_id"], event["event_id"])

    def test_every_allowed_gesture_maps_through_library(self):
        # 服务器是单客户端（accept -> 处理到断开才回来），所以复用同一条连接。
        client = self.connect()
        for i, gesture in enumerate(S.ALLOWED_GESTURES):
            client.send_event(gesture=gesture, event_id=f"g-{i}")
            reply = client.read_reply()
            self.assertTrue(reply["accepted"], f"{gesture} 应被接受")
            self.assertEqual(reply["action"], "head_roll")

    def test_reply_is_valid_utf8_json_line(self):
        client = self.connect()
        client.send_event(event_id="enc-1")
        reply = client.read_reply()  # 解析成功即说明是合法 UTF-8 JSON
        self.assertIsInstance(reply, dict)


# --------------------------------------------------------------------------- #
# 2) 分帧（半包 / 粘包 / CRLF）
# --------------------------------------------------------------------------- #
class FramingTests(_ServerTestCase):
    def test_split_across_two_packets(self):
        client = self.connect()
        raw = _line(_event(event_id="split-1"))
        half = len(raw) // 2
        client.send(raw[:half])
        time.sleep(0.1)  # 让服务器先处理半包
        client.send(raw[half:])
        reply = client.read_reply()
        self.assertEqual(reply["event_id"], "split-1")
        self.assertTrue(reply["accepted"])

    def test_two_events_in_one_packet(self):
        client = self.connect()
        client.send(_line(_event(event_id="a")) + _line(_event(event_id="b")))
        first = client.read_reply()
        second = client.read_reply()
        self.assertEqual({first["event_id"], second["event_id"]}, {"a", "b"})
        self.assertTrue(first["accepted"] and second["accepted"])

    def test_crlf_terminator_is_tolerated(self):
        client = self.connect()
        client.send(json.dumps(_event(event_id="crlf")).encode("utf-8") + b"\r\n")
        reply = client.read_reply()
        self.assertEqual(reply["event_id"], "crlf")
        self.assertTrue(reply["accepted"])


# --------------------------------------------------------------------------- #
# 3) 错误分支
# --------------------------------------------------------------------------- #
class ErrorTests(_ServerTestCase):
    def test_bad_json_rejected(self):
        client = self.connect()
        client.send(b"{not json}\n")
        reply = client.read_reply()
        self.assertFalse(reply["accepted"])
        self.assertEqual(reply["reason"], "bad_json")
        self.assertIsNone(reply["event_id"])

    def test_unknown_gesture_rejected(self):
        client = self.connect()
        client.send(_line(_event(gesture="kick")))
        reply = client.read_reply()
        self.assertFalse(reply["accepted"])
        self.assertEqual(reply["reason"], "bad_gesture")

    def test_invalid_utf8_rejected(self):
        client = self.connect()
        client.send(b"\xff\xfe\n")
        reply = client.read_reply()
        self.assertFalse(reply["accepted"])
        self.assertEqual(reply["reason"], "bad_encoding")

    def test_empty_line_rejected(self):
        client = self.connect()
        client.send(b"\n")
        reply = client.read_reply()
        self.assertEqual(reply["reason"], "empty")

    def test_oversized_line_rejected(self):
        client = self.connect()
        big = json.dumps({"event_id": "x" * 6000})
        self.assertGreater(len(big.encode("utf-8")), S.MAX_EVENT_BYTES)
        client.send(big.encode("utf-8") + b"\n")
        reply = client.read_reply()
        self.assertEqual(reply["reason"], "too_large")

    def test_error_then_valid_recovers(self):
        # 一个坏包不应污染后续分帧：下一条合法事件仍能正常处理。
        client = self.connect()
        client.send(b"garbage\n")
        self.assertEqual(client.read_reply()["reason"], "bad_json")
        client.send(_line(_event(event_id="after-bad")))
        reply = client.read_reply()
        self.assertTrue(reply["accepted"])
        self.assertEqual(reply["event_id"], "after-bad")


# --------------------------------------------------------------------------- #
# 4) 生命周期 / 集成
# --------------------------------------------------------------------------- #
class LifecycleTests(_ServerTestCase):
    def test_accepted_event_drives_runtime(self):
        # 端到端：TCP -> schema -> dispatcher -> DryRunRuntime
        client = self.connect()
        frames_before = self.runtime.status.frames
        client.send_event(event_id="drive-1")
        self.assertTrue(client.read_reply()["accepted"])
        self.assertEqual(self.runtime.status.enabled_ids, [13, 14])
        self.assertGreater(self.runtime.status.frames, frames_before)
        self.assertTrue(self.runtime.log)
        self.assertEqual(set(self.runtime.log[-1].targets), {13, 14})

    def test_duplicate_event_id_rejected(self):
        client = self.connect()
        client.send(_line(_event(event_id="dup")))
        first = client.read_reply()
        client.send(_line(_event(event_id="dup")))
        second = client.read_reply()
        self.assertTrue(first["accepted"])
        self.assertFalse(second["accepted"])
        self.assertEqual(second["reason"], "duplicate")

    def test_rejects_after_client_disconnects_then_accepts_new(self):
        first = self.connect()
        first.send(_line(_event(event_id="one")))
        self.assertTrue(first.read_reply()["accepted"])
        first.close()
        time.sleep(0.2)  # 让服务器回到 accept

        second = self.connect()
        second.send(_line(_event(event_id="two")))
        self.assertTrue(second.read_reply()["accepted"])


# --------------------------------------------------------------------------- #
# 5) 进程入口
# --------------------------------------------------------------------------- #
class MainEntrypointTests(unittest.TestCase):
    def test_duration_exits_zero(self):
        # 默认 dry-run；--port 0 让 OS 分配端口；0.3 秒后应干净退出并返回 0。
        from microduck.src import command_server as CS

        with contextlib.redirect_stdout(io.StringIO()):
            rc = CS.main(["--duration", "0.3", "--port", "0"])
        self.assertEqual(rc, 0)


class StepFailureReportingTests(unittest.TestCase):
    """硬件写入失败必须上报，不能伪装成 outcome=done。

    回归背景：实测出现过 ID14 连续 23 帧写入失败（no status packet!），
    舵机一步都没动，日志却打印 ``ACTION_END head_roll outcome=done``。
    排查时极易误导——看起来动作成功，实际硬件完全没响应。
    """

    def _server_with(self, runtime, logs):
        server = CommandServer(
            host="127.0.0.1",
            port=0,
            library=_instant_library(),
            runtime=runtime,
            dry_run=True,
            log=logs.append,
        )
        return server

    def test_step_failure_marks_outcome_error(self):
        logs = []

        class FailingRuntime:
            name = "failing"

            def prepare(self, ids):
                pass

            def step(self, targets):
                raise RuntimeError("ID14 write4@116: no status packet!")

            def rest(self):
                pass

            def close(self):
                pass

        server = self._server_with(FailingRuntime(), logs)
        # 写入失败会向上抛出（这样 on_action_end 才能拿到 outcome="error"），
        # 测试里显式捕获，同时校验日志内容。
        with self.assertRaises(RuntimeError):
            server.dispatcher.submit(S.parse_event(_event()))

        joined = "\n".join(logs)
        self.assertIn("STEP error", joined)
        self.assertIn("outcome=error", joined)
        self.assertNotIn("outcome=done", joined)
        # 失败次数应在结束行汇总
        self.assertIn("step_failures=", joined)

    def test_step_failure_stops_the_action(self):
        """首次写入失败应立即中止，不再下发剩余帧。

        继续向一个已经不响应的总线刷指令是危险的：舵机可能处于异常状态，
        再叠加指令只会让问题更难判断。
        """
        logs = []
        calls = {"n": 0}

        class FailOnFirst:
            name = "fail_once"

            def prepare(self, ids):
                pass

            def step(self, targets):
                calls["n"] += 1
                raise RuntimeError("boom")

            def rest(self):
                pass

            def close(self):
                pass

        multi = Action(
            name="head_roll",
            frames=[Keyframe(pose={13: 2100}, duration=0.25)],
            timeout_s=2.0,
        )
        server = CommandServer(
            host="127.0.0.1", port=0,
            library=ActionLibrary(actions={"head_roll": multi},
                                  gesture_map={"static_touch": "head_roll",
                                               "dynamic_touch": "head_roll",
                                               "dynamic_touch": "head_roll"}),
            runtime=FailOnFirst(),
            dry_run=True,
            log=logs.append,
        )
        with self.assertRaises(RuntimeError):
            server.dispatcher.submit(S.parse_event(_event()))

        step_lines = [m for m in logs if m.startswith("STEP error")]
        self.assertEqual(len(step_lines), 1, f"只应记录一次失败: {step_lines}")
        self.assertEqual(calls["n"], 1, "首次失败后就应中止，只调用一次 step()")


class ConnectionSurvivesFailureTests(unittest.TestCase):
    """执行期异常不能关闭客户端连接。

    dispatcher 会把硬件异常向上抛（以便 on_action_end 拿到 error），
    若 _handle_line 不捕获，异常会冒到 _handle_client 的循环外，
    整个连接被关闭，之后所有事件都收不到。
    """

    def test_server_keeps_serving_after_action_error(self):
        logs = []

        class FailOnce:
            name = "fail_once"

            def __init__(self):
                self.calls = 0

            def prepare(self, ids):
                pass

            def step(self, targets):
                self.calls += 1
                if self.calls == 1:
                    raise RuntimeError("first step fails")

            def rest(self):
                pass

            def close(self):
                pass

        runtime = FailOnce()
        server = CommandServer(
            host="127.0.0.1", port=0,
            library=_instant_library(),
            runtime=runtime,
            dry_run=True,
            log=logs.append,
        )
        # 第 1 个事件：动作失败（但仍应 produce 应答）
        d1 = None
        try:
            d1 = server.dispatcher.submit(S.parse_event(_event(event_id="e1")))
        except Exception:  # noqa: BLE001
            pass
        # 第 2 个事件：应当仍能处理（证明状态已恢复）
        d2 = server.dispatcher.submit(S.parse_event(_event(event_id="e2")))
        self.assertTrue(d2.accepted, "失败的动作不应让后续事件无法执行")
        self.assertFalse(server.dispatcher.busy, "失败后必须复位 running 状态")


if __name__ == "__main__":
    unittest.main()
