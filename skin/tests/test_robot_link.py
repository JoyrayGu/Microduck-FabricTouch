"""robot_link 的协议与传输测试。

两件事必须一直被验证：

1. **跨仓协议兼容**：skin 的 ``to_wire`` / ``encode_line`` 产出的字节，brain 的
   ``command_schema.parse_line`` 必须能解析。这是两端唯一的契约，一旦漂移，
   真机上会表现为「事件发出去但 brain 说 bad_json / bad_gesture」。
   （brain 仓库不在本工作区 git 内时，跳过对应的跨仓用例。）
2. **TCP 传输**：用一个本地假 brain 起 TCP 服务，验证 ``RobotLink`` 能连上、
   按行发送、并把应答回调出来。

运行::

    python -m unittest discover -s tests -v
"""

import json
import os
import socket
import sys
import threading
import time
import unittest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))

from tactile_mouse.config import RobotLinkConfig
from tactile_mouse.robot_gestures import (
    ContactMode,
    MotionMode,
    RobotGesture,
    RobotGestureEvent,
)
from tactile_mouse.robot_link import client as C
from tactile_mouse.robot_link import events as E


def sample_event(gesture=RobotGesture.STATIC_TOUCH, **overrides):
    kwargs = dict(gesture=gesture, timestamp=1.0, confidence=0.9)
    kwargs.update(overrides)
    return RobotGestureEvent(**kwargs)


class WireFormatTests(unittest.TestCase):
    def test_to_wire_has_required_fields(self):
        payload = E.to_wire(sample_event(), event_id="abc", timestamp_ms=1234)
        self.assertEqual(payload["version"], E.PROTOCOL_VERSION)
        self.assertEqual(payload["event_id"], "abc")
        self.assertEqual(payload["gesture"], "static_touch")
        self.assertEqual(payload["timestamp_ms"], 1234)
        self.assertIn("confidence", payload)
        self.assertEqual(payload["contact_mode"], "single")
        self.assertEqual(payload["motion_mode"], "unknown")
        self.assertEqual(payload["moving_contacts"], 0)
        self.assertEqual(payload["stationary_contacts"], 0)

    def test_timestamp_defaults_to_wall_clock(self):
        """默认 timestamp_ms 必须是**墙钟**毫秒，不能取自 event.timestamp。

        event.timestamp 源自 time.perf_counter()（系统启动后的单调秒数）。
        若把它乘 1000 上线，brain 的 check_freshness 会算出 9 个数量级的
        年龄差，把每条事件判为 expired 拒收 —— 实测修复前 age 约为
        1792030906287 ms（20 天）。
        """
        import time as _time

        before = int(_time.time() * 1000)
        payload = E.to_wire(sample_event(timestamp=1.5))
        after = int(_time.time() * 1000)
        self.assertGreaterEqual(payload["timestamp_ms"], before)
        self.assertLessEqual(payload["timestamp_ms"], after)
        self.assertNotEqual(payload["timestamp_ms"], 1500)

    def test_explicit_timestamp_is_respected(self):
        payload = E.to_wire(sample_event(timestamp=1.5), timestamp_ms=42)
        self.assertEqual(payload["timestamp_ms"], 42)

    def test_optional_fields_omitted_when_absent(self):
        payload = E.to_wire(sample_event())
        self.assertNotIn("direction", payload)
        self.assertNotIn("start", payload)
        self.assertNotIn("end", payload)

    def test_optional_fields_serialized(self):
        event = sample_event(direction="left_to_right", start=(1, 2), end=(3, 4))
        payload = E.to_wire(event)
        self.assertEqual(payload["direction"], "left_to_right")
        self.assertEqual(payload["start"], [1.0, 2.0])
        self.assertEqual(payload["end"], [3.0, 4.0])

    def test_encode_line_is_one_ndjson_line(self):
        line = E.encode_line(E.to_wire(sample_event()))
        self.assertTrue(line.endswith(b"\n"))
        self.assertEqual(line.count(b"\n"), 1)
        self.assertIsInstance(json.loads(line.decode("utf-8")), dict)

    def test_new_event_id_is_unique(self):
        self.assertNotEqual(E.new_event_id(), E.new_event_id())


class CrossRepoCompatibilityTests(unittest.TestCase):
    """skin 编码 -> brain 解析。"""

    @classmethod
    def setUpClass(cls):
        brain_src = os.path.abspath(
            os.path.join(
                os.path.dirname(os.path.dirname(_HERE)),
                "microduck_brain",
            )
        )
        cls.brain_available = os.path.isdir(os.path.join(brain_src, "microduck"))
        if cls.brain_available:
            sys.path.insert(0, brain_src)
            from microduck.src import command_schema as S  # noqa: E402

            cls.S = S

    def setUp(self):
        if not self.brain_available:
            self.skipTest("microduck_brain 不在同级目录，跳过跨仓协议校验")

    def test_brain_parses_skin_static_touch(self):
        line = E.encode_line(E.to_wire(sample_event(), event_id="x1")).decode("utf-8")
        event = self.S.parse_line(line)
        self.assertEqual(event.gesture, "static_touch")
        self.assertEqual(event.event_id, "x1")

    def test_brain_parses_every_skin_gesture(self):
        for g in (RobotGesture.STATIC_TOUCH, RobotGesture.DYNAMIC_TOUCH, RobotGesture.DYNAMIC_TOUCH):
            line = E.encode_line(E.to_wire(sample_event(g))).decode("utf-8")
            event = self.S.parse_line(line)
            self.assertEqual(event.gesture, g.value)

    def test_brain_accepts_skin_event_as_fresh(self):
        """回归：skin 生成的事件必须能通过 brain 的新鲜度校验。

        修复前 to_wire 用 event.timestamp（perf_counter 单调秒数）乘 1000
        作为 timestamp_ms，而 brain 用 time.time() 墙钟比较，年龄差达 20 天，
        所有事件都被判 expired。这个测试跨过真实的序列化边界，能拦住它。
        """
        import time as _time

        # 故意给一个 perf_counter 风格的时间戳，模拟真实识别事件。
        line = E.encode_line(E.to_wire(sample_event(timestamp=606929.3166))).decode("utf-8")
        event = self.S.parse_line(line)
        now_ms = int(_time.time() * 1000)
        try:
            self.S.check_freshness(event, now_ms, max_age_ms=self.S.MAX_EVENT_AGE_MS)
        except self.S.SchemaError as exc:  # noqa: BLE001
            self.fail(f"brain 拒收了 skin 事件: reason={exc.reason} {exc.detail}")

    def test_protocol_versions_match(self):
        self.assertEqual(E.PROTOCOL_VERSION, self.S.PROTOCOL_VERSION)

    def test_skin_gesture_names_subset_of_brain_allowlist(self):
        for g in RobotGesture:
            self.assertIn(g.value, self.S.ALLOWED_GESTURES)


class _FakeBrain:
    """一个极简的假 brain：收行、回一行 accepted。"""

    def __init__(self):
        self.received = []
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self.port = self._sock.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()

    def _serve(self):
        self._sock.settimeout(0.2)
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                conn.settimeout(0.2)
                buf = b""
                while not self._stop.is_set():
                    try:
                        data = conn.recv(4096)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not data:
                        break
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if not line.strip():
                            continue
                        self.received.append(line.decode("utf-8"))
                        conn.sendall(b'{"accepted": true, "reason": "ok"}\n')

    def stop(self):
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._thread.join(timeout=1.0)


class RobotLinkTcpTests(unittest.TestCase):
    def setUp(self):
        self.brain = _FakeBrain()
        self.brain.start()
        self.replies = []
        self.link = C.RobotLink(
            RobotLinkConfig(host="127.0.0.1", port=self.brain.port, connect_timeout=1.0, timeout=1.0),
            on_reply=self.replies.append,
        )
        self.link.start()

    def tearDown(self):
        self.link.stop()
        self.brain.stop()

    def _wait_for(self, predicate, timeout=2.0):
        end = time.time() + timeout
        while time.time() < end:
            if predicate():
                return True
            time.sleep(0.01)
        return False

    def test_event_reaches_fake_brain(self):
        self.link.send(sample_event())
        self.assertTrue(
            self._wait_for(lambda: len(self.brain.received) >= 1),
            "事件未在超时前到达假 brain",
        )
        payload = json.loads(self.brain.received[0])
        self.assertEqual(payload["gesture"], "static_touch")
        self.assertTrue(self.link.stats.sent >= 1)

    def test_reply_is_captured(self):
        self.link.send(sample_event())
        self.assertTrue(self._wait_for(lambda: bool(self.link.stats.last_reply)))
        self.assertTrue(self.link.stats.last_reply.get("accepted"))

    def test_multiple_events_sent_in_order(self):
        for gesture in (RobotGesture.STATIC_TOUCH, RobotGesture.DYNAMIC_TOUCH, RobotGesture.DYNAMIC_TOUCH):
            self.link.send(sample_event(gesture))
        self.assertTrue(self._wait_for(lambda: len(self.brain.received) >= 3))
        got = [json.loads(line)["gesture"] for line in self.brain.received[:3]]
        self.assertEqual(got, ["static_touch", "dynamic_touch", "dynamic_touch"])


class ReplyParsingTests(unittest.TestCase):
    """应答解析必须容忍 TCP 分片、多行粘连与空行。

    回归背景：实测出现过
        [brain] REJECTED reason=None event_id=-
    以及应答与事件错位（第 N 条的应答在下一次发送后才被读到）。
    根因是把单次 recv 的结果直接当完整 JSON 解析。
    """

    def _link(self, on_reply):
        return C.RobotLink(cfg=RobotLinkConfig(host="127.0.0.1", port=1),
                         on_reply=on_reply)

    def test_split_line_is_buffered_until_complete(self):
        """半行不应被解析成空字典；拼完后应正常解析。"""
        import socket as _socket

        class _FakeSock:
            def __init__(self, chunks):
                self._chunks = list(chunks)
                self._timeout = None

            def settimeout(self, t):
                self._timeout = t

            def recv(self, n):
                if self._chunks:
                    return self._chunks.pop(0)
                raise _socket.timeout()

        got = []
        link = C.RobotLink(cfg=RobotLinkConfig(host="127.0.0.1", port=1),
                         on_reply=got.append)
        sock = _FakeSock([
            b'{"event_id":"abc","acce',          # 半行
            b'pted":true,"action":"head_roll","reason":"ok"}\n',
        ])
        link._try_read_reply(sock)
        link._try_read_reply(sock)

        self.assertEqual(len(got), 1, f"应只解析出一条应答，实际 {got}")
        self.assertTrue(got[0]["accepted"])
        self.assertEqual(got[0]["event_id"], "abc")

    def test_blank_line_is_ignored(self):
        """空行不是应答，不能变成一个空的 reply 对象。"""
        import socket as _socket

        class _FakeSock:
            def __init__(self, chunks):
                self._chunks = list(chunks)

            def settimeout(self, t):
                pass

            def recv(self, n):
                if self._chunks:
                    return self._chunks.pop(0)
                raise _socket.timeout()

        got = []
        link = C.RobotLink(cfg=RobotLinkConfig(host="127.0.0.1", port=1),
                         on_reply=got.append)
        link._try_read_reply(_FakeSock([b"\n\n"]))
        self.assertEqual(got, [], "空行不应产生应答回调")

    def test_multiple_replies_in_one_recv(self):
        """一次收到多行时应逐行回调，不能只解析第一行。"""
        import socket as _socket

        class _FakeSock:
            def __init__(self, chunks):
                self._chunks = list(chunks)

            def settimeout(self, t):
                pass

            def recv(self, n):
                if self._chunks:
                    return self._chunks.pop(0)
                raise _socket.timeout()

        got = []
        link = C.RobotLink(cfg=RobotLinkConfig(host="127.0.0.1", port=1),
                         on_reply=got.append)
        payload = (b'{"event_id":"a","accepted":true,"action":"head_roll","reason":"ok"}\n'
                   b'{"event_id":"b","accepted":false,"reason":"busy"}\n')
        link._try_read_reply(_FakeSock([payload]))
        self.assertEqual([r["event_id"] for r in got], ["a", "b"])
        self.assertFalse(got[1]["accepted"])


class NoServerTests(unittest.TestCase):
    def test_send_does_not_raise_when_brain_unreachable(self):
        # 指向一个几乎肯定没人监听的端口：send 只入队，不应抛异常。
        link = C.RobotLink(
            RobotLinkConfig(host="127.0.0.1", port=1, connect_timeout=0.2, timeout=0.2, reconnect=False)
        )
        link.start()
        try:
            self.assertTrue(link.send(sample_event()))
            time.sleep(0.3)
            self.assertFalse(link.connected)
        finally:
            link.stop()


if __name__ == "__main__":
    unittest.main()
