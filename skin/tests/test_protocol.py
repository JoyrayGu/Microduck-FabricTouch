"""协议层单元测试：全部用例取自《采集设备使用说明-蓝牙_V5.0》中的通讯实例。

运行:
    python -m unittest discover -s tests -v
"""

import os
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse import protocol as P
from tactile_mouse.config import AppConfig
from tactile_mouse.session import Session
from tactile_mouse.transport import MockTransport


class TestChecksumAndBuild(unittest.TestCase):
    """3.通讯实例 中的每一条指令帧，逐字节比对。"""

    def test_frame_format(self):
        raw = P.cmd_start_continuous()
        self.assertEqual(len(raw), 7)
        self.assertEqual(raw[0], 0x98)
        self.assertEqual(raw[-1], 0x81)
        self.assertEqual(P.checksum(raw[:-2]), raw[-2])

    def test_documented_commands(self):
        self.assertEqual(P.DEFAULT_ROWS, 32)
        self.assertEqual(P.DEFAULT_COLS, 32)
        cases = [
            ("3.1 连续采集", P.cmd_start_continuous(), "98 00 00 01 00 99 81"),
            ("3.2 单次采集", P.cmd_single_acquire(), "98 00 00 02 00 9A 81"),
            ("3.3 停止采集", P.cmd_stop(), "98 00 00 03 00 9B 81"),
            ("3.4 读取电量", P.cmd_read_battery(), "98 00 00 07 00 9F 81"),
            ("3.5 设置阵列", P.cmd_set_array(32, 32), "98 00 02 08 00 20 20 E2 81"),
            ("3.6 设置阈值", P.cmd_set_threshold(100), "98 00 01 0C 00 64 09 81"),
            ("3.7 发送模式", P.cmd_set_transport_mode(P.TransportMode.BLE),
             "98 00 01 0D 00 01 A7 81"),
            ("3.8 设备信息", P.cmd_device_info(), "98 00 00 0B 00 A3 81"),
        ]
        for name, built, expected in cases:
            with self.subTest(name):
                self.assertEqual(P.to_hex(built), expected)
                self.assertEqual(P.from_hex(expected), built)

    def test_uuid_constants(self):
        self.assertTrue(P.BLE_SERVICE_UUID.startswith("0000ffe0"))
        self.assertTrue(P.BLE_WRITE_UUID.startswith("0000ffe1"))
        self.assertTrue(P.BLE_NOTIFY_UUID.startswith("0000ffe2"))

    def test_session_never_sends_stop_command(self):
        """设备固件对 0x03 有缺陷：STOP 之后 START 只吐 1 帧就彻底停。

        实测（同一台设备，连续 6 轮）：
            只发 STOP -> START       : 1 帧
            配置(不含 STOP) -> START : 82~83 帧，稳定
        设备上电本来就不推流，所以启动与停止都不得下发 0x03。
        """
        transport = MockTransport()
        session = Session(transport=transport)
        session.start()
        self.assertNotIn(P.cmd_stop(), transport.sent_commands)
        self.assertEqual(transport.sent_commands[0], P.cmd_set_transport_mode(0))
        session.stop()
        self.assertNotIn(P.cmd_stop(), transport.sent_commands)

    def test_session_starts_with_optional_auto_calibration(self):
        cfg = AppConfig()
        cfg.auto_calibrate_on_start = True
        transport = MockTransport()
        session = Session(cfg=cfg, transport=transport)
        with patch.object(session, "recalibrate") as mock_recalibrate:
            session.start()
            mock_recalibrate.assert_called_once()
        session.stop()

    def test_param_validation(self):
        with self.assertRaises(P.ProtocolError):
            P.cmd_set_threshold(256)
        with self.assertRaises(P.ProtocolError):
            P.cmd_set_array(0, 7)
        with self.assertRaises(P.ProtocolError):
            P.cmd_set_transport_mode(2)


class TestStreamParser(unittest.TestCase):
    def test_device_reply_examples(self):
        """文档中的设备应答帧（状态字 0x01）应能全部解析。"""
        for text in [
            "98 00 00 03 01 9C 81",
            "98 00 02 08 01 07 07 B1 81",
            "98 00 01 0C 01 64 0A 81",
            "98 00 01 0D 01 01 A8 81",
        ]:
            with self.subTest(text):
                frames = list(P.iter_frames(P.from_hex(text)))
                self.assertEqual(len(frames), 1)
                self.assertEqual(frames[0].dev, P.DEV_SLAVE)

    def test_battery_reply(self):
        """3.4 应答示例（文档中校验字印刷为 DE，正确值应为 D5，此处两种都接受）。"""
        for chk in ("D5", "DE"):
            frames = list(P.iter_frames(P.from_hex(f"98 00 01 07 01 34 {chk} 81")))
            if chk == "D5":
                self.assertEqual(len(frames), 1)
                self.assertEqual(P.parse_battery(frames[0].data), 52)
            else:
                self.assertEqual(len(frames), 0)  # 印刷错误值会被校验拒绝

    def test_garbage_prefix_and_split(self):
        """模拟蓝牙通知前 N 字节为无效数据 + 跨包分片。"""
        payload = bytes(range(0x11, 0x21))  # 16 个点
        frame = P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE,
                              b"\x00\x2A" + payload)
        stream = b"\xAA\xBB\xCC\x98\x00" + frame + b"\x98\x81\x99"
        parser = P.StreamParser()
        out = []
        for i in range(0, len(stream), 3):  # 每 3 字节喂一次，强制分片
            out += parser.feed(stream[i : i + 3])
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].cmd, P.Command.CONTINUOUS)
        self.assertEqual(out[0].data, b"\x00\x2A" + payload)
        self.assertEqual(parser.pending_bytes, 0)

    def test_sticky_packets(self):
        """两个帧粘在一个包里应全部解析出来。"""
        blob = P.cmd_stop() + P.cmd_read_battery() + P.cmd_device_info()
        frames = list(P.iter_frames(blob))
        self.assertEqual([f.cmd for f in frames],
                         [P.Command.STOP, P.Command.READ_BATTERY, P.Command.DEVICE_INFO])

    def test_bad_checksum_rejected(self):
        bad = bytearray(P.cmd_stop())
        bad[-2] ^= 0xFF
        parser = P.StreamParser()
        self.assertEqual(parser.feed(bytes(bad)), [])
        self.assertEqual(parser.stats["bad_checksum"], 1)

    def test_oversized_length_not_hanging(self):
        parser = P.StreamParser()
        parser.feed(b"\x98\xFF\xFF" + b"\x00" * 32)
        self.assertLess(len(parser._buf), 64)


class TestPayloadParsing(unittest.TestCase):
    def test_parse_acquire_7x7(self):
        points = [i * 10 for i in range(49)]
        body = b"".join(bytes([(p >> 8) & 0xFF, p & 0xFF]) for p in points)
        frame_no, parsed = P.parse_acquire_payload(b"\x12\x34" + body, 7, 7)
        self.assertEqual(frame_no, 0x1234)
        self.assertEqual(parsed, points)
        self.assertEqual(len(parsed), 49)

    def test_truncated_frame_is_zero_padded(self):
        body = b"\x00\x0A\x00\x14"  # 只有 2 个点
        _, parsed = P.parse_acquire_payload(b"\x00\x01" + body, 7, 7)
        self.assertEqual(len(parsed), 49)
        self.assertEqual(parsed[:2], [10, 20])
        self.assertEqual(sum(parsed[2:]), 0)

    def test_ascii_and_array_reply(self):
        self.assertEqual(P.parse_ascii_info(b"device_id:1\r\nhw_version:1.0\x00"),
                         "device_id:1\r\nhw_version:1.0")
        self.assertEqual(P.parse_array_reply(b"\x20\x20"), (32, 32))
        self.assertEqual(P.parse_threshold_reply(b"\x64"), 100)


if __name__ == "__main__":
    unittest.main(verbosity=2)
