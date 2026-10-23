"""USB 通道单元测试。

覆盖《采集设备使用说明-蓝牙_V5.0》里 USB 相关的条款：
  * 3.7 数据传输模式指令（0x0D：0x01=蓝牙 / 0x00=USB，掉电回蓝牙）；
  * 3.2 注 2：USB 单次采集返回 ASCII 明文（32 行 x 32 点、空格分隔、
    3 位小数、单位 kPa）。

这些测试都不需要真实硬件，也不需要 pyserial（串口模块是延迟导入的）。
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tactile_mouse import protocol as P
from tactile_mouse.config import AppConfig
from tactile_mouse.session import make_transport, normalize_kind
from tactile_mouse.transport import (
    TransportError,
    UsbChannelTransport,
    UsbStreamDecoder,
    UsbTransport,
)


def ascii_text(cols: int = 32, lines: int = 32, value: float = 1.5,
               final_newline: bool = True) -> str:
    """按文档格式造一块 ASCII 明文压力数据。"""
    rows = [" ".join(f"{value:7.3f}" for _ in range(cols)) for _ in range(lines)]
    text = "\n".join(rows)
    return text + "\n" if final_newline else text


class TestUsbModeCommand(unittest.TestCase):
    """3.7 数据传输模式指令。"""

    def test_usb_frame_bytes(self):
        """参数 00 = USB：校验和应为 0xA6。"""
        self.assertEqual(P.to_hex(P.cmd_set_transport_mode(P.TransportMode.USB)),
                         "98 00 01 0D 00 00 A6 81")
        self.assertEqual(P.to_hex(P.cmd_set_usb_mode()), "98 00 01 0D 00 00 A6 81")

    def test_ble_frame_matches_document(self):
        """参数 01 = 蓝牙：文档 3.7 示例逐字节一致。"""
        self.assertEqual(P.to_hex(P.cmd_set_transport_mode(P.TransportMode.BLE)),
                         "98 00 01 0D 00 01 A7 81")

    def test_usb_is_default(self):
        """本程序默认走 USB，且 device 指令默认参数也是 USB。"""
        self.assertEqual(P.DEFAULT_TRANSPORT_MODE, P.TransportMode.USB)
        self.assertEqual(P.cmd_set_transport_mode(), P.cmd_set_usb_mode())
        cfg = AppConfig()
        self.assertEqual(cfg.device_mode, P.TransportMode.USB)
        self.assertEqual(cfg.transport_kind, "usb")

    def test_mode_naming(self):
        self.assertEqual(P.TransportMode.name_of(0), "USB")
        self.assertEqual(P.TransportMode.name_of(1), "蓝牙")
        self.assertTrue(P.TransportMode.valid(0))
        self.assertFalse(P.TransportMode.valid(2))

    def test_reply_frame_parses(self):
        """设备应答 98 00 01 0D 01 00 A7 81 能解析出模式 0x00。"""
        frames = list(P.iter_frames(P.from_hex("98 00 01 0D 01 00 A7 81")))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].data[0], P.TransportMode.USB)


class TestLargeArray(unittest.TestCase):
    """32x32 阵列的帧长 2057 字节，必须能通过解析。"""

    def test_max_payload_covers_32x32(self):
        self.assertGreater(P.MAX_PAYLOAD_LEN, 2 + 32 * 32 * 2)

    def test_parse_32x32_frame(self):
        points = [i % 65535 for i in range(32 * 32)]
        body = b"".join(bytes([(p >> 8) & 0xFF, p & 0xFF]) for p in points)
        raw = P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, b"\x00\x2A" + body)
        self.assertEqual(len(raw), 2057)
        frames = list(P.iter_frames(raw))
        self.assertEqual(len(frames), 1)
        frame_no, parsed = P.parse_acquire_payload(frames[0].data, 32, 32)
        self.assertEqual(frame_no, 42)
        self.assertEqual(parsed, points)


class TestResample(unittest.TestCase):
    def test_identity(self):
        src = [float(i) for i in range(9)]
        self.assertEqual(P.resample_grid(src, 3, 3, 3, 3), src)

    def test_downsample_averages_not_sums(self):
        """降采样取的是「每格平均压力」而不是「求和」，所以总和会随格数变化。

        这一点很关键：activate_abs 这类单格阈值不受缩放影响，
        而 heavy_total 这类按整场总和比较的阈值必须按实测调整。
        """
        src = [10.0] * (32 * 32)
        out = P.resample_grid(src, 32, 32, 7, 7)
        self.assertEqual(len(out), 49)
        # 均值保持一致（每格压力含义不变）
        self.assertAlmostEqual(sum(out) / len(out), sum(src) / len(src), places=6)
        # 总和会缩小到约 49/1024
        self.assertAlmostEqual(sum(out) / sum(src), 49 / 1024, places=6)

    def test_downsample_averages_nonzero(self):
        src = [1.0] * (32 * 32)
        src[0] = 100.0  # 左上角一格
        out = P.resample_grid(src, 32, 32, 7, 7)
        self.assertGreater(out[0], out[-1])

    def test_upsample_2x2_to_4x4(self):
        out = P.resample_grid([1.0, 2.0, 3.0, 4.0], 2, 2, 4, 4)
        self.assertEqual(out, [1.0, 1.0, 2.0, 2.0,
                               1.0, 1.0, 2.0, 2.0,
                               3.0, 3.0, 4.0, 4.0,
                               3.0, 3.0, 4.0, 4.0])


class TestAsciiNumericLine(unittest.TestCase):
    def test_accepts_numbers(self):
        for tok in ("0.123", "1.004", "-2.5", "+3e-2", ".5", "1000"):
            self.assertTrue(P.looks_numeric_token(tok), tok)

    def test_rejects_device_info_and_junk(self):
        """设备信息 / 垃圾 token 不能被当成压力数据。"""
        for tok in ("device_id:1", "hw_version:1.0", "oops", "", "1_0", "nan", "inf"):
            self.assertFalse(P.looks_numeric_token(tok), tok)

    def test_numeric_tail_tolerates_leading_noise(self):
        """串口残留会和第一行数据粘在一起，不能因此丢掉整行。"""
        tokens = ["garbage0", "0.5", "0.6", "0.7", "0.8"]
        self.assertEqual(P.numeric_tail(tokens, min_width=4), [0.5, 0.6, 0.7, 0.8])

    def test_numeric_tail_rejects_info_lines(self):
        self.assertIsNone(P.numeric_tail(["device_id:1"], min_width=4))
        self.assertIsNone(P.numeric_tail(["hw_version:1.0", "1.0"], min_width=4))
        # 噪声 token 太多 -> 不是数据行
        self.assertIsNone(
            P.numeric_tail(["a", "b", "c", "0.5", "0.6", "0.7", "0.8"], min_width=4)
        )
        # 数字段里夹着非数字 -> 不是数据行
        self.assertIsNone(
            P.numeric_tail(["0.5", "0.6", "oops", "0.8"], min_width=4)
        )


class TestAsciiTelemetryParser(unittest.TestCase):
    def test_32x32_block_resampled_to_configured_grid(self):
        parser = P.AsciiTelemetryParser(rows=7, cols=7, scale=100.0)
        blocks, keep = parser.feed_text(ascii_text())
        self.assertEqual(len(blocks), 1)
        self.assertEqual(keep, "")
        block = blocks[0]
        self.assertEqual(block.native_shape, (32, 32))
        self.assertEqual((block.rows, block.cols), (7, 7))
        self.assertFalse(block.partial)
        self.assertEqual(len(block.values), 49)
        self.assertAlmostEqual(block.values[0], 150.0, places=3)  # 1.5 kPa x100
        self.assertEqual(parser.stats["resampled"], 1)

    def test_native_7x7_no_resample(self):
        parser = P.AsciiTelemetryParser(rows=7, cols=7, scale=1.0)
        text = "\n".join(" ".join(f"{r + c:5.2f}" for c in range(7)) for r in range(7)) + "\n"
        blocks, _ = parser.feed_text(text)
        self.assertEqual(len(blocks), 1)
        block = blocks[0]
        self.assertEqual((block.src_rows, block.src_cols), (7, 7))
        self.assertEqual(parser.stats["resampled"], 0)
        self.assertAlmostEqual(block.values[0], 0.0)
        self.assertAlmostEqual(block.values[6], 6.0)
        self.assertAlmostEqual(block.values[7], 1.0)

    def test_split_across_chunks(self):
        """明文被切成任意碎片喂入也要能拼回来。"""
        parser = P.AsciiTelemetryParser(rows=7, cols=7)
        text = ascii_text()
        got = []
        for i in range(0, len(text), 97):
            blocks, _ = parser.feed_text(text[i : i + 97])
            got.extend(blocks)
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0].native_shape, (32, 32))

    def test_missing_final_newline_still_completes(self):
        """很多设备最后一行不带换行，不能因此丢掉整块。"""
        parser = P.AsciiTelemetryParser(rows=7, cols=7, timeout=0.0)
        lines = ascii_text().rstrip("\n").split("\n")
        text = "\n".join(lines[:-1]) + "\n" + lines[-1]
        blocks, _ = parser.feed_text(text, now=1.0)
        self.assertEqual(blocks, [])
        block, _ = parser.flush(now=1.0)
        self.assertIsNotNone(block)
        self.assertFalse(block.partial)
        self.assertEqual(block.native_shape, (32, 32))

    def test_incomplete_block_abandoned_by_default(self):
        """只收到 5 行（不够 32 行）时默认丢弃，而不是给出一块错误的压力场。"""
        parser = P.AsciiTelemetryParser(rows=7, cols=7, timeout=0.0)
        blocks, _ = parser.feed_text(ascii_text(lines=5), now=1.0)
        self.assertEqual(blocks, [])
        block, _ = parser.flush(now=1.0)
        self.assertIsNone(block)
        self.assertEqual(parser.stats["abandoned"], 1)

    def test_incomplete_block_kept_when_allowed(self):
        parser = P.AsciiTelemetryParser(rows=7, cols=7, timeout=0.0, allow_partial=True)
        parser.feed_text(ascii_text(lines=5), now=1.0)
        block, _ = parser.flush(now=1.0)
        self.assertIsNotNone(block)
        self.assertTrue(block.partial)
        self.assertEqual(block.native_shape, (32, 32))
        self.assertEqual(len(block.values), 49)

    def test_flush_waits_while_still_receiving(self):
        parser = P.AsciiTelemetryParser(rows=7, cols=7, timeout=0.5)
        parser.feed_text(ascii_text(lines=4), now=100.0)
        block, _ = parser.flush(now=100.2)  # 还没超时
        self.assertIsNone(block)
        self.assertEqual(parser.pending_lines, 4)
        self.assertEqual(parser.stats["abandoned"], 0)

    def test_line_width_change_restarts_block(self):
        """行宽变化说明换了新的一块，旧的作废。"""
        parser = P.AsciiTelemetryParser(rows=7, cols=7, timeout=0.0)
        parser.feed_text(ascii_text(cols=16, lines=4), now=1.0)
        self.assertEqual(parser.native_width, 16)
        self.assertEqual(parser.stats["blocks"], 0)  # 16 列只来了 4 行，不成块
        parser.feed_text(ascii_text(cols=8, lines=8), now=1.0)
        self.assertEqual(parser.stats["blocks"], 1)  # 8 列 8 行 -> 成块
        self.assertEqual(parser.native_width, 0)  # 成块后已复位

    def test_non_numeric_lines_passthrough(self):
        """设备信息等明文要原样透出，且不干扰压力块的累积。"""
        parser = P.AsciiTelemetryParser(rows=7, cols=7)
        text = "device_id:1\r\nhw_version:1.0\r\n" + ascii_text()
        blocks, keep = parser.feed_text(text)
        self.assertEqual(len(blocks), 1)
        self.assertIn("device_id:1", keep)
        self.assertIn("hw_version:1.0", keep)

    def test_parse_ascii_telemetry_helper(self):
        block = P.parse_ascii_telemetry(ascii_text(), 7, 7, scale=1.0)
        self.assertIsNotNone(block)
        self.assertEqual(len(block.values), 49)
        self.assertAlmostEqual(block.values[0], 1.5)


class TestUsbStreamDecoder(unittest.TestCase):
    """USB 通道是「二进制帧 + ASCII 明文」的混合流。"""

    def test_ascii_block_becomes_acquire_frame(self):
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7, scale=100.0)
        decoder.feed(ascii_text().encode("ascii"))
        frames = list(P.iter_frames(b"".join(out)))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].cmd, P.Command.CONTINUOUS)
        self.assertEqual(frames[0].dev, P.DEV_SLAVE)
        self.assertEqual(decoder.mode, "ascii")
        frame_no, points = P.parse_acquire_payload(frames[0].data, 7, 7)
        self.assertEqual(len(points), 49)
        self.assertGreater(sum(points), 0)

    def test_ascii_and_binary_reply_coexist(self):
        """明文块与指令应答帧混在一起时都要恢复出来。"""
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7)
        decoder.feed(ascii_text().encode("ascii"))
        decoder.feed(b"\xAA\xBB" + P.cmd_set_usb_mode())  # 噪声 + 二进制应答
        frames = list(P.iter_frames(b"".join(out)))
        cmds = [f.cmd for f in frames]
        self.assertIn(P.Command.CONTINUOUS, cmds)
        self.assertIn(P.Command.SET_MODE, cmds)

    def test_chunked_and_noisy(self):
        """逐字节喂入 + 夹杂噪声。"""
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7)
        blob = b"\x00\x98\xFF" + ascii_text().encode("ascii")
        for i in range(len(blob)):
            decoder.feed(blob[i : i + 1])
        frames = list(P.iter_frames(b"".join(out)))
        self.assertEqual(len(frames), 1)
        self.assertEqual(decoder.mode, "ascii")

    def test_binary_latches_and_text_passes_through(self):
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7)
        payload = b"\x00\x01" + b"\x0A\x00" * 49
        decoder.feed(P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, payload))
        self.assertEqual(decoder.mode, "binary")
        decoder.feed(b"hello device\r\n")
        raw = b"".join(out)
        self.assertIn(b"hello device", raw)
        self.assertEqual(len(list(P.iter_frames(raw))), 1)

    def test_empty_ack_does_not_lock_binary(self):
        """0x01/0x02 的空载荷应答只是「执行成功」，不能判定为二进制通道。"""
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7)
        decoder.feed(P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, b""))
        self.assertIsNone(decoder.mode)
        decoder.feed(ascii_text().encode("ascii"))
        self.assertEqual(decoder.mode, "ascii")

    def test_empty_ack_does_not_count_as_data_frame(self):
        """同样地，Session 也不应把空载荷应答算成一帧采集数据。"""
        from tactile_mouse.mouse_backend import NullBackend
        from tactile_mouse.session import Session

        cfg = AppConfig()
        cfg.filter.baseline_frames = 0
        session = Session(
            cfg,
            transport=UsbTransport(port="COM_NONE"),
            backend=NullBackend(),
            on_status=lambda m: None,
        )
        try:
            session._on_raw(P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, b""))
            self.assertEqual(session.info.data_frames, 0)
            self.assertEqual(session.info.raw_frames, 1)
            session._on_raw(
                P.build_frame(P.Command.CONTINUOUS, P.DEV_SLAVE, b"\x00\x01" + b"\x00" * 98)
            )
            self.assertEqual(session.info.data_frames, 1)
        finally:
            session.stop()

    def test_forced_binary_format_ignores_text(self):
        out = []
        decoder = UsbStreamDecoder(out.append, rows=7, cols=7, stream_format="binary")
        decoder.feed(ascii_text().encode("ascii"))
        self.assertEqual(decoder.mode, None)
        self.assertEqual(list(P.iter_frames(b"".join(out))), [])

    def test_poll_reports_abandoned_block(self):
        messages = []
        out = []
        decoder = UsbStreamDecoder(
            out.append, rows=7, cols=7, ascii_timeout=0.0, on_status=messages.append
        )
        decoder.feed(ascii_text(lines=4).encode("ascii"), now=1.0)
        decoder.poll(now=1.0)
        self.assertEqual(decoder.ascii_stats["abandoned"], 1)
        self.assertTrue(any("凑不满" in m for m in messages))


class TestUsbChannelTransport(unittest.TestCase):
    """把模拟设备包成 USB 通道后，Session 应该照常工作。"""

    def test_mock_ascii_end_to_end(self):
        import time

        from tactile_mouse.mouse_backend import NullBackend
        from tactile_mouse.session import Session

        cfg = AppConfig()
        cfg.filter.baseline_frames = 0
        transport = make_transport(cfg, kind="mock", mock_mode="usb-ascii",
                                   on_status=lambda m: None)
        self.assertIsInstance(transport, UsbChannelTransport)
        backend = NullBackend()
        session = Session(cfg, transport=transport, backend=backend, on_status=lambda m: None)
        session.start()
        time.sleep(2.0)
        session.stop()
        self.assertEqual(transport.channel_mode, "ascii")
        self.assertGreater(session.info.data_frames, 5)
        self.assertEqual(session.info.dropped_frames, 0)
        self.assertTrue(any(a.startswith("move_rel") for a in backend.actions))


class TestTransportFactory(unittest.TestCase):
    def test_normalize_kind(self):
        for text in ("usb", "USB", "serial", "COM", "uart", "cdc", "", "auto", None, "default"):
            self.assertEqual(normalize_kind(text), "usb", text)
        for text in ("ble", "BLE", "bluetooth", "bt", "gatt"):
            self.assertEqual(normalize_kind(text), "ble", text)
        for text in ("mock", "sim", "fake", "demo"):
            self.assertEqual(normalize_kind(text), "mock", text)
        self.assertEqual(normalize_kind("ble", mock=True), "mock")
        with self.assertRaises(TransportError):
            normalize_kind("bluetoothxyz")

    def test_default_kind_is_usb(self):
        cfg = AppConfig()
        transport = make_transport(cfg, on_status=lambda m: None)
        self.assertIsInstance(transport, UsbTransport)
        self.assertEqual(transport.name, "usb")

    def test_usb_params_come_from_config(self):
        cfg = AppConfig()
        cfg.usb.port = "COM9"
        cfg.usb.baudrate = 921600
        cfg.usb.format = "ascii"
        cfg.filter.rows, cfg.filter.cols = 7, 7
        transport = make_transport(cfg, on_status=lambda m: None)
        self.assertEqual(transport.port, "COM9")
        self.assertEqual(transport.baudrate, 921600)
        self.assertEqual(transport._decoder.stream_format, "ascii")

    def test_ble_kind(self):
        cfg = AppConfig()
        transport = make_transport(cfg, kind="ble", name="TT-Skin", on_status=lambda m: None)
        self.assertEqual(transport.name, "ble")
        self.assertEqual(transport.device_name, "TT-Skin")


class TestConfigRoundTrip(unittest.TestCase):
    def test_usb_section_survives_json(self):
        cfg = AppConfig()
        cfg.usb.port = "COM7"
        cfg.usb.baudrate = 115200
        cfg.usb.allow_partial = True
        cfg.usb.ascii_scale = 50.0
        cfg.transport_kind = "usb"
        cfg.device_mode = P.TransportMode.USB

        clone = AppConfig.from_dict(cfg.to_dict())
        self.assertEqual(clone.usb.port, "COM7")
        self.assertEqual(clone.usb.baudrate, 115200)
        self.assertTrue(clone.usb.allow_partial)
        self.assertEqual(clone.usb.ascii_scale, 50.0)
        self.assertEqual(clone.transport_kind, "usb")
        self.assertEqual(clone.device_mode, P.TransportMode.USB)

    def test_old_config_without_usb_section_still_loads(self):
        """1.0 的配置文件里没有 usb 段，也要能读。"""
        cfg = AppConfig.from_dict({"mapping": {"mode": "absolute"}, "acquire": "single"})
        self.assertEqual(cfg.mapping.mode, "absolute")
        self.assertEqual(cfg.acquire, "single")
        self.assertEqual(cfg.usb.format, "auto")


if __name__ == "__main__":
    unittest.main(verbosity=2)
