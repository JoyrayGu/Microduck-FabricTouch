import unittest

from microduck_skin_diagnostics import frame_parser as fp


def build_frame(payload: bytes, header_size: int = 7) -> bytes:
    head = bytes([0x98]) + b"\x00" * (header_size - 1)
    body = head + payload + b"\x00" + bytes([0x81])
    return body


class FrameSyncTests(unittest.TestCase):
    def test_single_frame_is_extracted(self):
        payload = bytes([0x12, 0x34]) * 1024
        sync = fp.FrameSynchronizer()
        frames = sync.feed(build_frame(payload))
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].length, 2057)
        self.assertEqual(frames[0].payload, payload)

    def test_leading_garbage_is_discarded(self):
        payload = bytes([0x00]) * 2048
        sync = fp.FrameSynchronizer()
        frames = sync.feed(b"garbage" + build_frame(payload))
        self.assertEqual(len(frames), 1)
        self.assertEqual(sync.stats.discarded_bytes, 7)

    def test_split_across_chunks_reassembles(self):
        payload = bytes([0x05]) * 2048
        frame = build_frame(payload)
        sync = fp.FrameSynchronizer()
        out = []
        for i in range(0, len(frame), 500):
            out.extend(sync.feed(frame[i : i + 500]))
        self.assertEqual(len(out), 1)

    def test_empty_payload_frame_rejected(self):
        sync = fp.FrameSynchronizer()
        frames = sync.feed(bytes([0x98]) + b"\x00" * 60)
        self.assertEqual(frames, [])

    def test_nibble_and_word_layouts_have_expected_sizes(self):
        payload = bytes(range(256)) * 8
        self.assertEqual(len(fp.nibble_cells(payload)), 4096)
        self.assertEqual(len(fp.word_cells(payload)), 1024)
        self.assertEqual(fp.grid_shape(fp.nibble_cells(payload)), (64, 64))
        self.assertEqual(fp.grid_shape(fp.word_cells(payload)), (32, 32))

    def test_word_cells_is_big_endian(self):
        """Device sends high byte first.

        A little-endian reader turned a single 0x18 byte into 6144 and made the
        32x32 layout look like nonsense.
        """
        payload = bytes([0x00, 0x18, 0x01, 0x00] + [0x00] * 2044)
        cells = fp.word_cells(payload)
        self.assertEqual(cells[0], 0x0018)
        self.assertEqual(cells[1], 0x0100)

    def test_parity_split_rules_out_packed_nibbles(self):
        """Regression for the 64x64 mistake.

        Real frames show the even-index byte high nibble is never non-zero while
        the odd-index byte high nibble is often non-zero. Packed 4-bit cells
        cannot produce that asymmetry, so the layout must be 16-bit words.
        """
        # A frame with real touch data: high byte 0x00, low byte carrying value.
        body = bytearray(2048)
        body[1] = 0x18   # odd index, value 24
        body[3] = 0x2A   # odd index, value 42
        cells = fp.word_cells(bytes(body))
        self.assertEqual(cells[0], 24)
        self.assertEqual(cells[1], 42)
        # The nibble reader would split each value into two cells instead.
        nib = fp.nibble_cells(bytes(body))
        self.assertNotEqual(len(nib), len(cells))
        self.assertEqual(nib[2], 1)   # high nibble of 0x18
        self.assertEqual(nib[3], 8)   # low nibble of 0x18

    def test_centroid_moves_with_active_region(self):
        cells = [0] * 4096
        for y in range(0, 4):
            for x in range(0, 4):
                cells[y * 64 + x] = 15
        top_left = fp.analyze_layout(cells, "test", 1)
        self.assertEqual(top_left.active_cells, 16)
        self.assertLess(top_left.centroid[0], 5)
        self.assertLess(top_left.centroid[1], 5)

    def test_summary_reports_both_layouts(self):
        payload = bytes([0x0F, 0x00] * 1024)
        frames = [fp.Frame(2057, 7, payload, b"\x00", b"")]
        summary = fp.summarize(frames, threshold=1)
        self.assertIn("32x32_16bit_be", summary["layouts"])
        self.assertIn("64x64_4bit_rejected", summary["layouts"])


if __name__ == "__main__":
    unittest.main()
