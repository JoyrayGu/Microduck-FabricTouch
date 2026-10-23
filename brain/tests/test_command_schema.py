# SPDX-License-Identifier: GPL-3.0-or-later
"""@file test_command_schema.py
@brief 线上协议与第一道安全边界的测试。

对应 ``microduck/src/command_schema.py``。这些用例把「什么事件是合法的」固定
下来，日后改动协议时能立刻发现回归。

运行::

    python -m unittest discover -s tests -v
"""

from __future__ import annotations

import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from microduck.src import command_schema as S  # noqa: E402


def good_payload(**overrides):
    payload = {
        "version": S.PROTOCOL_VERSION,
        "event_id": "abc123",
        "gesture": "tap",
        "timestamp_ms": 1_791_500_000_000,
        "confidence": 0.9,
    }
    payload.update(overrides)
    return payload


class ParseEventTests(unittest.TestCase):
    def test_minimal_event_ok(self):
        event = S.parse_event(good_payload())
        self.assertEqual(event.gesture, "tap")
        self.assertEqual(event.event_id, "abc123")
        self.assertEqual(event.confidence, 0.9)

    def test_all_allowed_gestures_accepted(self):
        for gesture in S.ALLOWED_GESTURES:
            event = S.parse_event(good_payload(gesture=gesture))
            self.assertEqual(event.gesture, gesture)

    def test_unknown_gesture_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(good_payload(gesture="dance"))
        self.assertEqual(ctx.exception.reason, "bad_gesture")

    def test_gesture_is_case_sensitive(self):
        with self.assertRaises(S.SchemaError):
            S.parse_event(good_payload(gesture="TAP"))

    def test_bad_version_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(good_payload(version=99))
        self.assertEqual(ctx.exception.reason, "bad_version")

    def test_missing_event_id_rejected(self):
        payload = good_payload()
        del payload["event_id"]
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(payload)
        self.assertEqual(ctx.exception.reason, "bad_event_id")

    def test_blank_event_id_rejected(self):
        with self.assertRaises(S.SchemaError):
            S.parse_event(good_payload(event_id="   "))

    def test_confidence_must_be_in_range(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(good_payload(confidence=1.5))
        self.assertEqual(ctx.exception.reason, "bad_confidence")
        with self.assertRaises(S.SchemaError):
            S.parse_event(good_payload(confidence=-0.1))

    def test_timestamp_must_be_numeric(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(good_payload(timestamp_ms="just now"))
        self.assertEqual(ctx.exception.reason, "bad_timestamp")

    def test_direction_and_points_parsed(self):
        event = S.parse_event(
            good_payload(direction="left_to_right", start=[1, 2], end=[3.5, 4.0])
        )
        self.assertEqual(event.direction, "left_to_right")
        self.assertEqual(event.start, (1.0, 2.0))
        self.assertEqual(event.end, (3.5, 4.0))

    def test_bad_start_shape_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event(good_payload(start=[1, 2, 3]))
        self.assertEqual(ctx.exception.reason, "bad_field")

    def test_unknown_fields_kept_in_extra(self):
        event = S.parse_event(good_payload(servo_id=13, speed=200))
        self.assertEqual(event.extra.get("servo_id"), 13)
        self.assertEqual(event.extra.get("speed"), 200)

    def test_not_object_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_event([1, 2, 3])
        self.assertEqual(ctx.exception.reason, "not_object")


class ParseLineTests(unittest.TestCase):
    def test_round_trip_through_json(self):
        line = json.dumps(good_payload())
        event = S.parse_line(line)
        self.assertEqual(event.gesture, "tap")

    def test_bad_json_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_line("{not json")
        self.assertEqual(ctx.exception.reason, "bad_json")

    def test_empty_line_rejected(self):
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_line("   ")
        self.assertEqual(ctx.exception.reason, "empty")

    def test_oversized_line_rejected(self):
        blob = "x" * (S.MAX_EVENT_BYTES + 10)
        with self.assertRaises(S.SchemaError) as ctx:
            S.parse_line(blob)
        self.assertEqual(ctx.exception.reason, "too_large")


class FreshnessTests(unittest.TestCase):
    def _event(self, timestamp_ms):
        return S.parse_event(good_payload(timestamp_ms=timestamp_ms))

    def test_fresh_event_ok(self):
        now = 1_000_000
        S.check_freshness(self._event(now - 100), now)

    def test_expired_event_rejected(self):
        now = 1_000_000
        with self.assertRaises(S.SchemaError) as ctx:
            S.check_freshness(self._event(now - S.MAX_EVENT_AGE_MS - 1), now)
        self.assertEqual(ctx.exception.reason, "expired")

    def test_future_event_rejected(self):
        now = 1_000_000
        with self.assertRaises(S.SchemaError) as ctx:
            S.check_freshness(self._event(now + 10_000), now)
        self.assertEqual(ctx.exception.reason, "from_future")

    def test_small_future_clock_skew_tolerated(self):
        now = 1_000_000
        S.check_freshness(self._event(now + 100), now)


class ReplyTests(unittest.TestCase):
    def test_reply_ok(self):
        reply = S.make_reply("abc", True, action="head_roll")
        self.assertTrue(reply["accepted"])
        self.assertEqual(reply["action"], "head_roll")
        self.assertEqual(reply["reason"], "ok")

    def test_reply_rejected_has_no_action(self):
        reply = S.make_reply("abc", False, reason="busy")
        self.assertFalse(reply["accepted"])
        self.assertNotIn("action", reply)
        self.assertEqual(reply["reason"], "busy")

    def test_encode_line_is_single_ndjson_line(self):
        data = S.encode_line(S.make_reply("abc", True, action="head_roll"))
        self.assertTrue(data.endswith(b"\n"))
        self.assertEqual(data.count(b"\n"), 1)
        self.assertEqual(json.loads(data.decode("utf-8")), S.make_reply("abc", True, action="head_roll"))


if __name__ == "__main__":
    unittest.main()
