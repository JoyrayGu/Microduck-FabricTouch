import unittest
from pathlib import Path

from microduck_behavior_control.dry_run import report


class OfflineReportTests(unittest.TestCase):
    def test_incomplete_calibration_is_reported_as_refused(self):
        path = Path(__file__).resolve().parents[1] / "config" / "calibration_template.json"
        text = report(path)
        self.assertIn("ID NAME VALIDATED", text)
        self.assertIn("2 left_knee False", text)
        self.assertIn("STAND_PLAN=REFUSED calibration incomplete", text)


if __name__ == "__main__":
    unittest.main()
