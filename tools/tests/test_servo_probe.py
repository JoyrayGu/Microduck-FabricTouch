import unittest

from microduck_behavior_control.servo_probe import read_servo


class ServoProbeSafetyTests(unittest.TestCase):
    def test_module_has_no_import_side_effects(self):
        # Importing must not open a port or touch hardware.
        import microduck_behavior_control.servo_probe as probe
        self.assertTrue(callable(probe.main))
        self.assertTrue(callable(read_servo))


if __name__ == "__main__":
    unittest.main()
