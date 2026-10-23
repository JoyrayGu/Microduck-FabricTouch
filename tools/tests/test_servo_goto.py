import unittest

from microduck_behavior_control import servo_goto


class ServoGotoTemplateTests(unittest.TestCase):
    def test_template_renders_allow_torque_on_true(self):
        script = servo_goto._REMOTE_TEMPLATE.format(
            sid=14, target=2050, steps=10, step_time=0.05, allow_torque_on=True
        )
        self.assertIn("allow_torque_on = True", script)
        self.assertIn("not allow_torque_on", script)

    def test_template_renders_allow_torque_on_false(self):
        script = servo_goto._REMOTE_TEMPLATE.format(
            sid=14, target=2050, steps=10, step_time=0.05, allow_torque_on=False
        )
        self.assertIn("allow_torque_on = False", script)

    def test_script_compiles(self):
        script = servo_goto._REMOTE_TEMPLATE.format(
            sid=14, target=2050, steps=10, step_time=0.05, allow_torque_on=True
        )
        compile(script, "<remote>", "exec")


if __name__ == "__main__":
    unittest.main()
