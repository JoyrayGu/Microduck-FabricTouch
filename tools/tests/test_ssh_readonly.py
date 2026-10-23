import unittest

from microduck_behavior_control.ssh_readonly import PiConnection, ReadOnlySshClient


class ReadOnlySshTests(unittest.TestCase):
    def test_commands_require_batch_ssh_and_have_no_action_script(self):
        client = ReadOnlySshClient(PiConnection())
        for name in ("identity", "files", "status", "processes", "ports"):
            command = client.command(name)
            self.assertEqual(command[0], "ssh")
            self.assertIn("BatchMode=yes", command)
            self.assertIn("microduck_ed25519", " ".join(command))
            self.assertNotIn("return_to_stand.py", command[-1])
            self.assertNotIn("move_to_pose.py", command[-1])
            self.assertNotIn("password", " ".join(command).lower())

    def test_unknown_command_is_rejected(self):
        with self.assertRaises(ValueError):
            ReadOnlySshClient().command("start_action")


if __name__ == "__main__":
    unittest.main()
