import unittest

from microduck_behavior_control.snapshot import parse_status_output


class SnapshotTests(unittest.TestCase):
    def test_parser_is_physical_id_keyed(self):
        text = "\n".join(
            f"ID{i:02d} mode=3 t=0 goal={1000+i} present={1000+i}" for i in range(1, 15)
        )
        data = parse_status_output(text)
        self.assertEqual(data[2]["present_tick"], 1002)
        self.assertEqual(data[7]["present_tick"], 1007)

    def test_parser_rejects_missing_id(self):
        text = "\n".join(
            f"ID{i:02d} mode=3 t=0 goal={1000+i} present={1000+i}" for i in range(1, 14)
        )
        with self.assertRaises(ValueError):
            parse_status_output(text)

    def test_parser_rejects_comm_error(self):
        text = "\n".join(
            f"ID{i:02d} mode=3 t=0 goal={1000+i} present={1000+i}" for i in range(1, 14)
        ) + "\nID14 COMM_ERROR ['m']"
        with self.assertRaises(ValueError):
            parse_status_output(text)


if __name__ == "__main__":
    unittest.main()
