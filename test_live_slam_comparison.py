import tempfile
import unittest
from pathlib import Path

import yaml

from live_slam_comparison import OFF_NODE_NAME, live_off_launch_command, write_live_off_parameters


class LiveSlamComparisonTests(unittest.TestCase):
    def test_off_config_preserves_production_matcher_and_changes_only_routing_flag(self):
        source = Path("/home/robot1/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml")
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "slam_off.yaml"
            write_live_off_parameters(source, destination)
            production = yaml.safe_load(source.read_text())["slam_toolbox"]["ros__parameters"]
            off = yaml.safe_load(destination.read_text())[OFF_NODE_NAME]["ros__parameters"]

        self.assertFalse(off["use_scan_matching"])
        self.assertFalse(off["do_loop_closing"])
        self.assertEqual(off["map_frame"], "map_off")
        self.assertEqual(off["map_name"], "/map_off")
        self.assertEqual(off["resolution"], production["resolution"])
        self.assertEqual(off["map_update_interval"], production["map_update_interval"])
        for key, value in production.items():
            if key not in {"use_scan_matching", "map_frame", "map_name"}:
                self.assertEqual(off.get(key), value, key)

    def test_off_launch_uses_unique_node_and_no_motor_topic(self):
        command = live_off_launch_command("/tmp/slam_off.yaml")
        launcher = Path(command[1]).read_text()
        self.assertIn("name=OFF_NODE_NAME", launcher)
        self.assertIn('("/map", "/map_off")', launcher)
        self.assertNotIn("cmd_vel", " ".join(command))


if __name__ == "__main__":
    unittest.main()
