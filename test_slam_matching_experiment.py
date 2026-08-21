#!/usr/bin/env python3
import csv
import json
import os
import tempfile
import unittest

import yaml

import slam_matching_experiment as experiment
from fixed_reference_lidar import analyze_fixed_reference, best_fixed_reference_shift
from slam_map_renderer import common_canvas, render_map


class SlamMatchingExperimentTests(unittest.TestCase):
    @staticmethod
    def _read_correction_rows(path):
        with open(path, newline="", encoding="utf-8") as stream:
            return list(csv.DictReader(stream))

    def test_corrections_use_intervening_odom_prediction(self):
        with tempfile.TemporaryDirectory() as directory:
            os.makedirs(os.path.join(directory, "replay_on"))
            manager = experiment.SlamMatchingExperiment()
            manager.session_dir = directory
            odom = [{"timestamp": float(i), "x": 0.5 * i, "y": 0.0, "yaw": 0.0} for i in range(3)]
            slam = [{"timestamp": float(i), "x": 0.5 * i, "y": 0.0, "yaw": 0.0} for i in range(3)]
            summary = manager._write_corrections(slam, odom)
            rows = self._read_correction_rows(os.path.join(directory, "replay_on", "scan_matching_corrections.csv"))
            self.assertEqual(summary["large_correction_warnings"], 0)
            self.assertAlmostEqual(float(rows[1]["odom_predicted_x"]), 0.5)
            self.assertAlmostEqual(float(rows[2]["odom_predicted_x"]), 1.0)
            self.assertAlmostEqual(float(rows[1]["local_correction_distance_m"]), 0.0)

    def test_many_small_accumulated_corrections_do_not_trigger_teleport_warning(self):
        with tempfile.TemporaryDirectory() as directory:
            os.makedirs(os.path.join(directory, "replay_on"))
            manager = experiment.SlamMatchingExperiment()
            manager.session_dir = directory
            odom = [{"timestamp": float(i), "x": 0.0, "y": 0.0, "yaw": 0.0} for i in range(21)]
            # Each accepted SLAM update moves only 1 cm, but the cumulative
            # map/odom discrepancy grows to 20 cm.
            slam = [{"timestamp": float(i), "x": 0.01 * i, "y": 0.0, "yaw": 0.0} for i in range(21)]
            summary = manager._write_corrections(slam, odom)
            rows = self._read_correction_rows(os.path.join(directory, "replay_on", "scan_matching_corrections.csv"))
            self.assertEqual(summary["large_correction_warnings"], 0)
            self.assertLessEqual(summary["max_translation_m"], 0.010000001)
            self.assertGreater(summary["max_cumulative_translation_m"], 0.19)
            self.assertTrue(all(int(row["local_warning"]) == 0 for row in rows))

    def test_one_large_instantaneous_correction_triggers_local_warning(self):
        with tempfile.TemporaryDirectory() as directory:
            os.makedirs(os.path.join(directory, "replay_on"))
            manager = experiment.SlamMatchingExperiment()
            manager.session_dir = directory
            odom = [{"timestamp": float(i), "x": 0.0, "y": 0.0, "yaw": 0.0} for i in range(3)]
            slam = [
                {"timestamp": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0},
                {"timestamp": 1.0, "x": 0.11, "y": 0.0, "yaw": 0.0},
                {"timestamp": 2.0, "x": 0.11, "y": 0.0, "yaw": 0.0},
            ]
            summary = manager._write_corrections(slam, odom)
            rows = self._read_correction_rows(os.path.join(directory, "replay_on", "scan_matching_corrections.csv"))
            self.assertEqual(summary["large_correction_warnings"], 1)
            self.assertAlmostEqual(float(rows[1]["local_correction_distance_m"]), 0.11)
            self.assertEqual(int(rows[1]["local_warning"]), 1)
            self.assertGreater(float(rows[1]["cumulative_correction_distance_m"]), 0.10)

    def test_off_on_configs_have_one_intended_difference(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = experiment.SlamMatchingExperiment()
            manager.session_dir = directory
            for name in ("config",):
                os.makedirs(os.path.join(directory, name))
            manager._write_configuration()
            with open(os.path.join(directory, "config", "slam_off.yaml"), encoding="utf-8") as stream:
                off = yaml.safe_load(stream)["slam_toolbox"]["ros__parameters"]
            with open(os.path.join(directory, "config", "slam_on.yaml"), encoding="utf-8") as stream:
                on = yaml.safe_load(stream)["slam_toolbox"]["ros__parameters"]
            self.assertFalse(off["use_scan_matching"])
            self.assertTrue(on["use_scan_matching"])
            self.assertEqual({k: v for k, v in off.items() if k != "use_scan_matching"},
                             {k: v for k, v in on.items() if k != "use_scan_matching"})
            self.assertFalse(on["do_loop_closing"])
            self.assertEqual(on["resolution"], 0.03)

    def test_map_rendering_uses_common_extent_and_polarity(self):
        first = {"width": 2, "height": 2, "resolution": 0.03,
                 "origin_x": 0.0, "origin_y": 0.0, "data": [0, 100, -1, 0]}
        second = {"width": 1, "height": 1, "resolution": 0.03,
                  "origin_x": 0.03, "origin_y": 0.03, "data": [100]}
        canvas = common_canvas([first, second])
        self.assertEqual((canvas["width"], canvas["height"]), (2, 2))
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "map.png")
            render_map(first, canvas, path)
            self.assertTrue(os.path.exists(path))

    def test_fixed_reference_identical_scan_is_zero(self):
        scan = {"ranges": [1.0, 2.0, 1.5, 3.0] * 180,
                "range_min": 0.05, "range_max": 8.0,
                "angle_increment": 6.283185307179586 / 720.0}
        match = best_fixed_reference_shift(scan, scan)
        self.assertAlmostEqual(match["yaw_deg"], 0.0)
        self.assertTrue(analyze_fixed_reference([scan, scan])["valid"])

    def test_replay_command_never_contains_cmd_vel(self):
        command = experiment._shell_ros(["ros2", "bag", "play", "--topics", "/scan", "/odom", "bag"], os.environ.copy())
        text = " ".join(command)
        self.assertNotIn("/cmd_vel", text)
        self.assertIn("/scan", text)
        self.assertIn("/odom", text)

    def test_relative_pose_wraps_heading(self):
        rows = [{"x": 0.0, "y": 0.0, "yaw": 3.13},
                {"x": 1.0, "y": 0.0, "yaw": -3.13}]
        result = experiment._relative_pose(rows)
        self.assertAlmostEqual(result["position_m"], 1.0)
        self.assertLess(abs(result["yaw_deg"]), 2.0)


if __name__ == "__main__":
    unittest.main()
