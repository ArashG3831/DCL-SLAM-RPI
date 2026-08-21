#!/usr/bin/env python3
import math
import tempfile
import unittest
from pathlib import Path

from odom_drift_analyzer import (
    AnalysisError,
    analyze_rows,
    analyze_spin,
    analyze_straight_stop,
    compare_spin_results,
    wrap_to_pi,
    write_result_artifacts,
)


def make_row(
    phase,
    t,
    x,
    y,
    yaw,
    *,
    linear_command=0.0,
    angular_command=0.0,
    linear_velocity=0.0,
    angular_velocity=0.0,
    key="x",
    with_commands=False,
):
    row = {
        "phase": phase,
        "timestamp_ros_s": float(t),
        "timestamp_monotonic_s": float(t),
        "x": float(x),
        "y": float(y),
        "yaw": float(yaw),
        "linear_x": float(linear_velocity),
        "angular_z": float(angular_velocity),
    }
    if with_commands:
        row.update({
            "command_key": key,
            "slider_rpm": 30.0,
            "linear_command_mps": float(linear_command),
            "angular_command_radps": float(angular_command),
            "requested_linear_command_mps": float(linear_command),
            "requested_angular_command_radps": float(angular_command),
            "command_change_monotonic_s": float(t),
            "has_command_fields": True,
        })
    return row


def make_straight_session(cruise_yaw=0.0, stop_yaw=0.0, direction=1.0):
    rows = [
        make_row(
            "start_reference", i * 0.05, 0.0, 0.0, 0.0,
            with_commands=True,
        )
        for i in range(10)
    ]
    # A long enough commanded straight run with a measurable odometry path.
    for i in range(16):
        fraction = i / 15.0
        if i <= 2:
            segment_yaw = 0.0
        elif i >= 13:
            segment_yaw = cruise_yaw
        else:
            segment_yaw = cruise_yaw * ((i - 2) / 11.0)
        rows.append(make_row(
            "trajectory",
            1.0 + i * 0.1,
            direction * fraction,
            0.0,
            segment_yaw,
            linear_command=direction * 0.10,
            linear_velocity=direction * 0.10,
            key="w" if direction > 0 else "s",
            with_commands=True,
        ))
    # Zero-command settling interval; only the first 0.5 s is needed by the
    # analyzer, but the extra samples make the synthetic event unambiguous.
    for i in range(9):
        rows.append(make_row(
            "trajectory",
            2.6 + i * 0.1,
            direction,
            0.0,
            cruise_yaw + stop_yaw,
            with_commands=True,
        ))
    rows.extend(
        make_row(
            "final_reference", 4.0 + i * 0.05, direction, 0.0,
            cruise_yaw + stop_yaw, with_commands=True,
        )
        for i in range(10)
    )
    return rows


def make_spin_session(theta_odom, direction="CCW", expected_turns=5):
    rows = [
        make_row(
            "start_reference", i * 0.05, 0.0, 0.0, 0.0,
            with_commands=True,
        )
        for i in range(10)
    ]
    for i in range(101):
        raw_yaw = theta_odom * (i / 100.0)
        rows.append(make_row(
            "trajectory",
            1.0 + i * 0.05,
            0.0,
            0.0,
            wrap_to_pi(raw_yaw),
            angular_command=-0.2 if direction == "CW" else 0.2,
            angular_velocity=-0.2 if direction == "CW" else 0.2,
            key="a" if direction == "CCW" else "d",
            with_commands=True,
        ))
    rows.extend(
        make_row(
            "final_reference", 7.0 + i * 0.05, 0.0, 0.0,
            wrap_to_pi(theta_odom), with_commands=True,
        )
        for i in range(10)
    )
    return rows


class OdomDriftAnalyzerTests(unittest.TestCase):
    def test_zero_drift(self):
        rows = [make_row("start_reference", i * 0.1, 0, 0, 0) for i in range(10)]
        rows.extend(make_row("trajectory", i, i * 0.1, 0, 0) for i in range(1, 6))
        rows.extend(make_row("trajectory", i + 5, (5 - i) * 0.1, 0, 0) for i in range(1, 6))
        rows.extend(make_row("final_reference", 10.1 + i * 0.1, 0, 0, 0) for i in range(10))
        result = analyze_rows(rows)
        self.assertEqual(result["status"], "ok")
        self.assertAlmostEqual(result["position_error_m"], 0.0, places=9)
        self.assertAlmostEqual(result["yaw_error_deg"], 0.0, places=9)
        self.assertAlmostEqual(result["odom_estimated_path_length_m"], 1.0, places=9)

    def test_start_frame_translation_and_yaw(self):
        start_yaw = math.pi / 2.0
        rows = [make_row("start_reference", i * 0.1, 0, 0, start_yaw) for i in range(10)]
        rows.extend(make_row("trajectory", i, 0, i * 0.1, start_yaw) for i in range(1, 11))
        rows.extend(make_row("final_reference", 10.1 + i * 0.1, 0, 1.0, start_yaw + math.radians(2)) for i in range(10))
        result = analyze_rows(rows)
        self.assertAlmostEqual(result["longitudinal_error_m"], 1.0, places=9)
        self.assertAlmostEqual(result["lateral_error_m"], 0.0, places=9)
        self.assertAlmostEqual(result["yaw_error_deg"], 2.0, places=7)

    def test_wrapped_heading(self):
        self.assertAlmostEqual(math.degrees(wrap_to_pi(math.radians(359))), -1.0, places=7)
        self.assertAlmostEqual(math.degrees(wrap_to_pi(math.radians(-359))), 1.0, places=7)

    def test_malformed_session_is_rejected(self):
        rows = [make_row("trajectory", i, i * 0.1, 0, 0) for i in range(20)]
        with self.assertRaises(AnalysisError):
            analyze_rows(rows)

    def test_straight_stop_symmetric(self):
        result = analyze_straight_stop(make_straight_session())
        self.assertEqual(result["status"], "ok")
        self.assertEqual(result["valid_segment_count"], 1)
        self.assertAlmostEqual(result["mean_cruise_yaw_deg"], 0.0, places=7)
        self.assertAlmostEqual(result["mean_stop_yaw_deg"], 0.0, places=7)

    def test_straight_stop_cruise_only(self):
        result = analyze_straight_stop(make_straight_session(cruise_yaw=0.05))
        self.assertAlmostEqual(result["mean_cruise_yaw_deg"], math.degrees(0.05), places=4)
        self.assertAlmostEqual(result["mean_stop_yaw_deg"], 0.0, places=7)

    def test_straight_stop_asymmetric_stop_and_combined(self):
        stop_only = analyze_straight_stop(make_straight_session(stop_yaw=0.10))
        self.assertAlmostEqual(stop_only["mean_cruise_yaw_deg"], 0.0, places=7)
        self.assertAlmostEqual(stop_only["mean_stop_yaw_deg"], math.degrees(0.10), places=4)

        combined = analyze_straight_stop(
            make_straight_session(cruise_yaw=0.05, stop_yaw=0.10)
        )
        self.assertAlmostEqual(combined["mean_cruise_yaw_deg"], math.degrees(0.05), places=4)
        self.assertAlmostEqual(combined["mean_stop_yaw_deg"], math.degrees(0.10), places=4)
        self.assertEqual(combined["dominant_phase"], "STOP")

    def test_straight_stop_reverse_and_short_tap_rejected(self):
        result = analyze_straight_stop(
            make_straight_session(cruise_yaw=0.02, stop_yaw=-0.03, direction=-1.0)
        )
        self.assertEqual(result["reverse_summary"]["count"], 1)
        self.assertAlmostEqual(result["mean_stop_yaw_deg"], math.degrees(-0.03), places=4)

        short = make_straight_session()
        short = [row for row in short if row["timestamp_monotonic_s"] < 2.6]
        for row in short:
            if row["linear_command_mps"] != 0.0:
                row["x"] *= 0.01
        with self.assertRaises(AnalysisError):
            analyze_straight_stop(short)

    def test_straight_stop_yaw_wrap(self):
        rows = make_straight_session(cruise_yaw=0.0, stop_yaw=0.0)
        # Replace the straight run with headings crossing +pi/-pi.  The
        # physical change is small and positive, not a 360-degree jump.
        for row in rows:
            if row["phase"] == "trajectory" and row["linear_command_mps"] != 0.0:
                fraction = (row["timestamp_monotonic_s"] - 1.0) / 1.5
                row["yaw"] = 3.05 + 0.06 * fraction
            elif (
                row["phase"] == "trajectory"
                and row["timestamp_monotonic_s"] >= 2.6
            ):
                row["yaw"] = wrap_to_pi(3.11 + 0.04 * ((row["timestamp_monotonic_s"] - 2.6) / 0.8))
            elif row["phase"] == "final_reference":
                row["yaw"] = wrap_to_pi(3.15)
        result = analyze_straight_stop(rows)
        # Window averaging makes the exact values slightly smaller than the
        # endpoint changes; the important assertion is that crossing +/-pi
        # remains a small positive motion rather than a ~360-degree jump.
        self.assertGreater(result["mean_cruise_yaw_deg"], 2.0)
        self.assertLess(result["mean_cruise_yaw_deg"], 4.0)
        self.assertGreater(result["mean_stop_yaw_deg"], 0.5)
        self.assertLess(result["mean_stop_yaw_deg"], 3.0)

    def test_straight_stop_no_valid_event(self):
        rows = make_straight_session()
        for row in rows:
            if row["linear_command_mps"] != 0.0:
                row["angular_command_radps"] = 0.1
        with self.assertRaises(AnalysisError):
            analyze_straight_stop(rows)

    def test_spin_exact_and_scale_formula(self):
        current_b = 0.2216
        exact = analyze_spin(make_spin_session(5 * 2 * math.pi), 5, "CCW", current_b)
        self.assertAlmostEqual(exact["rotation_scale_error_percent"], 0.0, places=7)
        self.assertAlmostEqual(exact["inferred_wheel_separation_m"], current_b, places=9)

        over = analyze_spin(make_spin_session(5 * 2 * math.pi * 1.01), 5, "CCW", current_b)
        self.assertAlmostEqual(over["rotation_scale_error_percent"], 1.0, places=4)
        self.assertAlmostEqual(over["inferred_wheel_separation_m"], current_b * 1.01, places=9)

        under = analyze_spin(make_spin_session(-5 * 2 * math.pi * 0.99, "CW"), 5, "CW", current_b)
        self.assertAlmostEqual(under["rotation_scale_error_percent"], -1.0, places=4)

    def test_spin_sign_wrap_and_quality_checks(self):
        result = analyze_spin(make_spin_session(-2 * 2 * math.pi, "CW", 2), 2, "CW", 0.2216)
        self.assertAlmostEqual(result["theta_odom_deg"], -720.0, places=5)
        self.assertEqual(result["quality_warnings"], [])
        with self.assertRaises(AnalysisError):
            analyze_spin(make_spin_session(2 * math.pi), 0, "CCW", 0.2216)

        mixed = make_spin_session(5 * 2 * math.pi, "CCW")
        for index, row in enumerate(mixed):
            if row["phase"] == "trajectory" and 40 <= index < 60:
                row["yaw"] = wrap_to_pi(-row["yaw"])
        mixed_result = analyze_spin(mixed, 5, "CCW", 0.2216)
        self.assertIn("MIXED_OR_REVERSED_ROTATION_SEGMENTS", mixed_result["quality_warnings"])

    def test_spin_comparison_flags_direction_bias(self):
        cw = analyze_spin(make_spin_session(-5 * 2 * math.pi * 1.01, "CW"), 5, "CW", 0.2216)
        ccw = analyze_spin(make_spin_session(5 * 2 * math.pi * 0.99, "CCW"), 5, "CCW", 0.2216)
        comparison = compare_spin_results([cw, ccw])
        self.assertEqual(comparison["status"], "ok")
        self.assertIn("DIRECTION_DEPENDENT_ROTATION_BIAS", comparison["quality_warnings"])

    def test_result_artifacts_include_suite_alias(self):
        result = analyze_rows(
            [make_row("start_reference", i * 0.1, 0, 0, 0) for i in range(10)]
            + [make_row("trajectory", 1 + i * 0.1, i * 0.1, 0, 0) for i in range(60)]
            + [make_row("final_reference", 7.1 + i * 0.1, 0, 0, 0) for i in range(10)]
        )
        with tempfile.TemporaryDirectory() as directory:
            write_result_artifacts(result, directory)
            self.assertTrue(Path(directory, "odom_drift_result.json").exists())
            self.assertTrue(Path(directory, "odom_test_result.json").exists())


if __name__ == "__main__":
    unittest.main()
