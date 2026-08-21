#!/usr/bin/env python3
"""Generic analyzer for the phone-controller odometry test suite."""

import argparse
import csv
import json
import math
import os
import sys
from datetime import datetime, timezone


class AnalysisError(RuntimeError):
    """Raised when a saved session cannot produce a trustworthy result."""


BASE_FIELDS = {
    "phase",
    "timestamp_ros_s",
    "timestamp_monotonic_s",
    "x",
    "y",
    "yaw",
    "linear_x",
    "angular_z",
}

COMMAND_FIELDS = {
    "command_key",
    "slider_rpm",
    "linear_command_mps",
    "angular_command_radps",
    "requested_linear_command_mps",
    "requested_angular_command_radps",
    "command_change_monotonic_s",
}

STRAIGHT_MIN_DURATION_S = 0.50
STRAIGHT_MIN_DISTANCE_M = 0.05
STRAIGHT_START_WINDOW_S = 0.25
STRAIGHT_PRESTOP_WINDOW_S = 0.25
STRAIGHT_SETTLE_WINDOW_S = 0.50
SETTLED_LINEAR_MPS = 0.005
SETTLED_ANGULAR_RADPS = 0.02
COMMAND_EPSILON = 1e-6


def wrap_to_pi(angle):
    """Return an angle in [-pi, pi)."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def circular_mean(angles):
    values = [float(value) for value in angles]
    if not values:
        raise AnalysisError("no angles available for circular mean")
    sine = sum(math.sin(value) for value in values)
    cosine = sum(math.cos(value) for value in values)
    if abs(sine) < 1e-12 and abs(cosine) < 1e-12:
        raise AnalysisError("heading samples are circularly ambiguous")
    return math.atan2(sine, cosine)


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise AnalysisError(f"{name} is not numeric") from None
    if not math.isfinite(number):
        raise AnalysisError(f"{name} is not finite")
    return number


def _pose_mean(rows):
    if not rows:
        raise AnalysisError("pose reference contains no samples")
    return {
        "x": sum(row["x"] for row in rows) / len(rows),
        "y": sum(row["y"] for row in rows) / len(rows),
        "yaw": circular_mean(row["yaw"] for row in rows),
        "timestamp_ros_s": sum(row["timestamp_ros_s"] for row in rows) / len(rows),
        "timestamp_monotonic_s": sum(
            row["timestamp_monotonic_s"] for row in rows
        ) / len(rows),
    }


def _ordered_rows(rows):
    ordered = [
        row for row in rows
        if row.get("phase") in {"start_reference", "trajectory", "final_reference"}
    ]
    ordered.sort(key=lambda row: row["timestamp_monotonic_s"])
    if not ordered:
        raise AnalysisError("trajectory contains no usable pose samples")
    for previous, current in zip(ordered, ordered[1:]):
        if current["timestamp_monotonic_s"] < previous["timestamp_monotonic_s"]:
            raise AnalysisError("odometry timestamps are not ordered")
    return ordered


def _unwrap_rows(rows):
    result = []
    accumulated = rows[0]["yaw"]
    result.append(accumulated)
    for previous, current in zip(rows, rows[1:]):
        accumulated += wrap_to_pi(current["yaw"] - previous["yaw"])
        result.append(accumulated)
    return result


def _path_length(rows):
    return sum(
        math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        for previous, current in zip(rows, rows[1:])
    )


def _start_frame_delta(start, end):
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    cos_yaw = math.cos(start["yaw"])
    sin_yaw = math.sin(start["yaw"])
    return {
        "raw_delta_x_odom_m": dx,
        "raw_delta_y_odom_m": dy,
        "forward_displacement_m": cos_yaw * dx + sin_yaw * dy,
        "lateral_displacement_m": -sin_yaw * dx + cos_yaw * dy,
        "position_error_m": math.hypot(dx, dy),
    }


def load_trajectory_csv(path):
    """Load current or legacy odom_trajectory.csv files."""
    try:
        stream = open(path, newline="", encoding="utf-8")
    except OSError as exc:
        raise AnalysisError(f"cannot open trajectory: {exc}") from exc

    rows = []
    try:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        missing = sorted(BASE_FIELDS - fields)
        if missing:
            raise AnalysisError(f"trajectory CSV is missing columns: {missing}")
        has_command_fields = COMMAND_FIELDS.issubset(fields)
        for index, raw in enumerate(reader, start=2):
            try:
                row = {
                    "phase": str(raw["phase"] or "trajectory"),
                    "test_mode": str(raw.get("test_mode") or "CLOSED_LOOP"),
                    "timestamp_ros_s": _finite(raw["timestamp_ros_s"], "ROS timestamp"),
                    "timestamp_monotonic_s": _finite(
                        raw["timestamp_monotonic_s"], "monotonic timestamp"
                    ),
                    "x": _finite(raw["x"], "x"),
                    "y": _finite(raw["y"], "y"),
                    "yaw": _finite(raw["yaw"], "yaw"),
                    "linear_x": _finite(raw["linear_x"], "linear_x"),
                    "angular_z": _finite(raw["angular_z"], "angular_z"),
                    "command_key": str(raw.get("command_key") or "x"),
                    "slider_rpm": _finite(raw.get("slider_rpm") or 0.0, "slider RPM"),
                    "linear_command_mps": _finite(
                        raw.get("linear_command_mps") or 0.0, "linear command"
                    ),
                    "angular_command_radps": _finite(
                        raw.get("angular_command_radps") or 0.0, "angular command"
                    ),
                    "requested_linear_command_mps": _finite(
                        raw.get("requested_linear_command_mps") or 0.0,
                        "requested linear command",
                    ),
                    "requested_angular_command_radps": _finite(
                        raw.get("requested_angular_command_radps") or 0.0,
                        "requested angular command",
                    ),
                    "command_change_monotonic_s": _finite(
                        raw.get("command_change_monotonic_s")
                        or raw["timestamp_monotonic_s"],
                        "command-change timestamp",
                    ),
                    "has_command_fields": has_command_fields,
                }
            except AnalysisError as exc:
                raise AnalysisError(f"row {index}: {exc}") from exc
            rows.append(row)
    except csv.Error as exc:
        raise AnalysisError(f"malformed trajectory CSV: {exc}") from exc
    finally:
        stream.close()

    if not rows:
        raise AnalysisError("trajectory CSV contains no samples")
    return rows


def _references(rows):
    start_rows = [row for row in rows if row["phase"] == "start_reference"]
    final_rows = [row for row in rows if row["phase"] == "final_reference"]
    if not start_rows:
        raise AnalysisError("no starting reference was recorded")
    if not final_rows:
        raise AnalysisError("no final reference was recorded")
    return start_rows, final_rows


def analyze_rows(
    rows,
    minimum_samples=10,
    minimum_duration_s=5.0,
    minimum_path_length_m=0.05,
    allow_stationary=False,
):
    """Analyze the original CLOSED_LOOP endpoint-drift experiment."""
    start_rows, final_rows = _references(rows)
    trajectory_rows = _ordered_rows(rows)
    if len(trajectory_rows) < int(minimum_samples):
        raise AnalysisError(
            f"too few odometry samples: {len(trajectory_rows)} < {minimum_samples}"
        )

    start = _pose_mean(start_rows)
    final = _pose_mean(final_rows)
    duration_s = final["timestamp_monotonic_s"] - start["timestamp_monotonic_s"]
    if duration_s <= 0.0 or not math.isfinite(duration_s):
        raise AnalysisError("test duration is invalid")
    if duration_s < float(minimum_duration_s):
        raise AnalysisError(
            f"test duration is suspiciously short: {duration_s:.3f} s "
            f"< {minimum_duration_s:.3f} s"
        )

    deltas = _start_frame_delta(start, final)
    yaw_error_rad = wrap_to_pi(final["yaw"] - start["yaw"])
    path_length = _path_length(trajectory_rows)
    if path_length < float(minimum_path_length_m) and not allow_stationary:
        raise AnalysisError(
            f"odometry path is suspiciously short: {path_length:.4f} m "
            f"< {minimum_path_length_m:.4f} m"
        )

    unwrapped = _unwrap_rows(trajectory_rows)
    max_distance = max(
        math.hypot(row["x"] - start["x"], row["y"] - start["y"])
        for row in trajectory_rows
    )
    total_absolute_heading_change = sum(
        abs(current - previous) for previous, current in zip(unwrapped, unwrapped[1:])
    )
    return {
        "status": "ok",
        "test_mode": "CLOSED_LOOP",
        "analysis_version": 2,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "start_pose": start,
        "end_pose": final,
        **deltas,
        "longitudinal_error_m": deltas["forward_displacement_m"],
        "lateral_error_m": deltas["lateral_displacement_m"],
        "position_error_cm": deltas["position_error_m"] * 100.0,
        "yaw_error_rad": yaw_error_rad,
        "yaw_error_deg": math.degrees(yaw_error_rad),
        "odom_estimated_path_length_m": path_length,
        "endpoint_position_drift_percent": (
            100.0 * deltas["position_error_m"] / path_length
            if path_length > 0.0 else None
        ),
        "absolute_heading_drift_deg_per_m": (
            math.degrees(abs(yaw_error_rad)) / path_length
            if path_length > 0.0 else None
        ),
        "duration_s": duration_s,
        "sample_count": len(trajectory_rows),
        "approximate_sample_rate_hz": (
            (len(trajectory_rows) - 1) / duration_s if len(trajectory_rows) > 1 else 0.0
        ),
        "max_distance_from_start_m": max_distance,
        "total_absolute_heading_change_deg": math.degrees(total_absolute_heading_change),
        "start_reference_sample_count": len(start_rows),
        "final_reference_sample_count": len(final_rows),
        "stationary_sanity_mode": bool(allow_stationary),
        "telemetry_available": False,
        "interpretation": (
            "Endpoint error assumes the robot was physically returned to the marked "
            "starting position and heading; placement/alignment error is included."
        ),
    }


def _require_command_fields(rows):
    if not all(row.get("has_command_fields", False) for row in rows):
        raise AnalysisError(
            "STRAIGHT_STOP requires command fields; this is a legacy trajectory file"
        )


def _is_straight(row):
    return (
        abs(row["linear_command_mps"]) > COMMAND_EPSILON
        and abs(row["angular_command_radps"]) <= COMMAND_EPSILON
    )


def _is_zero_command(row):
    return (
        abs(row["linear_command_mps"]) <= COMMAND_EPSILON
        and abs(row["angular_command_radps"]) <= COMMAND_EPSILON
    )


def _mean(values):
    values = list(values)
    return sum(values) / len(values) if values else None


def _straight_summary(events):
    if not events:
        return {
            "count": 0,
            "mean_cruise_yaw_deg": None,
            "median_cruise_yaw_deg": None,
            "max_abs_cruise_yaw_deg": None,
            "mean_stop_yaw_deg": None,
            "median_stop_yaw_deg": None,
            "max_abs_stop_yaw_deg": None,
            "mean_lateral_displacement_per_m": None,
        }
    cruise = sorted(event["cruise_yaw_deg"] for event in events)
    stop = sorted(event["stop_yaw_deg"] for event in events)
    lateral_per_m = [
        event["lateral_displacement_m"] / event["segment_distance_m"]
        for event in events if event["segment_distance_m"] > 0.0
    ]
    return {
        "count": len(events),
        "mean_cruise_yaw_deg": _mean(cruise),
        "median_cruise_yaw_deg": cruise[len(cruise) // 2],
        "max_abs_cruise_yaw_deg": max(abs(value) for value in cruise),
        "mean_stop_yaw_deg": _mean(stop),
        "median_stop_yaw_deg": stop[len(stop) // 2],
        "max_abs_stop_yaw_deg": max(abs(value) for value in stop),
        "mean_lateral_displacement_per_m": _mean(lateral_per_m),
    }


def _settled_window(rows, start_index):
    low_start = None
    for index in range(start_index, len(rows)):
        row = rows[index]
        low = (
            abs(row["linear_x"]) <= SETTLED_LINEAR_MPS
            and abs(row["angular_z"]) <= SETTLED_ANGULAR_RADPS
        )
        if low:
            if low_start is None:
                low_start = index
            if (
                row["timestamp_monotonic_s"]
                - rows[low_start]["timestamp_monotonic_s"]
                >= STRAIGHT_SETTLE_WINDOW_S
            ):
                return rows[low_start : index + 1]
        else:
            low_start = None
    return None


def analyze_straight_stop(rows):
    """Analyze repeated straight-command to stop events."""
    _require_command_fields(rows)
    ordered = _ordered_rows(rows)
    start_rows, final_rows = _references(rows)
    if len(ordered) < 10:
        raise AnalysisError("too few odometry samples for STRAIGHT_STOP")

    unwrapped = _unwrap_rows(ordered)
    for index, value in enumerate(unwrapped):
        ordered[index]["_yaw_unwrapped"] = value

    events = []
    rejected = []
    index = 0
    while index < len(ordered):
        if not _is_straight(ordered[index]):
            index += 1
            continue
        segment_start = index
        while index + 1 < len(ordered) and _is_straight(ordered[index + 1]):
            index += 1
        segment_end = index
        run = ordered[segment_start : segment_end + 1]
        duration = run[-1]["timestamp_monotonic_s"] - run[0]["timestamp_monotonic_s"]
        distance = _path_length(run)
        if duration < STRAIGHT_MIN_DURATION_S or distance < STRAIGHT_MIN_DISTANCE_M:
            rejected.append({
                "reason": "short_straight_segment",
                "duration_s": duration,
                "distance_m": distance,
            })
            index += 1
            continue

        post_end = segment_end + 1
        while post_end < len(ordered) and _is_zero_command(ordered[post_end]):
            post_end += 1
        post_rows = ordered[segment_end + 1 : post_end]
        # Restrict settling analysis to the zero-command interval immediately
        # after this straight segment.  Otherwise a later stationary interval
        # could accidentally make an intervening motion command look settled.
        settled = _settled_window(post_rows, 0)
        if not post_rows or settled is None:
            rejected.append({
                "reason": "no_settled_zero_command_window",
                "duration_s": duration,
                "distance_m": distance,
            })
            index += 1
            continue

        start_window = [
            row for row in run
            if row["timestamp_monotonic_s"]
            <= run[0]["timestamp_monotonic_s"] + STRAIGHT_START_WINDOW_S
        ] or run[: min(5, len(run))]
        pre_window = [
            row for row in run
            if row["timestamp_monotonic_s"]
            >= run[-1]["timestamp_monotonic_s"] - STRAIGHT_PRESTOP_WINDOW_S
        ] or run[-min(5, len(run)) :]
        settled_window = settled
        yaw_start = _mean(row["_yaw_unwrapped"] for row in start_window)
        yaw_pre = _mean(row["_yaw_unwrapped"] for row in pre_window)
        yaw_settled = _mean(row["_yaw_unwrapped"] for row in settled_window)
        cruise_yaw = wrap_to_pi(yaw_pre - yaw_start)
        stop_yaw = wrap_to_pi(yaw_settled - yaw_pre)
        total_yaw = wrap_to_pi(yaw_settled - yaw_start)
        start_pose = {
            "x": run[0]["x"],
            "y": run[0]["y"],
            "yaw": yaw_start,
        }
        settled_pose = {
            "x": settled_window[-1]["x"],
            "y": settled_window[-1]["y"],
            "yaw": yaw_settled,
        }
        displacement = _start_frame_delta(start_pose, settled_pose)
        direction = "forward" if run[0]["linear_command_mps"] > 0 else "reverse"
        events.append({
            "segment_index": len(events) + 1,
            "direction": direction,
            "start_timestamp_monotonic_s": run[0]["timestamp_monotonic_s"],
            "pre_stop_timestamp_monotonic_s": run[-1]["timestamp_monotonic_s"],
            "settled_timestamp_monotonic_s": settled_window[-1]["timestamp_monotonic_s"],
            "segment_duration_s": duration,
            "segment_distance_m": distance,
            "yaw_start_rad": yaw_start,
            "yaw_pre_stop_rad": yaw_pre,
            "yaw_settled_rad": yaw_settled,
            "cruise_yaw_deg": math.degrees(cruise_yaw),
            "stop_yaw_deg": math.degrees(stop_yaw),
            "total_yaw_deg": math.degrees(total_yaw),
            **displacement,
            "left_stop_travel_m": None,
            "right_stop_travel_m": None,
            "differential_stop_travel_m": None,
            "predicted_stop_yaw_deg": None,
            "wheel_telemetry_available": False,
        })
        index = max(index + 1, post_end)

    if not events:
        raise AnalysisError(
            "no valid straight-to-stop events; use longer straight runs and allow the robot to settle"
        )
    forward = [event for event in events if event["direction"] == "forward"]
    reverse = [event for event in events if event["direction"] == "reverse"]
    all_cruise = [event["cruise_yaw_deg"] for event in events]
    all_stop = [event["stop_yaw_deg"] for event in events]
    total_abs = sum(abs(value) for value in all_cruise) + sum(abs(value) for value in all_stop)
    stop_fraction = 100.0 * sum(abs(value) for value in all_stop) / total_abs if total_abs else None
    stop_signs = {
        "positive": sum(value > 0.0 for value in all_stop),
        "negative": sum(value < 0.0 for value in all_stop),
        "zero": sum(value == 0.0 for value in all_stop),
    }
    mean_abs_cruise = _mean(abs(value) for value in all_cruise) or 0.0
    mean_abs_stop = _mean(abs(value) for value in all_stop) or 0.0
    return {
        "status": "ok",
        "test_mode": "STRAIGHT_STOP",
        "analysis_version": 2,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "start_pose": _pose_mean(start_rows),
        "end_pose": _pose_mean(final_rows),
        "valid_segment_count": len(events),
        "rejected_segment_count": len(rejected),
        "rejected_segments": rejected,
        "segments": events,
        "forward_summary": _straight_summary(forward),
        "reverse_summary": _straight_summary(reverse),
        "combined_summary": _straight_summary(events),
        "mean_cruise_yaw_deg": _mean(all_cruise),
        "mean_stop_yaw_deg": _mean(all_stop),
        "max_abs_stop_yaw_deg": max(abs(value) for value in all_stop),
        "mean_abs_stop_yaw_deg": mean_abs_stop,
        "fraction_total_abs_yaw_after_release_percent": stop_fraction,
        "dominant_phase": "STOP" if mean_abs_stop > mean_abs_cruise else "CRUISE",
        "stop_yaw_sign_counts": stop_signs,
        "straight_thresholds": {
            "minimum_duration_s": STRAIGHT_MIN_DURATION_S,
            "minimum_distance_m": STRAIGHT_MIN_DISTANCE_M,
            "settle_window_s": STRAIGHT_SETTLE_WINDOW_S,
            "settled_linear_mps": SETTLED_LINEAR_MPS,
            "settled_angular_radps": SETTLED_ANGULAR_RADPS,
        },
        "telemetry_available": False,
        "limitation": (
            "Stop-induced yaw and differential wheel travel are encoder-derived. "
            "Without an external reference, tire slip cannot be separated from chassis motion."
        ),
    }


def analyze_spin(rows, expected_turns, direction, current_wheel_separation_m):
    """Analyze known-count CW/CCW rotations using unwrapped odometry yaw."""
    start_rows, final_rows = _references(rows)
    ordered = _ordered_rows(rows)
    if len(ordered) < 10:
        raise AnalysisError("too few odometry samples for SPIN")
    if direction not in {"CW", "CCW"}:
        raise AnalysisError("SPIN direction must be CW or CCW")
    if not 1 <= int(expected_turns) <= 10:
        raise AnalysisError("expected SPIN turns must be from 1 to 10")
    current_b = _finite(current_wheel_separation_m, "current wheel separation")
    if current_b <= 0.0:
        raise AnalysisError("current wheel separation must be positive")

    unwrapped = _unwrap_rows(ordered)
    for index, value in enumerate(unwrapped):
        ordered[index]["_yaw_unwrapped"] = value
    start_count = len(start_rows)
    final_count = len(final_rows)
    start_unwrapped = _mean(row["_yaw_unwrapped"] for row in ordered[:start_count])
    final_unwrapped = _mean(row["_yaw_unwrapped"] for row in ordered[-final_count:])
    theta_odom = final_unwrapped - start_unwrapped
    theta_true = (1.0 if direction == "CCW" else -1.0) * int(expected_turns) * 2.0 * math.pi
    rotation_error = theta_odom - theta_true
    scale = theta_odom / theta_true
    inferred_b = current_b * scale
    increments = [
        current - previous
        for previous, current in zip(unwrapped, unwrapped[1:])
        if abs(current - previous) > 0.02
    ]
    wrong_sign_count = sum(
        (increment < 0.0 if direction == "CCW" else increment > 0.0)
        for increment in increments
    )
    warnings = []
    if theta_odom * theta_true <= 0.0:
        warnings.append("ODOMETRY_ROTATION_SIGN_MISMATCH")
    if abs(theta_odom) < 0.5 * abs(theta_true):
        warnings.append("INSUFFICIENT_TOTAL_ROTATION")
    if wrong_sign_count > max(3, len(increments) // 20):
        warnings.append("MIXED_OR_REVERSED_ROTATION_SEGMENTS")
    start = _pose_mean(start_rows)
    final = _pose_mean(final_rows)
    displacement = _start_frame_delta(start, final)
    path_length = _path_length(ordered)
    max_distance = max(
        math.hypot(row["x"] - start["x"], row["y"] - start["y"])
        for row in ordered
    )
    return {
        "status": "ok",
        "test_mode": "SPIN",
        "analysis_version": 2,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "start_pose": start,
        "end_pose": final,
        "expected_turns": int(expected_turns),
        "spin_direction": direction,
        "current_wheel_separation_m": current_b,
        "current_wheel_separation_mm": current_b * 1000.0,
        "theta_odom_rad": theta_odom,
        "theta_odom_deg": math.degrees(theta_odom),
        "theta_true_rad": theta_true,
        "theta_true_deg": math.degrees(theta_true),
        "rotation_error_rad": rotation_error,
        "rotation_error_deg": math.degrees(rotation_error),
        "rotation_scale_error_percent": 100.0 * (scale - 1.0),
        "inferred_wheel_separation_m": inferred_b,
        "inferred_wheel_separation_mm": inferred_b * 1000.0,
        "wheel_separation_difference_mm": (inferred_b - current_b) * 1000.0,
        "odom_path_length_m": path_length,
        "endpoint_displacement_m": displacement["position_error_m"],
        "max_distance_from_start_m": max_distance,
        "wrong_sign_increment_count": wrong_sign_count,
        "rotation_increment_count": len(increments),
        "quality_warnings": warnings,
        "telemetry_available": False,
        "limitation": (
            "The inferred separation is based on encoder odometry and known physical turn count. "
            "Tire scrub, slip, and manual physical alignment can affect the result."
        ),
    }


def compare_spin_results(results):
    """Compare independently saved CW and CCW result dictionaries."""
    grouped = {"CW": [], "CCW": []}
    for result in results:
        if result.get("status") == "ok" and result.get("test_mode") == "SPIN":
            direction = result.get("spin_direction")
            if direction in grouped:
                grouped[direction].append(result["inferred_wheel_separation_m"])
    means = {
        direction: _mean(values) for direction, values in grouped.items() if values
    }
    warnings = []
    if "CW" in means and "CCW" in means:
        difference_mm = abs(means["CW"] - means["CCW"]) * 1000.0
        if difference_mm > 1.0:
            warnings.append("DIRECTION_DEPENDENT_ROTATION_BIAS")
        return {
            "status": "ok",
            "cw_inferred_wheel_separation_m": means["CW"],
            "ccw_inferred_wheel_separation_m": means["CCW"],
            "difference_mm": difference_mm,
            "quality_warnings": warnings,
        }
    return {
        "status": "incomplete",
        "available_directions": sorted(means),
        "quality_warnings": ["BOTH_CW_AND_CCW_RESULTS_REQUIRED"],
    }


def _error_result(message, mode):
    return {
        "status": "error",
        "test_mode": mode,
        "analysis_version": 2,
        "error": str(message),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
    }


def write_report(result, path):
    with open(path, "w", encoding="utf-8") as stream:
        stream.write("Robot odometry test report\n")
        stream.write("==========================\n\n")
        stream.write(f"Mode: {result.get('test_mode', 'unknown')}\n")
        stream.write(f"Status: {result.get('status', 'unknown')}\n")
        if result.get("status") != "ok":
            stream.write(f"Error: {result.get('error', 'unknown error')}\n")
            return
        if result["test_mode"] == "CLOSED_LOOP":
            stream.write(f"Position error: {result['position_error_cm']:.3f} cm\n")
            stream.write(f"Heading error: {result['yaw_error_deg']:+.4f} deg\n")
            stream.write(
                f"Odometry-estimated travelled distance: "
                f"{result['odom_estimated_path_length_m']:.4f} m\n"
            )
            drift = result.get("endpoint_position_drift_percent")
            stream.write(
                "Endpoint position drift: "
                + (f"{drift:.4f}%" if drift is not None else "undefined (zero travel)")
                + "\n"
            )
            stream.write(
                f"Forward error: {result['forward_displacement_m']:+.6f} m\n"
                f"Lateral error: {result['lateral_displacement_m']:+.6f} m\n"
            )
        elif result["test_mode"] == "STRAIGHT_STOP":
            stream.write(f"Valid straight-stop events: {result['valid_segment_count']}\n")
            stream.write(f"Rejected candidate segments: {result['rejected_segment_count']}\n")
            stream.write(f"Mean cruise yaw: {result['mean_cruise_yaw_deg']:+.4f} deg\n")
            stream.write(f"Mean stop yaw: {result['mean_stop_yaw_deg']:+.4f} deg\n")
            stream.write(f"Maximum absolute stop yaw: {result['max_abs_stop_yaw_deg']:.4f} deg\n")
            fraction = result.get("fraction_total_abs_yaw_after_release_percent")
            stream.write(
                "Absolute yaw after release: "
                + (f"{fraction:.2f}%" if fraction is not None else "undefined")
                + "\n"
            )
            stream.write(f"Dominant phase: {result['dominant_phase']}\n")
            stream.write(f"Stop yaw signs: {result['stop_yaw_sign_counts']}\n")
            stream.write("Wheel target/measured/count/PWM telemetry: unavailable\n")
        elif result["test_mode"] == "SPIN":
            stream.write(f"Direction: {result['spin_direction']}\n")
            stream.write(f"Expected rotations: {result['expected_turns']}\n")
            stream.write(f"Odometry rotation: {result['theta_odom_deg']:+.4f} deg\n")
            stream.write(f"Expected rotation: {result['theta_true_deg']:+.4f} deg\n")
            stream.write(f"Rotation error: {result['rotation_error_deg']:+.4f} deg\n")
            stream.write(f"Rotation scale error: {result['rotation_scale_error_percent']:+.5f}%\n")
            stream.write(
                f"Current wheel separation: {result['current_wheel_separation_mm']:.3f} mm\n"
            )
            stream.write(
                f"Inferred wheel separation: {result['inferred_wheel_separation_mm']:.3f} mm\n"
            )
            stream.write(
                f"Difference: {result['wheel_separation_difference_mm']:+.3f} mm\n"
            )
            stream.write(f"Quality warnings: {result['quality_warnings']}\n")
        if "limitation" in result:
            stream.write(f"\nLimitation: {result['limitation']}\n")


def write_result_artifacts(result, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    result_path = os.path.join(output_dir, "odom_drift_result.json")
    report_path = os.path.join(output_dir, "odom_drift_report.txt")
    with open(result_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    # Keep the historical filename used by the phone UI and also provide the
    # generic suite filename requested for new experiments.
    suite_result_path = os.path.join(output_dir, "odom_test_result.json")
    with open(suite_result_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    write_report(result, report_path)
    return result_path, report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="saved odom_trajectory.csv")
    parser.add_argument("--output-dir", required=True, help="timestamped result directory")
    parser.add_argument(
        "--mode",
        choices=["CLOSED_LOOP", "STRAIGHT_STOP", "SPIN"],
        default="CLOSED_LOOP",
    )
    parser.add_argument("--direction", choices=["CW", "CCW"], default="CW")
    parser.add_argument("--expected-turns", type=int, default=5)
    parser.add_argument("--wheel-separation-m", type=float, default=0.2216)
    parser.add_argument("--minimum-samples", type=int, default=10)
    parser.add_argument("--minimum-duration-s", type=float, default=5.0)
    parser.add_argument("--minimum-path-length-m", type=float, default=0.05)
    parser.add_argument(
        "--allow-stationary",
        action="store_true",
        help="allow a zero-travel CLOSED_LOOP sanity test",
    )
    args = parser.parse_args(argv)

    try:
        rows = load_trajectory_csv(args.input)
        if args.mode == "CLOSED_LOOP":
            result = analyze_rows(
                rows,
                minimum_samples=args.minimum_samples,
                minimum_duration_s=args.minimum_duration_s,
                minimum_path_length_m=args.minimum_path_length_m,
                allow_stationary=args.allow_stationary,
            )
        elif args.mode == "STRAIGHT_STOP":
            result = analyze_straight_stop(rows)
        else:
            result = analyze_spin(
                rows,
                expected_turns=args.expected_turns,
                direction=args.direction,
                current_wheel_separation_m=args.wheel_separation_m,
            )
    except AnalysisError as exc:
        result = _error_result(str(exc), args.mode)

    write_result_artifacts(result, args.output_dir)
    print(json.dumps(result, sort_keys=True))
    return 0 if result.get("status") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
