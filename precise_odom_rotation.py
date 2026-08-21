#!/usr/bin/env python3
"""Profiled, closed-loop in-place rotation using calibrated /odom.

This is a test utility. It does not alter the production motor node, wheel
parameters, encoder signs, or lidar configuration. The outer loop generates
a cruise/braking velocity profile from unwrapped /odom; the production node
continues to close the individual wheel-speed PI loops.
"""

import argparse
from collections import deque
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node


CONTROL_PERIOD_S = 0.05  # Match real_diffdrive_node.py CONTROL_DT.
MIN_EFFECTIVE_OMEGA = 0.40  # Production motor deadband maps below this upward.
# Match the phone controller's current maximum spin command.
MAX_OMEGA = 0.75
STOP_LATENCY_S = 0.08
STOP_DECEL_RAD_S2 = 1.80
STOP_SPEED_TOLERANCE_RAD_S = 0.05
BRAKE_BUFFER_RAD = math.radians(60.0)
BRAKE_COMMAND_DECEL_RAD_S2 = 0.65
# Do not add a post-stop nudge.  The controller must finish with one
# continuous deceleration followed by zero, like a normal closed-loop stop.
# Empirical compensation for the measured odom distance travelled after the
# zero command reaches the production motor node.  This belongs only to this
# standalone rotation controller; it is not a wheel-separation, wheel-radius,
# CPR, lidar, or physical-calibration value.
STOP_MARGIN_RAD = math.radians(1.65)


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def coast_distance(angular_speed, margin_rad=STOP_MARGIN_RAD):
    """Conservative distance travelled after issuing zero command."""
    speed = abs(angular_speed)
    return (
        speed * STOP_LATENCY_S
        + (speed * speed) / (2.0 * STOP_DECEL_RAD_S2)
        + margin_rad
    )


def profile_command(
    remaining,
    measured_omega,
    direction,
    previous_command=None,
    stop_margin_rad=STOP_MARGIN_RAD,
    max_omega=MAX_OMEGA,
):
    """Return (command, phase) for a bounded cruise/braking profile.

    The command remains at the requested cruise speed until a deliberately
    calculated braking window. Inside that window its *command* is ramped
    down at a bounded rate, rather than jumping from cruise to the motor
    deadband floor. The motor node's minimum usable command is respected, and
    the command becomes exactly zero once the measured-speed coast distance
    reaches the target. There is deliberately no integral term and no
    post-stop correction pulse.
    """
    remaining = float(remaining)
    if remaining <= 0.0:
        return 0.0, "overshot", 0.0

    command_magnitude = (
        max_omega
        if previous_command is None
        else min(max_omega, max(MIN_EFFECTIVE_OMEGA, abs(float(previous_command))))
    )

    speed = max(0.0, direction * float(measured_omega))

    braking_start = coast_distance(max_omega, stop_margin_rad) + BRAKE_BUFFER_RAD
    if (
        remaining <= coast_distance(speed, stop_margin_rad)
        and command_magnitude <= MIN_EFFECTIVE_OMEGA + STOP_SPEED_TOLERANCE_RAD_S
        and speed <= MIN_EFFECTIVE_OMEGA + STOP_SPEED_TOLERANCE_RAD_S
    ):
        return 0.0, "stop", 0.0

    if remaining > braking_start:
        return direction * max_omega, "cruise", max_omega

    command_magnitude = max(
        MIN_EFFECTIVE_OMEGA,
        command_magnitude - BRAKE_COMMAND_DECEL_RAD_S2 * CONTROL_PERIOD_S,
    )
    return direction * command_magnitude, "braking", command_magnitude


class OdomFeedback(Node):
    def __init__(self):
        super().__init__("r1_precise_odom_rotation")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_unstamped", 10)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.create_subscription(String, "/motor_safety/fault", self._fault_cb, 10)
        self.unwrapped_yaw = None
        self._last_wrapped_yaw = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.angular_velocity = 0.0
        self.last_odom_monotonic = 0.0
        self.odom_sequence = 0
        self.fault = None
        self.odom_history = deque(maxlen=4000)
        self.capture_scans = False
        self.scan_records = []

    def _odom_cb(self, msg):
        wrapped = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.unwrapped_yaw is None:
            self.unwrapped_yaw = wrapped
            self._last_wrapped_yaw = wrapped
        else:
            self.unwrapped_yaw += wrap_pi(wrapped - self._last_wrapped_yaw)
            self._last_wrapped_yaw = wrapped
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.angular_velocity = float(msg.twist.twist.angular.z)
        receive_time = time.monotonic()
        self.last_odom_monotonic = receive_time
        self.odom_sequence += 1
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.odom_history.append((
            stamp_ns,
            receive_time,
            self.pose_x,
            self.pose_y,
            self.unwrapped_yaw,
            self.angular_velocity,
        ))

    def _scan_cb(self, msg):
        if not self.capture_scans or not self.odom_history:
            return
        receive_time = time.monotonic()
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns:
            sample = min(self.odom_history, key=lambda item: abs(item[0] - stamp_ns))
        else:
            sample = min(self.odom_history, key=lambda item: abs(item[1] - receive_time))
        self.scan_records.append({
            "ranges": tuple(float(value) for value in msg.ranges),
            "angle_min": float(msg.angle_min),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "scan_stamp_ns": stamp_ns,
            "receive_time": receive_time,
            "odom_stamp_ns": int(sample[0]),
            "pose_x": float(sample[2]),
            "pose_y": float(sample[3]),
            "pose_yaw": float(sample[4]),
            "odom_angular_velocity": float(sample[5]),
            "scan_time": float(msg.scan_time),
            "time_increment": float(msg.time_increment),
        })

    def _fault_cb(self, msg):
        self.fault = msg.data

    def publish_omega(self, omega):
        msg = Twist()
        msg.angular.z = float(omega)
        self.cmd_pub.publish(msg)


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=min(0.01, max(0.0, end - time.monotonic())))


def wait_for_odom(node, timeout_s):
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.unwrapped_yaw is not None:
            return True
    return False


def run_rotation(
    node,
    angle_rad,
    direction,
    tolerance_rad,
    max_omega=MAX_OMEGA,
    stop_margin_rad=STOP_MARGIN_RAD,
    capture_scans=False,
):
    if not wait_for_odom(node, 4.0):
        raise RuntimeError("No /odom received")
    node.publish_omega(0.0)
    spin_for(node, 0.5)
    if node.fault:
        raise RuntimeError(f"Motor safety fault: {node.fault}")

    start = node.unwrapped_yaw
    target = start + direction * angle_rad
    if capture_scans:
        node.scan_records.clear()
        node.capture_scans = True
        warmup_deadline = time.monotonic() + 4.0
        while (
            rclpy.ok()
            and len(node.scan_records) < 3
            and time.monotonic() < warmup_deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.05)
        if len(node.scan_records) < 3:
            node.capture_scans = False
            raise RuntimeError(
                "LaserScan warm-up failed: fewer than 3 live scans received"
            )
        node.scan_records.clear()
        print("LaserScan capture: enabled after 3-scan warm-up", flush=True)
    print(f"Initial unwrapped yaw: {math.degrees(start):+.4f} deg", flush=True)
    print(f"Target rotation: {math.degrees(angle_rad):.4f} deg", flush=True)
    print(f"Closed-loop speed ceiling: {max_omega:.3f} rad/s", flush=True)
    print(f"Stop-margin compensation: {math.degrees(stop_margin_rad):.3f} deg", flush=True)
    print(f"Acceptance tolerance: {math.degrees(tolerance_rad):.4f} deg", flush=True)

    # Drive the outer position controller from fresh /odom samples rather than
    # from an unrelated wall-clock timer.  The production motor loop remains
    # 20 Hz; this only removes an avoidable one-cycle command reaction delay in
    # the standalone test controller.
    last_odom_sequence = node.odom_sequence
    phase = "cruise"
    command_magnitude = max_omega
    while rclpy.ok():
        while rclpy.ok() and node.odom_sequence == last_odom_sequence:
            rclpy.spin_once(node, timeout_sec=CONTROL_PERIOD_S)
            if time.monotonic() - node.last_odom_monotonic > 0.30:
                node.publish_omega(0.0)
                raise RuntimeError("/odom became stale; motion aborted")
        if not rclpy.ok():
            break
        last_odom_sequence = node.odom_sequence
        now = time.monotonic()
        if node.fault:
            node.publish_omega(0.0)
            raise RuntimeError(f"Motor safety fault: {node.fault}")
        if time.monotonic() - node.last_odom_monotonic > 0.30:
            node.publish_omega(0.0)
            raise RuntimeError("/odom became stale; motion aborted")

        error = target - node.unwrapped_yaw
        remaining = direction * error
        cmd, next_phase, command_magnitude = profile_command(
            remaining,
            node.angular_velocity,
            direction,
            previous_command=command_magnitude,
            stop_margin_rad=stop_margin_rad,
            max_omega=max_omega,
        )
        if next_phase in {"stop", "overshot"}:
            node.publish_omega(0.0)
            phase = next_phase
            print(
                f"Stop threshold reached: progress={math.degrees(node.unwrapped_yaw-start):.4f} deg "
                f"remaining={math.degrees(error):+.4f} deg "
                f"speed={math.degrees(node.angular_velocity):+.4f} deg/s",
                flush=True,
            )
            break

        phase = next_phase
        node.publish_omega(cmd)
        if int(now * 2) != int((now - CONTROL_PERIOD_S) * 2):
            print(
                f"{phase}: progress={math.degrees(node.unwrapped_yaw-start):.3f} deg "
                f"error={math.degrees(error):+.3f} deg "
                f"omega={math.degrees(node.angular_velocity):+.3f} deg/s "
                f"cmd={cmd:+.3f}",
                flush=True,
            )

    node.publish_omega(0.0)
    spin_for(node, 1.2)
    # Deliberately no trim pulse here.  If the continuous stop lands outside
    # tolerance, the test reports FAIL so the stop model/controller can be
    # improved explicitly rather than hiding the error with a visible nudge.
    node.publish_omega(0.0)
    spin_for(node, 1.5)
    if capture_scans:
        node.capture_scans = False
        print(f"LaserScan capture: {len(node.scan_records)} synchronized scans", flush=True)
    final_delta = node.unwrapped_yaw - start
    final_error = final_delta - direction * angle_rad
    print(f"Final accumulated rotation: {math.degrees(final_delta):+.5f} deg", flush=True)
    print(f"Final heading error: {math.degrees(final_error):+.5f} deg", flush=True)
    print("Final command: zero; settle: 1.5 s", flush=True)
    return final_error


def _scan_points(record, stride=3):
    import numpy as np

    ranges = np.asarray(record["ranges"], dtype=float)
    indices = np.arange(0, ranges.size, stride)
    values = ranges[indices]
    valid = np.isfinite(values)
    valid &= values >= max(0.05, record["range_min"])
    valid &= values <= min(8.0, record["range_max"])
    indices = indices[valid]
    values = values[valid]
    angles = record["angle_min"] + indices * record["angle_increment"]
    return np.column_stack((values * np.cos(angles), values * np.sin(angles)))


def _select_heading_scans(records):
    """Select approximately one scan per 5-degree odometric heading bin."""
    import numpy as np

    usable = []
    for record in records:
        points = _scan_points(record)
        if points.shape[0] >= 60:
            usable.append((record, points))
    if not usable:
        raise RuntimeError("No usable synchronized scans were captured")

    first_yaw = usable[0][0]["pose_yaw"]
    bin_width = math.radians(5.0)
    bins = {}
    for record, points in usable:
        progress = record["pose_yaw"] - first_yaw
        index = int(round(progress / bin_width))
        if index < 0 or index > 74:
            continue
        distance_from_bin = abs(progress - index * bin_width)
        old = bins.get(index)
        if old is None or distance_from_bin < old[0]:
            bins[index] = (distance_from_bin, record, points)

    selected = [(item[1], item[2]) for _, item in sorted(bins.items())]
    if len(selected) < 20:
        raise RuntimeError(f"Only {len(selected)} heading bins contained usable scans")
    return selected


def _make_scan_pairs(selected):
    import numpy as np

    count = len(selected)
    offsets = sorted(set(max(2, round(count * fraction)) for fraction in (0.125, 0.25, 0.375, 0.50)))
    pairs = []
    for index in range(count):
        for offset in offsets:
            other = index + offset
            if other < count:
                pairs.append((index, other))
    # Keep the objective balanced across the turn while limiting processing.
    if len(pairs) > 160:
        chosen = np.linspace(0, len(pairs) - 1, 160, dtype=int)
        pairs = [pairs[index] for index in chosen]
    return pairs


def _pair_distances(selected, pairs, dx, dy):
    import numpy as np
    from scipy.spatial import cKDTree

    translation = np.array([dx, dy], dtype=float)
    trees = [cKDTree(points) for _, points in selected]
    distances = []
    for first, second in pairs:
        first_record, first_points = selected[first]
        second_record, second_points = selected[second]
        theta_first = first_record["pose_yaw"]
        theta_second = second_record["pose_yaw"]
        c1, s1 = math.cos(theta_first), math.sin(theta_first)
        c2, s2 = math.cos(theta_second), math.sin(theta_second)
        r1 = np.array([[c1, -s1], [s1, c1]])
        r2 = np.array([[c2, -s2], [s2, c2]])
        r2t = np.array([[c2, s2], [-s2, c2]])
        base_first = np.array([first_record["pose_x"], first_record["pose_y"]])
        base_second = np.array([second_record["pose_x"], second_record["pose_y"]])

        # Convert first-scan points into the second lidar frame under the
        # candidate base_link -> lidar translation.
        world_first = base_first + (translation + first_points) @ r1.T
        predicted_second = (r2t @ (world_first - base_second).T).T - translation
        d12 = trees[second].query(predicted_second, k=1, distance_upper_bound=0.40)[0]

        world_second = base_second + (translation + second_points) @ r2.T
        r1t = np.array([[c1, s1], [-s1, c1]])
        predicted_first = (r1t @ (world_second - base_first).T).T - translation
        d21 = trees[first].query(predicted_first, k=1, distance_upper_bound=0.40)[0]

        distances.append(d12[np.isfinite(d12) & (d12 < 0.40)])
        distances.append(d21[np.isfinite(d21) & (d21 < 0.40)])
    if not distances:
        return np.empty(0)
    return np.concatenate(distances)


def _objective(selected, pairs, dx, dy):
    import numpy as np

    distances = _pair_distances(selected, pairs, dx, dy)
    if distances.size < 100:
        return float("inf")
    clipped = np.minimum(distances, 0.20)
    return float(np.sqrt(np.mean(clipped * clipped)))


def _fit_grid(selected, pairs, center=(0.0, 0.0), half_width=0.020, step=0.002):
    import numpy as np

    cx, cy = center
    values = np.arange(-half_width, half_width + step * 0.5, step)
    best = (float("inf"), 0.0, 0.0)
    for dx_offset in values:
        for dy_offset in values:
            dx = cx + float(dx_offset)
            dy = cy + float(dy_offset)
            value = _objective(selected, pairs, dx, dy)
            if value < best[0]:
                best = (value, dx, dy)
    return best


def analyze_lidar_xy(node, output_path):
    import json
    import numpy as np

    selected = _select_heading_scans(node.scan_records)
    pairs = _make_scan_pairs(selected)
    baseline = _objective(selected, pairs, 0.0, 0.0)
    coarse = _fit_grid(selected, pairs, half_width=0.020, step=0.002)
    fitted = _fit_grid(selected, pairs, center=(coarse[1], coarse[2]), half_width=0.003, step=0.0005)

    bootstrap = []
    for group in range(4):
        subset = [pair for index, pair in enumerate(pairs) if index % 4 == group]
        if len(subset) >= 12:
            result = _fit_grid(
                selected,
                subset,
                center=(fitted[1], fitted[2]),
                half_width=0.004,
                step=0.001,
            )
            bootstrap.append({"dx": result[1], "dy": result[2], "residual": result[0]})

    dx_values = np.asarray([item["dx"] for item in bootstrap], dtype=float)
    dy_values = np.asarray([item["dy"] for item in bootstrap], dtype=float)
    improvement = 100.0 * (baseline - fitted[0]) / baseline if baseline else 0.0
    progress = [record["pose_yaw"] for record, _ in selected]
    result = {
        "scan_count_captured": len(node.scan_records),
        "heading_count_used": len(selected),
        "pair_count_used": len(pairs),
        "heading_span_deg": math.degrees(max(progress) - min(progress)),
        "baseline_dx_m": 0.0,
        "baseline_dy_m": 0.0,
        "baseline_residual_m": baseline,
        "fitted_dx_m": fitted[1],
        "fitted_dy_m": fitted[2],
        "fitted_residual_m": fitted[0],
        "improvement_percent": improvement,
        "bootstrap": bootstrap,
        "bootstrap_std_dx_m": float(np.std(dx_values)) if dx_values.size else None,
        "bootstrap_std_dy_m": float(np.std(dy_values)) if dy_values.size else None,
        "method": "whole-trajectory bidirectional nearest-neighbor scan consistency",
    }
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2), flush=True)
    return result


def save_scan_capture(node, output_path):
    import numpy as np

    if not node.scan_records:
        raise RuntimeError("No scan records to save")
    count = min(len(record["ranges"]) for record in node.scan_records)
    ranges = np.asarray([record["ranges"][:count] for record in node.scan_records], dtype=np.float32)
    np.savez_compressed(
        output_path,
        ranges=ranges,
        angle_min=np.asarray([record["angle_min"] for record in node.scan_records]),
        angle_increment=np.asarray([record["angle_increment"] for record in node.scan_records]),
        scan_time=np.asarray([record.get("scan_time", 0.0) for record in node.scan_records]),
        time_increment=np.asarray([record.get("time_increment", 0.0) for record in node.scan_records]),
        range_min=np.asarray([record["range_min"] for record in node.scan_records]),
        range_max=np.asarray([record["range_max"] for record in node.scan_records]),
        scan_stamp_ns=np.asarray([record["scan_stamp_ns"] for record in node.scan_records], dtype=np.int64),
        odom_stamp_ns=np.asarray([record["odom_stamp_ns"] for record in node.scan_records], dtype=np.int64),
        pose_x=np.asarray([record["pose_x"] for record in node.scan_records]),
        pose_y=np.asarray([record["pose_y"] for record in node.scan_records]),
        pose_yaw=np.asarray([record["pose_yaw"] for record in node.scan_records]),
        odom_angular_velocity=np.asarray([
            record.get("odom_angular_velocity", 0.0) for record in node.scan_records
        ]),
    )
    print(f"Raw synchronized capture saved: {output_path}", flush=True)


def _scan_profile(record):
    """Return a finite, acquisition-midpoint-deskewed range profile."""
    import numpy as np

    values = np.asarray(record["ranges"], dtype=float)
    valid = np.isfinite(values)
    valid &= values >= max(0.05, float(record["range_min"]))
    valid &= values <= min(8.0, float(record["range_max"]))

    # During a fast rotation, one D500 revolution is not instantaneous. The
    # production driver mirrors the raw scan by mapping output bin i to raw
    # bin j. Therefore acquisition time must follow raw bin j, not mirrored
    # output index i. Using i here applies the deskew in the wrong temporal
    # direction and biases accumulated lidar rotation.
    increment = float(record.get("angle_increment", 0.0))
    angle_min = float(record.get("angle_min", 0.0))
    time_increment = float(record.get("time_increment", 0.0))
    angular_velocity = float(record.get("odom_angular_velocity", 0.0))
    if (
        values.size
        and increment > 0.0
        and time_increment > 0.0
        and math.isfinite(angular_velocity)
        and abs(angular_velocity) > 1e-4
    ):
        indices = np.arange(values.size, dtype=float)
        raw_angles = (-(angle_min + indices * increment)) % (2.0 * math.pi)
        raw_indices = np.rint(raw_angles / increment).astype(int) % values.size
        time_from_midpoint = (
            raw_indices.astype(float) - 0.5 * (values.size - 1.0)
        ) * time_increment
        corrected_angles = (
            angle_min + indices * increment + angular_velocity * time_from_midpoint
        )
        corrected_indices = np.rint(
            (corrected_angles - angle_min) / increment
        ).astype(int) % values.size
        deskewed = np.full(values.size, np.nan, dtype=float)
        deskewed_valid = np.zeros(values.size, dtype=bool)
        valid_indices = np.flatnonzero(valid)
        for source_index in valid_indices:
            destination = corrected_indices[source_index]
            if (
                not deskewed_valid[destination]
                or values[source_index] < deskewed[destination]
            ):
                deskewed[destination] = values[source_index]
                deskewed_valid[destination] = True
        values = deskewed
        valid = deskewed_valid
    return values, valid


def _scan_shift_score(previous, previous_valid, current, current_valid, shift):
    """Score one circular-bin shift; lower is a better geometric match."""
    import numpy as np

    aligned_current = np.roll(current, shift)
    aligned_valid = np.roll(current_valid, shift)
    valid = previous_valid & aligned_valid
    if int(np.count_nonzero(valid)) < 120:
        return float("inf")
    difference = np.abs(previous[valid] - aligned_current[valid])
    # Robustly limit a few moving/occluded rays from dominating the score.
    return float(np.mean(np.minimum(difference, 0.50)))


def _best_scan_shift(previous_record, current_record):
    """Estimate the relative lidar rotation between adjacent scans."""
    import numpy as np

    previous, previous_valid = _scan_profile(previous_record)
    current, current_valid = _scan_profile(current_record)
    count = min(previous.size, current.size)
    previous = previous[:count]
    previous_valid = previous_valid[:count]
    current = current[:count]
    current_valid = current_valid[:count]
    increment = float(current_record["angle_increment"])
    if not math.isfinite(increment) or increment <= 0.0:
        return None

    max_shift_bins = max(4, int(round(math.radians(30.0) / increment)))
    candidates = []
    for shift in range(-max_shift_bins, max_shift_bins + 1):
        score = _scan_shift_score(
            previous,
            previous_valid,
            current,
            current_valid,
            shift,
        )
        if math.isfinite(score):
            candidates.append((score, shift))
    if not candidates:
        return None

    candidates.sort()
    best_score, best_shift = candidates[0]
    second_score = candidates[1][0] if len(candidates) > 1 else float("inf")

    # Quadratic interpolation around the integer-bin minimum gives a more
    # useful estimate than accumulating a whole 0.5-degree quantization error
    # on every scan pair.
    fractional_shift = float(best_shift)
    if -max_shift_bins < best_shift < max_shift_bins:
        left = _scan_shift_score(
            previous,
            previous_valid,
            current,
            current_valid,
            best_shift - 1,
        )
        right = _scan_shift_score(
            previous,
            previous_valid,
            current,
            current_valid,
            best_shift + 1,
        )
        curvature = left - 2.0 * best_score + right
        if curvature > 1e-9:
            offset = 0.5 * (left - right) / curvature
            fractional_shift += max(-0.5, min(0.5, offset))

    return {
        "shift_bins": fractional_shift,
        "shift_rad": fractional_shift * increment,
        "score": best_score,
        "second_score": second_score,
        "valid_bins": int(np.count_nonzero(previous_valid)),
    }


def analyze_lidar_rotation(node, expected_angle_deg, direction, output_path):
    """Estimate accumulated physical rotation from consecutive lidar scans.

    This is intentionally separate from odometry. It sums the absolute
    scan-to-scan angular changes, so a five-turn test is not reduced modulo
    360 degrees. The scan geometry is used only to match successive static
    environmental range profiles; odometry is not used to calculate the
    accumulated lidar angle.
    """
    import json
    import numpy as np

    records = node.scan_records
    pair_results = []
    cumulative_lidar_rad = 0.0
    cumulative_odom_rad = 0.0
    trajectory_points = []
    for previous, current in zip(records, records[1:]):
        result = _best_scan_shift(previous, current)
        if result is not None:
            pair_results.append(result)
            cumulative_lidar_rad += abs(float(result["shift_rad"]))
            odom_delta = abs(
                float(current.get("pose_yaw", 0.0))
                - float(previous.get("pose_yaw", 0.0))
            )
            cumulative_odom_rad += odom_delta
            trajectory_points.append((cumulative_odom_rad, cumulative_lidar_rad))

    if len(pair_results) < 8:
        raise RuntimeError(
            f"Only {len(pair_results)} usable lidar scan pairs; "
            "not enough for independent rotation validation"
        )

    increments = np.asarray([item["shift_rad"] for item in pair_results], dtype=float)
    increment_abs_deg = np.degrees(np.abs(increments))
    accumulated_deg = float(np.sum(increment_abs_deg))
    expected_abs_deg = abs(float(expected_angle_deg))
    error_deg = accumulated_deg - expected_abs_deg
    scores = np.asarray([item["score"] for item in pair_results], dtype=float)
    fit_points = np.asarray(trajectory_points, dtype=float)
    fit_margin_rad = math.radians(20.0)
    fit_mask = (
        (fit_points[:, 0] >= fit_margin_rad)
        & (fit_points[:, 0] <= max(fit_margin_rad, cumulative_odom_rad - fit_margin_rad))
    )
    if int(np.count_nonzero(fit_mask)) >= 8:
        fit_slope, fit_intercept = np.polyfit(
            fit_points[fit_mask, 0],
            fit_points[fit_mask, 1],
            1,
        )
        fitted_lidar_for_target_deg = math.degrees(fit_slope * math.radians(expected_abs_deg))
        fit_residual_deg = fitted_lidar_for_target_deg - expected_abs_deg
    else:
        fit_slope = float("nan")
        fit_intercept = float("nan")
        fitted_lidar_for_target_deg = float("nan")
        fit_residual_deg = float("nan")
    result = {
        "method": "midpoint-deskewed consecutive-scan circular range-profile matching",
        "scan_count_captured": len(records),
        "usable_pair_count": len(pair_results),
        "expected_turns": expected_abs_deg / 360.0,
        "expected_rotation_deg": expected_abs_deg,
        "direction_requested": int(direction),
        "lidar_accumulated_rotation_deg": accumulated_deg,
        "lidar_observed_turns": accumulated_deg / 360.0,
        "lidar_rotation_error_deg": error_deg,
        "lidar_rotation_scale_error_percent": (
            100.0 * error_deg / expected_abs_deg if expected_abs_deg else None
        ),
        "cruise_scale_fit": {
            "fit_pairs": int(np.count_nonzero(fit_mask)),
            "odom_span_deg": math.degrees(cumulative_odom_rad),
            "lidar_per_odom_scale": float(fit_slope),
            "intercept_rad": float(fit_intercept),
            "fitted_lidar_rotation_for_target_deg": fitted_lidar_for_target_deg,
            "fitted_rotation_error_deg": fit_residual_deg,
            "fitted_scale_error_percent": (
                100.0 * fit_residual_deg / expected_abs_deg
                if expected_abs_deg and math.isfinite(fit_residual_deg)
                else None
            ),
            "excluded_start_end_deg": 20.0,
        },
        "median_pair_rotation_deg": float(np.median(increment_abs_deg)),
        "p95_pair_rotation_deg": float(np.percentile(increment_abs_deg, 95)),
        "median_match_score_m": float(np.median(scores)),
        "p95_match_score_m": float(np.percentile(scores, 95)),
        "note": (
            "Accumulated angle is not wrapped modulo 360 degrees. "
            "This is an independent scan-geometry estimate and can fail in "
            "a geometrically feature-poor or highly symmetric environment."
        ),
    }
    with open(output_path, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2), flush=True)
    return result


def self_test():
    assert abs(abs(wrap_pi(3.0 * math.pi)) - math.pi) < 1e-12
    assert abs(coast_distance(0.4) - (0.4 * STOP_LATENCY_S + 0.4**2 / (2 * STOP_DECEL_RAD_S2) + STOP_MARGIN_RAD)) < 1e-12
    command, phase, magnitude = profile_command(2.0, 0.0, 1.0)
    assert command == MAX_OMEGA and magnitude == MAX_OMEGA and phase == "cruise"
    command, phase, magnitude = profile_command(
        math.radians(8.0), 0.0, 1.0, previous_command=MAX_OMEGA
    )
    assert 0.0 < command < MAX_OMEGA and phase == "braking"
    command, phase, magnitude = profile_command(
        math.radians(10.0), 0.75, 1.0, previous_command=MAX_OMEGA
    )
    assert 0.0 < command < MAX_OMEGA and phase == "braking"
    command, phase, magnitude = profile_command(
        math.radians(2.0), 0.30, 1.0, previous_command=MIN_EFFECTIVE_OMEGA
    )
    assert command == 0.0 and phase == "stop"
    command, phase, magnitude = profile_command(-0.01, 0.30, 1.0)
    assert command == 0.0 and phase == "overshot"
    command, phase, magnitude = profile_command(2.0, 0.0, -1.0)
    assert command == -MAX_OMEGA and phase == "cruise"
    print("self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle-deg", type=float, default=360.0)
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument(
        "--tolerance-deg",
        type=float,
        default=0.50,
        help="accepted final unwrapped /odom error (default: 0.50)",
    )
    parser.add_argument(
        "--max-omega",
        type=float,
        default=MAX_OMEGA,
        help="closed-loop angular-velocity ceiling in rad/s (default: 0.75)",
    )
    parser.add_argument(
        "--stop-margin-deg",
        type=float,
        default=math.degrees(STOP_MARGIN_RAD),
        help=(
            "test-controller-only coast-distance compensation in degrees "
            f"(default: {math.degrees(STOP_MARGIN_RAD):.2f}; not geometry calibration)"
        ),
    )
    parser.add_argument("--capture-lidar-xy", action="store_true")
    parser.add_argument(
        "--capture-lidar-rotation",
        action="store_true",
        help="capture scans and independently estimate accumulated rotation",
    )
    parser.add_argument("--capture-output", default="/home/robot1/r1_lidar_xy_rotation_capture.npz")
    parser.add_argument("--analysis-output", default="/home/robot1/r1_lidar_xy_rotation_analysis.json")
    parser.add_argument(
        "--rotation-analysis-output",
        default="/home/robot1/r1_lidar_rotation_analysis.json",
    )
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.angle_deg <= 0.0:
        parser.error("--angle-deg must be positive")
    if not 0.0 < args.tolerance_deg <= 10.0:
        parser.error("--tolerance-deg must be in (0, 10]")
    if not MIN_EFFECTIVE_OMEGA <= args.max_omega <= 2.0:
        parser.error(
            f"--max-omega must be in [{MIN_EFFECTIVE_OMEGA}, 2.0] rad/s"
        )
    if not -5.0 <= args.stop_margin_deg <= 20.0:
        parser.error("--stop-margin-deg must be between -5 and 20 degrees")

    rclpy.init()
    node = OdomFeedback()
    try:
        final_error = run_rotation(
            node,
            math.radians(args.angle_deg),
            args.direction,
            math.radians(args.tolerance_deg),
            max_omega=args.max_omega,
            stop_margin_rad=math.radians(args.stop_margin_deg),
            capture_scans=args.capture_lidar_xy or args.capture_lidar_rotation,
        )
        passed = abs(final_error) <= math.radians(args.tolerance_deg)
        print(
            f"Rotation result: {'PASS' if passed else 'FAIL'} "
            f"(absolute /odom error={math.degrees(abs(final_error)):.5f} deg)",
            flush=True,
        )
        if args.capture_lidar_xy:
            save_scan_capture(node, args.capture_output)
            analyze_lidar_xy(node, args.analysis_output)
        elif args.capture_lidar_rotation:
            save_scan_capture(node, args.capture_output)
        if args.capture_lidar_rotation:
            analyze_lidar_rotation(
                node,
                args.angle_deg,
                args.direction,
                args.rotation_analysis_output,
            )
    finally:
        node.publish_omega(0.0)
        spin_for(node, 0.3)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if passed else 2


if __name__ == "__main__":
    sys.exit(main())
