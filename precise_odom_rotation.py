#!/usr/bin/env python3
"""Actuator-aware, closed-loop in-place rotation using calibrated /odom.

This is a test utility. It does not alter the production motor node, wheel
parameters, encoder signs, or lidar configuration. The outer loop controls
body angular velocity; the production node continues to close the individual
wheel-speed PI loops.
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
MAX_OMEGA = 0.50
KP = 1.20
KD = 0.35
STOP_LATENCY_S = 0.08
STOP_DECEL_RAD_S2 = 1.80
STOP_MARGIN_RAD = math.radians(0.35)
TRIM_SETTLE_S = 0.55


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def coast_distance(angular_speed):
    """Conservative distance travelled after issuing zero command."""
    speed = abs(angular_speed)
    return (
        speed * STOP_LATENCY_S
        + (speed * speed) / (2.0 * STOP_DECEL_RAD_S2)
        + STOP_MARGIN_RAD
    )


def pid_command(error, measured_omega, direction):
    """Actuator-aware PD/PID velocity command in the requested direction.

    The motor node intentionally has a real deadband. A mathematically tiny
    angular command would therefore be silently converted to its minimum
    usable wheel speed. We keep the continuous PD law, then quantize only the
    nonzero output to the known minimum actuator command.
    """
    signed_error = direction * error
    signed_speed = direction * measured_omega
    raw = KP * signed_error - KD * signed_speed
    if raw <= 0.0:
        return 0.0
    return direction * min(MAX_OMEGA, max(MIN_EFFECTIVE_OMEGA, raw))


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
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.odom_history.append((stamp_ns, receive_time, self.pose_x, self.pose_y, self.unwrapped_yaw))

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


def send_for_period(node, omega, duration_s):
    """Publish at the production control period, then return only stopped."""
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_omega(omega)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(max(0.0, CONTROL_PERIOD_S - 0.01))
    node.publish_omega(0.0)


def run_rotation(node, angle_rad, direction, tolerance_rad, capture_scans=False):
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
        print("LaserScan capture: enabled", flush=True)
    print(f"Initial unwrapped yaw: {math.degrees(start):+.4f} deg", flush=True)
    print(f"Target rotation: {math.degrees(angle_rad):.4f} deg", flush=True)

    next_tick = time.monotonic()
    phase = "closed_loop"
    while rclpy.ok():
        now = time.monotonic()
        if now < next_tick:
            rclpy.spin_once(node, timeout_sec=min(0.01, next_tick - now))
            continue
        next_tick += CONTROL_PERIOD_S
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.fault:
            node.publish_omega(0.0)
            raise RuntimeError(f"Motor safety fault: {node.fault}")
        if time.monotonic() - node.last_odom_monotonic > 0.30:
            node.publish_omega(0.0)
            raise RuntimeError("/odom became stale; motion aborted")

        error = target - node.unwrapped_yaw
        signed_speed = direction * node.angular_velocity
        if signed_speed >= 0.0 and error * direction <= coast_distance(node.angular_velocity):
            node.publish_omega(0.0)
            phase = "coast_stop"
            print(
                f"Stop threshold reached: progress={math.degrees(node.unwrapped_yaw-start):.4f} deg "
                f"remaining={math.degrees(error):+.4f} deg "
                f"speed={math.degrees(node.angular_velocity):+.4f} deg/s",
                flush=True,
            )
            break

        cmd = pid_command(error, node.angular_velocity, direction)
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

    # The actuator is quantized by the motor-node deadband, so the final
    # correction is a bounded, measured pulse loop rather than a fake
    # sub-deadband command. Every pulse spans complete 50-ms motor updates.
    for index in range(12):
        error = target - node.unwrapped_yaw
        error_deg = math.degrees(error)
        if abs(error) <= tolerance_rad:
            break
        trim_direction = 1.0 if error > 0.0 else -1.0
        # Use a 60--100 ms pulse. Below 50 ms the production motor timer may
        # never apply the target before the zero command arrives.
        pulse_s = max(0.060, min(0.100, abs(error) / 0.30))
        print(
            f"Trim {index + 1}: error={error_deg:+.4f} deg, "
            f"command={trim_direction * MIN_EFFECTIVE_OMEGA:+.3f}, "
            f"duration={pulse_s * 1000:.0f} ms",
            flush=True,
        )
        send_for_period(node, trim_direction * MIN_EFFECTIVE_OMEGA, pulse_s)
        spin_for(node, TRIM_SETTLE_S)

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
        range_min=np.asarray([record["range_min"] for record in node.scan_records]),
        range_max=np.asarray([record["range_max"] for record in node.scan_records]),
        scan_stamp_ns=np.asarray([record["scan_stamp_ns"] for record in node.scan_records], dtype=np.int64),
        odom_stamp_ns=np.asarray([record["odom_stamp_ns"] for record in node.scan_records], dtype=np.int64),
        pose_x=np.asarray([record["pose_x"] for record in node.scan_records]),
        pose_y=np.asarray([record["pose_y"] for record in node.scan_records]),
        pose_yaw=np.asarray([record["pose_yaw"] for record in node.scan_records]),
    )
    print(f"Raw synchronized capture saved: {output_path}", flush=True)


def self_test():
    assert abs(abs(wrap_pi(3.0 * math.pi)) - math.pi) < 1e-12
    assert abs(coast_distance(0.4) - (0.4 * STOP_LATENCY_S + 0.4**2 / (2 * STOP_DECEL_RAD_S2) + STOP_MARGIN_RAD)) < 1e-12
    assert pid_command(2.0, 0.0, 1.0) == MAX_OMEGA
    assert pid_command(-0.1, 0.0, 1.0) == 0.0
    assert pid_command(-1.0, 0.0, -1.0) < 0.0
    print("self-test: PASS")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle-deg", type=float, default=360.0)
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--tolerance-deg", type=float, default=0.75)
    parser.add_argument("--capture-lidar-xy", action="store_true")
    parser.add_argument("--capture-output", default="/home/robot1/r1_lidar_xy_rotation_capture.npz")
    parser.add_argument("--analysis-output", default="/home/robot1/r1_lidar_xy_rotation_analysis.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if args.angle_deg <= 0.0:
        parser.error("--angle-deg must be positive")

    rclpy.init()
    node = OdomFeedback()
    try:
        final_error = run_rotation(
            node,
            math.radians(args.angle_deg),
            args.direction,
            math.radians(args.tolerance_deg),
            capture_scans=args.capture_lidar_xy,
        )
        if args.capture_lidar_xy:
            save_scan_capture(node, args.capture_output)
            analyze_lidar_xy(node, args.analysis_output)
    finally:
        node.publish_omega(0.0)
        spin_for(node, 0.3)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
