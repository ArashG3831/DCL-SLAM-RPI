#!/usr/bin/env python3
"""Shared command, map, diagnostics, and odometry-session components."""

import csv
import json
import math
import os
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from datetime import datetime

from geometry_msgs.msg import Twist

from phone_controller_config import (
    COMMAND_TIMEOUT_S,
    FORWARD_RPM,
    LIDAR_GAP_THRESHOLD_S,
    MAP_CHECKPOINT_INTERVAL_S,
    MAP_SAVE_DIR,
    MAP_SAVE_TIMEOUT_S,
    MAX_MAP_DISPLAY_DIM,
    MIN_EFFECTIVE_COMMAND_RPM,
    ODOM_ANALYZER,
    ODOM_FINALIZE_TIMEOUT_S,
    ODOM_FINAL_REFERENCE_SAMPLES,
    ODOM_MIN_STATIONARY_ANGULAR_RADPS,
    ODOM_MIN_STATIONARY_LINEAR_MPS,
    ODOM_MODE_CONFIGURATION_GRACE_S,
    ODOM_RESULTS_DIR,
    ODOM_START_REFERENCE_SAMPLES,
    SPIN_MAX_RADPS,
    TRANSITION_REST_S,
    WHEEL_RADIUS,
    WHEEL_SEPARATION,
)


class PhoneTimingDiagnostics:
    """Optional, asynchronous timing trace for the phone ROS/HTTP workload."""

    def __init__(self):
        self.enabled = os.environ.get("R1_PHONE_DIAGNOSTICS", "0") == "1"
        self.path = os.environ.get(
            "R1_PHONE_DIAGNOSTICS_PATH", "/tmp/r1_phone_timing.csv"
        )
        self.events = []
        self.lock = threading.Lock()
        self.wakeup = threading.Event()
        self.stop_event = threading.Event()
        self.thread = None
        if self.enabled:
            self.thread = threading.Thread(
                target=self._writer,
                name="phone_timing_writer",
                daemon=True,
            )
            self.thread.start()

    def record(self, name, phase, duration_ms=None, detail=""):
        if not self.enabled:
            return
        event = (
            time.time(),
            time.monotonic(),
            threading.current_thread().name,
            name,
            phase,
            "" if duration_ms is None else f"{duration_ms:.3f}",
            detail,
        )
        with self.lock:
            if len(self.events) < 50000:
                self.events.append(event)
        self.wakeup.set()

    @contextmanager
    def measure(self, name, detail=""):
        if not self.enabled:
            yield
            return
        started = time.monotonic()
        self.record(name, "start", detail=detail)
        try:
            yield
        finally:
            duration_ms = (time.monotonic() - started) * 1000.0
            self.record(name, "end", duration_ms=duration_ms, detail=detail)

    def _writer(self):
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.path, "w", newline="", buffering=1) as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "wall_time_iso", "wall_time_s", "monotonic_s", "thread",
                "event", "phase", "duration_ms", "detail",
            ])
            while not self.stop_event.is_set() or self.events:
                self.wakeup.wait(0.25)
                self.wakeup.clear()
                with self.lock:
                    batch = self.events[:]
                    del self.events[:]
                for wall_s, monotonic_s, thread_name, name, phase, duration, detail in batch:
                    writer.writerow([
                        datetime.fromtimestamp(wall_s).isoformat(timespec="milliseconds"),
                        f"{wall_s:.6f}",
                        f"{monotonic_s:.6f}",
                        thread_name,
                        name,
                        phase,
                        duration,
                        detail,
                    ])

    def close(self):
        if not self.enabled:
            return
        self.stop_event.set()
        self.wakeup.set()
        if self.thread is not None:
            self.thread.join(timeout=3.0)
            self.thread = None


PHONE_DIAGNOSTICS = PhoneTimingDiagnostics()


def rpm_to_mps(rpm):
    return (2.0 * math.pi * WHEEL_RADIUS) * (rpm / 60.0)


def make_twist(v=0.0, w=0.0):
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg


class SharedCommand:
    MOTION_KEYS = {"w", "s", "a", "d"}

    def __init__(self):
        self.lock = threading.Lock()
        self.key = "x"
        self.speed_rpm = FORWARD_RPM
        self.last_update = time.monotonic()
        self.rest_until = 0.0
        self.command_history = []

    @classmethod
    def normalize_key(cls, key):
        """Allow at most one key per axis and canonicalize valid pairs."""
        raw = str(key or "")
        if raw == "x":
            return "x"
        if any(char not in cls.MOTION_KEYS for char in raw):
            return "x"
        if len(raw) == 0 or len(raw) > 2 or len(set(raw)) != len(raw):
            return "x"
        if ("w" in raw and "s" in raw) or ("a" in raw and "d" in raw):
            return "x"
        return "".join(key_name for key_name in ("w", "s", "a", "d") if key_name in raw)

    @staticmethod
    def motion_axes(key):
        linear = 1 if "w" in key else -1 if "s" in key else 0
        turn = 1 if "a" in key else -1 if "d" in key else 0
        return linear, turn

    def set_key(self, key, speed_rpm=None):
        now = time.monotonic()
        key = self.normalize_key(key)

        parsed_speed = None
        if speed_rpm is not None:
            try:
                parsed_speed = float(speed_rpm)
            except (TypeError, ValueError):
                parsed_speed = None
            if parsed_speed is not None and not math.isfinite(parsed_speed):
                parsed_speed = None

        with self.lock:
            old_key = self.key
            old_linear, old_turn = self.motion_axes(old_key)
            new_linear, new_turn = self.motion_axes(key)
            changing_motion = (
                old_key != "x"
                and key != "x"
                and (
                    (old_linear and new_linear and old_linear != new_linear)
                    or (old_turn and new_turn and old_turn != new_turn)
                )
            )
            if changing_motion:
                self.rest_until = now + TRANSITION_REST_S
                print(f"Transition {old_key} -> {key}: rest {TRANSITION_REST_S:.2f}s")
            self.key = key
            if parsed_speed is not None:
                parsed_speed = max(0.0, min(FORWARD_RPM, parsed_speed))
                if key != "x" and parsed_speed > 0.0:
                    parsed_speed = max(MIN_EFFECTIVE_COMMAND_RPM, parsed_speed)
                self.speed_rpm = parsed_speed
            self.last_update = now
            linear_sign, turn_sign = self.motion_axes(self.key)
            requested_v = linear_sign * rpm_to_mps(self.speed_rpm)
            requested_w = turn_sign * SPIN_MAX_RADPS * (
                self.speed_rpm / FORWARD_RPM if FORWARD_RPM else 0.0
            )
            self.command_history.append({
                "timestamp_monotonic_s": now,
                "key": self.key,
                "slider_rpm": self.speed_rpm,
                "requested_linear_command_mps": requested_v,
                "requested_angular_command_radps": requested_w,
            })
            if len(self.command_history) > 12000:
                del self.command_history[:-12000]

    def status(self):
        with self.lock:
            return self.key, self.speed_rpm, self.last_update

    def command_state(self):
        now = time.monotonic()
        with self.lock:
            key = self.key
            speed_rpm = self.speed_rpm
            last_update = self.last_update
            rest_until = self.rest_until

        linear_sign, turn_sign = self.motion_axes(key)
        requested_v = linear_sign * rpm_to_mps(speed_rpm)
        requested_w = turn_sign * SPIN_MAX_RADPS * (
            speed_rpm / FORWARD_RPM if FORWARD_RPM else 0.0
        )
        if now < rest_until or now - last_update > COMMAND_TIMEOUT_S:
            linear_command = 0.0
            angular_command = 0.0
        else:
            linear_command = requested_v
            angular_command = requested_w
        return {
            "key": key,
            "slider_rpm": speed_rpm,
            "linear_command_mps": linear_command,
            "angular_command_radps": angular_command,
            "requested_linear_command_mps": requested_v,
            "requested_angular_command_radps": requested_w,
            "command_change_monotonic_s": last_update,
        }

    def history_since(self, timestamp_monotonic_s):
        with self.lock:
            return [
                dict(event)
                for event in self.command_history
                if event["timestamp_monotonic_s"] >= timestamp_monotonic_s
            ]

    def get_twist(self):
        state = self.command_state()
        return make_twist(
            state["linear_command_mps"],
            state["angular_command_radps"],
        )


class LiveMapState:
    def __init__(self):
        self.lock = threading.Lock()
        self.map_json_lock = threading.Lock()
        self.map = None
        self.map_json_body = None
        self.map_json_version = None
        self.map_off = None
        self.map_off_json_body = None
        self.map_off_json_version = None
        self.robot = None
        self.robot_off = None
        self.path = []
        self.path_off = []
        self.path_base_version = 0
        self.path_version = 0
        self.path_off_base_version = 0
        self.path_off_version = 0
        self.version = 0
        self.motor_rpm = {"left": 0.0, "right": 0.0}
        # Do not count lidar startup or pre-map delays. Monitoring begins only
        # when the first valid OccupancyGrid has arrived.
        self.lidar_monitor_started_at = None
        self.lidar_last_seen_at = None
        self.lidar_last_scan_at = None
        self.lidar_completed_gap_s = 0.0
        self.lidar_max_gap_s = 0.0
        self.lidar_gap_events = 0

    def update_map(self, msg):
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return

        # Copy the ROS message data before taking the shared-state lock.  The
        # copy can be large; pose, odometry, and scan callbacks must not wait
        # behind it while the map is being replaced.
        map_data = list(msg.data)
        with self.lock:
            if self.lidar_monitor_started_at is None:
                self.lidar_monitor_started_at = time.monotonic()
            self.version += 1
            self.map = {
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": map_data,
                "version": self.version,
            }
            self.map_json_body = None
            self.map_json_version = None

    def update_map_off(self, msg):
        """Store the diagnostic scan-matching-OFF map, when enabled."""
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            return
        map_data = list(msg.data)
        with self.lock:
            previous_version = self.map_off["version"] if self.map_off else 0
            self.map_off = {
                "width": width,
                "height": height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": map_data,
                "version": previous_version + 1,
            }
            self.map_off_json_body = None
            self.map_off_json_version = None

    def update_robot(self, transform):
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": float(yaw),
        }
        with self.lock:
            if self.robot is None:
                self.path = [pose]
                self.path_base_version = 0
                self.path_version = 1
            else:
                dx = pose["x"] - self.robot["x"]
                dy = pose["y"] - self.robot["y"]
                # Keep a useful trail without storing hundreds of identical
                # points while the robot is stationary.
                if dx * dx + dy * dy >= 0.001 ** 2:
                    self.path.append(pose)
                    self.path_version += 1
                    if len(self.path) > 5000:
                        dropped = len(self.path) - 5000
                        self.path = self.path[-5000:]
                        self.path_base_version += dropped
            self.robot = pose

    def update_robot_off(self, transform):
        """Store the diagnostic pose expressed in the map_off frame."""
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        pose = {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "yaw": float(yaw),
        }
        self._update_robot_off_pose(pose)

    def update_robot_off_from_odom(self, msg):
        """Track the OFF diagnostic pose directly from raw /odom.

        The OFF mapper is intentionally scan-matching-free, so its map_off
        frame is expected to coincide with odom.  Using the Odometry message
        directly keeps the diagnostic trajectory available even if the
        optional OFF mapper temporarily stops publishing map_off -> odom TF.
        """
        pose_msg = msg.pose.pose
        q = pose_msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._update_robot_off_pose({
            "x": float(pose_msg.position.x),
            "y": float(pose_msg.position.y),
            "yaw": float(yaw),
        })

    def _update_robot_off_pose(self, pose):
        with self.lock:
            if self.robot_off is None:
                self.path_off = [pose]
                self.path_off_base_version = 0
                self.path_off_version = 1
            else:
                dx = pose["x"] - self.robot_off["x"]
                dy = pose["y"] - self.robot_off["y"]
                if dx * dx + dy * dy >= 0.001 ** 2:
                    self.path_off.append(pose)
                    self.path_off_version += 1
                    if len(self.path_off) > 5000:
                        dropped = len(self.path_off) - 5000
                        self.path_off = self.path_off[-5000:]
                        self.path_off_base_version += dropped
            self.robot_off = pose

    def update_scan(self):
        now = time.monotonic()
        with self.lock:
            # Keep a separate startup/health timestamp even before the first
            # map exists.  Pre-map scans must not count as outage time.
            self.lidar_last_seen_at = now
            if self.lidar_monitor_started_at is None:
                return
            previous = (
                self.lidar_last_scan_at
                if self.lidar_last_scan_at is not None
                else self.lidar_monitor_started_at
            )
            gap = max(0.0, now - previous)
            if gap > LIDAR_GAP_THRESHOLD_S:
                self.lidar_completed_gap_s += gap
                self.lidar_max_gap_s = max(self.lidar_max_gap_s, gap)
                self.lidar_gap_events += 1
            self.lidar_last_scan_at = now

    def lidar_is_ready(self, timeout_s=1.0):
        with self.lock:
            return (
                self.lidar_last_seen_at is not None
                and time.monotonic() - self.lidar_last_seen_at <= timeout_s
            )

    def reset_for_new_map(self):
        """Clear phone-side map/pose state after slam_toolbox resets."""
        with self.lock:
            self.map = None
            self.map_json_body = None
            self.map_json_version = None
            self.map_off = None
            self.map_off_json_body = None
            self.map_off_json_version = None
            self.robot = None
            self.robot_off = None
            self.path = []
            self.path_off = []
            self.path_base_version = 0
            self.path_version = 0
            self.path_off_base_version = 0
            self.path_off_version = 0
            self.version = 0
            self.motor_rpm = {"left": 0.0, "right": 0.0}
            self.lidar_monitor_started_at = None
            self.lidar_last_seen_at = None
            self.lidar_last_scan_at = None
            self.lidar_completed_gap_s = 0.0
            self.lidar_max_gap_s = 0.0
            self.lidar_gap_events = 0

    def update_odom(self, msg):
        v = float(msg.twist.twist.linear.x)
        omega = float(msg.twist.twist.angular.z)
        v_left = v - omega * WHEEL_SEPARATION / 2.0
        v_right = v + omega * WHEEL_SEPARATION / 2.0
        meters_per_revolution = 2.0 * math.pi * WHEEL_RADIUS
        left_rpm = v_left / meters_per_revolution * 60.0
        right_rpm = v_right / meters_per_revolution * 60.0
        with self.lock:
            self.motor_rpm = {"left": left_rpm, "right": right_rpm}

    def _lidar_snapshot_locked(self, now):
        if self.lidar_monitor_started_at is None:
            return {
                "gap_seconds": 0.0,
                "current_gap_seconds": 0.0,
                "events": 0,
                "max_gap_seconds": 0.0,
                "healthy": True,
            }
        previous = (
            self.lidar_last_scan_at
            if self.lidar_last_scan_at is not None
            else self.lidar_monitor_started_at
        )
        current_gap = max(0.0, now - previous)
        active_gap = current_gap > LIDAR_GAP_THRESHOLD_S
        total_gap = self.lidar_completed_gap_s + (current_gap if active_gap else 0.0)
        return {
            "gap_seconds": total_gap,
            "current_gap_seconds": current_gap if active_gap else 0.0,
            "events": self.lidar_gap_events,
            "max_gap_seconds": self.lidar_max_gap_s,
            "healthy": not active_gap,
        }

    def lidar_snapshot(self):
        with self.lock:
            return self._lidar_snapshot_locked(time.monotonic())

    def map_body(self):
        with self.map_json_lock:
            with self.lock:
                if self.map is None:
                    return None
                if self.map_json_body is not None and self.map_json_version == self.version:
                    return self.map_json_body
            payload = self.display_map_payload(self.map)

            # Serialize only once per new SLAM map, in the HTTP worker rather
            # than inside the ROS callback/control loop.
            body = json.dumps(
                {"ok": True, **payload}, separators=(",", ":")
            ).encode("utf-8")
            with self.lock:
                if self.version == payload["version"]:
                    self.map_json_body = body
                    self.map_json_version = self.version
            return body

    def map_off_body(self):
        """Return the cached diagnostic OFF map JSON body."""
        with self.map_json_lock:
            with self.lock:
                if self.map_off is None:
                    return None
                version = self.map_off["version"]
                if (
                    self.map_off_json_body is not None
                    and self.map_off_json_version == version
                ):
                    return self.map_off_json_body
                record = dict(self.map_off)
            payload = self.display_map_payload(record)
            body = json.dumps(
                {"ok": True, **payload}, separators=(",", ":")
            ).encode("utf-8")
            with self.lock:
                if self.map_off is not None and self.map_off["version"] == version:
                    self.map_off_json_body = body
                    self.map_off_json_version = version
            return body

    @staticmethod
    def display_map_payload(map_record):
        width = map_record["width"]
        height = map_record["height"]
        source = map_record["data"]
        scale = max(
            1,
            math.ceil(max(width, height) / MAX_MAP_DISPLAY_DIM),
        )

        if scale == 1:
            return dict(map_record)

        display_width = math.ceil(width / scale)
        display_height = math.ceil(height / scale)
        display_data = []
        for display_y in range(display_height):
            source_y0 = display_y * scale
            source_y1 = min(height, source_y0 + scale)
            for display_x in range(display_width):
                source_x0 = display_x * scale
                source_x1 = min(width, source_x0 + scale)
                maximum_occupied = 0
                has_unknown = False
                for source_y in range(source_y0, source_y1):
                    row_start = source_y * width
                    for source_x in range(source_x0, source_x1):
                        value = int(source[row_start + source_x])
                        if value < 0:
                            has_unknown = True
                        elif value > maximum_occupied:
                            maximum_occupied = value

                if maximum_occupied >= 50:
                    display_data.append(maximum_occupied)
                elif has_unknown:
                    display_data.append(-1)
                else:
                    display_data.append(maximum_occupied)

        return {
            "width": display_width,
            "height": display_height,
            "resolution": map_record["resolution"] * scale,
            "origin_x": map_record["origin_x"],
            "origin_y": map_record["origin_y"],
            "data": display_data,
            "version": map_record["version"],
            "display_scale": scale,
        }

    def pose_snapshot(self, requested_path_version, requested_path_off_version=0):
        with self.lock:
            requested_path_version = max(0, int(requested_path_version))
            reset = requested_path_version < self.path_base_version
            if reset:
                points = list(self.path)
            else:
                start = requested_path_version - self.path_base_version
                points = list(self.path[max(0, start):])
            requested_path_off_version = max(0, int(requested_path_off_version))
            reset_off = requested_path_off_version < self.path_off_base_version
            if reset_off:
                points_off = list(self.path_off)
            else:
                start_off = requested_path_off_version - self.path_off_base_version
                points_off = list(self.path_off[max(0, start_off):])
            return {
                "ok": self.map is not None,
                "robot": self.robot,
                "robot_off": self.robot_off,
                "path_reset": reset,
                "path_base_version": self.path_base_version,
                "path_version": self.path_version,
                "path_points": points,
                "path_off_reset": reset_off,
                "path_off_base_version": self.path_off_base_version,
                "path_off_version": self.path_off_version,
                "path_points_off": points_off,
                "map_version": self.version,
                "lidar": self._lidar_snapshot_locked(time.monotonic()),
                "motor_rpm": dict(self.motor_rpm),
            }

    def has_map(self):
        with self.lock:
            return self.map is not None

    def version_number(self):
        with self.lock:
            return self.version

    def map_off_version_number(self):
        with self.lock:
            return self.map_off["version"] if self.map_off is not None else 0

    def has_map_off(self):
        with self.lock:
            return self.map_off is not None

    def map_snapshot_for_save(self):
        """Return a detached map snapshot for the background file writer."""
        with self.lock:
            if self.map is None:
                return None
            record = self.map
            data = list(record["data"])
            return {
                "width": record["width"],
                "height": record["height"],
                "resolution": record["resolution"],
                "origin_x": record["origin_x"],
                "origin_y": record["origin_y"],
                "data": data,
                "version": record["version"],
            }


class MapCheckpointSaver:
    """Persist the current OccupancyGrid without blocking ROS callbacks."""

    def __init__(self, map_state):
        self.map_state = map_state
        self.save_dir = MAP_SAVE_DIR
        self.stage_dir = os.path.join(self.save_dir, ".checkpoint_staging")
        os.makedirs(self.stage_dir, exist_ok=True)
        self.stop_event = threading.Event()
        self.save_lock = threading.Lock()
        self.thread = None
        self.last_saved_at = None
        self.last_error = None

    def start(self):
        self.thread = threading.Thread(
            target=self._run,
            name="map_checkpoint_saver",
            daemon=True,
        )
        self.thread.start()

    def _run(self):
        next_save = time.monotonic()
        while not self.stop_event.is_set():
            if self.map_state.has_map():
                self.save_now()
            next_save += MAP_CHECKPOINT_INTERVAL_S
            wait_s = max(0.0, next_save - time.monotonic())
            if self.stop_event.wait(wait_s):
                break

    def save_now(self):
        with PHONE_DIAGNOSTICS.measure("map_checkpoint_save"):
            return self._save_now()

    def _save_now(self):
        """Save one complete checkpoint; return true only on success."""
        with self.save_lock:
            snapshot = self.map_state.map_snapshot_for_save()
            if snapshot is None:
                return False

            stage_base = os.path.join(self.stage_dir, "recovery_map")
            stage_yaml = f"{stage_base}.yaml"
            stage_pgm = f"{stage_base}.pgm"
            final_base = os.path.join(self.save_dir, "recovery_map")
            final_yaml = f"{final_base}.yaml"
            final_pgm = f"{final_base}.pgm"

            try:
                width = int(snapshot["width"])
                height = int(snapshot["height"])
                data = snapshot["data"]
                if width <= 0 or height <= 0 or len(data) != width * height:
                    raise RuntimeError("cached map dimensions are invalid")

                with open(stage_pgm, "wb") as image_file:
                    image_file.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
                    # OccupancyGrid row zero is the bottom row; PGM row zero
                    # is the top row.  Write rows in reverse Y order.
                    for y in range(height - 1, -1, -1):
                        row = data[y * width : (y + 1) * width]
                        pixels = bytes(
                            205 if int(value) < 0
                            else 0 if int(value) >= 65
                            else 254 if int(value) <= 25
                            else 205
                            for value in row
                        )
                        image_file.write(pixels)

                with open(stage_yaml, "w", encoding="ascii") as metadata_file:
                    metadata_file.write(
                        "image: recovery_map.pgm\n"
                        "mode: trinary\n"
                        f"resolution: {float(snapshot['resolution']):.3f}\n"
                        f"origin: [{float(snapshot['origin_x']):.6f}, "
                        f"{float(snapshot['origin_y']):.6f}, 0]\n"
                        "negate: 0\n"
                        "occupied_thresh: 0.65\n"
                        "free_thresh: 0.196\n"
                    )

                # The staged YAML refers to recovery_map.pgm, so the pair is
                # self-consistent after both files are moved into place.
                os.replace(stage_pgm, final_pgm)
                os.replace(stage_yaml, final_yaml)
                self.last_saved_at = time.time()
                self.last_error = None
                print(
                    "Map checkpoint saved: ~/robot1_maps/recovery_map.yaml "
                    f"(version {snapshot['version']})"
                )
                return True
            except (OSError, RuntimeError, ValueError) as exc:
                self.last_error = str(exc)
                print(f"Map checkpoint warning: {exc}")
                return False

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=MAP_SAVE_TIMEOUT_S + 2.0)
            self.thread = None


class OdomDriftSession:
    """Lightweight in-memory capture and asynchronous endpoint analysis."""

    CSV_FIELDS = [
        "phase",
        "test_mode",
        "timestamp_ros_s",
        "timestamp_monotonic_s",
        "x",
        "y",
        "yaw",
        "linear_x",
        "angular_z",
        "command_key",
        "slider_rpm",
        "linear_command_mps",
        "angular_command_radps",
        "requested_linear_command_mps",
        "requested_angular_command_radps",
        "command_change_monotonic_s",
    ]

    COMMAND_FIELDS = [
        "timestamp_monotonic_s",
        "key",
        "slider_rpm",
        "requested_linear_command_mps",
        "requested_angular_command_radps",
    ]

    def __init__(self, command_source, comparison=None):
        self.command_source = command_source
        self.comparison = comparison
        self.condition = threading.Condition()
        self.state = "WAITING_FOR_ODOM"
        self.message = "Waiting for stationary odometry reference"
        self.mode = "CLOSED_LOOP"
        self.mode_configured = False
        self.spin_direction = "CW"
        self.expected_turns = 5
        self.initial_samples = []
        self.start_reference = []
        self.trajectory = []
        self.final_reference = []
        self.result = None
        self.result_dir = None
        self.analysis_thread = None
        self.session_start_monotonic_s = None
        self.reference_ready_monotonic_s = None

    @staticmethod
    def _row_from_msg(msg, command_state):
        pose = msg.pose.pose
        orientation = pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        stamp = msg.header.stamp
        ros_time_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        values = {
            "timestamp_ros_s": ros_time_s,
            "timestamp_monotonic_s": time.monotonic(),
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": float(yaw),
            "linear_x": float(msg.twist.twist.linear.x),
            "angular_z": float(msg.twist.twist.angular.z),
            "command_key": str(command_state.get("key", "x")),
            "slider_rpm": float(command_state.get("slider_rpm", 0.0)),
            "linear_command_mps": float(command_state.get("linear_command_mps", 0.0)),
            "angular_command_radps": float(command_state.get("angular_command_radps", 0.0)),
            "requested_linear_command_mps": float(
                command_state.get("requested_linear_command_mps", 0.0)
            ),
            "requested_angular_command_radps": float(
                command_state.get("requested_angular_command_radps", 0.0)
            ),
            "command_change_monotonic_s": float(
                command_state.get("command_change_monotonic_s", 0.0)
            ),
        }
        numeric_values = [value for key, value in values.items() if key != "command_key"]
        if not all(math.isfinite(value) for value in numeric_values):
            return None
        return values

    def record(self, msg):
        row = self._row_from_msg(msg, self.command_source.command_state())
        if row is None:
            return
        stationary = (
            abs(row["linear_x"]) <= ODOM_MIN_STATIONARY_LINEAR_MPS
            and abs(row["angular_z"]) <= ODOM_MIN_STATIONARY_ANGULAR_RADPS
        )
        with self.condition:
            if self.state == "WAITING_FOR_ODOM":
                if not stationary:
                    self.initial_samples.clear()
                    self.start_reference = []
                    self.reference_ready_monotonic_s = None
                    self.session_start_monotonic_s = None
                    self.message = "Waiting for stationary odometry reference"
                    return
                self.initial_samples.append(row)
                if len(self.initial_samples) >= ODOM_START_REFERENCE_SAMPLES:
                    if self.reference_ready_monotonic_s is None:
                        self.start_reference = [
                            dict(sample, phase="start_reference", test_mode=self.mode)
                            for sample in self.initial_samples[-ODOM_START_REFERENCE_SAMPLES:]
                        ]
                        self.session_start_monotonic_s = self.start_reference[0][
                            "timestamp_monotonic_s"
                        ]
                        self.reference_ready_monotonic_s = time.monotonic()
                        self.message = (
                            "Select odometry test mode; recording starts shortly"
                            if self.mode_configured
                            else "Select odometry test mode"
                        )
                    if not self.mode_configured:
                        self.message = "Select odometry test mode"
                        return
                    if (
                        time.monotonic() - self.reference_ready_monotonic_s
                        >= ODOM_MODE_CONFIGURATION_GRACE_S
                    ):
                        self.trajectory = [
                            dict(row, phase="trajectory", test_mode=self.mode)
                        ]
                        self.state = "RECORDING"
                        self.message = f"Recording {self.mode}"
                return

            if self.state == "RECORDING":
                self.trajectory.append(dict(row, phase="trajectory", test_mode=self.mode))
                return

            if self.state == "FINALIZING":
                self.final_reference.append(
                    dict(row, phase="final_reference", test_mode=self.mode)
                )
                self.condition.notify_all()

    def configure(self, mode=None, direction=None, expected_turns=None):
        prepare_comparison = False
        cancel_comparison = False
        with self.condition:
            if self.state != "WAITING_FOR_ODOM":
                return False, "Press RESET before changing odometry test mode"
            if mode is not None:
                mode = str(mode).upper()
                if mode not in {"CLOSED_LOOP", "STRAIGHT_STOP", "SPIN"}:
                    return False, f"Unknown odometry test mode: {mode}"
                self.mode = mode
                self.mode_configured = True
                prepare_comparison = mode == "CLOSED_LOOP"
                cancel_comparison = mode != "CLOSED_LOOP"
            if direction is not None:
                direction = str(direction).upper()
                if direction not in {"CW", "CCW"}:
                    return False, f"Unknown spin direction: {direction}"
                self.spin_direction = direction
            if expected_turns is not None:
                try:
                    expected_turns = int(expected_turns)
                except (TypeError, ValueError):
                    return False, "Expected spin turns must be an integer from 1 to 10"
                if not 1 <= expected_turns <= 10:
                    return False, "Expected spin turns must be from 1 to 10"
                self.expected_turns = expected_turns
            self.message = "Waiting for stationary odometry reference"
        if cancel_comparison and self.comparison is not None:
            self.comparison.cancel()
        if prepare_comparison and self.comparison is not None:
            if not self.comparison.prepare():
                return False, "Could not start the raw SLAM comparison recording"
        return True, "Odometry test configuration updated"

    def status(self):
        with self.condition:
            value = {
                "state": self.state,
                "message": self.message,
                "mode": self.mode,
                "mode_configured": self.mode_configured,
                "spin_direction": self.spin_direction,
                "expected_turns": self.expected_turns,
                "reference_ready": self.reference_ready_monotonic_s is not None,
                "sample_count": len(self.trajectory),
                "final_sample_count": len(self.final_reference),
                "result_dir": self.result_dir,
                "result": self.result,
            }
            if self.comparison is not None:
                value["comparison"] = self.comparison.status()
            return value

    def reset(self):
        with self.condition:
            if self.state == "FINALIZING":
                return False, "Odometry analysis is still finalizing"
            if self.comparison is not None and self.comparison.status()["state"] == "PROCESSING":
                return False, "SLAM scan-matching comparison is still processing"
            # RESET starts a clean session, but it should not silently discard
            # the mode the user just selected.  The phone workflow is
            # select-mode -> RESET -> wait -> drive; preserving the selection
            # lets the freshly reloaded page proceed into RECORDING after a
            # new stationary reference is collected.
            selected_mode = self.mode if self.mode_configured else None
            selected_direction = self.spin_direction
            selected_turns = self.expected_turns
            self.state = "WAITING_FOR_ODOM"
            self.message = "Waiting for stationary odometry reference"
            self.initial_samples = []
            self.start_reference = []
            self.trajectory = []
            self.final_reference = []
            self.result = None
            self.result_dir = None
            self.session_start_monotonic_s = None
            self.reference_ready_monotonic_s = None
            self.mode = selected_mode or "CLOSED_LOOP"
            self.mode_configured = selected_mode is not None
            self.spin_direction = selected_direction
            self.expected_turns = selected_turns
        if self.comparison is not None:
            self.comparison.cancel()
            if selected_mode == "CLOSED_LOOP" and not self.comparison.prepare():
                with self.condition:
                    self.mode_configured = False
                    self.message = "Could not start the raw SLAM comparison recording"
                return False, self.message
        return True, "Odometry test reset"

    def request_finish(self, allow_stationary=False):
        with self.condition:
            if self.state in {"FINALIZING", "COMPLETE"}:
                return self.status()
            if self.state != "RECORDING":
                self.message = "No active odometry recording exists"
                self.state = "ERROR"
                return self.status()
            self.state = "FINALIZING"
            self.message = "Waiting for final stationary odometry samples"
            self.analysis_thread = threading.Thread(
                target=self._finalize_worker,
                args=(bool(allow_stationary),),
                name="odom_drift_analysis",
                daemon=True,
            )
            self.analysis_thread.start()
            return self.status()

    def _write_trajectory(self, path, rows):
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.CSV_FIELDS)
            writer.writeheader()
            for row in rows:
                writer.writerow({field: row[field] for field in self.CSV_FIELDS})

    def _write_command_history(self, path, events):
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.COMMAND_FIELDS)
            writer.writeheader()
            for event in events:
                writer.writerow({field: event.get(field, "") for field in self.COMMAND_FIELDS})

    def _finalize_worker(self, allow_stationary=False):
        deadline = time.monotonic() + ODOM_FINALIZE_TIMEOUT_S
        with self.condition:
            while (
                len(self.final_reference) < ODOM_FINAL_REFERENCE_SAMPLES
                and time.monotonic() < deadline
            ):
                self.condition.wait(timeout=max(0.01, deadline - time.monotonic()))

            if len(self.final_reference) < ODOM_FINAL_REFERENCE_SAMPLES:
                self.state = "ERROR"
                self.message = (
                    "Insufficient final odometry samples: "
                    f"{len(self.final_reference)} / {ODOM_FINAL_REFERENCE_SAMPLES}"
                )
                return

            rows = list(self.start_reference) + list(self.trajectory) + list(self.final_reference)
            mode = self.mode
            spin_direction = self.spin_direction
            expected_turns = self.expected_turns
            session_start = self.session_start_monotonic_s or rows[0][
                "timestamp_monotonic_s"
            ]
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            result_dir = os.path.join(ODOM_RESULTS_DIR, f"odom_drift_test_{stamp}")
            suffix = 1
            while os.path.exists(result_dir):
                result_dir = os.path.join(
                    ODOM_RESULTS_DIR, f"odom_drift_test_{stamp}_{suffix}"
                )
                suffix += 1
            os.makedirs(result_dir, exist_ok=False)
            self.result_dir = result_dir

        trajectory_path = os.path.join(result_dir, "odom_trajectory.csv")
        self._write_trajectory(trajectory_path, rows)
        command_path = os.path.join(result_dir, "command_trajectory.csv")
        self._write_command_history(
            command_path,
            self.command_source.history_since(session_start),
        )
        command = [
            sys.executable,
            ODOM_ANALYZER,
            "--input",
            trajectory_path,
            "--output-dir",
            result_dir,
            "--mode",
            mode,
            "--wheel-separation-m",
            str(WHEEL_SEPARATION),
        ]
        if mode == "SPIN":
            command.extend([
                "--direction",
                spin_direction,
                "--expected-turns",
                str(expected_turns),
            ])
        if allow_stationary:
            command.append("--allow-stationary")
        result_path = os.path.join(result_dir, "odom_drift_result.json")
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=20.0,
                check=False,
            )
            with open(os.path.join(result_dir, "analyzer.log"), "w", encoding="utf-8") as log:
                log.write(completed.stdout or "")
                log.write(completed.stderr or "")
            with open(result_path, encoding="utf-8") as stream:
                result = json.load(stream)
        except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
            result = {"status": "error", "error": f"analysis launch failed: {exc}"}

        if mode == "CLOSED_LOOP" and self.comparison is not None:
            self.comparison.finish_async(result_path)

        with self.condition:
            self.result = result
            if result.get("status") == "ok":
                self.state = "COMPLETE"
                self.message = "Odometry drift analysis complete"
            else:
                self.state = "ERROR"
                self.message = str(result.get("error", "Odometry analysis failed"))
            self.condition.notify_all()

    def shutdown(self):
        with self.condition:
            if self.state == "RECORDING":
                self.state = "ERROR"
                self.message = "Phone controller stopped before FINISHED was pressed"
            self.condition.notify_all()
        if self.comparison is not None:
            self.comparison.shutdown()
