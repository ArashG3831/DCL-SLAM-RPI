"""Bounded forensic evidence writer for one cooperative exploration run.

This module is deliberately passive.  It serializes selected ROS messages and
TF lookups for offline comparison; it never publishes, calls a service, or
changes robot state.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import threading
from pathlib import Path

import numpy as np


def _stamp_value(stamp):
    return int(stamp.sec) + int(stamp.nanosec) * 1e-9


def _stamp_text(stamp):
    return f"{int(stamp.sec)}.{int(stamp.nanosec):09d}"


def _quat_dict(q):
    return {"x": float(q.x), "y": float(q.y), "z": float(q.z), "w": float(q.w)}


def _origin_dict(origin):
    return {
        "position": {
            "x": float(origin.position.x),
            "y": float(origin.position.y),
            "z": float(origin.position.z),
        },
        "orientation": _quat_dict(origin.orientation),
    }


def _grid_metadata(message, topic, robot, received_ros, received_wall):
    info = message.info
    return {
        "schema_version": "forensic_evidence_1.0",
        "topic": topic,
        "robot_id": robot,
        "frame_id": message.header.frame_id,
        "header_stamp": _stamp_text(message.header.stamp),
        "header_stamp_s": _stamp_value(message.header.stamp),
        "map_load_time": _stamp_text(info.map_load_time),
        "received_ros_time_s": float(received_ros),
        "received_wall_elapsed_s": float(received_wall),
        "width": int(info.width),
        "height": int(info.height),
        "resolution": float(info.resolution),
        "origin": _origin_dict(info.origin),
        "dtype": "int8",
        "data_length": len(message.data),
    }


def _atomic_npz(path, occupancy, metadata):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    with temporary.open("wb") as stream:
        np.savez_compressed(
            stream,
            occupancy=np.asarray(occupancy, dtype=np.int8),
            metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
        )
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _safe_name(value):
    return str(value).replace("/", "_").replace(" ", "_")


class ForensicEvidenceWriter:
    """Stream bounded maps, PeerMap records, raw odometry and TF evidence."""

    MAP_KEYS = ("map", "shared_map")
    HIGH_RATE_CSV_BATCH_SIZE = 128

    def __init__(self, directory, robots, interval_s=15.0,
                 scan_matching_enabled=False):
        self.root = Path(directory) / "forensic"
        self.maps_dir = self.root / "maps"
        self.peer_dir = self.root / "peer_maps"
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self.peer_dir.mkdir(parents=True, exist_ok=True)
        self.robots = tuple(robots)
        self.interval_s = max(5.0, float(interval_s))
        self.last_snapshot_ros = None
        self.last_map_digest = {}
        self.last_peer_revision = {robot: -1 for robot in self.robots}
        self.map_snapshot_count = 0
        self.peer_record_count = 0
        self._closed = False
        self._io_lock = threading.RLock()
        self._odom_files = {}
        self._odom_writers = {}
        odom_fields = [
            "robot_id", "received_ros_time_s", "received_wall_elapsed_s",
            "header_stamp", "frame_id", "pose_x", "pose_y", "pose_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w",
            "twist_linear_x", "twist_linear_y", "twist_linear_z",
            "twist_angular_x", "twist_angular_y", "twist_angular_z",
        ]
        for robot in self.robots:
            stream = (self.root / f"{robot}_odom.csv").open(
                "w", newline="", encoding="utf-8")
            writer = csv.writer(stream)
            writer.writerow(odom_fields)
            self._odom_files[robot] = stream
            self._odom_writers[robot] = writer
        self._odom_row_buffers = {robot: [] for robot in self.robots}
        self.peer_file = (self.root / "peer_map_records.csv").open(
            "w", newline="", encoding="utf-8")
        self.peer_writer = csv.DictWriter(self.peer_file, fieldnames=[
            "robot_id", "source_robot_id", "revision", "export_stamp",
            "occupancy_header_stamp", "local_evidence_only", "accepted",
            "received_ros_time_s", "received_wall_elapsed_s", "npz_path",
            "width", "height", "resolution", "frame_id",
        ])
        self.peer_writer.writeheader()
        self.tf_file = (self.root / "transforms.csv").open(
            "w", newline="", encoding="utf-8")
        self.tf_writer = csv.DictWriter(self.tf_file, fieldnames=[
            "query_ros_time_s", "query_wall_elapsed_s", "target_frame",
            "source_frame", "transform_stamp", "transform_age_s", "available",
            "translation_x", "translation_y", "translation_z",
            "rotation_x", "rotation_y", "rotation_z", "rotation_w", "error",
        ])
        self.tf_writer.writeheader()
        self.raw_tf_file = (self.root / "raw_tf.csv").open(
            "w", newline="", encoding="utf-8")
        raw_tf_fields = [
            "topic", "static", "received_ros_time_s", "received_wall_elapsed_s",
            "transform_stamp", "parent_frame", "child_frame",
            "translation_x", "translation_y", "translation_z",
            "rotation_x", "rotation_y", "rotation_z", "rotation_w",
        ]
        self.raw_tf_writer = csv.writer(self.raw_tf_file)
        self.raw_tf_writer.writerow(raw_tf_fields)
        self._raw_tf_row_buffer = []
        self.synchronized_file = (
            self.root / "synchronized_map_frame.jsonl").open(
                "w", encoding="utf-8", buffering=1)
        self.scan_matching_enabled = bool(scan_matching_enabled)
        self._scan_files = {}
        self._scan_writers = {}
        self._previous_map_to_odom = {}
        if self.scan_matching_enabled:
            self.scan_dir = self.root / "scan_matching"
            self.scan_dir.mkdir(parents=True, exist_ok=True)
            scan_fields = [
                "robot_id", "query_ros_time_s", "query_wall_elapsed_s",
                "odom_header_stamp", "map_to_odom_stamp", "source",
                "available", "accepted", "rejected", "rejection_reason",
                "response_expansion", "odom_predicted_x", "odom_predicted_y",
                "odom_predicted_yaw", "accepted_slam_x", "accepted_slam_y",
                "accepted_slam_yaw", "map_to_odom_x", "map_to_odom_y",
                "map_to_odom_yaw", "translation_correction_m",
                "yaw_correction_rad", "translation_bound_m",
                "yaw_bound_rad", "bound_violation", "correction_basis",
                "sample_kind", "map_to_odom_delta_x",
                "map_to_odom_delta_y", "map_to_odom_delta_yaw", "error",
            ]
            for robot in self.robots:
                stream = (self.scan_dir / f"{robot}_corrections.jsonl").open(
                    "w", encoding="utf-8", buffering=1)
                self._scan_files[robot] = stream
                self._scan_writers[robot] = scan_fields
        self._flush_counter = 0

    @staticmethod
    def _message_digest(message):
        info = message.info
        payload = np.asarray(message.data, dtype=np.int8).tobytes()
        digest = hashlib.sha256()
        digest.update(message.header.frame_id.encode('utf-8'))
        digest.update(str((int(info.width), int(info.height),
                           float(info.resolution),
                           float(info.origin.position.x),
                           float(info.origin.position.y),
                           float(info.origin.orientation.z),
                           float(info.origin.orientation.w))).encode('ascii'))
        digest.update(payload)
        return digest.hexdigest()

    def _map_path(self, robot, key, stamp, final=False):
        stem = f"{_safe_name(robot)}_{_safe_name(key)}"
        if final:
            return self.maps_dir / f"{stem}_final.npz"
        return self.maps_dir / (
            f"sim_{stamp.sec}_{stamp.nanosec:09d}_{stem}.npz")

    def save_map(self, robot, key, message, received_ros, received_wall,
                 final=False, force=False):
        if self._closed or message is None:
            return None
        digest = self._message_digest(message)
        cache_key = (robot, key)
        if not force and self.last_map_digest.get(cache_key) == digest:
            return None
        path = self._map_path(robot, key, message.header.stamp, final=final)
        metadata = _grid_metadata(
            message, f"/{robot}/{key}", robot, received_ros, received_wall)
        metadata.update({"snapshot_kind": "final" if final else "periodic",
                         "content_hash": digest})
        data = np.asarray(message.data, dtype=np.int8).reshape(
            int(message.info.height), int(message.info.width))
        _atomic_npz(path, data, metadata)
        self.last_map_digest[cache_key] = digest
        self.map_snapshot_count += 1
        return str(path)

    def capture_maps(self, latest, received_ros, received_wall, force=False):
        if self._closed:
            return []
        if not force and self.last_snapshot_ros is not None and (
                received_ros - self.last_snapshot_ros < self.interval_s):
            return []
        paths = []
        for robot in self.robots:
            for key in self.MAP_KEYS:
                message = latest.get(robot, {}).get(key)
                path = self.save_map(robot, key, message, received_ros,
                                     received_wall, force=force)
                if path:
                    paths.append(path)
        self.last_snapshot_ros = received_ros
        return paths

    def save_final_maps(self, latest, received_ros, received_wall):
        paths = []
        for robot in self.robots:
            for key in self.MAP_KEYS:
                message = latest.get(robot, {}).get(key)
                path = self.save_map(robot, key, message, received_ros,
                                     received_wall, final=True, force=True)
                if path:
                    paths.append(path)
        return paths

    def record_peer_map(self, robot, message, received_ros, received_wall):
        if self._closed or message is None:
            return False
        source = str(message.source_robot_id)
        revision = int(message.revision)
        accepted = (source == robot and bool(message.local_evidence_only)
                    and revision > self.last_peer_revision[robot])
        path = ""
        grid = message.occupancy_grid
        if accepted:
            self.last_peer_revision[robot] = revision
            path_obj = self.peer_dir / f"{robot}_revision_{revision:06d}.npz"
            metadata = _grid_metadata(
                grid, f"/cslam/{robot}/local_map", robot, received_ros,
                received_wall)
            metadata.update({
                "source_robot_id": source,
                "revision": revision,
                "export_stamp": _stamp_text(message.export_stamp),
                "local_evidence_only": bool(message.local_evidence_only),
                "accepted": True,
            })
            data = np.asarray(grid.data, dtype=np.int8).reshape(
                int(grid.info.height), int(grid.info.width))
            _atomic_npz(path_obj, data, metadata)
            path = str(path_obj)
            self.peer_record_count += 1
        self.peer_writer.writerow({
            "robot_id": robot, "source_robot_id": source, "revision": revision,
            "export_stamp": _stamp_text(message.export_stamp),
            "occupancy_header_stamp": _stamp_text(grid.header.stamp),
            "local_evidence_only": bool(message.local_evidence_only),
            "accepted": accepted, "received_ros_time_s": received_ros,
            "received_wall_elapsed_s": received_wall, "npz_path": path,
            "width": int(grid.info.width), "height": int(grid.info.height),
            "resolution": float(grid.info.resolution),
            "frame_id": grid.header.frame_id,
        })
        return accepted

    def record_odom(self, robot, message, received_ros, received_wall):
        if self._closed:
            return
        pose = message.pose.pose
        twist = message.twist.twist
        rows = self._odom_row_buffers[robot]
        rows.append((
            robot, received_ros, received_wall,
            _stamp_text(message.header.stamp), message.header.frame_id,
            pose.position.x, pose.position.y, pose.position.z,
            message.pose.pose.orientation.x, message.pose.pose.orientation.y,
            message.pose.pose.orientation.z, message.pose.pose.orientation.w,
            twist.linear.x, twist.linear.y, twist.linear.z,
            twist.angular.x, twist.angular.y, twist.angular.z,
        ))
        if len(rows) >= self.HIGH_RATE_CSV_BATCH_SIZE:
            self._odom_writers[robot].writerows(rows)
            rows.clear()

    def record_transform(self, query_ros, query_wall, target, source,
                         transform=None, error=""):
        row = {"query_ros_time_s": query_ros, "query_wall_elapsed_s": query_wall,
               "target_frame": target, "source_frame": source,
               "transform_stamp": "", "transform_age_s": None,
               "available": transform is not None,
               "translation_x": None, "translation_y": None,
               "translation_z": None, "rotation_x": None, "rotation_y": None,
               "rotation_z": None, "rotation_w": None, "error": error}
        if transform is not None:
            stamp = transform.header.stamp
            t = transform.transform.translation
            q = transform.transform.rotation
            row.update({"transform_stamp": _stamp_text(stamp),
                        "transform_age_s": query_ros - _stamp_value(stamp),
                        "translation_x": t.x, "translation_y": t.y,
                        "translation_z": t.z, "rotation_x": q.x,
                        "rotation_y": q.y, "rotation_z": q.z, "rotation_w": q.w})
        self.tf_writer.writerow(row)

    def record_raw_tf(self, topic, message, received_ros, received_wall,
                      static=False):
        """Persist raw TF edges for post-run map-frame evaluation."""
        if self._closed or message is None:
            return
        rows = self._raw_tf_row_buffer
        for item in getattr(message, "transforms", ()):
            stamp = item.header.stamp
            t = item.transform.translation
            q = item.transform.rotation
            rows.append((
                str(topic), bool(static), float(received_ros),
                float(received_wall), _stamp_text(stamp),
                str(item.header.frame_id), str(item.child_frame_id),
                float(t.x), float(t.y), float(t.z), float(q.x), float(q.y),
                float(q.z), float(q.w),
            ))
        if len(rows) >= self.HIGH_RATE_CSV_BATCH_SIZE:
            self.raw_tf_writer.writerows(rows)
            rows.clear()

    def record_synchronized_map_frame(self, row):
        """Write one passive, timestamped map-frame synchronization sample.

        The row intentionally contains observations only.  It is never
        published or read by the estimator, selector, navigation stack, or
        handoff protocol.  Supervisor world poses are joined offline using
        ``query_ros_time_s``; ROS TF samples carry their own stamp and age so
        any interpolation or temporal mismatch is explicit in the artifact.
        """
        if self._closed:
            return
        with self._io_lock:
            self.synchronized_file.write(
                json.dumps(row, sort_keys=True, allow_nan=False) + "\n")

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z +
                   quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y +
                         quaternion.z * quaternion.z))

    def record_scan_correction(self, robot, query_ros, query_wall,
                               odom_message, map_to_odom=None, error="",
                               translation_bound_m=0.06,
                               yaw_bound_rad=0.0558503168):
        """Persist a passive local-SLAM correction observation.

        Slam Toolbox does not expose a correction callback in this deployment.
        Therefore successive local ``map -> odom`` TF samples at local-map
        update callbacks are used as a bounded correction observation.  The
        correction is the delta between consecutive map-frame estimates, not
        the absolute accumulated map-to-odom offset.  The accepted pose is composed as
        ``T_map_base = T_map_odom * T_odom_base``; Supervisor data is not
        involved.  The first available sample establishes a baseline and is
        not assigned a correction magnitude.  A missing TF is recorded as
        unavailable, never as a zero correction or a rejection.
        """
        if not self.scan_matching_enabled or self._closed:
            return
        pose = odom_message.pose.pose if odom_message is not None else None
        odom_yaw = self._yaw(pose.orientation) if pose is not None else None
        row = {
            "robot_id": robot, "query_ros_time_s": query_ros,
            "query_wall_elapsed_s": query_wall, "odom_header_stamp": "",
            "map_to_odom_stamp": "", "source": "local_map_to_odom_tf",
            "available": map_to_odom is not None,
            "accepted": False,
            "rejected": False,
            "rejection_reason": "",
            "response_expansion": False,
            "odom_predicted_x": pose.position.x if pose else None,
            "odom_predicted_y": pose.position.y if pose else None,
            "odom_predicted_yaw": odom_yaw,
            "accepted_slam_x": None, "accepted_slam_y": None,
            "accepted_slam_yaw": None, "map_to_odom_x": None,
            "map_to_odom_y": None, "map_to_odom_yaw": None,
            "translation_correction_m": None, "yaw_correction_rad": None,
            "translation_bound_m": translation_bound_m,
            "yaw_bound_rad": yaw_bound_rad, "bound_violation": False,
            "correction_basis": "delta_map_to_odom_between_local_map_updates",
            "sample_kind": "local_map_update",
            "map_to_odom_delta_x": None, "map_to_odom_delta_y": None,
            "map_to_odom_delta_yaw": None,
            "error": error,
        }
        if odom_message is not None:
            row["odom_header_stamp"] = _stamp_text(odom_message.header.stamp)
        if map_to_odom is not None and pose is not None:
            stamp = map_to_odom.header.stamp
            transform = map_to_odom.transform
            tx, ty = transform.translation.x, transform.translation.y
            map_yaw = self._yaw(transform.rotation)
            c, s = math.cos(map_yaw), math.sin(map_yaw)
            row.update({
                "map_to_odom_stamp": _stamp_text(stamp),
                "accepted_slam_x": tx + c * pose.position.x - s * pose.position.y,
                "accepted_slam_y": ty + s * pose.position.x + c * pose.position.y,
                "accepted_slam_yaw": map_yaw + odom_yaw,
                "map_to_odom_x": tx, "map_to_odom_y": ty,
                "map_to_odom_yaw": map_yaw,
            })
            previous = self._previous_map_to_odom.get(robot)
            self._previous_map_to_odom[robot] = (tx, ty, map_yaw)
            if previous is not None:
                delta_x, delta_y = tx - previous[0], ty - previous[1]
                delta_yaw = (map_yaw - previous[2] + math.pi) % (2.0 * math.pi) - math.pi
                translation = math.hypot(delta_x, delta_y)
                row.update({
                    "accepted": True,
                    "translation_correction_m": translation,
                    "yaw_correction_rad": delta_yaw,
                    "map_to_odom_delta_x": delta_x,
                    "map_to_odom_delta_y": delta_y,
                    "map_to_odom_delta_yaw": delta_yaw,
                    "bound_violation": (
                        translation > translation_bound_m or
                        abs(delta_yaw) > yaw_bound_rad),
                })
            else:
                row["rejection_reason"] = "NO_PREVIOUS_MAP_TO_ODOM_BASELINE"
        self._scan_files[robot].write(
            json.dumps(row, sort_keys=True, allow_nan=False) + "\n")

    def flush(self):
        if self._closed:
            return
        for robot, rows in self._odom_row_buffers.items():
            if rows:
                self._odom_writers[robot].writerows(rows)
                rows.clear()
        if self._raw_tf_row_buffer:
            self.raw_tf_writer.writerows(self._raw_tf_row_buffer)
            self._raw_tf_row_buffer.clear()
        streams = [*self._odom_files.values(), self.peer_file, self.tf_file,
                   self.raw_tf_file, self.synchronized_file,
                   *self._scan_files.values()]
        for stream in streams:
            stream.flush()
        self._flush_counter += 1

    def close(self):
        if self._closed:
            return
        self.flush()
        for stream in [*self._odom_files.values(), self.peer_file, self.tf_file,
                       self.raw_tf_file, self.synchronized_file,
                       *self._scan_files.values()]:
            stream.close()
        self._closed = True

    def manifest(self):
        return {
            "schema_version": "forensic_evidence_1.0",
            "root": str(self.root),
            "map_snapshot_interval_s": self.interval_s,
            "map_snapshot_count": self.map_snapshot_count,
            "accepted_peer_map_count": self.peer_record_count,
            "scan_matching_enabled": self.scan_matching_enabled,
            "files": sorted(str(path.relative_to(self.root))
                             for path in self.root.rglob("*") if path.is_file()),
        }
