#!/usr/bin/env python3
"""Controlled one-run Slam Toolbox scan-matching A/B experiment.

The manager owns one raw /scan + /odom recording and two isolated, sequential
replays.  It never records or replays /cmd_vel and never changes production
motor, lidar, or SLAM files.
"""

import csv
import json
import math
import os
import signal
import shlex
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone

import yaml

from fixed_reference_lidar import analyze_fixed_reference, _profile
from slam_map_renderer import common_canvas, load_map, render_map, render_side_by_side


ROOT = os.path.expanduser(os.environ.get(
    "ROBOT1_SLAM_MATCHING_RESULTS",
    "~/robot1_slam_matching_comparison_results",
))
ACTIVE_PARAMS = os.path.expanduser(os.environ.get(
    "ROBOT1_SLAM_ACTIVE_PARAMS",
    "~/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml",
))
REPLAY_DOMAIN = int(os.environ.get("ROBOT1_SLAM_REPLAY_DOMAIN", "117"))

CONSERVATIVE_PARAMS = {
    "correlation_search_space_dimension": 0.12,
    "correlation_search_space_resolution": 0.01,
    "correlation_search_space_smear_deviation": 0.015,
    "distance_variance_penalty": 0.05,
    "angle_variance_penalty": math.radians(3.0),
    "minimum_distance_penalty": 0.15,
    "minimum_angle_penalty": 0.70,
    "coarse_search_angle_offset": math.radians(3.0),
    "coarse_angle_resolution": math.radians(1.0),
    # In this Karto version the parameter is the fine angular sampling step;
    # the fine range is half coarse_angle_resolution.
    "fine_search_angle_offset": math.radians(0.2),
    "use_response_expansion": False,
}


def _wrap(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _relative_pose(rows):
    if not rows:
        return None
    start = rows[0]
    end = rows[-1]
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    yaw = start["yaw"]
    return {
        "x_m": dx,
        "y_m": dy,
        "forward_m": math.cos(yaw) * dx + math.sin(yaw) * dy,
        "lateral_m": -math.sin(yaw) * dx + math.cos(yaw) * dy,
        "position_m": math.hypot(dx, dy),
        "yaw_rad": _wrap(end["yaw"] - start["yaw"]),
        "yaw_deg": math.degrees(_wrap(end["yaw"] - start["yaw"])),
    }


def _pose_compose(first, second):
    """Compose two planar poses, returning first * second."""
    c = math.cos(first["yaw"])
    s = math.sin(first["yaw"])
    return {
        "x": first["x"] + c * second["x"] - s * second["y"],
        "y": first["y"] + s * second["x"] + c * second["y"],
        "yaw": _wrap(first["yaw"] + second["yaw"]),
    }


def _pose_inverse(pose):
    """Return the inverse of a planar pose."""
    c = math.cos(pose["yaw"])
    s = math.sin(pose["yaw"])
    return {
        "x": -c * pose["x"] - s * pose["y"],
        "y": s * pose["x"] - c * pose["y"],
        "yaw": _wrap(-pose["yaw"]),
    }


def _pose_between(first, second):
    """Return the relative pose that takes first to second."""
    return _pose_compose(_pose_inverse(first), second)


def _read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as stream:
        result = []
        for raw in csv.DictReader(stream):
            result.append({key: float(value) if key not in {"frame_id"} else value
                           for key, value in raw.items()})
        return result


def _shell_ros(command, env=None):
    setup = [
        "source /opt/ros/jazzy/setup.bash",
        '[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"',
        '[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"',
        '[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"',
        "exec " + shlex.join(command),
    ]
    return ["bash", "-lc", "\n".join(setup)]


class SlamMatchingExperiment:
    """Session recorder and offline replay owner."""

    def __init__(self):
        self.lock = threading.RLock()
        self.session_dir = None
        self.state = "IDLE"
        self.message = "Not active"
        self.recorder = None
        self.recorder_log = None
        self.worker = None
        self.error = None

    def status(self):
        with self.lock:
            return {
                "state": self.state,
                "message": self.message,
                "session_dir": self.session_dir,
                "error": self.error,
            }

    def _set(self, state, message, error=None):
        with self.lock:
            self.state, self.message, self.error = state, message, error

    def _spawn(self, name, command, env, log_path):
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        log = open(log_path, "ab", buffering=0)
        try:
            child = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                env=env, start_new_session=True,
            )
        except Exception:
            log.close()
            raise
        return child, log

    def prepare(self):
        """Create a self-contained session and start recording only /scan,/odom."""
        with self.lock:
            if self.state not in {"IDLE", "ERROR"}:
                return False
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session = os.path.join(ROOT, f"slam_match_comparison_{stamp}")
            suffix = 1
            while os.path.exists(session):
                session = os.path.join(ROOT, f"slam_match_comparison_{stamp}_{suffix}")
                suffix += 1
            for name in ("raw", "replay_off", "replay_on", "maps", "logs", "config"):
                os.makedirs(os.path.join(session, name), exist_ok=False)
            self.session_dir = session
            self.error = None
            self._write_configuration()
            with open(os.path.join(session, "external_ground_truth.json"), "w", encoding="utf-8") as stream:
                json.dump({"available": False, "notes": "No independent physical reference supplied."}, stream, indent=2)

            env = os.environ.copy()
            bag_path = os.path.join(session, "raw", "sensor_bag")
            command = _shell_ros(["ros2", "bag", "record", "--storage", "sqlite3", "-o", bag_path, "/scan", "/odom"], env)
            self.recorder, self.recorder_log = self._spawn(
                "raw_recorder", command, env, os.path.join(session, "logs", "raw_record.log"))
            self.state = "RECORDING"
            self.message = "Recording raw /scan and /odom"
            return True

    def _write_configuration(self):
        with open(ACTIVE_PARAMS, encoding="utf-8") as stream:
            base = yaml.safe_load(stream)
        params = base.setdefault("slam_toolbox", {}).setdefault("ros__parameters", {})
        params["use_sim_time"] = True
        params["odom_frame"] = "odom"
        params["map_frame"] = "map"
        params["base_frame"] = "base_link"
        params["scan_topic"] = "/scan"
        params["resolution"] = 0.03
        params["do_loop_closing"] = False
        params["use_response_expansion"] = False
        # Pin the current production values that affect replay fairness.
        params.setdefault("minimum_time_interval", 0.1)
        params.setdefault("transform_timeout", 0.5)
        params.setdefault("tf_buffer_duration", 60.0)
        params.setdefault("map_update_interval", 0.5)
        params.setdefault("throttle_scans", 1)
        params.setdefault("minimum_travel_distance", 0.015)
        params.setdefault("minimum_travel_heading", 0.02)
        params.setdefault("use_scan_barycenter", True)
        params.setdefault("scan_queue_size", 1)
        for key, value in CONSERVATIVE_PARAMS.items():
            params[key] = value
        for enabled, name in ((False, "slam_off.yaml"), (True, "slam_on.yaml")):
            branch = yaml.safe_load(yaml.safe_dump(base, sort_keys=False))
            branch["slam_toolbox"]["ros__parameters"]["use_scan_matching"] = enabled
            with open(os.path.join(self.session_dir, "config", name), "w", encoding="utf-8") as stream:
                yaml.safe_dump(branch, stream, sort_keys=False)
        calibration = {
            "wheel_radius_m": 0.0350,
            "encoder_cpr": 4606,
            "wheel_separation_cmd_m": 0.22235,
            "wheel_separation_odom_m": 0.22235,
            "lidar_tf": {"parent": "base_link", "child": "d500_lidar", "x_m": 0.0, "y_m": 0.0, "z_m": 0.07, "roll_rad": 0.0, "pitch_rad": 0.0, "yaw_rad": 0.0},
            "map_resolution_m": 0.03,
            "loop_closing": False,
        }
        with open(os.path.join(self.session_dir, "config", "calibration_snapshot.json"), "w", encoding="utf-8") as stream:
            json.dump(calibration, stream, indent=2)
        manifest = {
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "active_params_source": ACTIVE_PARAMS,
            "slam_toolbox_version": "2.8.4 source checkout",
            "replay_domain": REPLAY_DOMAIN,
            "raw_topics_recorded": ["/scan", "/odom"],
            "raw_topics_not_recorded": ["/cmd_vel", "/tf", "/tf_static", "/map"],
            "replay_rate": 1.0,
            "conservative_params": CONSERVATIVE_PARAMS,
        }
        with open(os.path.join(self.session_dir, "config", "session_manifest.json"), "w", encoding="utf-8") as stream:
            json.dump(manifest, stream, indent=2, default=str)

    def stop_recording(self):
        with self.lock:
            recorder = self.recorder
            log = self.recorder_log
            self.recorder = None
            self.recorder_log = None
        if recorder is None:
            return
        try:
            os.killpg(recorder.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            recorder.wait(timeout=12.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(recorder.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                recorder.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(recorder.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if log:
            log.close()

    def finish_async(self, odom_result_path):
        with self.lock:
            if self.state in {"PROCESSING", "COMPLETE"}:
                return
            self.worker = threading.Thread(target=self._worker, args=(odom_result_path,), name="slam_ab_worker", daemon=True)
            self.worker.start()

    def cancel(self):
        """Stop an unstarted/in-progress recording without touching other ROS nodes."""
        with self.lock:
            if self.state == "PROCESSING":
                return False
            had_session = self.session_dir is not None
        self.stop_recording()
        if had_session:
            self._set("IDLE", "Not active")
        return True

    def _worker(self, odom_result_path):
        try:
            self._set("PROCESSING", "Finalizing raw recording")
            self.stop_recording()
            bag_path = os.path.join(self.session_dir, "raw", "sensor_bag")
            if not os.path.exists(os.path.join(bag_path, "metadata.yaml")):
                raise RuntimeError("raw bag metadata.yaml was not created")
            branch_results = {}
            for enabled, branch in ((False, "off"), (True, "on")):
                self._set("PROCESSING", f"Replaying scan matching {'ON' if enabled else 'OFF'}")
                branch_results[branch] = self._run_branch(branch, bag_path)
            self._set("PROCESSING", "Generating common-scale maps and metrics")
            result = self._build_summary(branch_results, odom_result_path, bag_path)
            with open(os.path.join(self.session_dir, "comparison_result.json"), "w", encoding="utf-8") as stream:
                json.dump(result, stream, indent=2, default=str)
            self._write_report(result)
            self._set("COMPLETE", "SLAM scan-matching A/B comparison complete")
        except Exception as exc:
            self._set("ERROR", f"SLAM A/B comparison failed: {exc}", str(exc))

    def _run_branch(self, branch, bag_path):
        branch_dir = os.path.join(self.session_dir, "replay_" + branch)
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(REPLAY_DOMAIN)
        env["ROS_LOCALHOST_ONLY"] = "1"
        processes = []
        try:
            tf_proc, tf_log = self._spawn("odom_tf", ["bash", "-lc", f"source /opt/ros/jazzy/setup.bash; source $HOME/ros2_ws/install/setup.bash; exec python3 {shlex.quote(os.path.join(os.path.dirname(__file__), 'offline_odom_tf_republisher.py'))}"], env, os.path.join(branch_dir, "odom_tf.log"))
            processes.append((tf_proc, tf_log))
            static_proc, static_log = self._spawn("static_tf", _shell_ros(["ros2", "run", "tf2_ros", "static_transform_publisher", "--x", "0.0", "--y", "0.0", "--z", "0.07", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0", "--frame-id", "base_link", "--child-frame-id", "d500_lidar"], env), env, os.path.join(branch_dir, "static_tf.log"))
            processes.append((static_proc, static_log))
            collector_cmd = ["bash", "-lc", f"source /opt/ros/jazzy/setup.bash; source $HOME/ros2_ws/install/setup.bash; exec python3 {shlex.quote(os.path.join(os.path.dirname(__file__), 'offline_slam_collector.py'))} --output-dir {shlex.quote(branch_dir)}"]
            collector, collector_log = self._spawn("collector", collector_cmd, env, os.path.join(branch_dir, "collector.log"))
            processes.append((collector, collector_log))
            slam_cmd = _shell_ros(["ros2", "launch", "slam_toolbox", "online_async_launch.py", f"slam_params_file:={os.path.join(self.session_dir, 'config', 'slam_' + branch + '.yaml')}", "use_sim_time:=true"])
            slam, slam_log = self._spawn("slam", slam_cmd, env, os.path.join(branch_dir, "slam.log"))
            processes.append((slam, slam_log))
            # Give the lifecycle node and collector time to discover the
            # replay topics.  The bag is not started until this delay elapses;
            # this avoids losing the first scans on a cold Pi without invoking
            # an extra ROS CLI observer in the replay domain.
            time.sleep(8.0)
            # ros2 bag play's --topics option consumes the remaining positional
            # arguments, so the bag URI must appear before it.
            bag_cmd = _shell_ros(["ros2", "bag", "play", bag_path, "--clock", "--rate", "1.0", "--delay", "2.0", "--topics", "/scan", "/odom"])
            bag, bag_log = self._spawn("bag_play", bag_cmd, env, os.path.join(branch_dir, "bag_play.log"))
            processes.append((bag, bag_log))
            bag_return = bag.wait(timeout=900.0)
            time.sleep(3.0)
            self._terminate_process(collector)
            self._terminate_process(slam)
            self._terminate_process(tf_proc)
            self._terminate_process(static_proc)
            for process, log in processes:
                if process.poll() is None:
                    self._terminate_process(process)
                log.close()
            map_path = os.path.join(branch_dir, "slam_map.json")
            valid = os.path.exists(map_path) and os.path.getsize(map_path) > 20
            if valid:
                self._write_branch_map_artifacts(branch_dir, map_path)
            expected_odom = self._bag_message_count(bag_path, "/odom")
            expected_scan = self._bag_message_count(bag_path, "/scan")
            counts_path = os.path.join(branch_dir, "replay_counts.json")
            try:
                with open(counts_path, encoding="utf-8") as stream:
                    counts = json.load(stream)
            except (OSError, json.JSONDecodeError):
                counts = {}
            received_odom = int(counts.get("odom_count", 0))
            received_scan = int(counts.get("scan_count", 0))
            odom_coverage = (received_odom / expected_odom) if expected_odom else None
            scan_coverage = (received_scan / expected_scan) if expected_scan else None
            if ((odom_coverage is not None and odom_coverage < 0.95) or
                    (scan_coverage is not None and scan_coverage < 0.95)):
                valid = False
            return {"valid": valid, "bag_play_returncode": bag_return, "map_path": map_path, "trajectory_path": os.path.join(branch_dir, "slam_trajectory.csv"), "expected_odom_samples": expected_odom, "received_odom_samples": received_odom, "odom_coverage": odom_coverage, "expected_scan_samples": expected_scan, "received_scan_samples": received_scan, "scan_coverage": scan_coverage}
        finally:
            for process, log in processes:
                if process.poll() is None:
                    self._terminate_process(process)
                try:
                    log.close()
                except Exception:
                    pass

    @staticmethod
    def _terminate_process(process):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass

    @staticmethod
    def _bag_message_count(bag_path, topic):
        try:
            import rosbag2_py
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
            count = 0
            while reader.has_next():
                name, _, _ = reader.read_next()
                count += int(name == topic)
            return count
        except Exception:
            return None


    def _build_summary(self, branches, odom_result_path, bag_path):
        odom_dir = os.path.dirname(odom_result_path)
        for filename in ("odom_trajectory.csv", "command_trajectory.csv", "odom_drift_result.json", "odom_test_result.json", "odom_drift_report.txt", "analyzer.log"):
            source = os.path.join(odom_dir, filename)
            if os.path.exists(source):
                shutil.copy2(source, os.path.join(self.session_dir, "raw", filename))
        off = load_map(branches["off"]["map_path"]) if branches["off"]["valid"] else None
        on = load_map(branches["on"]["map_path"]) if branches["on"]["valid"] else None
        maps_dir = os.path.join(self.session_dir, "maps")
        rendered = {}
        if off and on:
            canvas = common_canvas([off, on])
            render_map(off, canvas, os.path.join(maps_dir, "map_scan_matching_off.png"))
            render_map(on, canvas, os.path.join(maps_dir, "map_scan_matching_on.png"))
            render_side_by_side(off, on, os.path.join(maps_dir, "map_comparison_side_by_side.png"))
            rendered = {"canvas": canvas, "off": "maps/map_scan_matching_off.png", "on": "maps/map_scan_matching_on.png", "side_by_side": "maps/map_comparison_side_by_side.png"}

        off_rows = _read_csv(branches["off"]["trajectory_path"])
        on_rows = _read_csv(branches["on"]["trajectory_path"])
        off_odom = _read_csv(os.path.join(self.session_dir, "replay_off", "replay_odom.csv"))
        on_odom = _read_csv(os.path.join(self.session_dir, "replay_on", "replay_odom.csv"))
        corrections = self._write_corrections(on_rows, on_odom)
        try:
            with open(odom_result_path, encoding="utf-8") as stream:
                odom_result = json.load(stream)
        except Exception:
            odom_result = {"status": "unavailable", "path": odom_result_path}
        lidar_reference = self._lidar_reference(bag_path)
        branches_result = {
            "off": {**branches["off"], "relative_pose": _relative_pose(off_rows) if off_rows else None, "odom_relative_pose": _relative_pose(off_odom) if off_odom else None},
            "on": {**branches["on"], "relative_pose": _relative_pose(on_rows) if on_rows else None, "odom_relative_pose": _relative_pose(on_odom) if on_odom else None, "corrections": corrections},
        }
        result = {
            "status": "ok" if off and on else "invalid",
            "session_dir": self.session_dir,
            "bag_path": bag_path,
            "same_raw_sensor_recording": True,
            "loop_closing": False,
            "raw_topics": ["/scan", "/odom"],
            "raw_topics_excluded": ["/cmd_vel", "/tf", "/tf_static", "/map"],
            "odom_result": odom_result,
            "lidar_reference_endpoint": lidar_reference,
            "branches": branches_result,
            "rendered_maps": rendered,
            "external_ground_truth": self._ground_truth_metrics(
                odom_result, branches_result["off"].get("relative_pose"), branches_result["on"].get("relative_pose")),
            "interpretation": "Without independent external ground truth, endpoint values are closure residuals and lidar-referenced diagnostics, not absolute physical accuracy.",
        }
        off_pose = result["branches"]["off"].get("relative_pose")
        on_pose = result["branches"]["on"].get("relative_pose")
        if off_pose and on_pose:
            result["on_minus_off"] = {key: on_pose[key] - off_pose[key] for key in ("x_m", "y_m", "position_m", "yaw_deg")}
        return result

    def _write_branch_map_artifacts(self, branch_dir, map_path):
        map_data = load_map(map_path)
        canvas = common_canvas([map_data])
        pgm_path = os.path.join(branch_dir, "map.pgm")
        render_map(map_data, canvas, pgm_path)
        yaml_path = os.path.join(branch_dir, "map.yaml")
        with open(yaml_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump({
                "image": os.path.basename(pgm_path),
                "resolution": float(map_data["resolution"]),
                "origin": [float(map_data["origin_x"]), float(map_data["origin_y"]), 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            }, stream, sort_keys=False)

    def _ground_truth_metrics(self, odom_result, off_pose, on_pose):
        path = os.path.join(self.session_dir, "external_ground_truth.json")
        try:
            with open(path, encoding="utf-8") as stream:
                ground_truth = json.load(stream)
        except (OSError, json.JSONDecodeError):
            return {"available": False}
        if not ground_truth.get("available"):
            return ground_truth
        gt_x = float(ground_truth.get("x_m", 0.0))
        gt_y = float(ground_truth.get("y_m", 0.0))
        gt_yaw = math.radians(float(ground_truth.get("yaw_deg", 0.0)))

        def error(pose):
            if not pose:
                return None
            return {"position_error_m": math.hypot(pose["x_m"] - gt_x, pose["y_m"] - gt_y),
                    "yaw_error_deg": math.degrees(_wrap(pose["yaw_rad"] - gt_yaw))}

        raw = None
        if odom_result.get("status") == "ok":
            raw = {"position_error_m": math.hypot(float(odom_result.get("forward_displacement_m", 0.0)) - gt_x, float(odom_result.get("lateral_displacement_m", 0.0)) - gt_y),
                   "yaw_error_deg": math.degrees(_wrap(float(odom_result.get("yaw_error_rad", 0.0)) - gt_yaw))}
        off_error = error(off_pose)
        on_error = error(on_pose)
        value = {"available": True, "method": ground_truth.get("method"), "raw_odom": raw, "scan_matching_off": off_error, "scan_matching_on": on_error, "notes": ground_truth.get("notes", "")}
        if off_error and on_error:
            position_reduction = off_error["position_error_m"] - on_error["position_error_m"]
            yaw_reduction = abs(off_error["yaw_error_deg"]) - abs(on_error["yaw_error_deg"])
            value["on_vs_off_improvement"] = {
                "position_error_reduction_m": position_reduction,
                "position_error_reduction_percent": 100.0 * position_reduction / off_error["position_error_m"] if off_error["position_error_m"] > 1e-9 else None,
                "absolute_yaw_error_reduction_deg": yaw_reduction,
                "absolute_yaw_error_reduction_percent": 100.0 * yaw_reduction / abs(off_error["yaw_error_deg"]) if abs(off_error["yaw_error_deg"]) > 1e-9 else None,
            }
        return value

    def _write_corrections(self, slam_rows, odom_rows):
        path = os.path.join(self.session_dir, "replay_on", "scan_matching_corrections.csv")
        fields = [
            "timestamp",
            "odom_predicted_x", "odom_predicted_y", "odom_predicted_yaw",
            "slam_result_x", "slam_result_y", "slam_result_yaw",
            "local_correction_dx", "local_correction_dy",
            "local_correction_distance_m", "local_correction_yaw_deg",
            "local_warning",
            "cumulative_correction_dx", "cumulative_correction_dy",
            "cumulative_correction_distance_m", "cumulative_correction_yaw_deg",
            "is_initial",
            # Backward-compatible names.  These intentionally refer to the
            # LOCAL correction, never to the cumulative map/odom difference.
            "correction_dx", "correction_dy", "correction_distance_m",
            "correction_yaw_deg", "warning",
        ]
        empty = {
            "count": 0,
            "max_translation_m": None,
            "p95_translation_m": None,
            "max_yaw_deg": None,
            "p95_yaw_deg": None,
            "large_correction_warnings": 0,
            "max_cumulative_translation_m": None,
            "max_cumulative_yaw_deg": None,
        }
        if not slam_rows or not odom_rows:
            with open(path, "w", newline="", encoding="utf-8") as stream:
                csv.DictWriter(stream, fieldnames=fields).writeheader()
            return empty

        # Normalize both trajectories to their own first accepted pose.  This
        # removes arbitrary map/odom origins while retaining the original
        # timestamps and relative motion.
        slam_start = {key: slam_rows[0][key] for key in ("x", "y", "yaw")}
        odom_start = {key: odom_rows[0][key] for key in ("x", "y", "yaw")}
        pairs = []
        for pose in slam_rows:
            nearest = min(odom_rows, key=lambda item: abs(item["timestamp"] - pose["timestamp"]))
            pairs.append((pose, nearest))

        local_distances, local_angles = [], []
        cumulative_distances, cumulative_angles = [], []
        with open(path, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            previous_slam = None
            previous_odom = None
            for index, (pose, nearest) in enumerate(pairs):
                slam_rel = _pose_between(slam_start, {key: pose[key] for key in ("x", "y", "yaw")})
                odom_rel = _pose_between(odom_start, {key: nearest[key] for key in ("x", "y", "yaw")})

                if previous_slam is None:
                    # There is no preceding update with which to form a local
                    # correction.  Keep the initial row for traceability, but
                    # exclude it from warning/statistics calculations.
                    predicted = dict(slam_rel)
                    local = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                    initial = 1
                else:
                    odom_delta = _pose_between(previous_odom, odom_rel)
                    # Prediction = previous accepted SLAM pose plus the
                    # intervening odometry motion.  The local correction is
                    # the accepted SLAM pose relative to that prediction.
                    predicted = _pose_compose(previous_slam, odom_delta)
                    local = _pose_between(predicted, slam_rel)
                    initial = 0

                # This is a separate cumulative diagnostic: T_map_odom =
                # T_map_base * inverse(T_odom_base), both normalized at the
                # first accepted sample.  It is deliberately not used for
                # teleport warnings.
                cumulative = _pose_compose(slam_rel, _pose_inverse(odom_rel))
                local_yaw_deg = math.degrees(local["yaw"])
                local_distance = math.hypot(local["x"], local["y"])
                cumulative_yaw_deg = math.degrees(cumulative["yaw"])
                cumulative_distance = math.hypot(cumulative["x"], cumulative["y"])
                warning = int(not initial and (local_distance > 0.10 or abs(local_yaw_deg) > 3.0))

                if not initial:
                    local_distances.append(local_distance)
                    local_angles.append(abs(local_yaw_deg))
                cumulative_distances.append(cumulative_distance)
                cumulative_angles.append(abs(cumulative_yaw_deg))
                writer.writerow({
                    "timestamp": pose["timestamp"],
                    "odom_predicted_x": predicted["x"],
                    "odom_predicted_y": predicted["y"],
                    "odom_predicted_yaw": predicted["yaw"],
                    "slam_result_x": slam_rel["x"],
                    "slam_result_y": slam_rel["y"],
                    "slam_result_yaw": slam_rel["yaw"],
                    "local_correction_dx": local["x"],
                    "local_correction_dy": local["y"],
                    "local_correction_distance_m": local_distance,
                    "local_correction_yaw_deg": local_yaw_deg,
                    "local_warning": warning,
                    "cumulative_correction_dx": cumulative["x"],
                    "cumulative_correction_dy": cumulative["y"],
                    "cumulative_correction_distance_m": cumulative_distance,
                    "cumulative_correction_yaw_deg": cumulative_yaw_deg,
                    "is_initial": initial,
                    "correction_dx": local["x"],
                    "correction_dy": local["y"],
                    "correction_distance_m": local_distance,
                    "correction_yaw_deg": local_yaw_deg,
                    "warning": warning,
                })
                previous_slam = slam_rel
                previous_odom = odom_rel

        if not local_distances:
            return {**empty, "max_cumulative_translation_m": max(cumulative_distances), "max_cumulative_yaw_deg": max(cumulative_angles)}
        numpy = __import__("numpy")
        return {
            "count": len(local_distances),
            "max_translation_m": max(local_distances),
            "p95_translation_m": float(numpy.percentile(local_distances, 95)),
            "max_yaw_deg": max(local_angles),
            "p95_yaw_deg": float(numpy.percentile(local_angles, 95)),
            "large_correction_warnings": sum(1 for distance, angle in zip(local_distances, local_angles) if distance > 0.10 or angle > 3.0),
            "max_cumulative_translation_m": max(cumulative_distances),
            "max_cumulative_yaw_deg": max(cumulative_angles),
        }

    def _lidar_reference(self, bag_path):
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
            topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
            scan_type = get_message(topics["/scan"])
            records = []
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic != "/scan":
                    continue
                msg = deserialize_message(data, scan_type)
                records.append({"ranges": list(msg.ranges), "range_min": msg.range_min, "range_max": msg.range_max, "angle_increment": msg.angle_increment})
            return analyze_fixed_reference(records)
        except Exception as exc:
            return {"valid": False, "reason": f"fixed-reference extraction failed: {exc}"}

    def _write_report(self, result):
        path = os.path.join(self.session_dir, "slam_matching_comparison_report.txt")
        with open(path, "w", encoding="utf-8") as stream:
            stream.write("Robot 1 controlled Slam Toolbox scan-matching A/B experiment\n")
            stream.write("===========================================================\n\n")
            stream.write(f"Session: {self.session_dir}\n")
            stream.write("Both branches used the same raw /scan and /odom recording.\n")
            stream.write("/cmd_vel, /tf, /tf_static, and live /map were not recorded or replayed.\n")
            stream.write("Loop closure was disabled in both branches.\n")
            stream.write("The fixed-reference lidar endpoint is not independent ground truth.\n\n")
            stream.write("Scan-matching correction diagnostics:\n")
            stream.write("  Local correction = accepted current SLAM pose relative to the pose predicted from the previous accepted SLAM pose plus intervening odometry.\n")
            stream.write("  Cumulative correction = separate normalized T_map_odom diagnostic.\n")
            stream.write("  Teleport thresholds (>0.10 m or >3 deg) are applied only to local per-update correction.\n\n")
            stream.write("Frozen calibration snapshot:\n")
            stream.write("  wheel radius: 0.0350 m\n  encoder CPR: 4606\n")
            stream.write("  wheel separation command/odom: 0.22235 / 0.22235 m\n")
            stream.write("  lidar TF: base_link -> d500_lidar, (0, 0, 0.07), yaw 0\n")
            stream.write("  map resolution: 0.03 m\n\n")
            stream.write("Conservative ON matcher values (Karto source units):\n")
            for key, value in CONSERVATIVE_PARAMS.items():
                stream.write(f"  {key}: {value}\n")
            stream.write("\nPenalty model: max(1 - 0.2*delta^2/variance, minimum), with the ROS wrapper squaring the supplied variance values.\n")
            stream.write("Distance sigma=0.05 m: 0 cm=1.000, 2 cm=0.968, 5 cm=0.800, 10 cm=0.200 (before minimum).\n")
            stream.write("Angle sigma=3 deg: 0 deg=1.000, 0.5 deg=0.994, 1 deg=0.978, 2 deg=0.911, 5 deg=0.700 after minimum.\n")
            stream.write("Response expansion is disabled because this installed Karto retries with +20,+40,+60 degrees when the first response is zero.\n\n")
            stream.write("Raw odometry result:\n")
            stream.write(json.dumps(result.get("odom_result", {}), indent=2))
            stream.write("\n\nReplay results:\n")
            stream.write(json.dumps(result.get("branches", {}), indent=2))
            stream.write("\n\nLidar-referenced endpoint diagnostic:\n")
            stream.write(json.dumps(result.get("lidar_reference_endpoint", {}), indent=2))
            stream.write("\n\nGenerated maps:\n")
            stream.write(json.dumps(result.get("rendered_maps", {}), indent=2))
            stream.write("\n\nInterpretation:\n")
            stream.write(result["interpretation"] + "\n")

    def shutdown(self):
        self.stop_recording()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=2.0)
