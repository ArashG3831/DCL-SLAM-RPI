#!/usr/bin/env python3
"""Motor + D500 A/B benchmark with identical ROS-domain isolation."""

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
HOME = Path.home()


def proc_stats(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        rss = 0
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                rss = int(line.split()[1]) * 1024
                break
        return int(fields[13]) + int(fields[14]), rss
    except (FileNotFoundError, IndexError, ValueError):
        return None, None


def cpu_stats():
    total = None
    cores = {}
    for line in Path("/proc/stat").read_text().splitlines():
        fields = line.split()
        if not fields or (fields[0] != "cpu" and not fields[0].startswith("cpu")):
            continue
        values = [int(value) for value in fields[1:]]
        item = {"total": sum(values), "idle": values[3], "iowait": values[4] if len(values) > 4 else 0}
        if fields[0] == "cpu":
            total = item
        else:
            cores[fields[0]] = item
    return total, cores


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    return values[round((len(values) - 1) * p / 100.0)]


def temperature_c():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (FileNotFoundError, ValueError):
        return None


class Collector(Node):
    def __init__(self):
        super().__init__("hardware_pair_benchmark_collector")
        self.scan_stamps = []
        self.odom_count = 0
        self.faults = []
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(String, "/motor_safety/fault", self.on_fault, 10)

    def on_scan(self, message):
        self.scan_stamps.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)

    def on_odom(self, _message):
        self.odom_count += 1

    def on_fault(self, message):
        self.faults.append({"time": time.time(), "message": message.data})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--motor", choices=("python", "cpp"), required=True)
    parser.add_argument("--lidar", choices=("python", "cpp"), required=True)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--domain", type=int, default=180)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain)
    motor = (HOME / "webots_ws/install/my_epuck_project_cpp/lib/my_epuck_project_cpp/real_diffdrive_node_cpp" if args.motor == "cpp"
             else HOME / "webots_ws/install/my_epuck_project/lib/my_epuck_project/real_diffdrive_node")
    lidar = (HOME / "webots_ws/install/my_epuck_project_cpp/lib/my_epuck_project_cpp/d500_ros2_scan_cpp" if args.lidar == "cpp"
             else HOME / "d500_ros2_scan.py")
    motor_cmd = [str(motor)]
    lidar_cmd = ([str(lidar), "--ros-args", "-p", f"port:={PORT}", "-p", "frame_id:=d500_lidar", "-p", "topic:=/scan"]
                 if args.lidar == "cpp" else ["python3", str(lidar), "--port", PORT, "--topic", "/scan", "--frame-id", "d500_lidar"])
    log_path = Path(args.output).with_suffix(".nodes.log")
    log_stream = log_path.open("w")
    processes = []
    rclpy.init(args=None)
    node = Collector()
    publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)
    samples = []
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

    def send(linear):
        msg = Twist()
        msg.linear.x = linear
        publisher.publish(msg)

    try:
        for command in (motor_cmd, lidar_cmd):
            processes.append(subprocess.Popen(command, env=env, stdout=log_stream,
                                              stderr=subprocess.STDOUT, start_new_session=True))
        time.sleep(2.0)
        previous_process = {p.pid: proc_stats(p.pid)[0] for p in processes}
        previous_total, previous_cores = cpu_stats()
        previous_wall = time.monotonic()
        start = previous_wall
        linear = -50.0 * 2.0 * 3.141592653589793 * 0.035 / 60.0
        while time.monotonic() - start < args.seconds:
            send(linear)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - previous_wall >= 0.1:
                current_total, current_cores = cpu_stats()
                elapsed = max(now - previous_wall, 1e-9)
                process_cpu = {}
                rss = {}
                for process in processes:
                    current, current_rss = proc_stats(process.pid)
                    old = previous_process.get(process.pid)
                    process_cpu[str(process.pid)] = 100.0 * (current - old) / hz / elapsed if current is not None and old is not None else 0.0
                    rss[str(process.pid)] = current_rss or 0
                    previous_process[process.pid] = current
                active = 100.0 * ((current_total["total"] - previous_total["total"]) -
                                   (current_total["idle"] - previous_total["idle"]) -
                                   (current_total["iowait"] - previous_total["iowait"])) / max(current_total["total"] - previous_total["total"], 1)
                per_core = {}
                for name, current in current_cores.items():
                    old = previous_cores.get(name)
                    if old is None:
                        continue
                    delta = max(current["total"] - old["total"], 1)
                    per_core[name] = 100.0 * ((current["total"] - old["total"]) -
                                               (current["idle"] - old["idle"]) -
                                               (current["iowait"] - old["iowait"])) / delta
                samples.append({"process_cpu": process_cpu, "rss": rss,
                                "whole_system_active": active, "per_core_active": per_core,
                                "temperature_c": temperature_c()})
                previous_total, previous_cores, previous_wall = current_total, current_cores, now
            if node.faults:
                break
        send(0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
            send(0.0)
        intervals = [b - a for a, b in zip(node.scan_stamps, node.scan_stamps[1:]) if b >= a]
        process_summary = {}
        for index, process in enumerate(processes):
            key = "motor" if index == 0 else "lidar"
            values = [sample["process_cpu"].get(str(process.pid), 0.0) for sample in samples]
            process_summary[key] = {"mean": sum(values) / max(len(values), 1),
                                    "p95": percentile(values, 95), "max": max(values, default=0.0)}
        result = {
            "motor_backend": args.motor, "lidar_backend": args.lidar,
            "status": "FAIL_SAFETY" if node.faults else ("PASS" if node.scan_stamps and node.odom_count else "FAIL_NO_DATA"),
            "duration_s": time.monotonic() - start, "target_rpm": -50.0,
            "odom_samples": node.odom_count, "scan_count": len(node.scan_stamps),
            "scan_interval_s": {"p50": percentile(intervals, 50), "p95": percentile(intervals, 95), "max": max(intervals, default=0.0)},
            "safety_faults": node.faults,
            "process_cpu_percent_one_core": process_summary,
            "whole_system_active_cpu_percent": {
                "mean": sum(s["whole_system_active"] for s in samples) / max(len(samples), 1),
                "p95": percentile([s["whole_system_active"] for s in samples], 95),
                "max": max((s["whole_system_active"] for s in samples), default=0.0)},
            "per_core_active_cpu_percent_p95": {
                core: percentile([s["per_core_active"].get(core, 0.0) for s in samples], 95)
                for core in sorted({core for s in samples for core in s["per_core_active"]})},
            "rss_bytes_max": {"motor": max((s["rss"].get(str(processes[0].pid), 0) for s in samples), default=0),
                              "lidar": max((s["rss"].get(str(processes[1].pid), 0) for s in samples), default=0)},
            "temperature_c_max": max((s["temperature_c"] for s in samples if s["temperature_c"] is not None), default=None),
            "node_log": str(log_path),
        }
        Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        if result["status"] != "PASS":
            raise SystemExit(1)
    finally:
        send(0.0)
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        log_stream.close()


if __name__ == "__main__":
    main()
