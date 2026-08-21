#!/usr/bin/env python3
"""Real-D500-only benchmark for the opt-in C++ driver; never starts motors."""

import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def cpu_ticks(pid):
    try:
        fields = Path(f"/proc/{pid}/stat").read_text().split()
        return int(fields[13]) + int(fields[14])
    except (FileNotFoundError, IndexError):
        return None


def total_cpu_ticks():
    fields = Path("/proc/stat").read_text().splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def temperature_c():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (FileNotFoundError, ValueError):
        return None


class ScanCollector(Node):
    def __init__(self):
        super().__init__("d500_cpp_live_benchmark_collector")
        self.messages = []
        self.received_wall = []
        self.create_subscription(LaserScan, "/scan", self.receive, 10)

    def receive(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.messages.append({"stamp": stamp, "bins": len(message.ranges),
                              "frame": message.header.frame_id,
                              "scan_time": message.scan_time})
        self.received_wall.append(time.time())


def percentile(values, p):
    if not values:
        return 0.0
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * p / 100.0)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--output", required=True)
    ap.add_argument("--backend", choices=("cpp", "python"), default="cpp")
    args = ap.parse_args()

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "177"
    os.environ["ROS_DOMAIN_ID"] = "177"
    log_path = Path(args.output).with_suffix(".node.log")
    log_stream = log_path.open("w")
    process = None
    rclpy.init(args=None)
    collector = ScanCollector()
    samples = []
    try:
        if args.backend == "cpp":
            executable = Path.home() / "webots_ws/install/my_epuck_project_cpp/lib/my_epuck_project_cpp/d500_ros2_scan_cpp"
            command = [str(executable), "--ros-args", "-p", f"port:={args.port}",
                       "-p", "frame_id:=d500_lidar", "-p", "topic:=/scan"]
        else:
            command = ["python3", str(Path.home() / "d500_ros2_scan.py"),
                       "--port", args.port, "--topic", "/scan", "--frame-id", "d500_lidar"]
        process = subprocess.Popen(command, env=env, stdout=log_stream,
            stderr=subprocess.STDOUT, text=True, start_new_session=True)
        time.sleep(1.5)
        hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        old_process = cpu_ticks(process.pid)
        old_total = total_cpu_ticks()
        start = time.monotonic()
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(collector, timeout_sec=0.1)
            now_process = cpu_ticks(process.pid)
            now_total = total_cpu_ticks()
            if now_process is not None and old_process is not None:
                process_cpu = 100.0 * (now_process - old_process) / hz / 0.1
            else:
                process_cpu = 0.0
            total_delta = now_total[0] - old_total[0]
            busy_delta = (now_total[0] - old_total[0]) - (now_total[1] - old_total[1])
            total_cpu = 100.0 * busy_delta / total_delta if total_delta else 0.0
            samples.append({"process_cpu_percent": process_cpu,
                            "total_cpu_percent": total_cpu,
                            "temperature_c": temperature_c()})
            old_process, old_total = now_process, now_total
        stamps = [message["stamp"] for message in collector.messages]
        intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b >= a]
        result = {
            "backend": args.backend,
            "status": "PASS" if collector.messages else "FAIL_NO_SCAN",
            "duration_s": args.seconds,
            "scan_count": len(collector.messages),
            "bin_counts": sorted({message["bins"] for message in collector.messages}),
            "frames": sorted({message["frame"] for message in collector.messages}),
            "scan_interval_s": {
                "p50": percentile(intervals, 50),
                "p95": percentile(intervals, 95),
                "max": max(intervals, default=0.0),
            },
            "process_cpu_percent": {
                "mean": sum(s["process_cpu_percent"] for s in samples) / len(samples),
                "p95": percentile([s["process_cpu_percent"] for s in samples], 95),
            } if samples else {},
            "whole_system_cpu_percent": {
                "mean": sum(s["total_cpu_percent"] for s in samples) / len(samples),
                "p95": percentile([s["total_cpu_percent"] for s in samples], 95),
            } if samples else {},
            "temperature_c_max": max((s["temperature_c"] for s in samples if s["temperature_c"] is not None), default=None),
            "node_log": str(log_path),
        }
        Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2))
        if result["status"] != "PASS":
            raise SystemExit(1)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
        collector.destroy_node()
        rclpy.shutdown()
        log_stream.close()


if __name__ == "__main__":
    main()
