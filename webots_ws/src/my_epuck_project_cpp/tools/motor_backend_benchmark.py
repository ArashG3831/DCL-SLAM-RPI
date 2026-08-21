#!/usr/bin/env python3
"""Lifted-wheel A/B benchmark for the opt-in motor backends.

The test publishes the same straight command to either the Python or C++ motor
node, records odometry and safety output, and samples process/per-core CPU.
It is intentionally isolated with ROS_DOMAIN_ID so it cannot command the live
production motor node.
"""

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
from std_msgs.msg import String


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
    lines = Path("/proc/stat").read_text().splitlines()
    total = None
    cores = {}
    for line in lines:
        fields = line.split()
        if not fields or (fields[0] != "cpu" and not fields[0].startswith("cpu")):
            continue
        values = [int(value) for value in fields[1:]]
        idle = values[3] if len(values) > 3 else 0
        iowait = values[4] if len(values) > 4 else 0
        item = {"total": sum(values), "idle": idle, "iowait": iowait}
        if fields[0] == "cpu":
            total = item
        else:
            cores[fields[0]] = item
    return total, cores


def temperature_c():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (FileNotFoundError, ValueError):
        return None


def percentile(values, p):
    if not values:
        return 0.0
    values = sorted(values)
    index = round((len(values) - 1) * p / 100.0)
    return values[index]


class Collector(Node):
    def __init__(self):
        super().__init__("motor_backend_benchmark_collector")
        self.odom = []
        self.faults = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(String, "/motor_safety/fault", self.on_fault, 10)

    def on_odom(self, message):
        self.odom.append(time.monotonic())

    def on_fault(self, message):
        self.faults.append({"time": time.time(), "message": message.data})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("python", "cpp"), required=True)
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--output", required=True)
    parser.add_argument("--domain", type=int, default=178)
    parser.add_argument("--rpm", type=float, default=-50.0)
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(args.domain)
    log_path = Path(args.output).with_suffix(".node.log")
    log_stream = log_path.open("w")
    if args.backend == "cpp":
        command = [str(Path.home() / "webots_ws/install/my_epuck_project_cpp/lib/my_epuck_project_cpp/real_diffdrive_node_cpp")]
    else:
        command = [str(Path.home() / "webots_ws/install/my_epuck_project/lib/my_epuck_project/real_diffdrive_node")]
    process = None
    rclpy.init(args=None)
    node = Collector()
    publisher = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)
    samples = []
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])

    def send(linear):
        message = Twist()
        message.linear.x = linear
        publisher.publish(message)

    try:
        process = subprocess.Popen(command, env=env, stdout=log_stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        time.sleep(1.5)
        old_proc, _ = proc_stats(process.pid)
        old_total, old_cores = cpu_stats()
        start = time.monotonic()
        last_sample = start
        while time.monotonic() - start < args.seconds:
            send(args.rpm * 2.0 * 3.141592653589793 * 0.035 / 60.0)
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_sample >= 0.1:
                new_proc, rss = proc_stats(process.pid)
                new_total, new_cores = cpu_stats()
                elapsed = max(now - last_sample, 1e-9)
                process_cpu = 100.0 * (new_proc - old_proc) / hz / elapsed if new_proc is not None and old_proc is not None else 0.0
                active = 100.0 * ((new_total["total"] - old_total["total"]) -
                                   (new_total["idle"] - old_total["idle"]) -
                                   (new_total["iowait"] - old_total["iowait"])) / max(new_total["total"] - old_total["total"], 1)
                per_core = {}
                for name, value in new_cores.items():
                    old = old_cores.get(name)
                    if old is None:
                        continue
                    delta = max(value["total"] - old["total"], 1)
                    per_core[name] = 100.0 * ((value["total"] - old["total"]) -
                                               (value["idle"] - old["idle"]) -
                                               (value["iowait"] - old["iowait"])) / delta
                samples.append({"process_cpu_percent_one_core": process_cpu,
                                "whole_system_active_cpu_percent": active,
                                "per_core_active_cpu_percent": per_core,
                                "rss_bytes": rss,
                                "temperature_c": temperature_c()})
                old_proc, old_total, old_cores = new_proc, new_total, new_cores
                last_sample = now
            if node.faults:
                break
        send(0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
            send(0.0)
        result = {
            "backend": args.backend,
            "status": "FAIL_SAFETY" if node.faults else "PASS",
            "duration_s": time.monotonic() - start,
            "target_rpm": args.rpm,
            "target_linear_mps": args.rpm * 2.0 * 3.141592653589793 * 0.035 / 60.0,
            "odom_samples": len(node.odom),
            "safety_faults": node.faults,
            "process_cpu_percent_one_core": {
                "mean": sum(s["process_cpu_percent_one_core"] for s in samples) / max(len(samples), 1),
                "p95": percentile([s["process_cpu_percent_one_core"] for s in samples], 95),
                "max": max((s["process_cpu_percent_one_core"] for s in samples), default=0.0),
            },
            "whole_system_active_cpu_percent": {
                "mean": sum(s["whole_system_active_cpu_percent"] for s in samples) / max(len(samples), 1),
                "p95": percentile([s["whole_system_active_cpu_percent"] for s in samples], 95),
                "max": max((s["whole_system_active_cpu_percent"] for s in samples), default=0.0),
            },
            "per_core_active_cpu_percent_p95": {
                core: percentile([s["per_core_active_cpu_percent"].get(core, 0.0) for s in samples], 95)
                for core in sorted({core for s in samples for core in s["per_core_active_cpu_percent"]})
            },
            "rss_bytes_max": max((s["rss_bytes"] or 0 for s in samples), default=0),
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
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
        node.destroy_node()
        rclpy.shutdown()
        log_stream.close()


if __name__ == "__main__":
    main()
