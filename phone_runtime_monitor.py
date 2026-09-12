#!/usr/bin/env python3
"""Lightweight, read-only runtime profile for the phone-controlled stack."""

import csv
import json
import os
import threading
import time
from pathlib import Path

from phone_controller_config import (
    PHONE_RUNTIME_MONITOR_ENABLED,
    PHONE_RUNTIME_MONITOR_INTERVAL_S,
    PHONE_RUNTIME_PROFILE_DIR,
)


ROLE_NAMES = ("phone", "motor", "lidar", "slam_on", "slam_off", "bag", "other")


def _proc_stat(pid):
    try:
        raw = Path(f"/proc/{pid}/stat").read_text()
        closing = raw.rfind(")")
        fields = raw[closing + 2 :].split()
        # After the comm field, ppid is field 4 and utime/stime are 14/15.
        return {
            "pid": pid,
            "ppid": int(fields[1]),
            "ticks": int(fields[11]) + int(fields[12]),
            "command": Path(f"/proc/{pid}/cmdline").read_bytes()
            .replace(b"\x00", b" ")
            .decode(errors="replace")
            .strip(),
            "rss": _rss_bytes(pid),
        }
    except (FileNotFoundError, PermissionError, IndexError, ValueError):
        return None


def _rss_bytes(pid):
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except (FileNotFoundError, PermissionError, IndexError, ValueError):
        pass
    return 0


def _all_processes():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        process = _proc_stat(int(entry.name))
        if process is not None:
            processes[process["pid"]] = process
    return processes


def _descendants(processes, root_pid):
    result = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, process in processes.items():
            if process["ppid"] in result and pid not in result:
                result.add(pid)
                changed = True
    return result


def _role(command, root_pid, pid):
    if pid == root_pid:
        return "phone"
    if "real_diffdrive_node_cpp" in command or command.endswith("real_diffdrive_node"):
        return "motor"
    if "d500_ros2_scan_cpp" in command or "d500_ros2_scan.py" in command:
        return "lidar"
    if "live_slam_off_launcher.py" in command:
        return "slam_off"
    if "async_slam_toolbox_node" in command:
        if "slam_toolbox_off" in command or "map_off" in command or "slam_off" in command:
            return "slam_off"
        return "slam_on"
    if "ros2 bag record" in command or "rosbag2_recorder" in command:
        return "bag"
    return "other"


def _cpu_totals():
    totals = {}
    try:
        for line in Path("/proc/stat").read_text().splitlines():
            fields = line.split()
            if not fields or not (fields[0] == "cpu" or fields[0].startswith("cpu")):
                continue
            values = [int(value) for value in fields[1:]]
            totals[fields[0]] = {
                "total": sum(values),
                "idle": values[3] + (values[4] if len(values) > 4 else 0),
            }
    except (FileNotFoundError, ValueError, IndexError):
        return {}
    return totals


def _temperature_c():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (FileNotFoundError, ValueError):
        return None


def _throttle_flags():
    try:
        output = os.popen("vcgencmd get_throttled 2>/dev/null").read().strip()
        return output or "unknown"
    except OSError:
        return "unknown"


def _percentile(values, percentile):
    if not values:
        return 0.0
    values = sorted(values)
    index = round((len(values) - 1) * percentile / 100.0)
    return values[index]


class PhoneRuntimeMonitor:
    """Sample process/system load without touching ROS or hardware."""

    def __init__(self):
        self.enabled = PHONE_RUNTIME_MONITOR_ENABLED
        self.interval_s = max(0.25, PHONE_RUNTIME_MONITOR_INTERVAL_S)
        self.root_pid = os.getpid()
        self.stop_event = threading.Event()
        self.thread = None
        self.profile_dir = None
        self.samples = []

    def start(self):
        if not self.enabled:
            return
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.profile_dir = Path(PHONE_RUNTIME_PROFILE_DIR) / f"phone_run_{timestamp}"
        self.profile_dir.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(
            target=self._run,
            name="phone_runtime_monitor",
            daemon=True,
        )
        self.thread.start()
        print(f"Runtime profile: {self.profile_dir}")

    def _run(self):
        hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        started = time.monotonic()
        previous_total = _cpu_totals()
        previous_processes = {}
        rows = []
        while not self.stop_event.wait(self.interval_s):
            now = time.monotonic()
            totals = _cpu_totals()
            processes = _all_processes()
            tracked = _descendants(processes, self.root_pid)
            role_ticks = {role: 0.0 for role in ROLE_NAMES}
            role_rss = {role: 0 for role in ROLE_NAMES}
            for pid in tracked:
                process = processes.get(pid)
                if process is None:
                    continue
                role = _role(process["command"], self.root_pid, pid)
                old_ticks = previous_processes.get(pid, process["ticks"])
                role_ticks[role] += max(0, process["ticks"] - old_ticks)
                role_rss[role] = max(role_rss[role], process["rss"])

            whole = totals.get("cpu")
            old_whole = previous_total.get("cpu")
            whole_cpu = 0.0
            if whole and old_whole:
                delta = max(1, whole["total"] - old_whole["total"])
                idle_delta = whole["idle"] - old_whole["idle"]
                whole_cpu = 100.0 * (delta - idle_delta) / delta

            row = {
                "wall_time": time.time(),
                "elapsed_s": now - started,
                "whole_pi_cpu_percent": whole_cpu,
                "temperature_c": _temperature_c(),
                "throttled": _throttle_flags(),
                "tracked_processes": len(tracked),
            }
            for role in ROLE_NAMES:
                row[f"{role}_cpu_percent_one_core"] = 100.0 * role_ticks[role] / hz / self.interval_s
                row[f"{role}_rss_bytes"] = role_rss[role]
            rows.append(row)
            previous_total = totals
            previous_processes = {
                pid: process["ticks"]
                for pid, process in processes.items()
                if pid in tracked
            }

        self.samples = rows
        self._write(rows)

    def _write(self, rows):
        if self.profile_dir is None:
            return
        fields = list(rows[0]) if rows else ["wall_time", "elapsed_s"]
        with (self.profile_dir / "samples.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        summary = {
            "pid": self.root_pid,
            "sample_count": len(rows),
            "profile_dir": str(self.profile_dir),
            "roles": {},
        }
        for role in ROLE_NAMES:
            values = [row[f"{role}_cpu_percent_one_core"] for row in rows]
            summary["roles"][role] = {
                "mean_cpu_percent_one_core": sum(values) / max(1, len(values)),
                "p95_cpu_percent_one_core": _percentile(values, 95),
                "max_cpu_percent_one_core": max(values, default=0.0),
                "max_rss_bytes": max(
                    (row[f"{role}_rss_bytes"] for row in rows), default=0
                ),
            }
        whole = [row["whole_pi_cpu_percent"] for row in rows]
        summary["whole_pi_cpu_percent"] = {
            "mean": sum(whole) / max(1, len(whole)),
            "p95": _percentile(whole, 95),
            "max": max(whole, default=0.0),
        }
        (self.profile_dir / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

    def stop(self):
        if self.thread is None:
            return
        self.stop_event.set()
        self.thread.join(timeout=max(3.0, self.interval_s + 2.0))
        self.thread = None
