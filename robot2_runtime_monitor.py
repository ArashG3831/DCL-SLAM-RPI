#!/usr/bin/env python3
"""Low-overhead process and Raspberry Pi runtime monitor for Robot 2 runs."""

import argparse
import csv
import os
import signal
import time
from pathlib import Path


ROLE_PATTERNS = {
    "motor": ("real_diffdrive_node_cpp", "real_diffdrive_node"),
    "d500": ("d500_ros2_scan_cpp", "d500_ros2_scan.py"),
    "slam": ("async_slam_toolbox_node",),
    "controller": ("/nav2_controller/", "controller_server"),
    "planner": ("/nav2_planner/", "planner_server"),
    "behavior": ("/nav2_behaviors/", "behavior_server"),
    "bt_navigator": ("bt_navigator",),
    "frontier": ("frontier_explorer",),
    "zenoh": ("zenoh-bridge-ros2dds",),
}


def read_processes():
    processes = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        try:
            stat = (entry / "stat").read_text()
            close = stat.rfind(")")
            fields = stat[close + 2 :].split()
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                errors="replace"
            ).strip()
            rss = 0
            for line in (entry / "status").read_text().splitlines():
                if line.startswith("VmRSS:"):
                    rss = int(line.split()[1]) * 1024
                    break
            processes[pid] = {
                "ticks": int(fields[11]) + int(fields[12]),
                "command": command,
                "rss": rss,
            }
        except (FileNotFoundError, PermissionError, IndexError, ValueError):
            continue
    return processes


def role_for(command):
    for role, patterns in ROLE_PATTERNS.items():
        if any(pattern in command for pattern in patterns):
            return role
    return "other"


def cpu_totals():
    result = {}
    for line in Path("/proc/stat").read_text().splitlines():
        fields = line.split()
        if not fields or not (fields[0] == "cpu" or fields[0].startswith("cpu")):
            continue
        values = [int(value) for value in fields[1:]]
        result[fields[0]] = {
            "total": sum(values),
            "idle": values[3] + (values[4] if len(values) > 4 else 0),
        }
    return result


def memory_bytes():
    values = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        fields = line.split()
        if len(fields) >= 2:
            values[fields[0].rstrip(":")] = int(fields[1]) * 1024
    return values


def temperature():
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000
    except (FileNotFoundError, ValueError):
        return ""


def throttled():
    try:
        value = os.popen("vcgencmd get_throttled 2>/dev/null").read().strip()
        return value or "unknown"
    except OSError:
        return "unknown"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    path = output / "system_samples.csv"
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    previous_total = cpu_totals()
    previous_processes = {}
    started = time.time()
    roles = tuple(ROLE_PATTERNS) + ("other",)
    fields = ["wall_time", "elapsed_s", "whole_pi_cpu_percent", "temperature_c", "throttled", "available_ram_bytes", "swap_used_bytes"]
    for role in roles:
        fields.extend((f"{role}_cpu_percent_one_core", f"{role}_rss_bytes", f"{role}_process_count"))

    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        while not stopping:
            time.sleep(max(0.25, args.interval))
            now = time.time()
            totals = cpu_totals()
            processes = read_processes()
            role_ticks = {role: 0 for role in roles}
            role_rss = {role: 0 for role in roles}
            role_count = {role: 0 for role in roles}
            for pid, process in processes.items():
                role = role_for(process["command"])
                old = previous_processes.get(pid, process["ticks"])
                role_ticks[role] += max(0, process["ticks"] - old)
                role_rss[role] = max(role_rss[role], process["rss"])
                role_count[role] += 1

            whole = totals.get("cpu")
            old_whole = previous_total.get("cpu")
            whole_cpu = 0.0
            if whole and old_whole:
                delta = max(1, whole["total"] - old_whole["total"])
                idle_delta = whole["idle"] - old_whole["idle"]
                whole_cpu = 100.0 * (delta - idle_delta) / delta

            mem = memory_bytes()
            row = {
                "wall_time": now,
                "elapsed_s": now - started,
                "whole_pi_cpu_percent": whole_cpu,
                "temperature_c": temperature(),
                "throttled": throttled(),
                "available_ram_bytes": mem.get("MemAvailable", ""),
                "swap_used_bytes": max(0, mem.get("SwapTotal", 0) - mem.get("SwapFree", 0)),
            }
            for role in roles:
                row[f"{role}_cpu_percent_one_core"] = 100.0 * role_ticks[role] / hz / max(0.25, args.interval)
                row[f"{role}_rss_bytes"] = role_rss[role]
                row[f"{role}_process_count"] = role_count[role]
            writer.writerow(row)
            stream.flush()
            previous_total = totals
            previous_processes = {pid: process["ticks"] for pid, process in processes.items()}


if __name__ == "__main__":
    main()
