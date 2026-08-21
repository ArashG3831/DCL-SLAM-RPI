#!/usr/bin/env python3
"""Read-only process/system sampler for repeatable native-port benchmarks."""

import argparse
import csv
import os
import time
from pathlib import Path


def cpu_totals():
    rows = {}
    for line in Path("/proc/stat").read_text().splitlines():
        fields = line.split()
        if fields and (fields[0] == "cpu" or fields[0].startswith("cpu")):
            values = [int(x) for x in fields[1:]]
            idle = values[3] + (values[4] if len(values) > 4 else 0)
            rows[fields[0]] = (sum(values), idle)
    return rows


def process_cpu(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().split()
    return int(fields[13]) + int(fields[14])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", required=True)
    ap.add_argument("--pids", nargs="*", type=int, default=[])
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--interval", type=float, default=1.0)
    args = ap.parse_args()
    hz = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    previous = cpu_totals()
    previous_proc = {}
    for pid in args.pids:
        try:
            previous_proc[pid] = process_cpu(pid)
        except (FileNotFoundError, IndexError):
            pass
    start = time.monotonic()
    target = Path(args.output).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["wall_time", "cpu", "util_percent", "idle_percent"] +
                        [f"pid_{pid}_cpu_percent" for pid in args.pids])
        while time.monotonic() - start < args.seconds:
            time.sleep(args.interval)
            current = cpu_totals()
            process_values = []
            for pid in args.pids:
                try:
                    now_ticks = process_cpu(pid)
                    old_ticks = previous_proc.get(pid, now_ticks)
                    process_values.append(100.0 * (now_ticks - old_ticks) / hz / args.interval)
                except (FileNotFoundError, IndexError):
                    process_values.append(0.0)
            for cpu, (total, idle) in current.items():
                old_total, old_idle = previous.get(cpu, (total, idle))
                delta = total - old_total
                idle_delta = idle - old_idle
                util = 100.0 * (delta - idle_delta) / delta if delta else 0.0
                writer.writerow([time.time(), cpu, util, 100.0 - util] + process_values)
            previous = current
            previous_proc = {}
            for pid in args.pids:
                try:
                    previous_proc[pid] = process_cpu(pid)
                except (FileNotFoundError, IndexError):
                    pass
    print(target)


if __name__ == "__main__":
    main()
