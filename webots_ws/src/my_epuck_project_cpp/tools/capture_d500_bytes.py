#!/usr/bin/env python3
"""Optional lidar-only raw capture; never opens motor GPIO or publishes ROS."""

import argparse
import time
from pathlib import Path

import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--baud", type=int, default=230400)
    args = parser.parse_args()
    target = Path(args.output).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    with serial.Serial(args.port, baudrate=args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        end = time.monotonic() + args.seconds
        with target.open("wb") as output:
            while time.monotonic() < end:
                chunk = ser.read(4096)
                if chunk:
                    output.write(chunk)
    print(f"captured {target} ({target.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
