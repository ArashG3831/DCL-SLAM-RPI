#!/usr/bin/env python3
"""Compare the actual Python MotorPI logic with the native control core."""

import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

ORACLE_ROOT = Path(__file__).parents[4] / "webots_ws/src/my_epuck_project"
sys.path.insert(0, str(ORACLE_ROOT))


class FakeTime:
    now = 1.0

    @classmethod
    def monotonic(cls):
        return cls.now


class FakePin:
    edges = None
    bounce = None
    when_changed = None


class FakeInput:
    def __init__(self, *_args, **_kwargs):
        self.value = 0
        self.pin = FakePin()

    def close(self):
        pass


class FakeOutput:
    def __init__(self, *_args, **_kwargs):
        self.value = 0.0

    def on(self):
        pass

    def off(self):
        pass

    def close(self):
        pass


def load_motor_module():
    module = importlib.import_module("my_epuck_project.real_diffdrive_node")
    module.PWMOutputDevice = FakeOutput
    module.DigitalOutputDevice = FakeOutput
    module.DigitalInputDevice = FakeInput
    module.time = FakeTime
    return module


def add_signed_transitions(motor, signed_count, timestamp):
    # Right encoder sign is +1 and left is -1.  Feed the corresponding raw
    # phase direction so both Python wheels observe +192 signed counts.
    sequence = (1, 3, 2, 0) if motor.encoder.encoder_sign == 1 else (2, 3, 1, 0)
    for _ in range(signed_count // 4):
        for state in sequence:
            motor.encoder.process_state(state, timestamp)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_motor_golden.py MOTOR_CORE_CLI")
    module = load_motor_module()
    right = module.MotorPI("right", 18, 23, 24, 17, 27, motor_sign=1, encoder_sign=1)
    left = module.MotorPI("left", 13, 5, 6, 22, 26, motor_sign=-1, encoder_sign=-1)
    right.set_target_rpm(50.0)
    left.set_target_rpm(50.0)
    rows = []
    for t in (1.0, 1.05, 1.10, 1.15, 1.20):
        FakeTime.now = t
        if t > 1.0:
            add_signed_transitions(right, 192, int(t * 1e9))
            add_signed_transitions(left, 192, int(t * 1e9))
        left_state = left.update()
        right_state = right.update()
        if not rows:
            x = y = theta = 0.0
            last_t = t
        else:
            dt = t - last_t
            vl = left_state["measured_rpm"] / 60.0 * (2.0 * math.pi * 0.0350)
            vr = right_state["measured_rpm"] / 60.0 * (2.0 * math.pi * 0.0350)
            v = (vr + vl) / 2.0
            omega = (vr - vl) / 0.22235
            dtheta = omega * dt
            x += v * math.cos(theta + dtheta / 2.0) * dt
            y += v * math.sin(theta + dtheta / 2.0) * dt
            theta = math.atan2(math.sin(theta + dtheta), math.cos(theta + dtheta))
            last_t = t
        rows.append({
            "t": t,
            "left_target": left.target_rpm,
            "right_target": right.target_rpm,
            "left_rpm": left_state["measured_rpm"],
            "right_rpm": right_state["measured_rpm"],
            "left_count": left_state["signed_count"],
            "right_count": right_state["signed_count"],
            "left_valid": left_state["valid_transition_count"],
            "right_valid": right_state["valid_transition_count"],
            "left_invalid": left_state["invalid_transition_count"],
            "right_invalid": right_state["invalid_transition_count"],
            "left_command": left_state["command"],
            "right_command": right_state["command"],
            "x": x, "y": y, "theta": theta,
        })
    cpp = json.loads(subprocess.check_output([sys.argv[1]], text=True))["steps"]
    if len(cpp) != len(rows):
        raise AssertionError("step count mismatch")
    fields = ("left_target", "right_target", "left_rpm", "right_rpm",
              "left_count", "right_count", "left_valid", "right_valid",
              "left_invalid", "right_invalid",
              "left_command", "right_command", "x", "y", "theta")
    for index, (py_row, cpp_row) in enumerate(zip(rows, cpp)):
        for field in fields:
            if abs(py_row[field] - cpp_row[field]) > 1e-9:
                raise AssertionError(f"{field} mismatch at step {index}: {py_row[field]} vs {cpp_row[field]}")
    print(json.dumps({"status": "PASS", "steps": len(rows), "fields_compared": list(fields)}, indent=2))


if __name__ == "__main__":
    main()
