#!/usr/bin/env python3

import csv
import math
import os
import queue
import threading
import time
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from gpiozero import Device, DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice

from .quadrature_decoder import QuadratureDecoder


# GPIO pins, BCM numbering
M1_EN = 18
M1_IN1 = 23
M1_IN2 = 24
M1_ENC_A = 17
M1_ENC_B = 27  # Physical pin 13

M2_EN = 13
M2_IN1 = 5
M2_IN2 = 6
M2_ENC_A = 22
M2_ENC_B = 26  # Physical pin 37

# Physical forward-only calibration: four clean 10-revolution trials measured
# a combined mean of 4605.5 transitions/revolution. Use one shared integer CPR
# because left/right means differed by only about 0.06%.
DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION = 4606.0
MAX_RPM_ESTIMATE = 55.0
MAX_TARGET_RPM = 50.0
PWM_FREQ = 1000

# Keep the callback-rate diagnostic tied to the calibrated production CPR.
# At the maximum RPM estimate, 4606 X4 transitions/rev produces about 4222
# callbacks/s per wheel, or about 8444 callbacks/s for both wheels together.
QUADRATURE_TRANSITIONS_PER_REV_ESTIMATE = (
    DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION
)
MAX_EXPECTED_ENCODER_EDGE_RATE_HZ = (
    QUADRATURE_TRANSITIONS_PER_REV_ESTIMATE * MAX_RPM_ESTIMATE / 60.0
)

KP = 0.020
KI = 0.020
CONTROL_DT = 0.05

# Real motor deadband compensation.
# These gearmotors + L298N cannot reliably move at tiny nonzero RPM commands.
# Keep exact zero as zero, but boost small real commands to a usable minimum.
RPM_ZERO_EPS = 0.15
MIN_EFFECTIVE_RPM = 12.0


def apply_motor_deadband_rpm(rpm: float) -> float:
    if abs(rpm) < RPM_ZERO_EPS:
        return 0.0
    if abs(rpm) < MIN_EFFECTIVE_RPM:
        return math.copysign(MIN_EFFECTIVE_RPM, rpm)
    return rpm


# Motor electrical direction signs.
M1_SIGN = 1
M2_SIGN = -1

# Encoder sign maps the decoder's conventional state sequence to physical
# forward wheel motion. These signs are from the short powered forward test;
# they are not inferred from the motor command direction.
M1_ENCODER_SIGN = 1   # right/M1: physical forward decoded positive
M2_ENCODER_SIGN = -1  # left/M2: physical forward decoded negative raw

# Real chassis straight-line calibration.
LEFT_RPM_SCALE = 1.00
RIGHT_RPM_SCALE = 1.00

# PID benchmark logging.
PID_BENCH_ENABLE = False
PID_BENCH_DURATION_S = 2.0
PID_BENCH_DIR = "~/pid_bench_logs"

# A diagnostic deadline miss is a cycle that starts more than 1 ms after its
# monotonic deadline. This threshold is diagnostic-only and does not alter
# production safety behavior.
CONTROL_DEADLINE_MISS_TOLERANCE_S = 0.001


def _percentile(values, percentile):
    """Return an approximate percentile from a bounded diagnostic window."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * percentile / 100.0))
    return ordered[max(0, min(len(ordered) - 1, index))]


class EncoderTimingDiagnostics:
    """Bounded, low-overhead timing diagnostics for GPIO callback delivery."""

    WINDOW_SIZE = 8192

    def __init__(self, name):
        self.name = name
        self.enabled = os.environ.get("R1_ENCODER_DIAGNOSTICS", "0") == "1"
        self.backend_gaps = deque(maxlen=self.WINDOW_SIZE)
        self.callback_gaps = deque(maxlen=self.WINDOW_SIZE)
        self.callback_durations = deque(maxlen=self.WINDOW_SIZE)
        self.last_backend_ticks = {"A": None, "B": None}
        self.last_callback_entry = {"A": None, "B": None}
        self.backend_max_gap = 0.0
        self.callback_max_gap = 0.0
        self.callback_max_duration = 0.0
        self.edge_count = 0
        self.control_max_abs_count_delta = 0
        self.control_max_abs_edge_delta = 0

    def edge_entry(self, channel, backend_ticks, callback_entry):
        if not self.enabled:
            return

        self.edge_count += 1
        previous_ticks = self.last_backend_ticks[channel]
        if backend_ticks is not None:
            try:
                backend_ticks = float(backend_ticks)
            except (TypeError, ValueError):
                backend_ticks = None
        if backend_ticks is not None and previous_ticks is not None:
            gap = backend_ticks - previous_ticks
            if gap >= 0.0:
                self.backend_gaps.append(gap)
                self.backend_max_gap = max(self.backend_max_gap, gap)
        if backend_ticks is not None:
            self.last_backend_ticks[channel] = backend_ticks

        previous_entry = self.last_callback_entry[channel]
        if previous_entry is not None:
            gap = callback_entry - previous_entry
            if gap >= 0.0:
                self.callback_gaps.append(gap)
                self.callback_max_gap = max(self.callback_max_gap, gap)
        self.last_callback_entry[channel] = callback_entry

    def edge_exit(self, callback_entry, callback_exit):
        if not self.enabled:
            return
        duration = max(0.0, callback_exit - callback_entry)
        self.callback_durations.append(duration)
        self.callback_max_duration = max(self.callback_max_duration, duration)

    def control_batch(self, count_delta, a_edge_delta, b_edge_delta):
        if not self.enabled:
            return
        self.control_max_abs_count_delta = max(
            self.control_max_abs_count_delta, abs(int(count_delta))
        )
        self.control_max_abs_edge_delta = max(
            self.control_max_abs_edge_delta,
            abs(int(a_edge_delta)),
            abs(int(b_edge_delta)),
        )

    def summary(self):
        if not self.enabled:
            return "disabled"
        return (
            f"{self.name} edges={self.edge_count} "
            f"backend_gap_max={self.backend_max_gap * 1000.0:.3f}ms "
            f"backend_gap_p99={_percentile(list(self.backend_gaps), 99.0) * 1000.0:.3f}ms "
            f"callback_gap_max={self.callback_max_gap * 1000.0:.3f}ms "
            f"callback_gap_p99={_percentile(list(self.callback_gaps), 99.0) * 1000.0:.3f}ms "
            f"callback_exec_max={self.callback_max_duration * 1000.0:.3f}ms "
            f"callback_exec_p99={_percentile(list(self.callback_durations), 99.0) * 1000.0:.3f}ms "
            f"max_count_delta={self.control_max_abs_count_delta} "
            f"max_channel_edge_delta={self.control_max_abs_edge_delta}"
        )


class ControlTimingDiagnostics:
    """Bounded timing metrics for the dedicated 20 Hz control thread."""

    WINDOW_SIZE = 8192

    def __init__(self):
        self.enabled = (
            os.environ.get("R1_CONTROL_DIAGNOSTICS", "0") == "1"
            or os.environ.get("R1_ENCODER_DIAGNOSTICS", "0") == "1"
        )
        self.periods = deque(maxlen=self.WINDOW_SIZE)
        self.execution_times = deque(maxlen=self.WINDOW_SIZE)
        self.deadline_lateness = deque(maxlen=self.WINDOW_SIZE)
        self.encoder_ages = deque(maxlen=self.WINDOW_SIZE)
        self.last_actual_start = None
        self.deadline_misses = 0
        self.skipped_deadlines = 0
        self.max_overrun = 0.0
        self.max_encoder_age = 0.0
        self.encoder_age_unknown_cycles = 0

    def cycle_start(self, scheduled_start, actual_start):
        if not self.enabled:
            return

        if self.last_actual_start is not None:
            self.periods.append(actual_start - self.last_actual_start)
        self.last_actual_start = actual_start

        lateness = max(0.0, actual_start - scheduled_start)
        self.deadline_lateness.append(lateness)
        if lateness > CONTROL_DEADLINE_MISS_TOLERANCE_S:
            self.deadline_misses += 1

    def cycle_end(self, actual_start, actual_end, encoder_age):
        if not self.enabled:
            return

        execution = max(0.0, actual_end - actual_start)
        self.execution_times.append(execution)
        self.max_overrun = max(
            self.max_overrun,
            max(0.0, execution - CONTROL_DT),
        )
        if math.isfinite(encoder_age):
            self.encoder_ages.append(max(0.0, encoder_age))
            self.max_encoder_age = max(self.max_encoder_age, encoder_age)
        else:
            self.encoder_age_unknown_cycles += 1

    def skipped(self, count):
        if self.enabled:
            self.skipped_deadlines += max(0, int(count))

    def summary(self):
        if not self.enabled:
            return "disabled"

        return (
            "CONTROL_TIMING "
            f"period_p50={_percentile(list(self.periods), 50.0) * 1000.0:.3f}ms "
            f"period_p95={_percentile(list(self.periods), 95.0) * 1000.0:.3f}ms "
            f"period_p99={_percentile(list(self.periods), 99.0) * 1000.0:.3f}ms "
            f"period_max={max(self.periods, default=0.0) * 1000.0:.3f}ms "
            f"exec_p50={_percentile(list(self.execution_times), 50.0) * 1000.0:.3f}ms "
            f"exec_p95={_percentile(list(self.execution_times), 95.0) * 1000.0:.3f}ms "
            f"exec_p99={_percentile(list(self.execution_times), 99.0) * 1000.0:.3f}ms "
            f"exec_max={max(self.execution_times, default=0.0) * 1000.0:.3f}ms "
            f"deadline_misses={self.deadline_misses} "
            f"skipped_deadlines={self.skipped_deadlines} "
            f"max_overrun={self.max_overrun * 1000.0:.3f}ms "
            f"encoder_age_max={self.max_encoder_age * 1000.0:.3f}ms "
            f"encoder_age_unknown_cycles={self.encoder_age_unknown_cycles}"
        )


class MotorPI:
    def __init__(
        self,
        name,
        en_pin,
        in1_pin,
        in2_pin,
        enc_a_pin,
        enc_b_pin,
        motor_sign=1,
        encoder_sign=1,
        counts_per_wheel_revolution=DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION,
        safety_window_s=0.35,
        startup_grace_s=0.60,
    ):
        self.name = name
        self.state_lock = threading.RLock()
        self.motor_sign = motor_sign
        self.counts_per_wheel_revolution = float(counts_per_wheel_revolution)
        self.safety_window_s = float(safety_window_s)
        self.startup_grace_s = float(startup_grace_s)

        self.pwm = PWMOutputDevice(en_pin, frequency=PWM_FREQ, initial_value=0)
        self.in1 = DigitalOutputDevice(in1_pin)
        self.in2 = DigitalOutputDevice(in2_pin)
        self.enc_a = DigitalInputDevice(enc_a_pin, pull_up=True)
        self.enc_b = DigitalInputDevice(enc_b_pin, pull_up=True)

        initial_state = (int(self.enc_a.value) << 1) | int(self.enc_b.value)
        self.encoder = QuadratureDecoder(initial_state, encoder_sign=encoder_sign)

        # DigitalInputDevice already requests both edges from the pin factory.
        # Replace its higher-level active/inactive event handler with the
        # timestamped pin callback so the decoder receives every A/B edge with
        # minimal callback work and without reading GPIO from the callback.
        self.enc_a.pin.edges = "both"
        self.enc_b.pin.edges = "both"
        self.enc_a.pin.bounce = None
        self.enc_b.pin.bounce = None
        self.enc_a.pin.when_changed = self._encoder_a_changed
        self.enc_b.pin.when_changed = self._encoder_b_changed

        now = time.monotonic()

        self.target_rpm = 0.0
        self.command_direction = 0
        self.target_abs_rpm = 0.0
        self.integral = 0.0
        self.last_encoder_count = 0
        self.last_valid_transition_count = 0
        self.last_invalid_transition_count = 0
        self.last_a_edge_count = 0
        self.last_b_edge_count = 0
        self.last_time = now
        self.last_measured_rpm = 0.0
        self.last_command = 0.0
        self.motion_grace_until = now
        self.start_command_pending = None

        # Samples are (time, signed count delta, valid delta, invalid delta,
        # A edge delta, B edge delta).
        self.health_samples = deque()
        self.underspeed_since = None
        self.implausible_rpm_cycles = 0
        self.timing_diagnostics = EncoderTimingDiagnostics(name)

    def _encoder_a_changed(self, ticks, state):
        callback_entry = time.monotonic() if self.timing_diagnostics.enabled else None
        if callback_entry is not None:
            self.timing_diagnostics.edge_entry("A", ticks, callback_entry)
        self.encoder.process_edge("A", state, ticks)
        if callback_entry is not None:
            self.timing_diagnostics.edge_exit(callback_entry, time.monotonic())

    def _encoder_b_changed(self, ticks, state):
        callback_entry = time.monotonic() if self.timing_diagnostics.enabled else None
        if callback_entry is not None:
            self.timing_diagnostics.edge_entry("B", ticks, callback_entry)
        self.encoder.process_edge("B", state, ticks)
        if callback_entry is not None:
            self.timing_diagnostics.edge_exit(callback_entry, time.monotonic())

    def get_encoder_snapshot(self):
        snapshot = self.encoder.snapshot()
        snapshot["signed_count"] = snapshot["count"]
        snapshot["a_count"] = snapshot["a_edge_count"]
        snapshot["b_count"] = snapshot["b_edge_count"]
        return snapshot

    def raw_drive(self, command):
        # Physical motor writes are confined to the control thread, except
        # for the final shutdown stop after that thread has joined.
        command = max(-1.0, min(1.0, command))
        command *= self.motor_sign

        if command > 0:
            self.in1.on()
            self.in2.off()
            self.pwm.value = abs(command)
        elif command < 0:
            self.in1.off()
            self.in2.on()
            self.pwm.value = abs(command)
        else:
            self.pwm.value = 0.0
            self.in1.off()
            self.in2.off()

        # Keep this as the signed logical command before motor_sign is applied.
        self.last_command = command / self.motor_sign

    def stop(self):
        with self.state_lock:
            self.pwm.value = 0.0
            self.in1.off()
            self.in2.off()
            self.last_command = 0.0
            self.start_command_pending = None

    def clear_health_state(self):
        with self.state_lock:
            self.health_samples.clear()
            self.underspeed_since = None
            self.implausible_rpm_cycles = 0

    def set_target_rpm(self, target_rpm):
        with self.state_lock:
            target_rpm = max(
                -MAX_TARGET_RPM,
                min(MAX_TARGET_RPM, float(target_rpm)),
            )

            old_direction = self.command_direction
            new_direction = 1 if target_rpm > 0 else -1 if target_rpm < 0 else 0

            if abs(target_rpm - self.target_rpm) < 1e-6:
                return

            self.target_rpm = target_rpm
            self.command_direction = new_direction
            self.target_abs_rpm = abs(target_rpm)

            # Do not reset encoder measurement for ordinary target changes.
            # The control thread performs the physical stop on its next cycle.
            if self.command_direction == 0:
                self.integral = 0.0
                self.health_samples.clear()
                self.underspeed_since = None
                self.implausible_rpm_cycles = 0
                self.start_command_pending = None
                return

            # Apply grace only when motion begins or truly reverses. Nav2
            # changes target magnitude frequently, which must not continuously
            # reset safety. The initial physical drive is deferred to update().
            if old_direction == 0 or old_direction != self.command_direction:
                self.integral = 0.0
                self.health_samples.clear()
                self.underspeed_since = None
                self.implausible_rpm_cycles = 0
                self.motion_grace_until = time.monotonic() + self.startup_grace_s

                start_mag = self.target_abs_rpm / MAX_RPM_ESTIMATE
                start_mag = max(0.25, min(1.0, start_mag))
                self.start_command_pending = self.command_direction * start_mag

    def update(self):
        with self.state_lock:
            return self._update_locked()

    def _update_locked(self):
        now = time.monotonic()
        snapshot = self.get_encoder_snapshot()

        count_now = snapshot["signed_count"]
        valid_now = snapshot["valid_transition_count"]
        invalid_now = snapshot["invalid_transition_count"]
        a_edges_now = snapshot["a_edge_count"]
        b_edges_now = snapshot["b_edge_count"]

        dcount = count_now - self.last_encoder_count
        dvalid = valid_now - self.last_valid_transition_count
        dinvalid = invalid_now - self.last_invalid_transition_count
        da_edges = a_edges_now - self.last_a_edge_count
        db_edges = b_edges_now - self.last_b_edge_count
        dt = now - self.last_time

        self.timing_diagnostics.control_batch(dcount, da_edges, db_edges)

        if dt <= 0.0:
            return {
                "measured_rpm": self.last_measured_rpm,
                "command": self.last_command,
                "count_delta": 0,
                "valid_transition_delta": 0,
                "invalid_transition_delta": 0,
                "a_edge_delta": 0,
                "b_edge_delta": 0,
                **snapshot,
            }

        measured_signed_rpm = (
            dcount / self.counts_per_wheel_revolution / dt * 60.0
        )
        measured_abs_rpm = abs(measured_signed_rpm)

        if self.command_direction == 0:
            self.stop()
        else:
            if self.start_command_pending is not None:
                self.raw_drive(self.start_command_pending)
                self.start_command_pending = None

            error = self.target_abs_rpm - measured_abs_rpm
            self.integral += error * dt
            self.integral = max(-30.0, min(30.0, self.integral))

            base = self.target_abs_rpm / MAX_RPM_ESTIMATE
            correction = KP * error + KI * self.integral
            command_mag = max(0.0, min(1.0, base + correction))

            self.raw_drive(self.command_direction * command_mag)

        self.last_encoder_count = count_now
        self.last_valid_transition_count = valid_now
        self.last_invalid_transition_count = invalid_now
        self.last_a_edge_count = a_edges_now
        self.last_b_edge_count = b_edges_now
        self.last_time = now
        self.last_measured_rpm = measured_signed_rpm

        self.health_samples.append((
            now, dcount, dvalid, dinvalid, da_edges, db_edges,
        ))
        cutoff = now - self.safety_window_s
        while self.health_samples and self.health_samples[0][0] < cutoff:
            self.health_samples.popleft()

        return {
            "measured_rpm": measured_signed_rpm,
            "command": self.last_command,
            "count_delta": dcount,
            "valid_transition_delta": dvalid,
            "invalid_transition_delta": dinvalid,
            "a_edge_delta": da_edges,
            "b_edge_delta": db_edges,
            **snapshot,
        }

    def window_metrics(self, now):
        with self.state_lock:
            cutoff = now - self.safety_window_s
            while self.health_samples and self.health_samples[0][0] < cutoff:
                self.health_samples.popleft()

        count_window = sum(sample[1] for sample in self.health_samples)
        valid_window = sum(sample[2] for sample in self.health_samples)
        invalid_window = sum(sample[3] for sample in self.health_samples)
        a_edge_window = sum(sample[4] for sample in self.health_samples)
        b_edge_window = sum(sample[5] for sample in self.health_samples)

        snapshot = self.get_encoder_snapshot()
        return {
            "count_window": count_window,
            "valid_transition_window": valid_window,
            "invalid_transition_window": invalid_window,
            "a_edge_window": a_edge_window,
            "b_edge_window": b_edge_window,
            "invalid_transition_percentage": snapshot["invalid_transition_percentage"],
            "a_age_s": (
                max(0.0, now - snapshot["last_a_edge_time"])
                if snapshot["last_a_edge_time"] is not None else float("inf")
            ),
            "b_age_s": (
                max(0.0, now - snapshot["last_b_edge_time"])
                if snapshot["last_b_edge_time"] is not None else float("inf")
            ),
            "a_level": snapshot["a_level"],
            "b_level": snapshot["b_level"],
            "decoded_direction": snapshot["decoded_direction"],
        }

    def close(self):
        self.stop()
        self.enc_a.close()
        self.enc_b.close()
        self.pwm.close()
        self.in1.close()
        self.in2.close()


class RealDiffDriveNode(Node):
    def __init__(self):
        super().__init__("real_diffdrive_node")

        # Robot 1 physical wheel diameter is 0.070 m, so radius is 0.0350 m.
        # Confirmed by the 1.15 m/1.17 m floor run and encoder counts.
        self.declare_parameter("wheel_radius_m", 0.0350)
        # Shared wheel-separation calibration from the bidirectional spin
        # tests: 0.22235 m (222.35 mm).
        self.declare_parameter("wheel_separation_cmd_m", 0.22235)
        self.declare_parameter("wheel_separation_odom_m", 0.22235)
        self.declare_parameter("cmd_timeout_s", 0.7)
        self.declare_parameter(
            "encoder_counts_per_wheel_revolution",
            DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION,
        )

        # Conservative dual-channel safety defaults.
        self.declare_parameter("safety_enabled", True)
        self.declare_parameter("safety_window_s", 0.35)
        self.declare_parameter("safety_startup_grace_s", 0.60)
        self.declare_parameter("safety_min_target_rpm", 10.0)
        self.declare_parameter("safety_min_pwm", 0.22)
        self.declare_parameter("safety_no_pulse_timeout_s", 0.35)
        self.declare_parameter("safety_min_window_pulses", 20)
        self.declare_parameter("safety_channel_ratio_min", 0.35)
        # A rolling window can contain counts from the previous command during
        # a quick reversal, so this comparison is diagnostic-only by default.
        self.declare_parameter("safety_direction_check_enabled", False)
        self.declare_parameter("safety_direction_min_samples", 20)
        self.declare_parameter("safety_direction_agreement_min", 0.80)
        self.declare_parameter("safety_max_encoder_rpm", 90.0)
        self.declare_parameter("safety_implausible_rpm_cycles", 2)
        self.declare_parameter("safety_underspeed_ratio", 0.20)
        self.declare_parameter("safety_underspeed_pwm", 0.65)
        self.declare_parameter("safety_underspeed_timeout_s", 0.80)

        # The batch runner provides these as per-process environment variables,
        # so every run gets an exact, self-contained safety log path.
        default_safety_log_dir = os.environ.get(
            "MOTOR_SAFETY_LOG_DIR", "~/motor_safety_logs"
        )
        default_safety_log_name = os.environ.get(
            "MOTOR_SAFETY_LOG_NAME", ""
        )
        self.declare_parameter("motor_safety_log_dir", default_safety_log_dir)
        self.declare_parameter("motor_safety_log_name", default_safety_log_name)

        self.wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_separation_cmd = float(self.get_parameter("wheel_separation_cmd_m").value)
        self.wheel_separation_odom = float(self.get_parameter("wheel_separation_odom_m").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.encoder_counts_per_wheel_revolution = float(
            self.get_parameter("encoder_counts_per_wheel_revolution").value
        )
        if self.encoder_counts_per_wheel_revolution <= 0.0:
            raise ValueError("encoder_counts_per_wheel_revolution must be positive")

        self.safety_enabled = bool(self.get_parameter("safety_enabled").value)
        self.safety_window_s = float(self.get_parameter("safety_window_s").value)
        self.safety_startup_grace_s = float(self.get_parameter("safety_startup_grace_s").value)
        self.safety_min_target_rpm = float(self.get_parameter("safety_min_target_rpm").value)
        self.safety_min_pwm = float(self.get_parameter("safety_min_pwm").value)
        self.safety_no_pulse_timeout_s = float(self.get_parameter("safety_no_pulse_timeout_s").value)
        self.safety_min_window_pulses = int(self.get_parameter("safety_min_window_pulses").value)
        self.safety_channel_ratio_min = float(self.get_parameter("safety_channel_ratio_min").value)
        self.safety_direction_check_enabled = bool(
            self.get_parameter("safety_direction_check_enabled").value
        )
        self.safety_direction_min_samples = int(self.get_parameter("safety_direction_min_samples").value)
        self.safety_direction_agreement_min = float(self.get_parameter("safety_direction_agreement_min").value)
        self.safety_max_encoder_rpm = float(self.get_parameter("safety_max_encoder_rpm").value)
        self.safety_implausible_rpm_cycles = int(self.get_parameter("safety_implausible_rpm_cycles").value)
        self.safety_underspeed_ratio = float(self.get_parameter("safety_underspeed_ratio").value)
        self.safety_underspeed_pwm = float(self.get_parameter("safety_underspeed_pwm").value)
        self.safety_underspeed_timeout_s = float(self.get_parameter("safety_underspeed_timeout_s").value)

        # Corrected real chassis assignment: M2 left, M1 right.
        self.left = MotorPI(
            "left/M2",
            M2_EN,
            M2_IN1,
            M2_IN2,
            M2_ENC_A,
            M2_ENC_B,
            M2_SIGN,
            M2_ENCODER_SIGN,
            self.encoder_counts_per_wheel_revolution,
            self.safety_window_s,
            self.safety_startup_grace_s,
        )
        self.right = MotorPI(
            "right/M1",
            M1_EN,
            M1_IN1,
            M1_IN2,
            M1_ENC_A,
            M1_ENC_B,
            M1_SIGN,
            M1_ENCODER_SIGN,
            self.encoder_counts_per_wheel_revolution,
            self.safety_window_s,
            self.safety_startup_grace_s,
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.command_lock = threading.RLock()
        self.last_odom_time = time.monotonic()
        self.last_cmd_time = time.monotonic()
        self.last_debug_time = time.monotonic()
        self.last_encoder_diag_time = self.last_debug_time
        self.control_timing = ControlTimingDiagnostics()
        self.last_control_encoder_age = float("inf")
        self.control_stop_event = threading.Event()
        self.control_thread = None
        self.control_thread_error = None

        factory = Device.pin_factory
        self.get_logger().info(
            "GPIO pin factory: "
            f"{type(factory).__module__}.{type(factory).__name__}"
        )
        self.get_logger().info(
            "Encoder timing diagnostics: "
            f"{'enabled' if self.left.timing_diagnostics.enabled else 'disabled'} "
            "(set R1_ENCODER_DIAGNOSTICS=1 to enable)"
        )
        self.get_logger().info(
            "Control timing diagnostics: "
            f"{'enabled' if self.control_timing.enabled else 'disabled'} "
            "(set R1_CONTROL_DIAGNOSTICS=1 to enable)"
        )
        self.last_fault_ignore_log_time = 0.0

        self.fault_latched = False
        self.fault_reason = ""
        self.fault_detail = ""

        self.bench_active = False
        self.bench_prev_moving = False
        self.bench_start_time = 0.0
        self.bench_file = None
        self.bench_writer = None
        self.bench_path = None

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = None
        self.csv_rows_since_flush = 0
        self.safety_log_queue = queue.Queue(maxsize=256)
        self.safety_log_stop_event = threading.Event()
        self.safety_log_thread = None
        self.safety_log_drop_count = 0
        self.open_safety_log()
        if self.csv_file:
            self.safety_log_thread = threading.Thread(
                target=self._safety_log_loop,
                name="motor_safety_log_writer",
                daemon=True,
            )
            self.safety_log_thread.start()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        fault_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fault_pub = self.create_publisher(String, "/motor_safety/fault", fault_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, "/cmd_vel", self.cmd_vel_stamped_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_unstamped", self.cmd_vel_cb, 10)

        self.get_logger().info("real_diffdrive_node started with dual-channel encoder safety")
        self.get_logger().info("Subscribing: /cmd_vel [TwistStamped], /cmd_vel_unstamped [Twist]")
        self.get_logger().info("Publishing: /odom, /motor_safety/fault")
        self.get_logger().info(
            f"Wheel separation: cmd={self.wheel_separation_cmd:.3f} m, "
            f"odom={self.wheel_separation_odom:.3f} m"
        )
        self.get_logger().info(
            f"Encoder CPR={self.encoder_counts_per_wheel_revolution:.1f} X4 transitions/wheel rev; "
            f"estimated max callback rate={2.0 * MAX_EXPECTED_ENCODER_EDGE_RATE_HZ:.0f} Hz total"
        )
        self.get_logger().info(
            "Encoder wiring: left A=BCM22 B=BCM26; right A=BCM17 B=BCM27"
        )
        self.get_logger().info(
            f"Motor safety: enabled={self.safety_enabled}, "
            f"window={self.safety_window_s:.2f}s, "
            f"startup_grace={self.safety_startup_grace_s:.2f}s, "
            f"no_pulse_timeout={self.safety_no_pulse_timeout_s:.2f}s"
        )
        self.get_logger().info(
            "Encoder direction safety latch="
            f"{'enabled' if self.safety_direction_check_enabled else 'disabled (diagnostic-only)'}"
        )
        if self.csv_path:
            self.get_logger().info(f"Motor safety CSV: {self.csv_path}")

        self.control_thread = threading.Thread(
            target=self._control_loop,
            name="motor_control_20hz",
            daemon=True,
        )
        self.control_thread.start()
        self.get_logger().info(
            "Dedicated monotonic-deadline motor control thread started at 20 Hz"
        )

    def open_safety_log(self):
        log_dir = os.path.expanduser(
            str(self.get_parameter("motor_safety_log_dir").value)
        )
        requested_name = str(
            self.get_parameter("motor_safety_log_name").value
        ).strip()

        try:
            os.makedirs(log_dir, exist_ok=True)

            if requested_name:
                # Keep the log inside log_dir even if a malformed name is
                # supplied. The batch uses the simple name motor_safety.csv.
                safe_name = os.path.basename(requested_name)
                if safe_name != requested_name:
                    self.get_logger().warning(
                        "motor_safety_log_name contained a path; "
                        f"using basename {safe_name!r}"
                    )
                self.csv_path = os.path.join(log_dir, safe_name)
            else:
                stamp = time.strftime("%Y%m%d_%H%M%S")
                self.csv_path = os.path.join(
                    log_dir, f"motor_safety_{stamp}.csv"
                )

            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                "wall_time_iso", "monotonic_s",
                "left_target_rpm", "left_measured_rpm", "left_pwm",
                "left_count_delta", "left_valid_transition_delta",
                "left_invalid_transition_delta", "left_count_total",
                "left_valid_transition_total", "left_invalid_transition_total",
                "left_invalid_transition_percentage", "left_a_edge_delta",
                "left_b_edge_delta", "left_a_edge_total", "left_b_edge_total",
                "left_a_edge_window", "left_b_edge_window", "left_a_age_s",
                "left_b_age_s", "left_decoded_direction",
                "right_target_rpm", "right_measured_rpm", "right_pwm",
                "right_count_delta", "right_valid_transition_delta",
                "right_invalid_transition_delta", "right_count_total",
                "right_valid_transition_total", "right_invalid_transition_total",
                "right_invalid_transition_percentage", "right_a_edge_delta",
                "right_b_edge_delta", "right_a_edge_total", "right_b_edge_total",
                "right_a_edge_window", "right_b_edge_window", "right_a_age_s",
                "right_b_age_s", "right_decoded_direction",
                "fault_latched", "fault_reason", "fault_detail",
            ])
            self.csv_file.flush()
        except OSError as exc:
            self.csv_file = None
            self.csv_writer = None
            self.csv_path = None
            self.get_logger().error(f"Could not open motor safety CSV: {exc}")

    def _safety_log_loop(self):
        """Write safety CSV rows outside the latency-sensitive control loop."""
        while (
            not self.safety_log_stop_event.is_set()
            or not self.safety_log_queue.empty()
        ):
            try:
                row = self.safety_log_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            self.csv_writer.writerow(row)
            self.csv_rows_since_flush += 1
            if self.csv_rows_since_flush >= 20 or row[-3]:
                self.csv_file.flush()
                self.csv_rows_since_flush = 0

    def rpm_to_mps(self, rpm):
        return (rpm / 60.0) * (2.0 * math.pi * self.wheel_radius)

    def mps_to_rpm(self, mps):
        return (mps / (2.0 * math.pi * self.wheel_radius)) * 60.0

    def cmd_vel_stamped_cb(self, msg):
        self.handle_twist(msg.twist)

    def cmd_vel_cb(self, msg):
        self.handle_twist(msg)

    def handle_twist(self, twist):
        now = time.monotonic()
        with self.command_lock:
            if self.fault_latched:
                if now - self.last_fault_ignore_log_time >= 2.0:
                    self.get_logger().error(
                        f"Ignoring cmd_vel because motor fault is latched: {self.fault_reason}"
                    )
                    self.last_fault_ignore_log_time = now
                # Safety output is owned by the control thread.  A ROS command
                # callback must never be the clock that writes the H-bridge.
                return

            v = float(twist.linear.x)
            omega = float(twist.angular.z)

            v_left = v - omega * self.wheel_separation_cmd / 2.0
            v_right = v + omega * self.wheel_separation_cmd / 2.0

            rpm_left = self.mps_to_rpm(v_left) * LEFT_RPM_SCALE
            rpm_right = self.mps_to_rpm(v_right) * RIGHT_RPM_SCALE

            rpm_left = apply_motor_deadband_rpm(rpm_left)
            rpm_right = apply_motor_deadband_rpm(rpm_right)

            self.left.set_target_rpm(rpm_left)
            self.right.set_target_rpm(rpm_right)
            self.last_cmd_time = now

        # Deliberately do not reset last_debug_time here. Nav2 publishes
        # continuously, and resetting it hid all moving-state debug output.

    def motor_fault_candidates(self, side, motor, state, metrics, now):
        candidates = []

        target = abs(motor.target_rpm)
        pwm = abs(state["command"])
        measured = abs(state["measured_rpm"])
        active = target >= self.safety_min_target_rpm and pwm >= self.safety_min_pwm

        if not active or now < motor.motion_grace_until:
            motor.underspeed_since = None
            motor.implausible_rpm_cycles = 0
            return candidates

        a_window = metrics["a_edge_window"]
        b_window = metrics["b_edge_window"]
        max_count = max(a_window, b_window)
        min_count = min(a_window, b_window)

        if max_count >= self.safety_min_window_pulses:
            ratio = min_count / max_count if max_count else 1.0
            if ratio < self.safety_channel_ratio_min:
                missing = "A" if a_window < b_window else "B"
                candidates.append({
                    "priority": 10,
                    "kind": "CHANNEL_LOSS",
                    "reason": f"{side.upper()}_ENCODER_{missing}_SIGNAL_LOSS",
                    "detail": (
                        f"window={self.safety_window_s:.2f}s "
                        f"A_edges={a_window} B_edges={b_window} ratio={ratio:.3f} "
                        f"valid={metrics['valid_transition_window']} "
                        f"invalid={metrics['invalid_transition_window']} "
                        f"target={motor.target_rpm:+.2f}rpm pwm={state['command']:+.3f}"
                    ),
                })

        both_stopped = (
            metrics["a_age_s"] >= self.safety_no_pulse_timeout_s
            and metrics["b_age_s"] >= self.safety_no_pulse_timeout_s
        )
        if both_stopped:
            candidates.append({
                "priority": 20,
                "kind": "NO_PULSES",
                "reason": f"{side.upper()}_WHEEL_FEEDBACK_LOSS_OR_STALL",
                "detail": (
                    f"A_age={metrics['a_age_s']:.3f}s B_age={metrics['b_age_s']:.3f}s "
                    f"target={motor.target_rpm:+.2f}rpm measured={state['measured_rpm']:+.2f}rpm "
                    f"pwm={state['command']:+.3f}"
                ),
            })

        # Encoder phase remains the sole source of measured sign.  This
        # optional comparison is deliberately disabled by default: the rolling
        # window can straddle a command reversal and falsely report the old
        # motion sign as a direction fault. Keep it available for controlled
        # bench validation, but do not make it part of normal production
        # operation until that validation is complete.
        if self.safety_direction_check_enabled:
            direction_samples = metrics["valid_transition_window"]
            if direction_samples >= self.safety_direction_min_samples:
                decoded_delta = metrics["count_window"]
                if motor.command_direction and decoded_delta * motor.command_direction < 0:
                    candidates.append({
                        "priority": 30,
                        "kind": "WRONG_DIRECTION",
                        "reason": f"{side.upper()}_ENCODER_DIRECTION_MISMATCH",
                        "detail": (
                            f"decoded_delta={decoded_delta} "
                            f"decoded_direction={metrics['decoded_direction']} "
                            f"command_direction={motor.command_direction} "
                            f"valid={direction_samples} target={motor.target_rpm:+.2f}rpm"
                        ),
                    })

        if measured > self.safety_max_encoder_rpm:
            motor.implausible_rpm_cycles += 1
        else:
            motor.implausible_rpm_cycles = 0

        if motor.implausible_rpm_cycles >= self.safety_implausible_rpm_cycles:
            candidates.append({
                "priority": 40,
                "kind": "IMPLAUSIBLE_RPM",
                "reason": f"{side.upper()}_ENCODER_IMPLAUSIBLE_JUMP",
                "detail": (
                    f"measured={state['measured_rpm']:+.2f}rpm "
                    f"limit={self.safety_max_encoder_rpm:.2f}rpm "
                    f"count_delta={state['count_delta']} "
                    f"invalid={state['invalid_transition_delta']}"
                ),
            })

        underspeed = (
            pwm >= self.safety_underspeed_pwm
            and measured < target * self.safety_underspeed_ratio
            and not both_stopped
        )
        if underspeed:
            if motor.underspeed_since is None:
                motor.underspeed_since = now
            elif now - motor.underspeed_since >= self.safety_underspeed_timeout_s:
                candidates.append({
                    "priority": 50,
                    "kind": "UNDERSPEED",
                    "reason": f"{side.upper()}_WHEEL_SEVERE_UNDERSPEED",
                    "detail": (
                        f"duration={now - motor.underspeed_since:.3f}s "
                        f"target={motor.target_rpm:+.2f}rpm measured={state['measured_rpm']:+.2f}rpm "
                        f"pwm={state['command']:+.3f}"
                    ),
                })
        else:
            motor.underspeed_since = None

        return candidates

    def latch_fault(self, reason, detail):
        with self.command_lock:
            if self.fault_latched:
                return

            self.fault_latched = True
            self.fault_reason = reason
            self.fault_detail = detail

            self.left.set_target_rpm(0.0)
            self.right.set_target_rpm(0.0)
            self.left.stop()
            self.right.stop()

            message = String()
            message.data = f"{reason}: {detail}"
            self.fault_pub.publish(message)

            self.get_logger().fatal("=" * 72)
            self.get_logger().fatal(f"MOTOR SAFETY FAULT LATCHED: {reason}")
            self.get_logger().fatal(detail)
            self.get_logger().fatal("Both motors stopped. Restart the node to clear the fault.")
            self.get_logger().fatal("=" * 72)

            if self.csv_file:
                self.csv_file.flush()

    def evaluate_safety(self, left_state, right_state, left_metrics, right_metrics, now):
        if not self.safety_enabled or self.fault_latched:
            return

        left_candidates = self.motor_fault_candidates(
            "left", self.left, left_state, left_metrics, now
        )
        right_candidates = self.motor_fault_candidates(
            "right", self.right, right_state, right_metrics, now
        )

        left_no_pulse = next((c for c in left_candidates if c["kind"] == "NO_PULSES"), None)
        right_no_pulse = next((c for c in right_candidates if c["kind"] == "NO_PULSES"), None)
        if left_no_pulse and right_no_pulse:
            self.latch_fault(
                "BOTH_WHEELS_FEEDBACK_LOSS_OR_STALL",
                f"left[{left_no_pulse['detail']}] right[{right_no_pulse['detail']}]",
            )
            return

        all_candidates = left_candidates + right_candidates
        if all_candidates:
            selected = min(all_candidates, key=lambda candidate: candidate["priority"])
            self.latch_fault(selected["reason"], selected["detail"])

    def write_safety_log(self, now, left_state, right_state, left_metrics, right_metrics):
        if not self.csv_writer:
            return

        row = [
            datetime.now().isoformat(timespec="milliseconds"),
            f"{now:.6f}",
            f"{self.left.target_rpm:.6f}",
            f"{left_state['measured_rpm']:.6f}",
            f"{left_state['command']:.6f}",
            int(left_state["count_delta"]),
            int(left_state["valid_transition_delta"]),
            int(left_state["invalid_transition_delta"]),
            int(left_state["signed_count"]),
            int(left_state["valid_transition_count"]),
            int(left_state["invalid_transition_count"]),
            f"{left_state['invalid_transition_percentage']:.6f}",
            int(left_state["a_edge_delta"]),
            int(left_state["b_edge_delta"]),
            int(left_state["a_edge_count"]),
            int(left_state["b_edge_count"]),
            int(left_metrics["a_edge_window"]),
            int(left_metrics["b_edge_window"]),
            f"{left_metrics['a_age_s']:.6f}",
            f"{left_metrics['b_age_s']:.6f}",
            int(left_metrics["decoded_direction"]),
            f"{self.right.target_rpm:.6f}",
            f"{right_state['measured_rpm']:.6f}",
            f"{right_state['command']:.6f}",
            int(right_state["count_delta"]),
            int(right_state["valid_transition_delta"]),
            int(right_state["invalid_transition_delta"]),
            int(right_state["signed_count"]),
            int(right_state["valid_transition_count"]),
            int(right_state["invalid_transition_count"]),
            f"{right_state['invalid_transition_percentage']:.6f}",
            int(right_state["a_edge_delta"]),
            int(right_state["b_edge_delta"]),
            int(right_state["a_edge_count"]),
            int(right_state["b_edge_count"]),
            int(right_metrics["a_edge_window"]),
            int(right_metrics["b_edge_window"]),
            f"{right_metrics['a_age_s']:.6f}",
            f"{right_metrics['b_age_s']:.6f}",
            int(right_metrics["decoded_direction"]),
            int(self.fault_latched),
            self.fault_reason,
            self.fault_detail,
        ]

        try:
            self.safety_log_queue.put_nowait(row)
        except queue.Full:
            self.safety_log_drop_count += 1

    def _control_loop(self):
        """Run the physical 20 Hz cycle on a monotonic deadline schedule."""
        next_deadline = time.monotonic()

        try:
            while not self.control_stop_event.is_set():
                remaining = next_deadline - time.monotonic()
                if remaining > 0.0:
                    if self.control_stop_event.wait(remaining):
                        break

                if self.control_stop_event.is_set():
                    break

                scheduled_start = next_deadline
                actual_start = time.monotonic()
                self.control_timing.cycle_start(scheduled_start, actual_start)

                try:
                    self.update()
                except Exception as exc:
                    self.control_thread_error = repr(exc)
                    self.get_logger().fatal(
                        f"Dedicated motor control thread failed: {exc!r}"
                    )
                    self.left.stop()
                    self.right.stop()
                    break
                finally:
                    actual_end = time.monotonic()
                    self.control_timing.cycle_end(
                        actual_start,
                        actual_end,
                        self.last_control_encoder_age,
                    )

                next_deadline += CONTROL_DT
                now = time.monotonic()
                if next_deadline <= now:
                    skipped = int((now - next_deadline) // CONTROL_DT) + 1
                    self.control_timing.skipped(skipped)
                    next_deadline += skipped * CONTROL_DT
        finally:
            self.left.stop()
            self.right.stop()

    def update(self):
        # The control thread owns the physical cycle.  The RLock also makes
        # command updates and safety state transitions atomic with respect to
        # the cycle without holding a lock across ROS publication callbacks.
        with self.command_lock:
            return self._update_locked()

    def _update_locked(self):
        now = time.monotonic()

        if self.fault_latched:
            self.left.stop()
            self.right.stop()
        elif now - self.last_cmd_time > self.cmd_timeout:
            self.left.set_target_rpm(0.0)
            self.right.set_target_rpm(0.0)

        left_state = self.left.update()
        right_state = self.right.update()
        left_metrics = self.left.window_metrics(now)
        right_metrics = self.right.window_metrics(now)
        if (
            abs(self.left.target_rpm) > 0.2
            or abs(self.right.target_rpm) > 0.2
        ):
            self.last_control_encoder_age = max(
                left_metrics["a_age_s"],
                left_metrics["b_age_s"],
                right_metrics["a_age_s"],
                right_metrics["b_age_s"],
            )
        else:
            # Do not turn the intentional post-command stop interval into a
            # false encoder-age maximum in the timing report.
            self.last_control_encoder_age = 0.0

        self.evaluate_safety(left_state, right_state, left_metrics, right_metrics, now)
        self.write_safety_log(now, left_state, right_state, left_metrics, right_metrics)

        moving_cmd = abs(self.left.target_rpm) > 0.2 or abs(self.right.target_rpm) > 0.2

        if PID_BENCH_ENABLE and moving_cmd and (not self.bench_prev_moving) and (not self.bench_active):
            bench_dir = os.path.expanduser(PID_BENCH_DIR)
            os.makedirs(bench_dir, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.bench_path = os.path.join(bench_dir, f"pid_step_{stamp}.csv")
            self.bench_file = open(self.bench_path, "w", newline="")
            self.bench_writer = csv.writer(self.bench_file)
            self.bench_writer.writerow([
                "t_s",
                "left_target_rpm", "left_measured_rpm", "left_error_rpm", "left_pwm_cmd", "left_delta_counts",
                "right_target_rpm", "right_measured_rpm", "right_error_rpm", "right_pwm_cmd", "right_delta_counts",
            ])
            self.bench_start_time = now
            self.bench_active = True
            self.get_logger().info(f"PID benchmark started: {self.bench_path}")

        if self.bench_active:
            t_s = now - self.bench_start_time
            self.bench_writer.writerow([
                f"{t_s:.6f}",
                f"{self.left.target_rpm:.6f}",
                f"{left_state['measured_rpm']:.6f}",
                f"{(self.left.target_rpm - left_state['measured_rpm']):.6f}",
                f"{left_state['command']:.6f}",
                int(left_state["count_delta"]),
                f"{self.right.target_rpm:.6f}",
                f"{right_state['measured_rpm']:.6f}",
                f"{(self.right.target_rpm - right_state['measured_rpm']):.6f}",
                f"{right_state['command']:.6f}",
                int(right_state["count_delta"]),
            ])

            if t_s >= PID_BENCH_DURATION_S:
                self.bench_file.flush()
                self.bench_file.close()
                self.get_logger().info(f"PID benchmark saved: {self.bench_path}")
                self.bench_active = False
                self.bench_file = None
                self.bench_writer = None

        self.bench_prev_moving = moving_cmd

        dt = now - self.last_odom_time
        self.last_odom_time = now
        if dt <= 0.0:
            return

        v_left = self.rpm_to_mps(left_state["measured_rpm"])
        v_right = self.rpm_to_mps(right_state["measured_rpm"])

        v = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.wheel_separation_odom

        dtheta = omega * dt
        mid_theta = self.theta + dtheta / 2.0

        self.x += v * math.cos(mid_theta) * dt
        self.y += v * math.sin(mid_theta) * dt
        self.theta = math.atan2(
            math.sin(self.theta + dtheta),
            math.cos(self.theta + dtheta),
        )

        self.publish_odom(v, omega)

    def publish_odom(self, v, omega):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = omega

        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.10
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[35] = 0.10
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def stop_and_close(self):
        self.control_stop_event.set()
        if (
            self.control_thread
            and self.control_thread is not threading.current_thread()
        ):
            self.control_thread.join(timeout=2.0)

        if self.left.timing_diagnostics.enabled or self.control_timing.enabled:
            self.get_logger().info(
                "ENCODER_TIMING_FINAL "
                f"{self.control_timing.summary()} | "
                f"{self.left.timing_diagnostics.summary()} | "
                f"{self.right.timing_diagnostics.summary()}"
            )
        self.left.stop()
        self.right.stop()

        self.safety_log_stop_event.set()
        if (
            self.safety_log_thread
            and self.safety_log_thread is not threading.current_thread()
        ):
            self.safety_log_thread.join(timeout=2.0)

        if self.bench_file:
            self.bench_file.flush()
            self.bench_file.close()
            self.bench_file = None

        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None

        self.left.close()
        self.right.close()


def main(args=None):
    rclpy.init(args=args)
    node = RealDiffDriveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Stopping motors")
        node.stop_and_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
