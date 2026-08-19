#!/usr/bin/env python3

"""
Direct 100% PWM motor test for the real differential-drive robot.

This intentionally bypasses:
- ROS 2
- cmd_vel
- PI/PID speed control
- MAX_TARGET_RPM
- deadband compensation
- acceleration limiting

It drives the selected motor(s) at exactly 100% PWM and reports
encoder-derived RPM once per second.

IMPORTANT:
Run only when real_diffdrive_node is stopped.
"""

import argparse
import signal
import time
from threading import Event, Lock

from gpiozero import Device, DigitalInputDevice, DigitalOutputDevice, PWMOutputDevice


# Existing real robot GPIO configuration, BCM numbering.

# Right motor: M1 / L298N channel A
M1_EN = 18
M1_IN1 = 23
M1_IN2 = 24
M1_ENC_A = 17
M1_SIGN = 1

# Left motor: M2 / L298N channel B
M2_EN = 13
M2_IN1 = 5
M2_IN2 = 6
M2_ENC_A = 22
M2_SIGN = -1

PWM_FREQUENCY_HZ = 1000
COUNTS_PER_REV = 1160.0

stop_requested = Event()


class DirectMotor:
    def __init__(
        self,
        name: str,
        enable_pin: int,
        in1_pin: int,
        in2_pin: int,
        encoder_pin: int,
        motor_sign: int,
    ) -> None:
        self.name = name
        self.motor_sign = motor_sign

        self.pwm = PWMOutputDevice(
            enable_pin,
            frequency=PWM_FREQUENCY_HZ,
            initial_value=0.0,
        )
        self.in1 = DigitalOutputDevice(in1_pin, initial_value=False)
        self.in2 = DigitalOutputDevice(in2_pin, initial_value=False)

        self.encoder = DigitalInputDevice(encoder_pin, pull_up=True)
        self.encoder.when_activated = self._encoder_tick

        self._count = 0
        self._count_lock = Lock()

    def _encoder_tick(self) -> None:
        with self._count_lock:
            self._count += 1

    def encoder_count(self) -> int:
        with self._count_lock:
            return self._count

    def drive_full_power(self, logical_direction: int) -> None:
        """
        logical_direction:
            +1 = robot-forward wheel direction
            -1 = robot-reverse wheel direction
        """
        if logical_direction not in (-1, 1):
            raise ValueError("logical_direction must be +1 or -1")

        physical_command = logical_direction * self.motor_sign

        # Set direction before enabling 100% PWM.
        self.pwm.value = 0.0

        if physical_command > 0:
            self.in1.on()
            self.in2.off()
        else:
            self.in1.off()
            self.in2.on()

        # Direct full output: no controller and no speed cap.
        self.pwm.value = 1.0

    def stop(self) -> None:
        # Remove PWM first, then remove direction signals.
        self.pwm.value = 0.0
        self.in1.off()
        self.in2.off()

    def close(self) -> None:
        self.stop()

        # Disable callbacks before closing the lgpio-backed input.
        time.sleep(0.05)
        self.encoder.close()

        self.pwm.close()
        self.in1.close()
        self.in2.close()


def handle_signal(signum, frame) -> None:
    del signum, frame
    stop_requested.set()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the robot motors at direct 100% PWM."
    )
    parser.add_argument(
        "--motor",
        choices=("both", "left", "right"),
        default="both",
        help="Motor selection. Default: both",
    )
    parser.add_argument(
        "--direction",
        choices=("forward", "reverse"),
        default="forward",
        help="Logical robot direction. Default: forward",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Maximum powered duration in seconds. Default: 10",
    )
    parser.add_argument(
        "--countdown",
        type=int,
        default=3,
        help="Countdown before startup. Default: 3 seconds",
    )

    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be greater than zero")

    if args.countdown < 0:
        parser.error("--countdown cannot be negative")

    return args


def main() -> None:
    args = parse_arguments()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    left = DirectMotor(
        name="left/M2",
        enable_pin=M2_EN,
        in1_pin=M2_IN1,
        in2_pin=M2_IN2,
        encoder_pin=M2_ENC_A,
        motor_sign=M2_SIGN,
    )

    right = DirectMotor(
        name="right/M1",
        enable_pin=M1_EN,
        in1_pin=M1_IN1,
        in2_pin=M1_IN2,
        encoder_pin=M1_ENC_A,
        motor_sign=M1_SIGN,
    )

    all_motors = [left, right]

    if args.motor == "both":
        selected_motors = [left, right]
    elif args.motor == "left":
        selected_motors = [left]
    else:
        selected_motors = [right]

    logical_direction = 1 if args.direction == "forward" else -1

    try:
        print()
        print("DIRECT MOTOR TEST")
        print("=================")
        print(f"Motor selection : {args.motor}")
        print(f"Direction       : {args.direction}")
        print("PWM duty        : 100%")
        print(f"Duration limit  : {args.duration:.1f} seconds")
        print()
        print("PID, ROS commands, and RPM limiting are NOT active.")
        print("Press Ctrl+C at any time to stop.")
        print()

        for remaining in range(args.countdown, 0, -1):
            if stop_requested.is_set():
                return

            print(f"Starting in {remaining}...")
            time.sleep(1.0)

        previous_time = time.monotonic()
        previous_counts = {
            motor.name: motor.encoder_count()
            for motor in selected_motors
        }

        # Both motors are enabled as close together as Python allows.
        test_start = time.monotonic()

        for motor in selected_motors:
            motor.drive_full_power(logical_direction)

        print("100% PWM ACTIVE")

        while not stop_requested.is_set():
            elapsed = time.monotonic() - test_start

            if elapsed >= args.duration:
                break

            remaining_sleep = min(1.0, args.duration - elapsed)

            if remaining_sleep > 0.0:
                stop_requested.wait(remaining_sleep)

            now = time.monotonic()
            dt = now - previous_time

            if dt <= 0.0:
                continue

            rpm_parts = []

            for motor in selected_motors:
                current_count = motor.encoder_count()
                delta_count = current_count - previous_counts[motor.name]

                rpm = (
                    (delta_count / COUNTS_PER_REV)
                    / dt
                    * 60.0
                )

                rpm_parts.append(
                    f"{motor.name}: {rpm:6.2f} RPM "
                    f"({delta_count} counts)"
                )

                previous_counts[motor.name] = current_count

            previous_time = now
            elapsed = now - test_start

            print(
                f"t={elapsed:5.2f}s | "
                + " | ".join(rpm_parts)
            )

    finally:
        # Stop both motors even if only one was selected.
        for motor in all_motors:
            try:
                motor.stop()
            except Exception:
                pass

        print("PWM OFF — motors stopped")

        for motor in all_motors:
            try:
                motor.close()
            except Exception:
                pass

        # Shut down the shared GPIO backend and its notification thread.
        time.sleep(0.10)

        try:
            if Device.pin_factory is not None:
                Device.pin_factory.close()
        except Exception:
            pass

        time.sleep(0.10)


if __name__ == "__main__":
    main()
