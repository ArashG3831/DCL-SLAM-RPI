#!/usr/bin/env python3

import time
from threading import Lock

from gpiozero import (
    DigitalInputDevice,
    DigitalOutputDevice,
    PWMOutputDevice,
)

PWM_FREQ = 1000
TEST_PWM = 0.30
PHASE_DURATION_S = 4.0

# Left motor: M2
LEFT_EN = 13
LEFT_IN1 = 5
LEFT_IN2 = 6
LEFT_SIGN = -1
LEFT_A = 22
LEFT_B = 26

# Right motor: M1
RIGHT_EN = 18
RIGHT_IN1 = 23
RIGHT_IN2 = 24
RIGHT_SIGN = +1
RIGHT_A = 17
RIGHT_B = 27

lock = Lock()

encoders = {
    "left_a": DigitalInputDevice(LEFT_A, pull_up=True),
    "left_b": DigitalInputDevice(LEFT_B, pull_up=True),
    "right_a": DigitalInputDevice(RIGHT_A, pull_up=True),
    "right_b": DigitalInputDevice(RIGHT_B, pull_up=True),
}

counts = {
    "left_a": 0,
    "left_b": 0,
    "left_a_b0": 0,
    "left_a_b1": 0,
    "right_a": 0,
    "right_b": 0,
    "right_a_b0": 0,
    "right_a_b1": 0,
}


def left_a_rise():
    with lock:
        counts["left_a"] += 1
        if encoders["left_b"].value:
            counts["left_a_b1"] += 1
        else:
            counts["left_a_b0"] += 1


def left_b_rise():
    with lock:
        counts["left_b"] += 1


def right_a_rise():
    with lock:
        counts["right_a"] += 1
        if encoders["right_b"].value:
            counts["right_a_b1"] += 1
        else:
            counts["right_a_b0"] += 1


def right_b_rise():
    with lock:
        counts["right_b"] += 1


encoders["left_a"].when_activated = left_a_rise
encoders["left_b"].when_activated = left_b_rise
encoders["right_a"].when_activated = right_a_rise
encoders["right_b"].when_activated = right_b_rise

left_pwm = PWMOutputDevice(
    LEFT_EN,
    frequency=PWM_FREQ,
    initial_value=0.0,
)
left_in1 = DigitalOutputDevice(LEFT_IN1)
left_in2 = DigitalOutputDevice(LEFT_IN2)

right_pwm = PWMOutputDevice(
    RIGHT_EN,
    frequency=PWM_FREQ,
    initial_value=0.0,
)
right_in1 = DigitalOutputDevice(RIGHT_IN1)
right_in2 = DigitalOutputDevice(RIGHT_IN2)


def snapshot():
    with lock:
        return counts.copy()


def set_motor(pwm, in1, in2, logical_direction, motor_sign):
    electrical_direction = logical_direction * motor_sign

    if electrical_direction > 0:
        in1.on()
        in2.off()
    elif electrical_direction < 0:
        in1.off()
        in2.on()
    else:
        pwm.value = 0.0
        in1.off()
        in2.off()
        return

    pwm.value = TEST_PWM


def drive_both(logical_direction):
    set_motor(
        left_pwm,
        left_in1,
        left_in2,
        logical_direction,
        LEFT_SIGN,
    )
    set_motor(
        right_pwm,
        right_in1,
        right_in2,
        logical_direction,
        RIGHT_SIGN,
    )


def stop_all():
    left_pwm.value = 0.0
    right_pwm.value = 0.0

    left_in1.off()
    left_in2.off()
    right_in1.off()
    right_in2.off()


def delta(current, previous, key):
    return current[key] - previous[key]


def wheel_summary(name, current, start):
    a = delta(current, start, f"{name}_a")
    b = delta(current, start, f"{name}_b")
    b0 = delta(current, start, f"{name}_a_b0")
    b1 = delta(current, start, f"{name}_a_b1")

    total_direction_samples = b0 + b1
    agreement = (
        max(b0, b1) / total_direction_samples
        if total_direction_samples
        else 0.0
    )

    mismatch = (
        abs(a - b) / max(a, b)
        if max(a, b) > 0
        else 0.0
    )

    dominant_b = 0 if b0 > b1 else 1 if b1 > b0 else None

    return {
        "a": a,
        "b": b,
        "b0": b0,
        "b1": b1,
        "agreement": agreement,
        "mismatch": mismatch,
        "dominant_b": dominant_b,
    }


def run_phase(name, direction):
    print()
    print("=" * 64)
    print(name)
    print("=" * 64)

    phase_start = snapshot()
    previous = phase_start

    drive_both(direction)
    start_time = time.monotonic()

    while time.monotonic() - start_time < PHASE_DURATION_S:
        time.sleep(0.5)
        current = snapshot()

        print(
            "LEFT  "
            f"A={delta(current, previous, 'left_a'):4d} "
            f"B={delta(current, previous, 'left_b'):4d} | "
            "RIGHT "
            f"A={delta(current, previous, 'right_a'):4d} "
            f"B={delta(current, previous, 'right_b'):4d}"
        )

        previous = current

    stop_all()

    # Allow wheels to coast to a complete stop.
    time.sleep(1.0)

    current = snapshot()
    left = wheel_summary("left", current, phase_start)
    right = wheel_summary("right", current, phase_start)

    print()
    print(
        "LEFT summary:  "
        f"A={left['a']} B={left['b']} "
        f"A↑B0={left['b0']} A↑B1={left['b1']} "
        f"direction agreement={left['agreement'] * 100:.1f}% "
        f"A/B mismatch={left['mismatch'] * 100:.2f}%"
    )

    print(
        "RIGHT summary: "
        f"A={right['a']} B={right['b']} "
        f"A↑B0={right['b0']} A↑B1={right['b1']} "
        f"direction agreement={right['agreement'] * 100:.1f}% "
        f"A/B mismatch={right['mismatch'] * 100:.2f}%"
    )

    return left, right


def stationary_test():
    print()
    print("=" * 64)
    print("STATIONARY NOISE TEST")
    print("=" * 64)

    stop_all()
    time.sleep(1.0)

    before = snapshot()
    time.sleep(3.0)
    after = snapshot()

    left_a = delta(after, before, "left_a")
    left_b = delta(after, before, "left_b")
    right_a = delta(after, before, "right_a")
    right_b = delta(after, before, "right_b")

    print(f"LEFT stationary edges:  A={left_a}, B={left_b}")
    print(f"RIGHT stationary edges: A={right_a}, B={right_b}")

    return left_a, left_b, right_a, right_b


try:
    print("DUAL POWERED ENCODER TEST")
    print()
    print("Both wheels MUST be lifted.")
    print(f"PWM duty: {TEST_PWM:.2f}")
    print("Press Ctrl+C at any time for immediate motor stop.")
    print()
    time.sleep(2.0)

    forward_left, forward_right = run_phase(
        "PHASE 1: BOTH WHEELS FORWARD",
        +1,
    )

    time.sleep(1.0)

    reverse_left, reverse_right = run_phase(
        "PHASE 2: BOTH WHEELS REVERSE",
        -1,
    )

    stationary = stationary_test()

    print()
    print("=" * 64)
    print("FINAL AUTOMATIC CHECK")
    print("=" * 64)

    tests_passed = True

    for wheel_name, forward, reverse in (
        ("LEFT", forward_left, reverse_left),
        ("RIGHT", forward_right, reverse_right),
    ):
        if forward["a"] == 0 or forward["b"] == 0:
            print(f"FAIL: {wheel_name} produced no forward encoder pulses.")
            tests_passed = False

        if reverse["a"] == 0 or reverse["b"] == 0:
            print(f"FAIL: {wheel_name} produced no reverse encoder pulses.")
            tests_passed = False

        if forward["mismatch"] > 0.03:
            print(f"FAIL: {wheel_name} forward A/B mismatch exceeds 3%.")
            tests_passed = False

        if reverse["mismatch"] > 0.03:
            print(f"FAIL: {wheel_name} reverse A/B mismatch exceeds 3%.")
            tests_passed = False

        if forward["agreement"] < 0.90:
            print(
                f"FAIL: {wheel_name} forward direction agreement "
                "is below 90%."
            )
            tests_passed = False

        if reverse["agreement"] < 0.90:
            print(
                f"FAIL: {wheel_name} reverse direction agreement "
                "is below 90%."
            )
            tests_passed = False

        if forward["dominant_b"] == reverse["dominant_b"]:
            print(
                f"FAIL: {wheel_name} encoder direction did not reverse."
            )
            tests_passed = False

    if any(stationary):
        print("FAIL: encoder activity detected while stationary.")
        tests_passed = False

    if tests_passed:
        print("PASS: both dual-channel encoders are healthy under power.")
    else:
        print("FAIL: inspect the messages above before integration.")

except KeyboardInterrupt:
    print("\nInterrupted — stopping both motors immediately.")

finally:
    stop_all()

    left_pwm.close()
    left_in1.close()
    left_in2.close()

    right_pwm.close()
    right_in1.close()
    right_in2.close()

    for encoder in encoders.values():
        encoder.close()
