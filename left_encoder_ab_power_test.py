#!/usr/bin/env python3

import threading
import time

from gpiozero import (
    DigitalInputDevice,
    DigitalOutputDevice,
    PWMOutputDevice,
)

# Left motor M2
EN_PIN = 13
IN1_PIN = 5
IN2_PIN = 6

A_PIN = 22
B_PIN = 26

PWM_FREQ = 1000
TEST_PWM = 0.35

TRANSITION_DELTA = {
    (0b00, 0b01): +1,
    (0b01, 0b11): +1,
    (0b11, 0b10): +1,
    (0b10, 0b00): +1,

    (0b00, 0b10): -1,
    (0b10, 0b11): -1,
    (0b11, 0b01): -1,
    (0b01, 0b00): -1,
}

a = DigitalInputDevice(A_PIN, pull_up=True)
b = DigitalInputDevice(B_PIN, pull_up=True)

pwm = PWMOutputDevice(
    EN_PIN,
    frequency=PWM_FREQ,
    initial_value=0.0,
)
in1 = DigitalOutputDevice(IN1_PIN)
in2 = DigitalOutputDevice(IN2_PIN)

lock = threading.Lock()

stats = {
    "a_edges": 0,
    "b_edges": 0,
    "valid": 0,
    "invalid": 0,
    "duplicate": 0,
    "position": 0,
}

previous_state = (int(a.value) << 1) | int(b.value)


def edge(channel: str) -> None:
    global previous_state

    with lock:
        stats[f"{channel}_edges"] += 1

        new_state = (int(a.value) << 1) | int(b.value)

        if new_state == previous_state:
            stats["duplicate"] += 1
            return

        delta = TRANSITION_DELTA.get((previous_state, new_state))

        if delta is None:
            stats["invalid"] += 1
        else:
            stats["valid"] += 1
            stats["position"] += delta

        previous_state = new_state


a.when_activated = lambda: edge("a")
a.when_deactivated = lambda: edge("a")
b.when_activated = lambda: edge("b")
b.when_deactivated = lambda: edge("b")


def stop() -> None:
    pwm.value = 0.0
    in1.off()
    in2.off()


def drive(direction: int) -> None:
    # Same M2 polarity convention as the real motor node.
    electrical_direction = -direction

    if electrical_direction > 0:
        in1.on()
        in2.off()
    else:
        in1.off()
        in2.on()

    pwm.value = TEST_PWM


def snapshot():
    with lock:
        return stats.copy()


def run_phase(name: str, direction: int, duration: float) -> None:
    print(f"\n{name}")

    before = snapshot()
    drive(direction)
    start = time.monotonic()

    while time.monotonic() - start < duration:
        time.sleep(0.5)

        current = snapshot()

        da = current["a_edges"] - before["a_edges"]
        db = current["b_edges"] - before["b_edges"]
        dv = current["valid"] - before["valid"]
        di = current["invalid"] - before["invalid"]
        dp = current["position"] - before["position"]

        print(
            f"A edges={da:5d}  B edges={db:5d}  "
            f"valid={dv:5d}  invalid={di:3d}  "
            f"position={dp:+6d}"
        )

    stop()
    time.sleep(1.0)


try:
    print("LEFT ENCODER POWERED A/B TEST")
    print("Robot wheels must be lifted.")
    print("Press Ctrl+C for immediate stop.")

    time.sleep(2.0)

    run_phase("Direction 1", +1, 3.0)
    run_phase("Direction 2", -1, 3.0)

    print("\nFinal:", snapshot())

finally:
    stop()

    pwm.close()
    in1.close()
    in2.close()
    a.close()
    b.close()
