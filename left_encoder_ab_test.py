#!/usr/bin/env python3

import threading
import time

from gpiozero import DigitalInputDevice

A_PIN = 22   # Left encoder A, physical pin 15
B_PIN = 26   # Left encoder B, physical pin 37

# Valid quadrature transitions. Direction sign is arbitrary;
# it only needs to remain consistent for one rotation direction.
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

lock = threading.Lock()

stats = {
    "a_rise": 0,
    "a_fall": 0,
    "b_rise": 0,
    "b_fall": 0,
    "valid_steps": 0,
    "invalid": 0,
    "duplicate": 0,
    "position": 0,
}

previous_state = (int(a.value) << 1) | int(b.value)


def handle_edge(channel: str, rising: bool) -> None:
    global previous_state

    with lock:
        key = f"{channel.lower()}_{'rise' if rising else 'fall'}"
        stats[key] += 1

        new_state = (int(a.value) << 1) | int(b.value)

        if new_state == previous_state:
            stats["duplicate"] += 1
            return

        delta = TRANSITION_DELTA.get((previous_state, new_state))

        if delta is None:
            stats["invalid"] += 1
        else:
            stats["valid_steps"] += 1
            stats["position"] += delta

        previous_state = new_state


a.when_activated = lambda: handle_edge("A", True)
a.when_deactivated = lambda: handle_edge("A", False)
b.when_activated = lambda: handle_edge("B", True)
b.when_deactivated = lambda: handle_edge("B", False)

print("Left encoder A/B test")
print("A = BCM22 / physical pin 15")
print("B = BCM26 / physical pin 37")
print()
print("Test sequence:")
print("  1. Keep wheel completely still for 5 seconds.")
print("  2. Turn wheel slowly by hand in one direction.")
print("  3. Turn wheel slowly in the opposite direction.")
print("  4. Press Ctrl+C.")
print()

previous_snapshot = stats.copy()

try:
    while True:
        time.sleep(1.0)

        with lock:
            current = stats.copy()
            state = (int(a.value) << 1) | int(b.value)

        da = (
            current["a_rise"] + current["a_fall"]
            - previous_snapshot["a_rise"]
            - previous_snapshot["a_fall"]
        )
        db = (
            current["b_rise"] + current["b_fall"]
            - previous_snapshot["b_rise"]
            - previous_snapshot["b_fall"]
        )
        dinvalid = current["invalid"] - previous_snapshot["invalid"]
        dposition = current["position"] - previous_snapshot["position"]

        print(
            f"A={int(a.value)} B={int(b.value)} state={state:02b} | "
            f"edges/s A={da:4d} B={db:4d} | "
            f"position change={dposition:+5d} | "
            f"invalid/s={dinvalid:3d} | "
            f"totals: position={current['position']:+7d}, "
            f"invalid={current['invalid']}, "
            f"duplicate={current['duplicate']}"
        )

        previous_snapshot = current

except KeyboardInterrupt:
    print("\nFinal statistics:")

    with lock:
        for key, value in stats.items():
            print(f"  {key}: {value}")

finally:
    a.close()
    b.close()
