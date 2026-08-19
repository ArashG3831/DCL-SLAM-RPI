#!/usr/bin/env python3

import time
from threading import Lock

from gpiozero import DigitalInputDevice

# Left motor M2
LEFT_A = 22
LEFT_B = 26

# Right motor M1
RIGHT_A = 17
RIGHT_B = 27

lock = Lock()

pins = {
    "left_a": DigitalInputDevice(LEFT_A, pull_up=True),
    "left_b": DigitalInputDevice(LEFT_B, pull_up=True),
    "right_a": DigitalInputDevice(RIGHT_A, pull_up=True),
    "right_b": DigitalInputDevice(RIGHT_B, pull_up=True),
}

counts = {
    "left_a": 0,
    "left_b": 0,
    "right_a": 0,
    "right_b": 0,
}

# Direction snapshots taken only when A rises.
a_rise_b_high = {
    "left": 0,
    "right": 0,
}

a_rise_b_low = {
    "left": 0,
    "right": 0,
}


def count_edge(name):
    with lock:
        counts[name] += 1


def left_a_rise():
    with lock:
        counts["left_a"] += 1

        if pins["left_b"].value:
            a_rise_b_high["left"] += 1
        else:
            a_rise_b_low["left"] += 1


def right_a_rise():
    with lock:
        counts["right_a"] += 1

        if pins["right_b"].value:
            a_rise_b_high["right"] += 1
        else:
            a_rise_b_low["right"] += 1


pins["left_a"].when_activated = left_a_rise
pins["left_b"].when_activated = lambda: count_edge("left_b")

pins["right_a"].when_activated = right_a_rise
pins["right_b"].when_activated = lambda: count_edge("right_b")

previous = counts.copy()

print("Dual encoder rising-edge test")
print("Lift both wheels.")
print("Rotate each wheel by hand in both directions.")
print("Then stop both wheels and verify all rates return to zero.")
print("Press Ctrl+C to finish.\n")

try:
    while True:
        time.sleep(1.0)

        with lock:
            current = counts.copy()
            high = a_rise_b_high.copy()
            low = a_rise_b_low.copy()

        delta = {
            key: current[key] - previous[key]
            for key in current
        }

        print(
            f"LEFT  A/s={delta['left_a']:4d} "
            f"B/s={delta['left_b']:4d} "
            f"total A={current['left_a']:6d} "
            f"B={current['left_b']:6d} "
            f"A↑B0={low['left']:6d} "
            f"A↑B1={high['left']:6d}"
        )

        print(
            f"RIGHT A/s={delta['right_a']:4d} "
            f"B/s={delta['right_b']:4d} "
            f"total A={current['right_a']:6d} "
            f"B={current['right_b']:6d} "
            f"A↑B0={low['right']:6d} "
            f"A↑B1={high['right']:6d}"
        )

        print()

        previous = current

except KeyboardInterrupt:
    print("\nFinal counts:")

    with lock:
        print(counts)
        print("A-rise with B low:", a_rise_b_low)
        print("A-rise with B high:", a_rise_b_high)

finally:
    for pin in pins.values():
        pin.close()
