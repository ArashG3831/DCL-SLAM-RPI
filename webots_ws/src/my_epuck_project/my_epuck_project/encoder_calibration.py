#!/usr/bin/env python3

"""Non-driving manual full-quadrature wheel calibration utility."""

import argparse
import os
import sys

from gpiozero import DigitalInputDevice

from .quadrature_decoder import QuadratureDecoder


SIDES = {
    "left": (22, 26),
    "right": (17, 27),
}


def motor_node_pids():
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join("/proc", entry, "cmdline"), "rb") as cmdline:
                command = cmdline.read().replace(b"\0", b" ").decode(errors="replace")
        except (FileNotFoundError, PermissionError, OSError):
            continue
        if "real_diffdrive_node" in command and int(entry) != os.getpid():
            pids.append(int(entry))
    return pids


def capture(side, direction, revolutions):
    enc_a_pin, enc_b_pin = SIDES[side]
    enc_a = DigitalInputDevice(enc_a_pin, pull_up=True)
    enc_b = DigitalInputDevice(enc_b_pin, pull_up=True)
    initial_state = (int(enc_a.value) << 1) | int(enc_b.value)
    decoder = QuadratureDecoder(initial_state=initial_state, encoder_sign=1)

    enc_a.pin.edges = "both"
    enc_b.pin.edges = "both"
    enc_a.pin.bounce = None
    enc_b.pin.bounce = None
    # gpiozero stores pin.when_changed handlers as weak references. Do not
    # assign temporary lambdas here: they can be collected immediately and
    # make a real encoder appear to produce zero edges. Keep named callback
    # objects alive for the whole capture.
    def encoder_a_changed(ticks, state):
        decoder.process_edge("A", state, ticks)

    def encoder_b_changed(ticks, state):
        decoder.process_edge("B", state, ticks)

    callbacks = (encoder_a_changed, encoder_b_changed)
    enc_a.pin.when_changed = callbacks[0]
    enc_b.pin.when_changed = callbacks[1]

    try:
        print(f"{side.upper()} encoder A=GPIO{enc_a_pin}, B=GPIO{enc_b_pin}")
        print("This utility opens encoder inputs only; it does not open motor GPIOs.")
        print("Confirm real_diffdrive_node is stopped and the robot is safe to rotate manually.")
        input("Press Enter when the wheel reference mark is ready and the wheel is still...")
        decoder.reset_counts()
        print(
            f"Now rotate exactly {revolutions} complete wheel revolutions "
            f"in the physical {direction} direction, then stop."
        )
        input("Press Enter after the marked wheel has completed the revolutions...")
        snapshot = decoder.snapshot()
    finally:
        enc_a.pin.when_changed = None
        enc_b.pin.when_changed = None
        enc_a.close()
        enc_b.close()

    total = snapshot["valid_transition_count"] + snapshot["invalid_transition_count"]
    print(f"side={side}")
    print(f"direction={direction}")
    print(f"signed_quadrature_transitions={snapshot['count']}")
    print(f"absolute_quadrature_transitions={abs(snapshot['count'])}")
    print(f"measured_cpr={abs(snapshot['count']) / revolutions:.6f}")
    print(f"valid_transitions={snapshot['valid_transition_count']}")
    print(f"invalid_transitions={snapshot['invalid_transition_count']}")
    print(
        "invalid_transition_percentage="
        f"{100.0 * snapshot['invalid_transition_count'] / total if total else 0.0:.6f}"
    )
    print(f"a_edges={snapshot['a_edge_count']}")
    print(f"b_edges={snapshot['b_edge_count']}")
    print(f"final_state={snapshot['current_state']:02b}")
    return snapshot


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=sorted(SIDES), required=True)
    parser.add_argument("--direction", choices=("forward", "reverse"), required=True)
    parser.add_argument("--revolutions", type=int, default=10)
    args = parser.parse_args(argv)
    if args.revolutions <= 0:
        parser.error("--revolutions must be positive")

    pids = motor_node_pids()
    if pids:
        print(
            "Refusing calibration: real_diffdrive_node is running "
            f"(PID(s): {','.join(map(str, pids))}). Stop it first.",
            file=sys.stderr,
        )
        return 2

    try:
        capture(args.side, args.direction, args.revolutions)
    except KeyboardInterrupt:
        print("\nCalibration interrupted.", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
