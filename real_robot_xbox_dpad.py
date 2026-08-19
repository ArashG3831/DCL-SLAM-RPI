#!/usr/bin/env python3

import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist

from evdev import InputDevice, list_devices, ecodes


# Robot geometry
WHEEL_RADIUS = 0.0325
WHEEL_SEPARATION = 0.204

# Forward/backward still allowed to be strong
FORWARD_RPM = 45.0

# Spins are deliberately slower for SLAM
SPIN_MAX_RADPS = 0.25

# Rest between command direction changes
TRANSITION_REST_S = 0.50

# If no D-pad event is received for this long, stop
COMMAND_TIMEOUT_S = 0.35

PUB_RATE = 20.0


def rpm_to_mps(rpm):
    return (2.0 * math.pi * WHEEL_RADIUS) * (rpm / 60.0)


def make_twist(v=0.0, w=0.0):
    msg = Twist()
    msg.linear.x = float(v)
    msg.angular.z = float(w)
    return msg


def find_xbox_controller():
    devices = [InputDevice(path) for path in list_devices()]

    print("Detected input devices:")
    for dev in devices:
        print(f"  {dev.path}: {dev.name}")

    keywords = ["xbox", "x-box", "microsoft", "wireless controller", "gamepad"]

    for dev in devices:
        name = dev.name.lower()
        if any(k in name for k in keywords):
            print(f"\nUsing controller: {dev.path} ({dev.name})")
            return dev

    raise RuntimeError("No Xbox/gamepad-like controller found. Pair the controller to the Pi first.")


def command_from_dpad(hat_x, hat_y):
    v = rpm_to_mps(FORWARD_RPM)

    # Linux D-pad convention is usually:
    # ABS_HAT0Y = -1 up, +1 down
    # ABS_HAT0X = -1 left, +1 right

    if hat_y == -1:
        return "forward", make_twist(+v, 0.0)

    if hat_y == +1:
        return "backward", make_twist(-v, 0.0)

    if hat_x == -1:
        return "spin_left", make_twist(0.0, +SPIN_MAX_RADPS)

    if hat_x == +1:
        return "spin_right", make_twist(0.0, -SPIN_MAX_RADPS)

    return "stop", make_twist(0.0, 0.0)


def main():
    rclpy.init()
    node = rclpy.create_node("real_robot_xbox_dpad")
    pub = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)

    dev = find_xbox_controller()
    dev.grab()

    print()
    print("Xbox D-pad control active:")
    print("  D-pad up    = forward, 45 RPM")
    print("  D-pad down  = backward, 45 RPM")
    print("  D-pad left  = slow spin left / CCW")
    print("  D-pad right = slow spin right / CW")
    print()
    print(f"Forward speed: {rpm_to_mps(FORWARD_RPM):.3f} m/s")
    print(f"Spin speed:    {SPIN_MAX_RADPS:.3f} rad/s")
    print(f"Rest on direction change: {TRANSITION_REST_S:.2f} s")
    print("Press Ctrl+C to quit.")
    print()

    hat_x = 0
    hat_y = 0

    active_name = "stop"
    active_cmd = make_twist()
    last_motion_name = "stop"
    last_event_time = time.time()
    rest_until = 0.0

    dt = 1.0 / PUB_RATE

    try:
        while rclpy.ok():
            now = time.time()

            # Read all pending controller events without blocking.
            while True:
                event = dev.read_one()
                if event is None:
                    break

                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_HAT0X:
                        hat_x = int(event.value)
                        last_event_time = now

                    elif event.code == ecodes.ABS_HAT0Y:
                        hat_y = int(event.value)
                        last_event_time = now

                    new_name, new_cmd = command_from_dpad(hat_x, hat_y)

                    # Insert neutral rest only when switching between real motion commands.
                    if (
                        active_name != "stop"
                        and new_name != "stop"
                        and new_name != active_name
                    ):
                        rest_until = now + TRANSITION_REST_S
                        print(f"Transition {active_name} -> {new_name}: rest {TRANSITION_REST_S:.2f}s")

                    if new_name != active_name:
                        print(f"Command: {new_name}")

                    active_name = new_name
                    active_cmd = new_cmd

            # Safety timeout
            if now - last_event_time > COMMAND_TIMEOUT_S and active_name == "stop":
                active_cmd = make_twist()

            # Publish command
            if now < rest_until:
                pub.publish(make_twist())
            else:
                pub.publish(active_cmd)

            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass

    finally:
        print("\nStopping robot...")
        for _ in range(20):
            pub.publish(make_twist())
            time.sleep(0.02)

        try:
            dev.ungrab()
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()
        print("Stopped.")


if __name__ == "__main__":
    main()
