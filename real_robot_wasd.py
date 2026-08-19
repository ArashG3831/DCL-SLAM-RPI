#!/usr/bin/env python3

import sys
import termios
import tty
import select

import rclpy
from geometry_msgs.msg import Twist

V = 0.025      # m/s, about 7.3 RPM with 6.5 cm wheels
W = 0.25       # rad/s, slow spin
RATE = 10.0

HELP = """
Real robot slow WASD control:

  w = forward
  s = backward
  a = spin left
  d = spin right
  x = stop
  q = quit

Use short taps. Watch the wall cable.
"""

def get_key(timeout=0.1):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    return None

def main():
    rclpy.init()
    node = rclpy.create_node("real_robot_wasd")
    pub = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    cmd = Twist()
    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(1.0 / RATE)

            if key == "w":
                cmd.linear.x = V
                cmd.angular.z = 0.0
            elif key == "s":
                cmd.linear.x = -V
                cmd.angular.z = 0.0
            elif key == "a":
                cmd.linear.x = 0.0
                cmd.angular.z = W
            elif key == "d":
                cmd.linear.x = 0.0
                cmd.angular.z = -W
            elif key == "x":
                cmd = Twist()
            elif key == "q":
                break

            pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.0)

    finally:
        stop = Twist()
        for _ in range(10):
            pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped.")

if __name__ == "__main__":
    main()
