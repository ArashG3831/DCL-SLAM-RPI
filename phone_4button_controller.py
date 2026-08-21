#!/usr/bin/env python3
"""Safe phone teleoperation entry point."""

import os
import socket
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

from phone_controller_config import (
    FORWARD_RPM,
    HTTP_PORT,
    PUB_RATE_HZ,
    SPIN_MAX_RADPS,
    SLAM_MATCHING_EXPERIMENT_ENABLED,
)
from phone_controller_http import ReusableThreadingHTTPServer, make_handler
from phone_controller_ros import PhoneMapNode
from phone_controller_stack import RobotStackSupervisor
from phone_controller_state import (
    MapCheckpointSaver,
    LiveMapState,
    OdomDriftSession,
    PHONE_DIAGNOSTICS,
    SharedCommand,
    make_twist,
    rpm_to_mps,
)
from slam_matching_experiment import SlamMatchingExperiment


def advertised_host():
    configured = os.environ.get("PHONE_CONTROLLER_HOST", "").strip()
    if configured:
        return configured

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No packet is sent; this selects the address used for the LAN route.
        sock.connect(("192.0.2.1", 9))
        return sock.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        sock.close()


def main():
    rclpy.init()
    shared = SharedCommand()
    map_state = LiveMapState()
    comparison = SlamMatchingExperiment() if SLAM_MATCHING_EXPERIMENT_ENABLED else None
    odom_session = OdomDriftSession(shared, comparison=comparison)
    node = PhoneMapNode(map_state, odom_session)
    supervisor = RobotStackSupervisor()
    map_saver = MapCheckpointSaver(map_state)
    shutdown_event = threading.Event()
    server = None
    http_thread = None

    try:
        # Bind the phone page before starting the slower supervised ROS
        # components.  The browser can therefore show the initialization
        # screen immediately instead of waiting for lidar/SLAM/motor startup.
        server = ReusableThreadingHTTPServer(
            ("0.0.0.0", HTTP_PORT),
            make_handler(
                shared,
                map_state,
                odom_session,
                shutdown_event,
                supervisor,
                node,
            ),
        )
        http_thread = threading.Thread(
            target=server.serve_forever,
            name="phone_http_server",
            daemon=True,
        )
        http_thread.start()
        print(f"Phone page available immediately: http://{advertised_host()}:{HTTP_PORT}/")

        map_saver.start()
        supervisor.start()
    except Exception as exc:
        if server is not None:
            try:
                server.shutdown()
                server.server_close()
            except Exception:
                pass
        if http_thread is not None:
            http_thread.join(timeout=1.0)
        map_saver.stop()
        supervisor.stop_owned_children()
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(f"Could not open phone-controller port {HTTP_PORT}: {exc}")

    # Keep ROS callbacks running continuously.  The old loop dispatched only
    # one callback every 50 ms, even though /tf and the sensor topics together
    # can deliver many more callbacks per second.  That made the pose timer
    # read stale map -> base_link transforms.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="phone_ros_executor",
        daemon=True,
    )
    executor_thread.start()

    print()
    print("Robot 1 phone controller with live map running.")
    print(f"Open on the phone: http://{advertised_host()}:{HTTP_PORT}/")
    print(f"Forward/back RPM: {FORWARD_RPM:.0f}")
    print(f"Forward speed:    {rpm_to_mps(FORWARD_RPM):.3f} m/s")
    print(f"Spin command:     {SPIN_MAX_RADPS:.3f} rad/s")
    print("Live map source:   /map and map -> base_link TF")
    print("Stack startup:     lidar, motor, lidar TF, and SLAM are supervised")
    print("Disable motor auto-start with: ROBOT1_PHONE_START_MOTOR=0")
    print("Press Ctrl+C to quit.")
    print()

    dt = 1.0 / PUB_RATE_HZ
    pub = node.create_publisher(Twist, "/cmd_vel_unstamped", 10)
    try:
        while rclpy.ok() and not shutdown_event.is_set():
            pub.publish(shared.get_twist())
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping robot...")
        shared.set_key("x")
        try:
            stop = make_twist()
            for _ in range(20):
                if not rclpy.ok():
                    break
                try:
                    pub.publish(stop)
                except Exception as exc:
                    print(f"Stop publish skipped: {exc}")
                    break
                time.sleep(0.02)
        finally:
            try:
                server.shutdown()
                server.server_close()
                http_thread.join(timeout=1.0)
            except Exception as exc:
                print(f"HTTP shutdown warning: {exc}")

            # This is deliberately before any final ROS teardown can prevent
            # cleanup from running. It also removes reused/orphaned nodes.
            map_saver.stop()
            print("Saving final map checkpoint...")
            map_saver.save_now()
            odom_session.shutdown()
            supervisor.stop_all_stack_processes()
            PHONE_DIAGNOSTICS.close()

            try:
                executor.shutdown(timeout_sec=1.0)
                executor_thread.join(timeout=1.0)
            except Exception as exc:
                print(f"ROS executor shutdown warning: {exc}")

            try:
                node.destroy_node()
            except Exception as exc:
                print(f"ROS node shutdown warning: {exc}")
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as exc:
                print(f"ROS context shutdown warning: {exc}")
        print("Stopped.")


if __name__ == "__main__":
    main()
