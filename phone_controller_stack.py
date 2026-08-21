#!/usr/bin/env python3
"""Real-robot stack supervision for the phone controller."""

import os
import shlex
import signal
import subprocess
import time

from phone_controller_config import (
    SLAM_PARAMS,
    STACK_LOG_DIR,
    START_LIDAR,
    START_LIDAR_SCRIPT,
    START_MOTOR,
)


def ros_shell_command(command):
    """Run a ROS CLI command with the robot workspace overlays sourced."""
    setup = [
        "source /opt/ros/jazzy/setup.bash",
        '[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"',
        '[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"',
        '[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"',
    ]
    return ["bash", "-lc", "\n".join(setup + [f"exec {shlex.join(command)}"])]


class RobotStackSupervisor:
    """Start missing real-robot support nodes without duplicating existing ones."""

    def __init__(self):
        self.children = []
        os.makedirs(STACK_LOG_DIR, exist_ok=True)

    @staticmethod
    def process_running(pattern):
        try:
            result = subprocess.run(
                ["pgrep", "-f", pattern],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            return result.returncode == 0
        except OSError:
            return False

    @staticmethod
    def scan_publisher_exists():
        """Return true only when ROS reports a publisher on /scan."""
        try:
            result = subprocess.run(
                ros_shell_command(["ros2", "topic", "info", "/scan"]),
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False

        for line in result.stdout.splitlines():
            if line.strip().lower().startswith("publisher count:"):
                try:
                    return int(line.split(":", 1)[1].strip()) > 0
                except (IndexError, ValueError):
                    return False
        return False

    def spawn(self, name, command):
        log_path = os.path.join(STACK_LOG_DIR, f"{name}.log")
        log_file = open(log_path, "ab", buffering=0)
        try:
            child = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            log_file.close()
            raise

        self.children.append((name, child, log_file))
        print(f"Started {name} (pid {child.pid}); log: {log_path}")
        return child

    def start(self):
        if START_LIDAR and not self.scan_publisher_exists() and not self.process_running("d500_ros2_scan.py"):
            self.spawn("lidar", ["bash", START_LIDAR_SCRIPT])
        elif self.scan_publisher_exists():
            print("Reusing existing /scan publisher.")
        else:
            print("Reusing existing lidar process; waiting for /scan.")

        if START_MOTOR and not self.process_running("real_diffdrive_node"):
            self.spawn(
                "motor",
                ros_shell_command(
                    ["ros2", "run", "my_epuck_project", "real_diffdrive_node"]
                ),
            )
        elif self.process_running("real_diffdrive_node"):
            print("Reusing existing real_diffdrive_node.")

        if not self.process_running("static_transform_publisher.*d500_lidar"):
            self.spawn(
                "lidar_tf",
                ros_shell_command(
                    [
                        "ros2",
                        "run",
                        "tf2_ros",
                        "static_transform_publisher",
                        "--x",
                        "0.0",
                        "--y",
                        "0.0",
                        "--z",
                        "0.07",
                        "--roll",
                        "0.0",
                        "--pitch",
                        "0.0",
                        "--yaw",
                        "0.0",
                        "--frame-id",
                        "base_link",
                        "--child-frame-id",
                        "d500_lidar",
                    ]
                ),
            )

        if not self.process_running("slam_toolbox"):
            self.spawn(
                "slam",
                ros_shell_command(
                    [
                        "ros2",
                        "launch",
                        "slam_toolbox",
                        "online_async_launch.py",
                        f"slam_params_file:={SLAM_PARAMS}",
                        "use_sim_time:=false",
                    ]
                ),
            )
        else:
            print("Reusing existing slam_toolbox.")

    def startup_status(self, map_state):
        """Return the components that must be ready before teleoperation."""
        pending = []
        if not self.process_running("real_diffdrive_node"):
            pending.append("motor node")
        if not map_state.lidar_is_ready():
            pending.append("lidar")
        if not self.process_running("static_transform_publisher.*d500_lidar"):
            pending.append("lidar TF")
        if not self.process_running("slam_toolbox"):
            pending.append("SLAM")
        if not map_state.has_map():
            pending.append("first map")
        return {
            "ready": not pending,
            "pending": pending,
        }

    def stop_owned_children(self):
        for name, child, log_file in reversed(self.children):
            if child.poll() is not None:
                log_file.close()
                continue
            print(f"Stopping {name} (pid {child.pid})...")
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

        deadline = time.monotonic() + 5.0
        for _, child, _ in self.children:
            remaining = max(0.0, deadline - time.monotonic())
            try:
                child.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

        for _, child, log_file in self.children:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            log_file.close()

    def stop_all_stack_processes(self):
        """Stop this program's complete real-robot stack, including reused nodes."""
        self.stop_owned_children()

        # A previous run may have exited before its Popen handles reached the
        # cleanup code.  Kill the exact stack components so stale SLAM/map,
        # motor, TF, and lidar processes cannot survive into the next run.
        patterns = (
            "[s]tart_d500_with_recovery.sh",
            "[d]500_ros2_scan.py",
            "[r]eal_diffdrive_node",
            "[s]tatic_transform_publisher.*d500_lidar",
            "[s]lam_toolbox",
        )
        for sig in ("-INT", "-TERM", "-KILL"):
            for pattern in patterns:
                subprocess.run(
                    ["pkill", sig, "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    check=False,
                )
            if sig != "-KILL":
                time.sleep(1.0)



