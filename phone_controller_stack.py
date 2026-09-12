#!/usr/bin/env python3
"""Real-robot stack supervision for the phone controller."""

import os
import re
import shlex
import signal
import subprocess
import time

from phone_controller_config import (
    LIDAR_BACKEND,
    MOTOR_BACKEND,
    LIVE_SLAM_COMPARISON_ENABLED,
    SLAM_PARAMS,
    STACK_LOG_DIR,
    START_LIDAR,
    START_LIDAR_SCRIPT,
    START_MOTOR,
    START_SLAM,
)
from live_slam_comparison import (
    OFF_NODE_NAME,
    live_off_launch_command,
    write_live_off_parameters,
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
        self.live_off_params = None
        os.makedirs(STACK_LOG_DIR, exist_ok=True)

    @staticmethod
    def _motor_process_pattern(backend=MOTOR_BACKEND):
        if backend == "cpp":
            return r"(^|/)real_diffdrive_node_cpp($|[[:space:]])"
        return r"(^|/)real_diffdrive_node($|[[:space:]])"

    @staticmethod
    def _lidar_process_pattern(backend=LIDAR_BACKEND):
        if backend == "cpp":
            return r"(^|/)d500_ros2_scan_cpp($|[[:space:]])"
        return r"(^|/)d500_ros2_scan\.py($|[[:space:]])"

    @classmethod
    def _backend_process_patterns(cls):
        return cls._motor_process_pattern(), cls._lidar_process_pattern()

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

    @staticmethod
    def ros_node_exists(node_name):
        """Check an exact ROS node name without relying on process names."""
        try:
            result = subprocess.run(
                ros_shell_command(["ros2", "node", "list"]),
                capture_output=True,
                text=True,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return any(line.strip() == node_name for line in result.stdout.splitlines())

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
        motor_pattern, lidar_pattern = self._backend_process_patterns()
        motor_running = self.process_running(motor_pattern)
        lidar_running = self.process_running(lidar_pattern)
        other_motor_running = self.process_running(
            self._motor_process_pattern("python" if MOTOR_BACKEND == "cpp" else "cpp")
        )
        other_lidar_running = self.process_running(
            self._lidar_process_pattern("python" if LIDAR_BACKEND == "cpp" else "cpp")
        )
        scan_exists = self.scan_publisher_exists()

        if START_MOTOR and other_motor_running and not motor_running:
            raise RuntimeError(
                f"{MOTOR_BACKEND} motor requested, but the other motor backend "
                "is already running; stop it before changing backends"
            )
        if START_LIDAR and other_lidar_running and not lidar_running:
            raise RuntimeError(
                f"{LIDAR_BACKEND} lidar requested, but the other lidar backend "
                "is already running; stop it before changing backends"
            )

        if START_LIDAR and not scan_exists and not lidar_running:
            # Use the already-tested selector for all backend combinations.
            # A missing motor is included only when this controller owns motor
            # startup; this prevents duplicate GPIO users when a motor was
            # launched separately.
            selector_motor = MOTOR_BACKEND if START_MOTOR and not motor_running else "none"
            self.spawn(
                "hardware_selector",
                ros_shell_command(
                    [
                        "ros2",
                        "launch",
                        "my_epuck_project_cpp",
                        "hardware_backend_selector.launch.py",
                        f"motor_backend:={selector_motor}",
                        f"lidar_backend:={LIDAR_BACKEND}",
                    ]
                ),
            )
            lidar_running = True
            if selector_motor != "none":
                motor_running = True
        elif scan_exists:
            print("Reusing existing /scan publisher.")
        else:
            print(f"Reusing existing {LIDAR_BACKEND} lidar process; waiting for /scan.")

        if START_MOTOR and not motor_running:
            if MOTOR_BACKEND == "cpp":
                self.spawn(
                    "motor_cpp",
                    ros_shell_command(
                        [
                            "ros2",
                            "run",
                            "my_epuck_project_cpp",
                            "real_diffdrive_node_cpp",
                        ]
                    ),
                )
            else:
                self.spawn(
                    "motor",
                    ros_shell_command(
                        ["ros2", "run", "my_epuck_project", "real_diffdrive_node"]
                    ),
                )
        elif motor_running:
            print(f"Reusing existing {MOTOR_BACKEND} motor backend.")

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

        production_slam_running = self.ros_node_exists("/slam_toolbox") or self.process_running(
            r"slam_params_file:=" + re.escape(SLAM_PARAMS)
        )
        if START_SLAM and not production_slam_running:
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
        elif production_slam_running:
            print("Reusing existing slam_toolbox.")
        else:
            print("SLAM auto-start disabled; waiting for an externally launched slam_toolbox.")

        if LIVE_SLAM_COMPARISON_ENABLED and START_SLAM:
            off_running = self.ros_node_exists(f"/{OFF_NODE_NAME}") or self.process_running(
                OFF_NODE_NAME
            )
            if not off_running:
                self.live_off_params = os.path.join(
                    STACK_LOG_DIR, "live_slam_off", "slam_off.yaml"
                )
                write_live_off_parameters(SLAM_PARAMS, self.live_off_params)
                self.spawn(
                    "slam_off",
                    ros_shell_command(live_off_launch_command(self.live_off_params)),
                )
            else:
                print("Reusing existing slam_toolbox_off.")
        elif LIVE_SLAM_COMPARISON_ENABLED:
            print("Live SLAM comparison requested, but SLAM auto-start is disabled.")

    def startup_status(self, map_state):
        """Return the components that must be ready before teleoperation."""
        pending = []
        motor_pattern, _ = self._backend_process_patterns()
        if START_MOTOR and not self.process_running(motor_pattern):
            pending.append("motor node")
        if START_LIDAR and not map_state.lidar_is_ready():
            pending.append("lidar")
        if not self.process_running("static_transform_publisher.*d500_lidar"):
            pending.append("lidar TF")
        # This method is called by the browser status poll roughly twice per
        # second. Do not launch `ros2 node list` here: DDS discovery can take
        # several seconds on the Pi, pile up helper processes, consume CPU,
        # and leave the controls looking disabled while the stack is healthy.
        # The supervised process identity plus map readiness is sufficient for
        # this gate; start-time duplicate detection still uses exact ROS CLI
        # checks where it is only performed once.
        production_slam_ready = self.process_running(
            r"slam_params_file:=" + re.escape(SLAM_PARAMS)
        )
        if not production_slam_ready:
            pending.append("SLAM")
        if not map_state.has_map():
            pending.append("first map")
        comparison = {
            "enabled": LIVE_SLAM_COMPARISON_ENABLED,
            "off_topic": "/map_off" if LIVE_SLAM_COMPARISON_ENABLED else None,
            "off_ready": False,
        }
        if LIVE_SLAM_COMPARISON_ENABLED:
            off_ready = self.process_running(OFF_NODE_NAME)
            comparison["off_ready"] = off_ready and map_state.has_map_off()
            if not off_ready:
                pending.append("SLAM OFF")
            elif not map_state.has_map_off():
                pending.append("first OFF map")
        return {
            "ready": not pending,
            "pending": pending,
            "live_slam_comparison": comparison,
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
        patterns = [
            "[s]tart_d500_with_recovery.sh",
            self._lidar_process_pattern(),
            self._motor_process_pattern(),
            "hardware_backend_selector.launch.py",
            "[s]tatic_transform_publisher.*d500_lidar",
        ]
        # With external SLAM ownership, do not terminate that externally
        # launched mapper when the phone UI exits.
        if START_SLAM:
            patterns.append("[s]lam_toolbox")
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
