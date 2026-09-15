"""Safe Robot 2 solo/cooperative mode supervisor.

This module owns integration state only. It does not publish velocity, send a
navigation action, estimate a pose, or implement allocation. The current
physical profile deliberately fails closed at the late-handoff boundary until
the stationary registration and shared-Nav2 contracts are proven on hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import os
import signal
import shutil
import subprocess
import time

import rclpy
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import Twist, TwistStamped
from my_epuck_interfaces.msg import LocalMapDescriptor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ResilientMode(str, Enum):
    SOLO_LOCAL_MAPPING = 'SOLO_LOCAL_MAPPING'
    HANDOFF_PENDING = 'HANDOFF_PENDING'
    STATIONARY_ALIGNMENT = 'STATIONARY_ALIGNMENT'
    SHARED_MAPPING_READY = 'SHARED_MAPPING_READY'
    COOPERATIVE_ACTIVE = 'COOPERATIVE_ACTIVE'
    HANDOFF_FAILED_SOLO = 'HANDOFF_FAILED_SOLO'


@dataclass
class ResilientDecisionState:
    """Pure state-machine boundary used by the node and focused tests."""

    peer_robot_id: str = 'robot1'
    mode: ResilientMode = ResilientMode.SOLO_LOCAL_MAPPING
    peer_last_seen_ns: int = 0
    peer_keyframe_id: str = ''
    events: list[str] = field(default_factory=list)

    def record(self, event: str) -> None:
        self.events.append(str(event))

    def observe_descriptor(
        self,
        source_robot_id: str,
        stamp_ns: int,
        keyframe_id: str,
        now_ns: int,
        timeout_ns: int,
    ) -> bool:
        """Accept only a fresh, identity-valid descriptor as peer presence."""
        if str(source_robot_id) != self.peer_robot_id:
            return False
        if not str(keyframe_id) or int(stamp_ns) <= 0:
            return False
        age_ns = int(now_ns) - int(stamp_ns)
        if age_ns < 0 or age_ns > int(timeout_ns):
            return False
        self.peer_last_seen_ns = int(now_ns)
        self.peer_keyframe_id = str(keyframe_id)
        if self.mode == ResilientMode.SOLO_LOCAL_MAPPING:
            self.mode = ResilientMode.HANDOFF_PENDING
            self.record('PEER_DESCRIPTOR_VALID_HANDOFF_PENDING')
        return True

    def peer_fresh(self, now_ns: int, timeout_ns: int) -> bool:
        return (
            self.peer_last_seen_ns > 0
            and int(now_ns) - self.peer_last_seen_ns <= int(timeout_ns)
        )

    def mark_owner_terminal(self) -> None:
        if self.mode == ResilientMode.HANDOFF_PENDING:
            self.mode = ResilientMode.STATIONARY_ALIGNMENT
            self.record('SOLO_OWNER_TERMINAL')

    def mark_handoff_failed(self, reason: str) -> None:
        self.mode = ResilientMode.HANDOFF_FAILED_SOLO
        self.record(str(reason))

    def return_to_solo(self) -> None:
        self.mode = ResilientMode.SOLO_LOCAL_MAPPING
        self.record('HANDOFF_RETURNED_TO_SOLO')

    def mark_shared_mapping_ready(self) -> None:
        if self.mode == ResilientMode.STATIONARY_ALIGNMENT:
            self.mode = ResilientMode.SHARED_MAPPING_READY
            self.record('SHARED_MAPPING_READY')

    def mark_cooperative_active(self) -> None:
        if self.mode == ResilientMode.SHARED_MAPPING_READY:
            self.mode = ResilientMode.COOPERATIVE_ACTIVE
            self.record('COOPERATIVE_ACTIVE')


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _twist_nonzero(twist, epsilon: float = 1.0e-4) -> bool:
    return (
        abs(float(twist.linear.x)) > epsilon
        or abs(float(twist.linear.y)) > epsilon
        or abs(float(twist.linear.z)) > epsilon
        or abs(float(twist.angular.x)) > epsilon
        or abs(float(twist.angular.y)) > epsilon
        or abs(float(twist.angular.z)) > epsilon
    )


class Robot2ResilientModeManager(Node):
    """Start solo immediately and fail closed at an unproven handoff."""

    def __init__(self):
        super().__init__('robot2_resilient_mode_manager')
        self.robot_id = str(self.declare_parameter('robot_id', 'robot2').value)
        self.peer_robot_id = str(
            self.declare_parameter('peer_robot_id', 'robot1').value)
        self.peer_descriptor_topic = str(self.declare_parameter(
            'peer_descriptor_topic', '/cslam/relative_pose/descriptors').value)
        self.peer_timeout_s = max(1.0, float(self.declare_parameter(
            'peer_timeout_s', 5.0).value))
        self.handoff_timeout_s = max(2.0, float(self.declare_parameter(
            'handoff_timeout_s', 15.0).value))
        self.zero_velocity_settle_s = max(0.1, float(self.declare_parameter(
            'zero_velocity_settle_s', 0.5).value))
        # This remains false until physical stationary registration and shared
        # Nav2 ownership have been independently closed.
        self.late_handoff_supported = bool(self.declare_parameter(
            'late_handoff_supported', False).value)
        self.solo_launch_package = str(self.declare_parameter(
            'solo_launch_package', 'my_epuck_project').value)
        self.solo_launch_file = str(self.declare_parameter(
            'solo_launch_file', 'robot2_solo_frontier_launch.py').value)
        autostart_value = self.declare_parameter(
            'solo_frontier_autostart', True).value
        self.solo_frontier_autostart = (
            autostart_value if isinstance(autostart_value, bool) else
            str(autostart_value).strip().lower() in ('1', 'true', 'yes', 'on'))

        self.state = ResilientDecisionState(peer_robot_id=self.peer_robot_id)
        self._child: subprocess.Popen | None = None
        self._shutdown_requested = False
        self._stop_request_in_flight = False
        self._stop_accepted = False
        self._start_request_in_flight = False
        self._handoff_started_monotonic = 0.0
        self._zero_since_monotonic: float | None = None
        self._handoff_cooldown_until = 0.0
        self._last_cmd_nonzero = False
        self._last_cmd_monotonic = 0.0
        self._last_odom_nonzero = False
        self._last_odom_monotonic = 0.0

        descriptor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            LocalMapDescriptor, self.peer_descriptor_topic,
            self._descriptor_callback, descriptor_qos)
        self.create_subscription(
            TwistStamped, '/robot2/cmd_vel', self._stamped_cmd_callback, 10)
        self.create_subscription(
            Twist, '/robot2/cmd_vel_unstamped', self._unstamped_cmd_callback, 10)
        self.create_subscription(
            Odometry, '/robot2/odom', self._odom_callback, 10)
        self._control_client = self.create_client(
            ControlExploration, '/robot2/control_exploration')
        self._timer = self.create_timer(0.1, self._tick)
        self._launch_solo_stack()
        self._event('SOLO_LOCAL_MAPPING', reason='startup_immediate')

    def _event(self, event: str, **fields) -> None:
        self.state.record(event)
        details = ' '.join('%s=%s' % (key, value)
                           for key, value in sorted(fields.items()))
        self.get_logger().info('ROBOT2_RESILIENT_EVENT event=%s %s' %
                               (event, details))

    def _launch_solo_stack(self) -> None:
        if self._child is not None and self._child.poll() is None:
            return
        ros2 = shutil.which('ros2') or 'ros2'
        argv = [ros2, 'launch', self.solo_launch_package, self.solo_launch_file,
                'frontier_autostart:=%s' % (
                    'true' if self.solo_frontier_autostart else 'false')]
        self._child = subprocess.Popen(
            argv, env=os.environ.copy(), start_new_session=True, close_fds=True)
        self.get_logger().info(
            'SOLO_STACK_STARTED pid=%s package=%s launch=%s' %
            (self._child.pid, self.solo_launch_package, self.solo_launch_file))

    def _descriptor_callback(self, message: LocalMapDescriptor) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if time.monotonic() < self._handoff_cooldown_until:
            return
        accepted = self.state.observe_descriptor(
            message.source_robot_id, _stamp_ns(message.header.stamp),
            message.keyframe_id, now_ns, int(self.peer_timeout_s * 1.0e9))
        if accepted and self.state.mode == ResilientMode.HANDOFF_PENDING:
            self._event('HANDOFF_PENDING', peer=message.source_robot_id)
            self._request_solo_stop()

    def _stamped_cmd_callback(self, message: TwistStamped) -> None:
        self._last_cmd_nonzero = _twist_nonzero(message.twist)
        self._last_cmd_monotonic = time.monotonic()

    def _unstamped_cmd_callback(self, message: Twist) -> None:
        self._last_cmd_nonzero = _twist_nonzero(message)
        self._last_cmd_monotonic = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        self._last_odom_nonzero = _twist_nonzero(message.twist.twist)
        self._last_odom_monotonic = time.monotonic()

    def _request_solo_stop(self) -> None:
        if self._stop_request_in_flight or self._stop_accepted:
            return
        self._stop_request_in_flight = True
        self._handoff_started_monotonic = time.monotonic()
        self._event('SOLO_OWNER_FREEZE_REQUESTED')
        if not self._control_client.service_is_ready():
            self._stop_request_in_flight = False
            self._event('SOLO_OWNER_FREEZE_FAILED', reason='service_unavailable')
            self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')
            return
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_STOP
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self._control_client.call_async(request)
        future.add_done_callback(self._stop_response)

    def _stop_response(self, future) -> None:
        self._stop_request_in_flight = False
        try:
            response = future.result()
        except Exception as error:  # pragma: no cover - ROS transport path
            self._event('SOLO_OWNER_FREEZE_FAILED', reason=str(error))
            self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')
            return
        if not bool(response.accepted):
            self._event('SOLO_OWNER_FREEZE_FAILED', reason=response.message)
            self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')
            return
        self._stop_accepted = True

    def _request_solo_start(self) -> None:
        if self._start_request_in_flight or not self._control_client.service_is_ready():
            return
        self._start_request_in_flight = True
        request = ControlExploration.Request()
        request.action = ControlExploration.Request.ACTION_START
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        future = self._control_client.call_async(request)
        future.add_done_callback(self._start_response)

    def _start_response(self, future) -> None:
        self._start_request_in_flight = False
        try:
            response = future.result()
            if not bool(response.accepted):
                self.get_logger().error(
                    'HANDOFF_RETURN_TO_SOLO_FAILED message=%s' % response.message)
                return
        except Exception as error:  # pragma: no cover - ROS transport path
            self.get_logger().error(
                'HANDOFF_RETURN_TO_SOLO_FAILED error=%s' % error)
            return
        self.state.return_to_solo()
        self._event('HANDOFF_RETURNED_TO_SOLO')

    def _fail_handoff(self, reason: str) -> None:
        self.state.mark_handoff_failed(reason)
        self._event(reason)
        self._handoff_cooldown_until = time.monotonic() + self.peer_timeout_s
        self._stop_accepted = False
        if self._control_client.service_is_ready():
            self._request_solo_start()

    def _zero_velocity_observed(self) -> bool:
        now = time.monotonic()
        recent_cmd = now - self._last_cmd_monotonic <= 1.0
        recent_odom = now - self._last_odom_monotonic <= 1.0
        return recent_cmd and not self._last_cmd_nonzero and recent_odom and not self._last_odom_nonzero

    def _tick(self) -> None:
        if self._child is not None and self._child.poll() is not None:
            if not self._shutdown_requested:
                self.get_logger().error(
                    'SOLO_STACK_EXITED return_code=%s' % self._child.returncode)
            return
        if self.state.mode != ResilientMode.HANDOFF_PENDING:
            return
        if time.monotonic() - self._handoff_started_monotonic > self.handoff_timeout_s:
            self._event('SOLO_OWNER_FREEZE_FAILED', reason='timeout')
            self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')
            return
        if not self._stop_accepted or not self._zero_velocity_observed():
            if not self._zero_velocity_observed():
                self._zero_since_monotonic = None
            return
        if self._zero_since_monotonic is None:
            self._zero_since_monotonic = time.monotonic()
            self._event('HANDOFF_ZERO_VELOCITY')
            return
        if time.monotonic() - self._zero_since_monotonic < self.zero_velocity_settle_s:
            return
        self.state.mark_owner_terminal()
        self._event('SOLO_OWNER_TERMINAL')
        if not self.late_handoff_supported:
            self._event('LATE_HANDOFF_UNSUPPORTED_BY_CURRENT_REGISTRATION_CONTRACT')
            self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')
            return
        self._event('COOPERATIVE_HANDOFF_BLOCKED_SHARED_NAV2_NOT_INTEGRATED')
        self._fail_handoff('PEER_DETECTED_HANDOFF_NOT_READY')

    def destroy_node(self):
        self._shutdown_requested = True
        child = self._child
        if child is not None and child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
                deadline = time.monotonic() + 10.0
                while child.poll() is None and time.monotonic() < deadline:
                    try:
                        time.sleep(0.05)
                    except KeyboardInterrupt:
                        # A launch parent may deliver a second SIGINT while
                        # the owned child group is already shutting down.
                        break
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGTERM)
            except (ProcessLookupError, KeyboardInterrupt):
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robot2ResilientModeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
