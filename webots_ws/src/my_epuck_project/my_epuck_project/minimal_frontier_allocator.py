"""Compose the six primitives of the minimal decentralized coordinator.

This module owns ROS I/O and the small current evaluation/navigation state.
It does not detect frontiers, plan paths, implement traffic geometry, retain
history, run certificates or continuation rounds, or recover infrastructure.
"""

from __future__ import annotations

import json
import time
from dataclasses import replace

from action_msgs.msg import GoalStatus
from my_epuck_interfaces.msg import (
    DistributedExplorationStatus,
    DistributedExplorationEvent,
    FrontierCandidateArray,
    TaskBidArray,
)
from nav2_msgs.action import FollowPath, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from . import minimal_frontier_navigation as navigation
from . import minimal_frontier_protocol as protocol
from . import minimal_frontier_selection as selection
from . import minimal_frontier_sets as frontier_sets
from . import minimal_frontier_termination as termination
from . import minimal_frontier_traffic as traffic
from .distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    LocalNav2,
    NavigationOutcome,
    classify_dispatch_precondition_failure,
    classify_follow_path_controller_error,
    follow_path_controller_error_name,
)
from .distributed_assignment.models import FailureClass
from .mission_termination import CandidateEvidence


class MinimalFrontierAllocator:
    """Robot-local composition object; the caller supplies the ROS node."""

    IDLE = 'IDLE'
    EVALUATING = 'EVALUATING'
    WAITING_TRAFFIC = 'WAITING_TRAFFIC'
    GOAL_PENDING = 'GOAL_PENDING'
    NAVIGATING = 'NAVIGATING'
    SOLO_GEOMETRY_REJECTION_REASONS = frozenset({
        'NAV2_GLOBAL_PATH_INVALID',
        'FOOTPRINT_LETHAL_LOCAL_CELL',
        'FOOTPRINT_OUT_OF_LOCAL_WINDOW',
    })

    def __init__(
        self, node: Node, robot_id: str | None = None, *,
            configure_io: bool = True) -> None:
        self._node = node
        if robot_id is None:
            configured_id = node.declare_parameter('robot_id', '').value
            self._robot_id = str(configured_id)
        else:
            self._robot_id = str(robot_id)
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be robot1 or robot2')
        self._peer_id = 'robot2' if self._robot_id == 'robot1' else 'robot1'
        self._source_session_id = '0' * 32
        self._nav: LocalNav2 | None = None
        self._bid_publisher = None
        self._status_publisher = None
        self._event_publisher = None
        self._start_ready_publisher = None
        self._start_ready_published = False
        self._initialize_state()
        if configure_io:
            self._configure_io()

    def _initialize_state(self) -> None:
        self._candidates = {'robot1': None, 'robot2': None}
        self._evidence = {
            'robot1': CandidateEvidence(),
            'robot2': CandidateEvidence(),
        }
        self._union = None
        self._local_batch = None
        self._local_batch_from_costing = False
        self._peer_batch = None
        self._peer_active_goal: str | None = None
        self._peer_status_union_hash: str | None = None
        self._peer_status_state: int | None = None
        self._peer_status_current = False
        self._peer_bid_after_terminal = True
        self._peer_terminal = False
        self._peer_terminal_reason = ''
        self._peer_status_after_terminal = True
        self._active_goal_id: str | None = None
        self._active_goal_union_hash: str | None = None
        self._goal_token = 0
        self._state = self.IDLE
        self._completed_ids: set[str] = set()
        self._local_completed_ids: set[str] = set()
        self._traffic_decision = None
        self._terminal_reason: str | None = None
        self._terminal_epoch = 0
        self._terminal_map_revision = 0
        self._solo_terminal_latched = False
        self._allocation_reason = 'initial allocation'
        self._stable_since_s = self._now_s()
        self._released = True
        self._start_release_required = False
        self._stability_grace_s = 5.0
        self._bid_validity_s = 0.0
        self._minimum_visible_gain_m = 0.05
        self._safe_radius = 0.08
        self._reference_speed = 0.13
        self._eta_tie_s = 0.05
        # Pair coordination remains the default simulator behavior.  The
        # physical Robot 2 launch opts into the local-only branch explicitly.
        self._allow_solo_without_peer = False
        self._dispatch_enabled = True
        self._max_navigation_goals = 0
        self._navigation_goal_count = 0
        self._peer_candidate_timeout_s = 8.0
        self._candidate_received_steady_s = {
            'robot1': None,
            'robot2': None,
        }
        self._peer_candidate_available = False
        self._cooperative_pending = False
        self._cooperative_active = False
        self._solo_mode = False
        self._global_frame = 'shared_map'
        self._last_solo_snapshot_key = None
        self._solo_preflight_blocked = {}
        self._solo_pending_preflight = None
        self._solo_preflight_callback = None
        self._solo_planner_pause_pending = False
        self._solo_planner_idle = True
        self._solo_planner_arbitration_enabled = False
        self._solo_active_dispatch = None
        self._solo_evidence_seen = False
        self._solo_startup_spin_enabled = False
        self._solo_startup_spin_angle_rad = 0.1745
        self._solo_startup_spin_started = False
        self._solo_startup_spin_finished = False
        self._solo_startup_spin_succeeded = False
        self._solo_startup_spin_active = False
        self._solo_startup_spin_waiting_for_snapshot = False
        self._solo_startup_spin_baseline_stamp_ns = 0
        self._solo_startup_spin_baseline_generation_id = 0
        self._solo_startup_spin_client = None
        self._solo_follow_path_client = None
        self._solo_follow_path_goal_handle = None
        self._solo_follow_path_callback = None
        self._solo_follow_path_started_steady_s = 0.0
        self._solo_follow_path_start_distance_m = 0.0
        self._solo_follow_path_cancel_requested = False

    def _configure_io(self) -> None:
        declare = self._node.declare_parameter
        self._start_release_required = bool(
            declare('common_start_release_required', False).value)
        self._released = not self._start_release_required
        self._stability_grace_s = float(
            declare('map_stability_grace_s', 5.0).value)
        self._bid_validity_s = float(declare('bid_validity_s', 0.0).value)
        self._minimum_visible_gain_m = float(
            declare('minimum_solo_visible_gain_m', 0.05).value)
        self._safe_radius = float(
            declare('traffic_safe_radius_m', 0.08).value)
        self._reference_speed = float(
            declare('traffic_reference_speed_mps', 0.13).value)
        self._eta_tie_s = float(declare('traffic_eta_tie_s', 0.05).value)
        self._allow_solo_without_peer = bool(
            declare('allow_solo_without_peer', False).value)
        self._dispatch_enabled = bool(
            declare('dispatch_enabled', True).value)
        self._solo_startup_spin_enabled = bool(
            declare('solo_startup_spin_enabled', False).value)
        self._solo_startup_spin_angle_rad = float(
            declare('solo_startup_spin_angle_rad', 0.1745).value)
        self._solo_planner_arbitration_enabled = bool(
            declare('pause_planner_queries_while_navigating', False).value)
        self._max_navigation_goals = max(
            0, int(declare('max_navigation_goals', 0).value),
        )
        self._peer_candidate_timeout_s = max(
            0.1, float(declare('peer_candidate_timeout_s', 8.0).value),
        )
        self._nav = LocalNav2(self._node, phase_gated=False)
        self._global_frame = str(
            getattr(self._nav, '_global_frame', 'shared_map'))
        self._solo_mode = self._allow_solo_without_peer
        if (
                self._allow_solo_without_peer and
                self._solo_startup_spin_enabled and
                self._dispatch_enabled):
            self._solo_startup_spin_client = ActionClient(
                self._node, Spin, f'/{self._robot_id}/spin')
        if self._allow_solo_without_peer and self._dispatch_enabled:
            self._solo_follow_path_client = ActionClient(
                self._node, FollowPath, f'/{self._robot_id}/follow_path')

        reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        candidate_topic = str(
            declare('candidate_topic', 'frontier_candidates').value).lstrip('/')
        bid_topic = f'/{self._robot_id}/task_bids'
        status_topic = f'/{self._robot_id}/distributed_status'
        self._bid_publisher = self._node.create_publisher(
            TaskBidArray, bid_topic, reliable)
        self._status_publisher = self._node.create_publisher(
            DistributedExplorationStatus, status_topic, reliable)
        event_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._event_publisher = self._node.create_publisher(
            DistributedExplorationEvent,
            f'/{self._robot_id}/distributed_event',
            event_qos,
        )
        if bool(declare('publish_cooperative_start_ready', False).value):
            self._start_ready_publisher = self._node.create_publisher(
                String,
                f'/cslam/unknown_pose/cooperative_start_ready/{self._robot_id}',
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                ),
            )
        for robot in ('robot1', 'robot2'):
            topic = f'/{robot}/{candidate_topic}'
            self._node.create_subscription(
                FrontierCandidateArray, topic, self.on_candidate_array, volatile)
        self._node.create_subscription(
            TaskBidArray,
            f'/{self._peer_id}/task_bids',
            self.on_peer_bid,
            reliable,
        )
        self._node.create_subscription(
            DistributedExplorationStatus,
            f'/{self._peer_id}/distributed_status',
            self.on_peer_status,
            reliable,
        )
        self._node.create_subscription(
            DistributedExplorationEvent,
            f'/{self._robot_id}/distributed_event',
            self.on_local_event,
            event_qos,
        )
        self._node.create_subscription(
            DistributedExplorationEvent,
            f'/{self._peer_id}/distributed_event',
            self.on_peer_event,
            event_qos,
        )
        if self._start_release_required:
            self._node.create_subscription(
                String,
                '/cslam/unknown_pose/start_release',
                self.on_start_release,
                reliable,
            )
        self._node.create_timer(0.5, self._tick)

    def _now_s(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _log_info(self, message: str) -> None:
        get_logger = getattr(self._node, 'get_logger', None)
        if get_logger is None:
            return
        logger = get_logger()
        logger.info(message)

    @staticmethod
    def _message_stamp_ns(message) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _make_evidence(self, message: FrontierCandidateArray) -> CandidateEvidence:
        reachable = sum(
            candidate.reachability_state == candidate.REACHABLE
            for candidate in message.candidates
        )
        actionable = sum(
            candidate.reachability_state == candidate.REACHABLE and
            candidate.information_gain >= self._minimum_visible_gain_m
            for candidate in message.candidates
        )
        return CandidateEvidence(
            detected=int(message.detected_frontier_count),
            small=int(message.small_frontier_count),
            reachable=reachable,
            out_of_range=int(message.out_of_range_frontier_count),
            unreachable=int(message.unreachable_frontier_count),
            planner_failures=int(message.planner_failure_count),
            unclassified=int(message.unclassified_frontier_count),
            detected_not_queried=int(message.detected_not_queried_count),
            below_minimum_gain=reachable - actionable,
            actionable_reachable=actionable,
        )

    def _solo_evidence_is_fresh(self) -> bool:
        """Require a recent, valid local candidate snapshot before completion."""
        message = self._candidates[self._robot_id]
        received = self._candidate_received_steady_s.get(self._robot_id)
        if message is None or received is None or not str(message.header.frame_id):
            return False
        if self._message_stamp_ns(message) <= 0:
            return False
        return time.monotonic() - float(received) <= self._peer_candidate_timeout_s

    def _solo_nav_readiness_is_current(self) -> bool:
        """Do not turn a startup/infrastructure gap into mission completion."""
        if self._nav is None:
            return False
        for method_name in (
                '_ensure_compute_client', '_ensure_navigate_client',
                '_ensure_lifecycle_clients'):
            method = getattr(self._nav, method_name, None)
            if callable(method):
                method()
        refresh_health = getattr(self._nav, 'refresh_health', None)
        if callable(refresh_health):
            refresh_health()
        health_flags = getattr(self._nav, 'health_flags', None)
        if not callable(health_flags):
            return False
        try:
            nav2_healthy, tf_healthy = health_flags()
        except Exception:  # noqa: B902 - readiness is fail-closed
            return False
        return bool(nav2_healthy and tf_healthy)

    def _solo_startup_spin_required(self) -> bool:
        """Return whether this physical solo run needs the one-shot spin."""
        return bool(
            self._allow_solo_without_peer and self._solo_mode and
            self._solo_startup_spin_enabled and self._dispatch_enabled)

    def _finish_solo_startup_spin(self, succeeded: bool, reason: str) -> None:
        self._solo_startup_spin_active = False
        self._solo_startup_spin_finished = True
        self._solo_startup_spin_succeeded = bool(succeeded)
        self._solo_startup_spin_waiting_for_snapshot = bool(succeeded)
        self._allocation_reason = reason
        self._log_info(
            'MINIMAL_ALLOCATOR_SOLO_STARTUP_SPIN_RESULT '
            f'robot={self._robot_id} succeeded={bool(succeeded)} '
            f'reason={reason}',
        )
        self._publish_status()

    def _on_solo_startup_spin_result(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(getattr(wrapped, 'status', 0))
            result = getattr(wrapped, 'result', None)
            error_code = int(getattr(result, 'error_code', 0))
            succeeded = (
                status == GoalStatus.STATUS_SUCCEEDED and error_code == 0)
            reason = (
                'solo startup spin succeeded' if succeeded else
                f'solo startup spin failed status={status} '
                f'error_code={error_code} '
                f'error_msg={getattr(result, "error_msg", "")}'
            )
        except Exception as exc:  # noqa: B902 - action result is fail-closed
            self._finish_solo_startup_spin(
                False, f'solo startup spin result error: {exc}')
            return
        self._finish_solo_startup_spin(succeeded, reason)

    def _on_solo_startup_spin_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: B902 - action response is fail-closed
            self._finish_solo_startup_spin(
                False, f'solo startup spin goal error: {exc}')
            return
        if goal_handle is None or not goal_handle.accepted:
            self._finish_solo_startup_spin(
                False, 'solo startup spin goal rejected')
            return
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_solo_startup_spin_result)
        except Exception as exc:  # noqa: B902 - action result is fail-closed
            self._finish_solo_startup_spin(
                False, f'solo startup spin result setup error: {exc}')

    def _start_solo_startup_spin(self) -> None:
        if self._solo_startup_spin_started:
            return
        if not self._solo_nav_readiness_is_current():
            self._allocation_reason = 'waiting for Nav2 readiness before solo startup spin'
            return
        client = self._solo_startup_spin_client
        if client is None:
            self._allocation_reason = 'solo startup spin action client unavailable'
            return
        try:
            if not client.server_is_ready():
                self._allocation_reason = 'waiting for /robot2/spin action server'
                return
            goal = Spin.Goal()
            goal.target_yaw = float(self._solo_startup_spin_angle_rad)
            goal.time_allowance.sec = 10
            goal.time_allowance.nanosec = 0
            # The spin is the bootstrap that makes the first candidate
            # snapshot possible; it must be sent after Nav2 readiness, before
            # requiring a candidate for allocation.  If a snapshot already
            # exists, retain it as the pre-spin baseline so it is not reused
            # after the spin completes.  A candidate-free startup simply uses
            # a zero baseline and accepts the first valid post-spin snapshot.
            baseline = self._candidates.get(self._robot_id)
            if baseline is None:
                self._solo_startup_spin_baseline_stamp_ns = 0
                self._solo_startup_spin_baseline_generation_id = 0
            else:
                self._solo_startup_spin_baseline_stamp_ns = (
                    self._message_stamp_ns(baseline))
                self._solo_startup_spin_baseline_generation_id = int(
                    getattr(baseline, 'candidate_generation_id', 0))
            self._solo_startup_spin_started = True
            self._solo_startup_spin_active = True
            self._allocation_reason = 'solo startup spin active'
            self._log_info(
                'MINIMAL_ALLOCATOR_SOLO_STARTUP_SPIN_SENT '
                f'robot={self._robot_id} action=/{self._robot_id}/spin '
                f'angle_rad={float(goal.target_yaw):.4f} '
                'collision_checks=enabled',
            )
            future = client.send_goal_async(goal)
            future.add_done_callback(self._on_solo_startup_spin_goal_response)
        except Exception as exc:  # noqa: B902 - startup motion is fail-closed
            self._finish_solo_startup_spin(
                False, f'solo startup spin setup error: {exc}')

    def _solo_startup_spin_ready_for_snapshot(
            self, message: FrontierCandidateArray) -> bool:
        """Gate the first solo evaluation behind one completed map spin."""
        if not self._solo_startup_spin_required():
            return True
        if not self._solo_startup_spin_started:
            self._start_solo_startup_spin()
            return False
        if self._solo_startup_spin_active or not self._solo_startup_spin_finished:
            return False
        if not self._solo_startup_spin_succeeded:
            return False
        if self._solo_startup_spin_waiting_for_snapshot:
            stamp_ns = self._message_stamp_ns(message)
            generation_id = int(getattr(message, 'candidate_generation_id', 0))
            if (
                    stamp_ns <= self._solo_startup_spin_baseline_stamp_ns and
                    generation_id <= self._solo_startup_spin_baseline_generation_id):
                return False
            self._solo_startup_spin_waiting_for_snapshot = False
            self._allocation_reason = (
                'solo startup spin complete; updated snapshot received')
        return True

    def _local_status_evidence(self) -> CandidateEvidence:
        return self._evidence[self._robot_id]

    def _publish_status(self) -> None:
        if self._status_publisher is None:
            return
        message = DistributedExplorationStatus()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.header.frame_id = self._global_frame
        message.source_robot_id = self._robot_id
        message.active_canonical_task_id = self._active_goal_id or ''
        message.local_nav_goal_active = bool(self._active_goal_id)
        states = {
            self.IDLE: message.WAITING_FOR_INPUTS,
            self.EVALUATING: message.BIDDING,
            self.WAITING_TRAFFIC: message.WAITING_FOR_TRAFFIC,
            self.GOAL_PENDING: message.BIDDING,
            self.NAVIGATING: message.NAVIGATING,
        }
        message.state = (
            message.COMPLETE if self._solo_terminal_latched else
            states[self._state]
        )
        message.union_hash = self._union.union_hash if self._union else ''
        message.round_id = message.union_hash
        message.terminal = self._terminal_reason is not None
        message.terminal_reason = self._terminal_reason or ''
        message.terminal_epoch = int(self._terminal_epoch)
        local = self._local_status_evidence()
        local_message = self._candidates[self._robot_id]
        message.terminal_map_revision = int(
            self._terminal_map_revision or
            (getattr(local_message, 'map_revision', 0) if local_message else 0)
        )
        message.remaining_frontier_count = int(local.detected)
        message.remaining_small_frontier_count = int(
            local.terminal_small or local.small)
        message.remaining_out_of_range_count = int(local.out_of_range)
        message.remaining_unreachable_count = int(local.unreachable)
        message.planner_failure_count = int(local.planner_failures)
        message.detected_not_queried_count = int(local.detected_not_queried)
        message.below_minimum_gain_count = int(local.below_minimum_gain)
        message.actionable_reachable_count = int(
            local.actionable_reachable
            if local.actionable_reachable is not None else local.reachable
        )
        message.feasible_work_available = bool(local.reachable)
        message.actionable_work_available = bool(
            message.actionable_reachable_count)
        message.work_availability_reason = (
            self._terminal_reason or self._allocation_reason or self._state)
        message.reason = (
            self._terminal_reason or self._allocation_reason or self._state)
        if self._nav is not None and callable(getattr(self._nav, 'health_flags', None)):
            try:
                message.nav2_healthy, message.tf_healthy = (
                    bool(value) for value in self._nav.health_flags())
            except Exception:  # noqa: B902 - status must remain publishable
                message.nav2_healthy = False
                message.tf_healthy = False
        message.candidate_source_healthy = self._solo_evidence_is_fresh()
        message.peer_communication_healthy = bool(
            self._peer_candidate_available and self._peer_status_current)
        self._status_publisher.publish(message)
        self._allocation_reason = ''

    def _invalidate_pending_goal(self) -> None:
        if self._state != self.GOAL_PENDING:
            return
        self._goal_token += 1
        self._active_goal_id = None
        self._solo_pending_preflight = None
        self._solo_preflight_callback = None
        self._solo_planner_pause_pending = False
        self._solo_planner_idle = True
        self._traffic_decision = None
        self._state = self.EVALUATING if self._released else self.IDLE

    def _local_batch_for(self, union):
        current_ids = {task.canonical_id for task in union.tasks}
        self._completed_ids.intersection_update(current_ids)
        self._local_completed_ids.intersection_update(current_ids)
        array = self._candidates[self._robot_id]
        bids = frontier_sets.materialize_bids(array, union)
        bids = tuple(
            replace(
                bid,
                path_valid=False,
                path_length_m=0.0,
                estimated_travel_cost=0.0,
                heading_cost=0.0,
                path=(),
            )
            if bid.canonical_task_id in self._completed_ids else bid
            for bid in bids
        )
        return protocol.make_batch(
            self._robot_id,
            self._source_session_id,
            int(array.candidate_generation_id),
            union,
            bids,
            self._bid_validity_s,
        )

    @staticmethod
    def _has_cost_evidence(message) -> bool:
        return any(
            candidate.reachability_state == candidate.REACHABLE
            for candidate in message.candidates
        )

    @staticmethod
    def _is_geometry_only(message) -> bool:
        return (
            bool(message.candidates) and
            not MinimalFrontierAllocator._has_cost_evidence(message) and
            bool(message.detected_not_queried_count)
        )

    def _passive_batch_for(self, union):
        previous = {
            bid.canonical_task_id: bid
            for bid in (self._local_batch.bids if self._local_batch else ())
        }
        batch = self._local_batch_for(union)
        active_id = self._active_goal_id
        active_bid = previous.get(active_id)
        if not active_id:
            return batch
        if active_bid is None:
            active_bid = next(
                (bid for bid in batch.bids if bid.canonical_task_id == active_id),
                None,
            )
        unavailable = tuple(
            replace(
                bid,
                path_valid=False,
                path_length_m=0.0,
                estimated_travel_cost=0.0,
                heading_cost=0.0,
                path=(),
            )
            if bid.canonical_task_id != active_id or active_bid is None else active_bid
            for bid in batch.bids
        )
        return replace(batch, bids=unavailable)

    def _costing_pending(self) -> bool:
        local = self._candidates[self._robot_id]
        if self._is_geometry_only(local):
            return True
        peer = self._candidates[self._peer_id]
        return self._peer_active_goal is None and self._is_geometry_only(peer)

    def _publish_batch(self) -> None:
        if self._local_batch is None:
            return
        if self._bid_publisher is not None:
            stamp = self._node.get_clock().now().to_msg()
            self._bid_publisher.publish(protocol.to_msg(self._local_batch, stamp))
        self._publish_completion_events()

    def _publish_completion_events(
            self, task_ids: tuple[str, ...] | None = None,
            union_hash: str | None = None) -> None:
        if self._event_publisher is None or self._union is None:
            return
        union_hash = union_hash or self._union.union_hash
        task_ids = task_ids or tuple(sorted(self._local_completed_ids))
        for task_id in task_ids:
            event = DistributedExplorationEvent()
            event.header.stamp = self._node.get_clock().now().to_msg()
            event.header.frame_id = 'shared_map'
            event.source_robot_id = self._robot_id
            event.event_type = 'NAVIGATION_SUCCEEDED'
            event.round_id = union_hash
            event.union_hash = union_hash
            event.canonical_task_id = task_id
            event.physical_task_signature = task_id
            event.result = event.event_type
            self._event_publisher.publish(event)

    def _candidate_array_basic_valid(
            self, message: FrontierCandidateArray) -> bool:
        """Validate peer presence without treating ROS graph presence as proof."""
        return bool(
            str(message.source_robot_id) == self._peer_id and
            str(message.header.frame_id) and
            int(getattr(message, 'candidate_generation_id', 0)) > 0 and
            self._message_stamp_ns(message) > 0
        )

    def _peer_candidate_context_compatible(self) -> bool:
        """Require the peer stream to use the current local map frame."""
        local = self._candidates[self._robot_id]
        peer = self._candidates[self._peer_id]
        return bool(
            local is not None and peer is not None and
            str(local.header.frame_id) and
            str(local.header.frame_id) == str(peer.header.frame_id)
        )

    def _peer_candidate_fresh(self) -> bool:
        received = self._candidate_received_steady_s.get(self._peer_id)
        return bool(
            self._peer_candidate_available and received is not None and
            time.monotonic() - float(received) <= self._peer_candidate_timeout_s
        )

    def _restore_solo_after_peer_loss(self) -> None:
        """Return to local work only before cooperative mode is established."""
        if (not self._allow_solo_without_peer or
                self._cooperative_active or self._active_goal_id is not None):
            return
        self._solo_terminal_latched = False
        self._terminal_reason = None
        self._terminal_map_revision = 0
        self._peer_candidate_available = False
        self._cooperative_pending = False
        self._solo_mode = True
        self._candidates[self._peer_id] = None
        self._peer_batch = None
        self._peer_status_union_hash = None
        self._peer_status_state = None
        self._peer_status_current = False
        self._peer_terminal = False
        self._peer_terminal_reason = ''
        self._union = None
        self._local_batch = None
        self._local_batch_from_costing = False
        self._last_solo_snapshot_key = None
        self._state = self.EVALUATING if self._released else self.IDLE
        self._log_info(
            'MINIMAL_ALLOCATOR_PEER_LOST_BEFORE_COOPERATIVE robot=%s '
            'mode=SOLO_LOCAL' % self._robot_id,
        )

    def _clear_solo_terminal_latch(self) -> None:
        """Leave solo completion only when a real peer context appears."""
        if not self._solo_terminal_latched:
            return
        self._solo_terminal_latched = False
        self._terminal_reason = None
        self._terminal_map_revision = 0
        self._state = self.EVALUATING if self._released else self.IDLE
        self._allocation_reason = 'fresh peer context available'

    def _update_peer_presence(self) -> None:
        if (self._allow_solo_without_peer and
                self._peer_candidate_available and
                not self._peer_candidate_fresh()):
            self._restore_solo_after_peer_loss()

    def _internal_empty_peer_array(self, local: FrontierCandidateArray):
        """Build an in-memory idle side; never publish this placeholder."""
        empty = FrontierCandidateArray()
        empty.header = local.header
        empty.source_robot_id = self._peer_id
        empty.map_revision = int(getattr(local, 'map_revision', 0))
        empty.candidate_generation_id = 0
        if self._robot_id == 'robot1':
            return local, empty
        return empty, local

    def _solo_union_for(self, local: FrontierCandidateArray):
        first, second = self._internal_empty_peer_array(local)
        return frontier_sets.build_union(first, second)

    @staticmethod
    def _solo_snapshot_key(message: FrontierCandidateArray):
        """Return the existing source snapshot identity used by solo work."""
        candidates = []
        for candidate in message.candidates:
            pose = candidate.approach_pose.pose
            candidates.append((
                int(candidate.frontier_id),
                float(pose.position.x), float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x), float(pose.orientation.y),
                float(pose.orientation.z), float(pose.orientation.w),
                float(getattr(candidate, 'local_path_length_m', 0.0) or
                      getattr(candidate, 'path_length_m', 0.0)),
            ))
        return (
            int(getattr(message, 'map_revision', 0)),
            int(getattr(message, 'costmap_revision', 0)),
            tuple(sorted(candidates)),
        )

    def _solo_safety_context_key(self, _message, candidate, _path):
        """Return stable semantic geometry for solo rejection quarantine.

        A geometry rejection belongs to this candidate geometry.  Rolling
        costmaps, map revisions, timestamps, and freshly replanned paths are
        deliberately excluded: they are volatile runtime observations and
        must not reopen the same rejected candidate on every publication.
        Transient readiness failures are not passed to this helper.
        """
        approach = candidate.approach_pose.pose
        orientation = approach.orientation
        footprint = getattr(self._nav, '_latest_footprint', None)
        footprint_key = ()
        if footprint is not None:
            footprint_key = tuple(
                (round(float(point.x), 5), round(float(point.y), 5))
                for point in footprint.polygon.points
            )
        semantic_path = tuple(
            (round(float(point.x), 5), round(float(point.y), 5))
            for point in getattr(candidate, 'local_path_samples', ())
        )
        return (
            str(candidate.frontier_id),
            str(getattr(candidate.approach_pose.header, 'frame_id', '')),
            tuple(round(float(value), 5) for value in (
                candidate.centroid.x, candidate.centroid.y,
                approach.position.x, approach.position.y,
                orientation.x, orientation.y, orientation.z,
                orientation.w,
            )),
            round(float(
                getattr(candidate, 'local_path_length_m', 0.0) or
                getattr(candidate, 'path_length_m', 0.0)), 5),
            round(float(getattr(candidate, 'heading_change_rad', 0.0)), 5),
            semantic_path,
            footprint_key,
        )

    def _solo_candidate_is_blocked(self, message, candidate, path) -> bool:
        blocked = getattr(self, '_solo_preflight_blocked', {})
        candidate_id = str(candidate.frontier_id)
        if candidate_id not in blocked:
            return False
        context = blocked[candidate_id]
        return context == self._solo_safety_context_key(message, candidate, path)

    def _mark_solo_preflight_blocked(
            self, message, candidate, path, result: DispatchPreconditions) -> None:
        if not self._allow_solo_without_peer or not self._solo_mode:
            return
        if not hasattr(self, '_solo_preflight_blocked'):
            self._solo_preflight_blocked = {}
        context = self._solo_safety_context_key(message, candidate, path)
        self._solo_preflight_blocked[str(candidate.frontier_id)] = context
        self._log_info(
            'MINIMAL_ALLOCATOR_SOLO_PREFLIGHT_BLOCKED '
            f'frontier_id={int(candidate.frontier_id)} '
            f'reason={getattr(result, "reason", "PRECONDITION_REJECTED")} '
            'retry=on_safety_context_change',
        )

    @classmethod
    def _solo_geometry_rejection_reason(cls, result) -> str:
        """Return a quarantine reason only for definitive geometry failures."""
        for field in (
                'reason', 'path_valid_reason', 'local_footprint_reason',
                'local_path_reason'):
            value = str(getattr(result, field, '') or '')
            for reason in cls.SOLO_GEOMETRY_REJECTION_REASONS:
                if reason in value:
                    return reason
        return ''

    @staticmethod
    def _solo_preflight_is_transient(result) -> bool:
        """Keep readiness failures attached to the selected solo task."""
        if bool(getattr(result, 'ready', False)):
            return False
        try:
            return classify_dispatch_precondition_failure(result) == (
                FailureClass.TF_OR_LIFECYCLE)
        except (AttributeError, TypeError, ValueError):
            reason = str(getattr(result, 'reason', '') or '').lower()
            return any(token in reason for token in (
                'transform', 'tf ', 'costmap', 'lifecycle',
                'action server', 'unavailable', 'stale', 'timeout',
            ))

    def _retry_pending_solo_preflight(self) -> None:
        """Retry the retained solo task once readiness is current."""
        pending = self._solo_pending_preflight
        if (
                pending is None or not self._allow_solo_without_peer or
                not self._solo_mode or self._state != self.GOAL_PENDING or
                self._active_goal_id is None):
            return
        if not self._solo_nav_readiness_is_current():
            return
        if not self._solo_follow_path_server_ready():
            return
        inputs_ready = getattr(self._nav, 'preflight_inputs_available', None)
        if callable(inputs_ready) and not inputs_ready():
            return
        candidate, task, path, token = pending
        self._dispatch(
            candidate,
            path,
            _retry_token=token,
            _retry_task=task,
        )

    def _solo_batch_without_blocked(self, message, batch):
        """Mask only currently blocked solo tasks from the existing selector."""
        if batch is None:
            return batch
        candidates = {
            str(candidate.frontier_id): candidate
            for candidate in message.candidates
        }
        blocked_ids = set()
        for bid in batch.bids:
            candidate = candidates.get(str(bid.canonical_task_id))
            if candidate is None:
                continue
            if self._solo_candidate_is_blocked(
                    message, candidate, self._bid_path(batch, bid.canonical_task_id)):
                blocked_ids.add(str(bid.canonical_task_id))
        if not blocked_ids:
            return batch
        masked = tuple(
            replace(
                bid,
                path_valid=False,
                path_length_m=0.0,
                estimated_travel_cost=0.0,
                heading_cost=0.0,
                path=(),
            ) if str(bid.canonical_task_id) in blocked_ids else bid
            for bid in batch.bids
        )
        return replace(batch, bids=masked)

    def _try_allocate_solo(self, *, allow_consumed_snapshot: bool = False) -> None:
        """Select one local task with the existing cost-only pair scorer."""
        if (not self._allow_solo_without_peer or not self._solo_mode or
                not self._released or self._terminal_reason is not None or
                self._active_goal_id is not None or
                self._state == self.GOAL_PENDING):
            return
        if self._navigation_goal_cap_reached():
            self._state = self.IDLE
            return
        local = self._candidates[self._robot_id]
        if local is None:
            return
        if not self._solo_startup_spin_ready_for_snapshot(local):
            return
        snapshot_key = self._solo_snapshot_key(local)
        if (snapshot_key == self._last_solo_snapshot_key and
                not allow_consumed_snapshot):
            return
        self._last_solo_snapshot_key = snapshot_key
        union = self._solo_union_for(local)
        if union is None:
            return
        self._union = union
        self._local_batch = self._local_batch_for(union)
        self._local_batch_from_costing = self._has_cost_evidence(local)
        self._publish_batch()
        self._publish_status()
        internal_peer_batch = protocol.make_batch(
            self._peer_id, 'solo-internal-idle', 0, union, (), 0.0,
        )
        local_batch_for_selection = self._solo_batch_without_blocked(
            local, self._local_batch,
        )
        if self._robot_id == 'robot1':
            robot1_batch, robot2_batch = local_batch_for_selection, internal_peer_batch
        else:
            robot1_batch, robot2_batch = internal_peer_batch, local_batch_for_selection
        try:
            assignment = selection.choose_assignment(
                union, robot1_batch, robot2_batch,
            )
        except (TypeError, ValueError):
            return
        if assignment is None:
            self._state = self.IDLE
            self._allocation_reason = 'no eligible solo candidate after preflight'
            self._publish_status()
            return
        local_id = assignment[0] if self._robot_id == 'robot1' else assignment[1]
        if not local_id or local_id in self._completed_ids:
            self._state = self.IDLE
            self._allocation_reason = 'no eligible solo candidate after preflight'
            self._publish_status()
            return
        candidate = next(
            (
                item for item in local.candidates
                if str(item.frontier_id) == str(local_id)
            ),
            None,
        )
        if candidate is None:
            return
        self._state = self.EVALUATING
        self._allocation_reason = 'local-only frontier_cost_only selection'
        self._log_info(
            'MINIMAL_ALLOCATOR_SOLO_SELECTED robot=%s frontier_id=%s '
            'centroid=(%.6f,%.6f) approach=(%.6f,%.6f) frame=%s '
            'dispatch_enabled=%s' % (
                self._robot_id, int(candidate.frontier_id),
                float(candidate.centroid.x), float(candidate.centroid.y),
                float(candidate.approach_pose.pose.position.x),
                float(candidate.approach_pose.pose.position.y),
                str(candidate.approach_pose.header.frame_id),
                self._dispatch_enabled,
            ),
        )
        if not self._dispatch_enabled:
            self._state = self.IDLE
            self._publish_status()
            return
        path = self._bid_path(self._local_batch, str(local_id))
        self._dispatch(candidate, path)

    def on_candidate_array(self, message: FrontierCandidateArray) -> None:
        source = str(message.source_robot_id)
        if source not in self._candidates:
            return
        if (self._allow_solo_without_peer and source == self._peer_id and
                not self._candidate_array_basic_valid(message)):
            return
        evidence = self._make_evidence(message)
        first_local_snapshot = (
            self._allow_solo_without_peer and source == self._robot_id and
            not self._solo_evidence_seen
        )
        if evidence != self._evidence[source] or first_local_snapshot:
            self._stable_since_s = self._now_s()
        self._candidates[source] = message
        self._evidence[source] = evidence
        self._candidate_received_steady_s[source] = time.monotonic()
        if self._allow_solo_without_peer and source == self._robot_id:
            self._solo_evidence_seen = True
        if self._allow_solo_without_peer and source == self._peer_id:
            if self._peer_candidate_context_compatible():
                self._clear_solo_terminal_latch()
                self._peer_candidate_available = True
                self._cooperative_pending = True
                self._solo_mode = False
            else:
                self._peer_candidate_available = False
                return
        elif (self._allow_solo_without_peer and source == self._robot_id and
              not self._cooperative_active):
            peer = self._candidates[self._peer_id]
            if (peer is not None and
                    self._candidate_array_basic_valid(peer) and
                    self._peer_candidate_context_compatible()):
                self._clear_solo_terminal_latch()
                self._peer_candidate_available = True
                self._cooperative_pending = True
                self._solo_mode = False
            else:
                self._solo_mode = True
        if (self._allow_solo_without_peer and self._solo_mode and
                source == self._robot_id):
            if not self._solo_startup_spin_ready_for_snapshot(message):
                self._state = self.EVALUATING if self._released else self.IDLE
                self._publish_status()
                return
            self._try_allocate_solo()
            self._maybe_solo_terminal()
            return
        if (self._allow_solo_without_peer and source == self._peer_id and
                not self._peer_candidate_available):
            return
        first = self._candidates['robot1']
        second = self._candidates['robot2']
        if first is None or second is None:
            return
        self._publish_start_ready()

        union = frontier_sets.build_union(first, second)
        if union is None:
            self._invalidate_pending_goal()
            self._union = None
            self._local_batch = None
            self._peer_batch = None
            self._peer_status_union_hash = None
            self._peer_status_state = None
            self._peer_status_current = False
            self._peer_terminal = False
            self._peer_terminal_reason = ''
            self._terminal_reason = None
            if self._active_goal_id is None:
                self._state = self.EVALUATING if self._released else self.IDLE
            self._publish_status()
            return

        changed = self._union is None or union.union_hash != self._union.union_hash
        if changed:
            self._invalidate_pending_goal()
            self._stable_since_s = self._now_s()
            self._peer_batch = None
            self._peer_status_union_hash = None
            self._peer_status_state = None
            self._peer_status_current = False
            self._peer_terminal = False
            self._peer_terminal_reason = ''
            self._terminal_reason = None
            self._solo_terminal_latched = False
        self._union = union
        if self._active_goal_id is not None:
            if changed or self._local_batch is None:
                self._local_batch = self._passive_batch_for(union)
            self._publish_batch()
            self._publish_status()
            return
        geometry_only = self._is_geometry_only(message)
        same_batch_union = (
            self._local_batch is not None and
            self._local_batch.union_hash == union.union_hash
        )
        keep_cost_batch = (
            geometry_only and same_batch_union and
            self._local_batch_from_costing
        )
        if not keep_cost_batch:
            self._local_batch = self._local_batch_for(union)
            self._local_batch_from_costing = not geometry_only
            self._publish_batch()
        else:
            self._publish_completion_events()
        if self._active_goal_id is None and self._state != self.GOAL_PENDING:
            self._state = self.EVALUATING if self._released else self.IDLE
        self._publish_status()
        self._maybe_terminal()
        self._try_allocate()

    def _publish_start_ready(self) -> None:
        if self._start_ready_publisher is None or self._start_ready_published:
            return
        message = String()
        message.data = json.dumps({
            'event': 'COOPERATIVE_START_STATE_READY',
            'robot_id': self._robot_id,
            'traffic_scheduler_ready': True,
        }, sort_keys=True, separators=(',', ':'))
        self._start_ready_publisher.publish(message)
        self._start_ready_published = True

    def on_peer_bid(self, message: TaskBidArray) -> None:
        if (not self._peer_bid_after_terminal and
                self._message_stamp_ns(message) <
                int(self._stable_since_s * 1e9)):
            return
        try:
            batch = protocol.from_msg(message)
        except (TypeError, ValueError):
            return
        if batch.source_robot_id != self._peer_id:
            return
        if self._union is None or batch.union_hash != self._union.union_hash:
            return
        if self._allow_solo_without_peer:
            self._cooperative_active = True
            self._cooperative_pending = False
            self._solo_mode = False
        self._peer_batch = batch
        self._peer_bid_after_terminal = True
        self._publish_completion_events()
        self._try_allocate()

    def on_peer_status(self, message: DistributedExplorationStatus) -> None:
        if message.source_robot_id != self._peer_id:
            return
        if (not self._peer_status_after_terminal and
                self._message_stamp_ns(message) <
                int(self._stable_since_s * 1e9)):
            return
        if self._union is None:
            return
        if (str(message.union_hash) != self._union.union_hash or
                str(message.round_id) != self._union.union_hash):
            return
        if self._allow_solo_without_peer:
            self._cooperative_active = True
            self._cooperative_pending = False
            self._solo_mode = False
        if message.local_nav_goal_active and not message.terminal:
            task_id = str(message.active_canonical_task_id)
            self._peer_active_goal = task_id or None
        else:
            self._peer_active_goal = None
        self._peer_status_union_hash = self._union.union_hash
        self._peer_status_state = int(message.state)
        self._peer_terminal = bool(message.terminal)
        self._peer_terminal_reason = str(message.terminal_reason)
        self._peer_status_after_terminal = True
        self._peer_status_current = True
        self._maybe_terminal()
        self._try_allocate()

    def on_peer_event(self, message: DistributedExplorationEvent) -> None:
        if message.source_robot_id != self._peer_id:
            return
        if message.event_type != 'NAVIGATION_SUCCEEDED':
            return
        if self._union is None or message.union_hash != self._union.union_hash:
            return
        if self._allow_solo_without_peer:
            self._cooperative_active = True
            self._cooperative_pending = False
            self._solo_mode = False
        current_ids = {task.canonical_id for task in self._union.tasks}
        task_id = str(message.canonical_task_id)
        if task_id not in current_ids:
            return
        self._completed_ids.add(task_id)
        if self._active_goal_id is not None:
            self._local_batch = self._passive_batch_for(self._union)
        else:
            self._local_batch = self._local_batch_for(self._union)
            self._local_batch_from_costing = self._has_cost_evidence(
                self._candidates[self._robot_id]
            )
        self._publish_batch()
        self._publish_status()
        self._try_allocate()

    def on_start_release(self, message: String) -> None:
        try:
            released = json.loads(str(message.data)).get('event') == 'START_RELEASE'
        except (TypeError, ValueError, json.JSONDecodeError):
            released = False
        if released:
            self._released = True
            self._try_allocate()

    @staticmethod
    def _bid_path(batch, task_id):
        if batch is None or not task_id:
            return ()
        for bid in batch.bids:
            if bid.canonical_task_id == task_id:
                return bid.path
        return ()

    def _try_allocate(self) -> None:
        if not self._released:
            return
        if getattr(self, '_solo_startup_spin_active', False):
            return
        if self._terminal_reason is not None:
            return
        if self._union is None or self._local_batch is None:
            return
        if self._peer_batch is None or self._active_goal_id is not None:
            return
        if not self._peer_status_after_terminal:
            return
        if not self._peer_status_current:
            return
        if not self._peer_bid_after_terminal:
            return
        if self._state == self.GOAL_PENDING:
            return

        if self._robot_id == 'robot1':
            robot1_batch = self._local_batch
            robot2_batch = self._peer_batch
        else:
            robot1_batch = self._peer_batch
            robot2_batch = self._local_batch
        if not protocol.complete_pair(self._union, robot1_batch, robot2_batch):
            return
        if self._costing_pending():
            self._state = self.EVALUATING
            return

        known_ids = {task.canonical_id for task in self._union.tasks}
        peer_active = self._peer_active_goal
        peer_active_in_union = peer_active in known_ids
        if not peer_active_in_union:
            peer_active = ''
        peer_active_is_valid = any(
            bid.canonical_task_id == peer_active and bid.path_valid
            for bid in self._peer_batch.bids
        )
        if peer_active_in_union and not peer_active_is_valid:
            return
        selector_peer_active = peer_active if peer_active_is_valid else ''
        if self._robot_id == 'robot1':
            active1, active2 = '', selector_peer_active
        else:
            active1, active2 = selector_peer_active, ''
        self._state = self.EVALUATING
        assignment = selection.choose_assignment(
            self._union,
            robot1_batch,
            robot2_batch,
            active_robot1_id=active1,
            active_robot2_id=active2,
        )
        if assignment is None:
            self._state = self.IDLE
            self._publish_status()
            return

        if self._robot_id == 'robot1':
            local_id = assignment[0]
        else:
            local_id = assignment[1]
        if not local_id or local_id in self._completed_ids:
            self._state = self.IDLE
            self._publish_status()
            self._maybe_terminal()
            return
        if local_id == peer_active:
            self._state = self.IDLE
            self._publish_status()
            return

        path1 = self._bid_path(robot1_batch, assignment[0])
        path2 = self._bid_path(robot2_batch, assignment[1])
        active_robots = frozenset(
            robot
            for robot, task_id in (
                ('robot1', active1),
                ('robot2', active2),
            )
            if task_id
        )
        self._traffic_decision = traffic.decide(
            path1,
            path2,
            robot1_safe_radius_m=self._safe_radius,
            robot2_safe_radius_m=self._safe_radius,
            reference_speed_mps=self._reference_speed,
            eta_tie_s=self._eta_tie_s,
            active_robots=active_robots,
        )
        if traffic.should_wait(self._robot_id, self._traffic_decision):
            self._state = self.WAITING_TRAFFIC
            self._publish_status()
            return

        candidate = next(
            (
                item
                for item in self._candidates[self._robot_id].candidates
                if str(item.frontier_id) == local_id
            ),
            None,
        )
        if candidate is not None:
            path = path1 if self._robot_id == 'robot1' else path2
            self._dispatch(candidate, path)

    def _dispatch(
            self, candidate, path, *, _retry_token=None, _retry_task=None,
    ) -> None:
        if (not self._dispatch_enabled or self._nav is None or
                (self._active_goal_id is not None and _retry_token is None)):
            return
        if self._navigation_goal_cap_reached():
            self._cancel_pending_preflight()
            self._allocation_reason = 'max_navigation_goals reached'
            self._state = self.IDLE
            self._publish_status()
            return
        task = _retry_task or navigation.to_physical_task(
            candidate, self._robot_id)
        if self._allow_solo_without_peer and self._solo_mode:
            task = replace(
                task, planned_path=getattr(candidate, 'planned_path', None))
        pending_union_hash = self._union.union_hash if self._union else ''
        pending_task_id = str(candidate.frontier_id)
        dispatch_path = tuple(path)
        if _retry_token is None:
            self._goal_token += 1
            token = self._goal_token
            self._active_goal_id = str(candidate.frontier_id)
            self._active_goal_union_hash = pending_union_hash
            self._state = self.GOAL_PENDING
        else:
            token = int(_retry_token)
            if (
                    token != self._goal_token or
                    self._active_goal_id != pending_task_id or
                    self._state != self.GOAL_PENDING):
                return
        self._local_batch = self._passive_batch_for(self._union)
        send_started = False
        if self._allow_solo_without_peer and self._solo_mode:
            self._solo_pending_preflight = (
                candidate, task, dispatch_path, token)
            if (_retry_token is None and getattr(
                    self, '_solo_planner_arbitration_enabled', False)):
                self._solo_planner_pause_pending = True
                self._solo_planner_idle = False

        def precondition_result(result: DispatchPreconditions) -> None:
            nonlocal send_started
            if token != self._goal_token or self._state != self.GOAL_PENDING:
                return
            if (self._union is None or
                    self._union.union_hash != pending_union_hash or
                    self._active_goal_id != pending_task_id):
                self._clear_goal(token)
                return
            if self._peer_active_goal == pending_task_id:
                self._clear_goal(token)
                return
            if self._navigation_goal_cap_reached():
                self._cancel_pending_preflight()
                self._allocation_reason = 'max_navigation_goals reached'
                self._clear_goal(token)
                self._state = self.IDLE
                self._publish_status()
                return
            if send_started:
                return
            solo_rejection = (
                self._allow_solo_without_peer and self._solo_mode)
            if solo_rejection and result.ready and not self._solo_follow_path_server_ready():
                self._solo_pending_preflight = (
                    candidate, task, tuple(dispatch_path), token)
                self._allocation_reason = (
                    'solo preflight retryable: FollowPath action server unavailable')
                self._state = self.GOAL_PENDING
                self._publish_status()
                return
            if not result.ready:
                geometry_reason = (
                    self._solo_geometry_rejection_reason(result)
                    if solo_rejection else '')
                if solo_rejection and geometry_reason:
                    self._solo_pending_preflight = None
                    self._mark_solo_preflight_blocked(
                        self._candidates[self._robot_id], candidate,
                        tuple(path), result,
                    )
                elif solo_rejection and self._solo_preflight_is_transient(result):
                    self._solo_pending_preflight = (
                        candidate, task, tuple(dispatch_path), token)
                    self._allocation_reason = (
                        'solo preflight retryable: ' +
                        (str(getattr(result, 'reason', '')) or
                         'transient failure'))
                    self._state = self.GOAL_PENDING
                    self._publish_status()
                    return
                elif solo_rejection:
                    # Startup/readiness failures are retryable and must not
                    # quarantine the candidate or recurse into a retry storm.
                    self._last_solo_snapshot_key = None
                    self._allocation_reason = (
                        'solo preflight retryable: ' +
                        (str(getattr(result, 'reason', '')) or 'transient failure'))
                self._clear_goal(token)
                if solo_rejection and geometry_reason:
                    self._try_allocate_solo(allow_consumed_snapshot=True)
                return
            self._solo_pending_preflight = None
            send_started = True
            self._state = self.NAVIGATING
            self._publish_status()
            if self._allow_solo_without_peer and self._solo_mode:
                sent = self._send_solo_follow_path(
                    task, lambda outcome: self.on_navigation_outcome(token, outcome))
            else:
                sent = navigation.send(
                    self._nav,
                    task,
                    lambda outcome: self.on_navigation_outcome(token, outcome),
                    dispatch_path,
                )
            if sent:
                self._navigation_goal_count += 1
                if self._allow_solo_without_peer and self._solo_mode:
                    self._solo_active_dispatch = (
                        self._candidates[self._robot_id],
                        candidate,
                        dispatch_path,
                    )
                self._log_info(
                    'MINIMAL_ALLOCATOR_NAVIGATION_DISPATCH '
                    f'robot={self._robot_id} task_id={pending_task_id} '
                    f'union_hash={pending_union_hash} sim_time={self._now_s():.6f}'
                )
            if not sent:
                self._clear_goal(token)

        if (self._allow_solo_without_peer and self._solo_mode and
                getattr(self, '_solo_planner_arbitration_enabled', False)):
            self._solo_preflight_callback = precondition_result
        self._publish_batch()
        self._publish_status()

        if (not self._allow_solo_without_peer or
                not self._solo_mode):
            if self._solo_mode:
                self._nav.check_dispatch_preconditions(
                    task,
                    bool(task.local_path_valid),
                    precondition_result,
                    tuple(path),
                    self._global_frame,
                )
            else:
                navigation.check_preconditions(
                    self._nav,
                    task,
                    bool(task.local_path_valid),
                    precondition_result,
                    tuple(path),
                    'shared_map',
                )
            return

        # Solo mode trusts the candidate generator's completed
        # completed generator path result.  Keep only the readiness needed to submit
        # FollowPath; Nav2/RPP owns live collision and footprint handling.
        if (getattr(self, '_solo_planner_arbitration_enabled', False) and
                not self._solo_planner_idle):
            self._log_info(
                'MINIMAL_ALLOCATOR_WAITING_FOR_PLANNER_IDLE '
                f'task_id={pending_task_id}')
            return
        self._nav.check_navigation_readiness(precondition_result)

    def on_local_event(self, message: DistributedExplorationEvent) -> None:
        if (
                message.source_robot_id != self._robot_id or
                message.event_type != 'PLANNER_QUERY_IDLE' or
                not self._solo_planner_pause_pending or
                self._active_goal_id is None or
                self._state != self.GOAL_PENDING):
            return
        self._solo_planner_idle = True
        callback = self._solo_preflight_callback
        self._solo_preflight_callback = None
        self._log_info(
            'MINIMAL_ALLOCATOR_PLANNER_IDLE_ACK '
            f'reason={message.reason or "unspecified"}')
        if callback is not None:
            self._nav.check_navigation_readiness(callback)

    def _solo_follow_path_server_ready(self) -> bool:
        client = getattr(self, '_solo_follow_path_client', None)
        return bool(client is not None and client.server_is_ready())

    def _solo_follow_path_result(
            self, accepted: bool, status: int, error_code: int,
            error_message: str = '') -> None:
        callback = self._solo_follow_path_callback
        self._solo_follow_path_callback = None
        self._solo_follow_path_goal_handle = None
        duration = max(
            0.0, time.monotonic() - self._solo_follow_path_started_steady_s)
        current_distance = float(getattr(
            self._nav, 'travelled_distance_m',
            self._solo_follow_path_start_distance_m))
        travelled = max(
            0.0, current_distance - self._solo_follow_path_start_distance_m)
        succeeded = (
            accepted and status == GoalStatus.STATUS_SUCCEEDED and
            int(error_code) == int(FollowPath.Result.NONE))
        failure_class = (
            FailureClass.UNKNOWN if succeeded else
            classify_follow_path_controller_error(error_code))
        error_name = follow_path_controller_error_name(error_code)
        outcome = NavigationOutcome(
            bool(accepted), int(status), int(error_code), str(error_message),
            failure_class, duration, travelled, 0,
            error_name, int(error_code), error_name,
            'CONTROLLER_EXECUTION' if not succeeded and error_name else '',
            error_name if not succeeded and error_name else '',
            self._node.get_clock().now().nanoseconds,
            '',
        )
        if callback is not None:
            callback(outcome)

    def _solo_follow_path_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            self._solo_follow_path_result(
                False, GoalStatus.STATUS_UNKNOWN, 0,
                'FollowPath goal response exception: ' + str(error))
            return
        if goal_handle is None or not goal_handle.accepted:
            self._solo_follow_path_result(
                False, GoalStatus.STATUS_UNKNOWN, 0,
                'FollowPath goal rejected')
            return
        self._solo_follow_path_goal_handle = goal_handle
        if self._solo_follow_path_cancel_requested:
            goal_handle.cancel_goal_async()
        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._solo_follow_path_result_done)
        except Exception as error:  # noqa: B902
            self._solo_follow_path_result(
                True, GoalStatus.STATUS_UNKNOWN, 0,
                'FollowPath result setup exception: ' + str(error))

    def _solo_follow_path_result_done(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            self._solo_follow_path_result(
                True, int(wrapped.status), int(result.error_code),
                str(result.error_msg))
        except Exception as error:  # noqa: B902
            self._solo_follow_path_result(
                True, GoalStatus.STATUS_UNKNOWN, 0,
                'FollowPath result exception: ' + str(error))

    def _send_solo_follow_path(self, task, callback) -> bool:
        client = getattr(self, '_solo_follow_path_client', None)
        planned_path = getattr(task, 'planned_path', None)
        if (
                client is None or not client.server_is_ready() or
                planned_path is None or not planned_path.poses):
            return False
        goal = FollowPath.Goal()
        goal.path = planned_path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        self._solo_follow_path_callback = callback
        self._solo_follow_path_started_steady_s = time.monotonic()
        self._solo_follow_path_start_distance_m = float(getattr(
            self._nav, 'travelled_distance_m', 0.0))
        self._solo_follow_path_cancel_requested = False
        future = client.send_goal_async(goal)
        future.add_done_callback(self._solo_follow_path_goal_response)
        self._log_info(
            'MINIMAL_ALLOCATOR_SOLO_FOLLOW_PATH_SENT '
            f'poses={len(planned_path.poses)} '
            f'frame={planned_path.header.frame_id} '
            f'controller_id={goal.controller_id} '
            f'goal_checker_id={goal.goal_checker_id} '
            f'progress_checker_id={goal.progress_checker_id}')
        return True

    def _navigation_goal_cap_reached(self) -> bool:
        limit = int(getattr(self, '_max_navigation_goals', 0))
        return limit > 0 and int(getattr(
            self, '_navigation_goal_count', 0)) >= limit

    def _cancel_pending_preflight(self) -> None:
        if self._nav is None:
            return
        cancel = getattr(self._nav, 'cancel_pending_preflight', None)
        if callable(cancel):
            cancel()

    def on_navigation_outcome(
            self, token: int, outcome: NavigationOutcome) -> None:
        if token != self._goal_token or self._active_goal_id is None:
            return
        solo_failed_dispatch = None
        if (
                self._allow_solo_without_peer and self._solo_mode and
                outcome.status not in (
                    GoalStatus.STATUS_SUCCEEDED,
                    GoalStatus.STATUS_CANCELED,
                )
        ):
            solo_failed_dispatch = self._solo_active_dispatch
        if outcome.accepted and outcome.status == GoalStatus.STATUS_SUCCEEDED:
            completed_id = self._active_goal_id
            self._completed_ids.add(completed_id)
            self._local_completed_ids.add(completed_id)
            self._publish_completion_events(
                (completed_id,), self._active_goal_union_hash,
            )
        if outcome.status == GoalStatus.STATUS_SUCCEEDED:
            self._allocation_reason = 'goal success'
        elif outcome.status == GoalStatus.STATUS_CANCELED:
            self._allocation_reason = 'goal cancel/watchdog'
        else:
            self._allocation_reason = 'goal failure'
        self._log_info(
            'MINIMAL_ALLOCATOR_NAVIGATION_TERMINAL '
            f'robot={self._robot_id} status={outcome.status} '
            f'sim_time={self._now_s():.6f}'
        )
        self._peer_status_after_terminal = False
        self._peer_status_current = False
        self._peer_bid_after_terminal = False
        self._stable_since_s = self._now_s()
        self._clear_goal(token)
        self._solo_active_dispatch = None
        if solo_failed_dispatch is not None:
            message, candidate, path = solo_failed_dispatch
            if message is not None:
                self._solo_preflight_blocked[str(candidate.frontier_id)] = (
                    self._solo_safety_context_key(message, candidate, path)
                )
                self._log_info(
                    'MINIMAL_ALLOCATOR_SOLO_FAILURE_BLOCKED '
                    f'frontier_id={int(candidate.frontier_id)} '
                    'retry=on_safety_context_change'
                )
        if self._solo_mode:
            self._last_solo_snapshot_key = None
        if self._navigation_goal_cap_reached():
            self._state = self.IDLE
            self._allocation_reason = 'max_navigation_goals reached'
            self._publish_status()

    def _clear_goal(self, token: int) -> None:
        if token != self._goal_token:
            return
        self._goal_token += 1
        pending = self._solo_pending_preflight
        if pending is not None and pending[3] == token:
            self._solo_pending_preflight = None
        self._active_goal_id = None
        self._active_goal_union_hash = None
        self._solo_preflight_callback = None
        self._solo_planner_pause_pending = False
        self._solo_planner_idle = True
        self._traffic_decision = None
        self._state = self.EVALUATING if self._released else self.IDLE
        self._local_batch = None
        self._local_batch_from_costing = False
        self._publish_status()

    def cancel_active(self) -> bool:
        if self._nav is None or self._active_goal_id is None:
            return False
        if self._allow_solo_without_peer and self._solo_mode:
            self._solo_follow_path_cancel_requested = True
            if self._solo_follow_path_goal_handle is not None:
                self._solo_follow_path_goal_handle.cancel_goal_async()
                return True
            return True
        return navigation.cancel(self._nav)

    def _maybe_terminal(self) -> None:
        if self._allow_solo_without_peer and self._solo_mode:
            return
        reason = None
        if (self._active_goal_id is None and
                self._state not in (self.GOAL_PENDING, self.WAITING_TRAFFIC) and
                self._union is not None and
                self._candidates['robot1'] is not None and
                self._candidates['robot2'] is not None and
                self._peer_status_union_hash == self._union.union_hash and
                self._peer_active_goal is None and
                self._peer_state_allows_terminal() and
                self._local_and_peer_batches_complete()):
            classified = termination.classify(
                self._evidence['robot1'],
                self._evidence['robot2'],
                stable_for_s=max(0.0, self._now_s() - self._stable_since_s),
                stability_grace_s=self._stability_grace_s,
            )
            if classified is not None:
                local_reason = str(getattr(classified, 'value', classified))
                reason = termination.matching(
                    True, local_reason, self._peer_terminal,
                    self._peer_terminal_reason,
                )
        if reason != self._terminal_reason:
            self._terminal_reason = reason
            if reason is not None:
                self._terminal_epoch += 1
                local_message = self._candidates[self._robot_id]
                self._terminal_map_revision = int(
                    getattr(local_message, 'map_revision', 0)
                    if local_message is not None else 0
                )
            self._publish_status()

    def _maybe_solo_terminal(self) -> None:
        """Publish one stable local completion status without a peer."""
        if (not self._allow_solo_without_peer or not self._solo_mode or
                self._solo_terminal_latched or not self._solo_evidence_seen or
                self._active_goal_id is not None or
                self._state in (self.GOAL_PENDING, self.WAITING_TRAFFIC) or
                not self._solo_evidence_is_fresh() or
                not self._solo_nav_readiness_is_current()):
            return
        if (
                self._solo_startup_spin_required() and
                not self._solo_startup_spin_ready_for_snapshot(
                    self._candidates[self._robot_id])):
            return
        classified = termination.classify(
            self._evidence[self._robot_id],
            CandidateEvidence(),
            stable_for_s=max(0.0, self._now_s() - self._stable_since_s),
            stability_grace_s=self._stability_grace_s,
        )
        if classified is None:
            return
        reason = str(getattr(classified, 'value', classified))
        self._terminal_reason = reason
        self._solo_terminal_latched = True
        self._terminal_epoch += 1
        local_message = self._candidates[self._robot_id]
        self._terminal_map_revision = int(
            getattr(local_message, 'map_revision', 0)
            if local_message is not None else 0
        )
        self._state = self.IDLE
        self._allocation_reason = reason
        self._log_info(
            'MINIMAL_ALLOCATOR_SOLO_COMPLETE '
            f'robot={self._robot_id} reason={reason} '
            f'epoch={self._terminal_epoch} '
            f'map_revision={self._terminal_map_revision}',
        )
        self._publish_status()

    def _peer_state_allows_terminal(self) -> bool:
        return self._peer_status_state in (
            DistributedExplorationStatus.WAITING_FOR_INPUTS,
            DistributedExplorationStatus.COMPLETE,
        )

    def _local_and_peer_batches_complete(self) -> bool:
        if self._local_batch is None or self._peer_batch is None:
            return False
        if self._robot_id == 'robot1':
            robot1_batch, robot2_batch = self._local_batch, self._peer_batch
        else:
            robot1_batch, robot2_batch = self._peer_batch, self._local_batch
        return protocol.complete_pair(self._union, robot1_batch, robot2_batch)

    def _tick(self) -> None:
        self._update_peer_presence()
        if (self._solo_startup_spin_required() and
                not self._solo_startup_spin_started):
            self._start_solo_startup_spin()
        self._maybe_solo_terminal()
        self._maybe_terminal()
        if self._solo_pending_preflight is not None:
            self._retry_pending_solo_preflight()
        elif self._state in (self.EVALUATING, self.WAITING_TRAFFIC):
            self._try_allocate()


def main(args=None):
    import rclpy

    rclpy.init(args=args)
    node = Node('minimal_frontier_allocator')
    MinimalFrontierAllocator(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
