"""Regression coverage for sequential local-only frontier dispatch."""

import time
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from my_epuck_interfaces.msg import FrontierCandidate
from my_epuck_project.distributed_assignment.local_nav2 import (
    NavigationOutcome,
    PathEvaluation,
)
from my_epuck_project.distributed_assignment.models import (
    Bounds,
    FailureClass,
    PhysicalTask,
    TaskSnapshot,
)
from my_epuck_project.distributed_assignment.scoring import AssignmentWeights
from my_epuck_project.distributed_frontier_assignment import (
    ActiveNavigationAction,
    DistributedFrontierAssignment,
    classify_solo_dispatch_failure,
    eligible_solo_tasks,
)
from my_epuck_project.minimal_frontier_navigation import to_physical_task
from my_epuck_project.round_lifecycle import RoundGeneration


class _Logger:
    """Small logger double for the ROS-free allocator lifecycle test."""

    def info(self, *_args, **_kwargs):
        pass

    warning = info
    error = info


class _Nav2:
    """Immediate successful planner/precondition double."""

    def __init__(self):
        self.local_goal_active = False

    def evaluate_path(self, task, callback, caller):
        del caller
        path = ((0.0, 0.0), task.approach)
        callback(PathEvaluation(
            valid=True,
            length_m=task.local_path_length_m,
            samples=path,
            query_ros_ns=1,
            error_code=0,
            error_message='',
            failure_class=FailureClass.UNKNOWN,
            caller='DEGRADED_SOLO_DISPATCH',
            task_signature=task.physical_signature,
            path_frame_id='robot2/map',
        ))
        return True

    def check_dispatch_preconditions(
            self, task, final_path_valid, callback, **_kwargs):
        del task, final_path_valid
        callback(SimpleNamespace(ready=True))


def _task(signature, x, length):
    return PhysicalTask(
        source_robot_id='robot2',
        source_session_id='session-1',
        source_snapshot_epoch=1,
        source_map_revision=1,
        physical_signature=signature,
        local_frontier_id=int(x * 100),
        centroid=(x, 0.0),
        bounds=Bounds((x - 0.05, -0.05), (x + 0.05, 0.05)),
        approach=(x, 0.0),
        visible_reveal_gain=1.0,
        local_ordering_score=1.0,
        local_path_valid=True,
        local_path_length_m=length,
        local_path=((0.0, 0.0), (x, 0.0)),
        path_heading_cost_rad=0.0,
        generation_ros_ns=1,
    )


def _snapshot(epoch=1):
    return TaskSnapshot(
        source_robot_id='robot2',
        source_session_id='session-1',
        epoch=epoch,
        map_revision=epoch,
        map_fingerprint='map-%d' % epoch,
        generation_ros_ns=epoch,
        validity_s=30.0,
        tasks=(_task('frontier-a', 1.0, 1.0),
               _task('frontier-b', 3.0, 2.0)),
    )


def _node_for_local_only_test():
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._robot_id = 'robot2'
    node._peer_id = 'robot1'
    node._local_only = True
    node._dispatch_enabled = True
    node._common_start_release_required = False
    node._start_release_received = True
    node._initial_peer_readiness_barrier = False
    node._handoff_complete = False
    node._dispatch_in_progress = False
    node._hard_failure_signatures = {}
    node._completed_solo_physical_signatures = set()
    node._minimum_solo_visible_gain_m = 0.05
    node._minimum_solo_ordering_score = 0.0
    node._assignment_strategy = 'frontier_cost_only'
    node._solo_retry_not_before = {}
    node._solo_retry_counts = {}
    node._solo_route_history = []
    node._maximum_union_tasks = 10
    node._maximum_path_queries = 8
    node._weights = AssignmentWeights()
    node._last_solo_snapshot_key = None
    node._active_task = None
    node._active_round_id = ''
    node._active_decision_hash = ''
    node._active_navigation_action = None
    node._active_dispatch_path = ()
    node._navigation_action_sequence = 0
    node._dispatch_count = 0
    node._round = None
    node._round_lifecycle = RoundGeneration()
    node._bid_batches = {}
    node._peer_decision = None
    node._committed = SimpleNamespace()
    node._traffic_reallocation_after_clear = False
    node._released_traffic_winner_robot_id = ''
    node._settle_until_steady_s = 0.0
    node._post_goal_settle_s = 0.0
    node._last_semantic_fingerprint = ''
    node._nav2 = _Nav2()
    node.get_logger = lambda: _Logger()
    node._transition = lambda *_args, **_kwargs: None
    node._emit_event = lambda *_args, **_kwargs: None
    node._log_round_lifecycle = lambda *_args, **_kwargs: None
    node._clear_active_commitment = lambda *_args, **_kwargs: None
    return node


def test_two_sequential_successful_local_goals_rearm_dispatch():
    """A successful local goal must not strand the next eligible frontier."""
    node = _node_for_local_only_test()
    dispatched = []

    def dispatch_after_checks(task, final_path, checks, *args, **kwargs):
        del checks, args, kwargs
        member = task.members[0]
        action = ActiveNavigationAction(
            action_id='robot2:%d:%s' % (
                node._navigation_action_sequence + 1, task.canonical_id),
            task=task,
            canonical_task_id=task.canonical_id,
            physical_signature=member.physical_signature,
            round_id=node._active_round_id,
            decision_hash='DEGRADED_SOLO',
            generation=node._round_lifecycle.generation,
            path=tuple(final_path.samples),
        )
        node._active_navigation_action = action
        node._dispatch_count += 1
        node._navigation_action_sequence += 1
        node._dispatch_in_progress = False
        node._nav2.local_goal_active = True
        dispatched.append(action)

    node._dispatch_after_checks = dispatch_after_checks

    first = _snapshot(epoch=1)
    node._continue_degraded_solo(first)
    assert node._dispatch_count == 1
    assert dispatched[0].physical_signature == 'frontier-a'

    node._nav2.local_goal_active = False
    node._navigation_finished(
        NavigationOutcome(
            accepted=True,
            status=4,
            error_code=0,
            error_message='',
            failure_class=FailureClass.UNKNOWN,
            duration_s=1.0,
            travelled_distance_m=1.0,
            recoveries=0,
        ),
        dispatched[0],
    )

    second = replace(_snapshot(epoch=2), tasks=_snapshot(epoch=2).tasks)
    node._continue_degraded_solo(second)

    assert node._dispatch_count == 2
    assert len(dispatched) == 2
    assert dispatched[1].physical_signature == 'frontier-b'
    assert dispatched[1].round_id != dispatched[0].round_id


def test_unknown_map_gate_is_retryable_only_in_local_fallback():
    """Unknown occupancy remains rejected but does not become hard suppression."""
    checks = SimpleNamespace(
        ready=False,
        reason='goal occupancy-map cell is unknown or occupied',
        goal_inside_map=True,
        goal_map_value=-1,
        goal_inside_costmap=True,
        goal_costmap_value=0,
        local_path_clear=True,
        local_path_reason='CLEAR',
        final_path_valid=True,
        path_valid_checked=True,
        path_valid=True,
        path_valid_reason='',
        local_footprint_checked=True,
        local_footprint_clear=True,
        local_footprint_reason='CLEAR',
    )

    assert classify_solo_dispatch_failure(checks, local_only=True) == (
        FailureClass.TF_OR_LIFECYCLE)
    assert classify_solo_dispatch_failure(checks, local_only=False) == (
        FailureClass.HARD_UNREACHABLE)


def test_fallback_dispatch_contract_uses_member_approach_for_preflight_and_send():
    """The fallback validates and sends the same preserved approach member."""
    text = Path(__file__).parents[1].joinpath(
            'my_epuck_project', 'distributed_frontier_assignment.py',
        ).read_text()
    assert 'task.members[0], True,' in text
    assert 'self._nav2.send_navigation(\n                member,' in text


def test_retryable_unknown_rejection_leaves_next_candidate_eligible():
    """A retryable rejection must not suppress the other current candidate."""
    node = _node_for_local_only_test()
    snapshot = _snapshot()
    now = time.monotonic()
    node._solo_retry_not_before['frontier-a'] = now + 5.0

    candidates = eligible_solo_tasks(
        snapshot.tasks,
        set(node._hard_failure_signatures),
        node._completed_solo_physical_signatures,
        node._minimum_solo_visible_gain_m,
        node._minimum_solo_ordering_score,
        node._assignment_strategy,
    )
    ready = tuple(
        task for task in candidates
        if node._solo_retry_not_before.get(task.physical_signature, 0.0) <= now
    )

    assert [task.physical_signature for task in ready] == ['frontier-b']
    assert 'frontier-a' not in node._hard_failure_signatures


def test_centroid_and_approach_pose_remain_distinct_through_task_conversion():
    """The navigation task preserves the safe approach, not the centroid."""
    candidate = FrontierCandidate()
    candidate.frontier_id = 42
    candidate.centroid.x = 1.25
    candidate.centroid.y = -0.40
    candidate.approach_pose.pose.position.x = 0.80
    candidate.approach_pose.pose.position.y = -0.15
    candidate.approach_pose.pose.orientation.z = math.sin(0.35 / 2.0)
    candidate.approach_pose.pose.orientation.w = math.cos(0.35 / 2.0)
    candidate.reachability_state = candidate.REACHABLE

    task = to_physical_task(candidate, 'robot2')

    assert task.centroid == (1.25, -0.40)
    assert task.approach == (0.80, -0.15)
    assert task.approach != task.centroid
    assert task.approach_yaw == pytest.approx(0.35)


def test_minimal_coordinator_support_files_remain_byte_identical():
    """The solo extension leaves the simulator support modules unchanged."""
    repository = Path(__file__).parents[4]
    source_root = repository / 'cooperative_migration_source' / 'src'
    physical_root = repository / 'webots_ws' / 'src'
    relative_files = (
        'my_epuck_project/my_epuck_project/minimal_frontier_navigation.py',
        'my_epuck_project/my_epuck_project/minimal_frontier_protocol.py',
        'my_epuck_project/my_epuck_project/minimal_frontier_selection.py',
        'my_epuck_project/my_epuck_project/minimal_frontier_sets.py',
        'my_epuck_project/my_epuck_project/minimal_frontier_termination.py',
        'my_epuck_project/my_epuck_project/minimal_frontier_traffic.py',
    )
    for relative in relative_files:
        assert (physical_root / relative).read_bytes() == (
            source_root / relative).read_bytes()


def test_physical_solo_launch_enables_known_approach_mode_only():
    """Known/free approach filtering is isolated to the solo launch."""
    repository = Path(__file__).parents[4]
    generator = (
        repository / 'webots_ws' / 'src' / 'my_epuck_frontier_candidates' /
        'src' / 'frontier_candidate_generator.cpp'
    ).read_text()
    launch = (
        repository / 'webots_ws' / 'src' / 'my_epuck_project' / 'launch' /
        'robot2_custom_frontier_solo_launch.py'
    ).read_text()
    assert 'P(bool, require_known_approach, false);' in generator
    assert '"require_known_approach": True' in launch
    assert 'executable="minimal_frontier_allocator"' in launch
    assert 'executable="distributed_frontier_assignment"' not in launch
    assert 'executable="frontier_explorer"' not in launch


def test_dual_approach_search_keeps_unknown_map_cells_rejected():
    """The physical search evaluates occupancy and costmap cells together."""
    repository = Path(__file__).parents[4]
    source = (
        repository / 'webots_ws' / 'src' / 'my_epuck_frontier_candidates' /
        'src' / 'candidate_utils.cpp'
    ).read_text()
    assert 'find_safe_approach_known_free' in source
    assert 'map_value < 0 || map_value >= occupancy_threshold' in source
    assert 'costmap_value < 0 || costmap_value >= blocked_threshold' in source


def test_full_footprint_preflight_remains_active():
    """The fallback repair does not remove the physical polygon safety gate."""
    repository = Path(__file__).parents[4]
    source = (
        repository / 'webots_ws' / 'src' / 'my_epuck_project' /
        'my_epuck_project' / 'distributed_assignment' / 'local_nav2.py'
    ).read_text()
    assert 'local_footprint_checked' in source
    assert 'local_footprint_clear' in source
    assert 'published_footprint' in source
    assert 'FOOTPRINT_LETHAL_LOCAL_CELL' in source


def test_preflight_warmup_requires_consecutive_inputs_and_remains_retryable():
    """Missing local preflight inputs reset warm-up instead of suppressing tasks."""
    node = DistributedFrontierAssignment.__new__(DistributedFrontierAssignment)
    node._preflight_warmup_cycles_required = 3
    node._preflight_warmup_cycles_observed = 0
    transitions = []
    availability = iter((False, True, True, False, True, True, True))
    node._nav2 = SimpleNamespace(
        preflight_inputs_available=lambda: next(availability),
    )
    node._transition = lambda state, reason: transitions.append((state, reason))

    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_cycles_observed == 0
    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_cycles_observed == 1
    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_cycles_observed == 2
    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_cycles_observed == 0
    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_ready() is False
    assert node._preflight_warmup_ready() is True
    assert node._preflight_warmup_cycles_observed == 3
    assert transitions
