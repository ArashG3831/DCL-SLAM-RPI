"""ROS-free tests for the opt-in minimal allocator solo branch."""

import hashlib
import time
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch
from pathlib import Path

import pytest
from geometry_msgs.msg import Point, PoseStamped
from my_epuck_interfaces.msg import (
    DistributedExplorationStatus,
    FrontierCandidate,
    FrontierCandidateArray,
)
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as NavPath

from my_epuck_project.minimal_frontier_allocator import MinimalFrontierAllocator
from my_epuck_project import minimal_frontier_navigation as navigation
from my_epuck_project import minimal_frontier_termination as termination
from my_epuck_project.mission_termination import CandidateEvidence, TerminalReason


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    warning = info
    error = info


class _Clock:
    def __init__(self):
        self.seconds = 1.0

    def now(self):
        return SimpleNamespace(
            nanoseconds=int(self.seconds * 1_000_000_000),
            to_msg=lambda: SimpleNamespace(
                sec=int(self.seconds), nanosec=0),
        )


class _Node:
    def __init__(self):
        self.clock = _Clock()

    def get_logger(self):
        return _Logger()

    def get_clock(self):
        return self.clock


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _candidate(frontier_id, length, centroid, approach):
    item = FrontierCandidate()
    item.frontier_id = int(frontier_id)
    item.centroid.x, item.centroid.y = centroid
    item.bounding_box_min.x = centroid[0] - 0.05
    item.bounding_box_min.y = centroid[1] - 0.05
    item.bounding_box_max.x = centroid[0] + 0.05
    item.bounding_box_max.y = centroid[1] + 0.05
    item.approach_pose.header.frame_id = 'robot2/map'
    item.approach_pose.header.stamp.sec = 1
    item.approach_pose.pose.position.x, item.approach_pose.pose.position.y = approach
    item.approach_pose.pose.orientation.w = 1.0
    item.reachability_state = item.REACHABLE
    item.information_gain = 0.5
    item.score = 0.0
    item.local_path_length_m = float(length)
    item.path_length_m = float(length)
    item.heading_change_rad = 0.0
    item.local_path_samples = [Point(x=0.0, y=0.0), Point(x=approach[0], y=approach[1])]
    item.planned_path = NavPath()
    item.planned_path.header.frame_id = 'robot2/map'
    for x, y in ((0.0, 0.0), (approach[0] * 0.5, approach[1] * 0.5), approach):
        pose = PoseStamped()
        pose.header = item.planned_path.header
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        item.planned_path.poses.append(pose)
    return item


def _array(source, candidates, generation=1, *, detected=None, small=0,
           out_of_range=0, unreachable=0, planner_failures=0,
           detected_not_queried=0):
    message = FrontierCandidateArray()
    message.header.frame_id = 'robot2/map'
    message.header.stamp.sec = 1
    message.source_robot_id = source
    message.map_revision = 1
    message.candidate_generation_id = generation
    message.candidates = list(candidates)
    message.detected_frontier_count = (
        len(candidates) if detected is None else detected)
    message.small_frontier_count = small
    message.out_of_range_frontier_count = out_of_range
    message.unreachable_frontier_count = unreachable
    message.planner_failure_count = planner_failures
    message.detected_not_queried_count = detected_not_queried
    return message


def _allocator(local, *, dispatch=True):
    node = MinimalFrontierAllocator.__new__(MinimalFrontierAllocator)
    node._node = _Node()
    node._robot_id = 'robot2'
    node._peer_id = 'robot1'
    node._source_session_id = '0' * 32
    node._allow_solo_without_peer = True
    node._dispatch_enabled = dispatch
    node._max_navigation_goals = 0
    node._navigation_goal_count = 0
    node._peer_candidate_timeout_s = 8.0
    node._candidate_received_steady_s = {'robot1': None, 'robot2': time.monotonic()}
    node._peer_candidate_available = False
    node._cooperative_pending = False
    node._cooperative_active = False
    node._solo_mode = True
    node._global_frame = 'robot2/map'
    node._last_solo_snapshot_key = None
    node._solo_preflight_blocked = {}
    node._solo_pending_preflight = None
    node._solo_active_dispatch = None
    node._minimum_visible_gain_m = 0.05
    node._solo_startup_spin_enabled = False
    node._solo_startup_spin_angle_rad = 0.1745
    node._solo_startup_spin_started = False
    node._solo_startup_spin_finished = False
    node._solo_startup_spin_succeeded = False
    node._solo_startup_spin_active = False
    node._solo_startup_spin_waiting_for_snapshot = False
    node._solo_startup_spin_baseline_stamp_ns = 0
    node._solo_startup_spin_baseline_generation_id = 0
    node._solo_startup_spin_client = None
    node._solo_follow_path_client = SimpleNamespace(
        server_is_ready=lambda: True)
    node._solo_follow_path_goal_handle = None
    node._solo_follow_path_callback = None
    node._solo_follow_path_cancel_requested = False
    node._candidates = {'robot1': None, 'robot2': local}
    node._evidence = {'robot1': node._make_evidence(FrontierCandidateArray()),
                      'robot2': node._make_evidence(local)}
    node._union = None
    node._local_batch = None
    node._local_batch_from_costing = False
    node._peer_batch = None
    node._peer_active_goal = None
    node._peer_status_union_hash = None
    node._peer_status_state = None
    node._peer_status_current = False
    node._peer_bid_after_terminal = True
    node._peer_terminal = False
    node._peer_terminal_reason = ''
    node._peer_status_after_terminal = True
    node._active_goal_id = None
    node._active_goal_union_hash = None
    node._goal_token = 0
    node._state = node.IDLE
    node._completed_ids = set()
    node._local_completed_ids = set()
    node._traffic_decision = None
    node._terminal_reason = None
    node._terminal_epoch = 0
    node._terminal_map_revision = 0
    node._solo_terminal_latched = False
    node._allocation_reason = ''
    node._released = True
    node._stable_since_s = 1.0
    node._stability_grace_s = 5.0
    node._bid_validity_s = 0.0
    node._minimum_visible_gain_m = 0.05
    node._local_batch_from_costing = False
    node._bid_publisher = None
    node._status_publisher = None
    node._event_publisher = None
    node._start_ready_publisher = None
    node._nav = None
    node._solo_evidence_seen = True
    node._dispatches = []
    node._dispatch = lambda candidate, path: node._dispatches.append((candidate, path))
    return node


def test_local_only_selection_needs_no_peer_and_preserves_two_points():
    first = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [first]))
    node._try_allocate_solo()
    assert len(node._dispatches) == 1
    selected = node._dispatches[0][0]
    assert (selected.centroid.x, selected.centroid.y) == (3.0, 4.0)
    assert (selected.approach_pose.pose.position.x,
            selected.approach_pose.pose.position.y) == (1.0, 2.0)
    assert node._peer_batch is None


def test_local_only_frontier_cost_only_picks_shortest_path():
    long = _candidate(11, 2.0, (2.0, 0.0), (1.0, 0.0))
    short = _candidate(22, 0.4, (4.0, 0.0), (3.0, 0.0))
    node = _allocator(_array('robot2', [long, short]))
    node._try_allocate_solo()
    assert len(node._dispatches) == 1
    assert node._dispatches[0][0].frontier_id == 22


def test_dispatch_disabled_selects_but_never_enters_navigation():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [candidate]), dispatch=False)
    node._try_allocate_solo()
    node._try_allocate_solo()
    assert node._dispatches == []
    assert node._active_goal_id is None


def test_unchanged_solo_snapshot_is_consumed_once():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [candidate]))
    node._try_allocate_solo()
    node._try_allocate_solo()
    assert len(node._dispatches) == 1


def test_unblocked_solo_candidates_do_not_fingerprint_costmaps():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    message = _array('robot2', [candidate])
    node = _allocator(message)
    union = node._solo_union_for(message)
    batch = node._local_batch_for(union)

    with patch.object(node, '_solo_safety_context_key', side_effect=AssertionError):
        assert node._solo_batch_without_blocked(message, batch) is batch


def test_solo_dispatch_uses_candidate_path_without_second_planner_or_is_path_valid():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [candidate]))
    node._dispatch = MinimalFrontierAllocator._dispatch.__get__(
        node, MinimalFrontierAllocator,
    )
    node._union = node._solo_union_for(node._candidates['robot2'])
    node._local_batch = node._local_batch_for(node._union)

    class _Nav:
        def __init__(self):
            self.readiness_calls = 0
            self.sent_task = None
            self.sent_path = None

        def evaluate_path(self, task, callback, caller=''):
            raise AssertionError('solo dispatch must not replan')

        def check_dispatch_preconditions(
                self, task, final_path_valid, callback, path_samples=(),
                path_frame_id='', path=None):
            raise AssertionError('solo dispatch must not run allocator preflight')

        def check_navigation_readiness(self, callback):
            self.readiness_calls += 1
            callback(SimpleNamespace(
                ready=True,
                reason='',
                navigation_readiness_only=True,
            ))

        def send_follow_path(self, task, callback):
            self.sent_task = task
            self.sent_path = task.planned_path
            return True

    node._nav = _Nav()
    node._send_solo_follow_path = node._nav.send_follow_path
    node._dispatch(candidate, tuple(candidate.local_path_samples))

    assert node._nav.readiness_calls == 1
    assert node._nav.sent_task.approach == (1.0, 2.0)
    assert node._nav.sent_path is candidate.planned_path


def test_solo_follow_path_goal_receives_exact_full_planner_path():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [candidate]))
    node._nav = SimpleNamespace(travelled_distance_m=0.0)

    class _Future:
        def add_done_callback(self, callback):
            self.callback = callback

    class _FollowClient:
        def __init__(self):
            self.goals = []

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            self.goals.append(goal)
            return _Future()

    client = _FollowClient()
    node._solo_follow_path_client = client
    task = replace(
        navigation.to_physical_task(candidate, 'robot2'),
        planned_path=candidate.planned_path,
    )

    assert node._send_solo_follow_path(task, lambda _outcome: None)
    assert len(client.goals) == 1
    goal = client.goals[0]
    assert goal.controller_id == 'FollowPath'
    assert goal.goal_checker_id == 'goal_checker'
    assert goal.progress_checker_id == 'progress_checker'
    assert goal.path.header.frame_id == candidate.planned_path.header.frame_id
    assert len(goal.path.poses) == len(candidate.planned_path.poses)
    for actual, expected in zip(goal.path.poses, candidate.planned_path.poses):
        assert actual.pose.position.x == expected.pose.position.x
        assert actual.pose.position.y == expected.pose.position.y


def test_solo_allocator_has_no_planner_query_or_navigate_to_pose_dispatch():
    source = Path(
        '/home/robot1/webots_ws/src/my_epuck_project/my_epuck_project/'
        'minimal_frontier_allocator.py').read_text(encoding='utf-8')
    assert 'ComputePathToPose' not in source
    assert 'NavigateToPose' not in source
    assert 'FollowPath' in source


class _SoloReadinessNav:
    def __init__(self, ready=True, reason=''):
        self.ready = ready
        self.reason = reason
        self.sent = []
        self.preflight_canceled = False
        self.readiness_calls = 0

    def check_navigation_readiness(self, callback):
        self.readiness_calls += 1
        callback(SimpleNamespace(
            ready=self.ready,
            reason=self.reason,
            navigation_readiness_only=True,
        ))

    def cancel_pending_preflight(self):
        self.preflight_canceled = True

    def health_flags(self):
        return True, True

    def preflight_inputs_available(self):
        return True

    def send_follow_path(self, task, callback):
        self.sent.append((task, task.planned_path))
        return True


class _ReadyNav:
    def health_flags(self):
        return True, True


def _completion_node(message, *, nav=None):
    node = _allocator(message, dispatch=False)
    node._nav = nav or _ReadyNav()
    node._status_publisher = _Publisher()
    node._node.clock.seconds = 7.0
    return node


def _real_dispatch(node):
    node._dispatch = MinimalFrontierAllocator._dispatch.__get__(
        node, MinimalFrontierAllocator,
    )
    node._send_solo_follow_path = node._nav.send_follow_path


def test_solo_geometry_is_not_rechecked_by_allocator():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    node._try_allocate_solo()

    assert len(node._nav.sent) == 1
    assert node._nav.sent[0][0].approach == (1.0, 0.0)
    assert node._solo_preflight_blocked == {}


def test_solo_readiness_failure_is_retryable_without_geometry_quarantine():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._nav = _SoloReadinessNav(
        ready=False, reason='lifecycle services unavailable: controller_server')
    _real_dispatch(node)

    node._try_allocate_solo()
    assert node._nav.sent == []
    assert node._solo_preflight_blocked == {}
    assert node._solo_pending_preflight is not None

    node._nav.ready = True
    node._nav.reason = ''
    node._tick()

    assert len(node._nav.sent) == 1
    assert node._nav.sent[0][0].approach == (1.0, 0.0)


def _context_grid(values=None):
    grid = OccupancyGrid()
    grid.header.frame_id = 'robot2/map'
    grid.info.width = 20
    grid.info.height = 20
    grid.info.resolution = 0.1
    grid.info.origin.position.x = -1.0
    grid.info.origin.position.y = -1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = list(values or [0] * 400)
    return grid


def test_solo_unrelated_costmap_change_does_not_reopen_rejection():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    message = _array('robot2', [candidate])
    node = _allocator(message)

    class _Nav:
        _costmap = _context_grid()
        _local_costmap = _context_grid()
        _tf_buffer = None

    node._nav = _Nav()
    path = ((0.0, 0.0), (0.5, 0.5))
    context = node._solo_safety_context_key(message, candidate, path)
    node._solo_preflight_blocked['11'] = context

    # This cell is unrelated to the planner path.
    node._nav._costmap.data[399] = 100

    assert node._solo_candidate_is_blocked(message, candidate, path)


def test_solo_snapshot_generation_and_map_revision_do_not_reopen_rejection():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    message = _array('robot2', [candidate], generation=1)
    node = _allocator(message)

    class _Nav:
        _costmap = _context_grid()
        _local_costmap = _context_grid()
        _map = _context_grid()
        _tf_buffer = None

    node._nav = _Nav()
    path = ((0.0, 0.0), (0.5, 0.5))
    context = node._solo_safety_context_key(message, candidate, path)
    node._solo_preflight_blocked['11'] = context

    newer_snapshot = _array('robot2', [candidate], generation=99)
    newer_snapshot.map_revision = 987654321
    # Volatile rolling-costmap data is deliberately not quarantine identity.
    node._nav._costmap.data[399] = 100

    assert node._solo_candidate_is_blocked(
        newer_snapshot, candidate, path,
    )


def test_solo_changed_semantic_candidate_geometry_reopens_rejection():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    message = _array('robot2', [candidate])
    node = _allocator(message)

    class _Nav:
        _costmap = _context_grid()
        _local_costmap = _context_grid()
        _tf_buffer = None

    node._nav = _Nav()
    path = ((0.0, 0.0), (0.5, 0.5))
    context = node._solo_safety_context_key(message, candidate, path)
    node._solo_preflight_blocked['11'] = context

    changed = _candidate(11, 0.6, (1.0, 0.0), (1.1, 0.0))

    assert not node._solo_candidate_is_blocked(message, changed, path)


def test_max_navigation_goals_one_allows_one_and_blocks_second():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._max_navigation_goals = 1
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    node._try_allocate_solo()
    assert len(node._nav.sent) == 1
    assert node._navigation_goal_count == 1
    first_token = node._goal_token
    node.on_navigation_outcome(
        first_token,
        SimpleNamespace(accepted=True, status=4),
    )
    node._try_allocate_solo()

    assert len(node._nav.sent) == 1
    assert node._navigation_goal_count == 1
    assert node._state == node.IDLE
    assert node._terminal_reason is None


def test_solo_navigation_failure_blocks_candidate_and_allows_next():
    first = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    second = _candidate(22, 0.8, (3.0, 0.0), (3.0, 0.0))
    message = _array('robot2', [first, second])
    node = _allocator(message)
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    node._try_allocate_solo()
    first_token = node._goal_token
    assert [item[0].physical_signature for item in node._nav.sent] == ['11']

    node.on_navigation_outcome(
        first_token,
        SimpleNamespace(accepted=True, status=6),
    )

    assert '11' in node._solo_preflight_blocked
    node._try_allocate_solo()
    assert [item[0].physical_signature for item in node._nav.sent] == [
        '11', '22']


def test_solo_navigation_failure_does_not_repeat_same_candidate():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    node._try_allocate_solo()
    first_token = node._goal_token
    node.on_navigation_outcome(
        first_token,
        SimpleNamespace(accepted=True, status=6),
    )
    node._try_allocate_solo()
    node._try_allocate_solo()

    assert len(node._nav.sent) == 1


def test_max_navigation_goals_blocks_selection_after_first_dispatch():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._max_navigation_goals = 1
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    with patch(
            'my_epuck_project.minimal_frontier_allocator.selection.choose_assignment',
            wraps=__import__(
                'my_epuck_project.minimal_frontier_allocator',
                fromlist=['selection'],
            ).selection.choose_assignment,
    ) as choose:
        node._try_allocate_solo()
        calls_after_first_dispatch = choose.call_count
        first_token = node._goal_token
        node.on_navigation_outcome(
            first_token,
            SimpleNamespace(accepted=True, status=4),
        )
        node._try_allocate_solo()

        assert choose.call_count == calls_after_first_dispatch
        assert node._state == node.IDLE


def test_max_navigation_goals_cancels_pending_preflight_at_boundary():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    node = _allocator(_array('robot2', [candidate]))
    node._max_navigation_goals = 1
    node._navigation_goal_count = 1
    node._nav = _SoloReadinessNav()
    _real_dispatch(node)

    node._dispatch(candidate, tuple())

    assert getattr(node._nav, 'preflight_canceled', False)
    assert node._navigation_goal_count == 1
    assert node._terminal_reason is None


def test_fresh_compatible_peer_stops_new_solo_selection():
    local = _array('robot2', [_candidate(11, 0.6, (1.0, 0.0), (0.5, 0.0))])
    peer_candidate = _candidate(99, 0.7, (2.0, 0.0), (1.5, 0.0))
    peer = _array('robot1', [peer_candidate], generation=2)
    node = _allocator(local, dispatch=False)
    node.on_candidate_array(peer)
    assert node._peer_candidate_available
    assert node._cooperative_pending
    assert not node._solo_mode
    assert node._dispatches == []


def test_peer_loss_before_cooperative_activation_returns_to_solo():
    local = _array('robot2', [_candidate(11, 0.6, (1.0, 0.0), (0.5, 0.0))])
    node = _allocator(local, dispatch=False)
    node._candidates['robot1'] = _array(
        'robot1', [_candidate(99, 0.7, (2.0, 0.0), (1.5, 0.0))], generation=2,
    )
    node._peer_candidate_available = True
    node._cooperative_pending = True
    node._solo_mode = False
    node._candidate_received_steady_s['robot1'] = time.monotonic() - 20.0
    node._update_peer_presence()
    assert node._solo_mode
    assert not node._cooperative_pending
    assert node._candidates['robot1'] is None


def test_stable_zero_frontier_evidence_publishes_no_frontiers():
    node = _completion_node(_array('robot2', [], detected=0))

    node._maybe_solo_terminal()

    assert node._terminal_reason == TerminalReason.NO_FRONTIERS.value
    assert node._status_publisher.messages[-1].state == (
        DistributedExplorationStatus.COMPLETE)
    assert node._status_publisher.messages[-1].terminal
    assert node._status_publisher.messages[-1].reason == (
        TerminalReason.NO_FRONTIERS.value)
    assert not node._status_publisher.messages[-1].local_nav_goal_active


def test_stable_all_unreachable_evidence_publishes_no_reachable_frontiers():
    candidate = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    candidate.reachability_state = candidate.UNKNOWN
    node = _completion_node(_array(
        'robot2', [candidate], unreachable=1,
    ))

    node._maybe_solo_terminal()

    assert node._terminal_reason == TerminalReason.NO_REACHABLE_FRONTIERS.value


def test_unreachable_plus_reachable_does_not_complete():
    blocked = _candidate(11, 0.4, (1.0, 0.0), (1.0, 0.0))
    blocked.reachability_state = blocked.UNKNOWN
    reachable = _candidate(22, 0.8, (2.0, 0.0), (2.0, 0.0))
    node = _completion_node(_array(
        'robot2', [blocked, reachable], unreachable=1,
    ))

    node._maybe_solo_terminal()

    assert node._terminal_reason is None


def test_planner_failure_does_not_complete():
    node = _completion_node(_array(
        'robot2', [], detected=1, planner_failures=1,
    ))

    node._maybe_solo_terminal()

    assert node._terminal_reason is None


def test_detected_not_queried_does_not_complete():
    node = _completion_node(_array(
        'robot2', [], detected=1, detected_not_queried=1,
    ))

    node._maybe_solo_terminal()

    assert node._terminal_reason is None


def test_missing_or_transient_nav_readiness_does_not_complete():
    class _NotReady:
        def health_flags(self):
            return False, False

    node = _completion_node(
        _array('robot2', [], detected=0), nav=_NotReady(),
    )

    node._maybe_solo_terminal()

    assert node._terminal_reason is None


@pytest.mark.parametrize('state, active_goal', [
    (MinimalFrontierAllocator.NAVIGATING, '11'),
    (MinimalFrontierAllocator.GOAL_PENDING, '11'),
    (MinimalFrontierAllocator.WAITING_TRAFFIC, None),
])
def test_active_or_pending_goal_does_not_complete(state, active_goal):
    node = _completion_node(_array('robot2', [], detected=0))
    node._state = state
    node._active_goal_id = active_goal

    node._maybe_solo_terminal()

    assert node._terminal_reason is None


def test_identical_terminal_snapshot_is_latched_once():
    node = _completion_node(_array('robot2', [], detected=0))

    node._maybe_solo_terminal()
    node._maybe_solo_terminal()

    assert len(node._status_publisher.messages) == 1
    assert node._terminal_epoch == 1


def test_solo_completion_stops_future_selection_and_dispatch():
    node = _completion_node(_array('robot2', [], detected=0))
    node._maybe_solo_terminal()

    node._try_allocate_solo()

    assert node._dispatches == []
    assert node._state == node.IDLE


def test_fresh_peer_clears_only_solo_terminal_latch():
    node = _completion_node(_array('robot2', [], detected=0))
    node._maybe_solo_terminal()
    peer = _array('robot1', [_candidate(99, 0.7, (2.0, 0.0), (1.5, 0.0))])

    node.on_candidate_array(peer)

    assert not node._solo_terminal_latched
    assert node._terminal_reason is None
    assert not node._solo_mode
    assert node._peer_candidate_available
    assert node._dispatches == []


def test_cooperative_terminal_matching_remains_unchanged():
    reason = TerminalReason.NO_REACHABLE_FRONTIERS.value

    assert termination.matching(True, reason, True, reason) == reason
    assert termination.matching(
        True, reason, True, TerminalReason.NO_FRONTIERS.value,
    ) is None


def test_minimal_coordinator_modules_match_authoritative_source():
    source_root = Path('/home/robot1/cooperative_migration_source/src/my_epuck_project/my_epuck_project')
    destination_root = Path('/home/robot1/webots_ws/src/my_epuck_project/my_epuck_project')
    names = (
        'minimal_frontier_allocator.py',
        'minimal_frontier_navigation.py',
        'minimal_frontier_protocol.py',
        'minimal_frontier_selection.py',
        'minimal_frontier_sets.py',
        'minimal_frontier_termination.py',
        'minimal_frontier_traffic.py',
    )
    # The allocator is the one explicitly extended for physical solo
    # completion.  Its six imported cooperative primitives remain exact.
    for name in names[1:]:
        assert hashlib.sha256((source_root / name).read_bytes()).digest() == hashlib.sha256((destination_root / name).read_bytes()).digest(), name


def test_observer_terminal_status_contract_is_robot2_only_safe():
    observer = Path(
        '/home/robot1/webots_ws/src/my_epuck_project/my_epuck_project/'
        'cooperative_experiment_logger.py').read_text(encoding='utf-8')
    launch = Path(
        '/home/robot1/webots_ws/src/my_epuck_project/launch/'
        'robot2_custom_frontier_solo_launch.py').read_text(encoding='utf-8')
    for field in (
            'terminal=bool(msg.terminal)', 'terminal_reason=msg.terminal_reason',
            'self.mission_terminal_reason = msg.terminal_reason or msg.reason',
            "'terminal_reason': self.latest[robot].get('terminal_reason', '')",
            '"robot_ids": [ROBOT_NAMESPACE]'):
        assert field in (observer if 'robot_ids' not in field else launch)
    assert 'thin_experiment_recorder' not in launch
    assert 'executable="distributed_frontier_assignment"' not in launch
    assert 'executable="frontier_explorer"' not in launch


def test_solo_branch_is_opt_in_and_launch_selects_only_minimal_allocator():
    assert 'allow_solo_without_peer' in open(
        '/home/robot1/webots_ws/src/my_epuck_project/my_epuck_project/'
        'minimal_frontier_allocator.py', encoding='utf-8',
    ).read()
    launch = open(
        '/home/robot1/webots_ws/src/my_epuck_project/launch/'
        'robot2_custom_frontier_solo_launch.py', encoding='utf-8',
    ).read()
    assert 'executable="minimal_frontier_allocator"' in launch
    assert 'executable="distributed_frontier_assignment"' not in launch
    assert 'executable="frontier_explorer"' not in launch
    assert '"solo_startup_spin_enabled": True' in launch
    assert '"solo_startup_spin_angle_rad": 0.1745' in launch


def test_solo_startup_spin_is_one_shot_and_waits_for_updated_snapshot():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    message = _array('robot2', [candidate], generation=1)
    node = _allocator(message)
    node._solo_startup_spin_enabled = True
    node._nav = SimpleNamespace(health_flags=lambda: (True, True))
    node._publish_status = lambda: None

    class _Future:
        def __init__(self):
            self.callback = None

        def add_done_callback(self, callback):
            self.callback = callback

    class _SpinClient:
        def __init__(self):
            self.goals = []
            self.future = _Future()

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            self.goals.append(goal)
            return self.future

    client = _SpinClient()
    node._solo_startup_spin_client = client

    assert node._solo_startup_spin_ready_for_snapshot(message) is False
    assert len(client.goals) == 1
    assert client.goals[0].target_yaw == pytest.approx(0.1745)
    assert node._solo_startup_spin_ready_for_snapshot(message) is False
    assert len(client.goals) == 1

    node._finish_solo_startup_spin(True, 'solo startup spin succeeded')
    assert node._solo_startup_spin_ready_for_snapshot(message) is False
    updated = _array('robot2', [candidate], generation=2)
    assert node._solo_startup_spin_ready_for_snapshot(updated) is True


def test_solo_startup_spin_starts_from_tick_without_candidates():
    """The bootstrap spin must not depend on a candidate already existing."""
    node = _allocator(_array('robot2', []))
    node._candidates['robot2'] = None
    node._solo_startup_spin_enabled = True
    node._nav = SimpleNamespace(health_flags=lambda: (True, True))
    node._publish_status = lambda: None
    node._update_peer_presence = lambda: None
    node._maybe_solo_terminal = lambda: None
    node._maybe_terminal = lambda: None

    class _Future:
        def add_done_callback(self, callback):
            self.callback = callback

    class _SpinClient:
        def __init__(self):
            self.goals = []

        def server_is_ready(self):
            return True

        def send_goal_async(self, goal):
            self.goals.append(goal)
            return _Future()

    client = _SpinClient()
    node._solo_startup_spin_client = client

    node._tick()

    assert len(client.goals) == 1
    assert client.goals[0].target_yaw == pytest.approx(0.1745)
    assert node._solo_startup_spin_started is True


def test_dispatch_disabled_bypasses_startup_spin():
    candidate = _candidate(11, 0.6, (3.0, 4.0), (1.0, 2.0))
    node = _allocator(_array('robot2', [candidate]), dispatch=False)
    node._solo_startup_spin_enabled = True
    assert node._solo_startup_spin_ready_for_snapshot(
        node._candidates['robot2']) is True
    assert node._solo_startup_spin_started is False
