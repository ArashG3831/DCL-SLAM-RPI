import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _launch_text(name):
    return (PROJECT_ROOT / 'launch' / name).read_text(encoding='utf-8')


def _step_back(points, distance):
    lengths = [math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:])]
    total = sum(lengths)
    if total <= distance:
        return None
    remaining = distance
    for index in range(len(points) - 1, 0, -1):
        segment = lengths[index - 1]
        if segment == 0.0:
            continue
        if remaining <= segment:
            alpha = (segment - remaining) / segment
            a = points[index - 1]
            b = points[index]
            return (a[0] + alpha * (b[0] - a[0]),
                    a[1] + alpha * (b[1] - a[1]))
        remaining -= segment
    return None


def test_custom_fallback_delays_frontier_pipeline_for_costmap_readiness():
    text = _launch_text('robot2_custom_frontier_solo_launch.py')
    assert 'TimerAction' in text
    assert 'costmap_readiness_delay_s' in text
    assert 'delayed_pipeline = TimerAction' in text
    assert 'actions=[candidate_generator, proposal_adapter, allocator]' in text
    assert '"processing_rate_hz": 1.0' in text
    assert '"event_driven_costing": False' in text
    assert '"pause_planner_queries_while_navigating": True' in text


def test_solo_planner_queries_pause_without_opening_event_driven_costing_gate():
    launch = _launch_text('robot2_custom_frontier_solo_launch.py')
    source = (PROJECT_ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
              'frontier_candidate_generator.cpp').read_text(encoding='utf-8')
    assert '"event_driven_costing": False' in launch
    assert '"pause_planner_queries_while_navigating": True' in launch
    assert 'P(bool, pause_planner_queries_while_navigating, false)' in source
    assert 'if (event_driven_costing_ || pause_planner_queries_while_navigating_)' in source
    assert 'if (!event_driven_costing_)' in source
    assert 'FRONTIER_QUERY_PAUSED reason=NAVIGATION_ACTIVE' in source
    assert 'FRONTIER_QUERY_RESUMED reason=NAVIGATION_TERMINAL' in source


def test_solo_pause_drains_active_query_before_navigation_dispatch():
    """The pause handshake must never cancel an in-flight planner request."""
    launch = _launch_text('robot2_custom_frontier_solo_launch.py')
    generator = (PROJECT_ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
                 'frontier_candidate_generator.cpp').read_text(encoding='utf-8')
    allocator = (PROJECT_ROOT / 'my_epuck_project' /
                 'minimal_frontier_allocator.py').read_text(encoding='utf-8')

    pause_start = generator.index(
        'if (pause_planner_queries_while_navigating_)',
        generator.index('void coordinator_status_cb'))
    pause_end = generator.index('if (!event_driven_costing_)', pause_start)
    pause_body = generator[pause_start:pause_end]
    assert 'async_cancel_goal' not in pause_body
    assert '++request_generation_' not in pause_body
    assert 'FRONTIER_QUERY_PAUSE_WAITING_FOR_ACTIVE_REQUEST' in pause_body
    assert 'publish_planner_idle_ack("NO_ACTIVE_QUERY")' in pause_body

    result_start = generator.index('options.result_callback')
    result_end = generator.index('// The deadline belongs', result_start)
    result_body = generator[result_start:result_end]
    assert 'active_request_ = 0;' in result_body
    assert 'publish_planner_idle_ack("QUERY_DRAINED")' in result_body
    assert result_body.index('active_request_ = 0;') < result_body.index(
        'publish_planner_idle_ack("QUERY_DRAINED")')

    assert '"pause_planner_queries_while_navigating": True' in launch
    assert 'MINIMAL_ALLOCATOR_WAITING_FOR_PLANNER_IDLE' in allocator
    assert 'message.event_type != \'PLANNER_QUERY_IDLE\'' in allocator
    wait_index = allocator.index('MINIMAL_ALLOCATOR_WAITING_FOR_PLANNER_IDLE')
    readiness_index = allocator.index(
        'self._nav.check_navigation_readiness(precondition_result)', wait_index)
    assert wait_index < readiness_index


def test_solo_stepback_parameter_and_generator_contract():
    launch = _launch_text('robot2_custom_frontier_solo_launch.py')
    source = (PROJECT_ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
              'frontier_candidate_generator.cpp').read_text(encoding='utf-8')
    assert '"approach_clearance_m": 0.06' in launch
    assert '"frontier_goal_stepback_m": 0.20' in launch
    assert 'P(double, frontier_goal_stepback_m, 0.0)' in source
    assert 'step_back_path_pose' in source
    assert source.count('planner_->async_send_goal(') == 1
    assert source.rfind('step_back_path_pose') > source.index('result.result->path')


def test_generator_keeps_planner_request_through_path_validation_finalize():
    """The IsPathValid response must be able to publish the candidate."""
    source = (PROJECT_ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
              'frontier_candidate_generator.cpp').read_text(encoding='utf-8')
    result_callback = source.index('options.result_callback')
    validation_call = source.index('request_path_validation(', result_callback)
    nonempty_guard = source.index(
        'if (!effective_path.poses.empty())', result_callback)
    finalize = source.index('void finalize_path_candidate')
    finish = source.index('void finish(', finalize)
    validation_helper = source.index('void request_path_validation')
    validation_body = source[validation_helper:finalize]
    finalize_body = source[finalize:finish]

    # The planner result callback must retain the request ID until the
    # IsPathValid callback invokes finalize_path_candidate().
    assert 'active_request_ = 0;' not in source[result_callback:validation_call]
    assert nonempty_guard < validation_call
    request_region = source[nonempty_guard:source.index(
        'void request_path_validation', nonempty_guard)]
    assert 'const auto service_path = effective_path;' in request_region
    assert 'request_path_validation(\n              service_path, request,' in request_region
    assert 'callback_path = service_path' in request_region
    assert 'std::move(effective_path)' not in request_region
    assert 'request != active_request_' in validation_body
    assert 'FRONTIER_QUERY_RESULT' in finalize_body
    assert 'active_request_ = 0;' in finalize_body
    assert finalize_body.index('active_request_ = 0;') < finalize_body.index(
        'release_path_lock();')


def test_successful_planner_result_survives_rolling_context_change():
    """A rolling revision change must not discard a successful planner result."""
    source = (PROJECT_ROOT.parent / 'my_epuck_frontier_candidates' / 'src' /
              'frontier_candidate_generator.cpp').read_text(encoding='utf-8')
    refresh_start = source.index('void refresh_publication_context')
    refresh_end = source.index('void set_region_status', refresh_start)
    refresh_body = source[refresh_start:refresh_end]
    goal_response_start = source.index('options.goal_response_callback')
    result_start = source.index('options.result_callback')
    goal_response_body = source[goal_response_start:result_start]
    result_end = source.index('// The deadline belongs', result_start)
    result_body = source[result_start:result_end]

    # Successful Work entries are retained while the rolling map/costmap
    # advances.  Obsolete callbacks are still filtered by the request-current
    # guard in the result callback.
    assert 'auto valid = reachable_;' in refresh_body
    assert 'work_context_matches_snapshot' not in refresh_body
    assert 'reject_stale_work' not in refresh_body
    assert 'async_request_is_current' in goal_response_body
    assert 'work_context_matches_latest' not in goal_response_body
    assert 'STALE_REVISION_REJECTED' not in goal_response_body
    assert 'async_request_is_current' in result_body
    assert 'work_context_matches_latest' not in result_body
    assert 'STALE_REVISION_REJECTED' not in result_body
    assert 'reachable_.push_back(std::move(reachable));' in result_body


def test_exact_arc_length_stepback_interpolation():
    point = _step_back([(0.0, 0.0), (0.1, 0.0), (0.3, 0.0)], 0.20)
    assert math.isclose(point[0], 0.1)
    assert math.isclose(point[1], 0.0)


def test_stepback_rejects_short_path_without_fallback():
    assert _step_back([(0.0, 0.0), (0.2, 0.0)], 0.20) is None
    assert _step_back([(0.0, 0.0), (0.1, 0.0)], 0.20) is None


def test_stepback_preserves_centroid_and_recomputes_yaw_direction():
    approach = _step_back([(0.0, 0.0), (0.3, 0.0)], 0.20)
    centroid = (approach[0], approach[1] + 0.4)
    yaw = math.atan2(centroid[1] - approach[1], centroid[0] - approach[0])
    assert math.isclose(approach[0], 0.1)
    assert math.isclose(approach[1], 0.0)
    assert math.isclose(yaw, math.pi / 2.0)


def test_minimal_coordinator_launch_delays_frontier_pipeline_for_costmap_readiness():
    text = _launch_text('robot2_cooperative_allocator_launch.py')
    assert 'TimerAction' in text
    assert 'costmap_readiness_delay_s' in text
    assert 'delayed_pipeline = TimerAction' in text
    assert 'actions=[candidate_generator, proposal_adapter, allocator]' in text
