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
