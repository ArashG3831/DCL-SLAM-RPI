from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _launch_text(name):
    return (PROJECT_ROOT / 'launch' / name).read_text(encoding='utf-8')


def test_custom_fallback_delays_frontier_pipeline_for_costmap_readiness():
    text = _launch_text('robot2_custom_frontier_solo_launch.py')
    assert 'TimerAction' in text
    assert 'costmap_readiness_delay_s' in text
    assert 'delayed_pipeline = TimerAction' in text
    assert 'actions=[candidate_generator, proposal_adapter, allocator]' in text
    assert '"processing_rate_hz": 1.0' in text
    assert '"event_driven_costing": False' in text


def test_minimal_coordinator_launch_delays_frontier_pipeline_for_costmap_readiness():
    text = _launch_text('robot2_cooperative_allocator_launch.py')
    assert 'TimerAction' in text
    assert 'costmap_readiness_delay_s' in text
    assert 'delayed_pipeline = TimerAction' in text
    assert 'actions=[candidate_generator, proposal_adapter, allocator]' in text
