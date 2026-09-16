from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'my_epuck_project'
LAUNCH = ROOT / 'launch' / 'unknown_pose_validation_launch.py'


def test_validation_harness_starts_two_async_frontends():
    source = LAUNCH.read_text(encoding='utf-8')
    assert source.count("executable='unknown_pose_frontend'") == 1
    assert "_frontend('robot1', 'robot2')" in source
    assert "_frontend('robot2', 'robot1')" in source
    assert "'/robot1/map'" in source
    assert "'/robot2/map'" in source
    assert 'TimerAction' not in source
    assert 'common_start_release_required' not in source


def test_validation_harness_uses_only_live_unknown_pose_topics():
    source = LAUNCH.read_text(encoding='utf-8')
    for topic in (
            '/cslam/relative_pose/descriptors',
            '/cslam/relative_pose/crop_requests',
            '/cslam/relative_pose/crops',
            '/cslam/relative_pose/hypotheses',
            '/cslam/unknown_pose/{robot_id}/local_map',
            '/robot1/map', '/robot2/map'):
        assert topic in source
    assert 'world_profile' not in source
    assert 'ground_truth' not in source
    assert 'unknown_pose_phase_manager' not in source
    assert 'unknown_pose_shared_stack_activation' not in source
    assert 'minimal_frontier_allocator' not in source
    assert 'source_aware_map_fusion' not in source


def test_observer_records_one_accepted_map_transform_without_odom_correction():
    source = (PACKAGE / 'unknown_pose_validation_observer.py').read_text(
        encoding='utf-8')
    assert "message.status) != 'ACCEPTED'" in source
    assert 'self._accepted_key is not None' in source
    assert 'source_to_target' in source
    assert 'evidence_set_hash' in source
    assert 'post_acceptance_odometry_correction' in source
    assert 'ground_truth_transform_used' in source
    assert '/odom' not in source
    assert 'NavigateToPose' not in source
    assert 'cmd_vel' not in source


def test_pi_planned_path_contract_is_not_touched():
    message = (Path('/home/robot1/webots_ws') / 'src' /
               'my_epuck_interfaces' / 'msg' / 'PhysicalTask.msg')
    assert 'nav_msgs/Path planned_path' in message.read_text(encoding='utf-8')
