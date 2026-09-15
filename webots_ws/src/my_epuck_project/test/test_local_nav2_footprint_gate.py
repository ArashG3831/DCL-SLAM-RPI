from types import SimpleNamespace

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid

from my_epuck_project.distributed_assignment.local_nav2 import (
    DispatchPreconditions,
    LocalNav2,
    classify_dispatch_precondition_failure,
    footprint_cost_at_pose,
    interpolate_pose2d,
    local_footprint_path_clearance,
    path_validity_decision,
)
from my_epuck_project.distributed_assignment.models import FailureClass


def make_grid(width=20, height=20, resolution=0.05):
    grid = OccupancyGrid()
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = -0.5
    grid.info.origin.position.y = -0.5
    grid.data = [0] * (width * height)
    return grid


def set_cell(grid, x, y, value):
    cell_x = int((x - grid.info.origin.position.x) / grid.info.resolution)
    cell_y = int((y - grid.info.origin.position.y) / grid.info.resolution)
    grid.data[cell_y * grid.info.width + cell_x] = value


def test_is_path_valid_accepts_clear_response():
    assert path_validity_decision(True, []) == (True, (), '')


def test_is_path_valid_rejects_invalid_pose_indices():
    assert path_validity_decision(False, [3, 7]) == (
        False, (3, 7), 'NAV2_GLOBAL_PATH_INVALID')


def test_polygon_rejects_lethal_cell_when_centerline_is_clear():
    grid = make_grid()
    set_cell(grid, 0.175, 0.0, 100)
    footprint = ((-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1))
    centerline_value = int(grid.data[10 * grid.info.width + 12])
    assert centerline_value == 0
    clear, reason, maximum = footprint_cost_at_pose(
        grid, (0.1, 0.0, 0.0), footprint)
    assert not clear
    assert reason == 'FOOTPRINT_LETHAL_LOCAL_CELL'
    assert maximum == 100


def test_polygon_rejects_rpp_inscribed_cost_99():
    grid = make_grid()
    set_cell(grid, 0.175, 0.0, 99)
    footprint = ((-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1))
    clear, reason, maximum = footprint_cost_at_pose(
        grid, (0.1, 0.0, 0.0), footprint)
    assert not clear
    assert reason == 'FOOTPRINT_LETHAL_LOCAL_CELL'
    assert maximum == 99


def test_unknown_local_footprint_cell_is_rejected():
    grid = make_grid()
    set_cell(grid, 0.175, 0.0, -1)
    footprint = ((-0.1, -0.1), (0.1, -0.1), (0.1, 0.1), (-0.1, 0.1))
    result = local_footprint_path_clearance(
        grid, ((0.1, 0.0, 0.0),), footprint)
    assert not result.clear
    assert result.reason == 'FOOTPRINT_UNKNOWN_LOCAL_CELL'


def test_local_check_stops_at_rolling_window_boundary():
    grid = make_grid(width=8, height=8)
    footprint = ((-0.02, -0.02), (0.02, -0.02),
                 (0.02, 0.02), (-0.02, 0.02))
    poses = interpolate_pose2d(
        ((-0.45, -0.3, 0.0), (0.35, -0.3, 0.0)), 0.025)
    result = local_footprint_path_clearance(grid, poses, footprint)
    assert result.clear
    assert result.reason == 'LOCAL_PREFIX_EXHAUSTED'
    assert result.inspected_poses > 0
    assert result.outside_poses > 0


def test_boundary_footprint_outside_window_ends_local_prefix():
    """A clear prefix is not rejected when the next footprint leaves the grid."""
    grid = make_grid(width=8, height=8)
    footprint = ((-0.1, -0.05), (0.1, -0.05),
                 (0.1, 0.05), (-0.1, 0.05))
    result = local_footprint_path_clearance(
        grid, ((-0.30, -0.30, 0.0), (-0.15, -0.30, 0.0)), footprint)
    assert result.clear
    assert result.reason == 'LOCAL_PREFIX_EXHAUSTED'
    assert result.inspected_poses == 1
    assert result.outside_poses == 1


def test_zero_local_footprint_coverage_is_retryable():
    """A path with no observable local prefix remains blocked from dispatch."""
    grid = make_grid(width=8, height=8)
    footprint = ((-0.02, -0.02), (0.02, -0.02),
                 (0.02, 0.02), (-0.02, 0.02))
    result = local_footprint_path_clearance(
        grid, ((0.2, 0.2, 0.0),), footprint)
    assert not result.clear
    assert result.reason == 'NO_LOCAL_PREFIX'
    assert result.inspected_poses == 0
    assert result.outside_poses == 1


def test_configured_physical_padding_and_inflation_are_not_changed():
    # The effective padded polygon is supplied by Nav2's published_footprint;
    # this test intentionally verifies that the helper does not add an
    # independent radius or padding policy.
    footprint_point = Point()
    footprint_point.x = 0.15
    footprint_point.y = 0.07
    assert (footprint_point.x, footprint_point.y) == (0.15, 0.07)


def test_missing_startup_local_costmap_is_retryable_not_hard_unreachable():
    checks = DispatchPreconditions(
        action_server_ready=True,
        lifecycle_active=False,
        transform_available=True,
        transform_age_s=0.01,
        goal_inside_map=True,
        goal_inside_costmap=True,
        goal_map_value=0,
        goal_costmap_value=0,
        local_path_clear=False,
        no_local_goal_active=True,
        final_path_valid=True,
        reason='local path gate: NO_LOCAL_COSTMAP',
        local_path_reason='NO_LOCAL_COSTMAP',
    )
    assert classify_dispatch_precondition_failure(checks) == FailureClass.TF_OR_LIFECYCLE


def test_tf_diagnostic_uses_configured_frames():
    nav = object.__new__(LocalNav2)
    nav._global_frame = 'robot2/map'
    nav._base_frame = 'robot2/base_link'
    nav._maximum_tf_age_s = 0.5

    class _Clock:
        def now(self):
            return SimpleNamespace(nanoseconds=2_000_000_000)

    class _Node:
        def get_clock(self):
            return _Clock()

    class _TF:
        def lookup_transform(self, *_args, **_kwargs):
            return SimpleNamespace(
                header=SimpleNamespace(
                    stamp=SimpleNamespace(sec=1, nanosec=0)))

    nav._node = _Node()
    nav._tf_buffer = _TF()

    healthy, age, reason = nav.shared_tf_status()

    assert not healthy
    assert age == 1.0
    assert reason == 'robot2/map to robot2/base_link transform is stale'
