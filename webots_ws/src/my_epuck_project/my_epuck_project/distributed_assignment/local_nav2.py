"""Small namespace-local Nav2 path evaluation and execution boundary."""

from dataclasses import dataclass, replace
from collections import deque
import fcntl
import hashlib
import json
import math
import os
import time
from typing import Callable, Optional

from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PolygonStamped

from lifecycle_msgs.srv import GetState

from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import IsPathValid

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from tf2_ros import Buffer, TransformException, TransformListener

from .failures import classify_failure
from .models import FailureClass, FailureEvidence, PhysicalTask, Point, TravelDistance


# NavigateToPose does not declare child-controller result constants, but the
# Nav2 BT propagates FollowPath's result code in the observed terminal result.
# Keep this mapping local and explicit so a controller abort becomes hard
# evidence for task suppression without changing navigation behavior.
FOLLOW_PATH_TF_FAILURE_CODES = frozenset({102})
FOLLOW_PATH_CONTROLLER_FAILURE_CODES = frozenset({104, 105, 106, 107})

FOLLOW_PATH_CONTROLLER_ERROR_NAMES = {
    102: 'TF_ERROR',
    104: 'PATIENCE_EXCEEDED',
    105: 'FAILED_TO_MAKE_PROGRESS',
    106: 'NO_VALID_CONTROL',
    107: 'CONTROLLER_TIMED_OUT',
}


def initial_path_heading_cost(
        samples: tuple[Point, ...], robot_yaw: float,
        minimum_segment_m: float = 0.05) -> float:
    """Measure initial planned-path direction mismatch in radians.

    The path's first meaningful segment is used instead of the approach-pose
    bearing.  Nav2 commonly repeats the first pose, so nearly coincident
    samples are skipped.  A path without a meaningful segment has no heading
    evidence and contributes zero rather than inventing a turn cost.
    """
    if not math.isfinite(robot_yaw) or minimum_segment_m <= 0.0:
        return 0.0
    if len(samples) < 2:
        return 0.0
    first = samples[0]
    if not all(math.isfinite(float(value)) for value in first):
        return 0.0
    for second in samples[1:]:
        if not all(math.isfinite(float(value)) for value in second):
            return 0.0
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        if math.hypot(dx, dy) < minimum_segment_m:
            continue
        path_yaw = math.atan2(dy, dx)
        return abs(math.atan2(
            math.sin(path_yaw - robot_yaw),
            math.cos(path_yaw - robot_yaw),
        ))
    return 0.0


def classify_follow_path_controller_error(error_code: int) -> FailureClass:
    """Classify propagated FollowPath failures without hiding TF faults."""
    if int(error_code) in FOLLOW_PATH_TF_FAILURE_CODES:
        return FailureClass.TF_OR_LIFECYCLE
    if int(error_code) in FOLLOW_PATH_CONTROLLER_FAILURE_CODES:
        return FailureClass.CONTROLLER_NO_PROGRESS
    return FailureClass.UNKNOWN


def follow_path_controller_error_name(error_code: int) -> str:
    """Return the installed Nav2 FollowPath meaning without flattening it."""
    return FOLLOW_PATH_CONTROLLER_ERROR_NAMES.get(int(error_code), '')


def classify_dispatch_precondition_failure(
        checks: 'DispatchPreconditions') -> FailureClass:
    """Classify a final dispatch rejection by its first-order evidence.

    Geometry can be rejected before lifecycle queries complete.  In that case
    ``lifecycle_active`` is deliberately still false and must not turn a
    stale/unknown goal into a TF/lifecycle failure.
    """
    map_geometry_bad = (
        not checks.goal_inside_map or checks.goal_map_value is None or
        checks.goal_map_value < 0 or checks.goal_map_value >= 50
    )
    costmap_geometry_bad = (
        not checks.goal_inside_costmap or checks.goal_costmap_value is None or
        checks.goal_costmap_value < 0 or
        checks.goal_costmap_value >= 253
    )
    transient_geometry_missing = (
        not checks.path_valid_checked or not checks.local_footprint_checked or
        checks.local_path_reason in {
            'NO_LOCAL_COSTMAP', 'TRANSFORM_UNAVAILABLE',
        } or checks.local_footprint_reason in {
            'NO_LOCAL_COSTMAP', 'NO_EFFECTIVE_FOOTPRINT', 'NO_LOCAL_PREFIX',
            'STALE_EFFECTIVE_FOOTPRINT', 'FOOTPRINT_BASE_TF_UNAVAILABLE',
            'PATH_LOCAL_TF_UNAVAILABLE', 'STALE_PATH_LOCAL_TF',
        } or checks.path_valid_reason.startswith('PATH_VALID_SERVICE_') or
        checks.local_footprint_reason.startswith((
            'FOOTPRINT_BASE_TF_UNAVAILABLE', 'PATH_LOCAL_TF_UNAVAILABLE',
        ))
    )
    if transient_geometry_missing:
        return FailureClass.TF_OR_LIFECYCLE
    if (map_geometry_bad or costmap_geometry_bad or
            not checks.local_path_clear or not checks.final_path_valid or
            (checks.path_valid_checked and not checks.path_valid) or
            (checks.local_footprint_checked and not checks.local_footprint_clear)):
        return FailureClass.HARD_UNREACHABLE
    if (
            'UNAVAILABLE' in checks.path_valid_reason or
            'TIMEOUT' in checks.path_valid_reason or
            'UNAVAILABLE' in checks.local_footprint_reason or
            'STALE' in checks.local_footprint_reason or
            'NO_' in checks.local_footprint_reason):
        return FailureClass.TF_OR_LIFECYCLE
    if (not checks.action_server_ready or not checks.lifecycle_active or
            not checks.transform_available):
        return FailureClass.TF_OR_LIFECYCLE
    if not checks.no_local_goal_active:
        return FailureClass.ACTION_REJECTION
    return FailureClass.UNKNOWN


@dataclass(frozen=True)
class PathEvaluation:
    """Observable result of one local ComputePathToPose request."""

    valid: bool
    length_m: float
    samples: tuple[Point, ...]
    query_ros_ns: int
    error_code: int
    error_message: str
    failure_class: FailureClass
    query_started_ros_ns: int = 0
    duration_s: float = 0.0
    caller: str = 'UNKNOWN'
    task_signature: str = ''
    map_stamp_ns: int = 0
    costmap_stamp_ns: int = 0
    heading_cost: float = 0.0
    # Diagnostic provenance only.  Nav2 returns the path frame in the action
    # result; retaining it lets the final dispatch audit compare the planner
    # path with the local-costmap transform without changing path semantics.
    path_frame_id: str = ''
    # Preserve the exact planner result for Nav2's IsPathValid request.  The
    # bounded samples above remain the existing allocator/scoring interface.
    path: Optional[Path] = None


@dataclass(frozen=True)
class LocalFootprintClearance:
    """Full-polygon evidence for the rolling local costmap prefix."""

    clear: bool
    reason: str
    inspected_poses: int
    outside_poses: int
    maximum_cost: Optional[int] = None
    first_blocked_pose_index: Optional[int] = None
    first_blocked_pose: Optional[Point] = None
    first_blocked_pose_yaw: Optional[float] = None
    first_blocked_cell: Optional[tuple[int, int]] = None
    first_blocked_cell_world: Optional[Point] = None
    effective_footprint: tuple[Point, ...] = ()
    costmap_frame: str = ''
    costmap_origin: Optional[Point] = None
    costmap_origin_yaw: Optional[float] = None
    costmap_width: int = 0
    costmap_height: int = 0
    costmap_resolution: Optional[float] = None
    costmap_stamp_ns: int = 0
    path_transform_translation: Optional[Point] = None
    path_transform_yaw: Optional[float] = None
    path_transform_stamp_ns: int = 0
    path_transform_age_s: Optional[float] = None


@dataclass(frozen=True)
class DispatchPreconditions:
    """Auditable final local dispatch preconditions."""

    action_server_ready: bool
    lifecycle_active: bool
    transform_available: bool
    transform_age_s: Optional[float]
    goal_inside_map: bool
    goal_inside_costmap: bool
    goal_map_value: Optional[int]
    goal_costmap_value: Optional[int]
    local_path_clear: bool
    no_local_goal_active: bool
    final_path_valid: bool
    reason: str
    local_path_reason: str = ''
    local_path_inspected_points: int = 0
    local_path_outside_points: int = 0
    local_path_gate_mode: str = 'MODE_A'
    local_path_gate_threshold: int = 80
    local_path_maximum_cost: Optional[int] = None
    local_path_first_blocked_point_index: Optional[int] = None
    local_path_first_blocked_point: Optional[Point] = None
    local_path_first_blocked_local_point: Optional[Point] = None
    path_valid_service_name: str = 'is_path_valid'
    path_valid_checked: bool = False
    path_valid: bool = False
    path_invalid_pose_indices: tuple[int, ...] = ()
    path_valid_reason: str = ''
    # The physical solo opt-in can use the candidate generator's already
    # completed ComputePathToPose result.  In that mode this record represents
    # only Nav2 action/lifecycle readiness; the geometry fields are deliberately
    # not evaluated here.
    navigation_readiness_only: bool = False
    local_footprint_checked: bool = False
    local_footprint_clear: bool = False
    local_footprint_reason: str = ''
    local_footprint_inspected_poses: int = 0
    local_footprint_outside_poses: int = 0
    local_footprint_maximum_cost: Optional[int] = None
    local_footprint_first_blocked_pose_index: Optional[int] = None
    local_footprint_first_blocked_pose: Optional[Point] = None
    local_footprint_first_blocked_pose_yaw: Optional[float] = None
    local_footprint_first_blocked_cell: Optional[tuple[int, int]] = None
    local_footprint_first_blocked_cell_world: Optional[Point] = None
    local_footprint_effective_footprint: tuple[Point, ...] = ()
    local_footprint_costmap_frame: str = ''
    local_footprint_costmap_origin: Optional[Point] = None
    local_footprint_costmap_origin_yaw: Optional[float] = None
    local_footprint_costmap_width: int = 0
    local_footprint_costmap_height: int = 0
    local_footprint_costmap_resolution: Optional[float] = None
    local_footprint_costmap_stamp_ns: int = 0
    local_footprint_path_transform_translation: Optional[Point] = None
    local_footprint_path_transform_yaw: Optional[float] = None
    local_footprint_path_transform_stamp_ns: int = 0
    local_footprint_path_transform_age_s: Optional[float] = None

    @property
    def ready(self) -> bool:
        """Return whether every required dispatch condition passed."""
        if self.navigation_readiness_only:
            return (
                self.action_server_ready and self.lifecycle_active and
                self.no_local_goal_active and not self.reason
            )
        return (
            self.action_server_ready and self.lifecycle_active and
            self.transform_available and self.goal_inside_map and
            self.goal_inside_costmap and self.local_path_clear and
            self.no_local_goal_active and
            self.final_path_valid and self.path_valid_checked and
            self.path_valid and self.local_footprint_checked and
            self.local_footprint_clear and not self.reason
        )


@dataclass(frozen=True)
class NavigationOutcome:
    """Terminal local NavigateToPose evidence and measured motion."""

    accepted: bool
    status: int
    error_code: int
    error_message: str
    failure_class: FailureClass
    duration_s: float
    travelled_distance_m: float
    recoveries: int
    nav2_error_name: str = ''
    follow_path_error_code: int = 0
    follow_path_error_name: str = ''
    controller_failure_family: str = ''
    deepest_failure_classification: str = ''
    deepest_failure_timestamp_ros_ns: int = 0
    diagnostic_snapshot_json: str = ''


# Controlled final-dispatch experiment modes. MODE_A is the historical
# policy and remains the default. MODE_B uses Nav2 costmap semantics for the
# local execution corridor: unknown cells remain unsafe, while known
# inflation costs below the lethal/inscribed boundary remain traversable.
LOCAL_PATH_GATE_MODE_A = 'MODE_A'
LOCAL_PATH_GATE_MODE_B = 'MODE_B'
LOCAL_PATH_GATE_MODES = frozenset({
    LOCAL_PATH_GATE_MODE_A, LOCAL_PATH_GATE_MODE_B,
})
LOCAL_PATH_GATE_MODE_A_THRESHOLD = 80
# The final gate reads nav_msgs/OccupancyGrid values, whose known cost range
# is 0..100.  The physical RPP footprint collision behavior treats the
# inscribed cost published as 99 as a hard collision for this gate.
LOCAL_PATH_GATE_MODE_B_THRESHOLD = 100
LOCAL_FOOTPRINT_GATE_THRESHOLD = 99


def local_path_gate_threshold(mode: str, lethal_threshold: int = 253) -> int:
    """Return the selected local-path acceptance threshold."""
    if mode == LOCAL_PATH_GATE_MODE_A:
        return LOCAL_PATH_GATE_MODE_A_THRESHOLD
    if mode == LOCAL_PATH_GATE_MODE_B:
        # This path reads nav_msgs/OccupancyGrid, not raw uint8 costmap data:
        # -1 is unknown, 0..99 is known free/inflation cost, and 100 is the
        # occupied/lethal boundary used by the project’s costmap observer.
        return LOCAL_PATH_GATE_MODE_B_THRESHOLD
    raise ValueError('local_path_gate_mode must be MODE_A or MODE_B')


UPSTREAM_POINT_BLOCK_THRESHOLD = 50


def upstream_point_validation(
        grid: Optional[OccupancyGrid], point: Point,
        threshold: int = UPSTREAM_POINT_BLOCK_THRESHOLD) -> dict:
    """Mirror v1.6.0 ``world_point_cost``/``is_world_point_blocked``.

    The upstream public point helper treats absent and out-of-bounds points as
    not blocked, and blocks only a present occupancy value strictly greater
    than ``OCC_THRESHOLD``.  Keep the raw value and the reason separate so
    this diagnostic cannot be mistaken for a dispatch gate.
    """
    if grid is None:
        return {'status': 'NO_DATA', 'cost': None, 'in_bounds': False}
    value = occupancy_value(grid, point)
    if value is None:
        return {'status': 'OUT_OF_BOUNDS', 'cost': None, 'in_bounds': False}
    if int(value) < 0:
        return {'status': 'UNKNOWN', 'cost': int(value), 'in_bounds': True}
    return {
        'status': 'BLOCKED' if int(value) > int(threshold) else 'FREE_OR_ACCEPTED',
        'cost': int(value),
        'in_bounds': True,
    }


def _grid_cell(grid: OccupancyGrid, point: tuple[float, float]) -> Optional[tuple[int, int]]:
    """Return a world point's integer cell without exposing an unbounded grid."""
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return None
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    return math.floor(local_x / resolution), math.floor(local_y / resolution)


def bounded_grid_crop(
        grid: Optional[OccupancyGrid], center: Optional[tuple[float, float]],
        radius_cells: int = 10) -> dict:
    """Return a bounded row-major occupancy crop for failure diagnostics."""
    if grid is None or center is None:
        return {}
    cell = _grid_cell(grid, center)
    if cell is None:
        return {}
    cx, cy = cell
    radius = max(1, min(int(radius_cells), 20))
    values = []
    for y in range(cy - radius, cy + radius + 1):
        for x in range(cx - radius, cx + radius + 1):
            if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
                values.append(None)
            else:
                index = y * grid.info.width + x
                values.append(int(grid.data[index]) if index < len(grid.data) else None)
    return {
        'center_cell': [cx, cy],
        'radius_cells': radius,
        'resolution_m': float(grid.info.resolution),
        'width': 2 * radius + 1,
        'height': 2 * radius + 1,
        'values': values,
    }


def _transform_translation(tf_buffer, target_frame: str, source_frame: str):
    """Return the latest source-frame origin expressed in target_frame."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.0),
        )
    except TransformException:
        return None
    return (
        float(transform.transform.translation.x),
        float(transform.transform.translation.y),
    )


def _transform_point(tf_buffer, target_frame: str, source_frame: str,
                     point: Point):
    """Return a source-frame point expressed in target_frame, if available."""
    try:
        transform = tf_buffer.lookup_transform(
            target_frame, source_frame, Time(),
            timeout=Duration(seconds=0.0),
        )
    except TransformException:
        return None
    rotation = transform.transform.rotation
    yaw = math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
    )
    translation = transform.transform.translation
    return (
        math.cos(yaw) * point[0] - math.sin(yaw) * point[1] + translation.x,
        math.sin(yaw) * point[0] + math.cos(yaw) * point[1] + translation.y,
    ), _stamp_ns_static(transform.header.stamp)


def _stamp_ns_static(stamp) -> int:
    """Convert a ROS builtin time stamp without requiring a node instance."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def execution_geometry_signature(
        target: Optional[tuple[float, float]], costmap_crop: dict) -> str:
    """Hash execution-relevant local geometry, excluding timestamps."""
    payload = {
        'target': None if target is None else [round(target[0], 3), round(target[1], 3)],
        'resolution_m': costmap_crop.get('resolution_m'),
        'width': costmap_crop.get('width'),
        'height': costmap_crop.get('height'),
        'values': costmap_crop.get('values', []),
    }
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8'),
    ).hexdigest()[:20]


def path_length(points: tuple[Point, ...]) -> float:
    """Measure a path polyline in its declared frame."""
    return sum(math.dist(first, second) for first, second in zip(points, points[1:]))


def path_samples_digest(points: tuple[Point, ...]) -> str:
    """Return a compact deterministic digest for planner/gate path identity."""
    return hashlib.sha256(
        json.dumps(
            [[float(point[0]), float(point[1])] for point in points],
            separators=(',', ':'),
        ).encode('utf-8'),
    ).hexdigest()[:20]


def path_is_valid_finite(evaluation: PathEvaluation) -> bool:
    """Return whether a successful path is structurally safe to consume.

    Path length is deliberately not bounded here.  A finite, otherwise valid
    Nav2 path remains eligible regardless of distance; distance is a scoring
    and navigation-cost input, not an artificial reachability gate.
    """
    return bool(
        evaluation.valid and math.isfinite(evaluation.length_m) and
        evaluation.length_m >= 0.0 and evaluation.samples and
        all(
            math.isfinite(float(x)) and math.isfinite(float(y))
            for x, y in evaluation.samples
        )
    )


def downsample_path(points: tuple[Point, ...], maximum_samples: int) -> tuple[Point, ...]:
    """Keep deterministic endpoints and bounded evenly spaced path samples."""
    if len(points) <= maximum_samples:
        return points
    if maximum_samples < 2:
        return points[:maximum_samples]
    indices = {
        round(index * (len(points) - 1) / (maximum_samples - 1))
        for index in range(maximum_samples)
    }
    return tuple(points[index] for index in sorted(indices))


def occupancy_value(grid: OccupancyGrid, point: Point) -> Optional[int]:
    """Read a world point from a grid with a possibly rotated origin."""
    resolution = grid.info.resolution
    if resolution <= 0.0:
        return None
    origin = grid.info.origin
    quaternion = origin.orientation
    yaw = math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
    )
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    cell_x = math.floor(local_x / resolution)
    cell_y = math.floor(local_y / resolution)
    if (cell_x < 0 or cell_y < 0 or
            cell_x >= grid.info.width or cell_y >= grid.info.height):
        return None
    index = cell_y * grid.info.width + cell_x
    if index >= len(grid.data):
        return None
    return int(grid.data[index])


def _point_in_polygon(point: Point, polygon: tuple[Point, ...]) -> bool:
    """Return whether a 2D point lies inside a polygon."""
    inside = False
    x, y = point
    for index in range(len(polygon)):
        x1, y1 = polygon[index - 1]
        x2, y2 = polygon[index]
        if ((y1 > y) != (y2 > y)) and (
                x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
            inside = not inside
    return inside


def _segment_distance(point: Point, first: Point, second: Point) -> float:
    """Return the distance from a point to a finite segment."""
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    if dx == 0.0 and dy == 0.0:
        return math.dist(point, first)
    projection = (
        (point[0] - first[0]) * dx + (point[1] - first[1]) * dy
    ) / (dx * dx + dy * dy)
    projection = max(0.0, min(1.0, projection))
    closest = (first[0] + projection * dx, first[1] + projection * dy)
    return math.dist(point, closest)


def interpolate_pose2d(
        poses: tuple[tuple[float, float, float], ...],
        maximum_step_m: float) -> tuple[tuple[float, float, float], ...]:
    """Interpolate planar poses at no more than the requested spacing."""
    if not poses:
        return ()
    if maximum_step_m <= 0.0 or len(poses) == 1:
        return poses
    output = [poses[0]]
    for first, second in zip(poses, poses[1:]):
        distance = math.hypot(second[0] - first[0], second[1] - first[1])
        steps = max(1, math.ceil(distance / maximum_step_m))
        yaw_delta = math.atan2(
            math.sin(second[2] - first[2]),
            math.cos(second[2] - first[2]),
        )
        for step in range(1, steps + 1):
            fraction = step / steps
            output.append((
                first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]),
                first[2] + fraction * yaw_delta,
            ))
    return tuple(output)


def path_validity_decision(
        is_valid: bool, invalid_pose_indices,
) -> tuple[bool, tuple[int, ...], str]:
    """Normalize the IsPathValid response for deterministic gate handling."""
    indices = tuple(int(index) for index in invalid_pose_indices)
    return (
        bool(is_valid),
        indices,
        '' if is_valid else 'NAV2_GLOBAL_PATH_INVALID',
    )


def footprint_cost_at_pose(
        grid: OccupancyGrid, pose: tuple[float, float, float],
        footprint: tuple[Point, ...],
        lethal_threshold: int = LOCAL_FOOTPRINT_GATE_THRESHOLD,
) -> tuple[bool, str, Optional[int]]:
    """Check an effective polygon against an OccupancyGrid.

    OccupancyGrid cost values are 0..100 with -1 unknown.  The physical RPP
    collision behavior treats the published inscribed cost 99 as a hard stop;
    keep that threshold aligned here while retaining unknown-cell rejection.
    """
    if grid is None or not footprint:
        return False, 'NO_LOCAL_FOOTPRINT', None
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return False, 'INVALID_LOCAL_RESOLUTION', None
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    polygon = tuple((
        x + cosine * point[0] - sine * point[1],
        y + sine * point[0] + cosine * point[1],
    ) for point in footprint)
    if len(polygon) < 3:
        return False, 'INVALID_LOCAL_FOOTPRINT', None

    origin = grid.info.origin
    origin_q = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
        1.0 - 2.0 * (origin_q.y ** 2 + origin_q.z ** 2),
    )
    origin_cosine, origin_sine = math.cos(origin_yaw), math.sin(origin_yaw)

    def grid_coordinates(point: Point) -> tuple[float, float]:
        dx = point[0] - origin.position.x
        dy = point[1] - origin.position.y
        return (
            (origin_cosine * dx + origin_sine * dy) / resolution,
            (-origin_sine * dx + origin_cosine * dy) / resolution,
        )

    grid_points = tuple(grid_coordinates(point) for point in polygon)
    min_x = max(0, math.floor(min(point[0] for point in grid_points)) - 1)
    max_x = min(grid.info.width - 1, math.ceil(max(point[0] for point in grid_points)) + 1)
    min_y = max(0, math.floor(min(point[1] for point in grid_points)) - 1)
    max_y = min(grid.info.height - 1, math.ceil(max(point[1] for point in grid_points)) + 1)
    if min_x > max_x or min_y > max_y:
        return False, 'FOOTPRINT_OUT_OF_LOCAL_WINDOW', None

    maximum_cost = None
    checked_cells = set()
    cell_half_diagonal = resolution * math.sqrt(2.0) / 2.0
    for cell_y in range(min_y, max_y + 1):
        for cell_x in range(min_x, max_x + 1):
            local_x = (cell_x + 0.5) * resolution
            local_y = (cell_y + 0.5) * resolution
            cell_center = (
                origin.position.x + origin_cosine * local_x - origin_sine * local_y,
                origin.position.y + origin_sine * local_x + origin_cosine * local_y,
            )
            in_polygon = _point_in_polygon(cell_center, polygon)
            near_edge = any(
                _segment_distance(cell_center, polygon[index - 1], polygon[index])
                <= cell_half_diagonal
                for index in range(len(polygon))
            )
            if not in_polygon and not near_edge:
                continue
            index = cell_y * grid.info.width + cell_x
            if index in checked_cells or index >= len(grid.data):
                continue
            checked_cells.add(index)
            value = int(grid.data[index])
            maximum_cost = value if maximum_cost is None else max(maximum_cost, value)
            if value < 0:
                return False, 'FOOTPRINT_UNKNOWN_LOCAL_CELL', maximum_cost
            if value >= lethal_threshold:
                return False, 'FOOTPRINT_LETHAL_LOCAL_CELL', maximum_cost

    # Explicitly sample the polygon boundary.  This catches a lethal cell whose
    # center lies just outside a thin edge but whose footprint edge crosses it.
    for index, first in enumerate(polygon):
        second = polygon[index - 1]
        distance = math.dist(first, second)
        steps = max(1, math.ceil(distance / (resolution * 0.5)))
        for step in range(steps + 1):
            fraction = step / steps
            sample = (
                first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]),
            )
            value = occupancy_value(grid, sample)
            if value is None:
                return False, 'FOOTPRINT_OUT_OF_LOCAL_WINDOW', maximum_cost
            maximum_cost = value if maximum_cost is None else max(maximum_cost, value)
            if value < 0:
                return False, 'FOOTPRINT_UNKNOWN_LOCAL_CELL', maximum_cost
            if value >= lethal_threshold:
                return False, 'FOOTPRINT_LETHAL_LOCAL_CELL', maximum_cost
    return True, 'CLEAR', maximum_cost


@dataclass(frozen=True)
class FootprintPoseEvidence:
    """Diagnostic location for the first hard footprint cell."""

    clear: bool
    reason: str
    maximum_cost: Optional[int]
    blocked_cell: Optional[tuple[int, int]] = None
    blocked_cell_world: Optional[Point] = None


def _grid_cell_world_coordinates(
        grid: OccupancyGrid, point: Point) -> Optional[tuple[tuple[int, int], Point]]:
    """Return the grid cell and center containing a local-frame point."""
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return None
    origin = grid.info.origin
    origin_q = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
        1.0 - 2.0 * (origin_q.y ** 2 + origin_q.z ** 2),
    )
    origin_cosine, origin_sine = math.cos(origin_yaw), math.sin(origin_yaw)
    dx = point[0] - origin.position.x
    dy = point[1] - origin.position.y
    local_x = (origin_cosine * dx + origin_sine * dy) / resolution
    local_y = (-origin_sine * dx + origin_cosine * dy) / resolution
    cell_x = math.floor(local_x)
    cell_y = math.floor(local_y)
    if (cell_x < 0 or cell_y < 0 or
            cell_x >= int(grid.info.width) or cell_y >= int(grid.info.height)):
        return None
    center_x = (cell_x + 0.5) * resolution
    center_y = (cell_y + 0.5) * resolution
    return (
        (cell_x, cell_y),
        (
            origin.position.x + origin_cosine * center_x - origin_sine * center_y,
            origin.position.y + origin_sine * center_x + origin_cosine * center_y,
        ),
    )


def footprint_pose_diagnostic(
        grid: OccupancyGrid, pose: tuple[float, float, float],
        footprint: tuple[Point, ...],
        lethal_threshold: int = LOCAL_FOOTPRINT_GATE_THRESHOLD,
) -> FootprintPoseEvidence:
    """Locate the first hard footprint cell without changing gate semantics."""
    clear, reason, maximum_cost = footprint_cost_at_pose(
        grid, pose, footprint, lethal_threshold,
    )
    if clear or grid is None or not footprint:
        return FootprintPoseEvidence(clear, reason, maximum_cost)
    resolution = float(grid.info.resolution)
    if resolution <= 0.0:
        return FootprintPoseEvidence(False, reason, maximum_cost)
    x, y, yaw = pose
    cosine, sine = math.cos(yaw), math.sin(yaw)
    polygon = tuple(
        (x + cosine * point[0] - sine * point[1],
         y + sine * point[0] + cosine * point[1])
        for point in footprint
    )
    origin = grid.info.origin
    origin_q = origin.orientation
    origin_yaw = math.atan2(
        2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
        1.0 - 2.0 * (origin_q.y ** 2 + origin_q.z ** 2),
    )
    origin_cosine, origin_sine = math.cos(origin_yaw), math.sin(origin_yaw)
    grid_points = tuple(
        ((origin_cosine * (point[0] - origin.position.x) +
          origin_sine * (point[1] - origin.position.y)) / resolution,
         (-origin_sine * (point[0] - origin.position.x) +
          origin_cosine * (point[1] - origin.position.y)) / resolution)
        for point in polygon
    )
    min_x = max(0, math.floor(min(point[0] for point in grid_points)) - 1)
    max_x = min(grid.info.width - 1, math.ceil(max(point[0] for point in grid_points)) + 1)
    min_y = max(0, math.floor(min(point[1] for point in grid_points)) - 1)
    max_y = min(grid.info.height - 1, math.ceil(max(point[1] for point in grid_points)) + 1)
    cell_half_diagonal = resolution * math.sqrt(2.0) / 2.0
    for cell_y in range(min_y, max_y + 1):
        for cell_x in range(min_x, max_x + 1):
            center_x = (cell_x + 0.5) * resolution
            center_y = (cell_y + 0.5) * resolution
            center = (
                origin.position.x + origin_cosine * center_x - origin_sine * center_y,
                origin.position.y + origin_sine * center_x + origin_cosine * center_y,
            )
            in_polygon = _point_in_polygon(center, polygon)
            near_edge = any(
                _segment_distance(center, polygon[index - 1], polygon[index])
                <= cell_half_diagonal for index in range(len(polygon))
            )
            if not in_polygon and not near_edge:
                continue
            index = cell_y * grid.info.width + cell_x
            if index >= len(grid.data):
                continue
            value = int(grid.data[index])
            if value < 0 or value >= lethal_threshold:
                return FootprintPoseEvidence(
                    False, reason, maximum_cost, (cell_x, cell_y), center,
                )
    for index, first in enumerate(polygon):
        second = polygon[index - 1]
        distance = math.dist(first, second)
        steps = max(1, math.ceil(distance / (resolution * 0.5)))
        for step in range(steps + 1):
            fraction = step / steps
            sample = (
                first[0] + fraction * (second[0] - first[0]),
                first[1] + fraction * (second[1] - first[1]),
            )
            value = occupancy_value(grid, sample)
            if value is None:
                continue
            if value < 0 or value >= lethal_threshold:
                location = _grid_cell_world_coordinates(grid, sample)
                if location is None:
                    return FootprintPoseEvidence(False, reason, maximum_cost)
                return FootprintPoseEvidence(
                    False, reason, maximum_cost, location[0], location[1],
                )
    return FootprintPoseEvidence(False, reason, maximum_cost)


def local_footprint_path_clearance(
        grid: Optional[OccupancyGrid],
        poses: tuple[tuple[float, float, float], ...],
        footprint: tuple[Point, ...],
        lethal_threshold: int = LOCAL_FOOTPRINT_GATE_THRESHOLD,
) -> LocalFootprintClearance:
    """Check the contiguous local-costmap prefix with the complete polygon."""
    def metadata() -> dict:
        if grid is None:
            return {'effective_footprint': footprint}
        origin = grid.info.origin
        origin_q = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (origin_q.w * origin_q.z + origin_q.x * origin_q.y),
            1.0 - 2.0 * (origin_q.y ** 2 + origin_q.z ** 2),
        )
        stamp = grid.header.stamp
        return {
            'effective_footprint': footprint,
            'costmap_frame': str(grid.header.frame_id or ''),
            'costmap_origin': (
                float(origin.position.x), float(origin.position.y)),
            'costmap_origin_yaw': origin_yaw,
            'costmap_width': int(grid.info.width),
            'costmap_height': int(grid.info.height),
            'costmap_resolution': float(grid.info.resolution),
            'costmap_stamp_ns': int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
        }

    common = metadata()
    if grid is None:
        return LocalFootprintClearance(
            clear=False, reason='NO_LOCAL_COSTMAP', inspected_poses=0,
            outside_poses=0, **common,
        )
    if not poses:
        return LocalFootprintClearance(
            clear=False, reason='NO_LOCAL_PATH', inspected_poses=0,
            outside_poses=0, **common,
        )
    if not footprint:
        return LocalFootprintClearance(
            clear=False, reason='NO_EFFECTIVE_FOOTPRINT', inspected_poses=0,
            outside_poses=0, **common,
        )
    inspected = 0
    outside = 0
    maximum_cost = None
    for pose_index, pose in enumerate(poses):
        center_value = occupancy_value(grid, (pose[0], pose[1]))
        if center_value is None:
            outside += 1
            if inspected == 0:
                return LocalFootprintClearance(
                    clear=False, reason='NO_LOCAL_PREFIX',
                    inspected_poses=inspected, outside_poses=outside,
                    maximum_cost=maximum_cost, **common,
                )
            return LocalFootprintClearance(
                clear=True, reason='LOCAL_PREFIX_EXHAUSTED',
                inspected_poses=inspected, outside_poses=outside,
                maximum_cost=maximum_cost, **common,
            )
        evidence = footprint_pose_diagnostic(
            grid, pose, footprint, lethal_threshold,
        )
        clear, reason, pose_maximum = (
            evidence.clear, evidence.reason, evidence.maximum_cost)
        maximum_cost = (
            pose_maximum if maximum_cost is None else
            max(maximum_cost, pose_maximum or maximum_cost)
        )
        if not clear:
            # The path center is still in the rolling window, but the
            # footprint at this boundary pose extends beyond the currently
            # observable grid.  Treat the remaining path as an unobserved
            # local prefix, not as a collision.  The global IsPathValid check
            # remains authoritative for the complete planner path.
            if reason == 'FOOTPRINT_OUT_OF_LOCAL_WINDOW':
                outside += 1
                if inspected == 0:
                    return LocalFootprintClearance(
                        clear=False, reason='NO_LOCAL_PREFIX',
                        inspected_poses=inspected, outside_poses=outside,
                        maximum_cost=maximum_cost, **common,
                    )
                return LocalFootprintClearance(
                    clear=True, reason='LOCAL_PREFIX_EXHAUSTED',
                    inspected_poses=inspected, outside_poses=outside,
                    maximum_cost=maximum_cost, **common,
                )
            return LocalFootprintClearance(
                clear=False, reason=reason, inspected_poses=inspected + 1,
                outside_poses=outside, maximum_cost=maximum_cost,
                first_blocked_pose_index=pose_index,
                first_blocked_pose=(float(pose[0]), float(pose[1])),
                first_blocked_pose_yaw=float(pose[2]),
                first_blocked_cell=evidence.blocked_cell,
                first_blocked_cell_world=evidence.blocked_cell_world,
                **common,
            )
        inspected += 1
    return LocalFootprintClearance(
        clear=True, reason='CLEAR', inspected_poses=inspected,
        outside_poses=outside, maximum_cost=maximum_cost, **common,
    )


@dataclass(frozen=True)
class LocalPathClearance:
    """Bounded evidence for the contiguous path prefix in the local grid."""

    clear: bool
    reason: str
    inspected_points: int
    outside_points: int

    maximum_cost: Optional[int] = None
    first_blocked_point_index: Optional[int] = None
    first_blocked_path_point: Optional[Point] = None
    first_blocked_local_point: Optional[Point] = None


def local_path_clearance(
        grid: Optional[OccupancyGrid], points: tuple[Point, ...],
        transform_point: Callable[[Point], Optional[Point]],
        blocked_threshold: int = 80) -> LocalPathClearance:
    """Check the portion of a global path visible in the rolling local grid.

    Global NavFn is intentionally allowed to traverse unknown space so it can
    reach a frontier.  That makes a valid global path insufficient at the
    dispatch boundary: its first metres can still enter an inflated obstacle
    that the rolling local controller will stop for.  Only the contiguous
    prefix that lies inside the rolling window is an execution-horizon gate.
    Once the path leaves that window, the farther segment is covered by the
    global path validation and is not re-entered if the path later loops back.
    The transform is injected so this geometry rule remains deterministic and
    unit-testable without a ROS TF graph.
    """
    if grid is None:
        return LocalPathClearance(False, 'NO_LOCAL_COSTMAP', 0, 0)
    inspected = 0
    outside = 0
    maximum_cost = None
    for point_index, point in enumerate(points):
        local = transform_point(point)
        if local is None:
            return LocalPathClearance(
                False, 'TRANSFORM_UNAVAILABLE', inspected, outside,
            )
        value = occupancy_value(grid, local)
        if value is None:
            outside += 1
            break
        maximum_cost = (
            int(value) if maximum_cost is None else
            max(int(maximum_cost), int(value))
        )
        # Nav2's path normally starts at the robot pose.  The rolling local
        # costmap can mark that exact footprint cell as inflated/lethal even
        # while the immediately-following execution corridor is clear.  The
        # start sample is a pose anchor, not a segment the controller must
        # enter; keep unknown start cells conservative, but do not reject a
        # valid path solely because the footprint anchor is occupied.
        if point_index == 0 and value >= blocked_threshold:
            continue
        if value < 0 or value >= blocked_threshold:
            reason = 'UNKNOWN_LOCAL_CELL' if value < 0 else 'BLOCKED_LOCAL_CELL'
            return LocalPathClearance(
                False, reason, inspected + 1, outside, maximum_cost,
                point_index, (float(point[0]), float(point[1])),
                (float(local[0]), float(local[1])),
            )
        inspected += 1
    return LocalPathClearance(
        True, 'CLEAR', inspected, outside, maximum_cost,
    )


def local_path_clear(
        grid: Optional[OccupancyGrid], points: tuple[Point, ...],
        transform_point: Callable[[Point], Optional[Point]],
        blocked_threshold: int = 80) -> bool:
    """Boolean compatibility wrapper for the local path safety predicate."""
    return local_path_clearance(
        grid, points, transform_point, blocked_threshold,
    ).clear


class LocalNav2:
    """Own only this node namespace's planner, navigator, state, TF, and odometry."""

    def __init__(self, node: Node, *, phase_gated: bool = False):
        """Create relative interfaces that resolve inside the local robot namespace."""
        self._node = node
        self._phase_inputs_active = not phase_gated
        self._robot_id = node.get_namespace().strip('/') or 'root'
        self._global_frame = node.declare_parameter('global_frame', 'shared_map').value
        self._base_frame = node.declare_parameter('robot_base_frame', 'base_footprint').value
        self._nav2_node_prefix = str(node.declare_parameter(
            'nav2_node_prefix', '').value)
        self._planner_id = node.declare_parameter('planner_id', 'GridBased').value
        self._path_timeout_s = float(node.declare_parameter('path_query_timeout_s', 1.5).value)
        self._navigation_timeout_s = float(
            node.declare_parameter('navigation_timeout_s', 240.0).value,
        )
        self._navigation_no_progress_timeout_s = float(
            node.declare_parameter(
                'navigation_no_progress_timeout_s', 30.0,
            ).value,
        )
        self._navigation_min_progress_m = float(
            node.declare_parameter('navigation_min_progress_m', 0.05).value,
        )
        self._maximum_path_samples = int(
            node.declare_parameter('maximum_path_samples', 32).value,
        )
        self._maximum_tf_age_s = float(
            node.declare_parameter('maximum_tf_age_s', 1.0).value,
        )
        self._costmap_lethal_threshold = int(
            node.declare_parameter('costmap_lethal_threshold', 253).value,
        )
        self._local_path_gate_mode = str(node.declare_parameter(
            'local_path_gate_mode', LOCAL_PATH_GATE_MODE_A).value).upper()
        if self._local_path_gate_mode not in LOCAL_PATH_GATE_MODES:
            raise ValueError(
                'local_path_gate_mode must be MODE_A or MODE_B')
        namespace = node.get_namespace().strip('/') or 'root'
        self._path_query_lock_path = str(node.declare_parameter(
            'path_query_lock_path',
            '/tmp/my_epuck_%s_compute_path.lock' % namespace,
        ).value)
        self._path_priority_path = self._path_query_lock_path + '.fallback_priority'
        self._path_query_lock_file = None
        self._last_path_start_failure_reason = ''
        # These waitables are created on first real local work.  Constructing
        # them at process startup makes every idle assignment peer poll two
        # action clients and three lifecycle services even before a frontier
        # exists; lazy creation preserves the exact interfaces and checks
        # once a candidate is available.
        self._compute_client = None
        self._navigate_client = None
        self._path_valid_client = None
        self._lifecycle_clients = {}
        transient_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_topic = str(node.declare_parameter('map_topic', 'shared_map').value)
        self._path_valid_service_name = str(node.declare_parameter(
            'path_valid_service', 'is_path_valid').value,
        )
        self._path_valid_timeout_s = float(node.declare_parameter(
            'path_valid_timeout_s', 0.5).value)
        self._local_footprint_topic = str(node.declare_parameter(
            'local_footprint_topic', 'local_costmap/published_footprint').value,
        )
        self._local_footprint_max_age_s = float(node.declare_parameter(
            'local_footprint_max_age_s', 2.0).value)
        self._maximum_local_footprint_samples = int(node.declare_parameter(
            'maximum_local_footprint_samples', 2000).value)
        self._phase_subscriptions = []
        # Scan freshness is diagnostic-only.  The production Nav2/SLAM stack
        # publishes the corrected scan as scan_d500_fixed.  The allocator
        # intentionally does not create a cmd_vel interface; command capture
        # belongs to the observer node so exploration remains a local Nav2
        # action boundary.
        self._scan_topic = str(node.declare_parameter(
            'diagnostic_scan_topic', 'scan_d500_fixed').value)
        self._tf_buffer = Buffer(node=node)
        self._tf_listener = None
        self._map: Optional[OccupancyGrid] = None
        self._costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._latest_footprint: Optional[PolygonStamped] = None
        self._distance = TravelDistance(maximum_step_m=0.5)
        self._active_path_request = 0
        self._path_deadline_steady_s = 0.0
        self._path_callback: Optional[Callable[[PathEvaluation], None]] = None
        self._path_started_steady_s = 0.0
        self._path_started_ros_ns = 0
        self._path_caller = 'UNKNOWN'
        self._path_task_signature = ''
        self._path_map_stamp_ns = 0
        self._path_costmap_stamp_ns = 0
        self._path_goal_handle = None
        self._path_valid_pending = None
        self._navigation_goal_handle = None
        self._navigation_send_pending = False
        self._navigation_callback: Optional[Callable[[NavigationOutcome], None]] = None
        self._navigation_started_steady_s = 0.0
        self._navigation_start_distance_m = 0.0
        self._navigation_last_progress_distance_m = 0.0
        self._navigation_last_progress_steady_s = 0.0
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        self._navigation_no_progress_requested = False
        self._navigation_no_progress_requested_ros_ns = 0
        self._navigation_cancel_requested = False
        self._navigation_target: Optional[tuple[float, float]] = None
        self._navigation_physical_signature = ''
        self._navigation_diagnostic_path: tuple[tuple[float, float], ...] = ()
        self._navigation_goal_number = 0
        self._navigation_point_validation: Optional[dict] = None
        self._navigation_point_last_sample_steady_s = 0.0
        self._navigation_point_sample_period_s = 0.5
        self._last_odom_stamp_ns = 0
        self._last_odom_linear = (0.0, 0.0)
        self._last_odom_angular_z = 0.0
        self._last_scan_stamp_ns = 0
        self._last_scan_min_range: Optional[float] = None
        self._last_scan_summary_steady_s = 0.0
        self._recent_odom_samples = deque(maxlen=32)
        self._last_lifecycle_active: Optional[bool] = None
        self._lifecycle_health_pending = False
        self._timer = None
        if self._phase_inputs_active:
            self._activate_phase_inputs(transient_qos)

    def _ensure_compute_client(self) -> None:
        if self._compute_client is None:
            self._compute_client = ActionClient(
                self._node, ComputePathToPose, 'compute_path_to_pose')

    def _ensure_navigate_client(self) -> None:
        if self._navigate_client is None:
            self._navigate_client = ActionClient(
                self._node, NavigateToPose, 'navigate_to_pose')

    def _ensure_path_valid_client(self) -> None:
        if self._path_valid_client is None:
            self._path_valid_client = self._node.create_client(
                IsPathValid, self._path_valid_service_name)

    def _ensure_lifecycle_clients(self) -> None:
        if self._lifecycle_clients:
            return
        self._lifecycle_clients = {
            name: self._node.create_client(
                GetState, f'{self._nav2_node_prefix}{name}/get_state')
            for name in ('planner_server', 'controller_server', 'bt_navigator')
        }

    def check_navigation_readiness(self, callback: Callable[[DispatchPreconditions], None]) -> None:
        """Check only the readiness needed to submit NavigateToPose.

        The Robot 2 solo opt-in already receives a completed planner result
        from the candidate generator.  It must not run a second planner,
        IsPathValid, or allocator-side geometry/footprint gate.  Nav2 remains
        responsible for live costmap, footprint, and RPP collision handling.
        """
        self._ensure_navigate_client()
        self._ensure_lifecycle_clients()

        def result(action_server_ready: bool, lifecycle_active: bool, reason: str):
            callback(DispatchPreconditions(
                action_server_ready=action_server_ready,
                lifecycle_active=lifecycle_active,
                transform_available=False,
                transform_age_s=None,
                goal_inside_map=False,
                goal_inside_costmap=False,
                goal_map_value=None,
                goal_costmap_value=None,
                local_path_clear=False,
                no_local_goal_active=not self.local_goal_active,
                final_path_valid=False,
                reason=reason,
                navigation_readiness_only=True,
            ))

        if not self._navigate_client.server_is_ready():
            result(False, False, 'local NavigateToPose action server unavailable')
            return

        unavailable = [
            name for name, client in self._lifecycle_clients.items()
            if not client.service_is_ready()
        ]
        if unavailable:
            result(
                True,
                False,
                'lifecycle services unavailable: ' + ','.join(unavailable),
            )
            return

        states = {}

        def completed(name, future):
            try:
                states[name] = future.result().current_state.label
            except Exception as error:  # noqa: B902
                states[name] = 'error:' + str(error)
            if len(states) != len(self._lifecycle_clients):
                return
            active = all(value == 'active' for value in states.values())
            self._last_lifecycle_active = active
            result(
                True,
                active,
                '' if active else 'inactive lifecycle nodes: ' + str(states),
            )

        for name, client in self._lifecycle_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda response, key=name: completed(key, response))

    def _activate_phase_inputs(self, transient_qos=None) -> None:
        """Start map/health callbacks for a post-handoff shared phase."""
        if self._phase_inputs_active and self._timer is not None:
            return
        if transient_qos is None:
            transient_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
        self._phase_inputs_active = True
        if self._tf_listener is None:
            self._tf_listener = TransformListener(
                self._tf_buffer, self._node, spin_thread=False)
        self._phase_subscriptions.extend([
            self._node.create_subscription(
                OccupancyGrid, self._map_topic, self._on_map, transient_qos),
            self._node.create_subscription(
                OccupancyGrid, 'global_costmap/costmap', self._on_costmap,
                transient_qos),
            self._node.create_subscription(
                OccupancyGrid, 'local_costmap/costmap', self._on_local_costmap,
                transient_qos),
            self._node.create_subscription(
                PolygonStamped, self._local_footprint_topic,
                self._on_footprint, 10),
            self._node.create_subscription(Odometry, 'odom', self._on_odom, 20),
            self._node.create_subscription(LaserScan, self._scan_topic,
                                            self._on_scan, 20),
        ])
        self._timer = self._node.create_timer(0.1, self._check_timeouts)

    @property
    def local_goal_active(self) -> bool:
        """Return whether this wrapper owns an unresolved local navigation goal."""
        return (
            self._navigation_goal_handle is not None or
            self._navigation_send_pending
        )

    @property
    def shared_map(self) -> Optional[OccupancyGrid]:
        """Expose the local shared-map replica for allocator LOS evaluation."""
        return self._map

    @property
    def travelled_distance_m(self) -> float:
        """Return measured cumulative local odometry displacement."""
        return self._distance.distance_m

    def interface_names(self) -> dict[str, str]:
        """Expose resolved local names for static integration auditing."""
        namespace = self._node.get_namespace().rstrip('/')
        return {
            'compute_path': f'{namespace}/compute_path_to_pose',
            'navigate': f'{namespace}/navigate_to_pose',
            'odom': f'{namespace}/odom',
        }

    def refresh_health(self) -> None:
        """Refresh managed-node state without blocking the coordinator timer."""
        if not self._lifecycle_clients:
            self._last_lifecycle_active = False
            return
        if self._lifecycle_health_pending:
            return
        if any(not client.service_is_ready() for client in self._lifecycle_clients.values()):
            self._last_lifecycle_active = False
            return
        self._lifecycle_health_pending = True
        states = {}

        def completed(name, future):
            try:
                states[name] = future.result().current_state.label
            except Exception:  # noqa: B902
                states[name] = 'error'
            if len(states) == len(self._lifecycle_clients):
                self._last_lifecycle_active = all(
                    value == 'active' for value in states.values()
                )
                self._lifecycle_health_pending = False

        for name, client in self._lifecycle_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda result, key=name: completed(key, result))

    def health_flags(self) -> tuple[bool, bool]:
        """Return conservative current Nav2 and required-transform health flags."""
        nav2_healthy = (
            self._compute_client is not None and
            self._navigate_client is not None and
            self._compute_client.server_is_ready() and
            self._navigate_client.server_is_ready() and
            self._last_lifecycle_active is True and
            self._map is not None and self._costmap is not None
        )
        tf_healthy = self.shared_tf_status()[0]
        return nav2_healthy, tf_healthy

    def shared_tf_status(self) -> tuple[bool, Optional[float], str]:
        """Return the exact shared-map/base TF readiness used by dispatch.

        The distributed allocator uses this as a *pre-decision* gate.  It
        deliberately applies the same frame pair and freshness limit as the
        final dispatch health check; it does not synthesize or fall back to a
        remembered/ground-truth pose.
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as error:
            return False, None, 'required transform unavailable: %s' % error
        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_s = 0.0 if stamp_ns == 0 else max(
            0.0, (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
        )
        if stamp_ns != 0 and age_s > self._maximum_tf_age_s:
            return False, age_s, (
                f'{self._global_frame} to {self._base_frame} '
                'transform is stale'
            )
        return True, age_s, ''

    def synchronized_test_inputs_ready(self) -> bool:
        """Return the local inputs needed before the test barrier advertises a round.

        The shared phase manager has already established lifecycle readiness.
        This test-only predicate deliberately avoids depending on the
        asynchronous diagnostic lifecycle cache, while still requiring both
        action servers and the current map/costmap samples.
        """
        self._ensure_compute_client()
        self._ensure_navigate_client()
        return bool(
            self._compute_client.server_is_ready() and
            self._navigate_client.server_is_ready() and
            self._map is not None and self._costmap is not None
        )

    def lookup_pose_in_global(
            self, target_frame: str) -> Optional[tuple[Point, int, float]]:
        """Return a fresh target-frame pose expressed in this Nav2 global frame.

        The traffic gate uses this only for an already committed peer path.
        A zero-time lookup asks tf2 for the newest available transform; the
        returned age is measured against the node's ROS clock and callers must
        reject an unavailable/stale result rather than masking a remembered
        location.
        """
        if not target_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, str(target_frame), Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None
        stamp = transform.header.stamp
        stamp_ns = self._stamp_ns(stamp)
        now_ns = self._node.get_clock().now().nanoseconds
        age_s = 0.0 if stamp_ns == 0 else max(
            0.0, (now_ns - stamp_ns) / 1e9,
        )
        if stamp_ns != 0 and age_s > self._maximum_tf_age_s:
            return None
        return (
            (float(transform.transform.translation.x),
             float(transform.transform.translation.y)),
            stamp_ns,
            age_s,
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self._map = message

    def _on_costmap(self, message: OccupancyGrid) -> None:
        self._costmap = message

    def _on_local_costmap(self, message: OccupancyGrid) -> None:
        """Retain only the latest rolling costmap for failure diagnostics."""
        self._local_costmap = message

    def _on_footprint(self, message: PolygonStamped) -> None:
        """Retain Nav2's effective, padded footprint for local preflight."""
        self._latest_footprint = message

    def preflight_inputs_available(self) -> bool:
        """Return whether the local preflight inputs are currently present."""
        grid = self._local_costmap
        footprint = self._latest_footprint
        return bool(
            grid is not None and footprint is not None and
            len(footprint.polygon.points) >= 3 and
            str(footprint.header.frame_id or '') ==
            str(grid.header.frame_id or '')
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2),
        )

    def _effective_footprint_relative(self) -> tuple[Optional[tuple[Point, ...]], str]:
        """Recover Nav2's effective padded polygon in the base frame."""
        grid = self._local_costmap
        message = self._latest_footprint
        if grid is None:
            return None, 'NO_LOCAL_COSTMAP'
        if message is None or len(message.polygon.points) < 3:
            return None, 'NO_EFFECTIVE_FOOTPRINT'
        if message.header.frame_id != grid.header.frame_id:
            return None, 'FOOTPRINT_FRAME_MISMATCH'
        stamp_ns = self._stamp_ns(message.header.stamp)
        if stamp_ns:
            age_s = max(
                0.0,
                (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
            )
            if age_s > self._local_footprint_max_age_s:
                return None, 'STALE_EFFECTIVE_FOOTPRINT'
        try:
            lookup_time = Time.from_msg(message.header.stamp) if stamp_ns else Time()
            transform = self._tf_buffer.lookup_transform(
                grid.header.frame_id, self._base_frame, lookup_time,
                timeout=Duration(seconds=0.1),
            )
        except TransformException as error:
            return None, 'FOOTPRINT_BASE_TF_UNAVAILABLE: %s' % error
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        translation = transform.transform.translation
        relative = []
        for point in message.polygon.points:
            dx = float(point.x) - float(translation.x)
            dy = float(point.y) - float(translation.y)
            relative.append((
                cosine * dx + sine * dy,
                -sine * dx + cosine * dy,
            ))
        return tuple(relative), ''

    def _path_poses_in_local_frame(
            self, path: Path,
    ) -> tuple[Optional[tuple[tuple[float, float, float], ...]], str]:
        """Transform every planner pose into the current rolling-costmap frame."""
        self._last_path_transform_evidence = {}
        if path is None or not path.poses:
            return None, 'NO_PLANNER_PATH'
        local_frame = '' if self._local_costmap is None else self._local_costmap.header.frame_id
        if not local_frame:
            return None, 'NO_LOCAL_COSTMAP_FRAME'
        path_frame = str(path.header.frame_id or self._global_frame)
        transform = None
        if path_frame != local_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    local_frame, path_frame, Time(), timeout=Duration(seconds=0.1),
                )
            except TransformException as error:
                return None, 'PATH_LOCAL_TF_UNAVAILABLE: %s' % error
            stamp_ns = self._stamp_ns(transform.header.stamp)
            if stamp_ns:
                age_s = max(
                    0.0,
                    (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
                )
                if age_s > self._maximum_tf_age_s:
                    return None, 'STALE_PATH_LOCAL_TF'
        if transform is None:
            transform_yaw = 0.0
            transform_x = 0.0
            transform_y = 0.0
        else:
            transform_yaw = self._yaw_from_quaternion(transform.transform.rotation)
            transform_x = float(transform.transform.translation.x)
            transform_y = float(transform.transform.translation.y)
            stamp = transform.header.stamp
            stamp_ns = self._stamp_ns(stamp)
            age_s = None
            if stamp_ns:
                age_s = max(
                    0.0,
                    (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
                )
            self._last_path_transform_evidence = {
                'path_transform_translation': (transform_x, transform_y),
                'path_transform_yaw': transform_yaw,
                'path_transform_stamp_ns': stamp_ns,
                'path_transform_age_s': age_s,
            }
        cosine, sine = math.cos(transform_yaw), math.sin(transform_yaw)
        poses = []
        for pose_stamped in path.poses:
            pose_frame = str(pose_stamped.header.frame_id or path_frame)
            if pose_frame != path_frame:
                return None, 'MIXED_PATH_FRAMES'
            position = pose_stamped.pose.position
            pose_yaw = self._yaw_from_quaternion(pose_stamped.pose.orientation)
            poses.append((
                cosine * float(position.x) - sine * float(position.y) + transform_x,
                sine * float(position.x) + cosine * float(position.y) + transform_y,
                math.atan2(
                    math.sin(transform_yaw + pose_yaw),
                    math.cos(transform_yaw + pose_yaw),
                ),
            ))
        return tuple(poses), ''

    def _local_footprint_path_check(self, path: Optional[Path]) -> LocalFootprintClearance:
        """Run the minimal rolling-costmap polygon preflight."""
        footprint, footprint_reason = self._effective_footprint_relative()
        if footprint is None:
            return LocalFootprintClearance(False, footprint_reason, 0, 0)
        poses, pose_reason = self._path_poses_in_local_frame(path)
        if poses is None:
            return LocalFootprintClearance(False, pose_reason, 0, 0)
        resolution = float(self._local_costmap.info.resolution)
        samples = interpolate_pose2d(poses, resolution * 0.5)
        if len(samples) > self._maximum_local_footprint_samples:
            return LocalFootprintClearance(
                False, 'LOCAL_FOOTPRINT_SAMPLE_LIMIT', 0, 0,
            )
        clearance = local_footprint_path_clearance(
            self._local_costmap, samples, footprint,
            lethal_threshold=LOCAL_FOOTPRINT_GATE_THRESHOLD,
        )
        return replace(clearance, **getattr(
            self, '_last_path_transform_evidence', {}))

    def _request_path_valid(
            self, path: Optional[Path], callback: Callable[[bool, tuple[int, ...], str], None],
    ) -> None:
        """Request Nav2's global full-footprint check with a bounded timeout."""
        self._ensure_path_valid_client()
        if path is None or not path.poses:
            callback(False, (), 'NO_PLANNER_PATH')
            return
        if self._path_valid_pending is not None:
            callback(False, (), 'PATH_VALID_SERVICE_BUSY')
            return
        if not self._path_valid_client.service_is_ready():
            callback(False, (), 'PATH_VALID_SERVICE_UNAVAILABLE')
            return
        request = IsPathValid.Request()
        request.path = path
        generation = getattr(self, '_path_valid_generation', 0) + 1
        self._path_valid_generation = generation
        self._path_valid_pending = (
            generation, time.monotonic() + self._path_valid_timeout_s, callback,
        )
        future = self._path_valid_client.call_async(request)
        future.add_done_callback(
            lambda result: self._path_valid_response(generation, result),
        )

    def _path_valid_response(self, generation: int, future) -> None:
        pending = self._path_valid_pending
        if pending is None or pending[0] != generation:
            return
        self._path_valid_pending = None
        callback = pending[2]
        try:
            response = future.result()
            valid, indices, reason = path_validity_decision(
                response.is_valid, response.invalid_pose_indices,
            )
        except Exception as error:  # noqa: B902
            valid = False
            indices = ()
            reason = 'PATH_VALID_SERVICE_ERROR: %s' % error
        self._node.get_logger().info(
            'PATH_VALIDATION_RESULT service=%s valid=%s invalid_pose_indices=%s reason=%s' % (
                self._path_valid_service_name, valid, indices, reason,
            ),
        )
        callback(valid, indices, reason)

    @staticmethod
    def _grid_stamp(message: Optional[OccupancyGrid]) -> int:
        """Return the source timestamp used for bounded path freshness."""
        if message is None:
            return 0
        return int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec)

    def path_context_matches(self, evaluation: PathEvaluation) -> bool:
        """Reject bid reuse when either local planning input has advanced."""
        if evaluation.map_stamp_ns == 0 or evaluation.costmap_stamp_ns == 0:
            return False
        return (
            self._grid_stamp(self._map) == evaluation.map_stamp_ns and
            self._grid_stamp(self._costmap) == evaluation.costmap_stamp_ns
        )

    def _on_odom(self, message: Odometry) -> None:
        self._last_odom_stamp_ns = self._stamp_ns(message.header.stamp)
        self._last_odom_linear = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.linear.y),
        )
        self._last_odom_angular_z = float(message.twist.twist.angular.z)
        self._recent_odom_samples.append({
            'stamp_ns': self._last_odom_stamp_ns,
            'linear_x_mps': self._last_odom_linear[0],
            'angular_z_radps': self._last_odom_angular_z,
        })
        position = message.pose.pose.position
        distance = self._distance.observe((position.x, position.y))
        if (self._navigation_goal_handle is not None and
                distance - self._navigation_last_progress_distance_m >=
                self._navigation_min_progress_m):
            self._navigation_last_progress_distance_m = distance
            self._navigation_last_progress_steady_s = time.monotonic()

    @staticmethod
    def _stamp_ns(stamp) -> int:
        """Convert a ROS builtin time stamp to nanoseconds."""
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _on_scan(self, message: LaserScan) -> None:
        self._last_scan_stamp_ns = self._stamp_ns(message.header.stamp)
        # The assignment boundary only retains scan freshness and a value for
        # bounded failure diagnostics; Nav2/Collision Monitor own scan-driven
        # safety.  Avoid rebuilding a 720-element Python list for every scan
        # in every assignment process.
        now = time.monotonic()
        if now - self._last_scan_summary_steady_s >= 0.5:
            finite = (float(value) for value in message.ranges
                      if math.isfinite(value))
            self._last_scan_min_range = min(finite, default=None)
            self._last_scan_summary_steady_s = now

    def _age_s(self, stamp_ns: int, now_ns: int) -> Optional[float]:
        """Return a non-negative source age, or None for missing time."""
        if not stamp_ns:
            return None
        return max(0.0, (now_ns - stamp_ns) / 1e9)

    def _point_validation_sample(self) -> dict:
        """Capture point-level upstream validation without affecting dispatch."""
        now_ns = self._node.get_clock().now().nanoseconds
        target = self._navigation_target
        global_result = upstream_point_validation(self._costmap, target) if target else {
            'status': 'NO_DATA', 'cost': None, 'in_bounds': False,
        }
        local_result = {'status': 'NO_DATA', 'cost': None, 'in_bounds': False}
        local_point = None
        local_tf_stamp_ns = 0
        local_frame = '' if self._local_costmap is None else self._local_costmap.header.frame_id
        if self._local_costmap is not None and target is not None:
            transformed = _transform_point(
                self._tf_buffer, local_frame, self._global_frame, target,
            )
            if transformed is None:
                local_result = {
                    'status': 'LOCAL_TF_UNAVAILABLE', 'cost': None, 'in_bounds': False,
                }
            else:
                local_point, local_tf_stamp_ns = transformed
                local_result = upstream_point_validation(self._local_costmap, local_point)
                if local_result['status'] == 'OUT_OF_BOUNDS':
                    local_result = {
                        **local_result, 'status': 'OUT_OF_LOCAL_WINDOW',
                    }
        return {
            'sample_ros_ns': int(now_ns),
            'global_costmap_frame': '' if self._costmap is None else self._costmap.header.frame_id,
            'global_costmap_stamp_ns': self._grid_stamp(self._costmap),
            'global_costmap_age_s': self._age_s(self._grid_stamp(self._costmap), now_ns),
            'global_point_status': global_result['status'],
            'global_point_cost': global_result['cost'],
            'global_point_in_bounds': bool(global_result['in_bounds']),
            'local_costmap_frame': local_frame,
            'local_costmap_stamp_ns': self._grid_stamp(self._local_costmap),
            'local_costmap_age_s': self._age_s(self._grid_stamp(self._local_costmap), now_ns),
            'local_point_frame': local_frame,
            'local_point_xy': local_point,
            'local_tf_stamp_ns': int(local_tf_stamp_ns),
            'local_tf_age_s': self._age_s(local_tf_stamp_ns, now_ns),
            'local_point_status': local_result['status'],
            'local_point_cost': local_result['cost'],
            'local_point_in_bounds': bool(local_result['in_bounds']),
            'tf_stamp_ns': self._last_tf_stamp_ns(),
            'tf_age_s': self._age_s(self._last_tf_stamp_ns(), now_ns),
            'odom_stamp_ns': self._last_odom_stamp_ns,
            'odom_age_s': self._age_s(self._last_odom_stamp_ns, now_ns),
            'scan_stamp_ns': self._last_scan_stamp_ns,
            'scan_age_s': self._age_s(self._last_scan_stamp_ns, now_ns),
        }

    def _last_tf_stamp_ns(self) -> int:
        """Read the latest shared-map-to-base transform stamp for telemetry."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return 0
        return _stamp_ns_static(transform.header.stamp)

    def _record_point_validation_sample(self, event: str, force: bool = False) -> None:
        """Append only meaningful bounded target-cost transitions."""
        record = self._navigation_point_validation
        if record is None:
            return
        sample = self._point_validation_sample()
        previous = record.get('_last_sample')
        changed = previous is None or any(
            sample.get(key) != previous.get(key)
            for key in (
                'global_point_status', 'global_point_cost', 'local_point_status',
                'local_point_cost', 'local_point_in_bounds',
            )
        )
        if not force and not changed:
            return
        if previous is not None and previous.get('local_point_status') == 'OUT_OF_LOCAL_WINDOW' \
                and sample.get('local_point_in_bounds'):
            event = 'TARGET_ENTERED_LOCAL_WINDOW'
        transition = {**sample, 'event': event}
        transitions = record.setdefault('transitions', [])
        if len(transitions) < 128:
            transitions.append(transition)
        record['_last_sample'] = sample

    def _emit_point_validation(self, phase: str, result: Optional[NavigationOutcome] = None) -> None:
        """Publish compact diagnostic-only dispatch/transition lifecycle data."""
        record = self._navigation_point_validation
        if record is None:
            return
        payload = {
            'schema_version': 'dispatch_point_validation.v1',
            'phase': phase,
            'robot': self._robot_id,
            'goal_number': record['goal_number'],
            'physical_task_signature': record['physical_task_signature'],
            'target': record['target'],
            'path_length_m': record['path_length_m'],
            'dispatch': record['dispatch'],
            'transitions': record.get('transitions', []),
        }
        if result is not None:
            payload['result'] = {
                'accepted': bool(result.accepted),
                'status': int(result.status),
                'error_code': int(result.error_code),
                'error_message': result.error_message,
                'failure_class': result.failure_class.value,
                'duration_s': result.duration_s,
                'travelled_distance_m': result.travelled_distance_m,
                'recoveries': result.recoveries,
                'follow_path_error_code': result.follow_path_error_code,
                'follow_path_error_name': result.follow_path_error_name,
                'controller_failure_family': result.controller_failure_family,
                'deepest_failure_classification': result.deepest_failure_classification,
            }
        self._node.get_logger().info(
            'NAVIGATION_POINT_VALIDATION %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')),
        )

    def _failure_snapshot(self, error_code: int, error_message: str,
                          failure_class: FailureClass) -> str:
        """Serialize one bounded synchronized failure snapshot."""
        now_ns = self._node.get_clock().now().nanoseconds
        tf_stamp_ns = 0
        tf_age_s = None
        tf_error = ''
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
            tf_stamp_ns = self._stamp_ns(transform.header.stamp)
            if tf_stamp_ns:
                tf_age_s = max(0.0, (now_ns - tf_stamp_ns) / 1e9)
        except TransformException as error:
            tf_error = str(error)
        global_costmap_crop = bounded_grid_crop(
            self._costmap, self._navigation_target,
        )
        local_costmap_target_crop = bounded_grid_crop(
            self._local_costmap, self._navigation_target,
        )
        local_costmap_robot_pose = None
        local_costmap_robot_pose_error = ''
        local_costmap_robot_crop = {}
        if self._local_costmap is not None:
            local_frame = self._local_costmap.header.frame_id
            local_costmap_robot_pose = _transform_translation(
                self._tf_buffer, local_frame, self._base_frame)
            if local_costmap_robot_pose is None:
                local_costmap_robot_pose_error = (
                    f'robot pose unavailable in local costmap frame {local_frame!r}')
            local_costmap_robot_crop = bounded_grid_crop(
                self._local_costmap, local_costmap_robot_pose,
            )
        # Keep the legacy local_costmap_crop key robot-centered: it is the
        # rolling navigation window. The target-centered local crop is retained
        # separately because it is normally outside that window by design.
        local_costmap_crop = local_costmap_robot_crop
        signature_crop = local_costmap_robot_crop or local_costmap_target_crop or global_costmap_crop
        snapshot = {
            'schema_version': 'navigation_failure_snapshot.v1',
            'stamp_ros_ns': now_ns,
            'physical_task_signature': self._navigation_physical_signature,
            'target': self._navigation_target,
            'failure_class': failure_class.value,
            'navigate_to_pose_error_code': int(error_code),
            'navigate_to_pose_error_text': error_message,
            'map_stamp_ns': self._grid_stamp(self._map),
            'costmap_stamp_ns': self._grid_stamp(self._costmap),
            'global_costmap_stamp_ns': self._grid_stamp(self._costmap),
            'local_costmap_stamp_ns': self._grid_stamp(self._local_costmap),
            'tf_stamp_ns': tf_stamp_ns,
            'tf_age_s': tf_age_s,
            'tf_error': tf_error,
            'odom_stamp_ns': self._last_odom_stamp_ns,
            'odom_age_s': (
                None if not self._last_odom_stamp_ns else
                max(0.0, (now_ns - self._last_odom_stamp_ns) / 1e9)
            ),
            'odom_linear_x_mps': self._last_odom_linear[0],
            'odom_angular_z_radps': self._last_odom_angular_z,
            'scan_stamp_ns': self._last_scan_stamp_ns,
            'scan_age_s': (
                None if not self._last_scan_stamp_ns else
                max(0.0, (now_ns - self._last_scan_stamp_ns) / 1e9)
            ),
            'scan_min_range_m': self._last_scan_min_range,
            'cmd_capture': 'observer_only; allocator creates no cmd_vel interface',
            'recent_odom_samples': list(self._recent_odom_samples),
            'global_costmap_crop': global_costmap_crop,
            'local_costmap_crop': local_costmap_crop,
            'local_costmap_robot_pose': local_costmap_robot_pose,
            'local_costmap_robot_pose_frame': (
                self._local_costmap.header.frame_id
                if self._local_costmap is not None else ''),
            'local_costmap_robot_pose_error': local_costmap_robot_pose_error,
            'local_costmap_robot_crop': local_costmap_robot_crop,
            'local_costmap_target_crop': local_costmap_target_crop,
            'costmap_source_for_signature': (
                'local_costmap_robot_centered' if local_costmap_robot_crop else
                'local_costmap_target_fallback' if local_costmap_target_crop else
                'global_costmap_fallback' if global_costmap_crop else 'unavailable'
            ),
            # Keep the legacy key for readers of earlier diagnostic bundles.
            'costmap_crop': signature_crop,
            'validated_path_samples': [
                {'x': float(point[0]), 'y': float(point[1])}
                for point in self._navigation_diagnostic_path[:32]
            ],
            'validated_path_sample_count': len(self._navigation_diagnostic_path),
            'footprint_capture': 'Nav2 footprint configuration is external to allocator diagnostics',
        }
        snapshot['execution_geometry_signature'] = execution_geometry_signature(
            self._navigation_target, signature_crop,
        )
        return json.dumps(snapshot, sort_keys=True, separators=(',', ':'))

    def _pose(self, task: PhysicalTask):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = task.approach
        pose.pose.orientation.z = math.sin(task.approach_yaw / 2.0)
        pose.pose.orientation.w = math.cos(task.approach_yaw / 2.0)
        return pose

    def evaluate_path(
            self, task: PhysicalTask,
            callback: Callable[[PathEvaluation], None],
            caller: str = 'ALLOCATOR_BID') -> bool:
        """Start one bounded local path request; return false if busy/unavailable."""
        self._last_path_start_failure_reason = ''
        self._ensure_compute_client()
        if self._path_callback is not None:
            self._last_path_start_failure_reason = 'PATH_REQUEST_ACTIVE'
            return False
        if not self._compute_client.server_is_ready():
            self._last_path_start_failure_reason = 'ACTION_SERVER_UNAVAILABLE'
            return False
        if not self._acquire_path_query_lock():
            self._last_path_start_failure_reason = 'PATH_QUERY_LEASE_BUSY'
            self._request_path_priority()
            return False
        self._clear_path_priority()
        self._active_path_request += 1
        generation = self._active_path_request
        self._path_callback = callback
        self._path_deadline_steady_s = time.monotonic() + self._path_timeout_s
        self._path_started_steady_s = time.monotonic()
        self._path_started_ros_ns = self._node.get_clock().now().nanoseconds
        self._path_caller = str(caller)
        self._path_task_signature = task.physical_signature
        self._path_map_stamp_ns = self._grid_stamp(self._map)
        self._path_costmap_stamp_ns = self._grid_stamp(self._costmap)
        goal = ComputePathToPose.Goal()
        goal.goal = self._pose(task)
        goal.planner_id = self._planner_id
        goal.use_start = False
        future = self._compute_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._path_goal_response(generation, result),
        )
        return True

    def path_start_failure_reason(self) -> str:
        """Return why the most recent path request could not start."""
        return self._last_path_start_failure_reason

    def clear_path_query_priority(self) -> None:
        """Drop an obsolete fallback priority request."""
        self._clear_path_priority()

    def _request_path_priority(self) -> None:
        """Publish a one-shot priority hint for the shared planner lease."""
        temporary = '%s.tmp.%s' % (self._path_priority_path, os.getpid())
        try:
            with open(temporary, 'w', encoding='ascii') as stream:
                stream.write('%d\n' % os.getpid())
            os.replace(temporary, self._path_priority_path)
        except OSError:
            try:
                os.unlink(temporary)
            except OSError:
                pass

    def _clear_path_priority(self) -> None:
        try:
            os.unlink(self._path_priority_path)
        except FileNotFoundError:
            pass
        except OSError:
            pass

    def _path_goal_response(self, generation: int, future) -> None:
        if generation != self._active_path_request or self._path_callback is None:
            return
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self._finish_path(PathEvaluation(
                False, 0.0, (), self._node.get_clock().now().nanoseconds,
                0, 'ComputePathToPose goal rejected', FailureClass.ACTION_REJECTION,
            ))
            return
        self._path_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._path_result(generation, result),
        )

    def _path_result(self, generation: int, future) -> None:
        if generation != self._active_path_request or self._path_callback is None:
            return
        wrapped = future.result()
        result = wrapped.result
        points = tuple(
            (pose.pose.position.x, pose.pose.position.y)
            for pose in result.path.poses
        ) if result is not None else ()
        path_frame_id = (
            '' if result is None else str(result.path.header.frame_id or '')
        )
        valid = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and result is not None and
            result.error_code == ComputePathToPose.Result.NONE and bool(points) and
            all(
                math.isfinite(float(x)) and math.isfinite(float(y))
                for x, y in points
            )
        )
        measured_length = path_length(points) if valid else 0.0
        if valid and (not math.isfinite(measured_length) or measured_length < 0.0):
            valid = False
        error_code = 0 if result is None else result.error_code
        error_message = 'missing action result' if result is None else result.error_msg
        heading_cost = 0.0
        if valid:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._global_frame, self._base_frame, Time(),
                    timeout=Duration(seconds=0.0),
                )
                orientation = transform.transform.rotation
                robot_yaw = math.atan2(
                    2.0 * (orientation.w * orientation.z +
                            orientation.x * orientation.y),
                    1.0 - 2.0 * (orientation.y * orientation.y +
                                  orientation.z * orientation.z),
                )
                heading_cost = initial_path_heading_cost(points, robot_yaw)
            except TransformException:
                # Heading is preference evidence only.  Path validity and the
                # final dispatch TF gate remain independent safety checks.
                heading_cost = 0.0
        evidence = FailureEvidence()
        if error_code in (
                ComputePathToPose.Result.GOAL_OCCUPIED,
                ComputePathToPose.Result.GOAL_OUTSIDE_MAP,
                ComputePathToPose.Result.NO_VALID_PATH):
            evidence = FailureEvidence(compute_path_error='HARD_UNREACHABLE')
        elif error_code == ComputePathToPose.Result.TF_ERROR:
            evidence = FailureEvidence(tf_unavailable=True)
        elif error_code == ComputePathToPose.Result.TIMEOUT:
            evidence = FailureEvidence(timed_out=True)
        elif not valid and error_code != 0:
            evidence = FailureEvidence(compute_path_error='PLANNER_FAILURE')
        self._finish_path(PathEvaluation(
            valid=valid,
            length_m=measured_length if valid else 0.0,
            samples=downsample_path(points, self._maximum_path_samples),
            query_ros_ns=self._node.get_clock().now().nanoseconds,
            error_code=error_code,
            error_message=error_message,
            failure_class=FailureClass.UNKNOWN if valid else classify_failure(evidence),
            query_started_ros_ns=self._path_started_ros_ns,
            duration_s=max(0.0, time.monotonic() - self._path_started_steady_s),
            caller=self._path_caller,
            task_signature=self._path_task_signature,
            heading_cost=heading_cost,
            path_frame_id=path_frame_id,
            path=(result.path if result is not None and valid else None),
        ))

    def _finish_path(self, result: PathEvaluation) -> None:
        if self._path_started_steady_s:
            result = PathEvaluation(
                **{**result.__dict__,
                   'query_started_ros_ns': self._path_started_ros_ns,
                   'duration_s': (
                       result.duration_s if result.duration_s > 0.0 else
                       max(0.0, time.monotonic() - self._path_started_steady_s)
                   ),
                   'caller': self._path_caller,
                   'task_signature': self._path_task_signature,
                   'map_stamp_ns': self._path_map_stamp_ns,
                   'costmap_stamp_ns': self._path_costmap_stamp_ns},
            )
        self._node.get_logger().info(
            'COMPUTE_PATH_RESULT source=%s task=%s valid=%s error_code=%d '
            'failure_class=%s duration_s=%.3f path_frame=%s samples=%d '
            'path_digest=%s error=%r' % (
                result.caller, result.task_signature, result.valid,
                result.error_code, result.failure_class.value,
                result.duration_s, result.path_frame_id or self._global_frame,
                len(result.samples), path_samples_digest(result.samples),
                result.error_message,
            )
        )
        callback, self._path_callback = self._path_callback, None
        self._path_goal_handle = None
        self._release_path_query_lock()
        if callback is not None:
            callback(result)

    def _acquire_path_query_lock(self) -> bool:
        """Serialize this robot's planner action with the C++ candidate node."""
        if self._path_query_lock_file is not None:
            return True
        try:
            handle = open(self._path_query_lock_path, 'a+')
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, IOError):
            try:
                handle.close()
            except UnboundLocalError:
                pass
            return False
        self._path_query_lock_file = handle
        return True

    def _release_path_query_lock(self) -> None:
        """Release the bounded per-robot planner lease, if held."""
        handle, self._path_query_lock_file = self._path_query_lock_file, None
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def check_dispatch_preconditions(
            self, task: PhysicalTask, final_path_valid: bool,
            callback: Callable[[DispatchPreconditions], None],
            path_samples: tuple[Point, ...] = (),
            path_frame_id: str = '', path: Optional[Path] = None) -> None:
        """Asynchronously confirm local lifecycle plus map, costmap, and TF context."""
        self._ensure_navigate_client()
        self._ensure_lifecycle_clients()
        base = self._basic_preconditions(
            task, final_path_valid, path_samples, path_frame_id,
        )
        if base.reason:
            callback(base)
            return
        self._request_path_valid(
            path,
            lambda valid, indices, reason: self._continue_dispatch_preconditions(
                base, path, valid, indices, reason, callback,
            ),
        )

    def _continue_dispatch_preconditions(
            self, base: DispatchPreconditions, path: Optional[Path],
            path_valid: bool, invalid_indices: tuple[int, ...],
            path_valid_reason: str,
            callback: Callable[[DispatchPreconditions], None]) -> None:
        """Finish global service, local polygon, and lifecycle preconditions."""
        local_footprint = self._local_footprint_path_check(path)
        reason = base.reason
        if not path_valid:
            reason = reason or path_valid_reason or 'Nav2 IsPathValid rejected path'
        if not local_footprint.clear:
            reason = reason or 'local footprint gate: ' + local_footprint.reason
        checked = DispatchPreconditions(
            **{
                **base.__dict__,
                'reason': reason,
                'path_valid_service_name': self._path_valid_service_name,
                'path_valid_checked': True,
                'path_valid': path_valid,
                'path_invalid_pose_indices': invalid_indices,
                'path_valid_reason': path_valid_reason,
                'local_footprint_checked': True,
                'local_footprint_clear': local_footprint.clear,
                'local_footprint_reason': local_footprint.reason,
                'local_footprint_inspected_poses': local_footprint.inspected_poses,
                'local_footprint_outside_poses': local_footprint.outside_poses,
                'local_footprint_maximum_cost': local_footprint.maximum_cost,
                'local_footprint_first_blocked_pose_index': (
                    local_footprint.first_blocked_pose_index),
                'local_footprint_first_blocked_pose': (
                    local_footprint.first_blocked_pose),
                'local_footprint_first_blocked_pose_yaw': (
                    local_footprint.first_blocked_pose_yaw),
                'local_footprint_first_blocked_cell': (
                    local_footprint.first_blocked_cell),
                'local_footprint_first_blocked_cell_world': (
                    local_footprint.first_blocked_cell_world),
                'local_footprint_effective_footprint': (
                    local_footprint.effective_footprint),
                'local_footprint_costmap_frame': local_footprint.costmap_frame,
                'local_footprint_costmap_origin': local_footprint.costmap_origin,
                'local_footprint_costmap_origin_yaw': (
                    local_footprint.costmap_origin_yaw),
                'local_footprint_costmap_width': local_footprint.costmap_width,
                'local_footprint_costmap_height': local_footprint.costmap_height,
                'local_footprint_costmap_resolution': (
                    local_footprint.costmap_resolution),
                'local_footprint_costmap_stamp_ns': (
                    local_footprint.costmap_stamp_ns),
                'local_footprint_path_transform_translation': (
                    local_footprint.path_transform_translation),
                'local_footprint_path_transform_yaw': (
                    local_footprint.path_transform_yaw),
                'local_footprint_path_transform_stamp_ns': (
                    local_footprint.path_transform_stamp_ns),
                'local_footprint_path_transform_age_s': (
                    local_footprint.path_transform_age_s),
            },
        )
        if checked.reason:
            callback(checked)
            return
        unavailable = [name for name, client in self._lifecycle_clients.items()
                       if not client.service_is_ready()]
        if unavailable:
            callback(DispatchPreconditions(
                **{**checked.__dict__, 'lifecycle_active': False,
                   'reason': 'lifecycle services unavailable: ' + ','.join(unavailable)},
            ))
            return
        states = {}

        def completed(name, future):
            try:
                states[name] = future.result().current_state.label
            except Exception as error:  # noqa: B902
                states[name] = 'error:' + str(error)
            if len(states) != len(self._lifecycle_clients):
                return
            active = all(value == 'active' for value in states.values())
            self._last_lifecycle_active = active
            callback(DispatchPreconditions(
                **{**checked.__dict__, 'lifecycle_active': active,
                   'reason': '' if active else 'inactive lifecycle nodes: ' + str(states)},
            ))

        for name, client in self._lifecycle_clients.items():
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda result, key=name: completed(key, result))

    def _basic_preconditions(
            self, task: PhysicalTask, final_path_valid: bool,
            path_samples: tuple[Point, ...] = (),
            path_frame_id: str = '') -> DispatchPreconditions:
        map_value = None if self._map is None else occupancy_value(self._map, task.approach)
        cost_value = (
            None if self._costmap is None else occupancy_value(self._costmap, task.approach)
        )
        inside_map = map_value is not None
        inside_costmap = cost_value is not None
        reason = ''
        transform_available = False
        transform_age_s = None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(), timeout=Duration(seconds=0.1),
            )
            transform_available = True
            stamp = transform.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns > 0:
                transform_age_s = max(
                    0.0, (self._node.get_clock().now().nanoseconds - stamp_ns) / 1e9,
                )
                if transform_age_s > self._maximum_tf_age_s:
                    transform_available = False
                    reason = (
                        f'{self._global_frame} to {self._base_frame} '
                        'transform is stale'
                    )
        except TransformException as error:
            reason = 'required transform unavailable: ' + str(error)
        if not inside_map:
            reason = reason or 'goal lies outside current shared map'
        elif map_value < 0 or map_value >= 50:
            reason = reason or 'goal occupancy-map cell is unknown or occupied'
        if not inside_costmap:
            reason = reason or 'goal lies outside current global costmap'
        elif cost_value < 0 or cost_value >= self._costmap_lethal_threshold:
            reason = reason or 'goal costmap cell is unknown or lethal'
        gate_mode = getattr(
            self, '_local_path_gate_mode', LOCAL_PATH_GATE_MODE_A)
        gate_threshold = local_path_gate_threshold(
            gate_mode, getattr(self, '_costmap_lethal_threshold', 253),
        )
        local_path_evidence = local_path_clearance(
            self._local_costmap, path_samples,
            lambda point: (
                None if self._local_costmap is None else
                (_transform_point(
                    self._tf_buffer, self._local_costmap.header.frame_id,
                    self._global_frame, point) or (None, 0))[0]
            ),
            blocked_threshold=gate_threshold,
        )
        local_path_is_clear = local_path_evidence.clear
        if not local_path_is_clear:
            reason = reason or 'local path gate: ' + local_path_evidence.reason
            if local_path_evidence.reason == 'BLOCKED_LOCAL_CELL':
                self._log_blocked_local_path(
                    task, path_samples, path_frame_id, local_path_evidence,
                    blocked_threshold=gate_threshold,
                )
        if self.local_goal_active:
            reason = reason or 'another local navigation goal is active'
        if not final_path_valid:
            reason = reason or 'final local ComputePathToPose validation failed'
        if not self._navigate_client.server_is_ready():
            reason = reason or 'local NavigateToPose action server unavailable'
        return DispatchPreconditions(
            action_server_ready=self._navigate_client.server_is_ready(),
            lifecycle_active=False,
            transform_available=transform_available,
            transform_age_s=transform_age_s,
            goal_inside_map=inside_map,
            goal_inside_costmap=inside_costmap,
            goal_map_value=map_value,
            goal_costmap_value=cost_value,
            local_path_clear=local_path_is_clear,
            no_local_goal_active=not self.local_goal_active,
            final_path_valid=final_path_valid,
            reason=reason,
            local_path_reason=local_path_evidence.reason,
            local_path_inspected_points=local_path_evidence.inspected_points,
            local_path_outside_points=local_path_evidence.outside_points,
            local_path_gate_mode=gate_mode,
            local_path_gate_threshold=gate_threshold,
            local_path_maximum_cost=local_path_evidence.maximum_cost,
            local_path_first_blocked_point_index=(
                local_path_evidence.first_blocked_point_index),
            local_path_first_blocked_point=(
                local_path_evidence.first_blocked_path_point),
            local_path_first_blocked_local_point=(
                local_path_evidence.first_blocked_local_point),
        )

    def _log_blocked_local_path(
            self, task: PhysicalTask, path_samples: tuple[Point, ...],
            path_frame_id: str, evidence: LocalPathClearance,
            blocked_threshold: int) -> None:
        """Emit bounded point-level evidence for a local-path rejection.

        This is intentionally rejection-only telemetry.  It repeats the
        read-only transform/grid lookup used by ``local_path_clearance`` and
        does not feed any value back into the dispatch decision.
        """
        now_ns = self._node.get_clock().now().nanoseconds
        robot_pose = None
        robot_yaw = None
        robot_tf_stamp_ns = 0
        robot_tf_error = ''
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time(),
                timeout=Duration(seconds=0.0),
            )
            rotation = transform.transform.rotation
            robot_yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y ** 2 + rotation.z ** 2),
            )
            robot_pose = (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
            )
            robot_tf_stamp_ns = _stamp_ns_static(transform.header.stamp)
        except TransformException as error:
            robot_tf_error = str(error)

        local_grid = self._local_costmap
        local_frame = '' if local_grid is None else str(local_grid.header.frame_id)
        local_stamp_ns = self._grid_stamp(local_grid)
        origin = None
        if local_grid is not None:
            origin = {
                'x': float(local_grid.info.origin.position.x),
                'y': float(local_grid.info.origin.position.y),
                'yaw_rad': math.atan2(
                    2.0 * (local_grid.info.origin.orientation.w *
                            local_grid.info.origin.orientation.z),
                    1.0 - 2.0 * (local_grid.info.origin.orientation.y ** 2 +
                                  local_grid.info.origin.orientation.z ** 2),
                ),
            }

        points = []
        for point_index, point in enumerate(path_samples[:12]):
            transformed = None if local_grid is None else _transform_point(
                self._tf_buffer, local_frame, self._global_frame, point,
            )
            local_point = None if transformed is None else transformed[0]
            local_tf_stamp_ns = 0 if transformed is None else int(transformed[1])
            cell = None if local_grid is None or local_point is None else _grid_cell(
                local_grid, local_point,
            )
            value = None if local_grid is None or local_point is None else occupancy_value(
                local_grid, local_point,
            )
            point_record = {
                'index': point_index,
                'path_point': [float(point[0]), float(point[1])],
                'path_frame': path_frame_id or self._global_frame,
                'local_point': (
                    None if local_point is None else
                    [float(local_point[0]), float(local_point[1])]
                ),
                'local_frame': local_frame,
                'local_cell': None if cell is None else [int(cell[0]), int(cell[1])],
                'occupancy_cost': None if value is None else int(value),
                'global_map_value': (
                    None if self._map is None else occupancy_value(
                        self._map, point,
                    )
                ),
                'global_costmap_value': (
                    None if self._costmap is None else occupancy_value(
                        self._costmap, point,
                    )
                ),
                'transform_stamp_ns': local_tf_stamp_ns,
                'transform_age_s': self._age_s(local_tf_stamp_ns, now_ns),
                'distance_from_robot_m': None,
                'distance_from_goal_m': None,
            }
            effective_path_frame = path_frame_id or self._global_frame
            if effective_path_frame == self._global_frame:
                if robot_pose is not None:
                    point_record['distance_from_robot_m'] = math.dist(
                        (float(point[0]), float(point[1])), robot_pose,
                    )
                point_record['distance_from_goal_m'] = math.dist(
                    (float(point[0]), float(point[1])), task.approach,
                )
            points.append(point_record)

        payload = {
            'schema_version': 'dispatch_local_path_gate.v1',
            'sample_ros_ns': int(now_ns),
            'robot': self._robot_id,
            'task_physical_signature': task.physical_signature,
            'task_canonical_id': getattr(task, 'canonical_id', ''),
            'robot_pose': robot_pose,
            'robot_pose_frame': self._global_frame,
            'robot_yaw_rad': robot_yaw,
            'robot_tf_stamp_ns': int(robot_tf_stamp_ns),
            'robot_tf_age_s': self._age_s(robot_tf_stamp_ns, now_ns),
            'robot_tf_error': robot_tf_error,
            'goal_pose': [float(task.approach[0]), float(task.approach[1])],
            'goal_yaw_rad': float(task.approach_yaw),
            'goal_frame': self._global_frame,
            'path_frame': path_frame_id or self._global_frame,
            'path_frame_recorded': bool(path_frame_id),
            'local_costmap_frame': local_frame,
            'local_costmap_stamp_ns': int(local_stamp_ns),
            'local_costmap_age_s': self._age_s(local_stamp_ns, now_ns),
            'local_costmap_resolution_m': (
                None if local_grid is None else float(local_grid.info.resolution)
            ),
            'local_costmap_size': (
                None if local_grid is None else
                [int(local_grid.info.width), int(local_grid.info.height)]
            ),
            'local_costmap_origin': origin,
            'blocked_threshold': int(blocked_threshold),
            'gate_reason': evidence.reason,
            'gate_inspected_points': int(evidence.inspected_points),
            'gate_outside_points': int(evidence.outside_points),
            'path_sample_count': len(path_samples),
            'path_digest': path_samples_digest(path_samples),
            'first_path_points': points,
        }
        self._node.get_logger().warning(
            'DISPATCH_LOCAL_PATH_GATE_DIAGNOSTIC %s' % json.dumps(
                payload, sort_keys=True, separators=(',', ':')),
        )

    def send_navigation(
            self, task: PhysicalTask,
            callback: Callable[[NavigationOutcome], None],
            diagnostic_path: tuple[tuple[float, float], ...] = ()) -> bool:
        """Send one already validated goal to this namespace's navigator only."""
        self._ensure_navigate_client()
        if self.local_goal_active or not self._navigate_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(task)
        self._navigation_callback = callback
        self._navigation_started_steady_s = time.monotonic()
        self._navigation_start_distance_m = self.travelled_distance_m
        self._navigation_last_progress_distance_m = self.travelled_distance_m
        self._navigation_last_progress_steady_s = self._navigation_started_steady_s
        self._navigation_recoveries = 0
        self._navigation_timeout_requested = False
        self._navigation_no_progress_requested = False
        self._navigation_no_progress_requested_ros_ns = 0
        self._navigation_cancel_requested = False
        self._navigation_target = task.approach
        self._navigation_physical_signature = task.physical_signature
        self._navigation_diagnostic_path = tuple(diagnostic_path)
        self._navigation_goal_number += 1
        self._navigation_point_validation = {
            'goal_number': self._navigation_goal_number,
            'physical_task_signature': task.physical_signature,
            'target': [float(task.approach[0]), float(task.approach[1])],
            'path_length_m': path_length(tuple(diagnostic_path)),
            'dispatch': self._point_validation_sample(),
            'transitions': [],
        }
        self._navigation_point_validation['_last_sample'] = (
            self._navigation_point_validation['dispatch'])
        self._navigation_point_last_sample_steady_s = time.monotonic()
        self._emit_point_validation('DISPATCH')
        self._navigation_send_pending = True
        future = self._navigate_client.send_goal_async(
            goal, feedback_callback=self._navigation_feedback,
        )
        future.add_done_callback(self._navigation_goal_response)
        return True

    def _navigation_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: B902
            self._navigation_send_pending = False
            self._finish_navigation(NavigationOutcome(
                False, GoalStatus.STATUS_UNKNOWN, 0,
                'NavigateToPose goal response exception: ' + str(error),
                FailureClass.UNKNOWN,
                time.monotonic() - self._navigation_started_steady_s,
                self.travelled_distance_m - self._navigation_start_distance_m,
                self._navigation_recoveries,
            ))
            return
        self._navigation_send_pending = False
        if goal_handle is None or not goal_handle.accepted:
            self._finish_navigation(NavigationOutcome(
                False, GoalStatus.STATUS_UNKNOWN, 0, 'NavigateToPose goal rejected',
                FailureClass.ACTION_REJECTION,
                time.monotonic() - self._navigation_started_steady_s,
                self.travelled_distance_m - self._navigation_start_distance_m,
                self._navigation_recoveries,
            ))
            return
        self._navigation_goal_handle = goal_handle
        if self._navigation_cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result)

    def _navigation_feedback(self, feedback) -> None:
        self._navigation_recoveries = max(
            self._navigation_recoveries,
            int(feedback.feedback.number_of_recoveries),
        )

    def cancel_navigation(self) -> bool:
        """Request explicit cancellation of this namespace's active goal."""
        if not self.local_goal_active:
            return False
        self._navigation_cancel_requested = True
        if self._navigation_goal_handle is not None:
            self._navigation_goal_handle.cancel_goal_async()
        return True

    def cancel_pending_preflight(self) -> None:
        """Invalidate a pending planner/path-validity preflight."""
        self._active_path_request += 1
        path_goal_handle, self._path_goal_handle = self._path_goal_handle, None
        self._path_callback = None
        self._path_valid_pending = None
        self._path_deadline_steady_s = 0.0
        self._release_path_query_lock()
        if path_goal_handle is not None:
            path_goal_handle.cancel_goal_async()

    def _navigation_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as error:  # noqa: B902
            self._finish_navigation(NavigationOutcome(
                True, GoalStatus.STATUS_UNKNOWN, 0,
                'NavigateToPose result exception: ' + str(error),
                FailureClass.UNKNOWN,
                time.monotonic() - self._navigation_started_steady_s,
                max(0.0, self.travelled_distance_m - self._navigation_start_distance_m),
                self._navigation_recoveries,
            ))
            return
        result = wrapped.result
        error_code = 0 if result is None else result.error_code
        error_message = 'missing action result' if result is None else result.error_msg
        succeeded = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and
            result is not None and error_code == NavigateToPose.Result.NONE
        )
        evidence = FailureEvidence()
        if self._navigation_timeout_requested:
            evidence = FailureEvidence(timed_out=True)
        elif self._navigation_no_progress_requested:
            evidence = FailureEvidence(controller_no_progress=True)
        elif self._navigation_cancel_requested or wrapped.status == GoalStatus.STATUS_CANCELED:
            evidence = FailureEvidence(explicitly_cancelled=True)
        elif classify_follow_path_controller_error(error_code) == FailureClass.TF_OR_LIFECYCLE:
            evidence = FailureEvidence(tf_unavailable=True)
        elif classify_follow_path_controller_error(error_code) == FailureClass.CONTROLLER_NO_PROGRESS:
            evidence = FailureEvidence(controller_no_progress=True)
        error_name = follow_path_controller_error_name(error_code)
        if self._navigation_no_progress_requested and not error_message:
            error_message = 'local controller no-progress timeout'
        failure_class = FailureClass.UNKNOWN if succeeded else classify_failure(evidence)
        deepest_classification = ''
        controller_failure_family = ''
        deepest_stamp = 0
        if not succeeded:
            deepest_stamp = self._navigation_no_progress_requested_ros_ns or (
                self._node.get_clock().now().nanoseconds)
            if self._navigation_no_progress_requested:
                deepest_classification = 'CONTROLLER_EXECUTION_NO_PROGRESS'
                controller_failure_family = 'CONTROLLER_EXECUTION_NO_PROGRESS'
            elif error_code in FOLLOW_PATH_TF_FAILURE_CODES:
                deepest_classification = error_name or 'TF_ERROR'
                controller_failure_family = 'NAV_INFRASTRUCTURE_TF'
            elif error_code in FOLLOW_PATH_CONTROLLER_FAILURE_CODES:
                deepest_classification = error_name or 'FOLLOW_PATH_CONTROLLER_FAILURE'
                controller_failure_family = 'CONTROLLER_EXECUTION'
            else:
                deepest_classification = 'NAV2_ABORT_CAUSE_UNAVAILABLE'
        snapshot_json = ''
        if not succeeded:
            snapshot_json = self._failure_snapshot(error_code, error_message, failure_class)
        self._finish_navigation(NavigationOutcome(
            True, wrapped.status, error_code, error_message,
            failure_class,
            time.monotonic() - self._navigation_started_steady_s,
            max(0.0, self.travelled_distance_m - self._navigation_start_distance_m),
            self._navigation_recoveries,
            error_name,
            error_code if error_code in (
                FOLLOW_PATH_TF_FAILURE_CODES | FOLLOW_PATH_CONTROLLER_FAILURE_CODES
            ) else 0,
            error_name if error_code in (
                FOLLOW_PATH_TF_FAILURE_CODES | FOLLOW_PATH_CONTROLLER_FAILURE_CODES
            ) else '',
            controller_failure_family,
            deepest_classification,
            deepest_stamp,
            snapshot_json,
        ))

    def _finish_navigation(self, result: NavigationOutcome) -> None:
        self._record_point_validation_sample('FAILURE_OR_SUCCESS', force=True)
        self._emit_point_validation('FINAL', result)
        callback, self._navigation_callback = self._navigation_callback, None
        self._navigation_goal_handle = None
        self._navigation_send_pending = False
        self._navigation_target = None
        self._navigation_physical_signature = ''
        self._navigation_diagnostic_path = ()
        self._navigation_point_validation = None
        self._navigation_point_last_sample_steady_s = 0.0
        if callback is not None:
            callback(result)

    def _check_timeouts(self) -> None:
        now = time.monotonic()
        pending_path_valid = self._path_valid_pending
        if pending_path_valid is not None and now > pending_path_valid[1]:
            self._path_valid_pending = None
            pending_path_valid[2](False, (), 'PATH_VALID_SERVICE_TIMEOUT')
        if (self._navigation_point_validation is not None and
                now - self._navigation_point_last_sample_steady_s >=
                self._navigation_point_sample_period_s):
            self._record_point_validation_sample('POINT_STATUS_CHANGE')
            self._navigation_point_last_sample_steady_s = now
        if self._path_callback is not None and now > self._path_deadline_steady_s:
            if self._path_goal_handle is not None:
                self._path_goal_handle.cancel_goal_async()
            self._active_path_request += 1
            self._finish_path(PathEvaluation(
                False, 0.0, (), self._node.get_clock().now().nanoseconds,
                ComputePathToPose.Result.TIMEOUT, 'local path query timeout',
                FailureClass.TIMEOUT,
            ))
        if (self._navigation_goal_handle is not None and
                not self._navigation_timeout_requested and
                now - self._navigation_started_steady_s > self._navigation_timeout_s):
            self._navigation_timeout_requested = True
            self._navigation_goal_handle.cancel_goal_async()
        if (self._navigation_goal_handle is not None and
                not self._navigation_no_progress_requested and
                now - self._navigation_last_progress_steady_s >
                self._navigation_no_progress_timeout_s):
            self._navigation_no_progress_requested = True
            self._navigation_no_progress_requested_ros_ns = (
                self._node.get_clock().now().nanoseconds
            )
            self._navigation_goal_handle.cancel_goal_async()
