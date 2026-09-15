import math
import signal
import threading
import time

import numpy as np
from my_epuck_interfaces.msg import PeerMap, RelativePoseHypothesis
from my_epuck_project.live_map_sanitizer import (
    apply_incremental_patch,
    footprint_cell_indices,
    swept_footprint_cell_indices,
)
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Bool
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


# SLAM map origins/dimensions change as exploration expands.  Retaining the
# vectorized world-coordinate arrays for every historical geometry grows
# without bound (each entry is two large float arrays).  A small bounded cache
# still avoids duplicate work within the current pair of source maps without
# retaining old map revisions.
_MAX_GEOMETRY_CACHE_ENTRIES = 4


def _quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _grid_geometry_key(message):
    origin = message.info.origin
    orientation = origin.orientation
    return (
        message.header.frame_id,
        int(message.info.width), int(message.info.height),
        float(message.info.resolution),
        float(getattr(origin.position, 'x', 0.0)),
        float(getattr(origin.position, 'y', 0.0)),
        float(getattr(origin.position, 'z', 0.0)),
        float(getattr(orientation, 'x', 0.0)),
        float(getattr(orientation, 'y', 0.0)),
        float(getattr(orientation, 'z', 0.0)),
        float(getattr(orientation, 'w', 1.0)),
    )


def _grid_array(message):
    """View OccupancyGrid data without making Python list copies."""
    values = np.asarray(message.data, dtype=np.int8)
    return values.reshape((int(message.info.height), int(message.info.width)))


def _same_grid_content(previous, current):
    if previous is None or _grid_geometry_key(previous) != _grid_geometry_key(current):
        return False
    previous_data = np.asarray(previous.data, dtype=np.int8)
    current_data = np.asarray(current.data, dtype=np.int8)
    return (previous_data.size == current_data.size
            and np.array_equal(previous_data, current_data))


def _snapshot_pose_age_s(snapshot_time, pose_stamp):
    """Return temporal mismatch between a map snapshot and a TF pose.

    The pose is intentionally looked up at the map snapshot timestamp.  The
    existing freshness tolerance therefore measures correspondence to that
    snapshot, not age relative to the node's current clock.
    """
    if snapshot_time is None or pose_stamp is None:
        return math.inf
    return abs(snapshot_time.nanoseconds - pose_stamp.nanoseconds) / 1e9


def _changed_update_bounds(current, previous):
    """Return the smallest ``x, y, width, height`` changed rectangle."""
    current = np.asarray(current, dtype=np.int8)
    previous = np.asarray(previous, dtype=np.int8)
    if current.shape != previous.shape:
        return None
    changed = np.flatnonzero(current != previous)
    if changed.size == 0:
        return None
    rows, columns = np.unravel_index(changed, current.shape)
    minimum_column, maximum_column = int(columns.min()), int(columns.max())
    minimum_row, maximum_row = int(rows.min()), int(rows.max())
    return (
        minimum_column,
        minimum_row,
        maximum_column - minimum_column + 1,
        maximum_row - minimum_row + 1,
    )


def _scalar_fused_data(messages, transforms, minimum_x, minimum_y, width,
                       height, resolution):
    """Retain the scalar reference implementation for equivalence tests."""
    fused_data = [-1] * (width * height)
    for message, transform in zip(messages, transforms):
        source_resolution = message.info.resolution
        origin_yaw = _quaternion_yaw(message.info.origin.orientation)
        origin_cos = math.cos(origin_yaw)
        origin_sin = math.sin(origin_yaw)
        transform_x, transform_y, transform_yaw = transform
        transform_cos = math.cos(transform_yaw)
        transform_sin = math.sin(transform_yaw)
        for index, value in enumerate(message.data):
            if value < 0:
                continue
            column = index % message.info.width
            row = index // message.info.width
            local_x = (column + 0.5) * source_resolution
            local_y = (row + 0.5) * source_resolution
            map_x = (message.info.origin.position.x
                     + origin_cos * local_x - origin_sin * local_y)
            map_y = (message.info.origin.position.y
                     + origin_sin * local_x + origin_cos * local_y)
            output_x = (transform_x + transform_cos * map_x
                        - transform_sin * map_y)
            output_y = (transform_y + transform_sin * map_x
                        + transform_cos * map_y)
            output_column = int(math.floor(
                (output_x - minimum_x) / resolution))
            output_row = int(math.floor(
                (output_y - minimum_y) / resolution))
            if 0 <= output_column < width and 0 <= output_row < height:
                target = output_row * width + output_column
                fused_data[target] = max(fused_data[target], int(value))
    return np.asarray(fused_data, dtype=np.int8)


def _vectorized_fused_data(messages, transforms, minimum_x, minimum_y, width,
                           height, resolution, geometry_cache=None):
    """Vectorized equivalent of the scalar source-aware occupancy merge."""
    fused_data = np.full(width * height, -1, dtype=np.int8)
    geometry_cache = geometry_cache if geometry_cache is not None else {}
    for message, transform in zip(messages, transforms):
        geometry_key = _grid_geometry_key(message)
        cached = geometry_cache.get(geometry_key)
        if cached is None:
            rows, columns = np.indices(
                (int(message.info.height), int(message.info.width)),
                dtype=np.float64,
            )
            local_x = (columns + 0.5) * float(message.info.resolution)
            local_y = (rows + 0.5) * float(message.info.resolution)
            origin_yaw = _quaternion_yaw(message.info.origin.orientation)
            origin_cos = math.cos(origin_yaw)
            origin_sin = math.sin(origin_yaw)
            map_x = (float(message.info.origin.position.x)
                     + origin_cos * local_x - origin_sin * local_y)
            map_y = (float(message.info.origin.position.y)
                     + origin_sin * local_x + origin_cos * local_y)
            cached = (map_x, map_y)
            geometry_cache[geometry_key] = cached
            while len(geometry_cache) > _MAX_GEOMETRY_CACHE_ENTRIES:
                geometry_cache.pop(next(iter(geometry_cache)))
        map_x, map_y = cached
        values = _grid_array(message)
        known = values >= 0
        if not np.any(known):
            continue
        transform_x, transform_y, transform_yaw = transform
        transform_cos = math.cos(transform_yaw)
        transform_sin = math.sin(transform_yaw)
        output_x = (transform_x + transform_cos * map_x[known]
                    - transform_sin * map_y[known])
        output_y = (transform_y + transform_sin * map_x[known]
                    + transform_cos * map_y[known])
        output_column = np.floor(
            (output_x - minimum_x) / resolution).astype(np.int64)
        output_row = np.floor(
            (output_y - minimum_y) / resolution).astype(np.int64)
        valid = (
            (output_column >= 0) & (output_column < width)
            & (output_row >= 0) & (output_row < height)
        )
        if np.any(valid):
            targets = output_row[valid] * width + output_column[valid]
            np.maximum.at(fused_data, targets, values[known][valid])
    return fused_data


def message_key(messages, local_revision, remote_revision):
    return tuple(
        (
            message.header.frame_id,
            message.header.stamp.sec,
            message.header.stamp.nanosec,
            message.info.width,
            message.info.height,
            message.info.resolution,
        )
        for message in messages
    ) + (local_revision, remote_revision)


class SourceAwareMapFusion(Node):
    """Fuse own local SLAM evidence with one validated remote peer map."""

    def __init__(self):
        super().__init__('map_fusion')
        self.declare_parameter('local_map_topic', 'map')
        self.declare_parameter('remote_peer_topic', '/cslam/remote/local_map')
        self.declare_parameter('expected_remote_source', '')
        self.declare_parameter('output_topic', 'shared_map')
        self.declare_parameter('metadata_topic', 'shared_map_metadata')
        self.declare_parameter('visualization_topic', 'shared_map_visualization')
        self.declare_parameter(
            'visualization_updates_topic', 'shared_map_visualization_updates')
        self.declare_parameter('output_frame', 'shared_map')
        self.declare_parameter('resolution', 0.01)
        self.declare_parameter(
            'live_robot_frames',
            ['robot1/base_footprint', 'robot2/base_footprint'])
        self.declare_parameter('live_footprint_radius_m', 0.037)
        self.declare_parameter('live_footprint_uncertainty_cells', 1)
        self.declare_parameter('live_pose_max_age_s', 0.5)
        self.declare_parameter('publish_on_callback', False)
        self.declare_parameter('sanitize_live_footprints', False)
        self.declare_parameter('min_fusion_rebuild_period_s', 1.0)
        self.declare_parameter('source_freshness_max_age_s', 3.0)
        self.declare_parameter('handoff_gated', False)
        self.declare_parameter('historical_cleanup_required', False)
        self.declare_parameter('local_prehandoff_path_topic', '')
        self.declare_parameter('remote_prehandoff_path_topic', '')
        self.declare_parameter('historical_cleanup_ready_topic', '')
        self.declare_parameter('historical_footprint_radius_m', 0.037)

        local_topic = self.get_parameter('local_map_topic').value
        remote_topic = self.get_parameter('remote_peer_topic').value
        self.expected_source = self.get_parameter('expected_remote_source').value
        output_topic = self.get_parameter('output_topic').value
        metadata_topic = self.get_parameter('metadata_topic').value
        visualization_topic = self.get_parameter('visualization_topic').value
        visualization_updates_topic = self.get_parameter(
            'visualization_updates_topic').value
        self.output_frame = self.get_parameter('output_frame').value
        self.resolution = float(self.get_parameter('resolution').value)
        self.live_robot_frames = list(
            self.get_parameter('live_robot_frames').value)
        namespace = self.get_namespace().strip('/')
        self.own_robot_id = namespace or self.live_robot_frames[0].split('/')[0]
        self.peer_robot_id = next(
            (frame.split('/')[0] for frame in self.live_robot_frames
             if frame.split('/')[0] != self.own_robot_id),
            None)
        self.live_footprint_radius = float(
            self.get_parameter('live_footprint_radius_m').value)
        self.live_footprint_uncertainty_cells = int(
            self.get_parameter('live_footprint_uncertainty_cells').value)
        self.live_pose_max_age_s = float(
            self.get_parameter('live_pose_max_age_s').value)
        self.publish_on_callback = bool(
            self.get_parameter('publish_on_callback').value)
        self.sanitize_live_footprints = bool(
            self.get_parameter('sanitize_live_footprints').value)
        self.handoff_gated = bool(
            self.get_parameter('handoff_gated').value)
        self.historical_cleanup_required = bool(
            self.get_parameter('historical_cleanup_required').value)
        self.local_path_topic = str(
            self.get_parameter('local_prehandoff_path_topic').value)
        self.remote_path_topic = str(
            self.get_parameter('remote_prehandoff_path_topic').value)
        self.cleanup_ready_topic = str(
            self.get_parameter('historical_cleanup_ready_topic').value)
        if not self.cleanup_ready_topic:
            self.cleanup_ready_topic = (
                f'/cslam/unknown_pose/{self.own_robot_id}/'
                'historical_cleanup_ready')
        self.historical_radius = float(
            self.get_parameter('historical_footprint_radius_m').value)
        self.phase_active = not self.handoff_gated
        self.min_fusion_rebuild_period_s = max(
            0.0, float(self.get_parameter('min_fusion_rebuild_period_s').value))
        self.source_freshness_max_age_s = max(
            3.0, float(self.get_parameter('source_freshness_max_age_s').value),
            2.0 * max(0.1, self.min_fusion_rebuild_period_s or 1.0))
        if not self.expected_source:
            raise ValueError('expected_remote_source must not be empty')
        if self.resolution <= 0.0:
            raise ValueError('resolution must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, output_topic, qos)
        self.metadata_publisher = self.create_publisher(MapMetaData, metadata_topic, qos)
        self.visualization_map_publisher = self.create_publisher(
            OccupancyGrid, visualization_topic, qos)
        self.visualization_update_publisher = self.create_publisher(
            OccupancyGridUpdate, visualization_updates_topic,
            QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ))
        self.local_subscription = None
        self.peer_subscription = None
        self.local_path_subscription = None
        self.remote_path_subscription = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = (
            None if self.handoff_gated else TransformListener(self.tf_buffer, self))
        self.local_map = None
        self.remote_map = None
        self.local_revision = 0
        self.last_remote_revision = 0
        self.remote_content_revision = 0
        self.map_revision = 0
        self.live_pose_cache = {}
        self.base_grid = None
        self.base_key = None
        self.output_grid = None
        self.last_footprint_cells = set()
        self.last_pose_key = None
        self.last_full_rebuild_wall = 0.0
        self.last_output_publish_ros_s = 0.0
        self.last_freshness_source_age_s = None
        self.last_freshness_previous_publication_age_s = None
        self.last_freshness_output_stamp_s = None
        self.geometry_cache = {}
        self.map_dirty = False
        self.dirty_event_count = 0
        self.coalesced_event_count = 0
        self.rebuild_skipped_count = 0
        self.rebuild_busy = False
        self.visualization_data = None
        self.visualization_geometry = None
        self.historical_paths = {'local': None, 'remote': None}
        self.historical_mask_cells = set()
        self.historical_cleanup_ready = not self.historical_cleanup_required
        self.historical_mask_built = not self.historical_cleanup_required
        self.historical_cells_cleared = 0
        self.cleanup_ready_publisher = self.create_publisher(
            Bool, self.cleanup_ready_topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.profile_window = {
            'invocations': 0, 'full_rebuilds': 0, 'pose_updates': 0,
            'cells_inspected': 0, 'cells_copied': 0, 'cells_modified': 0,
            'publications': 0, 'visualization_bases': 0,
            'visualization_updates': 0, 'dirty_events': 0,
            'coalesced_events': 0, 'rebuild_skipped': 0,
            'wall_s': 0.0, 'cpu_s': 0.0,
            'last_log_wall': time.monotonic(),
        }
        self.rebuild_period_s = max(
            0.1, self.min_fusion_rebuild_period_s or 1.0)
        self.retry_timer = None
        self.handoff_subscription = None
        if self.handoff_gated:
            self.handoff_subscription = self.create_subscription(
                RelativePoseHypothesis,
                '/cslam/relative_pose/hypotheses',
                self._handoff_callback,
                QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                ),
            )
            self.get_logger().info(
                'FUSION_PHASE pre_handoff=true map_inputs=false timer=false')
        else:
            self._activate_fusion_phase(local_topic, remote_topic, qos)
        self.get_logger().info(
            f'Local {self.resolve_topic_name(local_topic)} + remote-only '
            f'{self.resolve_topic_name(remote_topic)} from {self.expected_source} '
            f'-> {self.resolve_topic_name(output_topic)}; '
            f'visualization={self.resolve_topic_name(visualization_topic)} '
            f'period_s={self.rebuild_period_s:.3f} '
            f'vectorized=True'
        )

    def _activate_fusion_phase(self, local_topic, remote_topic, qos):
        """Start map subscriptions and the rebuild scheduler after handoff."""
        if self.phase_active and self.retry_timer is not None:
            return
        self.phase_active = True
        if self.tf_listener is None:
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.local_subscription = self.create_subscription(
            OccupancyGrid, local_topic, self.local_callback, qos)
        self.peer_subscription = self.create_subscription(
            PeerMap, remote_topic, self.peer_callback, qos)
        if self.historical_cleanup_required:
            self.local_path_subscription = self.create_subscription(
                Path, self.local_path_topic,
                lambda message: self._path_callback('local', message), qos)
            self.remote_path_subscription = self.create_subscription(
                Path, self.remote_path_topic,
                lambda message: self._path_callback('remote', message), qos)
        self.retry_timer = self.create_timer(
            self.rebuild_period_s, self.scheduled_fuse)
        self.get_logger().info(
            'FUSION_PHASE post_handoff=true map_inputs=true timer=true')

    def _path_callback(self, role, message):
        """Cache one immutable pre-handoff path and trigger first fusion."""
        frame = str(message.header.frame_id)
        if not frame:
            # An empty path is still a valid answer for a robot that had no
            # usable pre-handoff map pose; retain it so the barrier cannot
            # wait forever for a second publication.
            frame = 'unknown'
        self.historical_paths[role] = message
        self.get_logger().info(
            f'HISTORICAL_PATH_RECEIVED role={role} frame={frame} '
            f'samples={len(message.poses)}')
        self.mark_dirty()

    def _historical_points_in_output(self):
        if not self.historical_cleanup_required:
            return []
        if any(path is None for path in self.historical_paths.values()):
            return None
        points = []
        for path in self.historical_paths.values():
            if not path.poses:
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.output_frame, path.header.frame_id, Time(),
                    timeout=Duration(seconds=0.05))
            except TransformException as error:
                self.get_logger().warning(
                    f'Waiting for historical path transform: {error}',
                    throttle_duration_sec=5.0)
                return None
            transform_2d = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                self.yaw(transform.transform.rotation),
            )
            points.extend(self.transform_point(
                pose.pose.position.x, pose.pose.position.y, transform_2d)
                for pose in path.poses)
        return points

    def _apply_historical_mask(self, grid):
        points = self._historical_points_in_output()
        if points is None:
            return None
        first_build = not self.historical_mask_built
        cells = swept_footprint_cell_indices(
            grid, points, self.historical_radius)
        cleared = 0
        for index in cells:
            if grid.data[index] >= 0:
                grid.data[index] = 0
                cleared += 1
        self.historical_mask_cells = cells
        self.historical_mask_built = True
        self.historical_cells_cleared = cleared
        if first_build:
            self.get_logger().info(
                f'HISTORICAL_CLEANUP_READY mask_cells={len(cells)} '
                f'occupied_cells_cleared={cleared} '
                f'physical_radius_m={self.historical_radius:.3f}')
        return cells

    def _publish_cleanup_ready(self):
        if self.historical_cleanup_ready:
            return
        self.historical_cleanup_ready = True
        message = Bool()
        message.data = True
        self.cleanup_ready_publisher.publish(message)
        self.get_logger().info(
            f'HISTORICAL_CLEANUP_READY published=true topic={self.cleanup_ready_topic}')

    def _handoff_callback(self, message):
        if not bool(message.accepted) or str(message.status) != 'ACCEPTED':
            return
        self._activate_fusion_phase(
            self.get_parameter('local_map_topic').value,
            self.get_parameter('remote_peer_topic').value,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def local_callback(self, message):
        changed = not _same_grid_content(self.local_map, message)
        self.local_map = message
        if changed:
            self.local_revision += 1
            self.mark_dirty()

    def reject(self, reason):
        self.get_logger().warning(reason)

    def peer_callback(self, message):
        if message.source_robot_id != self.expected_source:
            self.reject(
                f'Rejected peer source {message.source_robot_id!r}; '
                f'expected {self.expected_source!r}'
            )
            return
        if not message.local_evidence_only:
            self.reject(
                f'Rejected revision {message.revision}: remote map is not '
                'marked local_evidence_only'
            )
            return
        if message.revision <= self.last_remote_revision:
            self.reject(
                f'Ignored stale/duplicate revision {message.revision}; '
                f'last accepted is {self.last_remote_revision}'
            )
            return
        changed = not _same_grid_content(
            self.remote_map, message.occupancy_grid)
        self.last_remote_revision = message.revision
        self.remote_map = message.occupancy_grid
        if changed:
            self.remote_content_revision += 1
            self.mark_dirty()

    def mark_dirty(self):
        self.dirty_event_count += 1
        self.profile_window['dirty_events'] += 1
        if self.map_dirty:
            self.coalesced_event_count += 1
            self.profile_window['coalesced_events'] += 1
        self.map_dirty = True

    @staticmethod
    def yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    @staticmethod
    def transform_point(x, y, transform):
        tx, ty, heading = transform
        cosine, sine = math.cos(heading), math.sin(heading)
        return tx + cosine*x - sine*y, ty + sine*x + cosine*y

    @staticmethod
    def _message_time(message):
        """Return the ROS time carried by an occupancy-map sample."""
        return Time.from_msg(message.header.stamp)

    @classmethod
    def _common_snapshot_time(cls, messages):
        """Use one deterministic timestamp for a local/peer map pair.

        Both fusion peers receive the same two source maps, but callbacks are
        scheduled independently.  Looking up the latest TF at callback time
        therefore made live-footprint sanitization differ between peers.  The
        newest input-map stamp is a shared, message-derived snapshot boundary.
        """
        return max(
            (cls._message_time(message) for message in messages),
            key=lambda value: (value.nanoseconds,))

    def map_transform(self, message, snapshot_time=None):
        if snapshot_time is None:
            snapshot_time = self._message_time(message)
        transform = self.tf_buffer.lookup_transform(
            self.output_frame, message.header.frame_id, snapshot_time,
            timeout=Duration(seconds=0.05)
        )
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
            self.yaw(transform.transform.rotation),
        )

    def point_in_output(self, message, transform, x, y):
        heading = self.yaw(message.info.origin.orientation)
        cosine, sine = math.cos(heading), math.sin(heading)
        map_x = message.info.origin.position.x + cosine*x - sine*y
        map_y = message.info.origin.position.y + sine*x + cosine*y
        return self.transform_point(map_x, map_y, transform)

    def corners(self, message, transform):
        width = message.info.width * message.info.resolution
        height = message.info.height * message.info.resolution
        return [
            self.point_in_output(message, transform, x, y)
            for x, y in ((0.0, 0.0), (width, 0.0), (0.0, height), (width, height))
        ]

    def live_footprints(self, snapshot_time=None):
        """Return independent own/peer footprints and freshness telemetry.

        Own clearing is deliberately independent of peer TF availability. A
        stale peer suppresses only that peer's footprint; it must never make
        the local robot's own live footprint stale as a side effect.
        """
        footprints = []
        key = []
        for frame in self.live_robot_frames:
            role = 'own' if frame.split('/')[0] == self.own_robot_id else 'peer'
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.output_frame, frame,
                    snapshot_time if snapshot_time is not None else Time(),
                    timeout=Duration(seconds=0.05))
                stamp = Time.from_msg(transform.header.stamp)
                reference_time = (
                    snapshot_time if snapshot_time is not None
                    else self.get_clock().now())
                age = _snapshot_pose_age_s(reference_time, stamp)
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                heading = self.yaw(transform.transform.rotation)
                footprints.append({
                    'robot_frame': frame, 'x': x, 'y': y,
                    'radius_m': self.live_footprint_radius,
                    'role': role,
                    'pose_age_s': age,
                })
                self.live_pose_cache[frame] = (x, y, stamp)
                key.append((frame, True, round(x, 3), round(y, 3),
                            round(heading, 3), age <= self.live_pose_max_age_s))
            except TransformException as error:
                cached = self.live_pose_cache.get(frame)
                cached_age = None
                if cached is not None:
                    reference_time = (
                        snapshot_time if snapshot_time is not None
                        else self.get_clock().now())
                    cached_age = _snapshot_pose_age_s(
                        reference_time, cached[2])
                if cached is not None and cached_age <= self.live_pose_max_age_s:
                    footprints.append({
                        'robot_frame': frame, 'x': cached[0], 'y': cached[1],
                        'radius_m': self.live_footprint_radius,
                        'role': role,
                        'pose_age_s': cached_age,
                    })
                    key.append((frame, True, round(cached[0], 3),
                                round(cached[1], 3), round(cached_age, 2),
                                True))
                else:
                    self.get_logger().warning(
                        f'Live footprint unavailable for {frame}: {error}',
                        throttle_duration_sec=5.0)
                    key.append((frame, False))
        telemetry = {}
        for role in ('own', 'peer'):
            matches = [item for item in footprints if item['role'] == role]
            if matches:
                telemetry[f'{role}_pose_age_s'] = min(
                    item['pose_age_s'] for item in matches)
                telemetry[f'{role}_pose_available'] = True
                telemetry[f'{role}_clear_skipped_reason'] = (
                    'POSE_STALE' if telemetry[f'{role}_pose_age_s']
                    > self.live_pose_max_age_s else 'NONE')
            else:
                telemetry[f'{role}_pose_age_s'] = None
                telemetry[f'{role}_pose_available'] = False
                telemetry[f'{role}_clear_skipped_reason'] = 'TF_UNAVAILABLE'
        return footprints, tuple(key), telemetry

    def _make_base_grid(self, messages, transforms, minimum_x, minimum_y,
                        width, height):
        """Build the unsanitized fused base once per map revision."""
        fused_data = _vectorized_fused_data(
            messages, transforms, minimum_x, minimum_y, width, height,
            self.resolution, self.geometry_cache)
        inspected = sum(len(message.data) for message in messages)
        stamp = max(
            (message.header.stamp for message in messages),
            key=lambda value: (value.sec, value.nanosec),
        )
        fused = OccupancyGrid()
        fused.header.stamp = stamp
        fused.header.frame_id = self.output_frame
        fused.info.map_load_time = stamp
        fused.info.resolution = self.resolution
        fused.info.width = width
        fused.info.height = height
        fused.info.origin.position.x = minimum_x
        fused.info.origin.position.y = minimum_y
        fused.info.origin.orientation.w = 1.0
        fused.data = fused_data.tolist()
        return fused, inspected

    def _visualization_geometry_key(self, grid):
        return (
            grid.header.frame_id,
            int(grid.info.width), int(grid.info.height),
            float(grid.info.resolution),
            float(grid.info.origin.position.x),
            float(grid.info.origin.position.y),
            float(grid.info.origin.position.z),
            float(grid.info.origin.orientation.x),
            float(grid.info.origin.orientation.y),
            float(grid.info.origin.orientation.z),
            float(grid.info.origin.orientation.w),
        )

    def _publish_visualization(self, grid):
        """Keep RViz incremental without changing the canonical map topic."""
        geometry = self._visualization_geometry_key(grid)
        current = np.asarray(grid.data, dtype=np.int8).reshape(
            (int(grid.info.height), int(grid.info.width)))
        if (self.visualization_data is None
                or self.visualization_geometry != geometry):
            self.visualization_map_publisher.publish(grid)
            self.visualization_data = current.copy()
            self.visualization_geometry = geometry
            self.profile_window['visualization_bases'] += 1
            return
        bounds = _changed_update_bounds(current, self.visualization_data)
        if bounds is None:
            return
        minimum_column, minimum_row, update_width, update_height = bounds
        update = OccupancyGridUpdate()
        update.header = grid.header
        update.x = minimum_column
        update.y = minimum_row
        update.width = update_width
        update.height = update_height
        update.data = current[
            minimum_row:minimum_row + update_height,
            minimum_column:minimum_column + update_width,
        ].reshape(-1).tolist()
        self.visualization_update_publisher.publish(update)
        self.visualization_data = current.copy()
        self.profile_window['visualization_updates'] += 1

    def _publish_fused(self, grid):
        if not rclpy.ok():
            return
        self.map_publisher.publish(grid)
        self.metadata_publisher.publish(grid.info)
        self._publish_visualization(grid)
        self.last_output_publish_ros_s = (
            self.get_clock().now().nanoseconds / 1e9)

    def _maybe_republish_freshness(self, messages, started_wall, started_cpu):
        """Republish unchanged content while the local source is live.

        The local map callback receives timestamp-advancing samples even when
        occupancy bytes are unchanged.  Those samples are sufficient to prove
        that this fusion input is still live; the accepted peer grid remains
        the same content revision until a peer content/revision change arrives.
        """
        if self.output_grid is None or self.local_map is None:
            return False
        now = self.get_clock().now()
        source_stamp = self._message_time(self.local_map)
        source_age_s = max(
            0.0, (now.nanoseconds - source_stamp.nanoseconds) / 1e9)
        if source_age_s > self.source_freshness_max_age_s:
            return False
        now_s = now.nanoseconds / 1e9
        previous_age_s = now_s - self.last_output_publish_ros_s
        if previous_age_s < self.rebuild_period_s:
            return False
        snapshot_time = self._common_snapshot_time(messages)
        self.output_grid.header.stamp = snapshot_time.to_msg()
        self._publish_fused(self.output_grid)
        self.last_freshness_source_age_s = source_age_s
        self.last_freshness_previous_publication_age_s = previous_age_s
        self.last_freshness_output_stamp_s = snapshot_time.nanoseconds / 1e9
        self._profile(
            mode='FRESHNESS_REPUBLISH', map_changed=False,
            dimensions=(int(self.output_grid.info.width),
                        int(self.output_grid.info.height)),
            cells_inspected=0, cells_copied=0, cells_modified=0,
            published=True, wall_s=time.perf_counter() - started_wall,
            cpu_s=time.process_time() - started_cpu)
        return True

    def _fresh_footprint_cells(self, grid, footprints):
        cells = set()
        for footprint in footprints:
            age = footprint.get('pose_age_s')
            if age is None or age < 0.0 or age > self.live_pose_max_age_s:
                continue
            cells.update(footprint_cell_indices(
                grid, footprint,
                uncertainty_cells=self.live_footprint_uncertainty_cells))
        return cells

    def _apply_pose_cells(self, cells):
        """Restore old patches and clear new patches without map-sized copies."""
        if self.base_grid is None or self.output_grid is None:
            return 0
        modified = apply_incremental_patch(
            self.base_grid.data, self.output_grid.data,
            self.last_footprint_cells, cells)
        self.last_footprint_cells = set(cells)
        return modified

    def _profile(self, *, mode, map_changed, dimensions, cells_inspected,
                 cells_copied, cells_modified, published, wall_s, cpu_s):
        stats = self.profile_window
        stats['invocations'] += 1
        stats['full_rebuilds'] += int(mode == 'FULL_REBUILD')
        stats['pose_updates'] += int(mode == 'POSE_PATCH')
        stats['cells_inspected'] += cells_inspected
        stats['cells_copied'] += cells_copied
        stats['cells_modified'] += cells_modified
        stats['publications'] += int(published)
        stats['wall_s'] += wall_s
        stats['cpu_s'] += cpu_s
        now = time.monotonic()
        if now - stats['last_log_wall'] < 1.0:
            return
        elapsed = max(1e-9, now - stats['last_log_wall'])
        profile_fields = [
            f'mode={mode}', f'map_changed={map_changed}',
            f'dimensions={dimensions[0]}x{dimensions[1]}',
            f'invocations={stats["invocations"]}',
            f'full_rebuilds={stats["full_rebuilds"]}',
            f'pose_updates={stats["pose_updates"]}',
            f'cells_inspected={stats["cells_inspected"]}',
            f'cells_copied={stats["cells_copied"]}',
            f'cells_modified={stats["cells_modified"]}',
            f'publications={stats["publications"]}',
            f'visualization_bases={stats["visualization_bases"]}',
            f'visualization_updates={stats["visualization_updates"]}',
            f'dirty_events={stats["dirty_events"]}',
            f'coalesced_events={stats["coalesced_events"]}',
            f'rebuild_skipped={stats["rebuild_skipped"]}',
            f'wall_duration_s={stats["wall_s"]:.6f}',
            f'cpu_duration_s={stats["cpu_s"]:.6f}',
            f'window_wall_s={elapsed:.3f}',
        ]
        if mode == 'FRESHNESS_REPUBLISH' and self.last_freshness_source_age_s is not None:
            profile_fields.extend((
                f'freshness_source_age_s={self.last_freshness_source_age_s:.3f}',
                f'freshness_previous_publication_age_s='
                f'{self.last_freshness_previous_publication_age_s:.3f}',
                f'freshness_output_stamp_s={self.last_freshness_output_stamp_s:.3f}',
            ))
        self.get_logger().info('FUSION_PROFILE ' + ' '.join(profile_fields))
        stats.update({
            'invocations': 0, 'full_rebuilds': 0, 'pose_updates': 0,
            'cells_inspected': 0, 'cells_copied': 0, 'cells_modified': 0,
            'publications': 0, 'visualization_bases': 0,
            'visualization_updates': 0, 'dirty_events': 0,
            'coalesced_events': 0, 'rebuild_skipped': 0,
            'wall_s': 0.0, 'cpu_s': 0.0,
            'last_log_wall': now,
        })

    def scheduled_fuse(self):
        if not self.phase_active:
            return
        if self.rebuild_busy:
            self.rebuild_skipped_count += 1
            self.profile_window['rebuild_skipped'] += 1
            return
        self.try_fuse()

    def try_fuse(self):
        if self.rebuild_busy:
            self.rebuild_skipped_count += 1
            self.profile_window['rebuild_skipped'] += 1
            return
        self.rebuild_busy = True
        try:
            self._try_fuse()
        finally:
            self.rebuild_busy = False

    def _try_fuse(self):
        started_wall = time.perf_counter()
        started_cpu = time.process_time()
        if self.local_map is None or self.remote_map is None:
            return
        if self.historical_cleanup_required and not self.historical_cleanup_ready:
            if not self.historical_mask_built:
                # The first base grid below is where the mask is rasterized;
                # both paths must arrive before any shared map can publish.
                if any(path is None for path in self.historical_paths.values()):
                    return
        if not self.map_dirty and not self.sanitize_live_footprints:
            if self._maybe_republish_freshness(
                    [self.local_map, self.remote_map],
                    started_wall, started_cpu):
                return
            self._profile(
                mode='NOOP', map_changed=False, dimensions=(0, 0),
                cells_inspected=0, cells_copied=0, cells_modified=0,
                published=False, wall_s=time.perf_counter() - started_wall,
                cpu_s=time.process_time() - started_cpu)
            return
        messages = [self.local_map, self.remote_map]
        snapshot_time = self._common_snapshot_time(messages)
        try:
            transforms = [
                self.map_transform(message, snapshot_time)
                for message in messages
            ]
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for local-map transforms: {error}',
                throttle_duration_sec=5.0,
            )
            return

        corners = [
            point for message, transform in zip(messages, transforms)
            for point in self.corners(message, transform)
        ]
        minimum_x = math.floor(min(p[0] for p in corners)/self.resolution)*self.resolution
        minimum_y = math.floor(min(p[1] for p in corners)/self.resolution)*self.resolution
        maximum_x = math.ceil(max(p[0] for p in corners)/self.resolution)*self.resolution
        maximum_y = math.ceil(max(p[1] for p in corners)/self.resolution)*self.resolution
        width = max(1, int(round((maximum_x-minimum_x)/self.resolution)))
        height = max(1, int(round((maximum_y-minimum_y)/self.resolution)))

        footprints = []
        pose_key = None
        pose_telemetry = {}
        if self.sanitize_live_footprints:
            footprints, pose_key, pose_telemetry = self.live_footprints(
                snapshot_time)
        if not self.sanitize_live_footprints:
            fused, _ = self._make_base_grid(
                messages, transforms, minimum_x, minimum_y, width, height)
            self._publish_fused(fused)
            self.map_dirty = False
            self._profile(
                mode='FULL_REBUILD', map_changed=True,
                dimensions=(width, height), cells_inspected=sum(
                    len(message.data) for message in messages),
                cells_copied=len(fused.data), cells_modified=0, published=True,
                wall_s=time.perf_counter() - started_wall,
                cpu_s=time.process_time() - started_cpu)
            return
        base_key = (
            self.local_revision, self.remote_content_revision,
            tuple(round(value, 4) for transform in transforms
                  for value in transform),
            minimum_x, minimum_y, width, height,
        )
        map_changed = base_key != self.base_key
        if map_changed:
            now_wall = time.monotonic()
            if (self.base_grid is not None
                    and now_wall - self.last_full_rebuild_wall
                    < self.min_fusion_rebuild_period_s):
                self._profile(
                    mode='COALESCED', map_changed=True,
                    dimensions=(width, height), cells_inspected=0,
                    cells_copied=0, cells_modified=0, published=False,
                    wall_s=time.perf_counter() - started_wall,
                    cpu_s=time.process_time() - started_cpu)
                return
            base, inspected = self._make_base_grid(
                messages, transforms, minimum_x, minimum_y, width, height)
            if self.historical_cleanup_required:
                # Reapply the fixed geometric mask on every local-map rebuild.
                # Slam Toolbox can reintroduce pre-handoff observations into a
                # later OccupancyGrid, and the grid origin/extent may change.
                if self._apply_historical_mask(base) is None:
                    return
            self.base_grid = base
            self.base_key = base_key
            self.output_grid = OccupancyGrid()
            self.output_grid.header = base.header
            self.output_grid.info = base.info
            self.output_grid.data = list(base.data)
            self.last_footprint_cells = set()
            self.map_revision += 1
            self.last_full_rebuild_wall = now_wall
            cells = self._fresh_footprint_cells(base, footprints)
            modified = self._apply_pose_cells(cells)
            mode = 'FULL_REBUILD'
            copied = len(base.data)
            published = True
        else:
            if pose_key == self.last_pose_key:
                if self._maybe_republish_freshness(
                        messages, started_wall, started_cpu):
                    return
                self._profile(
                    mode='NOOP', map_changed=False,
                    dimensions=(width, height), cells_inspected=0,
                    cells_copied=0, cells_modified=0, published=False,
                    wall_s=time.perf_counter() - started_wall,
                    cpu_s=time.process_time() - started_cpu)
                return
            inspected = 0
            cells = self._fresh_footprint_cells(self.base_grid, footprints)
            modified = self._apply_pose_cells(cells)
            mode = 'POSE_PATCH'
            copied = 0
            published = modified > 0
        self.last_pose_key = pose_key
        if published and rclpy.ok():
            self._publish_fused(self.output_grid)
            if self.historical_cleanup_required and not self.historical_cleanup_ready:
                self._publish_cleanup_ready()
        self.map_dirty = False
        self._profile(
            mode=mode, map_changed=map_changed,
            dimensions=(width, height), cells_inspected=inspected,
            cells_copied=copied, cells_modified=modified,
            published=published, wall_s=time.perf_counter() - started_wall,
            cpu_s=time.process_time() - started_cpu)
        if map_changed:
            stale_count = sum(
                1 for item in footprints
                if item.get('pose_age_s') is None
                or item.get('pose_age_s') > self.live_pose_max_age_s)
            own_fresh = sum(
                1 for item in footprints
                if item.get('role') == 'own'
                and item.get('pose_age_s') is not None
                and item.get('pose_age_s') <= self.live_pose_max_age_s)
            peer_fresh = sum(
                1 for item in footprints
                if item.get('role') == 'peer'
                and item.get('pose_age_s') is not None
                and item.get('pose_age_s') <= self.live_pose_max_age_s)
            self.get_logger().info(
                'MAP_SANITIZE ' + ' '.join((
                    f'revision={self.map_revision}',
                    f'cleared_cell_count={modified}',
                    f'stale_pose_count={stale_count}',
                    f'own_pose_age_s={pose_telemetry["own_pose_age_s"]}',
                    f'peer_pose_age_s={pose_telemetry["peer_pose_age_s"]}',
                    f'own_cells_cleared={own_fresh}',
                    f'peer_cells_cleared={peer_fresh}',
                    f'own_clear_skipped_reason={pose_telemetry["own_clear_skipped_reason"]}',
                    f'peer_clear_skipped_reason={pose_telemetry["peer_clear_skipped_reason"]}',
                    f'footprint_radius_m={self.live_footprint_radius}',
                    f'uncertainty_cells={self.live_footprint_uncertainty_cells}',
                )))


def main(args=None):
    rclpy.init(args=args)
    node = SourceAwareMapFusion()
    stopping = threading.Event()

    def stop(signum, frame):
        del signum, frame
        stopping.set()
        # Stop executor callbacks before DDS/rclpy tears down subscriptions.
        # Without this ordering, a final subscription take can race
        # pybind11 destruction and raise a shutdown-only conversion error.
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not stopping.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        stopping.set()
    except RuntimeError:
        # A take/conversion RuntimeError is benign only after shutdown has
        # started; unexpected active-runtime errors must remain visible.
        if not stopping.is_set() and rclpy.ok():
            raise
    finally:
        if node.context.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
