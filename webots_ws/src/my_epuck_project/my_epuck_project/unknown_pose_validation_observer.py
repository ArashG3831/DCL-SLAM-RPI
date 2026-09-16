"""One-shot passive recorder for the initial unknown-pose acceptance."""

from __future__ import annotations

import json
import math
from pathlib import Path
import time

import rclpy
from my_epuck_interfaces.msg import (
    FullMapSnapshotResponse,
    RelativePoseHypothesis,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _stamp_dict(stamp):
    return {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)}


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _yaw(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class UnknownPoseValidationObserver(Node):
    """Record one accepted map-to-map hypothesis and then stop cleanly."""

    def __init__(self):
        super().__init__('unknown_pose_validation_observer')
        self.output_path = Path(str(self.declare_parameter(
            'output_path', '').value))
        self.source_robot_id = str(self.declare_parameter(
            'source_robot_id', 'robot1').value)
        self.target_robot_id = str(self.declare_parameter(
            'target_robot_id', 'robot2').value)
        self.source_map_topic = str(self.declare_parameter(
            'source_map_topic', '/robot1/map').value)
        self.target_map_topic = str(self.declare_parameter(
            'target_map_topic', '/robot2/map').value)
        self.source_map_frame = str(self.declare_parameter(
            'source_map_frame', 'robot1/map').value)
        self.target_map_frame = str(self.declare_parameter(
            'target_map_frame', 'robot2/map').value)
        self._maps = {}
        self._map_samples = {'source': 0, 'target': 0}
        self._snapshot_revisions = {}
        self._accepted_key = None
        self._finished = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        hypothesis_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            OccupancyGrid, self.source_map_topic,
            lambda message: self._map_callback('source', message), map_qos)
        self.create_subscription(
            OccupancyGrid, self.target_map_topic,
            lambda message: self._map_callback('target', message), map_qos)
        for owner in (self.source_robot_id, self.target_robot_id):
            self.create_subscription(
                FullMapSnapshotResponse,
                f'/cslam/unknown_pose/{owner}/full_map_snapshot_response',
                lambda message, item=owner: self._snapshot_callback(item, message),
                hypothesis_qos,
            )
        self.create_subscription(
            RelativePoseHypothesis,
            '/cslam/relative_pose/hypotheses',
            self._hypothesis_callback,
            hypothesis_qos,
        )
        self._finish_timer = None

    def _map_callback(self, role, message):
        self._maps[role] = message
        self._map_samples[role] += 1

    def _snapshot_callback(self, owner, message):
        if not bool(message.available):
            return
        self._snapshot_revisions[owner] = {
            'revision': int(message.revision),
            'snapshot_id': str(message.snapshot_id),
            'timestamp_ns': int(message.timestamp_ns),
            'frame_id': str(message.frame_id),
        }

    def _map_record(self, role, fallback_frame):
        message = self._maps.get(role)
        if message is None:
            return {
                'frame_id': fallback_frame,
                'timestamp': None,
                'timestamp_ns': None,
                'samples_observed': int(self._map_samples[role]),
            }
        return {
            'frame_id': str(message.header.frame_id) or fallback_frame,
            'timestamp': _stamp_dict(message.header.stamp),
            'timestamp_ns': _stamp_ns(message.header.stamp),
            'samples_observed': int(self._map_samples[role]),
        }

    @staticmethod
    def _metrics(message):
        names = (
            'descriptor_similarity', 'descriptor_margin',
            'geometric_inlier_ratio', 'reverse_inlier_ratio',
            'registration_residual_m', 'occupied_free_agreement',
            'overlap_fraction', 'temporal_consistency', 'final_confidence',
            'constraint_count', 'consistent_constraint_count',
            'spatial_baseline_m', 'angular_spread_rad',
            'median_registration_residual_m',
            'p95_registration_residual_m', 'projected_error_m',
            'translation_uncertainty_m', 'yaw_uncertainty_rad',
            'condition_number', 'selector_score', 'selector_null_score',
            'selector_runner_up_score', 'selector_runner_up_margin',
            'mode_index', 'mode_support', 'mode_log_weight',
        )
        return {name: getattr(message, name) for name in names}

    def _hypothesis_callback(self, message):
        if (not bool(message.accepted) or
                str(message.status) != 'ACCEPTED'):
            return
        if (str(message.source_robot_id) != self.source_robot_id or
                str(message.target_robot_id) != self.target_robot_id):
            return
        key = (
            str(message.source_robot_id), str(message.target_robot_id),
            str(message.source_keyframe_id), str(message.target_keyframe_id),
            str(message.evidence_set_hash),
        )
        if self._accepted_key == key:
            return
        if self._accepted_key is not None:
            return
        self._accepted_key = key
        transform = message.source_to_target
        record = {
            'record_type': 'INITIAL_RELATIVE_POSE_ACCEPTED',
            'acceptance_timestamp': _stamp_dict(message.header.stamp),
            'acceptance_timestamp_ns': _stamp_ns(message.header.stamp),
            'source_robot_id': str(message.source_robot_id),
            'target_robot_id': str(message.target_robot_id),
            'source_keyframe_id': str(message.source_keyframe_id),
            'target_keyframe_id': str(message.target_keyframe_id),
            'source_to_target': {
                'translation_x': float(transform.translation.x),
                'translation_y': float(transform.translation.y),
                'translation_z': float(transform.translation.z),
                'yaw_rad': _yaw(transform.rotation),
            },
            'source_map': self._map_record('source', self.source_map_frame),
            'target_map': self._map_record('target', self.target_map_frame),
            'source_snapshot': self._snapshot_revisions.get(
                self.source_robot_id),
            'target_snapshot': self._snapshot_revisions.get(
                self.target_robot_id),
            'evidence_set_hash': str(message.evidence_set_hash),
            'evidence_source_keyframe_ids': list(
                message.evidence_source_keyframe_ids),
            'evidence_target_keyframe_ids': list(
                message.evidence_target_keyframe_ids),
            'status': str(message.status),
            'rejection_reason': str(message.rejection_reason),
            'selector_status': str(message.selector_status),
            'physical_evidence_id': str(message.physical_evidence_id),
            'covariance': list(message.covariance),
            'selector_inlier_probabilities': list(
                message.selector_inlier_probabilities),
            'source_viewpoint': {
                'available': bool(message.source_viewpoint_available),
                'x': float(message.source_viewpoint_x),
                'y': float(message.source_viewpoint_y),
                'yaw': float(message.source_viewpoint_yaw),
            },
            'target_viewpoint': {
                'available': bool(message.target_viewpoint_available),
                'x': float(message.target_viewpoint_x),
                'y': float(message.target_viewpoint_y),
                'yaw': float(message.target_viewpoint_yaw),
            },
            'metrics': self._metrics(message),
            'post_acceptance_odometry_correction': False,
            'ground_truth_transform_used': False,
        }
        if not self.output_path:
            self.get_logger().info(json.dumps(record, sort_keys=True))
        else:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output_path.with_suffix(
                self.output_path.suffix + '.tmp')
            temporary.write_text(json.dumps(record, indent=2, sort_keys=True) + '\n')
            temporary.replace(self.output_path)
        self.get_logger().info(
            'INITIAL_RELATIVE_POSE_ACCEPTED source=%s target=%s '
            'evidence_set_hash=%s' % (
                message.source_robot_id, message.target_robot_id,
                message.evidence_set_hash))
        self._finish_timer = self.create_timer(0.1, self._finish)

    def _finish(self):
        if self._finished:
            return
        self._finished = True
        if self._finish_timer is not None:
            self.destroy_timer(self._finish_timer)
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = UnknownPoseValidationObserver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
