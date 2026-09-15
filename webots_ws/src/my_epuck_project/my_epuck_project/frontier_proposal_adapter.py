"""Adapt current reachable candidates into bounded physical task snapshots."""

import hashlib
import json
import uuid

from geometry_msgs.msg import Point

from my_epuck_interfaces.msg import (
    FrontierCandidateArray,
    PhysicalTask,
    RelativePoseHypothesis,
    TaskSnapshot,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .distributed_assignment.ros_conversion import (
    seconds_to_duration,
    text_to_uuid,
)


def physical_signature(candidate, quantum_m: float = 0.05) -> str:
    """Hash quantized physical geometry without transient frontier identity."""
    def quantize(value):
        return round(value / quantum_m)

    payload = {
        'approach': [
            quantize(candidate.approach_pose.pose.position.x),
            quantize(candidate.approach_pose.pose.position.y),
        ],
        'centroid': [quantize(candidate.centroid.x), quantize(candidate.centroid.y)],
        'bounds': [
            quantize(candidate.bounding_box_min.x),
            quantize(candidate.bounding_box_min.y),
            quantize(candidate.bounding_box_max.x),
            quantize(candidate.bounding_box_max.y),
        ],
    }
    data = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()
    return hashlib.sha256(data).hexdigest()[:24]


class FrontierProposalAdapter(Node):
    """Publish stable top-K physical tasks; never command navigation."""

    def __init__(self):
        """Configure one identity-bound namespaced adapter."""
        super().__init__('frontier_proposal_adapter')
        self._robot_id = self.declare_parameter('robot_id', '').value
        if self._robot_id not in ('robot1', 'robot2'):
            raise ValueError('robot_id must be robot1 or robot2')
        self._maximum_tasks = int(self.declare_parameter('maximum_tasks', 5).value)
        self._validity_s = float(self.declare_parameter('validity_s', 8.0).value)
        self._stop_after_handoff = bool(self.declare_parameter(
            'stop_after_handoff', False,
        ).value)
        self._stopped_after_handoff = False
        self._signature_quantum_m = float(
            self.declare_parameter('signature_quantization_m', 0.05).value,
        )
        input_topic = self.declare_parameter(
            'candidate_topic', 'frontier_candidates',
        ).value
        output_topic = self.declare_parameter('task_snapshot_topic', 'task_snapshot').value
        self._session_text = uuid.uuid4().hex
        self._session_uuid = text_to_uuid(self._session_text)
        self._epoch = 0
        # FrontierCandidateArray is published by the existing C++ generator
        # with the ordinary volatile profile.  Keep that transport contract
        # on the input side; a transient-local subscriber is incompatible
        # with a volatile publisher and silently receives no batches.
        input_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        snapshot_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            TaskSnapshot, output_topic, snapshot_qos,
        )
        self._subscription = self.create_subscription(
            FrontierCandidateArray, input_topic, self._on_candidates, input_qos,
        )
        if self._stop_after_handoff:
            self._handoff_subscription = self.create_subscription(
                RelativePoseHypothesis,
                '/cslam/relative_pose/hypotheses',
                self._on_handoff,
                input_qos,
            )
        self.get_logger().info(
            'proposal adapter robot=%s session=%s top_k=%d dispatch=false' % (
                self._robot_id, self._session_text, self._maximum_tasks,
            )
        )

    @staticmethod
    def _geometry(candidate) -> list[Point]:
        """Return a bounded diagnostic sample from existing candidate geometry."""
        minimum, maximum = candidate.bounding_box_min, candidate.bounding_box_max
        return [
            Point(x=minimum.x, y=minimum.y),
            Point(x=minimum.x, y=maximum.y),
            Point(x=maximum.x, y=minimum.y),
            Point(x=maximum.x, y=maximum.y),
            Point(x=candidate.centroid.x, y=candidate.centroid.y),
        ]

    def _on_candidates(self, candidates: FrontierCandidateArray) -> None:
        """Publish one immutable bounded snapshot for a fresh candidate batch."""
        if self._stopped_after_handoff:
            return
        if candidates.source_robot_id != self._robot_id:
            self.get_logger().error('candidate source identity mismatch; batch rejected')
            return
        self._epoch += 1
        message = TaskSnapshot()
        message.header = candidates.header
        message.source_robot_id = self._robot_id
        message.source_session_id = self._session_uuid
        message.source_snapshot_epoch = self._epoch
        message.source_map_revision = candidates.map_revision
        message.source_costmap_revision = int(
            getattr(candidates, 'costmap_revision', 0))
        fingerprint_data = '%s:%d:%d' % (
            self._robot_id, candidates.map_revision, len(candidates.candidates),
        )
        message.lower_bound_context_fingerprint = str(
            getattr(candidates, 'lower_bound_context_fingerprint', '') or ''
        )
        message.source_map_fingerprint = hashlib.sha256(
            (fingerprint_data + ':' +
             message.lower_bound_context_fingerprint).encode(),
        ).hexdigest()[:24]
        message.task_generation_stamp = candidates.header.stamp
        message.candidate_generation_id = int(
            getattr(candidates, 'candidate_generation_id', 0) or 0
        )
        message.validity = seconds_to_duration(self._validity_s)
        for candidate in candidates.candidates[:self._maximum_tasks]:
            task = PhysicalTask()
            task.source_robot_id = self._robot_id
            task.source_session_id = self._session_uuid
            task.source_snapshot_epoch = self._epoch
            task.source_map_revision = candidates.map_revision
            task.physical_signature = physical_signature(
                candidate, self._signature_quantum_m,
            )
            task.local_frontier_id = candidate.frontier_id
            task.centroid = candidate.centroid
            task.bounding_box_min = candidate.bounding_box_min
            task.bounding_box_max = candidate.bounding_box_max
            task.approach_pose = candidate.approach_pose
            task.frontier_geometry = self._geometry(candidate)
            task.visible_reveal_gain = candidate.information_gain
            task.local_ordering_score = candidate.score
            task.mrtsp_route_rank = candidate.mrtsp_route_rank
            task.mrtsp_route_generation = candidate.mrtsp_route_generation
            task.mrtsp_solver = candidate.mrtsp_solver
            task.local_path_valid = candidate.reachability_state == candidate.REACHABLE
            task.local_path_length_m = candidate.local_path_length_m or candidate.path_length_m
            task.local_path_samples = list(candidate.local_path_samples)
            task.path_heading_cost_rad = candidate.heading_change_rad
            task.generation_stamp = candidates.header.stamp
            message.tasks.append(task)
        self._publisher.publish(message)
        self.get_logger().info(
            'TASK_SNAPSHOT robot=%s session=%s epoch=%d map_revision=%d tasks=%d' % (
                self._robot_id, self._session_text, self._epoch,
                candidates.map_revision, len(message.tasks),
            )
        )

    def _on_handoff(self, message: RelativePoseHypothesis) -> None:
        """Stop the pre-handoff proposal stream at canonical handoff."""
        if (self._stop_after_handoff and not self._stopped_after_handoff and
                bool(message.accepted) and str(message.status) == 'ACCEPTED'):
            self._stopped_after_handoff = True
            self.get_logger().info(
                'FRONTIER_PHASE pre_handoff_stopped=true reason=ACCEPTED_HANDOFF')


def main(args=None):
    """Run the current-candidate proposal adapter."""
    rclpy.init(args=args)
    node = FrontierProposalAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # Launch may invalidate the context before delivering the process
        # signal.  Preserve fail-fast behavior for application errors while
        # making shutdown idempotent.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
