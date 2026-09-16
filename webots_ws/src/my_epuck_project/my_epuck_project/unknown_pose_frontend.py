"""Decentralized descriptor-first unknown relative-pose front end."""

from __future__ import annotations

from collections import Counter, OrderedDict, deque
import copy
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
import json
import hashlib
import math
from pathlib import Path
import time
import zlib

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from my_epuck_interfaces.msg import (
    FullMapSnapshotRequest,
    FullMapSnapshotResponse,
    LocalMapCrop,
    LocalMapCropRequest,
    LocalMapDescriptor,
    PeerMap,
    RelativePoseHypothesis,
)
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)

from .unknown_pose_frontend_core import (
    DedicatedDiagnosticJsonl,
    BoundedVerificationBatchController,
    candidate_reuses_accepted_physical_view,
    candidate_views_are_spatially_separated,
    GridCrop,
    RegistrationResult,
    compare_descriptors,
    compare_descriptor_pairs,
    confirmation_window_for_cadence,
    crop_grid,
    crop_batch_is_ready,
    accumulate_physical_candidates,
    bounded_candidate_verification_order,
    descriptor_match_is_ambiguous,
    descriptor_checksum,
    deduplicate_physical_candidates_with_reasons,
    evidence_batch_is_spatially_diverse,
    evidence_candidates_for_pool,
    evidence_pairs_for_selection,
    invert_se2,
    polar_descriptor,
    prioritize_unambiguous_candidates,
    physical_crop_identity,
    physical_candidate_geometry_identity,
    physical_candidate_identity,
    register_crops,
    reestimate_registration_from_seed,
    refine_registration_locally,
    register_crop_hypotheses,
    register_crop_set,
    select_hypothesis_family,
    iter_stationary_witness_partitions,
    stationary_witness_supports_disjoint,
    consensus_admission_quality,
    consensus_crop_maturity,
    should_accept_hypothesis,
    temporal_support_count,
    temporal_consistency,
)


STATIONARY_WITNESS_SCHEME_VERSION = 'stationary-disjoint-partition-v1'
from .robust_relative_pose_selector import IncrementalHypothesisAccumulator


class UnknownPoseFrontend(Node):
    """Run bounded peer-to-peer place recognition and map registration."""

    def __init__(self):
        super().__init__('unknown_pose_frontend')
        self.declare_parameter('robot_id', self.get_namespace().strip('/'))
        self.declare_parameter('peer_robot_id', '')
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('descriptor_topic', '/cslam/relative_pose/descriptors')
        self.declare_parameter('crop_request_topic', '/cslam/relative_pose/crop_requests')
        self.declare_parameter('crop_topic', '/cslam/relative_pose/crops')
        self.declare_parameter('hypothesis_topic', '/cslam/relative_pose/hypotheses')
        self.declare_parameter('evidence_status_topic', '')
        self.declare_parameter('peer_map_topic', '/cslam/unknown_pose/local_map')
        # Peer maps are full OccupancyGrid samples.  Keep the SLAM map update
        # cadence unchanged, but rate-limit this inter-robot export so a
        # growing map is not retransmitted on every map callback.  A fresh
        # export is still sent immediately when the handoff is accepted.
        self.declare_parameter('peer_map_publish_period_s', 5.0)
        self.declare_parameter('full_map_registration', False)
        # Bounded latest-pair startup cadence; this is not an acceptance gate.
        self.declare_parameter('full_map_registration_period_s', 1.0)
        self.declare_parameter('full_map_max_snapshots', 12)
        self.declare_parameter('descriptor_period_s', 2.0)
        self.declare_parameter('crop_size_m', 8.0)
        # Descriptor/crop requests can arrive after several descriptor periods
        # under the existing synchronous DDS path.  Keep a larger but bounded
        # history so an advertised keyframe remains requestable; this is not a
        # full-map exchange and does not make the history unbounded.
        self.declare_parameter('max_keyframes', 64)
        self.declare_parameter('descriptor_similarity_gate', 0.72)
        self.declare_parameter('descriptor_margin_gate', 0.005)
        self.declare_parameter('minimum_keyframe_confirmations', 2)
        self.declare_parameter('confirmation_window_s', 8.0)
        self.declare_parameter('min_consistent_constraints', 3)
        self.declare_parameter('max_evidence_constraints', 5)
        self.declare_parameter('candidate_verification_budget', 8)
        self.declare_parameter('max_verification_batches', 4)
        self.declare_parameter('verification_lifetime_s', 600.0)
        self.declare_parameter('verification_novelty_spacing_m', 0.40)
        # Evidence keyframes use a conservative physical viewpoint spacing.
        # This is an engineering adaptation for occupancy-map crops; it is
        # not the final consensus baseline and is not cumulative path length.
        self.declare_parameter('evidence_keyframe_translation_threshold_m',
                               0.80)
        self.declare_parameter('evidence_acquisition_window_s', 8.0)
        self.declare_parameter('target_map_radius_m', 40.0)
        self.declare_parameter('max_projected_registration_error_m', 0.20)
        self.declare_parameter('shared_frame', 'shared_map')
        self.declare_parameter('diagnostic_output', '')
        # Optional bounded capture of the exact arrays passed to the existing
        # single-pair registration worker.  This is diagnostic-only and is
        # deliberately disabled unless a run explicitly supplies a directory.
        self.declare_parameter('registration_input_capture_output', '')
        self.declare_parameter('registration_input_capture_max_pairs', 64)
        self.declare_parameter('consensus_min_known_fraction', 0.25)
        self.declare_parameter('consensus_min_occupied_cells', 400)
        # MRPT is the bounded production registration backend.  The legacy
        # matcher remains available only as an explicit compatibility/debug
        # option; MRPT modes are retained per physical pair and selected
        # jointly by the existing consensus logic.
        self.declare_parameter('registration_backend', 'legacy')
        self.declare_parameter('mrpt_max_kld', 0.05)
        self.declare_parameter('mrpt_max_modes_per_call', 64)
        self.declare_parameter('mrpt_repetitions_per_pair', 10)
        self.declare_parameter('mrpt_max_distinct_modes_per_pair', 10)

        self.robot_id = str(self.get_parameter('robot_id').value)
        self.peer_robot_id = str(self.get_parameter('peer_robot_id').value)
        if not self.robot_id or not self.peer_robot_id or self.robot_id == self.peer_robot_id:
            raise ValueError('robot_id and distinct peer_robot_id are required')
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.descriptor_topic = str(self.get_parameter('descriptor_topic').value)
        self.crop_request_topic = str(self.get_parameter('crop_request_topic').value)
        self.crop_topic = str(self.get_parameter('crop_topic').value)
        self.hypothesis_topic = str(self.get_parameter('hypothesis_topic').value)
        self.evidence_status_topic = str(
            self.get_parameter('evidence_status_topic').value)
        if not self.evidence_status_topic:
            self.evidence_status_topic = (
                f'/cslam/relative_pose/{self.robot_id}/'
                'evidence_acquisition_active')
        self.peer_map_topic = str(self.get_parameter('peer_map_topic').value)
        self.peer_map_publish_period_s = max(0.1, float(
            self.get_parameter('peer_map_publish_period_s').value))
        full_map_value = self.get_parameter('full_map_registration').value
        self.full_map_registration = (
            full_map_value if isinstance(full_map_value, bool) else
            str(full_map_value).strip().lower() in ('1', 'true', 'yes', 'on'))
        self.full_map_registration_period_s = max(1.0, float(
            self.get_parameter('full_map_registration_period_s').value))
        self.full_map_max_snapshots = max(3, int(self.get_parameter(
            'full_map_max_snapshots').value))
        self.descriptor_period_s = max(0.2, float(
            self.get_parameter('descriptor_period_s').value))
        self.crop_size_m = max(2.0, float(self.get_parameter('crop_size_m').value))
        self.max_keyframes = max(2, int(self.get_parameter('max_keyframes').value))
        self.similarity_gate = float(
            self.get_parameter('descriptor_similarity_gate').value)
        self.margin_gate = float(self.get_parameter('descriptor_margin_gate').value)
        self.minimum_confirmations = max(2, int(
            self.get_parameter('minimum_keyframe_confirmations').value))
        self.confirmation_window_ns = int(max(1.0, float(
            self.get_parameter('confirmation_window_s').value)) * 1.0e9)
        self.confirmation_cadence_factor = 2.5
        # Three mutually compatible constraints are a non-negotiable safety
        # gate.  Configuration may not weaken estimator acceptance below it.
        self.min_consistent_constraints = max(3, int(
            self.get_parameter('min_consistent_constraints').value))
        self.max_evidence_constraints = max(
            self.min_consistent_constraints, int(
                self.get_parameter('max_evidence_constraints').value))
        self.candidate_verification_budget = max(
            self.min_consistent_constraints, int(
                self.get_parameter('candidate_verification_budget').value))
        self.max_verification_batches = max(1, int(
            self.get_parameter('max_verification_batches').value))
        self.verification_lifetime_s = max(1.0, float(
            self.get_parameter('verification_lifetime_s').value))
        self.verification_novelty_spacing_m = max(0.01, float(
            self.get_parameter('verification_novelty_spacing_m').value))
        self.evidence_keyframe_translation_threshold_m = max(
            0.75, float(self.get_parameter(
                'evidence_keyframe_translation_threshold_m').value))
        self.evidence_acquisition_window_s = max(0.5, float(
            self.get_parameter('evidence_acquisition_window_s').value))
        self.target_map_radius_m = max(1.0, float(
            self.get_parameter('target_map_radius_m').value))
        self.max_projected_registration_error_m = max(0.01, float(
            self.get_parameter('max_projected_registration_error_m').value))
        self.shared_frame = str(self.get_parameter('shared_frame').value)
        self.diagnostic_output = str(self.get_parameter('diagnostic_output').value)
        capture_output = str(self.get_parameter(
            'registration_input_capture_output').value)
        self.registration_input_capture_output = (
            Path(capture_output) if capture_output else None)
        self.registration_input_capture_max_pairs = max(0, int(
            self.get_parameter('registration_input_capture_max_pairs').value))
        self.consensus_min_known_fraction = max(
            0.25, float(self.get_parameter(
                'consensus_min_known_fraction').value))
        self.consensus_min_occupied_cells = max(
            400, int(self.get_parameter(
                'consensus_min_occupied_cells').value))
        self.registration_backend = str(self.get_parameter(
            'registration_backend').value).strip().lower()
        if self.registration_backend not in ('mrpt', 'legacy'):
            raise ValueError(
                f'unsupported registration_backend={self.registration_backend!r}')
        self.mrpt_max_kld = max(0.0, float(self.get_parameter(
            'mrpt_max_kld').value))
        self.mrpt_max_modes_per_call = max(1, int(self.get_parameter(
            'mrpt_max_modes_per_call').value))
        self.mrpt_repetitions_per_pair = max(1, int(self.get_parameter(
            'mrpt_repetitions_per_pair').value))
        self.mrpt_max_distinct_modes_per_pair = max(1, int(
            self.get_parameter('mrpt_max_distinct_modes_per_pair').value))
        self._registration_capture_count = 0
        self._registration_capture_paths = {}
        self.consensus_diagnostics = None
        if self.diagnostic_output:
            self.consensus_diagnostics = DedicatedDiagnosticJsonl(
                Path(self.diagnostic_output) /
                f'{self.robot_id}_consensus_diagnostics.jsonl')
            self.physical_evidence_diagnostics = DedicatedDiagnosticJsonl(
                Path(self.diagnostic_output) /
                f'{self.robot_id}_physical_evidence_diagnostics.jsonl',
                # Physical evidence is a forensic stream, not a raw data
                # archive.  Keep it bounded so repeated candidate rejections
                # cannot consume hundreds of megabytes and starve DDS.
                max_records=20_000, max_bytes=32 * 1024 * 1024)
        else:
            self.physical_evidence_diagnostics = None
        self.counters = {
            'map_messages_received': 0,
            'descriptors_published': 0,
            'descriptors_received': 0,
            'candidate_comparisons': 0,
            'cheap_candidates': 0,
            'cheap_rejections': 0,
            'descriptor_ambiguity_rejections': 0,
            'descriptor_ambiguity_advisories': 0,
            'crop_requests_sent': 0,
            'crop_requests_queued': 0,
            'crop_request_duplicates_suppressed': 0,
            'crop_requests_received': 0,
            'crops_sent': 0,
            'crops_received': 0,
            'crop_responses_accepted': 0,
            'crop_response_rejections': 0,
            'registrations': 0,
            'registration_callback_entries': 0,
            'registration_callback_exits': 0,
            'registration_callback_exceptions': 0,
            'proposals_published': 0,
            'hypothesis_summaries_published': 0,
            'hypothesis_summaries_received': 0,
            'evidence_announcements_published': 0,
            'evidence_announcements_received': 0,
            'evidence_reverification_requests': 0,
            'acks_published': 0,
            'accepted_hypotheses': 0,
            'rejected_hypotheses': 0,
            'tf_handoffs': 0,
            'peer_maps_published': 0,
            'full_map_evidence_requests': 0,
            'full_map_evidence_responses': 0,
            'full_map_evidence_cache_hits': 0,
            'full_map_evidence_cache_misses': 0,
            'full_map_verifications_pending': 0,
            'full_map_verifications_resumed': 0,
            'merge_handoff_started': 0,
            'multi_constraint_attempts': 0,
            'multi_constraint_rejections': 0,
            'candidate_selections': 0,
            'candidate_selection_deferrals': 0,
            'physical_candidate_duplicates_suppressed': 0,
            'physical_geometry_rejections_suppressed': 0,
            'physical_evidence_duplicates_suppressed': 0,
            'spatial_diversity_deferrals': 0,
            'pending_candidate_additions': 0,
            'pending_candidate_removals': 0,
            'constraints_accumulated': 0,
            'evidence_sets_formed': 0,
            'temporal_gate_rejections': 0,
            'temporal_support_evaluations': 0,
            'temporal_support_max': 0,
            'candidate_verification_attempts': 0,
            'candidate_verification_accepted': 0,
            'candidate_verification_rejected': 0,
            'candidate_verification_budget_exhausted': 0,
            'candidate_verification_budget_waits': 0,
            'immature_candidates_not_scheduled': 0,
            'verification_batches_opened': 0,
            'verification_batches_exhausted': 0,
            'verification_batch_reentries': 0,
            'verification_novelty_deferrals': 0,
            'verification_lifetime_expired': 0,
            'stale_verification_batch_responses': 0,
            'diagnostic_write_failures': 0,
            'post_handoff_protocol_ticks_skipped': 0,
            'keyframe_content_duplicates_suppressed': 0,
            'keyframe_motion_novelty_admitted': 0,
            'physical_content_duplicates_suppressed': 0,
            'strong_consensus_candidates': 0,
            'weak_consensus_candidates_rejected': 0,
        }
        self.gate_rejection_counts = Counter()
        self.temporal_gate_rejection_counts = Counter()
        self.crop_response_rejection_counts = Counter()
        self.consensus_gate_rejection_counts = Counter()
        self.descriptor_gate_survivors = set()
        self.descriptor_ambiguous_pairs = set()
        self.temporal_gate_survivors = set()
        self.diagnostic_events = []
        self.diagnostic_event_drops = 0
        # Keep the bounded ordinary trace small because descriptor/crop
        # traffic is high-volume, but retain the complete protocol handshake
        # trace in a separate bounded stream.  Without this split a long
        # smoke can evict the proposal/ack reason before finalization.
        self.protocol_lifecycle_events = []
        self.protocol_lifecycle_event_drops = 0
        if self.registration_backend == 'mrpt':
            self._record_diagnostic_event(
                'MRPT_CONFIGURATION', method='amModifiedRANSAC',
                max_kld=float(self.mrpt_max_kld),
                repetitions_per_physical_pair=int(
                    self.mrpt_repetitions_per_pair),
                mode_dedup_translation_m=0.05,
                mode_dedup_yaw_deg=0.5,
                max_distinct_modes_per_pair=int(
                    self.mrpt_max_distinct_modes_per_pair))
        self.callback_stats = {}
        self.callback_started = 0
        self.callback_completed = 0
        self.callback_inflight = 0
        self.max_callback_inflight = 0
        self.max_backlog_estimate = 0
        self.cpu_samples = []
        self._last_cpu_wall = time.monotonic()
        self._last_cpu_process = time.process_time()
        self.best_similarity = 0.0
        self.best_margin = 0.0
        self.best_known_fraction = 0.0
        self.merge_handoff_logged = False
        # Bounded diagnostic timing/size state.  These values are never used
        # by descriptor matching, registration, or merger acceptance.
        self.first_map_wall = None
        self.first_candidate_wall = None
        self.accepted_wall = None
        self.descriptor_bytes = 0
        self.crop_cells_sent = 0
        self.crop_cells_received = 0
        self.active_candidate_pairs = []
        self.pending_candidate_pairs = {}
        self.request_own_by_peer_key = {}
        self.request_own_by_request_key = {}
        self.evidence_pairs = {}
        # Retain the descriptor metadata for accepted evidence even after the
        # bounded live keyframe cache evicts its advertisement.  Consensus
        # may complete on a later callback; proposal publication still needs
        # the exact source/target IDs and descriptor provenance for that
        # already-accepted crop pair.
        self.evidence_candidates = {}
        self.evidence_physical_keys = {}
        self.evidence_physical_geometry_keys = set()
        self.evidence_content_pairs = set()
        self.evidence_source_content = set()
        self.evidence_peer_content = set()
        self.candidate_verification_attempted = set()
        self.candidate_verification_results = {}
        # Preserve bounded MRPT alternatives for each physical pair.  These
        # are mutually exclusive modes, not additional independent evidence.
        self.candidate_verification_hypotheses = {}
        # Acquisition-only history used to keep one repeatedly rejected
        # physical view from monopolising later verification batches.  Exact
        # pair rejection and accepted-evidence deduplication remain separate
        # and authoritative; this counter changes ordering only.
        self.attempted_physical_view_reuse_counts = {}
        self.request_candidate_by_request_key = {}
        self.request_metadata_by_request_key = {}
        self.completed_request_keys = set()
        self.candidate_verification_attempts = 0
        self.candidate_verification_batch_attempts = 0
        self.verification_attempt_sequence = 0
        self.rejected_physical_evidence_keys = set()
        # Exact physical identities retain epoch/checksum freshness.  This
        # companion set prevents a rejected crop footprint from being retried
        # repeatedly within the same bounded verification batch.  A geometry
        # rejection is not permanent: after a batch rolls over, a new map
        # revision/keyframe may make the same footprint useful evidence again.
        # Keeping the batch association avoids starving acquisition while
        # preserving incremental evidence across later batches.
        self.rejected_physical_geometry_keys = set()
        self.rejected_physical_geometry_batches = {}
        # Track geometry as soon as a request is admitted, not only after its
        # asynchronous result arrives.  Otherwise several in-flight keyframe
        # IDs can represent the same crop footprint and consume one batch's
        # bounded verification budget before the first rejection is recorded.
        self.attempted_physical_geometry_batches = {}
        self._diagnosed_physical_candidates = set()
        # Repeated observations of an already-pending physical candidate are
        # represented by the counter below.  Persisting one diagnostic record
        # per repetition can dominate the single-threaded executor and delay
        # the actual crop/registration callbacks; the estimator state itself
        # remains unchanged and every request/result/rejection is still
        # recorded.
        self._diagnosed_duplicate_physical_candidates = set()
        self._physical_diagnostic_summary = Counter()
        self._physical_diagnostic_suppressed = Counter()
        self.received_peer_crops = {}
        self.batch_proposal_published = False
        self.pending_target_proposal = False
        self.local_hypothesis_summary = None
        self.local_hypothesis_result = None
        self.peer_hypothesis_summary = None
        self.peer_summary_source_ids = set()
        self.peer_summary_target_ids = {}
        # Before either peer has three local constraints, exchange each
        # geometrically accepted constraint so the other peer can request the
        # same crops and independently re-register it.  This is the missing
        # incremental, peer-to-peer evidence path; it never marks a single
        # constraint as a handoff.
        self.peer_evidence_announcements = {}
        self.pending_peer_evidence_announcements = {}
        self.evidence_announcements_published = set()
        self._peer_evidence_requested = set()
        # Canonical R1->R2 physical evidence shared by both frontends.  The
        # key is the canonical physical pair, so reciprocal discovery and
        # repeated delivery cannot create a second independent constraint.
        self.canonical_constraint_pool = OrderedDict()
        self.canonical_union_summary_published = False
        # A summary is an immutable digest of a selected evidence set.  Once
        # this peer has independently verified that digest, repeated DDS
        # deliveries of the same CANDIDATE must not trigger another crop
        # request/registration cycle.
        self._verified_peer_summary_hash = ''
        self.hypothesis_accumulator = IncrementalHypothesisAccumulator(
            min_inliers=self.min_consistent_constraints)
        self.registration_callback_depth = 0
        # Geometric registration is CPU-heavy and must never run inside a
        # subscription callback.  A single bounded worker preserves ordering
        # while allowing DDS/heartbeat callbacks and the watchdog timer to
        # continue being serviced.  Results are applied only by ``tick`` on
        # the ROS executor thread.
        self._registration_executor = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix=f'{self.robot_id}_registration')
        self._registration_future = None
        self._registration_context = None
        self._registration_shutdown = False
        self._registration_queue_drops = 0
        # Crop responses can arrive faster than the single CPU-heavy
        # registration worker.  The previous implementation discarded every
        # response observed while the worker was busy, even though the
        # corresponding verification attempt had already been consumed.  Keep
        # a bounded FIFO of immutable work items instead; a full queue is an
        # explicit, diagnosable overflow rather than silent evidence loss.
        self._registration_pending_contexts = deque(maxlen=max(
            32, self.candidate_verification_budget * 4))
        # Keep in-flight crop responses below the worker's service capacity.
        # Responses already on the wire may still enter the FIFO, so the
        # separate hard capacity above remains the final safety bound.
        self._registration_backpressure_depth = max(
            4, min(8, self.candidate_verification_budget))
        self._registration_backpressure_events = 0
        self._registration_pending_keys = set()
        self._registration_queue_enqueues = 0
        self._registration_queue_dequeues = 0
        self._registration_queue_max_depth = 0
        self.evidence_acquisition_deadline_wall = None
        self.evidence_acquisition_started = False
        self._last_evidence_status = None
        self._evidence_opportunity_deadline_wall = None
        self.verification_batches = BoundedVerificationBatchController(
            budget=self.candidate_verification_budget,
            max_batches=self.max_verification_batches,
            lifetime_s=self.verification_lifetime_s,
            novelty_spacing_m=self.verification_novelty_spacing_m)

        qos = QoSProfile(
            depth=20, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.descriptor_pub = self.create_publisher(
            LocalMapDescriptor, self.descriptor_topic, qos)
        self.request_pub = self.create_publisher(
            LocalMapCropRequest, self.crop_request_topic, qos)
        self.crop_pub = self.create_publisher(LocalMapCrop, self.crop_topic, qos)
        self.hypothesis_pub = self.create_publisher(
            RelativePoseHypothesis, self.hypothesis_topic, qos)
        self.evidence_status_pub = self.create_publisher(
            Bool, self.evidence_status_topic, qos)
        self.peer_map_pub = self.create_publisher(PeerMap, self.peer_map_topic, map_qos)
        self.peer_full_map_topic = (
            f'/cslam/unknown_pose/{self.peer_robot_id}/local_map')
        self.peer_full_map_sub = self.create_subscription(
            PeerMap, self.peer_full_map_topic,
            lambda message: self._timed_callback(
                'peer_full_map_callback', self.peer_full_map_callback, message),
            map_qos)
        self.full_map_snapshot_request_topic = (
            f'/cslam/unknown_pose/{self.peer_robot_id}/full_map_snapshot_request')
        self.full_map_snapshot_response_topic = (
            f'/cslam/unknown_pose/{self.peer_robot_id}/full_map_snapshot_response')
        self.full_map_snapshot_request_pub = self.create_publisher(
            FullMapSnapshotRequest, self.full_map_snapshot_request_topic, qos)
        self.full_map_snapshot_response_pub = self.create_publisher(
            FullMapSnapshotResponse, self.full_map_snapshot_response_topic, qos)
        self.full_map_snapshot_request_sub = self.create_subscription(
            FullMapSnapshotRequest,
            f'/cslam/unknown_pose/{self.robot_id}/full_map_snapshot_request',
            lambda message: self._timed_callback(
                'full_map_snapshot_request_callback',
                self.full_map_snapshot_request_callback, message), qos)
        self.full_map_snapshot_response_sub = self.create_subscription(
            FullMapSnapshotResponse,
            f'/cslam/unknown_pose/{self.robot_id}/full_map_snapshot_response',
            lambda message: self._timed_callback(
                'full_map_snapshot_response_callback',
                self.full_map_snapshot_response_callback, message), qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic,
            lambda message: self._timed_callback(
                'map_callback', self.map_callback, message), map_qos)
        self.descriptor_sub = self.create_subscription(
            LocalMapDescriptor, self.descriptor_topic,
            lambda message: self._timed_callback(
                'descriptor_callback', self.descriptor_callback, message), qos)
        self.request_sub = self.create_subscription(
            LocalMapCropRequest, self.crop_request_topic,
            lambda message: self._timed_callback(
                'request_callback', self.request_callback, message), qos)
        self.crop_sub = self.create_subscription(
            LocalMapCrop, self.crop_topic,
            lambda message: self._timed_callback(
                'crop_callback', self.crop_callback, message), qos)
        self.hypothesis_sub = self.create_subscription(
            RelativePoseHypothesis, self.hypothesis_topic,
            lambda message: self._timed_callback(
                'hypothesis_callback', self.hypothesis_callback, message), qos)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # The accepted alignment is immutable for the lifetime of this
        # frontend. Publish it on /tf_static so a post-handoff fusion process
        # can start later and still resolve the canonical map chain; a
        # one-shot dynamic /tf sample expires from a late listener's buffer.
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.latest_map = None
        self.latest_map_fingerprint = None
        self.latest_full_map_snapshot = None
        self.peer_full_map_snapshots = OrderedDict()
        self.local_full_map_snapshots = OrderedDict()
        # Evidence pacing is expressed in ROS/simulation seconds.  Using a
        # wall clock here made a fast Webots run stretch every one-second
        # registration export and five-second peer-map export by the
        # real-time factor while the evidence itself was timestamped in ROS
        # time.  Keep wall clocks for diagnostics/worker bookkeeping only.
        self._last_full_map_export_ros_s = 0.0
        self._last_full_map_export_fingerprint = None
        self._last_full_map_attempt_ros_s = 0.0
        self._last_full_map_attempt_pair = None
        self._full_map_registration_future = None
        self._full_map_registration_context = None
        self._full_map_registration_sequence = 0
        self.full_map_confirmation_records = []
        self.full_map_confirmation_families = []
        self.full_map_proposal = None
        self._full_map_proposal_pins = set()
        self._pending_full_map_verification = None
        self._requested_full_map_snapshot_ids = set()
        self._unavailable_full_map_snapshot_ids = set()
        self.last_export_map_fingerprint = None
        self.map_revision = 0
        self.keyframe_sequence = 0
        self.last_descriptor_wall = 0.0
        self._latest_observation_pose = None
        self.keyframe_content_history = OrderedDict()
        self.keyframes = OrderedDict()
        # Local odometry pose at immutable evidence-keyframe creation. This
        # is selector diversity metadata, not an estimator transform.
        self.keyframe_viewpoints = OrderedDict()
        self._last_evidence_viewpoint = None
        self._evidence_cumulative_travel_m = 0.0
        self.peer_descriptors = OrderedDict()
        self.own_descriptor_stamps_ns = deque(maxlen=16)
        self.peer_descriptor_stamps_ns = deque(maxlen=16)
        self.effective_confirmation_window_ns = self.confirmation_window_ns
        self.matches = {}
        self.compared_pairs = set()
        # Descriptor subscriptions are serviced by the ROS executor.  Keep
        # their callbacks bounded: comparison work is drained by the existing
        # timer one keyframe at a time so a busy peer cannot starve receipt of
        # newer descriptors.
        self.pending_peer_descriptor_keys = deque(maxlen=self.max_keyframes)
        self.pending_own_descriptor_keys = deque(maxlen=self.max_keyframes)
        self.pending_descriptor_pair_keys = deque(maxlen=8192)
        self.pending_descriptor_pair_key_set = set()
        self.descriptor_pair_budget_per_tick = 16
        self.descriptor_gate_status = {}
        self.temporal_support_cache = {}
        self.temporal_gate_rejected_pairs = set()
        self.confirmations = {}
        self.pending_requests = set()
        self.pending_proposals = {}
        # Retain the canonical proposal envelope until the peer ACK arrives.
        # The bounded keyframe cache may evict the source/target descriptors
        # while confirmation is in flight; finalizing an ACK must not depend
        # on re-looking up those descriptors.  This is deliberately bounded
        # by the same proposal lifecycle (publish -> ACK/REJECT), not an
        # unbounded evidence stream.
        self.pending_proposal_messages = {}
        # Keep the exact evidence-set fingerprint alongside the local
        # RegistrationResult.  RegistrationResult intentionally contains
        # geometry/quality only; the fingerprint belongs to the replicated
        # protocol envelope and must not be read as an attribute on it.
        self.pending_proposal_evidence_hashes = {}
        self.peer_proposals = {}
        self.pending_target_proposal = False
        self.negotiation_started = False
        self.accepted = None
        # Once the canonical transform is immutable, retain only the local
        # map -> PeerMap relay and the accepted static TF publisher.  The
        # transition is deliberately separate from ``accepted`` so delayed
        # DDS callbacks can be rejected even after their subscriptions have
        # been destroyed.
        self._post_handoff_quiesced = False
        # Diagnostic provenance only: the local ROS simulation timestamp at
        # which this frontend accepted the canonical handoff message.  It is
        # never read by registration, selector, verification, or TF publish.
        self.accepted_ros_time_s = None
        self.last_export_ros_s = 0.0
        self.timer = self.create_timer(
            0.2, lambda: self._timed_callback('timer_tick', self.tick))
        self.get_logger().info(
            f'Unknown-pose front end {self.robot_id}<->{self.peer_robot_id}; '
            'no transform is published before mutual acceptance')
        self._publish_evidence_status(False)

    def _publish_evidence_status(self, active: bool) -> None:
        """Publish a lease-backed evidence-opportunity status.

        This status contains no pose, map, or acceptance result.  It only lets
        the local allocator stop issuing a new exploration goal while the
        existing frontend is collecting/validating a promising set of views.
        The allocator expires the status if the frontend stops refreshing it.
        """
        message = Bool()
        message.data = bool(active)
        # Finalization may run after ROS has begun shutting down (for
        # example, when a bounded runner receives SIGINT).  This advisory
        # status must never turn teardown into a frontend crash.
        publisher = getattr(self, 'evidence_status_pub', None)
        try:
            if publisher is not None:
                publisher.publish(message)
        except Exception as error:  # rclpy.RCLError varies by ROS release
            if rclpy.ok():
                self.get_logger().warning(
                    'EVIDENCE_STATUS_PUBLISH_FAILED active=%s error=%s' %
                    (bool(active), error))
        if self._last_evidence_status is None or \
                bool(active) != self._last_evidence_status:
            self._last_evidence_status = bool(active)
            self._record_diagnostic_event(
                'EVIDENCE_ACQUISITION_STATUS', active=bool(active))

    def _destroy_post_handoff_entity(self, attribute, kind):
        """Destroy one registration-only ROS entity, tolerating late teardown."""
        entity = getattr(self, attribute, None)
        if entity is None:
            return
        try:
            if kind == 'subscription':
                self.destroy_subscription(entity)
            elif kind == 'publisher':
                self.destroy_publisher(entity)
            elif kind == 'timer':
                self.destroy_timer(entity)
        except Exception as error:
            # A queued callback or an already-started ROS teardown must not
            # turn an otherwise accepted handoff into a process failure.
            logger = getattr(self, 'get_logger', lambda: None)()
            warning = getattr(logger, 'warning', None)
            if warning is not None:
                warning('post-handoff %s cleanup failed for %s: %s' %
                        (kind, attribute, error))
        finally:
            setattr(self, attribute, None)

    def _enter_post_handoff_quiescence(self):
        """Stop registration work while retaining fusion-facing state.

        Acceptance is immutable.  After this transition no descriptor, crop,
        hypothesis, or full-map registration callback is allowed to create
        new work.  The local OccupancyGrid subscription and PeerMap publisher
        remain alive because source-aware fusion consumes the latter for
        ongoing shared-map updates.  The static TF broadcaster also remains
        alive for late/reconnected TF consumers.
        """
        if getattr(self, '_post_handoff_quiesced', False):
            return
        self._post_handoff_quiesced = True
        self._registration_shutdown = True
        self.evidence_acquisition_started = False
        self._evidence_opportunity_deadline_wall = None
        self.evidence_acquisition_deadline_wall = None
        self._publish_evidence_status(False)

        self._record_diagnostic_event(
            'POST_HANDOFF_QUIESCED',
            retained_local_map_relay=True,
            retained_static_tf=True,
            registration_work_disabled=True)

        for attribute in ('_registration_future',
                          '_full_map_registration_future'):
            future = getattr(self, attribute, None)
            if future is not None:
                try:
                    future.cancel()
                except Exception:
                    pass
            setattr(self, attribute, None)
        self._registration_context = None
        self._full_map_registration_context = None
        self._clear_pending_registration_contexts('POST_HANDOFF_QUIESCED')
        for attribute in (
                'pending_requests', 'completed_request_keys',
                'request_candidate_by_request_key',
                'request_metadata_by_request_key', 'pending_descriptor_pair_keys',
                'pending_descriptor_pair_key_set', 'pending_peer_descriptor_keys',
                'pending_own_descriptor_keys', 'pending_peer_evidence_announcements',
                'peer_evidence_announcements', 'received_peer_crops',
                'peer_descriptors', 'keyframes', 'keyframe_viewpoints',
                'local_full_map_snapshots', 'peer_full_map_snapshots'):
            value = getattr(self, attribute, None)
            if hasattr(value, 'clear'):
                value.clear()
        self.latest_full_map_snapshot = None
        self.full_map_proposal = None
        self.pending_proposals.clear()
        self.peer_proposals.clear()

        executor = getattr(self, '_registration_executor', None)
        if executor is not None:
            try:
                executor.shutdown(wait=False, cancel_futures=True)
            except Exception:
                pass

        self._destroy_post_handoff_entity('timer', 'timer')
        for attribute in (
                'peer_full_map_sub', 'full_map_snapshot_request_sub',
                'full_map_snapshot_response_sub', 'descriptor_sub',
                'request_sub', 'crop_sub', 'hypothesis_sub'):
            self._destroy_post_handoff_entity(attribute, 'subscription')
        for attribute in (
                'descriptor_pub', 'request_pub', 'crop_pub', 'hypothesis_pub',
                'evidence_status_pub', 'full_map_snapshot_request_pub',
                'full_map_snapshot_response_pub'):
            self._destroy_post_handoff_entity(attribute, 'publisher')

        # This listener belongs only to the pre-handoff registration frontend.
        # Keep the StaticTransformBroadcaster above it alive; other production
        # nodes have their own TF listeners.
        listener = getattr(self, 'tf_listener', None)
        if listener is not None:
            try:
                listener.unregister()
            except Exception:
                pass
            self.tf_listener = None

    def _advertise_evidence_opportunity(self, candidate_count: int) -> None:
        """Advertise the first bounded, bidirectionally eligible encounter.

        Descriptor similarity is only an advisory trigger.  Once temporal
        support has made a peer/own pair actionable, publish the existing
        lease before candidate selection can consume the verification budget
        or the allocator can issue another goal.  The allocator checks this
        lease only when it is otherwise ready to dispatch, so an active Nav2
        goal is left untouched.  The lease expires through the normal frontend
        tick path if no requestable, genuinely new evidence arrives.
        """
        if (self.evidence_acquisition_started or self.batch_proposal_published
                or self._evidence_opportunity_deadline_wall is not None):
            return
        self._evidence_opportunity_deadline_wall = (
            time.monotonic() + self.evidence_acquisition_window_s)
        self._publish_evidence_status(True)
        self._record_diagnostic_event(
            'EVIDENCE_OPPORTUNITY_ADVERTISED',
            candidate_count=int(candidate_count),
            window_s=float(self.evidence_acquisition_window_s),
            active_goal_untouched=True)
        self._write_physical_evidence_diagnostic(
            'EVIDENCE_OPPORTUNITY_ADVERTISED',
            candidate_count=int(candidate_count),
            window_s=float(self.evidence_acquisition_window_s),
            active_goal_untouched=True,
            action='HOLD_NEW_DISPATCH_ONLY')

    def _write_consensus_diagnostic(self, record_type, **fields):
        """Stream consensus records independently of bounded protocol events."""
        if self.consensus_diagnostics is None:
            return
        try:
            record = {
                'record_type': str(record_type),
                'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'wall_monotonic_s': time.monotonic(),
                'ros_time_s': self.get_clock().now().nanoseconds / 1.0e9,
                'acquisition_batch_id': int(self.verification_batches.batch_id),
                'verification_batch_attempts': int(
                    self.candidate_verification_batch_attempts),
            }
            record.update(fields)
            self.consensus_diagnostics.write(record)
        except Exception as error:  # diagnostics must never kill estimation
            self.counters['diagnostic_write_failures'] += 1
            self.get_logger().error(
                'UNKNOWN_POSE_DIAGNOSTIC_WRITE_FAILURE record=%s error=%s' %
                (record_type, error))

    def _write_physical_evidence_diagnostic(self, record_type, **fields):
        """Stream formation evidence independently of protocol-event storage."""
        self._physical_diagnostic_summary[str(record_type)] += 1
        if self.physical_evidence_diagnostics is None:
            return
        # High-volume negative scheduling records are summarized after a
        # small forensic sample.  Important lifecycle, crop, registration,
        # and handoff records remain lossless until the bounded writer cap.
        repetitive = {
            'CANDIDATE_VERIFICATION_SKIPPED',
            'PENDING_CANDIDATE_DUPLICATE_SUPPRESSED',
            'CANDIDATE_REJECTED_BEFORE_CONSENSUS',
        }
        if str(record_type) in repetitive and \
                self._physical_diagnostic_summary[str(record_type)] > 64:
            self._physical_diagnostic_suppressed[str(record_type)] += 1
            return
        try:
            record = {
                'record_type': str(record_type),
                'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'wall_monotonic_s': time.monotonic(),
                'ros_time_s': self.get_clock().now().nanoseconds / 1.0e9,
                'acquisition_batch_id': int(self.verification_batches.batch_id),
                'verification_batch_attempts': int(
                    self.candidate_verification_batch_attempts),
            }
            record.update(fields)
            self.physical_evidence_diagnostics.write(record)
        except Exception as error:  # diagnostics must never kill estimation
            self.counters['diagnostic_write_failures'] += 1
            self.get_logger().error(
                'UNKNOWN_POSE_DIAGNOSTIC_WRITE_FAILURE record=%s error=%s' %
                (record_type, error))

    @staticmethod
    def _descriptor_geometry(descriptor):
        resolution = float(descriptor.resolution)
        width = int(descriptor.crop_width)
        height = int(descriptor.crop_height)
        origin = [float(descriptor.crop_origin_x),
                  float(descriptor.crop_origin_y),
                  float(getattr(descriptor, 'crop_origin_yaw', 0.0))]
        center = [origin[0] + 0.5 * width * resolution,
                  origin[1] + 0.5 * height * resolution]
        return {
            'origin': origin,
            'center': center,
            'width': width,
            'height': height,
            'resolution': resolution,
            'orientation_yaw': 0.0,
            'map_epoch': int(descriptor.map_epoch),
            'checksum': int(descriptor.checksum),
            'keyframe_id': str(descriptor.keyframe_id),
            'keyframe_creation_timestamp_ns': int(
                UnknownPoseFrontend._stamp_ns(descriptor)),
            'viewpoint': (
                None if not bool(getattr(descriptor, 'viewpoint_available', False))
                else [float(getattr(descriptor, 'viewpoint_x', 0.0)),
                      float(getattr(descriptor, 'viewpoint_y', 0.0)),
                      float(getattr(descriptor, 'viewpoint_yaw', 0.0))]),
        }

    @staticmethod
    def _crop_geometry(crop, keyframe_id='', map_epoch=0, checksum=0):
        height, width = crop.values.shape[:2]
        origin = [float(crop.origin_x), float(crop.origin_y),
                  float(crop.origin_yaw)]
        centre = [
            float(crop.origin_x + 0.5 * width * crop.resolution),
            float(crop.origin_y + 0.5 * height * crop.resolution)]
        return {
            'origin': origin,
            'center': centre,
            'width': int(width),
            'height': int(height),
            'resolution': float(crop.resolution),
            'orientation_yaw': float(crop.origin_yaw),
            'map_epoch': int(map_epoch),
            'checksum': int(checksum),
            'keyframe_id': str(keyframe_id),
        }

    def _candidate_diagnostic(self, candidate, status=None, reason=None,
                              compact=False):
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        own_entry = self.keyframes.get(own_key)
        own_crop = None if own_entry is None else own_entry[1]
        if own_entry is None or own_crop is None:
            return {
                'own_keyframe_id': str(own_key),
                'peer_keyframe_id': str(peer_key),
                'status': status,
                'rejection_reason': reason,
            }
        match = self.matches.get((peer_key, own_key))
        common = {
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'peer_descriptor': self._descriptor_geometry(peer),
            'own_crop': self._crop_geometry(
                own_crop, own.keyframe_id, own.map_epoch, own.checksum),
            'physical_identity': list(physical_candidate_identity(
                candidate, {own_key: own_crop}) ),
            'descriptor_similarity': (
                None if match is None else float(match.similarity)),
            'descriptor_margin': (
                None if match is None else float(match.margin)),
            'status': status,
            'rejection_reason': reason,
        }
        if compact:
            # Selection/formation diagnostics can contain many candidates.
            # Keep every field needed to reconstruct physical identity and
            # geometry without repeating the redundant own descriptor object.
            common['own_keyframe_creation_timestamp_ns'] = int(
                self._stamp_ns(own))
            return common
        common['own_descriptor'] = self._descriptor_geometry(own)
        return common

    def _candidate_diagnostic_reference(self, candidate, status=None,
                                        reason=None):
        """Return a small reference for repeated selection-attempt records."""
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        match = self.matches.get((peer_key, own_key))
        own_entry = self.keyframes.get(own_key)
        own_crop = None if own_entry is None else own_entry[1]
        return {
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'physical_identity': list(self._candidate_physical_key(candidate)),
            'own_crop': None if own_crop is None else self._crop_geometry(
                own_crop, own_key, own.map_epoch, own.checksum),
            'peer_crop': self._descriptor_geometry(peer),
            'own_map_epoch': int(own.map_epoch),
            'own_checksum': int(own.checksum),
            'peer_map_epoch': int(peer.map_epoch),
            'peer_checksum': int(peer.checksum),
            'descriptor_similarity': (
                None if match is None else float(match.similarity)),
            'descriptor_margin': (
                None if match is None else float(match.margin)),
            'status': status,
            'rejection_reason': reason,
        }

    def _record_diagnostic_event(self, event_type, **fields):
        """Retain a bounded wall/ROS timestamped protocol trace."""
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        event = {
            'event': event_type,
            'wall_monotonic_s': now,
            'ros_time_s': ros_now / 1.0e9,
        }
        event.update(fields)
        lifecycle = (
            str(event_type).startswith(('EVIDENCE_ANNOUNCEMENT',
                                        'PEER_EVIDENCE_',
                                        'PROPOSAL_',
                                        'LOCAL_HYPOTHESIS',
                                        'PEER_HYPOTHESIS',
                                        'HYPOTHESIS_',
                                        'CANONICAL_',
                                        'UNKNOWN_POSE_MERGE_')))
        if lifecycle:
            if len(self.protocol_lifecycle_events) < 512:
                self.protocol_lifecycle_events.append(event)
            else:
                self.protocol_lifecycle_event_drops += 1
        if len(self.diagnostic_events) < 512:
            self.diagnostic_events.append(event)
        else:
            self.diagnostic_event_drops += 1

    def _timed_callback(self, name, callback, *args):
        """Measure callback service time without changing callback behavior."""
        started = time.perf_counter()
        self.callback_started += 1
        self.callback_inflight += 1
        self.max_callback_inflight = max(
            self.max_callback_inflight, self.callback_inflight)
        self.max_backlog_estimate = max(
            self.max_backlog_estimate,
            max(0, self.callback_started - self.callback_completed - 1))
        try:
            return callback(*args)
        finally:
            duration_ms = (time.perf_counter() - started) * 1000.0
            stats = self.callback_stats.setdefault(name, {
                'count': 0, 'total_ms': 0.0, 'max_ms': 0.0,
                'samples_ms': [],
            })
            stats['count'] += 1
            stats['total_ms'] += duration_ms
            stats['max_ms'] = max(stats['max_ms'], duration_ms)
            if len(stats['samples_ms']) < 256:
                stats['samples_ms'].append(duration_ms)
            self.callback_inflight -= 1
            self.callback_completed += 1

    def _sample_cpu(self):
        now = time.monotonic()
        process = time.process_time()
        elapsed = now - self._last_cpu_wall
        if elapsed < 1.0:
            return
        self.cpu_samples.append({
            'wall_monotonic_s': now,
            'process_cpu_percent_one_core': (
                100.0 * (process - self._last_cpu_process) / elapsed),
        })
        self.cpu_samples = self.cpu_samples[-256:]
        self._last_cpu_wall = now
        self._last_cpu_process = process

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))

    def _update_confirmation_cadence(self, stamps, stamp_ns):
        """Track message-clock cadence without using wall time as evidence."""
        stamp_ns = int(stamp_ns)
        if stamps and stamp_ns <= stamps[-1]:
            return
        if stamps:
            interval = stamp_ns - stamps[-1]
            if interval > 0:
                stamps.append(stamp_ns)
        else:
            stamps.append(stamp_ns)
        intervals = []
        for values in (self.own_descriptor_stamps_ns,
                       self.peer_descriptor_stamps_ns):
            intervals.extend(
                right - left for left, right in zip(values, list(values)[1:])
                if right > left)
        self.effective_confirmation_window_ns = (
            confirmation_window_for_cadence(
                self.confirmation_window_ns, intervals,
                self.confirmation_cadence_factor))

    def _ros_time_s(self) -> float:
        """Return the configured ROS clock in seconds for evidence pacing.

        The frontend's map and snapshot messages carry ROS/simulation stamps,
        so their cadence gates must use this same clock.  Wall monotonic time
        remains appropriate for process/worker diagnostics, but not for
        deciding when a new evidence sample is due.
        """
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, message):
        self.counters['map_messages_received'] += 1
        if self.first_map_wall is None:
            self.first_map_wall = time.monotonic()
        self.latest_map = message
        self.latest_map_fingerprint = self._map_fingerprint(message)
        self.map_revision += 1
        if (self.full_map_registration and
                not getattr(self, '_post_handoff_quiesced', False)):
            self._store_local_full_map_snapshot()
            self._publish_full_map_snapshot_if_due()
        now_ros = self._ros_time_s()
        export_due = (
            self.last_export_ros_s <= 0.0 or
            now_ros - self.last_export_ros_s >= self.peer_map_publish_period_s
        )
        if (self.accepted is not None and export_due and
                self.latest_map_fingerprint != self.last_export_map_fingerprint):
            self.publish_local_map()

    @staticmethod
    def _full_map_crop(message):
        """Make an immutable, metric-preserving GridCrop from an OccupancyGrid."""
        info = message.info
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (int(info.height), int(info.width))).copy()
        values.setflags(write=False)
        return GridCrop(
            values=values,
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=UnknownPoseFrontend._yaw(info.origin.orientation))

    @staticmethod
    def _full_map_snapshot_id(robot_id, revision, fingerprint):
        return f'{robot_id}-full-map-{int(revision):08d}-{fingerprint[:12]}'

    def _make_full_map_snapshot(self, message, robot_id, revision, fingerprint):
        crop = self._full_map_crop(message)
        known = int(np.count_nonzero(crop.values >= 0))
        occupied = int(np.count_nonzero(crop.values >= 50))
        free = int(np.count_nonzero((crop.values >= 0) &
                                    (crop.values < 50)))
        snapshot_id = self._full_map_snapshot_id(
            robot_id, revision, fingerprint)
        return {
            'id': snapshot_id,
            'robot_id': str(robot_id),
            'revision': int(revision),
            'timestamp_ns': self._stamp_ns(message),
            'fingerprint': str(fingerprint),
            'frame_id': str(message.header.frame_id),
            'crop': crop,
            'width': int(crop.values.shape[1]),
            'height': int(crop.values.shape[0]),
            'resolution': float(crop.resolution),
            'origin_x': float(crop.origin_x),
            'origin_y': float(crop.origin_y),
            'origin_yaw': float(crop.origin_yaw),
            'known_cells': known,
            'occupied_cells': occupied,
            'free_cells': free,
        }

    def _store_local_full_map_snapshot(self):
        if (getattr(self, '_post_handoff_quiesced', False) or
                self.latest_map is None or not self.latest_map_fingerprint):
            return None
        snapshot = self._make_full_map_snapshot(
            self.latest_map, self.robot_id, self.map_revision,
            self.latest_map_fingerprint)
        self.latest_full_map_snapshot = snapshot
        self.local_full_map_snapshots[snapshot['id']] = snapshot
        self._trim_full_map_cache(self.local_full_map_snapshots)
        return snapshot

    def _trim_full_map_cache(self, cache):
        """Evict only unpinned full-map snapshots from a bounded cache."""
        while len(cache) > self.full_map_max_snapshots:
            victim = next((key for key in cache
                           if key not in self._full_map_proposal_pins), None)
            if victim is None:
                break
            cache.pop(victim, None)

    def peer_full_map_callback(self, message):
        if (getattr(self, '_post_handoff_quiesced', False) or
                not self.full_map_registration or
                str(message.source_robot_id) != self.peer_robot_id):
            return
        grid = message.occupancy_grid
        fingerprint = self._map_fingerprint(grid)
        snapshot = self._make_full_map_snapshot(
            grid, self.peer_robot_id, int(message.revision), fingerprint)
        self.peer_full_map_snapshots[snapshot['id']] = snapshot
        self._trim_full_map_cache(self.peer_full_map_snapshots)
        self._record_diagnostic_event(
            'FULL_MAP_SNAPSHOT_RECEIVED', snapshot_id=snapshot['id'],
            revision=snapshot['revision'], timestamp_ns=snapshot['timestamp_ns'],
            width=snapshot['width'], height=snapshot['height'],
            known_cells=snapshot['known_cells'],
            occupied_cells=snapshot['occupied_cells'])
        self._resume_pending_full_map_verification()

    def _publish_full_map_snapshot_if_due(self, force=False):
        if (getattr(self, '_post_handoff_quiesced', False) or
                self.latest_full_map_snapshot is None):
            return False
        now = self._ros_time_s()
        snapshot = self.latest_full_map_snapshot
        if (not force and
                now - self._last_full_map_export_ros_s <
                self.full_map_registration_period_s):
            return False
        if (not force and
                getattr(self, '_last_full_map_export_fingerprint', None) ==
                snapshot['fingerprint']):
            return False
        message = PeerMap()
        message.source_robot_id = self.robot_id
        message.revision = snapshot['revision']
        message.export_stamp = self.get_clock().now().to_msg()
        message.local_evidence_only = True
        message.occupancy_grid = self.latest_map
        self.peer_map_pub.publish(message)
        self._last_full_map_export_ros_s = now
        self._last_full_map_export_fingerprint = snapshot['fingerprint']
        self.counters['peer_maps_published'] += 1
        self._record_diagnostic_event(
            'FULL_MAP_SNAPSHOT_PUBLISHED', snapshot_id=snapshot['id'],
            revision=snapshot['revision'], timestamp_ns=snapshot['timestamp_ns'],
            width=snapshot['width'], height=snapshot['height'],
            known_cells=snapshot['known_cells'],
            occupied_cells=snapshot['occupied_cells'])
        return True

    @staticmethod
    def _full_map_proposal_snapshot_ids(message):
        source_ids = [str(value) for value in getattr(
            message, 'evidence_source_keyframe_ids', [])]
        target_ids = [str(value) for value in getattr(
            message, 'evidence_target_keyframe_ids', [])]
        return source_ids, target_ids

    def _full_map_snapshot_owner(self, snapshot_id):
        for robot_id in (self.robot_id, self.peer_robot_id):
            if str(snapshot_id).startswith(f'{robot_id}-full-map-'):
                return robot_id
        return ''

    def _find_full_map_snapshot(self, snapshot_id):
        snapshot_id = str(snapshot_id)
        snapshot = self.local_full_map_snapshots.get(snapshot_id)
        if snapshot is not None:
            return snapshot
        return self.peer_full_map_snapshots.get(snapshot_id)

    def _full_map_missing_snapshot_ids(self, message):
        source_ids, target_ids = self._full_map_proposal_snapshot_ids(message)
        required = list(source_ids) + list(target_ids)
        missing = []
        for snapshot_id in dict.fromkeys(required):
            if self._find_full_map_snapshot(snapshot_id) is None:
                missing.append(snapshot_id)
                self.counters['full_map_evidence_cache_misses'] += 1
                self._record_diagnostic_event(
                    'FULL_MAP_EVIDENCE_CACHE_MISS', snapshot_id=snapshot_id)
            else:
                self.counters['full_map_evidence_cache_hits'] += 1
        return missing

    def _request_full_map_snapshots(self, snapshot_ids):
        request_ids = [str(value) for value in dict.fromkeys(snapshot_ids)
                       if str(value) not in self._requested_full_map_snapshot_ids]
        if not request_ids:
            return False
        owners = {self._full_map_snapshot_owner(value) for value in request_ids}
        owners.discard(self.robot_id)
        owners.discard('')
        if not owners:
            return False
        # This frontend has one peer.  Keep the owner field explicit so the
        # request remains unambiguous if the transport is later extended.
        owner = self.peer_robot_id
        request_ids = [value for value in request_ids
                       if self._full_map_snapshot_owner(value) == owner]
        if not request_ids:
            return False
        request = FullMapSnapshotRequest()
        request.requester_robot_id = self.robot_id
        request.owner_robot_id = owner
        request.snapshot_ids = request_ids
        self.full_map_snapshot_request_pub.publish(request)
        self._requested_full_map_snapshot_ids.update(request_ids)
        self.counters['full_map_evidence_requests'] += len(request_ids)
        self._record_diagnostic_event(
            'FULL_MAP_EVIDENCE_REQUEST', owner_robot_id=owner,
            snapshot_ids=request_ids)
        return True

    def _snapshot_response(self, snapshot_id):
        response = FullMapSnapshotResponse()
        response.owner_robot_id = self.robot_id
        response.snapshot_id = str(snapshot_id)
        snapshot = self.local_full_map_snapshots.get(str(snapshot_id))
        if snapshot is None:
            response.available = False
            response.reason = 'SNAPSHOT_NOT_AVAILABLE'
            return response, 0
        response.available = True
        response.timestamp_ns = int(snapshot['timestamp_ns'])
        response.revision = int(snapshot['revision'])
        response.frame_id = str(snapshot['frame_id'])
        response.width = int(snapshot['width'])
        response.height = int(snapshot['height'])
        response.resolution = float(snapshot['resolution'])
        response.origin_x = float(snapshot['origin_x'])
        response.origin_y = float(snapshot['origin_y'])
        response.origin_yaw = float(snapshot['origin_yaw'])
        values = np.asarray(snapshot['crop'].values, dtype=np.int16).reshape(-1)
        response.occupancy_data = [int(value) for value in values]
        response.known_cells = int(snapshot['known_cells'])
        response.occupied_cells = int(snapshot['occupied_cells'])
        response.free_cells = int(snapshot['free_cells'])
        viewpoint = snapshot.get('viewpoint')
        response.viewpoint_available = viewpoint is not None
        if viewpoint is not None:
            response.viewpoint_x = float(viewpoint[0])
            response.viewpoint_y = float(viewpoint[1])
            response.viewpoint_yaw = float(viewpoint[2])
        return response, len(response.occupancy_data)

    def full_map_snapshot_request_callback(self, message):
        if (getattr(self, '_post_handoff_quiesced', False) or
                not self.full_map_registration or
                str(message.requester_robot_id) != self.peer_robot_id or
                str(message.owner_robot_id) != self.robot_id):
            return
        for snapshot_id in dict.fromkeys(
                str(value) for value in getattr(message, 'snapshot_ids', [])):
            response, cell_count = self._snapshot_response(snapshot_id)
            self.full_map_snapshot_response_pub.publish(response)
            self.counters['full_map_evidence_responses'] += 1
            self._record_diagnostic_event(
                'FULL_MAP_EVIDENCE_RESPONSE', snapshot_id=snapshot_id,
                available=bool(response.available), cell_count=cell_count,
                reason=str(response.reason))

    def _cache_full_map_snapshot_response(self, message):
        snapshot_id = str(message.snapshot_id)
        if not bool(message.available):
            self._unavailable_full_map_snapshot_ids.add(snapshot_id)
            self._requested_full_map_snapshot_ids.discard(snapshot_id)
            return False
        values = np.asarray(message.occupancy_data, dtype=np.int16)
        expected = int(message.width) * int(message.height)
        if (int(message.width) <= 0 or int(message.height) <= 0 or
                values.size != expected):
            self._unavailable_full_map_snapshot_ids.add(snapshot_id)
            return False
        values = values.reshape((int(message.height), int(message.width))).copy()
        values.setflags(write=False)
        crop = GridCrop(
            values=values, resolution=float(message.resolution),
            origin_x=float(message.origin_x), origin_y=float(message.origin_y),
            origin_yaw=float(message.origin_yaw))
        snapshot = {
            'id': snapshot_id,
            'robot_id': str(message.owner_robot_id),
            'revision': int(message.revision),
            'timestamp_ns': int(message.timestamp_ns),
            'fingerprint': hashlib.sha256(values.tobytes()).hexdigest(),
            'frame_id': str(message.frame_id),
            'crop': crop,
            'width': int(message.width), 'height': int(message.height),
            'resolution': float(message.resolution),
            'origin_x': float(message.origin_x),
            'origin_y': float(message.origin_y),
            'origin_yaw': float(message.origin_yaw),
            'known_cells': int(message.known_cells),
            'occupied_cells': int(message.occupied_cells),
            'free_cells': int(message.free_cells),
        }
        self.peer_full_map_snapshots[snapshot_id] = snapshot
        self._trim_full_map_cache(self.peer_full_map_snapshots)
        self._requested_full_map_snapshot_ids.discard(snapshot_id)
        self._record_diagnostic_event(
            'FULL_MAP_EVIDENCE_CACHE_HIT', snapshot_id=snapshot_id,
            owner_robot_id=str(message.owner_robot_id),
            width=int(message.width), height=int(message.height))
        return True

    def full_map_snapshot_response_callback(self, message):
        if (getattr(self, '_post_handoff_quiesced', False) or
                not self.full_map_registration or
                str(message.owner_robot_id) != self.peer_robot_id):
            return
        if bool(message.available):
            self._cache_full_map_snapshot_response(message)
        else:
            self._unavailable_full_map_snapshot_ids.add(str(message.snapshot_id))
            self._requested_full_map_snapshot_ids.discard(str(message.snapshot_id))
        self._resume_pending_full_map_verification()

    @staticmethod
    def _map_fingerprint(message):
        """Return a compact content/geometry identity for a map export.

        The fingerprint is metadata plus occupancy bytes, not a retained map
        copy.  It lets the peer-map publisher suppress identical DDS samples
        while still exporting origin, dimensions, and occupancy changes.
        """
        info = message.info
        origin = info.origin
        orientation = origin.orientation
        metadata = (
            int(info.width), int(info.height), float(info.resolution),
            float(origin.position.x), float(origin.position.y),
            float(origin.position.z), float(orientation.x),
            float(orientation.y), float(orientation.z), float(orientation.w),
        )
        values = np.asarray(message.data, dtype=np.int8)
        digest = hashlib.blake2b(digest_size=16)
        digest.update(repr(metadata).encode('ascii'))
        digest.update(values.tobytes())
        return digest.hexdigest()

    def _map_crop(self):
        if self.latest_map is None:
            return None
        message = self.latest_map
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (message.info.height, message.info.width))
        center_x = center_y = None
        self._latest_observation_pose = None
        try:
            transform = self.tf_buffer.lookup_transform(
                message.header.frame_id,
                f'{self.robot_id}/base_footprint',
                rclpy.time.Time(), timeout=Duration(seconds=0.02))
            center_x = transform.transform.translation.x
            center_y = transform.transform.translation.y
            self._latest_observation_pose = (
                float(center_x), float(center_y),
                float(self._yaw(transform.transform.rotation)))
        except TransformException:
            pass
        origin_yaw = self._yaw(message.info.origin.orientation)
        return crop_grid(
            values, float(message.info.resolution),
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            center_x=center_x, center_y=center_y,
            size_m=self.crop_size_m, origin_yaw=origin_yaw)

    def _allocate_keyframe_id(self):
        """Allocate an identity for each materially sampled descriptor.

        ``map_revision`` remains the map epoch, but it is not a keyframe
        identity: with a slower SLAM publication interval the robot can move
        and produce new spatial views between two map revisions.
        """
        self.keyframe_sequence += 1
        return f'{self.robot_id}-{self.keyframe_sequence:08d}'

    @staticmethod
    def _crop_content_identity(crop):
        """Identify exact occupancy content independently of keyframe IDs."""
        values = np.asarray(crop.values, dtype=np.int16)
        digest = hashlib.blake2b(digest_size=16)
        digest.update(str(tuple(values.shape)).encode('ascii'))
        digest.update(repr((float(crop.resolution),
                            float(crop.origin_yaw))).encode('ascii'))
        digest.update(np.ascontiguousarray(values).tobytes())
        return (tuple(values.shape), float(crop.resolution),
                float(crop.origin_yaw), digest.digest())

    @staticmethod
    def _pose_has_novelty(current, previous, translation_threshold_m,
                          rotation_threshold_rad):
        """Test Euclidean translation between two local-odometry poses.

        The rotation argument is retained for compatibility with the earlier
        descriptor trigger, but rotation is intentionally not sufficient for
        an independent evidence viewpoint.
        """
        if current is None or previous is None:
            return False
        distance = math.hypot(float(current[0]) - float(previous[0]),
                              float(current[1]) - float(previous[1]))
        return distance >= float(translation_threshold_m)

    def _evidence_keyframe_admission(self, crop, observation_pose):
        """Return ``(admit, reason, displacement_m)`` for one crop.

        The first usable crop is retained immediately.  Every subsequent
        evidence crop must be separated from the last retained evidence
        viewpoint by the configured Euclidean local-odometry translation.
        Map revision, elapsed time, checksum novelty, and in-place rotation
        cannot bypass this physical-spacing rule.
        """
        identity = self._crop_content_identity(crop)
        history = getattr(self, 'keyframe_content_history', {})
        if not getattr(self, 'keyframes', {}) and not history:
            return True, 'FIRST_EVIDENCE_KEYFRAME', None
        previous_viewpoint = getattr(self, '_last_evidence_viewpoint', None)
        # A bounded-cache restoration/test fixture may retain only the
        # content-history pose.  It is still the last known physical pose for
        # that exact crop content.
        if previous_viewpoint is None and identity in history:
            previous_viewpoint = history[identity]
        # Descriptor/crop retention is allowed while TF is temporarily
        # unavailable.  Such a record is not a physical-spacing baseline,
        # and must never poison the first later pose-bearing observation.
        if observation_pose is None:
            if identity in history:
                return False, 'DUPLICATE_CONTENT', None
            return True, 'NO_PHYSICAL_VIEWPOINT_RETAINED', None
        if previous_viewpoint is None:
            return True, 'FIRST_POSE_BEARING_EVIDENCE_KEYFRAME', None
        displacement = math.hypot(
            float(observation_pose[0]) - float(previous_viewpoint[0]),
            float(observation_pose[1]) - float(previous_viewpoint[1]))
        threshold = float(getattr(
            self, 'evidence_keyframe_translation_threshold_m',
            getattr(self, 'verification_novelty_spacing_m', 0.80)))
        if displacement < threshold:
            reason = ('DUPLICATE_PHYSICAL_VIEW' if identity in history else
                      'INSUFFICIENT_TRANSLATION')
            return False, reason, displacement
        reason = ('TRANSLATION_NOVELTY_DUPLICATE_CONTENT' if identity in history
                  else 'TRANSLATION_NOVELTY')
        return True, reason, displacement

    def _should_publish_keyframe(self, crop, observation_pose):
        """Compatibility boolean for physical evidence-keyframe admission."""
        admitted, _, _ = self._evidence_keyframe_admission(
            crop, observation_pose)
        if admitted and (getattr(self, 'keyframes', {}) or
                         getattr(self, 'keyframe_content_history', {})):
            self.counters['keyframe_motion_novelty_admitted'] += 1
        return admitted

    def publish_descriptor(self):
        crop = self._map_crop()
        if crop is None:
            return
        admitted, admission_reason, displacement = (
            self._evidence_keyframe_admission(
                crop, self._latest_observation_pose))
        if not admitted:
            self.counters['keyframe_content_duplicates_suppressed'] += 1
            self._record_diagnostic_event(
                'EVIDENCE_KEYFRAME_SUPPRESSED',
                would_be_keyframe_id=(
                    f'{self.robot_id}-{self.keyframe_sequence + 1:08d}'),
                timestamp_ns=int(self.get_clock().now().nanoseconds),
                reason=admission_reason,
                displacement_from_last_m=displacement,
                threshold_m=float(getattr(
                    self, 'evidence_keyframe_translation_threshold_m',
                    getattr(self, 'verification_novelty_spacing_m', 0.80))))
            self._record_diagnostic_event(
                'KEYFRAME_SUPPRESSED_DUPLICATE_CONTENT',
                content_identity=list(self._crop_content_identity(crop)[:3]))
            self.last_descriptor_wall = time.monotonic()
            return
        if self.keyframes:
            self.counters['keyframe_motion_novelty_admitted'] += 1
        # GridCrop is frozen but its ndarray is not.  Store a private copy and
        # make it read-only so a later map callback cannot mutate an advertised
        # keyframe's registration input.
        immutable_values = np.array(crop.values, dtype=np.int16, copy=True)
        immutable_values.setflags(write=False)
        crop = GridCrop(
            values=immutable_values, resolution=float(crop.resolution),
            origin_x=float(crop.origin_x), origin_y=float(crop.origin_y),
            origin_yaw=float(crop.origin_yaw))
        descriptor = polar_descriptor(crop)
        self.descriptor_bytes = len(descriptor)
        keyframe_id = self._allocate_keyframe_id()
        checksum = descriptor_checksum(descriptor)
        message = LocalMapDescriptor()
        message.header.frame_id = self.latest_map.header.frame_id
        # The map epoch identifies the source map revision.  The descriptor
        # timestamp identifies when this crop/keyframe was actually sampled,
        # which remains meaningful when SLAM publishes maps less frequently.
        message.header.stamp = self.get_clock().now().to_msg()
        message.source_robot_id = self.robot_id
        message.keyframe_id = keyframe_id
        message.map_epoch = self.map_revision
        message.resolution = float(crop.resolution)
        message.ring_count = 12
        message.sector_count = 24
        message.descriptor_version = 1
        message.descriptor_bytes = list(descriptor)
        message.crop_origin_x = float(crop.origin_x)
        message.crop_origin_y = float(crop.origin_y)
        message.crop_origin_yaw = float(crop.origin_yaw)
        message.crop_width = int(crop.values.shape[1])
        message.crop_height = int(crop.values.shape[0])
        message.checksum = checksum
        # Carry only intrinsic crop-quality metadata in the cheap descriptor.
        # Descriptors remain published and retained immediately; these fields
        # let the scheduler avoid an expensive crop request when an endpoint
        # is already known to fail the unchanged consensus maturity policy.
        known_fraction = float(np.count_nonzero(crop.values >= 0) /
                               crop.values.size) if crop.values.size else 0.0
        occupied_cells = int(np.count_nonzero(crop.values >= 50))
        message.crop_known_fraction = known_fraction
        message.crop_occupied_cells = occupied_cells
        viewpoint = self._latest_observation_pose
        message.viewpoint_available = viewpoint is not None
        if viewpoint is not None:
            message.viewpoint_x = float(viewpoint[0])
            message.viewpoint_y = float(viewpoint[1])
            message.viewpoint_yaw = float(viewpoint[2])
        self._update_confirmation_cadence(
            self.own_descriptor_stamps_ns, self._stamp_ns(message))
        self.descriptor_pub.publish(message)
        self.counters['descriptors_published'] += 1
        self._record_diagnostic_event(
            'DESCRIPTOR_PUBLISHED', keyframe_id=keyframe_id,
            map_revision=self.map_revision, descriptor_bytes=len(descriptor))
        if self._last_evidence_viewpoint is None or displacement is None:
            self._evidence_cumulative_travel_m = 0.0
        else:
            self._evidence_cumulative_travel_m += float(displacement)
        content_identity = self._crop_content_identity(crop)
        crop_center = (
            float(crop.origin_x + 0.5 * crop.values.shape[1] * crop.resolution),
            float(crop.origin_y + 0.5 * crop.values.shape[0] * crop.resolution))
        self._record_diagnostic_event(
            'EVIDENCE_KEYFRAME_RETAINED', keyframe_id=keyframe_id,
            timestamp_ns=self._stamp_ns(message),
            local_odom=(None if self._latest_observation_pose is None else
                        [float(value) for value in self._latest_observation_pose]),
            displacement_from_last_m=displacement,
            cumulative_travel_m=float(self._evidence_cumulative_travel_m),
            crop_center=[float(value) for value in crop_center],
            content_hash=content_identity[3].hex(),
            reason_retained=admission_reason)
        self.keyframe_content_history[
            content_identity] = self._latest_observation_pose
        # Only a real local physical pose can establish or advance the
        # spacing baseline.  A pose-less startup crop remains usable for
        # descriptor/crop exchange but is not an evidence viewpoint.
        if self._latest_observation_pose is not None:
            self._last_evidence_viewpoint = tuple(
                float(value) for value in self._latest_observation_pose)
        while len(self.keyframe_content_history) > self.max_keyframes:
            self.keyframe_content_history.popitem(last=False)
        self.keyframes[keyframe_id] = (message, crop)
        if self._latest_observation_pose is not None:
            self.keyframe_viewpoints[keyframe_id] = tuple(
                float(value) for value in self._latest_observation_pose)
        while len(self.keyframes) > self.max_keyframes:
            expired_key, _ = self.keyframes.popitem(last=False)
            self.keyframe_viewpoints.pop(expired_key, None)
        self._prune_expired_descriptor_state()
        self.last_descriptor_wall = time.monotonic()
        self.pending_own_descriptor_keys.append(keyframe_id)

    def _full_map_pair_mature(self, local, peer):
        local_ok, local_details = consensus_crop_maturity(
            local['crop'], self.consensus_min_known_fraction,
            self.consensus_min_occupied_cells)
        peer_ok, peer_details = consensus_crop_maturity(
            peer['crop'], self.consensus_min_known_fraction,
            self.consensus_min_occupied_cells)
        return bool(local_ok and peer_ok), {
            'local': local_details, 'peer': peer_details}

    @staticmethod
    def _full_map_pair_operable(source, target):
        """Check only the minimum safe input validity for startup matching."""
        details = {}
        for name, snapshot in (('source', source), ('target', target)):
            crop = snapshot.get('crop') if snapshot else None
            values = None if crop is None else np.asarray(crop.values)
            occupied = int(snapshot.get('occupied_cells', 0))
            valid = bool(
                values is not None and values.ndim == 2 and
                values.size > 0 and float(crop.resolution) > 0.0 and
                occupied >= 12)
            details[name] = {
                'operable': valid,
                'known_cells': int(snapshot.get('known_cells', 0)),
                'occupied_cells': occupied,
                'width': (int(values.shape[1]) if values is not None and
                          values.ndim == 2 else 0),
                'height': (int(values.shape[0]) if values is not None and
                           values.ndim == 2 else 0),
            }
        return bool(details['source']['operable'] and
                    details['target']['operable']), details

    def _capture_full_map_attempt(self, source, target, sequence):
        if (self.registration_input_capture_output is None or
                sequence > self.registration_input_capture_max_pairs):
            return None
        try:
            root = (self.registration_input_capture_output /
                    'full_maps' / self.robot_id)
            root.mkdir(parents=True, exist_ok=True)
            stem = f'{self.robot_id}_full_map_registration_{sequence:04d}'
            npz_path = root / f'{stem}.npz'
            json_path = root / f'{stem}.json'
            np.savez_compressed(
                npz_path,
                source_values=np.asarray(source['crop'].values,
                                         dtype=np.int16).copy(),
                target_values=np.asarray(target['crop'].values,
                                         dtype=np.int16).copy())
            metadata = {
                'schema_version': 'full_map_registration_capture_1.0',
                'status': 'SUBMITTED', 'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'source_robot_id': source['robot_id'],
                'target_robot_id': target['robot_id'],
                'source_snapshot_id': source['id'],
                'target_snapshot_id': target['id'],
                'source_timestamp_ns': int(source['timestamp_ns']),
                'target_timestamp_ns': int(target['timestamp_ns']),
                'source_revision': int(source['revision']),
                'target_revision': int(target['revision']),
                'source_fingerprint': source['fingerprint'],
                'target_fingerprint': target['fingerprint'],
                'source_frame': str(source.get('frame_id',
                                               source['robot_id'] + '/map')),
                'target_frame': str(target.get('frame_id',
                                              target['robot_id'] + '/map')),
                'source_to_target_convention':
                    'p_target = R(yaw) * p_source + translation',
                'source_map': {key: source[key] for key in (
                    'width', 'height', 'resolution', 'origin_x',
                    'origin_y', 'origin_yaw', 'known_cells',
                    'occupied_cells', 'free_cells')},
                'target_map': {key: target[key] for key in (
                    'width', 'height', 'resolution', 'origin_x',
                    'origin_y', 'origin_yaw', 'known_cells',
                    'occupied_cells', 'free_cells')},
                'npz_file': npz_path.name,
            }
            json_path.write_text(json.dumps(metadata, indent=2,
                                            sort_keys=True), encoding='utf-8')
            return json_path
        except Exception as exc:
            self.get_logger().warning('full-map capture failed: %s', exc)
            return None

    def _full_map_mode_result(self, result, local, peer):
        """Return a result already computed in canonical R1->R2 order."""
        del local, peer
        return result

    @staticmethod
    def _run_full_map_registration(source, target, initial_transform=None):
        """Global-register once, or locally verify an already proposed basin."""
        if initial_transform is None:
            global_result = register_crops(
                source, target, minimum_agreement=0.0)
            seed = global_result.transform
            if not all(math.isfinite(float(value)) for value in seed):
                return global_result
        else:
            global_result = None
            seed = tuple(float(value) for value in initial_transform)
        refined = refine_registration_locally(
            source, target, seed, translation_bound_m=0.18,
            yaw_bound_rad=math.radians(1.0))
        diagnostics = tuple(refined.consensus_diagnostics)
        if global_result is not None:
            diagnostics += ({
                'kind': 'full_map_global_seed',
                'transform': [float(value) for value in global_result.transform],
                'accepted': bool(global_result.accepted),
                'residual_m': float(global_result.residual_m),
            },)
        return replace(refined, consensus_diagnostics=diagnostics)

    @staticmethod
    def _stationary_witness_evidence_id(source, target, partition):
        suffix = str(partition.support_hash)[:32]
        return (f"{source['id']}-witness-{suffix}|"
                f"{target['id']}-witness-{suffix}")

    @staticmethod
    def _stationary_support_baseline(partitions):
        points = [partition.target_centroid for partition in partitions]
        if len(points) < 2:
            return 0.0
        return float(max(
            math.hypot(left[0] - right[0], left[1] - right[1])
            for index, left in enumerate(points)
            for right in points[index + 1:]))

    @staticmethod
    def _run_stationary_full_map_registration(source, target,
                                              source_snapshot_id='',
                                              target_snapshot_id=''):
        """Find three real stationary witnesses using unchanged gates.

        The whole-map fit is retained as the immutable seed.  Candidate
        sectors are deterministic and disjoint; every sector is independently
        registered, then the existing family selector is the sole acceptance
        decision.  A family is rejected if its actual support does not span
        the existing 0.75 m physical diversity floor, even though the legacy
        full-map path historically passed zero viewpoint metadata.
        """
        whole = UnknownPoseFrontend._run_full_map_registration(source, target)
        if not whole.accepted:
            return whole, ()
        for left_fraction, right_fraction, partitions in \
                iter_stationary_witness_partitions(source, target,
                                                    whole.transform):
            if not stationary_witness_supports_disjoint(partitions):
                continue
            support_baseline = UnknownPoseFrontend._stationary_support_baseline(
                partitions)
            if support_baseline < 0.75:
                continue
            results = [reestimate_registration_from_seed(
                partition.source, partition.target, whole.transform,
                minimum_agreement=0.0) for partition in partitions]
            source_record = {'id': str(source_snapshot_id)}
            target_record = {'id': str(target_snapshot_id)}
            evidence_ids = [UnknownPoseFrontend._stationary_witness_evidence_id(
                source_record, target_record, partition)
                for partition in partitions]
            family = select_hypothesis_family(
                [(partition.source, partition.target)
                 for partition in partitions],
                [(result,) for result in results],
                target_map_radius_m=40.0,
                min_consistent_constraints=3,
                # Keep the production full-map selector's setting.  The
                # independent physical-support floor is enforced above.
                min_spatial_baseline_m=0.0,
                max_translation_consistency_m=0.15,
                max_yaw_consistency_rad=math.radians(1.0),
                max_projected_registration_error_m=0.20,
                minimum_agreement=0.0,
                evidence_ids=evidence_ids)
            if not family.accepted:
                continue
            diagnostics = tuple(family.consensus_diagnostics) + ({
                'kind': 'stationary_witness_family',
                'cut_fractions': [float(left_fraction),
                                  float(right_fraction)],
                'support_baseline_m': float(support_baseline),
                'witnesses': [{
                    'index': int(partition.index),
                    'evidence_id': evidence_id,
                    'support_hash': str(partition.support_hash),
                    'source_support_count': int(
                        partition.source_support_count),
                    'target_support_count': int(
                        partition.target_support_count),
                    'source_bbox': list(partition.source_bbox),
                    'target_bbox': list(partition.target_bbox),
                    'source_centroid': list(partition.source_centroid),
                    'target_centroid': list(partition.target_centroid),
                    'source_extent_m': list(partition.source_extent_m),
                    'target_extent_m': list(partition.target_extent_m),
                    'individual_transform': [float(value)
                                             for value in results[index].transform],
                    'individual_inlier_ratio': float(results[index].inlier_ratio),
                    'individual_residual_m': float(results[index].residual_m),
                    'individual_agreement': float(
                        results[index].occupied_free_agreement),
                    'individual_overlap': float(results[index].overlap_fraction),
                } for index, (partition, evidence_id) in enumerate(
                    zip(partitions, evidence_ids))],
            }, {
                'kind': 'stationary_witness_canonical_seed',
                'scheme_version': STATIONARY_WITNESS_SCHEME_VERSION,
                'transform': [float(value) for value in whole.transform],
                'source_snapshot_id': str(source_snapshot_id),
                'target_snapshot_id': str(target_snapshot_id),
            },)
            return replace(family, consensus_diagnostics=diagnostics), tuple(
                (partition, result, evidence_id)
                for partition, result, evidence_id in zip(
                    partitions, results, evidence_ids))
        return replace(
            whole, accepted=False,
            reason='INSUFFICIENT_STATIONARY_WITNESSES',
            constraint_count=0, consistent_constraint_count=0,
            consensus_diagnostics=tuple(whole.consensus_diagnostics) + ({
                'kind': 'stationary_witness_search',
                'reason': 'NO_ACCEPTED_DISJOINT_THREE_MEMBER_FAMILY',
            },)), ()

    @staticmethod
    def _derived_stationary_snapshot(base, crop, support_hash, index):
        """Materialize an immutable witness under the existing owner prefix."""
        digest = hashlib.sha256()
        digest.update(np.asarray(crop.values, dtype=np.int16).tobytes())
        digest.update(repr((crop.resolution, crop.origin_x, crop.origin_y,
                            crop.origin_yaw)).encode('ascii'))
        fingerprint = digest.hexdigest()
        snapshot_id = (f"{base['id']}-witness-{str(support_hash)[:32]}")
        return {
            'id': snapshot_id,
            'robot_id': str(base['robot_id']),
            'revision': int(base['revision']),
            'timestamp_ns': int(base['timestamp_ns']),
            'fingerprint': fingerprint,
            'frame_id': str(base['frame_id']),
            'crop': crop,
            'width': int(crop.values.shape[1]),
            'height': int(crop.values.shape[0]),
            'resolution': float(crop.resolution),
            'origin_x': float(crop.origin_x),
            'origin_y': float(crop.origin_y),
            'origin_yaw': float(crop.origin_yaw),
            'known_cells': int(np.count_nonzero(crop.values >= 0)),
            'occupied_cells': int(np.count_nonzero(crop.values >= 50)),
            'free_cells': int(np.count_nonzero(
                (crop.values >= 0) & (crop.values < 50))),
            'stationary_witness_index': int(index),
            'stationary_support_hash': str(support_hash),
            'stationary_base_snapshot_id': str(base['id']),
        }

    def _materialize_stationary_witness_snapshots(self, source, target,
                                                   partitions):
        """Cache exact derived crops on both owner sides for peer replay."""
        records = []
        for partition, result, evidence_id in partitions:
            source_snapshot = self._derived_stationary_snapshot(
                source, partition.source, partition.support_hash,
                partition.index)
            target_snapshot = self._derived_stationary_snapshot(
                target, partition.target, partition.support_hash,
                partition.index)
            for snapshot in (source_snapshot, target_snapshot):
                cache = (self.local_full_map_snapshots
                         if snapshot['robot_id'] == self.robot_id else
                         self.peer_full_map_snapshots)
                cache[snapshot['id']] = snapshot
                self._trim_full_map_cache(cache)
            records.append({
                'evidence_id': str(evidence_id),
                'source': source_snapshot,
                'target': target_snapshot,
                'base_source': source,
                'base_target': target,
                'result': result,
                'partition': partition,
            })
        return tuple(records)

    def _ensure_stationary_witness_snapshots_for_proposal(self, proposal):
        source_id = str(getattr(proposal, 'source_keyframe_id', ''))
        target_id = str(getattr(proposal, 'target_keyframe_id', ''))
        source = self._find_full_map_snapshot(source_id)
        target = self._find_full_map_snapshot(target_id)
        if source is None or target is None:
            return False
        # A stationary proposal uses the canonical robot1->robot2 envelope;
        # the owner caches may be reversed on robot2, but the IDs are not.
        if str(source['robot_id']) != 'robot1' or \
                str(target['robot_id']) != 'robot2':
            return False
        seed = self._stationary_proposal_seed(proposal)
        if seed is None:
            return False
        if (str(getattr(proposal,
                       'stationary_canonical_source_snapshot_id', '')) !=
                str(source['id']) or
                str(getattr(proposal,
                            'stationary_canonical_target_snapshot_id', '')) !=
                str(target['id']) or
                str(getattr(proposal,
                            'stationary_canonical_source_map_hash', '')) !=
                str(source['fingerprint']) or
                str(getattr(proposal,
                            'stationary_canonical_target_map_hash', '')) !=
                str(target['fingerprint'])):
            return False
        source_ids = [str(value) for value in getattr(
            proposal, 'evidence_source_keyframe_ids', [])]
        target_ids = [str(value) for value in getattr(
            proposal, 'evidence_target_keyframe_ids', [])]
        if len(source_ids) != len(target_ids) or len(source_ids) < 3:
            return False
        # Recompute the same deterministic whole-map basin used by the
        # proposer from the immutable base snapshots.  The accepted family
        # transform is a consensus output and is not a stable partition seed;
        # using it here can move a strip boundary by one cell and change the
        # support hash without changing the physical evidence.
        whole = self._run_full_map_registration(
            source['crop'], target['crop'])
        if not whole.accepted or not self._stationary_seed_compatible(
                seed, whole.transform):
            return False
        for _left, _right, candidate in iter_stationary_witness_partitions(
                source['crop'], target['crop'], seed):
            expected = [self._stationary_witness_evidence_id(
                source, target, partition) for partition in candidate]
            expected_source = [value.split('|', 1)[0] for value in expected]
            expected_target = [value.split('|', 1)[1] for value in expected]
            if (expected_source != source_ids or
                    expected_target != target_ids or
                    not stationary_witness_supports_disjoint(candidate) or
                    self._stationary_support_baseline(candidate) < 0.75):
                continue
            if all(self._find_full_map_snapshot(value) is not None
                   for value in source_ids + target_ids):
                return True
            self._materialize_stationary_witness_snapshots(
                source, target,
                tuple((partition, None, evidence_id)
                      for partition, evidence_id in zip(candidate, expected)))
            return all(self._find_full_map_snapshot(value) is not None
                       for value in source_ids + target_ids)
        return False

    def _full_map_family_result(self):
        if len(self.full_map_confirmation_records) < 3:
            return None
        records = self.full_map_confirmation_records[-8:]
        pairs = [(record['source']['crop'], record['target']['crop'])
                 for record in records]
        hypotheses = [(record['result'],) for record in records]
        evidence_ids = [record['evidence_id'] for record in records]
        timestamps = [(int(record['source']['timestamp_ns']),
                       int(record['target']['timestamp_ns']))
                      for record in records]
        return select_hypothesis_family(
            pairs, hypotheses,
            target_map_radius_m=self.target_map_radius_m,
            min_consistent_constraints=3,
            min_spatial_baseline_m=0.0,
            max_translation_consistency_m=0.15,
            max_yaw_consistency_rad=math.radians(1.0),
            max_projected_registration_error_m=(
                self.max_projected_registration_error_m),
            minimum_agreement=0.0,
            evidence_timestamps=timestamps,
            evidence_ids=evidence_ids)

    def _record_full_map_result(self, local, peer, result, capture_path=None,
                                witness_records=()):
        canonical = result
        source, target = local, peer
        records = list(witness_records)
        if not records:
            evidence_id = f"{source['id']}|{target['id']}"
            records = [{
                'evidence_id': evidence_id, 'source': source, 'target': target,
                'base_source': source, 'base_target': target,
                'result': canonical, 'partition': None,
            }]
        self.full_map_confirmation_records.extend(records)
        self.full_map_confirmation_records = (
            self.full_map_confirmation_records[-12:])
        for record in records:
            witness = record.get('partition')
            individual = record.get('result') or canonical
            fields = {
                'evidence_id': record['evidence_id'],
                'source_snapshot_id': record['source']['id'],
                'target_snapshot_id': record['target']['id'],
                'source_timestamp_ns': record['source']['timestamp_ns'],
                'target_timestamp_ns': record['target']['timestamp_ns'],
                'transform': [float(value) for value in
                              individual.transform],
                'accepted': bool(individual.accepted),
                'residual_m': float(individual.residual_m),
                'inlier_ratio': float(individual.inlier_ratio),
                'reverse_inlier_ratio': float(individual.reverse_inlier_ratio),
                'occupied_free_agreement': float(
                    individual.occupied_free_agreement),
                'overlap_fraction': float(individual.overlap_fraction),
                'confirmation_count': len(self.full_map_confirmation_records),
            }
            if witness is not None:
                fields.update({
                    'stationary_witness': True,
                    'stationary_witness_index': int(witness.index),
                    'stationary_support_hash': str(witness.support_hash),
                    'source_support_count': int(
                        witness.source_support_count),
                    'target_support_count': int(
                        witness.target_support_count),
                    'source_bbox': list(witness.source_bbox),
                    'target_bbox': list(witness.target_bbox),
                    'source_centroid': list(witness.source_centroid),
                    'target_centroid': list(witness.target_centroid),
                    'source_extent_m': list(witness.source_extent_m),
                    'target_extent_m': list(witness.target_extent_m),
                })
            self._record_diagnostic_event(
                'FULL_MAP_REGISTRATION_RESULT', **fields)
        if capture_path is not None:
            try:
                metadata = json.loads(capture_path.read_text(encoding='utf-8'))
                metadata.update({
                    'status': 'ACCEPTED' if canonical.accepted else 'REJECTED',
                    'returned_transform_local': [float(value) for value in
                                                canonical.transform],
                    'returned_transform_canonical': [float(value) for value in
                                                    canonical.transform],
                    'residual_m': float(canonical.residual_m),
                    'forward_inlier_ratio': float(canonical.inlier_ratio),
                    'reverse_inlier_ratio': float(canonical.reverse_inlier_ratio),
                    'occupied_free_agreement': float(
                        canonical.occupied_free_agreement),
                    'overlap_fraction': float(canonical.overlap_fraction),
                    'rejection_reason': '' if canonical.accepted else str(
                        canonical.reason),
                    'stationary_witness_family': [
                        diagnostic for diagnostic in
                        getattr(canonical, 'consensus_diagnostics', ())
                        if diagnostic.get('kind') ==
                        'stationary_witness_family'],
                })
                capture_path.write_text(json.dumps(
                    metadata, indent=2, sort_keys=True), encoding='utf-8')
            except (OSError, ValueError):
                pass

    def _drain_full_map_registration(self):
        if getattr(self, '_post_handoff_quiesced', False):
            return
        future = self._full_map_registration_future
        if future is None or not future.done():
            return
        context = self._full_map_registration_context
        self._full_map_registration_future = None
        self._full_map_registration_context = None
        if context is None:
            return
        source, target, capture_path = context
        try:
            worker_output = future.result()
            if (isinstance(worker_output, tuple) and len(worker_output) == 2
                    and isinstance(worker_output[1], tuple)):
                result, witness_candidates = worker_output
            else:
                result, witness_candidates = worker_output, ()
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self._record_diagnostic_event(
                'FULL_MAP_REGISTRATION_EXCEPTION', exception=repr(exc),
                source_snapshot_id=source['id'], target_snapshot_id=target['id'])
            return
        self.counters['registrations'] += 1
        witness_records = self._materialize_stationary_witness_snapshots(
            source, target, witness_candidates)
        self._record_full_map_result(
            source, target, result, capture_path,
            witness_records=witness_records)
        if not result.accepted or self.full_map_proposal is not None:
            return
        family = result if witness_records else self._full_map_family_result()
        if family is None or not family.accepted:
            return
        self._record_diagnostic_event(
            'FULL_MAP_FAMILY_ACCEPTED',
            consistent_constraint_count=int(
                family.consistent_constraint_count),
            transform=[float(value) for value in family.transform])
        # Full-map startup discovery is decentralized: either robot may have
        # the first valid updated-map family.  The receiver still performs
        # the existing independent verification before handoff.
        self._publish_full_map_proposal(family)

    def _maybe_schedule_full_map_registration(self):
        if (getattr(self, '_post_handoff_quiesced', False) or
                getattr(self, '_registration_shutdown', False) or
                self.accepted is not None or
                self._full_map_registration_future is not None or
                self.latest_full_map_snapshot is None or
                not self.peer_full_map_snapshots):
            return
        now_ros = self._ros_time_s()
        if (now_ros - self._last_full_map_attempt_ros_s <
                self.full_map_registration_period_s):
            return
        local = self.latest_full_map_snapshot
        peer = next(reversed(self.peer_full_map_snapshots.values()))
        if self.robot_id == 'robot1':
            source, target = local, peer
        else:
            source, target = peer, local
        operable, input_details = self._full_map_pair_operable(source, target)
        if not operable:
            self._record_diagnostic_event(
                'FULL_MAP_REGISTRATION_SKIPPED',
                reason='MINIMUM_MAP_INPUT_NOT_OPERABLE',
                source_snapshot_id=source['id'], target_snapshot_id=target['id'],
                input_details=input_details)
            return
        pair_identity = (source['fingerprint'], target['fingerprint'])
        if pair_identity == self._last_full_map_attempt_pair:
            return
        self._last_full_map_attempt_pair = pair_identity
        self._last_full_map_attempt_ros_s = now_ros
        self._full_map_registration_sequence += 1
        capture_path = self._capture_full_map_attempt(
            source, target, self._full_map_registration_sequence)
        self._record_diagnostic_event(
            'FULL_MAP_REGISTRATION_SUBMITTED',
            canonical_source_robot='robot1', canonical_target_robot='robot2',
            source_snapshot_id=source['id'], target_snapshot_id=target['id'],
            source_timestamp_ns=source['timestamp_ns'],
            target_timestamp_ns=target['timestamp_ns'],
            input_details=input_details)
        self._record_diagnostic_event(
            'FULL_MAP_CANONICAL_ORDER', canonical_source_robot='robot1',
            canonical_target_robot='robot2',
            source_snapshot_id=source['id'], target_snapshot_id=target['id'])
        self._full_map_registration_context = (source, target, capture_path)
        self._full_map_registration_future = self._registration_executor.submit(
            self._run_stationary_full_map_registration,
            source['crop'], target['crop'], source['id'], target['id'])

    def _full_map_descriptor(self, snapshot):
        descriptor = LocalMapDescriptor()
        descriptor.header = copy.deepcopy(self.latest_map.header)
        descriptor.header.frame_id = str(snapshot.get(
            'frame_id', snapshot['robot_id'] + '/map'))
        descriptor.header.stamp.sec = int(snapshot['timestamp_ns'] // 1_000_000_000)
        descriptor.header.stamp.nanosec = int(snapshot['timestamp_ns'] %
                                              1_000_000_000)
        descriptor.source_robot_id = snapshot['robot_id']
        descriptor.keyframe_id = snapshot['id']
        descriptor.map_epoch = int(snapshot['revision'])
        descriptor.resolution = float(snapshot['resolution'])
        descriptor.crop_width = int(snapshot['width'])
        descriptor.crop_height = int(snapshot['height'])
        descriptor.crop_origin_x = float(snapshot['origin_x'])
        descriptor.crop_origin_y = float(snapshot['origin_y'])
        descriptor.crop_origin_yaw = float(snapshot['origin_yaw'])
        descriptor.checksum = int(snapshot['fingerprint'][:8], 16)
        return descriptor

    def _full_map_family_records_from_result(self, result):
        ids = []
        for diagnostic in reversed(getattr(result, 'consensus_diagnostics', ())):
            if diagnostic.get('kind') == 'mrpt_selected_family':
                ids = [str(value) for value in diagnostic.get(
                    'physical_evidence_ids', ())]
                break
        if not ids:
            ids = [record['evidence_id'] for record in
                   self.full_map_confirmation_records[-3:]]
        by_id = {record['evidence_id']: record for record in
                 self.full_map_confirmation_records}
        return [by_id[value] for value in ids if value in by_id]

    def _publish_full_map_proposal(self, result):
        records = self._full_map_family_records_from_result(result)
        if len(records) < 3 or self.full_map_proposal is not None:
            return False
        # Keep the proposal envelope anchored to the immutable full-map
        # snapshots.  The evidence arrays name derived witness snapshots;
        # the peer uses these base IDs to deterministically reconstruct the
        # same partition crops before independently verifying them.
        own = self._full_map_descriptor(records[0].get(
            'base_source', records[0]['source']))
        peer = self._full_map_descriptor(records[0].get(
            'base_target', records[0]['target']))
        seed_diagnostic = next((diagnostic for diagnostic in reversed(
            getattr(result, 'consensus_diagnostics', ())) if
            diagnostic.get('kind') == 'stationary_witness_canonical_seed'),
            None)
        if seed_diagnostic is None:
            self._record_diagnostic_event(
                'FULL_MAP_PROPOSAL_REJECTED_MISSING_CANONICAL_SEED')
            return False
        proposal = self._hypothesis_message(
            own, peer, result, status='PROPOSED', accepted=False,
            rejection_reason='',
            evidence_source_keyframe_ids=[record['source']['id']
                                          for record in records],
            evidence_target_keyframe_ids=[record['target']['id']
                                          for record in records],
            stationary_canonical_seed=seed_diagnostic['transform'],
            stationary_canonical_source_snapshot_id=own.keyframe_id,
            stationary_canonical_target_snapshot_id=peer.keyframe_id,
            stationary_canonical_source_map_hash=records[0][
                'base_source']['fingerprint'],
            stationary_canonical_target_map_hash=records[0][
                'base_target']['fingerprint'])
        self.full_map_proposal = proposal
        self._full_map_proposal_pins.update(
            list(proposal.evidence_source_keyframe_ids) +
            list(proposal.evidence_target_keyframe_ids))
        self.pending_proposal_messages[('full-map', 'full-map')] = proposal
        self.hypothesis_pub.publish(proposal)
        self.counters['proposals_published'] += 1
        self._record_diagnostic_event(
            'FULL_MAP_PROPOSAL_PUBLISHED',
            evidence_set_hash=str(proposal.evidence_set_hash),
            stationary_witness_scheme_version=str(
                proposal.stationary_witness_scheme_version),
            stationary_canonical_seed=[float(value) for value in
                                       proposal.stationary_canonical_seed],
            stationary_canonical_source_snapshot_id=str(
                proposal.stationary_canonical_source_snapshot_id),
            stationary_canonical_target_snapshot_id=str(
                proposal.stationary_canonical_target_snapshot_id),
            stationary_canonical_source_map_hash=str(
                proposal.stationary_canonical_source_map_hash),
            stationary_canonical_target_map_hash=str(
                proposal.stationary_canonical_target_map_hash),
            evidence_source_snapshot_ids=[record['source']['id']
                                         for record in records],
            evidence_target_snapshot_ids=[record['target']['id']
                                         for record in records])
        return True

    @staticmethod
    def _is_full_map_hypothesis(message):
        return ('-full-map-' in str(getattr(message, 'source_keyframe_id', ''))
                or '-full-map-' in str(getattr(
                    message, 'target_keyframe_id', '')))

    @staticmethod
    def _is_stationary_witness_proposal(message):
        return any('-witness-' in str(value) for value in (
            list(getattr(message, 'evidence_source_keyframe_ids', [])) +
            list(getattr(message, 'evidence_target_keyframe_ids', []))))

    @staticmethod
    def _stationary_proposal_seed(message):
        """Return the explicit canonical partition seed, or ``None``."""
        if (str(getattr(message, 'stationary_witness_scheme_version', '')) !=
                STATIONARY_WITNESS_SCHEME_VERSION):
            return None
        try:
            seed = tuple(float(value) for value in getattr(
                message, 'stationary_canonical_seed', ()))
        except (TypeError, ValueError):
            return None
        if len(seed) != 3 or not all(math.isfinite(value) for value in seed):
            return None
        return seed

    @staticmethod
    def _stationary_seed_compatible(seed, verified):
        """Apply the existing whole-map consistency bounds to the seed."""
        translation = math.hypot(float(seed[0]) - float(verified[0]),
                                 float(seed[1]) - float(verified[1]))
        yaw = abs(math.atan2(math.sin(float(seed[2]) - float(verified[2])),
                             math.cos(float(seed[2]) - float(verified[2]))))
        return (translation <= 0.15 and yaw <= math.radians(1.0))

    def _full_map_snapshots_for_proposal(self, message):
        """Resolve the immutable full-map evidence named by a proposal."""
        source_ids = [str(value) for value in getattr(
            message, 'evidence_source_keyframe_ids', [])]
        target_ids = [str(value) for value in getattr(
            message, 'evidence_target_keyframe_ids', [])]
        if len(source_ids) != len(target_ids) or len(source_ids) < 3:
            return None
        sources = []
        targets = []
        for source_id, target_id in zip(source_ids, target_ids):
            source = (self.local_full_map_snapshots.get(source_id)
                      if self.robot_id == 'robot1' else
                      self.peer_full_map_snapshots.get(source_id))
            target = (self.peer_full_map_snapshots.get(target_id)
                      if self.robot_id == 'robot1' else
                      self.local_full_map_snapshots.get(target_id))
            if source is None or target is None:
                return None
            sources.append(source)
            targets.append(target)
        return list(zip(sources, targets))

    def _handle_full_map_proposal(self, message):
        """Resolve exact evidence, then independently verify the proposal."""
        proposal = copy.deepcopy(message)
        source_ids, target_ids = self._full_map_proposal_snapshot_ids(proposal)
        required_ids = list(dict.fromkeys(source_ids + target_ids))
        self._full_map_proposal_pins.update(required_ids)
        if (self._is_stationary_witness_proposal(proposal) and
                not self._ensure_stationary_witness_snapshots_for_proposal(
                    proposal)):
            self._record_diagnostic_event(
                'FULL_MAP_PEER_VERIFICATION_REJECTED',
                reason='STATIONARY_WITNESS_RECONSTRUCTION_MISMATCH')
            self._full_map_proposal_pins.difference_update(required_ids)
            return
        missing = self._full_map_missing_snapshot_ids(proposal)
        unavailable = [snapshot_id for snapshot_id in missing
                       if (snapshot_id in self._unavailable_full_map_snapshot_ids or
                           self._full_map_snapshot_owner(snapshot_id) ==
                           self.robot_id)]
        if unavailable:
            self._full_map_proposal_pins.difference_update(required_ids)
            ack = self._ack_message(
                proposal, RegistrationResult(
                    accepted=False, transform=(0.0, 0.0, 0.0),
                    covariance=(0.0,) * 36, inlier_ratio=0.0,
                    residual_m=math.inf, occupied_free_agreement=0.0,
                    overlap_fraction=0.0,
                    reason='FULL_MAP_EVIDENCE_UNAVAILABLE', constraint_count=0,
                    consistent_constraint_count=0, projected_error_m=math.inf,
                    final_confidence=0.0), False,
                'FULL_MAP_EVIDENCE_UNAVAILABLE')
            self.hypothesis_pub.publish(ack)
            self._record_diagnostic_event(
                'FULL_MAP_PEER_VERIFICATION_REJECTED',
                reason='FULL_MAP_EVIDENCE_UNAVAILABLE',
                snapshot_ids=unavailable)
            return
        if missing:
            self._pending_full_map_verification = proposal
            self.counters['full_map_verifications_pending'] += 1
            self._record_diagnostic_event(
                'FULL_MAP_VERIFICATION_PENDING',
                reason='PENDING_MISSING_EVIDENCE', snapshot_ids=missing)
            self._request_full_map_snapshots(missing)
            return
        self._verify_full_map_proposal(proposal)

    def _resume_pending_full_map_verification(self):
        proposal = self._pending_full_map_verification
        if proposal is None:
            return False
        missing = self._full_map_missing_snapshot_ids(proposal)
        unavailable = [snapshot_id for snapshot_id in missing
                       if (snapshot_id in self._unavailable_full_map_snapshot_ids or
                           self._full_map_snapshot_owner(snapshot_id) ==
                           self.robot_id)]
        if unavailable:
            self._pending_full_map_verification = None
            self._full_map_proposal_pins.difference_update(
                self._full_map_proposal_snapshot_ids(proposal)[0] +
                self._full_map_proposal_snapshot_ids(proposal)[1])
            ack = self._ack_message(
                proposal, RegistrationResult(
                    accepted=False, transform=(0.0, 0.0, 0.0),
                    covariance=(0.0,) * 36, inlier_ratio=0.0,
                    residual_m=math.inf, occupied_free_agreement=0.0,
                    overlap_fraction=0.0,
                    reason='FULL_MAP_EVIDENCE_UNAVAILABLE', constraint_count=0,
                    consistent_constraint_count=0, projected_error_m=math.inf,
                    final_confidence=0.0), False,
                'FULL_MAP_EVIDENCE_UNAVAILABLE')
            self.hypothesis_pub.publish(ack)
            return False
        if missing:
            self._request_full_map_snapshots(missing)
            return False
        self._pending_full_map_verification = None
        self.counters['full_map_verifications_resumed'] += 1
        self._record_diagnostic_event(
            'FULL_MAP_VERIFICATION_RESUMED',
            evidence_set_hash=str(proposal.evidence_set_hash))
        self._verify_full_map_proposal(proposal)
        return True

    def _verify_full_map_proposal(self, proposal):
        """Verify the proposed basin in canonical order, without global search."""
        pairs = self._full_map_snapshots_for_proposal(proposal)
        if pairs is None:
            ack = self._ack_message(
                proposal, RegistrationResult(
                    accepted=False, transform=(0.0, 0.0, 0.0),
                    covariance=(0.0,) * 36, inlier_ratio=0.0,
                    residual_m=math.inf, occupied_free_agreement=0.0,
                    overlap_fraction=0.0,
                    reason='MISSING_FULL_MAP_EVIDENCE', constraint_count=0,
                    consistent_constraint_count=0, projected_error_m=math.inf,
                    final_confidence=0.0), False,
                'MISSING_FULL_MAP_EVIDENCE')
            self.hypothesis_pub.publish(ack)
            self._record_diagnostic_event(
                'FULL_MAP_PEER_VERIFICATION_REJECTED',
                reason='MISSING_FULL_MAP_EVIDENCE')
            self._release_full_map_proposal_pins(proposal)
            return
        stationary_witness_proposal = self._is_stationary_witness_proposal(
            proposal)
        canonical_seed = (self._stationary_proposal_seed(proposal)
                          if stationary_witness_proposal else None)
        if stationary_witness_proposal and canonical_seed is None:
            self._record_diagnostic_event(
                'FULL_MAP_PEER_VERIFICATION_REJECTED',
                reason='MISSING_STATIONARY_CANONICAL_SEED')
            self._release_full_map_proposal_pins(proposal)
            return
        canonical_results = []
        proposal_seed = (canonical_seed if canonical_seed is not None else
                         self._summary_transform(proposal))
        for source, target in pairs:
            try:
                # The proposer has already done the global discovery.  The
                # receiver independently checks the exact maps around that
                # basin with the deterministic symmetric local refinement;
                # it must not perform a source/target-dependent global search.
                if stationary_witness_proposal:
                    # Stationary witnesses are defined by their immutable,
                    # disjoint support.  Re-run that same support-local
                    # estimator on the peer; using the generic whole-crop
                    # refinement here can select a different local mode and
                    # falsely fail the unchanged compatibility gate.
                    result = reestimate_registration_from_seed(
                        source['crop'], target['crop'], proposal_seed,
                        minimum_agreement=0.0)
                else:
                    result = self._run_full_map_registration(
                        source['crop'], target['crop'],
                        initial_transform=proposal_seed)
            except Exception as exc:
                self._record_diagnostic_event(
                    'FULL_MAP_PEER_VERIFICATION_EXCEPTION',
                    exception=repr(exc))
                result = None
            if result is None or not result.accepted:
                reason = ('FULL_MAP_PEER_REGISTRATION_REJECTED' if result is not None
                           else 'FULL_MAP_PEER_REGISTRATION_EXCEPTION')
                if result is not None:
                    reason = str(result.reason)
                fallback = result or RegistrationResult(
                    accepted=False, transform=(0.0, 0.0, 0.0),
                    covariance=(0.0,) * 36, inlier_ratio=0.0,
                    residual_m=math.inf, occupied_free_agreement=0.0,
                    overlap_fraction=0.0, reason=reason,
                    constraint_count=1, consistent_constraint_count=0,
                    projected_error_m=math.inf, final_confidence=0.0)
                ack = self._ack_message(proposal, fallback, False, reason)
                self.hypothesis_pub.publish(ack)
                self._record_diagnostic_event(
                    'FULL_MAP_PEER_VERIFICATION_REJECTED', reason=reason)
                self._release_full_map_proposal_pins(proposal)
                return
            canonical_results.append(result)
        self._record_diagnostic_event(
            'FULL_MAP_CANONICAL_ORDER', canonical_source_robot='robot1',
            canonical_target_robot='robot2', verification=True,
            evidence_set_hash=str(proposal.evidence_set_hash))
        evidence_ids = [f"{source['id']}|{target['id']}"
                        for source, target in pairs]
        family = select_hypothesis_family(
            [(source['crop'], target['crop']) for source, target in pairs],
            [(result,) for result in canonical_results],
            target_map_radius_m=self.target_map_radius_m,
            min_consistent_constraints=3,
            min_spatial_baseline_m=0.0,
            max_translation_consistency_m=0.15,
            max_yaw_consistency_rad=math.radians(1.0),
            max_projected_registration_error_m=(
                self.max_projected_registration_error_m),
            minimum_agreement=0.0,
            # A stationary witness family is three spatially disjoint
            # re-estimates from one immutable map pair.  Its timestamps are
            # intentionally identical; applying the temporal-span gate here
            # would reject valid stationary evidence.  Ordinary proposals
            # retain the existing time-separation gate unchanged.
            evidence_timestamps=(None if stationary_witness_proposal else [(
                int(source['timestamp_ns']), int(target['timestamp_ns']))
                for source, target in pairs]),
            evidence_ids=evidence_ids)
        if not family.accepted:
            ack = self._ack_message(
                proposal, family, False, str(family.reason))
            self.hypothesis_pub.publish(ack)
            self._record_diagnostic_event(
                'FULL_MAP_PEER_VERIFICATION_REJECTED',
                reason=str(family.reason),
                consistent_constraint_count=int(
                    family.consistent_constraint_count))
            self._release_full_map_proposal_pins(proposal)
            return
        ack = self._ack_message(proposal, family, True, '')
        ack.selector_status = 'ACCEPTED_HYPOTHESIS'
        ack.accepted = True
        self.hypothesis_pub.publish(ack)
        self._record_diagnostic_event(
            'FULL_MAP_PEER_VERIFICATION_ACCEPTED',
            evidence_set_hash=str(proposal.evidence_set_hash),
            consistent_constraint_count=int(family.consistent_constraint_count),
            transform=[float(value) for value in family.transform])
        self.counters['accepted_hypotheses'] += 1
        self.accepted = ack
        self.accepted_ros_time_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.accepted_wall = time.monotonic()
        self.publish_accepted_tf()
        self.publish_local_map(force=True)
        self._enter_post_handoff_quiescence()
        self._release_full_map_proposal_pins(proposal)

    def _release_full_map_proposal_pins(self, proposal):
        source_ids, target_ids = self._full_map_proposal_snapshot_ids(proposal)
        self._full_map_proposal_pins.difference_update(source_ids + target_ids)

    def _handle_full_map_ack(self, message):
        proposal = self.full_map_proposal
        if proposal is None:
            self._record_diagnostic_event(
                'FULL_MAP_ACK_IGNORED_NO_PROPOSAL')
            return
        if str(message.evidence_set_hash) != str(proposal.evidence_set_hash):
            self._record_diagnostic_event(
                'FULL_MAP_ACK_REJECTED_HASH_MISMATCH',
                proposal_hash=str(proposal.evidence_set_hash),
                peer_hash=str(message.evidence_set_hash))
            return
        if message.status != 'ACCEPTED' or not bool(message.accepted):
            self._record_diagnostic_event(
                'FULL_MAP_ACK_REJECTED', reason=str(message.rejection_reason))
            self.full_map_proposal = None
            self._release_full_map_proposal_pins(proposal)
            return
        final = copy.deepcopy(proposal)
        final.status = 'ACCEPTED'
        final.accepted = True
        final.rejection_reason = ''
        final.selector_status = 'ACCEPTED_HYPOTHESIS'
        self.hypothesis_pub.publish(final)
        self._record_diagnostic_event(
            'FULL_MAP_CANONICAL_HANDOFF_ACCEPTED',
            evidence_set_hash=str(final.evidence_set_hash),
            transform=[float(final.source_to_target.translation.x),
                       float(final.source_to_target.translation.y),
                       float(self._yaw(final.source_to_target.rotation))])
        self.counters['accepted_hypotheses'] += 1
        self.accepted = final
        self.accepted_ros_time_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.accepted_wall = time.monotonic()
        self.publish_accepted_tf()
        self.publish_local_map(force=True)
        self._enter_post_handoff_quiescence()
        self._release_full_map_proposal_pins(proposal)

    def tick(self):
        if getattr(self, '_post_handoff_quiesced', False):
            return
        self._sample_cpu()
        if self.full_map_registration:
            # This branch is intentionally before the legacy worker drain,
            # descriptor publication, peer comparison, and crop-request
            # machinery.  Full-map discovery has exactly one registration
            # input: the latest immutable full local map and peer map.
            self._drain_full_map_registration()
            if self.accepted is None:
                self._maybe_schedule_full_map_registration()
            return
        self._drain_candidate_registration()
        self._drain_full_map_registration()
        now = time.monotonic()
        if (self.evidence_acquisition_started or
                (self._evidence_opportunity_deadline_wall is not None and
                 now < self._evidence_opportunity_deadline_wall)):
            self._publish_evidence_status(True)
        elif self._evidence_opportunity_deadline_wall is not None:
            self._evidence_opportunity_deadline_wall = None
            self._publish_evidence_status(False)
        # Once both peers have installed the immutable canonical handoff,
        # descriptor/crop acquisition and hypothesis re-auctioning are no
        # longer valid work.  Continuing them only creates avoidable reliable
        # DDS traffic and can compete with shared-map/navigation callbacks.
        # Map callbacks remain active and independently rate-limit PeerMap
        # export, so this does not stop local SLAM or fusion inputs.
        if self.accepted is not None:
            self.counters['post_handoff_protocol_ticks_skipped'] += 1
            return
        self._process_pending_peer_evidence()
        if time.monotonic() - self.last_descriptor_wall >= self.descriptor_period_s:
            self.publish_descriptor()
        # Drain at most one descriptor key per timer tick.  Descriptor
        # callbacks only retain messages and enqueue keys, preventing the
        # single-threaded executor from losing current peer views while
        # descriptor/temporal work is performed.
        if self.pending_descriptor_pair_keys:
            self._compare_peer_descriptors(
                new_peer_key=None, new_own_key=None)
        elif self.pending_peer_descriptor_keys:
            key = self.pending_peer_descriptor_keys.popleft()
            if key in self.peer_descriptors:
                self._compare_peer_descriptors(new_peer_key=key)
        elif self.pending_own_descriptor_keys:
            key = self.pending_own_descriptor_keys.popleft()
            if key in self.keyframes:
                self._compare_peer_descriptors(new_own_key=key)
        self._maybe_finalize_evidence_acquisition()

    def _drain_candidate_registration(self):
        """Apply one completed geometric registration on the ROS thread.

        ``register_crops`` is intentionally isolated in the worker.  No ROS
        objects or mutable frontend state are touched there; this method is
        the only place where counters, evidence, and protocol messages are
        updated from a registration result.
        """
        if getattr(self, '_post_handoff_quiesced', False):
            return
        future = self._registration_future
        if future is None or not future.done():
            return
        context = self._registration_context
        self._registration_future = None
        self._registration_context = None
        if context is None:
            return
        (
            request_key, pair_key, candidate, request_metadata, physical_key,
            candidate_geometry_key, own_key, peer_key, own_crop,
            received_crop, map_epoch, descriptor_checksum,
        ) = context
        try:
            raw_result = future.result()
            if isinstance(raw_result, (tuple, list)):
                hypotheses = tuple(raw_result)
            else:
                hypotheses = (raw_result,)
            if not hypotheses:
                raise RuntimeError('registration backend returned no result')
            # The primary result is only the diagnostic/legacy protocol view;
            # canonical consensus receives the complete bounded mode set.
            result = max(
                hypotheses,
                key=lambda item: (
                    bool(item.accepted), int(getattr(item, 'mode_support', 1)),
                    float(getattr(item, 'mode_log_weight', -math.inf)),
                    float(item.inlier_ratio), -float(item.residual_m),
                    -int(getattr(item, 'mode_index', -1))))
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self.consensus_gate_rejection_counts['REGISTRATION_EXCEPTION'] += 1
            self._record_diagnostic_event(
                'REGISTRATION_WORKER_EXCEPTION', keyframe_id=peer_key,
                exception=repr(exc))
            self._finalize_registration_capture(pair_key, error=exc)
            self._record_physical_worker_result(
                candidate, request_metadata, error=repr(exc))
            self.rejected_physical_evidence_keys.add(physical_key)
            self.rejected_physical_geometry_keys.add(candidate_geometry_key)
            self.rejected_physical_geometry_batches[candidate_geometry_key] = (
                self._request_batch_id(request_metadata))
            self._request_next_candidate_verification()
            self._start_next_registration_context()
            return
        self._apply_candidate_verification_result(
            pair_key, candidate, result, request_metadata, physical_key,
            candidate_geometry_key, own_key, peer_key, own_crop,
            received_crop, map_epoch, descriptor_checksum,
            hypotheses=hypotheses)
        self._start_next_registration_context()

    def _request_batch_id(self, request_metadata):
        """Return the acquisition batch that created a worker request.

        Registration is asynchronous: a response from an older batch can be
        applied after a newer batch has opened.  Geometry-only suppression is
        intentionally batch-scoped, so it must use the request's immutable
        batch ID rather than the currently active batch.
        """
        try:
            return int((request_metadata or {}).get(
                'acquisition_batch_id', self.verification_batches.batch_id))
        except (TypeError, ValueError):
            return int(self.verification_batches.batch_id)

    def _start_registration_context(self, context):
        """Submit one immutable crop pair to the serialized worker."""
        if (getattr(self, '_post_handoff_quiesced', False) or
                getattr(self, '_registration_shutdown', False)):
            return False
        (
            request_key, pair_key, candidate, request_metadata, physical_key,
            candidate_geometry_key, own_key, peer_key, own_crop,
            received_crop, map_epoch, descriptor_checksum,
        ) = context
        mature_evidence, maturity = self._consensus_pair_maturity(
            own_crop, received_crop)
        self._capture_registration_inputs(
            pair_key, candidate, request_metadata, own_crop, received_crop,
            map_epoch, descriptor_checksum)
        self._registration_context = context
        self._registration_future = self._registration_executor.submit(
            register_crop_hypotheses, own_crop, received_crop,
            backend=self.registration_backend,
            minimum_agreement=(0.0 if mature_evidence else 0.55),
            mrpt_max_kld=self.mrpt_max_kld,
            mrpt_max_modes=self.mrpt_max_modes_per_call,
            mrpt_repetitions=self.mrpt_repetitions_per_pair,
            max_distinct_modes=self.mrpt_max_distinct_modes_per_pair)
        self._record_diagnostic_event(
            'REGISTRATION_WORKER_SUBMITTED', keyframe_id=peer_key,
            constraint_count=1, queue_depth=len(
                self._registration_pending_contexts),
            consensus_evidence_mature=mature_evidence,
            consensus_maturity=maturity,
            request_key=list(request_key))
        return True

    def _queue_registration_context(self, context):
        """Queue a response while the worker is busy, with a hard bound."""
        if (getattr(self, '_post_handoff_quiesced', False) or
                getattr(self, '_registration_shutdown', False)):
            return False
        request_key = context[0]
        if request_key in self._registration_pending_keys:
            return True
        if len(self._registration_pending_contexts) >= \
                self._registration_pending_contexts.maxlen:
            self._registration_queue_drops += 1
            self._record_diagnostic_event(
                'REGISTRATION_QUEUE_FULL', request_key=list(request_key),
                queue_capacity=self._registration_pending_contexts.maxlen)
            self._write_physical_evidence_diagnostic(
                'REGISTRATION_QUEUE_DROPPED', request_key=list(request_key),
                reason='QUEUE_FULL',
                queue_capacity=self._registration_pending_contexts.maxlen)
            return False
        self._registration_pending_contexts.append(context)
        self._registration_pending_keys.add(request_key)
        self._registration_queue_enqueues += 1
        self._registration_queue_max_depth = max(
            self._registration_queue_max_depth,
            len(self._registration_pending_contexts))
        self._record_diagnostic_event(
            'REGISTRATION_QUEUED', request_key=list(request_key),
            queue_depth=len(self._registration_pending_contexts))
        return True

    def _start_next_registration_context(self):
        """Start the next queued response after the worker completes."""
        if self._registration_future is not None:
            return
        if (getattr(self, '_post_handoff_quiesced', False) or
                getattr(self, '_registration_shutdown', False) or
                self.accepted is not None or
                not self._registration_pending_contexts):
            return
        context = self._registration_pending_contexts.popleft()
        self._registration_pending_keys.discard(context[0])
        self._registration_queue_dequeues += 1
        self._start_registration_context(context)

    @staticmethod
    def _registration_capture_crop(crop):
        values = np.asarray(crop.values, dtype=np.int16)
        unique, counts = np.unique(values, return_counts=True)
        return {
            'origin_x': float(crop.origin_x),
            'origin_y': float(crop.origin_y),
            'origin_yaw': float(crop.origin_yaw),
            'resolution': float(crop.resolution),
            'width': int(values.shape[1]),
            'height': int(values.shape[0]),
            'dtype': str(values.dtype),
            'min_value': int(values.min()) if values.size else None,
            'max_value': int(values.max()) if values.size else None,
            'value_counts': {str(int(value)): int(count)
                             for value, count in zip(unique, counts)},
            'occupancy_encoding': {
                'unknown_value': -1,
                'occupied_threshold': 50,
                'free_or_unoccupied_range': '0..49',
            },
        }

    @staticmethod
    def _render_registration_capture(values, path):
        """Render occupancy values without changing the registration array."""
        from PIL import Image
        array = np.asarray(values, dtype=np.int16)
        image = np.full(array.shape, 127, dtype=np.uint8)
        image[array >= 50] = 0
        image[(array >= 0) & (array < 50)] = 255
        Image.fromarray(image, mode='L').save(path)

    def _consensus_pair_maturity(self, source_crop, target_crop):
        """Return intrinsic maturity for both immutable registration crops."""
        source_mature, source_details = consensus_crop_maturity(
            source_crop, self.consensus_min_known_fraction,
            self.consensus_min_occupied_cells)
        target_mature, target_details = consensus_crop_maturity(
            target_crop, self.consensus_min_known_fraction,
            self.consensus_min_occupied_cells)
        return bool(source_mature and target_mature), {
            'source': source_details,
            'target': target_details,
        }

    def _capture_registration_inputs(
            self, pair_key, candidate, request_metadata, source_crop,
            target_crop, target_map_epoch, target_checksum):
        """Persist one bounded, lossless registration input pair.

        The capture is placed immediately before the unchanged worker submit,
        so the arrays in the NPZ are exactly the arrays passed to
        ``register_crops``.  It is opt-in and has no effect on estimator
        inputs, scheduling, thresholds, or acceptance.
        """
        if (self.registration_input_capture_output is None or
                self._registration_capture_count >=
                self.registration_input_capture_max_pairs):
            return
        try:
            peer_key, own_key, peer_descriptor, own_descriptor = (
                self._candidate_fields(candidate))
            root = self.registration_input_capture_output / self.robot_id
            root.mkdir(parents=True, exist_ok=True)
            sequence = self._registration_capture_count + 1
            stem = f'{self.robot_id}_registration_{sequence:04d}'
            npz_path = root / f'{stem}.npz'
            json_path = root / f'{stem}.json'
            source_values = np.asarray(source_crop.values, dtype=np.int16).copy()
            target_values = np.asarray(target_crop.values, dtype=np.int16).copy()
            source_descriptor = np.frombuffer(
                bytes(own_descriptor.descriptor_bytes), dtype=np.uint8).copy()
            target_descriptor = np.frombuffer(
                bytes(peer_descriptor.descriptor_bytes), dtype=np.uint8).copy()
            np.savez_compressed(
                npz_path,
                source_values=source_values,
                target_values=target_values,
                source_descriptor=source_descriptor,
                target_descriptor=target_descriptor)
            self._render_registration_capture(
                source_values, root / f'{stem}_source.png')
            self._render_registration_capture(
                target_values, root / f'{stem}_target.png')
            match = self.matches.get((peer_key, own_key))
            metadata = {
                'schema_version': 'registration_input_capture_1.0',
                'status': 'SUBMITTED',
                'robot_id': self.robot_id,
                'peer_robot_id': self.peer_robot_id,
                'source_robot_id': self.robot_id,
                'target_robot_id': self.peer_robot_id,
                'source_keyframe_id': str(own_key),
                'target_keyframe_id': str(peer_key),
                'request_id': str((request_metadata or {}).get(
                    'verification_attempt_id', '')),
                'evidence_id': str((request_metadata or {}).get(
                    'candidate_correlation_id', '')),
                'acquisition_batch_id': int((request_metadata or {}).get(
                    'acquisition_batch_id', 0)),
                'source_timestamp_ns': int(self._stamp_ns(own_descriptor)),
                'target_timestamp_ns': int(self._stamp_ns(peer_descriptor)),
                'request_timestamp_ns': int((request_metadata or {}).get(
                    'request_timestamp_ns', 0)),
                'source_map_epoch': int(own_descriptor.map_epoch),
                'target_map_epoch': int(target_map_epoch),
                'source_map_frame': str(own_descriptor.header.frame_id),
                'target_map_frame': str(peer_descriptor.header.frame_id),
                'source_to_target_convention':
                    'p_target = R(yaw) * p_source + translation',
                'source_crop': self._registration_capture_crop(source_crop),
                'target_crop': self._registration_capture_crop(target_crop),
                'source_descriptor': {
                    'rings': int(own_descriptor.ring_count),
                    'sectors': int(own_descriptor.sector_count),
                    'version': int(own_descriptor.descriptor_version),
                    'checksum': int(own_descriptor.checksum),
                    'values_shape': [12, 24, 2],
                    'values': source_descriptor.reshape(12, 24, 2).tolist(),
                },
                'target_descriptor': {
                    'rings': int(peer_descriptor.ring_count),
                    'sectors': int(peer_descriptor.sector_count),
                    'version': int(peer_descriptor.descriptor_version),
                    'checksum': int(peer_descriptor.checksum),
                    'values_shape': [12, 24, 2],
                    'values': target_descriptor.reshape(12, 24, 2).tolist(),
                },
                'descriptor_similarity': (
                    None if match is None else float(match.similarity)),
                'descriptor_margin': (
                    None if match is None else float(match.margin)),
                'descriptor_sector_shift': (
                    None if match is None else int(match.sector_shift)),
                'descriptor_known_fraction': (
                    None if match is None else float(match.known_fraction)),
                'npz_file': npz_path.name,
                'source_png_file': f'{stem}_source.png',
                'target_png_file': f'{stem}_target.png',
            }
            json_path.write_text(json.dumps(metadata, indent=2, sort_keys=True),
                                encoding='utf-8')
            self._registration_capture_paths[pair_key] = json_path
            self._registration_capture_count = sequence
        except Exception as exc:
            self.get_logger().warning(
                'registration input capture failed: %s', exc)

    def _finalize_registration_capture(self, pair_key, result=None, error=None,
                                       strong_consensus_eligible=None,
                                       strong_consensus_reason=''):
        path = self._registration_capture_paths.get(pair_key)
        if path is None:
            return
        try:
            metadata = json.loads(path.read_text(encoding='utf-8'))
            if error is not None:
                metadata.update({'status': 'EXCEPTION', 'error': repr(error)})
            else:
                source_resolution = float(metadata['source_crop']['resolution'])
                gates = {
                    'forward_inlier_ratio': (float(result.inlier_ratio), 0.35,
                                             '>=', 'FORWARD_INLIER_BELOW_GATE'),
                    'reverse_inlier_ratio': (
                        float(getattr(result, 'reverse_inlier_ratio',
                                      result.inlier_ratio)), 0.30, '>=',
                        'REVERSE_INLIER_BELOW_GATE'),
                    'residual_m': (float(result.residual_m),
                                   max(0.12, 3.0 * source_resolution), '<=',
                                   'RESIDUAL_ABOVE_GATE'),
                    'occupied_free_agreement': (
                        float(result.occupied_free_agreement), 0.55, '>=',
                        'OCCUPIED_FREE_AGREEMENT_BELOW_GATE'),
                    'overlap_fraction': (float(result.overlap_fraction), 0.15,
                                         '>=', 'OVERLAP_BELOW_GATE'),
                }
                failed = []
                for name, (value, threshold, relation, reason) in gates.items():
                    passed = (value >= threshold if relation == '>=' else
                              value <= threshold)
                    if not passed:
                        failed.append(reason)
                metadata.update({
                    'status': 'ACCEPTED' if result.accepted else 'REJECTED',
                    'returned_transform_source_to_target': [
                        float(value) for value in result.transform],
                    'residual_m': float(result.residual_m),
                    'median_residual_m': float(result.median_residual_m),
                    'p95_residual_m': float(result.p95_residual_m),
                    'forward_inlier_ratio': float(result.inlier_ratio),
                    'reverse_inlier_ratio': float(getattr(
                        result, 'reverse_inlier_ratio', result.inlier_ratio)),
                    'occupied_free_agreement': float(
                        result.occupied_free_agreement),
                    'overlap_fraction': float(result.overlap_fraction),
                    'translation_uncertainty_m': float(
                        result.translation_uncertainty_m),
                    'yaw_uncertainty_rad': float(result.yaw_uncertainty_rad),
                    'condition_number': float(result.condition_number),
                    'failed_gates': failed,
                    'rejection_reason': '' if result.accepted else str(
                        result.reason),
                    'strong_consensus_eligible': (
                        None if strong_consensus_eligible is None else
                        bool(strong_consensus_eligible)),
                    'strong_consensus_rejection_reason': str(
                        strong_consensus_reason or ''),
                })
            path.write_text(json.dumps(metadata, indent=2, sort_keys=True),
                            encoding='utf-8')
        except Exception as exc:
            self.get_logger().warning(
                'registration input capture finalization failed: %s', exc)

    def _clear_pending_registration_contexts(self, reason):
        """Release queued crop arrays once no further registration is needed."""
        count = len(self._registration_pending_contexts)
        if not count:
            return
        self._registration_pending_contexts.clear()
        self._registration_pending_keys.clear()
        self._record_diagnostic_event(
            'REGISTRATION_QUEUE_CLEARED', reason=str(reason),
            cleared_count=count)

    def descriptor_callback(self, message):
        if (getattr(self, '_post_handoff_quiesced', False) or
                message.source_robot_id != self.peer_robot_id):
            return
        if message.descriptor_version != 1 or not message.descriptor_bytes:
            return
        self.counters['descriptors_received'] += 1
        self._record_diagnostic_event(
            'DESCRIPTOR_RECEIVED', keyframe_id=message.keyframe_id,
            map_epoch=int(message.map_epoch), descriptor_bytes=len(
                message.descriptor_bytes))
        key = message.keyframe_id
        self._update_confirmation_cadence(
            self.peer_descriptor_stamps_ns, self._stamp_ns(message))
        self.peer_descriptors[key] = message
        while len(self.peer_descriptors) > self.max_keyframes:
            self.peer_descriptors.popitem(last=False)
        self._prune_expired_descriptor_state()
        self.pending_peer_descriptor_keys.append(key)

    @staticmethod
    def _stamp_ns(message):
        return (int(message.header.stamp.sec) * 1_000_000_000 +
                int(message.header.stamp.nanosec))

    def _prune_expired_descriptor_state(self):
        """Discard pair caches whose bounded descriptor history expired.

        Descriptor matching and temporal confirmation only use currently
        retained own/peer keyframes.  Keeping match/cache entries for every
        historical Cartesian pair made each later descriptor callback scan an
        ever-growing set, even though the configured history is bounded.
        Physical verification/rejection history is intentionally not touched:
        it is the separate safety record that prevents retrying an old pair.
        """
        live_own = set(self.keyframes)
        live_peer = set(self.peer_descriptors)
        live_pairs = {
            (peer_key, own_key)
            for peer_key in live_peer
            for own_key in live_own
        }
        for name in (
                'matches', 'compared_pairs', 'descriptor_gate_status',
                'temporal_support_cache', 'temporal_gate_rejected_pairs',
                'confirmations', 'descriptor_ambiguous_pairs'):
            values = getattr(self, name)
            if isinstance(values, dict):
                for pair in tuple(values):
                    if pair not in live_pairs:
                        values.pop(pair, None)
            else:
                values.intersection_update(live_pairs)
        for name in ('descriptor_gate_survivors', 'temporal_gate_survivors'):
            getattr(self, name).intersection_update(live_pairs)

    def _temporal_observations(self):
        """Snapshot valid cached matches once for one descriptor callback."""
        observations = []
        for (other_peer_key, other_own_key), other_match in self.matches.items():
            other_own_entry = self.keyframes.get(other_own_key)
            other_peer_message = self.peer_descriptors.get(other_peer_key)
            if other_own_entry is None or other_peer_message is None:
                continue
            observations.append((
                (other_peer_key, other_own_key),
                self._stamp_ns(other_own_entry[0]),
                self._stamp_ns(other_peer_message),
                other_match.similarity, other_match.margin,
                other_match.known_fraction, other_match.sector_shift))
        return observations

    def _temporal_support(self, peer_key, own_key, match, observations=None):
        """Count distinct nearby cheap matches for this candidate.

        A descriptor/keyframe pair is advertised once, so counting repeated
        comparisons of that same pair can never provide confirmation.  Use
        distinct nearby own/peer keyframes instead; geometric registration is
        still the authoritative gate after this cheap temporal check.
        """
        own_entry = self.keyframes.get(own_key)
        peer_message = self.peer_descriptors.get(peer_key)
        if own_entry is None or peer_message is None:
            return 0
        own_stamp = self._stamp_ns(own_entry[0])
        peer_stamp = self._stamp_ns(peer_message)
        if observations is None:
            observations = self._temporal_observations()
        return temporal_support_count(
            (peer_key, own_key), own_stamp, peer_stamp,
            match.sector_shift, observations, self.similarity_gate,
            self.margin_gate, 0.12, self.effective_confirmation_window_ns,
            descriptor_advisory=True)

    def _temporal_candidate_affected(self, pair_key, changed_pair_keys):
        """Return whether a newly queued descriptor can change this gate."""
        if not changed_pair_keys or pair_key in changed_pair_keys:
            return bool(changed_pair_keys)
        peer_key, own_key = pair_key
        own_entry = self.keyframes.get(own_key)
        peer = self.peer_descriptors.get(peer_key)
        if own_entry is None or peer is None:
            return False
        own_stamp = self._stamp_ns(own_entry[0])
        peer_stamp = self._stamp_ns(peer)
        for changed_peer_key, changed_own_key in changed_pair_keys:
            changed_peer = self.peer_descriptors.get(changed_peer_key)
            changed_own_entry = self.keyframes.get(changed_own_key)
            if changed_peer is None or changed_own_entry is None:
                continue
            if (abs(self._stamp_ns(changed_own_entry[0]) - own_stamp) <=
                    self.effective_confirmation_window_ns and
                    abs(self._stamp_ns(changed_peer) - peer_stamp) <=
                    self.effective_confirmation_window_ns):
                return True
        return False

    def _update_temporal_support_cache(self, changed_pair_keys):
        """Update temporal support incrementally for newly computed pairs.

        A new descriptor can add evidence to existing anchors, but it cannot
        change the relationship between two already cached pairs.  The old
        implementation re-ran every affected anchor against the complete
        Cartesian observation list, which became quadratic as descriptor
        history accumulated.  Compute the new anchors exactly once, then add
        only the newly arrived valid observations to existing cached counts.
        This preserves the same timestamp, quality, and circular-sector gates.
        """
        changed = []
        changed_keys = set(changed_pair_keys)
        for pair_key in changed_keys:
            match = self.matches.get(pair_key)
            if match is None or self.descriptor_gate_status.get(pair_key) is not None:
                continue
            peer_key, own_key = pair_key
            own_entry = self.keyframes.get(own_key)
            peer = self.peer_descriptors.get(peer_key)
            if own_entry is None or peer is None:
                continue
            changed.append((
                pair_key,
                self._stamp_ns(own_entry[0]),
                self._stamp_ns(peer),
                int(match.sector_shift),
            ))
        if not changed:
            return

        observations = self._temporal_observations()
        for pair_key, own_stamp, peer_stamp, sector_shift in changed:
            match = self.matches[pair_key]
            self.temporal_support_cache[pair_key] = temporal_support_count(
                pair_key, own_stamp, peer_stamp, sector_shift, observations,
                self.similarity_gate, self.margin_gate, 0.12,
                self.effective_confirmation_window_ns,
                descriptor_advisory=True)

        existing = []
        for pair_key in self.temporal_support_cache:
            if pair_key in changed_keys or pair_key not in self.matches:
                continue
            if self.descriptor_gate_status.get(pair_key) is not None:
                continue
            peer_key, own_key = pair_key
            own_entry = self.keyframes.get(own_key)
            peer = self.peer_descriptors.get(peer_key)
            if own_entry is None or peer is None:
                continue
            existing.append((
                pair_key,
                self._stamp_ns(own_entry[0]),
                self._stamp_ns(peer),
                int(self.matches[pair_key].sector_shift),
            ))
        if not existing:
            return
        anchor_own = np.asarray([item[1] for item in existing], dtype=np.int64)
        anchor_peer = np.asarray([item[2] for item in existing], dtype=np.int64)
        anchor_shift = np.asarray([item[3] for item in existing], dtype=np.int16)
        new_own = np.asarray([item[1] for item in changed], dtype=np.int64)
        new_peer = np.asarray([item[2] for item in changed], dtype=np.int64)
        new_shift = np.asarray([item[3] for item in changed], dtype=np.int16)
        within_time = (
            (np.abs(anchor_own[:, None] - new_own[None, :]) <=
             self.effective_confirmation_window_ns) &
            (np.abs(anchor_peer[:, None] - new_peer[None, :]) <=
             self.effective_confirmation_window_ns))
        shift_delta = np.abs(anchor_shift[:, None] - new_shift[None, :])
        shift_delta = np.minimum(shift_delta, 24 - shift_delta)
        additions = np.sum(within_time & (shift_delta <= 2), axis=1)
        for item, addition in zip(existing, additions):
            self.temporal_support_cache[item[0]] += int(addition)

    def _compare_peer_descriptors(self, new_peer_key=None, new_own_key=None):
        if not self.keyframes or not self.peer_descriptors:
            return
        if self.batch_proposal_published:
            return
        if new_peer_key is not None or new_own_key is not None:
            peer_items = list(self.peer_descriptors.items())
            own_items = list(self.keyframes.items())
            if new_peer_key is not None:
                peer_items = [(new_peer_key,
                               self.peer_descriptors[new_peer_key])]
            if new_own_key is not None:
                own_items = [(new_own_key,
                              self.keyframes[new_own_key])]
            new_pairs = [
                (peer_key, own_key, peer, own)
                for peer_key, peer in peer_items
                for own_key, own in own_items]
            for peer_key, own_key, _, _ in new_pairs:
                pair_key = (peer_key, own_key)
                if (pair_key not in self.compared_pairs and
                        pair_key not in self.pending_descriptor_pair_key_set):
                    self.pending_descriptor_pair_keys.append(pair_key)
                    self.pending_descriptor_pair_key_set.add(pair_key)

        uncomputed_pairs = []
        while (self.pending_descriptor_pair_keys and
               len(uncomputed_pairs) < self.descriptor_pair_budget_per_tick):
            pair_key = self.pending_descriptor_pair_keys.popleft()
            self.pending_descriptor_pair_key_set.discard(pair_key)
            if pair_key in self.compared_pairs:
                continue
            peer_key, own_key = pair_key
            peer = self.peer_descriptors.get(peer_key)
            own_entry = self.keyframes.get(own_key)
            if peer is None or own_entry is None:
                continue
            uncomputed_pairs.append((peer_key, own_key, peer, own_entry))
        changed_pair_keys = {
            (peer_key, own_key)
            for peer_key, own_key, _, _ in uncomputed_pairs}
        for peer_key, own_key, _, _ in uncomputed_pairs:
            self.compared_pairs.add((peer_key, own_key))
        if uncomputed_pairs:
            first_descriptors = [
                bytes(own_entry[0].descriptor_bytes)
                for _, _, _, own_entry in uncomputed_pairs]
            second_descriptors = [
                bytes(peer.descriptor_bytes)
                for _, _, peer, _ in uncomputed_pairs]
            try:
                matches = compare_descriptor_pairs(
                    first_descriptors, second_descriptors,
                    int(uncomputed_pairs[0][3][0].ring_count),
                    int(uncomputed_pairs[0][3][0].sector_count))
            except ValueError:
                # Preserve the historical per-pair rejection behavior for a
                # malformed descriptor without affecting valid pairs in the
                # same bounded callback batch.
                matches = []
                for _, _, peer, own_entry in uncomputed_pairs:
                    own = own_entry[0]
                    try:
                        matches.append(compare_descriptors(
                            bytes(own.descriptor_bytes),
                            bytes(peer.descriptor_bytes),
                            int(own.ring_count), int(own.sector_count)))
                    except ValueError:
                        matches.append(None)
            gate_candidates = {}
            for (peer_key, own_key, peer, own_entry), match in zip(
                    uncomputed_pairs, matches):
                if match is None:
                    continue
                self.counters['candidate_comparisons'] += 1
                self.matches[(peer_key, own_key)] = match
                self.best_similarity = max(
                    self.best_similarity, match.similarity)
                self.best_margin = max(self.best_margin, match.margin)
                self.best_known_fraction = max(
                    self.best_known_fraction, match.known_fraction)
                rejection_reason = None
                if match.known_fraction < 0.12:
                    rejection_reason = 'KNOWN_FRACTION_BELOW_GATE'
                self.descriptor_gate_status[
                    (peer_key, own_key)] = rejection_reason
                if rejection_reason is not None:
                    self.counters['cheap_rejections'] += 1
                    self.gate_rejection_counts[rejection_reason] += 1
                    continue
                # Similarity and margin are advisory ranking metadata.  They
                # are intentionally not allowed to delete a possible native
                # resolution crop correspondence before geometric checking.
                gate_candidates[(peer_key, own_key)] = match

            # A weak match is not useful merely because it cleared the
            # minimum margin by a few thousandths.  If a competing keyframe
            # for either side has nearly the same similarity, classify the
            # survivor as ambiguous and keep it out of the bounded crop
            # budget.  The evidence lease may still be advertised, allowing
            # later genuinely different views to arrive.
            for pair_key, match in gate_candidates.items():
                peer_key, own_key = pair_key
                alternatives = [other for other_pair, other in
                                gate_candidates.items()
                                if other_pair != pair_key and
                                (other_pair[0] == peer_key or
                                 other_pair[1] == own_key)]
                if not descriptor_match_is_ambiguous(
                        match, alternatives, self.margin_gate):
                    continue
                self.descriptor_ambiguous_pairs.add((peer_key, own_key))
                self.counters['descriptor_ambiguity_advisories'] += 1
                self._write_physical_evidence_diagnostic(
                    'DESCRIPTOR_MATCH_AMBIGUOUS',
                    own_keyframe_id=str(own_key),
                    peer_keyframe_id=str(peer_key),
                    descriptor_similarity=float(match.similarity),
                    descriptor_margin=float(match.margin),
                    reason='LOW_MARGIN_NEAR_TWIN',
                    action='DEFER_UNLESS_NO_UNAMBIGUOUS_CANDIDATE')
            for (peer_key, own_key), match in gate_candidates.items():
                if self.descriptor_gate_status.get((peer_key, own_key)):
                    continue
                self.counters['cheap_candidates'] += 1
                self.descriptor_gate_survivors.add((peer_key, own_key))
                self._record_diagnostic_event(
                    'DESCRIPTOR_GATE_SURVIVED', peer_key=peer_key,
                    own_key=own_key, similarity=float(match.similarity),
                    margin=float(match.margin),
                    known_fraction=float(match.known_fraction),
                    descriptor_advisory=True)

        eligible = []
        self._update_temporal_support_cache(changed_pair_keys)
        for (peer_key, own_key), match in list(self.matches.items()):
            rejection_reason = self.descriptor_gate_status.get(
                (peer_key, own_key))
            if rejection_reason is not None:
                continue
            peer = self.peer_descriptors.get(peer_key)
            own_entry = self.keyframes.get(own_key)
            if peer is None or own_entry is None:
                continue
            own = own_entry[0]
            pair_key = (peer_key, own_key)
            support_count = self.temporal_support_cache.get(pair_key)
            if support_count is None:
                continue
            confirmations = self.confirmations.setdefault(
                pair_key, set())
            confirmations.add((own_key, peer_key))
            if pair_key in changed_pair_keys:
                self.counters['temporal_support_evaluations'] += 1
            self.counters['temporal_support_max'] = max(
                self.counters['temporal_support_max'], support_count)
            if support_count < self.minimum_confirmations:
                if pair_key not in self.temporal_gate_rejected_pairs:
                    self.temporal_gate_rejected_pairs.add(pair_key)
                    self.counters['temporal_gate_rejections'] += 1
                    self.temporal_gate_rejection_counts[
                        'INSUFFICIENT_CONFIRMATIONS'] += 1
                continue
            self.temporal_gate_survivors.add((peer_key, own_key))
            if self.first_candidate_wall is None:
                self.first_candidate_wall = time.monotonic()
            eligible.append((
                -float(match.similarity), peer_key, own_key, peer, own))
        if not eligible:
            # A cheap descriptor survivor without temporal eligibility is not
            # actionable evidence: no crop can be requested yet.  Do not
            # renew the allocator's navigation lease for this condition,
            # otherwise repeated descriptor updates can hold both robots
            # indefinitely while producing no new registration input.
            # Temporal, geometric, and three-inlier gates remain unchanged.
            if any(pair_key in changed_pair_keys for pair_key in
                   self.descriptor_gate_survivors):
                self._write_physical_evidence_diagnostic(
                    'EVIDENCE_OPPORTUNITY_NOT_ACTIONABLE',
                    reason='NO_TEMPORALLY_ELIGIBLE_REQUESTABLE_CANDIDATE')
            return
        # This is the earliest safe encounter signal: the pair has survived
        # the existing temporal support gate, but no crop request, candidate
        # ordering, or verification-budget slot has been consumed yet.
        self._advertise_evidence_opportunity(len(eligible))
        eligible.sort(key=lambda value: (
            0 if (value[2], value[1]) in self.evidence_pairs else 1,
            value[0], value[1], value[2]))
        own_crops = {key: entry[1] for key, entry in self.keyframes.items()}
        physical_eligible, early_duplicates = (
            deduplicate_physical_candidates_with_reasons(eligible, own_crops))
        self.counters['physical_candidate_duplicates_suppressed'] += (
            len(early_duplicates))
        for identity, candidate in early_duplicates:
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_SKIPPED',
                candidate=self._candidate_diagnostic(
                    candidate, status='SKIPPED',
                    reason='IDENTICAL_CONTENT_PAIR_ALREADY_ATTEMPTED',
                    compact=True),
                physical_identity=list(identity),
                reason='IDENTICAL_CONTENT_PAIR_ALREADY_ATTEMPTED',
                stage='PRE_CROP_REQUEST')
        for candidate in physical_eligible:
            physical_key = self._candidate_physical_key(candidate)
            if physical_key in self._diagnosed_physical_candidates:
                continue
            self._diagnosed_physical_candidates.add(physical_key)
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_OBSERVED',
                candidate=self._candidate_diagnostic(candidate, compact=True))
        added, duplicates = accumulate_physical_candidates(
            self.pending_candidate_pairs, physical_eligible, own_crops)
        for identity, candidate in added:
            self.counters['pending_candidate_additions'] += 1
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_ADDED',
                identity=list(identity), candidate=self._candidate_diagnostic(
                    candidate, status='PENDING', compact=True))
        for identity, candidate in duplicates:
            self.counters['physical_candidate_duplicates_suppressed'] += 1
            if identity in self._diagnosed_duplicate_physical_candidates:
                continue
            self._diagnosed_duplicate_physical_candidates.add(identity)
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_DUPLICATE_SUPPRESSED',
                identity=list(identity), duplicate_record='first_observation',
                candidate=self._candidate_diagnostic(
                    candidate, status='DUPLICATE',
                    reason='IDENTICAL_CONTENT_PAIR_ALREADY_ATTEMPTED',
                    compact=True),
                reason='IDENTICAL_CONTENT_PAIR_ALREADY_ATTEMPTED',
                stage='PENDING_POOL')
        for identity, candidate in list(self.pending_candidate_pairs.items()):
            if len(candidate) == 5:
                _, peer_key, own_key, _, _ = candidate
            else:
                peer_key, own_key, _, _ = candidate
            if peer_key in self.peer_descriptors and own_key in self.keyframes:
                continue
            self.pending_candidate_pairs.pop(identity, None)
            self.counters['pending_candidate_removals'] += 1
            self._write_physical_evidence_diagnostic(
                'PENDING_CANDIDATE_REMOVED', identity=list(identity),
                candidate=self._candidate_diagnostic(
                    candidate, status='REMOVED', reason='KEYFRAME_EXPIRED',
                    compact=True))
        pending_candidates = list(self.pending_candidate_pairs.values())
        pending_candidates.sort(key=lambda value: (
            0 if (value[2], value[1]) in self.evidence_pairs else 1,
            value[0], value[1], value[2]))
        selected = []
        own_centres = []
        peer_centres = []
        # Requests are serialized at publication time but their responses can
        # be delayed.  Treat already-in-flight physical views as part of the
        # current acquisition set so a newly observed candidate cannot be
        # selected next to a view whose registration is still pending.
        inflight_own_centres = []
        inflight_peer_centres = []
        for inflight in self.request_candidate_by_request_key.values():
            inflight_peer_key, inflight_own_key, inflight_peer, _ = (
                self._candidate_fields(inflight))
            inflight_entry = self.keyframes.get(inflight_own_key)
            if inflight_entry is None:
                continue
            inflight_crop = inflight_entry[1]
            inflight_own_centres.append(np.array([
                inflight_crop.origin_x +
                inflight_crop.values.shape[1] * inflight_crop.resolution / 2.0,
                inflight_crop.origin_y +
                inflight_crop.values.shape[0] * inflight_crop.resolution / 2.0]))
            inflight_peer_centres.append(np.array([
                float(inflight_peer.crop_origin_x),
                float(inflight_peer.crop_origin_y)]))
        for _, peer_key, own_key, peer, own in pending_candidates:
            if any(item[1] == peer_key or item[2] == own_key
                   for item in selected):
                continue
            own_crop = self.keyframes[own_key][1]
            own_centre = np.array([
                own_crop.origin_x + own_crop.values.shape[1] * own_crop.resolution / 2.0,
                own_crop.origin_y + own_crop.values.shape[0] * own_crop.resolution / 2.0])
            peer_centre = np.array([float(peer.crop_origin_x),
                                    float(peer.crop_origin_y)])
            if not candidate_views_are_spatially_separated(
                    own_centre, peer_centre,
                    own_centres + inflight_own_centres,
                    peer_centres + inflight_peer_centres):
                continue
            selected.append((peer_key, own_key, peer, own))
            own_centres.append(own_centre)
            peer_centres.append(peer_centre)
            if len(selected) >= self.max_evidence_constraints:
                break
        self.active_candidate_pairs = selected
        self.counters['candidate_selections'] += 1
        self._record_diagnostic_event(
            'CANDIDATE_SELECTION', selected_count=len(selected),
            candidate_count=len(pending_candidates),
            minimum_constraints=self.min_consistent_constraints,
            selected_pairs=[{'peer_key': item[0], 'own_key': item[1]}
                            for item in selected])
        selected_identity = []
        for candidate in selected:
            selected_identity.append(self._candidate_diagnostic_reference(
                candidate, status='SELECTED'))
        centres = []
        for candidate in selected:
            own_crop = self.keyframes[candidate[1]][1]
            centres.append(self._crop_geometry(
                own_crop, candidate[1], self.keyframes[candidate[1]][0].map_epoch,
                self.keyframes[candidate[1]][0].checksum)['center'])
        baseline = 0.0
        if len(centres) > 1:
            baseline = float(np.max(np.linalg.norm(
                np.asarray(centres)[:, None, :] - np.asarray(centres)[None, :, :],
                axis=2)))
        selected_pair_ids = {(candidate[0], candidate[1])
                             for candidate in selected}
        self._write_physical_evidence_diagnostic(
            'SELECTION_ATTEMPT',
            candidate_pool=[self._candidate_diagnostic_reference(
                candidate, status='PENDING')
                for candidate in pending_candidates],
            selected_candidates=selected_identity,
            pending_candidates=[self._candidate_diagnostic_reference(
                candidate, status='PENDING')
                for candidate in pending_candidates
                if (candidate[1], candidate[2]) not in selected_pair_ids],
            measured_spatial_baseline_m=baseline,
            minimum_spatial_baseline_m=0.75,
            selected_count=len(selected),
            candidate_pool_count=len(pending_candidates),
            state_transition='SELECTED' if selected else 'PENDING')

        batch_ready = crop_batch_is_ready(
            len(selected), self.min_consistent_constraints)
        if not batch_ready:
            self.counters['candidate_selection_deferrals'] += 1
            self._record_diagnostic_event(
                'CANDIDATE_SELECTION_DEFERRED',
                reason='INSUFFICIENT_SPATIAL_CONSTRAINTS',
                selected_count=len(selected),
                minimum_constraints=self.min_consistent_constraints)
        # A partial selection is intentionally requestable.  It remains
        # pending, while later descriptor callbacks can add a physically
        # distinct candidate to the same bounded selection.
        self.negotiation_started = bool(selected)
        # Verification is deliberately serialized.  A crop that is merely a
        # descriptor survivor must not consume one of the consensus slots
        # before its unchanged geometric quality gate has passed.
        if selected:
            self._begin_evidence_acquisition()
            # A batch can open before the next spatially distinct candidate
            # arrives.  Keep the active batch requestable on subsequent map /
            # descriptor selection callbacks; otherwise the batch remains
            # stuck in WAITING after its initial empty request attempt.
            self._schedule_active_evidence_request()

    def _schedule_active_evidence_request(self):
        """Request newly available evidence without reopening a batch."""
        if (self.evidence_acquisition_started and
                not self.batch_proposal_published and
                not getattr(self, '_registration_shutdown', False)):
            self._request_next_candidate_verification()

    @staticmethod
    def _candidate_fields(candidate):
        if len(candidate) == 5:
            _, peer_key, own_key, peer, own = candidate
        else:
            peer_key, own_key, peer, own = candidate
        return peer_key, own_key, peer, own

    def _keyframe_viewpoint(self, keyframe_id):
        """Return the retained local-odometry viewpoint for one keyframe."""
        viewpoint = getattr(self, 'keyframe_viewpoints', {}).get(keyframe_id)
        if viewpoint is None:
            return None
        return tuple(float(value) for value in viewpoint[:2])

    def _keyframe_pose(self, keyframe_id):
        """Return the complete retained local-odometry pose, if available."""
        viewpoint = getattr(self, 'keyframe_viewpoints', {}).get(keyframe_id)
        if viewpoint is None:
            return None
        return tuple(float(value) for value in viewpoint[:3])

    @staticmethod
    def _descriptor_viewpoint(descriptor):
        """Read peer-published physical viewpoint metadata without inference."""
        if descriptor is None or not bool(getattr(
                descriptor, 'viewpoint_available', False)):
            return None
        values = (
            getattr(descriptor, 'viewpoint_x', None),
            getattr(descriptor, 'viewpoint_y', None),
            getattr(descriptor, 'viewpoint_yaw', None),
        )
        try:
            if not all(math.isfinite(float(value)) for value in values):
                return None
        except (TypeError, ValueError):
            return None
        return tuple(float(value) for value in values)

    def _batch_snapshot(self):
        own = {}
        for key, (descriptor, crop) in self.keyframes.items():
            geometry = self._crop_geometry(
                crop, key, descriptor.map_epoch, descriptor.checksum)
            own[str(key)] = {
                'timestamp_ns': self._stamp_ns(descriptor),
                'center': geometry['center'],
            }
        peer = {
            str(key): {
                'timestamp_ns': self._stamp_ns(descriptor),
                'center': self._descriptor_geometry(descriptor)['center'],
            }
            for key, descriptor in self.peer_descriptors.items()}
        return own, peer

    def _candidate_physical_key(self, candidate):
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return None
        own_crop = own_entry[1]
        return (
            physical_crop_identity(
                own_crop, int(own_entry[0].map_epoch),
                int(own_entry[0].checksum)),
            physical_crop_identity(
                GridCrop(
                    values=np.empty((int(peer.crop_height), int(peer.crop_width)),
                                   dtype=np.int16),
                    resolution=float(peer.resolution),
                    origin_x=float(peer.crop_origin_x),
                    origin_y=float(peer.crop_origin_y),
                    origin_yaw=float(getattr(peer, 'crop_origin_yaw', 0.0))),
                int(peer.map_epoch), int(peer.checksum)))

    def _candidate_physical_geometry_key(self, candidate):
        """Return the revision-independent geometry of one candidate pair."""
        own_crops = {key: value[1] for key, value in self.keyframes.items()}
        return physical_candidate_geometry_identity(candidate, own_crops)

    def _candidate_intrinsically_immature(self, candidate):
        """Return whether a candidate endpoint is known to be immature.

        Local maturity is computed from the immutable crop already retained.
        Peer maturity is read from descriptor metadata when available.  A
        descriptor from an older interface that lacks those fields remains
        schedulable because its peer maturity is unknown, not known-bad.
        This is a transient pre-request scheduling filter: it does not add a
        rejection identity or blacklist either keyframe.
        """
        peer_key, own_key, peer, _ = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return False
        own_mature, own_details = consensus_crop_maturity(
            own_entry[1], self.consensus_min_known_fraction,
            self.consensus_min_occupied_cells)
        peer_known = getattr(peer, 'crop_known_fraction', None)
        peer_occupied = getattr(peer, 'crop_occupied_cells', None)
        peer_known = (None if peer_known is None else float(peer_known))
        peer_occupied = (None if peer_occupied is None else int(peer_occupied))
        peer_mature = None
        if (peer_known is not None and peer_occupied is not None and
                math.isfinite(peer_known)):
            peer_mature = (
                peer_known >= self.consensus_min_known_fraction and
                peer_occupied >= self.consensus_min_occupied_cells)
        if not own_mature or peer_mature is False:
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_SKIPPED',
                candidate=self._candidate_diagnostic(
                    candidate, status='SKIPPED',
                    reason='IMMATURE_EVIDENCE_MAP', compact=True),
                reason='IMMATURE_EVIDENCE_MAP', stage='PRE_CROP_REQUEST',
                own_maturity=own_details,
                peer_maturity={
                    'known_fraction': peer_known,
                    'occupied_cells': peer_occupied,
                    'minimum_known_fraction': self.consensus_min_known_fraction,
                    'minimum_occupied_cells': self.consensus_min_occupied_cells,
                })
            self.counters['immature_candidates_not_scheduled'] += 1
            return True
        return False

    def _geometry_rejected_in_active_batch(self, geometry_key):
        """Return whether geometry was rejected in this acquisition batch.

        Geometry-only suppression is deliberately scoped to one bounded
        verification batch.  Permanent suppression conflated a transient
        registration failure with a bad physical view and prevented later
        independent keyframes from restoring a valid three-inlier set.
        """
        return (
            geometry_key in self.rejected_physical_geometry_keys and
            self.rejected_physical_geometry_batches.get(geometry_key) ==
            self.verification_batches.batch_id)

    def _geometry_attempted_in_active_batch(self, geometry_key):
        """Return whether a crop footprint is already admitted this batch."""
        return (self.attempted_physical_geometry_batches.get(geometry_key) ==
                self.verification_batches.batch_id)

    def _candidate_is_novel_for_reentry(self, candidate):
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return False
        own_geometry = self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)
        peer_geometry = self._descriptor_geometry(peer)
        physical_key = self._candidate_physical_key(candidate)
        return self.verification_batches.is_novel(
            own_key, peer_key,
            self._stamp_ns(own_entry[0]), self._stamp_ns(peer),
            own_geometry['center'], peer_geometry['center'],
            attempted_pairs=self.candidate_verification_attempted,
            rejected_physical=(
                physical_key in self.rejected_physical_evidence_keys or
                self._geometry_rejected_in_active_batch(
                    self._candidate_physical_geometry_key(candidate))))

    def _queue_crop_request(self, peer_key, own_key, peer, own,
                            batch_id, correlation_id):
        request_key = (peer_key, int(peer.checksum))
        if request_key in self.pending_requests:
            self.counters['crop_request_duplicates_suppressed'] += 1
            return False
        self.pending_requests.add(request_key)
        self.counters['crop_requests_queued'] += 1
        self.request_own_by_peer_key[peer_key] = own_key
        self.request_own_by_request_key[request_key] = own_key
        self.request_candidate_by_request_key[request_key] = (
            peer_key, own_key, peer, own)
        self.request_metadata_by_request_key[request_key] = {
            'acquisition_batch_id': int(batch_id),
            'candidate_correlation_id': str(correlation_id),
            'verification_attempt_id': str(correlation_id),
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
            'request_timestamp_ns': int(self._stamp_ns(own)),
            'own_keyframe_creation_timestamp_ns': int(self._stamp_ns(own)),
            'peer_keyframe_creation_timestamp_ns': int(self._stamp_ns(peer)),
        }
        request = LocalMapCropRequest()
        request.header = own.header
        request.requester_robot_id = self.robot_id
        request.source_robot_id = self.peer_robot_id
        request.keyframe_id = peer_key
        request.descriptor_checksum = int(peer.checksum)
        self.request_pub.publish(request)
        self.counters['crop_requests_sent'] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_REQUEST_SENT',
            request_key=[peer_key, int(peer.checksum)],
            **self.request_metadata_by_request_key[request_key],
            candidate=self._candidate_diagnostic(
                (peer_key, own_key, peer, own), status='REQUESTED',
                compact=True))
        self._record_diagnostic_event(
            'CROP_REQUEST_PUBLISHED', peer_key=peer_key,
            own_key=own_key, descriptor_checksum=int(peer.checksum),
            peer_epoch=int(peer.map_epoch),
            request_header_stamp_ns=self._stamp_ns(request))
        return True

    def _candidate_is_distinct_from_evidence(self, candidate):
        """Apply the existing physical-spacing rule to a next candidate."""
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return False
        own_center = np.asarray(self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)['center'], dtype=np.float64)
        peer_center = np.asarray(
            self._descriptor_geometry(peer)['center'], dtype=np.float64)
        own_centers = []
        peer_centers = []
        for evidence_own_key, evidence_peer_key in self.evidence_pairs:
            evidence_own = self.keyframes.get(evidence_own_key)
            evidence_peer = self.peer_descriptors.get(evidence_peer_key)
            if evidence_own is None or evidence_peer is None:
                continue
            own_centers.append(np.asarray(self._crop_geometry(
                evidence_own[1], evidence_own_key,
                evidence_own[0].map_epoch, evidence_own[0].checksum)[
                    'center'], dtype=np.float64))
            peer_centers.append(np.asarray(
                self._descriptor_geometry(evidence_peer)['center'],
                dtype=np.float64))
        # A rejected attempt is not accepted evidence and must not become a
        # permanent spatial exclusion.  Exact rejected physical footprints
        # remain suppressed by the physical-evidence identities above.  Do
        # not also block every later keyframe within the acquisition spacing:
        # that can starve a valid third view when a robot advances through a
        # narrow environment in sub-spacing keyframe increments.  In-flight
        # views are still included below so a fast descriptor callback cannot
        # queue aliases before the admitted registration is processed.
        for inflight in getattr(
                self, 'request_candidate_by_request_key', {}).values():
            inflight_peer_key, inflight_own_key, inflight_peer, _ = (
                self._candidate_fields(inflight))
            inflight_own = self.keyframes.get(inflight_own_key)
            if inflight_own is not None:
                own_centers.append(np.asarray(self._crop_geometry(
                    inflight_own[1], inflight_own_key,
                    inflight_own[0].map_epoch,
                    inflight_own[0].checksum)['center'], dtype=np.float64))
            peer_centers.append(np.asarray(
                self._descriptor_geometry(inflight_peer)['center'],
                dtype=np.float64))
        # The production spatial-baseline gate is measured from source/own
        # crop centres.  Do not spend verification attempts pairing one local
        # view with many peer keyframes: a local crop that is near any
        # accepted source view cannot increase that baseline.  This is an
        # acquisition-order filter only; registration and consensus gates
        # remain unchanged.
        spacing = float(getattr(self, 'verification_novelty_spacing_m', 0.40))
        if own_centers and any(np.linalg.norm(own_center - prior) < spacing
                               for prior in own_centers):
            return False
        if peer_centers and any(np.linalg.norm(peer_center - prior) < spacing
                                for prior in peer_centers):
            return False
        return True

    def _candidate_spatial_novelty_key(self, candidate):
        """Prefer displaced views when choosing the next crop request."""
        peer_key, own_key, peer, _ = self._candidate_fields(candidate)
        own_entry = self.keyframes.get(own_key)
        if own_entry is None:
            return (0.0, 0.0, 0.0, str(peer_key), str(own_key))
        own_center = np.asarray(self._crop_geometry(
            own_entry[1], own_key, own_entry[0].map_epoch,
            own_entry[0].checksum)['center'], dtype=np.float64)
        peer_center = np.asarray(
            self._descriptor_geometry(peer)['center'], dtype=np.float64)
        reference_own = []
        reference_peer = []
        for evidence_own_key, evidence_peer_key in self.evidence_pairs:
            evidence_own = self.keyframes.get(evidence_own_key)
            evidence_peer = self.peer_descriptors.get(evidence_peer_key)
            if evidence_own is not None:
                reference_own.append(np.asarray(self._crop_geometry(
                    evidence_own[1], evidence_own_key,
                    evidence_own[0].map_epoch,
                    evidence_own[0].checksum)['center'], dtype=np.float64))
            if evidence_peer is not None:
                reference_peer.append(np.asarray(
                    self._descriptor_geometry(evidence_peer)['center'],
                    dtype=np.float64))
        own_distance = min(
            (float(np.linalg.norm(own_center - reference))
             for reference in reference_own), default=0.0)
        peer_distance = min(
            (float(np.linalg.norm(peer_center - reference))
             for reference in reference_peer), default=0.0)
        similarity = float(candidate[0]) if len(candidate) == 5 else 0.0
        # A useful cross-robot evidence view must move on both sides of the
        # candidate pair.  Prioritising own_distance first selected very far
        # local crops whose peer crop was still inside the existing physical
        # baseline, starving the consensus pool of independent views.  Rank
        # by the weaker displacement first; this changes acquisition order
        # only and leaves every geometric/consensus gate unchanged.
        balanced_distance = min(own_distance, peer_distance)
        total_distance = max(own_distance, peer_distance)
        geometry_key = self._candidate_physical_geometry_key(candidate)
        reuse_count = 0
        if geometry_key is not None:
            reuse_count = sum(
                int(self.attempted_physical_view_reuse_counts.get(view, 0))
                for view in geometry_key)
        # If a high-scoring descriptor repeatedly leads to geometric
        # rejection, ranking another pair with the same local or peer crop
        # first can starve genuinely new views.  Prefer never-attempted
        # physical views, then retain the existing balanced spatial novelty
        # ordering.  This is scheduling only: all registration and consensus
        # gates and all exact duplicate/rejection sets are unchanged.
        return (reuse_count, -balanced_distance, -total_distance, similarity,
                str(peer_key), str(own_key))

    def _rank_next_verification_candidate(self):
        candidates = []
        for candidate in self.pending_candidate_pairs.values():
            peer_key, own_key, peer, own = self._candidate_fields(candidate)
            pair_key = (own_key, peer_key)
            request_key = (peer_key, int(peer.checksum))
            if pair_key in self.candidate_verification_attempted:
                continue
            if pair_key in self.evidence_pairs:
                continue
            if request_key in self.pending_requests:
                continue
            # Maturity is an intrinsic endpoint property.  Check it before
            # any crop request, registration-worker submission, or batch-slot
            # increment.  The candidate remains in the pending pool and may
            # become schedulable if a later descriptor carries mature
            # metadata; no rejection/blacklist state is written here.
            if self._candidate_intrinsically_immature(candidate):
                continue
            physical_key = self._candidate_physical_key(candidate)
            if physical_key in self.rejected_physical_evidence_keys:
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_EVIDENCE_PREVIOUSLY_REJECTED',
                        compact=True),
                    reason='PHYSICAL_EVIDENCE_PREVIOUSLY_REJECTED')
                continue
            geometry_key = self._candidate_physical_geometry_key(candidate)
            if candidate_reuses_accepted_physical_view(
                    candidate, self.evidence_physical_geometry_keys,
                    {key: value[1] for key, value in self.keyframes.items()}):
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_VIEW_ALREADY_ACCEPTED', compact=True),
                    reason='PHYSICAL_VIEW_ALREADY_ACCEPTED')
                continue
            if geometry_key in self.evidence_physical_geometry_keys:
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED',
                        compact=True),
                    reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED')
                continue
            if self._geometry_rejected_in_active_batch(geometry_key):
                self.counters['physical_geometry_rejections_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_GEOMETRY_PREVIOUSLY_REJECTED',
                        compact=True),
                    reason='PHYSICAL_GEOMETRY_PREVIOUSLY_REJECTED')
                continue
            if self._geometry_attempted_in_active_batch(geometry_key):
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='PHYSICAL_GEOMETRY_ALREADY_ATTEMPTED_IN_BATCH',
                        compact=True),
                    reason='PHYSICAL_GEOMETRY_ALREADY_ATTEMPTED_IN_BATCH')
                continue
            if not self._candidate_is_distinct_from_evidence(candidate):
                self._write_physical_evidence_diagnostic(
                    'CANDIDATE_VERIFICATION_SKIPPED',
                    candidate=self._candidate_diagnostic(
                        candidate, status='SKIPPED',
                        reason='TOO_CLOSE_TO_ACCEPTED_EVIDENCE', compact=True),
                    reason='TOO_CLOSE_TO_ACCEPTED_EVIDENCE')
                continue
            candidates.append(candidate)
        candidates = prioritize_unambiguous_candidates(
            candidates, self.descriptor_ambiguous_pairs)
        # Keep the verification budget bounded while ensuring its initial
        # contents cover distinct local/peer keyframes.  The helper retains
        # descriptor score as the ordering signal, then applies the existing
        # spatial-novelty tie-break within that bounded diverse set.
        ranked = bounded_candidate_verification_order(
            candidates, self.candidate_verification_attempted,
            min(len(candidates), self.candidate_verification_budget))
        ranked.sort(key=self._candidate_spatial_novelty_key)
        return None if not ranked else ranked[0]

    def _request_next_candidate_verification(self):
        if (self.batch_proposal_published or
                not self.evidence_acquisition_started or
                getattr(self, '_registration_shutdown', False)):
            return False
        # Do not create more crop requests while the bounded registration
        # FIFO is at its service watermark.  This preserves fresh evidence
        # for the worker instead of filling the queue and dropping responses.
        if len(self._registration_pending_contexts) >= \
                self._registration_backpressure_depth:
            self._registration_backpressure_events += 1
            self._record_diagnostic_event(
                'REGISTRATION_BACKPRESSURE',
                queue_depth=len(self._registration_pending_contexts),
                service_watermark=self._registration_backpressure_depth)
            return False
        if self.candidate_verification_batch_attempts >= self.candidate_verification_budget:
            # A bounded batch must drain all responses already admitted to the
            # worker before it is closed.  Otherwise a late result is applied
            # after a newer batch opens and its rejection/acceptance is
            # attributed to the wrong acquisition window.
            if self._verification_worker_busy():
                self.counters['candidate_verification_budget_waits'] += 1
                self._record_diagnostic_event(
                    'CANDIDATE_VERIFICATION_BUDGET_WAITING_FOR_WORKER',
                    acquisition_batch_id=self.verification_batches.batch_id,
                    batch_attempts=self.candidate_verification_batch_attempts,
                    budget=self.candidate_verification_budget,
                    queue_depth=len(self._registration_pending_contexts),
                    inflight=bool(self._registration_future is not None))
                return False
            self.counters['candidate_verification_budget_exhausted'] += 1
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_BUDGET_EXHAUSTED',
                attempts=self.candidate_verification_attempts,
                batch_attempts=self.candidate_verification_batch_attempts,
                acquisition_batch_id=self.verification_batches.batch_id,
                accepted_count=len(self.evidence_pairs),
                budget=self.candidate_verification_budget)
            self._end_evidence_acquisition('BUDGET_EXHAUSTED')
            return False
        candidate = self._rank_next_verification_candidate()
        if candidate is None:
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_VERIFICATION_WAITING',
                attempts=self.candidate_verification_attempts,
                accepted_count=len(self.evidence_pairs))
            # A completed rejection with no pending request and no worker
            # backlog has no evidence work in flight.  End this lease now so
            # the allocator can resume navigation and create a genuinely new
            # paired viewpoint; the next descriptor callback may reopen a
            # batch when a requestable candidate exists.
            if (not self.pending_requests and
                    not self._verification_worker_busy()):
                self._end_evidence_acquisition(
                    'NO_REQUESTABLE_NOVEL_EVIDENCE')
            return False
        peer_key, own_key, peer, own = self._candidate_fields(candidate)
        pair_key = (own_key, peer_key)
        geometry_key = self._candidate_physical_geometry_key(candidate)
        if geometry_key is not None:
            for view in geometry_key:
                self.attempted_physical_view_reuse_counts[view] = (
                    int(self.attempted_physical_view_reuse_counts.get(view, 0))
                    + 1)
            self.attempted_physical_geometry_batches[geometry_key] = (
                self.verification_batches.batch_id)
        self.candidate_verification_attempted.add(pair_key)
        self.candidate_verification_attempts += 1
        self.candidate_verification_batch_attempts += 1
        self.verification_batches.batch_attempts += 1
        self.verification_attempt_sequence += 1
        correlation_id = (
            f'{self.robot_id}-b{self.verification_batches.batch_id:04d}-'
            f'a{self.candidate_verification_batch_attempts:04d}-'
            f'c{self.verification_attempt_sequence:06d}')
        self.counters['candidate_verification_attempts'] = (
            self.candidate_verification_attempts)
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_REQUESTED',
            candidate=self._candidate_diagnostic(
                candidate, status='REQUESTED', compact=True),
            attempt=self.candidate_verification_attempts,
            batch_attempt=self.candidate_verification_batch_attempts,
            budget=self.candidate_verification_budget,
            acquisition_batch_id=self.verification_batches.batch_id,
            candidate_correlation_id=correlation_id,
            verification_attempt_id=correlation_id)
        return self._queue_crop_request(
            peer_key, own_key, peer, own,
            self.verification_batches.batch_id, correlation_id)

    def _verification_worker_busy(self):
        """Whether admitted registration work still needs to be drained."""
        return (self._registration_future is not None or
                bool(self._registration_pending_contexts))

    def _request_additional_evidence_candidates(self):
        """Request one more candidate; retained as a compatibility wrapper."""
        if not self.evidence_acquisition_started:
            return 0
        return int(self._request_next_candidate_verification())

    def _begin_evidence_acquisition(self):
        if self.evidence_acquisition_started or self.batch_proposal_published:
            return
        own_snapshot, peer_snapshot = self._batch_snapshot()
        now = time.monotonic()
        initial = self.verification_batches.batch_id == 0
        if not initial and not self.verification_batches.waiting_for_novelty:
            return
        if not initial:
            novel = any(
                self._candidate_is_novel_for_reentry(candidate)
                for candidate in self.pending_candidate_pairs.values())
            if not novel:
                self.counters['verification_novelty_deferrals'] += 1
                self._write_physical_evidence_diagnostic(
                    'VERIFICATION_BATCH_WAITING_FOR_NOVELTY',
                    acquisition_batch_id=self.verification_batches.batch_id,
                    pending_candidate_count=len(self.pending_candidate_pairs),
                    reason='NO_MATERIAL_NEW_SPATIAL_EVIDENCE')
                return
        batch_id = self.verification_batches.open(
            now, own_snapshot, peer_snapshot, initial=initial)
        if batch_id is None:
            if self.verification_batches.lifetime_expired:
                self.counters['verification_lifetime_expired'] += 1
            return
        if not initial:
            self.counters['verification_batch_reentries'] += 1
        self.counters['verification_batches_opened'] += 1
        self.candidate_verification_batch_attempts = 0
        self.evidence_acquisition_started = True
        self._evidence_opportunity_deadline_wall = None
        self.evidence_acquisition_deadline_wall = (
            time.monotonic() + self.evidence_acquisition_window_s)
        self._record_diagnostic_event(
            'EVIDENCE_ACQUISITION_STARTED',
            acquisition_batch_id=batch_id,
            constraints_accumulated=len(self.evidence_physical_keys),
            window_s=self.evidence_acquisition_window_s)
        self._write_physical_evidence_diagnostic(
            'VERIFICATION_BATCH_OPENED',
            acquisition_batch_id=batch_id,
            batch_attempts=0,
            constraints_accumulated=len(self.evidence_physical_keys),
            window_s=self.evidence_acquisition_window_s,
            deadline_wall=self.evidence_acquisition_deadline_wall)
        requested = self._request_next_candidate_verification()
        # A descriptor/keyframe novelty signal is not itself an evidence
        # acquisition opportunity.  If the existing request-ranking path
        # cannot produce a candidate (for example because every pair is too
        # close to accepted evidence), do not hold local navigation for the
        # entire batch window.  This lets both robots continue to acquire
        # genuinely displaced views.  A busy registration worker remains
        # protected by the normal bounded lease and is not interrupted here.
        if (not requested and self.evidence_acquisition_started and
                self.candidate_verification_batch_attempts == 0 and
                not self._verification_worker_busy()):
            self._end_evidence_acquisition(
                'NO_REQUESTABLE_NOVEL_EVIDENCE')
            return
        # Announce the lease only after a requestable candidate (or already
        # admitted worker backlog) exists.  A novelty-only advisory therefore
        # cannot briefly hold navigation before being discovered unusable.
        self._publish_evidence_status(True)

    def _end_evidence_acquisition(self, reason):
        if not self.evidence_acquisition_started:
            return
        own_snapshot, peer_snapshot = self._batch_snapshot()
        self.evidence_acquisition_started = False
        self._evidence_opportunity_deadline_wall = None
        self._publish_evidence_status(False)
        self.evidence_acquisition_deadline_wall = None
        self.verification_batches.exhaust(
            time.monotonic(), own_snapshot, peer_snapshot)
        self.counters['verification_batches_exhausted'] += 1
        self._write_physical_evidence_diagnostic(
            'VERIFICATION_BATCH_WAITING',
            acquisition_batch_id=self.verification_batches.batch_id,
            batch_attempts=self.candidate_verification_batch_attempts,
            constraints_accumulated=len(self.evidence_physical_keys),
            reason=str(reason),
            state='WAITING_FOR_NOVEL_EVIDENCE')
        if len(self.evidence_physical_keys) >= self.min_consistent_constraints:
            self._publish_multi_constraint_proposal()

    def _maybe_finalize_evidence_acquisition(self):
        if not self.evidence_acquisition_started:
            return
        if (self.evidence_acquisition_deadline_wall is None or
                time.monotonic() < self.evidence_acquisition_deadline_wall):
            return
        self._end_evidence_acquisition('WINDOW_EXPIRED')
        self._record_diagnostic_event(
            'EVIDENCE_ACQUISITION_TIMEOUT',
            acquisition_batch_id=self.verification_batches.batch_id,
            constraints_accumulated=len(self.evidence_physical_keys))
        self._write_physical_evidence_diagnostic(
            'EVIDENCE_ACQUISITION_TIMEOUT',
            acquisition_batch_id=self.verification_batches.batch_id,
            constraints_accumulated=len(self.evidence_physical_keys),
            state_transition='FINALIZE')

    def request_callback(self, request):
        if (getattr(self, '_post_handoff_quiesced', False) or
                request.source_robot_id != self.robot_id):
            return
        self.counters['crop_requests_received'] += 1
        self._record_diagnostic_event(
            'CROP_REQUEST_RECEIVED', keyframe_id=request.keyframe_id,
            descriptor_checksum=int(request.descriptor_checksum),
            request_header_stamp_ns=self._stamp_ns(request))
        stored = self.keyframes.get(request.keyframe_id)
        if stored is None:
            self._record_crop_rejection('KEYFRAME_NOT_RETAINED', request)
            return
        self._write_physical_evidence_diagnostic(
            'CROP_REQUEST_RECEIVED',
            request_key=[request.keyframe_id,
                         int(request.descriptor_checksum)],
            source_descriptor=self._descriptor_geometry(stored[0]),
            requester_robot_id=str(request.requester_robot_id),
            source_robot_id=str(request.source_robot_id),
            status='RECEIVED')
        descriptor, crop = stored
        if request.descriptor_checksum and int(descriptor.checksum) != int(
                request.descriptor_checksum):
            self._record_crop_rejection('CHECKSUM_MISMATCH', request)
            return
        crop_message = LocalMapCrop()
        crop_message.header = descriptor.header
        crop_message.source_robot_id = self.robot_id
        crop_message.keyframe_id = request.keyframe_id
        crop_message.map_epoch = descriptor.map_epoch
        crop_message.descriptor_checksum = descriptor.checksum
        crop_message.occupancy_checksum = self._occupancy_checksum(crop.values)
        crop_message.occupancy_grid = self._crop_message(crop, descriptor.header)
        self.crop_cells_sent = max(
            self.crop_cells_sent, len(crop_message.occupancy_grid.data))
        self.crop_pub.publish(crop_message)
        self.counters['crops_sent'] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_SENT',
            keyframe_id=str(request.keyframe_id),
            map_epoch=int(descriptor.map_epoch),
            checksum=int(descriptor.checksum),
            crop=self._crop_geometry(
                crop, request.keyframe_id, descriptor.map_epoch,
                descriptor.checksum),
            status='SENT')
        self.counters['crop_responses_accepted'] += 1
        self._record_diagnostic_event(
            'CROP_RESPONSE_PUBLISHED', keyframe_id=request.keyframe_id,
            map_epoch=int(descriptor.map_epoch),
            descriptor_checksum=int(descriptor.checksum))

    def _record_crop_rejection(self, reason, message, **fields):
        self.counters['crop_response_rejections'] += 1
        self.crop_response_rejection_counts[str(reason)] += 1
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_REJECTED',
            keyframe_id=str(getattr(message, 'keyframe_id', '')),
            map_epoch=int(getattr(message, 'map_epoch', 0)),
            checksum=int(getattr(message, 'descriptor_checksum', 0)),
            reason=str(reason), status='REJECTED', **fields)
        self._record_diagnostic_event(
            'CROP_RESPONSE_REJECTED', reason=str(reason),
            keyframe_id=getattr(message, 'keyframe_id', ''),
            descriptor_checksum=int(getattr(message, 'descriptor_checksum', 0)))

    def _verify_candidate_crop(self, pair_key, candidate, own_crop,
                               received_crop, metadata=None):
        """Run the unchanged single-pair geometric gate before consensus."""
        peer_key, _, _, _ = self._candidate_fields(candidate)
        self.counters['registrations'] += 1
        self.counters['registration_callback_entries'] += 1
        self.registration_callback_depth += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_ENTRY', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1)
        try:
            mature_evidence, maturity = self._consensus_pair_maturity(
                own_crop, received_crop)
            hypotheses = register_crop_hypotheses(
                own_crop, received_crop, backend=self.registration_backend,
                minimum_agreement=(0.0 if mature_evidence else 0.55),
                mrpt_max_kld=self.mrpt_max_kld,
                mrpt_max_modes=self.mrpt_max_modes_per_call,
                mrpt_repetitions=self.mrpt_repetitions_per_pair,
                max_distinct_modes=self.mrpt_max_distinct_modes_per_pair)
            result = max(
                hypotheses,
                key=lambda item: (
                    bool(item.accepted), int(getattr(item, 'mode_support', 1)),
                    float(getattr(item, 'mode_log_weight', -math.inf)),
                    float(item.inlier_ratio), -float(item.residual_m)))
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self.consensus_gate_rejection_counts['REGISTRATION_EXCEPTION'] += 1
            self._record_diagnostic_event(
                'REGISTRATION_CALLBACK_EXCEPTION',
                source='candidate_verification', keyframe_id=peer_key,
                exception=repr(exc))
            raise
        finally:
            self.registration_callback_depth -= 1
            self.counters['registration_callback_exits'] += 1
        self.candidate_verification_results[pair_key] = result
        self.candidate_verification_hypotheses[pair_key] = tuple(hypotheses)
        self._finalize_registration_capture(pair_key, result=result)
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_EXIT', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1,
            accepted=bool(result.accepted), reason=str(result.reason),
            residual_m=float(result.residual_m),
            inlier_ratio=float(result.inlier_ratio),
            projected_error_m=float(result.projected_error_m))
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_RESULT',
            **(metadata or {}),
            candidate=self._candidate_diagnostic(
                candidate, status='GEOMETRIC_ACCEPTED' if result.accepted
                else 'GEOMETRIC_REJECTED', reason=str(result.reason),
                compact=True),
            transform=[float(value) for value in result.transform],
            residual_m=float(result.residual_m),
            median_residual_m=float(result.median_residual_m),
            p95_residual_m=float(result.p95_residual_m),
            inlier_ratio=float(result.inlier_ratio),
            occupied_free_agreement=float(result.occupied_free_agreement),
            overlap_fraction=float(result.overlap_fraction),
            translation_uncertainty_m=float(result.translation_uncertainty_m),
            yaw_uncertainty_rad=float(result.yaw_uncertainty_rad),
            condition_number=float(result.condition_number),
            projected_error_m=float(result.projected_error_m),
            accepted_geometric=bool(result.accepted),
            rejection_reason='' if result.accepted else str(result.reason),
            )
        if result.accepted:
            self.counters['candidate_verification_accepted'] += 1
        else:
            self.counters['candidate_verification_rejected'] += 1
        return result

    @staticmethod
    def _occupancy_checksum(values):
        """Checksum the exact signed occupancy bytes placed on the wire."""
        array = np.asarray(values, dtype=np.int16)
        return int(zlib.crc32(array.astype(np.int8, copy=False).tobytes()) &
                   0xffffffff)

    @staticmethod
    def _crop_message(crop, header):
        message = OccupancyGrid()
        message.header = header
        message.info.resolution = float(crop.resolution)
        message.info.width = int(crop.values.shape[1])
        message.info.height = int(crop.values.shape[0])
        message.info.origin.position.x = float(crop.origin_x)
        message.info.origin.position.y = float(crop.origin_y)
        message.info.origin.orientation.z = math.sin(crop.origin_yaw / 2.0)
        message.info.origin.orientation.w = math.cos(crop.origin_yaw / 2.0)
        message.data = [int(value) for value in crop.values.ravel()]
        return message

    def crop_callback(self, message):
        if (getattr(self, '_post_handoff_quiesced', False) or
                message.source_robot_id != self.peer_robot_id):
            return
        self.counters['crops_received'] += 1
        self._record_diagnostic_event(
            'CROP_RECEIVED', keyframe_id=message.keyframe_id,
            map_epoch=int(message.map_epoch),
            descriptor_checksum=int(message.descriptor_checksum))
        self.crop_cells_received = max(
            self.crop_cells_received, len(message.occupancy_grid.data))
        try:
            received_crop = self._grid_crop_from_message(message.occupancy_grid)
        except (TypeError, ValueError) as exc:
            self._record_crop_rejection('INVALID_CROP_METADATA', message,
                                        error=repr(exc))
            return
        if (int(getattr(message, 'occupancy_checksum', 0)) and
                int(message.occupancy_checksum) != self._occupancy_checksum(
                    received_crop.values)):
            self._record_crop_rejection('OCCUPANCY_CHECKSUM_MISMATCH', message)
            return
        expected_peer = self.peer_descriptors.get(message.keyframe_id)
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_RECEIVED',
            keyframe_id=str(message.keyframe_id),
            map_epoch=int(message.map_epoch),
            checksum=int(message.descriptor_checksum),
            crop=self._crop_geometry(
                received_crop, message.keyframe_id, message.map_epoch,
                message.descriptor_checksum),
            peer_descriptor=(None if expected_peer is None else
                             self._descriptor_geometry(expected_peer)),
            status='RECEIVED')
        self.received_peer_crops[message.keyframe_id] = received_crop
        proposal = self.peer_proposals.get(message.keyframe_id)
        if self.robot_id > self.peer_robot_id and proposal is not None:
            self._try_confirm_pending_proposal(proposal, message)
            return
        # A peer summary is independently verified by *both* robots using the
        # same physical evidence IDs.  This path is deliberately separate
        # from proposal confirmation: no TF or shared map activation occurs
        # until the canonical proposal/ack exchange.
        if (message.keyframe_id in self.peer_summary_source_ids):
            self.peer_summary_source_ids.discard(message.keyframe_id)
            if not self.peer_summary_source_ids:
                self._verify_peer_hypothesis_summary()
            return

        response_request_key = (
            str(message.keyframe_id), int(message.descriptor_checksum))
        request_metadata = self.request_metadata_by_request_key.get(
            response_request_key)
        if response_request_key not in self.pending_requests:
            self.counters['stale_verification_batch_responses'] += 1
            self._record_crop_rejection(
                'STALE_VERIFICATION_BATCH', message,
                metadata=request_metadata,
                response_request_key=list(response_request_key))
            return
        peer_key = message.keyframe_id
        request_key = (peer_key, int(message.descriptor_checksum))
        own_key = self.request_own_by_request_key.get(request_key)
        if own_key is None:
            # Backwards-compatible fallback for confirmation requests, which
            # intentionally carry checksum zero and are keyed by source ID.
            own_key = self.request_own_by_peer_key.get(peer_key)
        if own_key is None or own_key not in self.keyframes:
            self._record_crop_rejection('UNMATCHED_REQUEST_KEY', message)
            return
        expected_peer = self.peer_descriptors.get(peer_key)
        if expected_peer is None:
            self._record_crop_rejection('PEER_DESCRIPTOR_NOT_RETAINED', message)
            return
        if int(message.descriptor_checksum) != int(expected_peer.checksum):
            self._record_crop_rejection('RESPONSE_CHECKSUM_MISMATCH', message)
            return
        if int(message.map_epoch) != int(expected_peer.map_epoch):
            self._record_crop_rejection('RESPONSE_MAP_EPOCH_MISMATCH', message)
            return
        pair_key = (own_key, peer_key)
        physical_key = (
            physical_crop_identity(
                self.keyframes[own_key][1],
                int(self.keyframes[own_key][0].map_epoch),
                int(self.keyframes[own_key][0].checksum)),
            physical_crop_identity(
                received_crop, int(message.map_epoch),
                int(message.descriptor_checksum)))
        if physical_key in self.evidence_physical_keys.values():
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='IDENTICAL_PHYSICAL_EVIDENCE')
            self._record_diagnostic_event(
                'CROP_DUPLICATE_PHYSICAL_EVIDENCE_SUPPRESSED',
                own_key=own_key, peer_key=peer_key,
                descriptor_checksum=int(message.descriptor_checksum))
            return
        candidate = self.request_candidate_by_request_key.get(
            request_key, (peer_key, own_key, expected_peer,
                          self.keyframes[own_key][0]))
        candidate_geometry_key = self._candidate_physical_geometry_key(candidate)
        # A response can arrive after another in-flight request has already
        # accepted the same local or peer crop footprint.  Exact pair
        # deduplication is insufficient here: reusing either physical view
        # creates a second, non-independent registration constraint and can
        # poison the unchanged multi-constraint consensus gate.  Apply the
        # existing acquisition-only physical-view rule again at response
        # time, when the accepted-evidence set is authoritative.
        if candidate_reuses_accepted_physical_view(
                candidate, self.evidence_physical_geometry_keys,
                {key: value[1] for key, value in self.keyframes.items()}):
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='PHYSICAL_VIEW_ALREADY_ACCEPTED')
            self.pending_requests.discard(request_key)
            self.completed_request_keys.add(request_key)
            self.request_candidate_by_request_key.pop(request_key, None)
            self.request_metadata_by_request_key.pop(request_key, None)
            self._request_next_candidate_verification()
            return
        if candidate_geometry_key in self.evidence_physical_geometry_keys:
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_DUPLICATE_SUPPRESSED',
                physical_identity=list(physical_key),
                own_keyframe_id=str(own_key),
                peer_keyframe_id=str(peer_key),
                reason='PHYSICAL_GEOMETRY_ALREADY_ACCEPTED')
            self.pending_requests.discard(request_key)
            self.completed_request_keys.add(request_key)
            self.request_candidate_by_request_key.pop(request_key, None)
            self._request_next_candidate_verification()
            return
        request_metadata = self.request_metadata_by_request_key.get(
            request_key, {})
        candidate = self.request_candidate_by_request_key.get(
            request_key, candidate)
        self.counters['crop_responses_accepted'] += 1
        accepted_metadata = dict(request_metadata or {})
        # Keep the request metadata schema, while making the response's
        # canonical identities authoritative without passing duplicate Python
        # keyword arguments.
        accepted_metadata.update({
            'own_keyframe_id': str(own_key),
            'peer_keyframe_id': str(peer_key),
        })
        self._write_physical_evidence_diagnostic(
            'CROP_RESPONSE_ACCEPTED',
            **accepted_metadata,
            physical_identity=list(physical_key),
            own_crop=self._crop_geometry(
                self.keyframes[own_key][1], own_key,
                self.keyframes[own_key][0].map_epoch,
                self.keyframes[own_key][0].checksum),
            peer_crop=self._crop_geometry(
                received_crop, peer_key, message.map_epoch,
                message.descriptor_checksum),
            constraints_accumulated=len(self.evidence_pairs),
            status='ACCEPTED')
        # Registration is deliberately offloaded from this subscription
        # callback.  The ROS executor must remain available for clock,
        # lifecycle, DDS heartbeat, and the peer's next crop response while
        # the CPU-heavy geometric gate runs.  Responses are retained in a
        # bounded FIFO when the worker is busy; they are not silently lost.
        registration_context = (
            request_key, pair_key, candidate, dict(request_metadata or {}),
            physical_key, candidate_geometry_key, own_key, peer_key,
            self.keyframes[own_key][1], received_crop,
            int(message.map_epoch), int(message.descriptor_checksum))
        self.pending_requests.discard(request_key)
        self.completed_request_keys.add(request_key)
        self.request_candidate_by_request_key.pop(request_key, None)
        self.request_metadata_by_request_key.pop(request_key, None)
        if self._registration_shutdown:
            self._registration_queue_drops += 1
            self._record_diagnostic_event(
                'REGISTRATION_QUEUE_DROPPED_SHUTDOWN', keyframe_id=peer_key)
            return
        if self._registration_future is not None:
            self._queue_registration_context(registration_context)
            return
        self._start_registration_context(registration_context)
        return

    def _try_confirm_pending_proposal(self, proposal, trigger_message=None):
        """Confirm a canonical proposal once its complete evidence is local.

        A proposal can arrive after the responder has already cached all of
        the source crops while independently verifying the peer summary.  The
        old path only attempted confirmation from a later crop callback, so a
        complete proposal could remain pending forever.  This helper is called
        both on proposal receipt and on each crop response.
        """
        evidence_sources = list(getattr(
            proposal, 'evidence_source_keyframe_ids', []))
        evidence_targets = list(getattr(
            proposal, 'evidence_target_keyframe_ids', []))
        if not evidence_sources:
            evidence_sources = [proposal.source_keyframe_id]
            evidence_targets = [proposal.target_keyframe_id]
        if len(evidence_sources) != len(evidence_targets):
            self._record_diagnostic_event(
                'PROPOSAL_CONFIRMATION_REJECTED',
                reason='EVIDENCE_KEY_LENGTH_MISMATCH',
                source_keyframe_id=str(proposal.source_keyframe_id))
            if trigger_message is not None:
                self._record_crop_rejection(
                    'EVIDENCE_KEY_LENGTH_MISMATCH', trigger_message)
            return True
        missing_source = [
            source_key for source_key in evidence_sources
            if source_key not in self.received_peer_crops]
        missing_target = [
            target_key for target_key in evidence_targets
            if target_key not in self.keyframes]
        if missing_source or missing_target:
            self._record_diagnostic_event(
                'PROPOSAL_CONFIRMATION_WAITING',
                source_keyframe_id=str(proposal.source_keyframe_id),
                missing_source_keyframe_ids=[str(value) for value in missing_source],
                missing_target_keyframe_ids=[str(value) for value in missing_target])
            if trigger_message is not None:
                self._record_crop_rejection(
                    'INCOMPLETE_EVIDENCE_SET', trigger_message,
                    missing_source_keyframe_ids=missing_source,
                    missing_target_keyframe_ids=missing_target)
            return True
        # The canonical proposal is source-robot -> target-robot.  The
        # responder independently verifies the same physical views in its
        # local direction (target-robot -> source-robot), then inverts exactly
        # once below before comparing with the canonical proposal.  Passing
        # the crops in canonical order here while treating the result as
        # reverse direction caused valid proposals to be rejected.
        evidence_pairs = [
            (self.keyframes[target_key][1],
             self.received_peer_crops[source_key])
            for source_key, target_key in zip(evidence_sources, evidence_targets)]
        evidence_timestamps = [
            (self._stamp_ns(self.keyframes[target_key][0]),
             self._stamp_ns(self.peer_descriptors[source_key]))
            for source_key, target_key in zip(evidence_sources, evidence_targets)
            if source_key in self.peer_descriptors]
        if len(evidence_timestamps) != len(evidence_pairs):
            self._record_diagnostic_event(
                'PROPOSAL_CONFIRMATION_REJECTED',
                reason='MISSING_EVIDENCE_METADATA',
                source_keyframe_id=str(proposal.source_keyframe_id))
            return True
        self._record_diagnostic_event(
            'PROPOSAL_CONFIRMATION_STARTED',
            source_keyframe_id=str(proposal.source_keyframe_id),
            target_keyframe_id=str(proposal.target_keyframe_id),
            evidence_source_keyframe_ids=[str(value) for value in evidence_sources],
            evidence_target_keyframe_ids=[str(value) for value in evidence_targets])
        reverse_result = self._run_registration(
            evidence_pairs, 'target_confirmation',
            proposal.target_keyframe_id,
            evidence_timestamps=evidence_timestamps,
            source_viewpoints=[self._keyframe_viewpoint(target_key)
                               for target_key in evidence_targets],
            target_viewpoints=[self._descriptor_viewpoint(
                self.peer_descriptors.get(source_key))
                               for source_key in evidence_sources],
            evidence_ids=[self._canonical_evidence_id(
                proposal.source_robot_id, proposal.target_robot_id,
                source_key, target_key)
                for source_key, target_key in zip(evidence_sources, evidence_targets)])
        # ``target_confirmation`` registered target->source above.  Convert
        # that independently verified result into the canonical source->target
        # direction exactly once before comparing and acknowledging it.
        result = replace(reverse_result,
                         transform=invert_se2(reverse_result.transform))
        tx, ty, yaw = result.transform
        proposed_tx = proposal.source_to_target.translation.x
        proposed_ty = proposal.source_to_target.translation.y
        proposed_yaw = math.atan2(
            2.0 * (proposal.source_to_target.rotation.w *
                    proposal.source_to_target.rotation.z),
            1.0 - 2.0 * proposal.source_to_target.rotation.z ** 2)
        translation_error = math.hypot(tx - proposed_tx, ty - proposed_ty)
        yaw_error = abs(math.atan2(
            math.sin(yaw - proposed_yaw), math.cos(yaw - proposed_yaw)))
        selector_agrees = (
            str(getattr(proposal, 'selector_status', '')) in
            ('', 'ACCEPTED_HYPOTHESIS'))
        mutually_consistent = (
            translation_error <= 0.12 and yaw_error <= 0.04 and
            selector_agrees and
            str(getattr(proposal, 'evidence_set_hash', '')) != '')
        accepted = reverse_result.accepted and mutually_consistent
        self._record_diagnostic_event(
            'PROPOSAL_CONFIRMATION_RESULT',
            accepted=bool(accepted), result_accepted=bool(result.accepted),
            translation_error_m=float(translation_error),
            yaw_error_rad=float(yaw_error),
            selector_agrees=bool(selector_agrees),
            evidence_set_hash=str(getattr(proposal, 'evidence_set_hash', '')))
        response = self._ack_message(
            proposal, result, accepted,
            '' if accepted else 'MUTUAL_TRANSFORM_INCONSISTENT')
        self.hypothesis_pub.publish(response)
        self.counters['acks_published'] += 1
        if accepted:
            self.counters['accepted_hypotheses'] += 1
        else:
            self.counters['rejected_hypotheses'] += 1
        self._record_diagnostic_event(
            'PROPOSAL_ACK_PUBLISHED', accepted=bool(accepted),
            evidence_set_hash=str(getattr(proposal, 'evidence_set_hash', '')))
        self.pending_target_proposal = False
        self.peer_proposals.pop(str(proposal.source_keyframe_id), None)
        return True

    def _record_physical_worker_result(self, candidate, metadata, error=None):
        """Record a bounded worker failure without touching ROS state."""
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_RESULT',
            **(metadata or {}),
            candidate=self._candidate_diagnostic(
                candidate, status='GEOMETRIC_REJECTED',
                reason='REGISTRATION_EXCEPTION', compact=True),
            accepted_geometric=False,
            rejection_reason='REGISTRATION_EXCEPTION',
            worker_exception=error or '')

    def _content_pair_reuses_evidence(self, own_crop, peer_crop):
        """Reject byte-identical or reciprocal crop content as non-independent."""
        own_content = self._crop_content_identity(own_crop)
        peer_content = self._crop_content_identity(peer_crop)
        return (
            (own_content, peer_content) in self.evidence_content_pairs or
            (peer_content, own_content) in self.evidence_content_pairs or
            own_content in self.evidence_source_content or
            peer_content in self.evidence_peer_content)

    @staticmethod
    def _viewpoint_distance(first, second):
        if first is None or second is None:
            return None
        return math.hypot(float(first[0]) - float(second[0]),
                          float(first[1]) - float(second[1]))

    def _canonical_constraint_duplicate_reason(
            self, source_key, target_key, source_crop, target_crop,
            source_viewpoint, target_viewpoint):
        """Return a physical-identity rejection for the canonical pool.

        Keyframe IDs are only labels.  Content and local physical viewpoints
        remain the authority for independence, and a reciprocal observation
        maps to the same canonical ``(R1, R2)`` identity.
        """
        canonical_id = self._canonical_evidence_id(
            'robot1', 'robot2', source_key, target_key)
        if canonical_id in self.canonical_constraint_pool:
            return 'RECIPROCAL_OR_REPEATED_PHYSICAL_EVIDENCE'
        source_content = self._crop_content_identity(source_crop)
        target_content = self._crop_content_identity(target_crop)
        for record in self.canonical_constraint_pool.values():
            if (record.get('source_content') is not None and
                    source_content == record['source_content']):
                return 'REUSED_SOURCE_CROP_CONTENT'
            if (record.get('target_content') is not None and
                    target_content == record['target_content']):
                return 'REUSED_TARGET_CROP_CONTENT'
            if (self._viewpoint_distance(
                    source_viewpoint, record.get('source_viewpoint')) is not None
                    and self._viewpoint_distance(
                        source_viewpoint, record.get('source_viewpoint')) <
                    float(self.evidence_keyframe_translation_threshold_m)):
                return 'REUSED_SOURCE_PHYSICAL_VIEWPOINT'
            if (self._viewpoint_distance(
                    target_viewpoint, record.get('target_viewpoint')) is not None
                    and self._viewpoint_distance(
                        target_viewpoint, record.get('target_viewpoint')) <
                    float(self.evidence_keyframe_translation_threshold_m)):
                return 'REUSED_TARGET_PHYSICAL_VIEWPOINT'
        return ''

    @staticmethod
    def _canonical_mode_distance(first, second):
        """Return the bounded same-pair mode distance in SE(2)."""
        translation = math.hypot(
            float(first.transform[0]) - float(second.transform[0]),
            float(first.transform[1]) - float(second.transform[1]))
        yaw = abs(math.atan2(
            math.sin(float(first.transform[2]) - float(second.transform[2])),
            math.cos(float(first.transform[2]) - float(second.transform[2]))))
        return translation, yaw

    def _merge_canonical_modes(self, existing, incoming):
        """Merge alternatives for one physical ID without adding evidence."""
        modes = list(existing or ())
        for candidate in tuple(incoming or ()):
            duplicate = None
            for index, prior in enumerate(modes):
                distance, yaw = self._canonical_mode_distance(candidate, prior)
                if distance <= 0.05 and yaw <= math.radians(0.5):
                    duplicate = index
                    break
            if duplicate is None:
                modes.append(candidate)
                self._record_diagnostic_event(
                    'CANONICAL_MODE_ADDED',
                    mode_index=int(getattr(candidate, 'mode_index', -1)))
                continue
            prior = modes[duplicate]
            prior_support = int(getattr(prior, 'mode_support', 1))
            candidate_support = int(getattr(candidate, 'mode_support', 1))
            # Support is evidence about repeatability within this physical
            # pair only.  It is never used as another independent constraint.
            preferred = candidate if (
                candidate_support,
                float(getattr(candidate, 'mode_log_weight', -math.inf)),
                float(candidate.inlier_ratio),
            ) > (
                prior_support,
                float(getattr(prior, 'mode_log_weight', -math.inf)),
                float(prior.inlier_ratio),
            ) else prior
            modes[duplicate] = replace(
                preferred,
                mode_support=max(prior_support, candidate_support),
            )
            self._record_diagnostic_event(
                'CANONICAL_MODE_DEDUPED',
                mode_index=int(getattr(candidate, 'mode_index', -1)),
                mode_support=max(prior_support, candidate_support),
                translation_distance_m=float(distance),
                yaw_distance_rad=float(yaw))
        modes.sort(key=lambda item: (
            -int(getattr(item, 'mode_support', 1)),
            -float(getattr(item, 'mode_log_weight', -math.inf)),
            -float(item.inlier_ratio), float(item.residual_m)))
        mode_cap = getattr(self, 'mrpt_max_distinct_modes_per_pair', 10)
        return tuple(modes[:max(1, int(mode_cap))])

    def _add_canonical_constraint(self, candidate, result, own_crop,
                                  received_crop, own_key, peer_key,
                                  hypotheses=None):
        """Insert one strong locally verified constraint in R1->R2 form."""
        hypotheses = tuple(hypotheses or (result,))
        _, _, peer_descriptor, _ = self._candidate_fields(candidate)
        if self.robot_id == 'robot1':
            source_key, target_key = str(own_key), str(peer_key)
            source_crop, target_crop = own_crop, received_crop
            canonical_result = result
            source_viewpoint = self._keyframe_viewpoint(own_key)
            target_viewpoint = self._descriptor_viewpoint(peer_descriptor)
            source_descriptor, target_descriptor = (
                self.keyframes[own_key][0], peer_descriptor)
            canonical_hypotheses = hypotheses
        else:
            source_key, target_key = str(peer_key), str(own_key)
            source_crop, target_crop = received_crop, own_crop
            canonical_result = replace(
                result,
                transform=invert_se2(result.transform),
                inlier_ratio=float(result.reverse_inlier_ratio),
                reverse_inlier_ratio=float(result.inlier_ratio))
            source_viewpoint = None
            source_viewpoint = self._descriptor_viewpoint(peer_descriptor)
            target_viewpoint = self._keyframe_viewpoint(own_key)
            source_descriptor, target_descriptor = (
                peer_descriptor, self.keyframes[own_key][0])
            canonical_hypotheses = tuple(
                replace(
                    hypothesis,
                    transform=invert_se2(hypothesis.transform),
                    inlier_ratio=float(hypothesis.reverse_inlier_ratio),
                    reverse_inlier_ratio=float(hypothesis.inlier_ratio))
                for hypothesis in hypotheses)
        canonical_id = self._canonical_evidence_id(
            'robot1', 'robot2', source_key, target_key)
        existing = self.canonical_constraint_pool.get(canonical_id)
        if existing is not None:
            existing['hypotheses'] = self._merge_canonical_modes(
                existing.get('hypotheses', ()), canonical_hypotheses)
            if existing.get('pair') is not None:
                self._record_diagnostic_event(
                    'CANONICAL_CONSTRAINT_DUPLICATE',
                    evidence_id=canonical_id,
                    reason='RECIPROCAL_OR_REPEATED_PHYSICAL_EVIDENCE')
                return False
            existing.update({
                'source_key': source_key,
                'target_key': target_key,
                'pair': (source_crop, target_crop),
                'result': canonical_result,
                'source_content': self._crop_content_identity(source_crop),
                'target_content': self._crop_content_identity(target_crop),
                'source_viewpoint': source_viewpoint,
                'target_viewpoint': target_viewpoint,
                'source_descriptor': source_descriptor,
                'target_descriptor': target_descriptor,
                'timestamps': (
                    self._stamp_ns(source_descriptor),
                    self._stamp_ns(target_descriptor)),
            })
            self._record_diagnostic_event(
                'CANONICAL_CONSTRAINT_ADDED', evidence_id=canonical_id,
                source_keyframe_id=source_key, target_keyframe_id=target_key,
                canonical_pool_size=len(self.canonical_constraint_pool),
                source_viewpoint_available=source_viewpoint is not None,
                target_viewpoint_available=target_viewpoint is not None,
                merged_peer_modes=True)
            return True
        duplicate_reason = self._canonical_constraint_duplicate_reason(
            source_key, target_key, source_crop, target_crop,
            source_viewpoint, target_viewpoint)
        if duplicate_reason:
            self._write_physical_evidence_diagnostic(
                'CANONICAL_CONSTRAINT_SUPPRESSED',
                canonical_evidence_id=self._canonical_evidence_id(
                    'robot1', 'robot2', source_key, target_key),
                source_keyframe_id=source_key,
                target_keyframe_id=target_key,
                reason=duplicate_reason)
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            return False
        timestamp_pair = (
            self._stamp_ns(source_descriptor),
            self._stamp_ns(target_descriptor))
        self.canonical_constraint_pool[canonical_id] = {
            'evidence_id': canonical_id,
            'source_key': source_key,
            'target_key': target_key,
            'pair': (source_crop, target_crop),
            'result': canonical_result,
            'hypotheses': canonical_hypotheses,
            'source_content': self._crop_content_identity(source_crop),
            'target_content': self._crop_content_identity(target_crop),
            'source_viewpoint': source_viewpoint,
            'target_viewpoint': target_viewpoint,
            'source_descriptor': source_descriptor,
            'target_descriptor': target_descriptor,
            'timestamps': timestamp_pair,
        }
        self._record_diagnostic_event(
            'CANONICAL_MODE_ADDED', evidence_id=canonical_id,
            mode_count=len(canonical_hypotheses), discovery='local')
        while len(self.canonical_constraint_pool) > max(
                self.max_evidence_constraints,
                self.min_consistent_constraints):
            self.canonical_constraint_pool.popitem(last=False)
        self._record_diagnostic_event(
            'CANONICAL_CONSTRAINT_ADDED',
            evidence_id=canonical_id,
            source_keyframe_id=source_key,
            target_keyframe_id=target_key,
            canonical_transform=[float(value) for value in
                                 canonical_result.transform],
            canonical_pool_size=len(self.canonical_constraint_pool))
        return True

    def _canonical_union_consensus(self):
        """Run the unchanged selector over the bounded canonical pool."""
        records = [record for record in self.canonical_constraint_pool.values()
                   if record.get('pair') is not None and
                   record.get('source_viewpoint') is not None and
                   record.get('target_viewpoint') is not None]
        if len(records) < self.min_consistent_constraints:
            self._record_diagnostic_event(
                'CANONICAL_FAMILY_EVALUATED',
                canonical_pool_size=len(self.canonical_constraint_pool),
                complete_physical_records=len(records), accepted=False,
                reason='INCOMPLETE_PHYSICAL_METADATA')
            return None
        self._record_diagnostic_event(
            'CANONICAL_FAMILY_EVALUATED',
            canonical_pool_size=len(self.canonical_constraint_pool),
            complete_physical_records=len(records), accepted=False,
            reason='EVALUATING')
        result = self._run_registration(
            [record['pair'] for record in records], 'canonical_union',
            records[0]['evidence_id'],
            individual_results=[record['result'] for record in records],
            hypothesis_sets=[record.get('hypotheses', (record['result'],))
                             for record in records],
            evidence_timestamps=[record['timestamps'] for record in records],
            evidence_ids=[record['evidence_id'] for record in records],
            source_viewpoints=[record['source_viewpoint']
                               for record in records],
            target_viewpoints=[record['target_viewpoint']
                               for record in records])
        self._record_diagnostic_event(
            'CANONICAL_UNION_CONSENSUS',
            canonical_pool_size=len(records), accepted=bool(result.accepted),
            reason=str(result.reason),
            consistent_constraint_count=int(
                result.consistent_constraint_count),
            spatial_baseline_m=float(result.spatial_baseline_m))
        return result

    def _publish_canonical_union_candidate(self, result):
        """Publish one canonical candidate for the existing peer protocol."""
        if (self.canonical_union_summary_published or
                self.batch_proposal_published or
                self.robot_id != min(self.robot_id, self.peer_robot_id) or
                not result.accepted):
            return False
        winner_ids = []
        for diagnostic in reversed(getattr(result, 'consensus_diagnostics', ())):
            if diagnostic.get('kind') == 'incremental_hypothesis_accumulator':
                winner_ids = [str(value) for value in diagnostic.get(
                    'winner_evidence_ids', ())]
                break
        if not winner_ids:
            for diagnostic in getattr(result, 'consensus_diagnostics', ()):
                if (diagnostic.get('kind') == 'robust_hypothesis' and
                        int(diagnostic.get('rank', 1)) == 0):
                    selected = [int(value) for value in diagnostic.get(
                        'selected_indices', ())]
                    winner_ids = [list(self.canonical_constraint_pool)[index]
                                  for index in selected
                                  if 0 <= index < len(
                                      self.canonical_constraint_pool)]
                    break
        if not winner_ids:
            winner_ids = list(self.canonical_constraint_pool)
        records = [self.canonical_constraint_pool[value]
                   for value in winner_ids
                   if value in self.canonical_constraint_pool]
        if len(records) < self.min_consistent_constraints:
            return False
        source_ids = [record['source_key'] for record in records]
        target_ids = [record['target_key'] for record in records]
        if any('source_descriptor' not in record or
               'target_descriptor' not in record for record in records):
            self._record_diagnostic_event(
                'CANONICAL_UNION_PROPOSAL_DEFERRED',
                reason='MISSING_DESCRIPTOR_METADATA')
            return False
        own_descriptor = records[0]['source_descriptor']
        peer_descriptor = records[0]['target_descriptor']
        for record in records:
            descriptor = record['source_descriptor']
            crop = record['pair'][0]
            message = LocalMapCrop()
            message.header = descriptor.header
            message.source_robot_id = self.robot_id
            message.keyframe_id = record['source_key']
            message.map_epoch = descriptor.map_epoch
            message.descriptor_checksum = descriptor.checksum
            message.occupancy_checksum = self._occupancy_checksum(crop.values)
            message.occupancy_grid = self._crop_message(crop, descriptor.header)
            self.crop_pub.publish(message)
            self.counters['crops_sent'] += 1
        proposal = self._hypothesis_message(
            own_descriptor, peer_descriptor, result, status='CANDIDATE',
            accepted=False, rejection_reason='',
            evidence_source_keyframe_ids=source_ids,
            evidence_target_keyframe_ids=target_ids)
        self.hypothesis_pub.publish(proposal)
        self.canonical_union_summary_published = True
        self.local_hypothesis_summary = proposal
        self.local_hypothesis_result = result
        self.counters['hypothesis_summaries_published'] += 1
        self._record_diagnostic_event(
            'CANONICAL_UNION_SUMMARY_PUBLISHED',
            evidence_set_hash=str(proposal.evidence_set_hash),
            consistent_constraint_count=int(
                proposal.consistent_constraint_count))
        return True

    def _apply_candidate_verification_result(
            self, pair_key, candidate, result, request_metadata, physical_key,
            candidate_geometry_key, own_key, peer_key, own_crop,
            received_crop, map_epoch, descriptor_checksum, hypotheses=None):
        """Apply one worker result and continue the existing protocol path."""
        hypotheses = tuple(hypotheses or (result,))
        self.counters['registrations'] += 1
        self.counters['registration_callback_entries'] += 1
        self.registration_callback_depth += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_ENTRY', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1)
        self.candidate_verification_results[pair_key] = result
        self.candidate_verification_hypotheses[pair_key] = hypotheses
        mature_evidence, maturity = self._consensus_pair_maturity(
            own_crop, received_crop)
        admitted_hypotheses = tuple(
            hypothesis for hypothesis in hypotheses
            if consensus_admission_quality(
                hypothesis, mature_evidence=mature_evidence)[0])
        strong_eligible = bool(admitted_hypotheses)
        strong_reason = ('STRONG_CONSENSUS_ELIGIBLE' if strong_eligible else
                         (consensus_admission_quality(
                             result, mature_evidence=mature_evidence)[1]))
        self._finalize_registration_capture(
            pair_key, result=result,
            strong_consensus_eligible=strong_eligible,
            strong_consensus_reason=strong_reason)
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_EXIT', source='candidate_verification',
            keyframe_id=peer_key, constraint_count=1,
            accepted=bool(result.accepted), reason=str(result.reason),
            residual_m=float(result.residual_m),
            inlier_ratio=float(result.inlier_ratio),
            projected_error_m=float(result.projected_error_m))
        self._write_physical_evidence_diagnostic(
            'CANDIDATE_VERIFICATION_RESULT',
            **(request_metadata or {}),
            candidate=self._candidate_diagnostic(
                candidate, status='GEOMETRIC_ACCEPTED' if result.accepted
                else 'GEOMETRIC_REJECTED', reason=str(result.reason),
                compact=True),
            transform=[float(value) for value in result.transform],
            residual_m=float(result.residual_m),
            median_residual_m=float(result.median_residual_m),
            p95_residual_m=float(result.p95_residual_m),
            inlier_ratio=float(result.inlier_ratio),
            occupied_free_agreement=float(result.occupied_free_agreement),
            overlap_fraction=float(result.overlap_fraction),
            translation_uncertainty_m=float(result.translation_uncertainty_m),
            yaw_uncertainty_rad=float(result.yaw_uncertainty_rad),
            condition_number=float(result.condition_number),
            projected_error_m=float(result.projected_error_m),
            accepted_geometric=bool(result.accepted),
            rejection_reason='' if result.accepted else str(result.reason),
            strong_consensus_eligible=bool(strong_eligible),
            strong_consensus_rejection_reason=str(strong_reason),
            consensus_evidence_mature=bool(mature_evidence),
            consensus_maturity=maturity,
            backend=str(getattr(result, 'backend', self.registration_backend)),
            hypothesis_count=len(hypotheses),
            admitted_hypothesis_count=len(admitted_hypotheses),
            hypothesis_mode_indices=[int(getattr(item, 'mode_index', -1))
                                     for item in hypotheses])
        if result.accepted:
            self.counters['candidate_verification_accepted'] += 1
        else:
            self.counters['candidate_verification_rejected'] += 1
        self.registration_callback_depth -= 1
        if not result.accepted:
            self.rejected_physical_evidence_keys.add(physical_key)
            self.rejected_physical_geometry_keys.add(candidate_geometry_key)
            self.rejected_physical_geometry_batches[candidate_geometry_key] = (
                self._request_batch_id(request_metadata))
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_REJECTED_BEFORE_CONSENSUS',
                **(request_metadata or {}),
                candidate=self._candidate_diagnostic(
                    candidate, status='REJECTED', reason=str(result.reason),
                    compact=True),
                rejection_reason=str(result.reason))
            self._request_next_candidate_verification()
            return
        if not strong_eligible:
            # Keep the finite individual result in diagnostics, but do not
            # let it consume an evidence/consensus slot.  The bounded request
            # budget still limits expensive registration work; a later map
            # revision or displaced viewpoint can produce a new candidate.
            self.counters['weak_consensus_candidates_rejected'] += 1
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_REJECTED_BEFORE_CONSENSUS',
                **(request_metadata or {}),
                candidate=self._candidate_diagnostic(
                    candidate, status='REJECTED', reason=str(strong_reason),
                    compact=True),
                rejection_reason=str(strong_reason),
                accepted_geometric=True,
                strong_consensus_eligible=False)
            self.rejected_physical_evidence_keys.add(physical_key)
            self.rejected_physical_geometry_keys.add(candidate_geometry_key)
            self.rejected_physical_geometry_batches[candidate_geometry_key] = (
                self._request_batch_id(request_metadata))
            self._request_next_candidate_verification()
            return
        if self._content_pair_reuses_evidence(own_crop, received_crop):
            self.counters['physical_content_duplicates_suppressed'] += 1
            self.counters['physical_evidence_duplicates_suppressed'] += 1
            self._write_physical_evidence_diagnostic(
                'CANDIDATE_REJECTED_BEFORE_CONSENSUS',
                **(request_metadata or {}),
                candidate=self._candidate_diagnostic(
                    candidate, status='REJECTED',
                    reason='IDENTICAL_CROP_CONTENT_ALREADY_ACCEPTED',
                    compact=True),
                rejection_reason='IDENTICAL_CROP_CONTENT_ALREADY_ACCEPTED')
            self._request_next_candidate_verification()
            return
        self.evidence_pairs[pair_key] = (
            own_crop, received_crop)
        self.evidence_candidates[pair_key] = candidate
        self.evidence_physical_keys[pair_key] = physical_key
        self.evidence_physical_geometry_keys.add(candidate_geometry_key)
        own_content = self._crop_content_identity(own_crop)
        peer_content = self._crop_content_identity(received_crop)
        self.evidence_content_pairs.add((own_content, peer_content))
        self.evidence_source_content.add(own_content)
        self.evidence_peer_content.add(peer_content)
        self.counters['constraints_accumulated'] = len(
            self.evidence_physical_keys)
        self.counters['strong_consensus_candidates'] += 1
        self._record_diagnostic_event(
            'CROP_ACCEPTED', own_key=own_key, peer_key=peer_key,
            map_epoch=int(map_epoch),
            descriptor_checksum=int(descriptor_checksum),
            constraints_accumulated=len(self.evidence_physical_keys))
        self._publish_evidence_announcement(
            candidate, result, own_crop, received_crop,
            hypotheses=admitted_hypotheses)
        canonical_added = self._add_canonical_constraint(
            candidate, result, own_crop, received_crop, own_key, peer_key,
            hypotheses=admitted_hypotheses)
        if canonical_added:
            canonical_consensus = self._canonical_union_consensus()
            if canonical_consensus is not None and canonical_consensus.accepted:
                self.evidence_acquisition_started = False
                self._publish_evidence_status(False)
                self.evidence_acquisition_deadline_wall = None
                self.verification_batches.mark_completed()
                self._publish_canonical_union_candidate(canonical_consensus)
                self._clear_pending_registration_contexts(
                    'CANONICAL_UNION_CONSENSUS_ACCEPTED')
                return
        if len(self.evidence_physical_keys) < self.min_consistent_constraints:
            self._record_diagnostic_event(
                'EVIDENCE_SET_WAITING',
                constraints_accumulated=len(self.evidence_physical_keys),
                minimum_constraints=self.min_consistent_constraints)
            self._request_next_candidate_verification()
            return
        self.counters['evidence_sets_formed'] += 1
        self._record_diagnostic_event(
            'EVIDENCE_SET_FORMED',
            constraints_accumulated=len(self.evidence_physical_keys))
        evidence_items = list(self.evidence_pairs.items())
        pairs = [evidence for _, evidence in evidence_items]
        cached_results = [
            self.candidate_verification_results.get(pair_key)
            for pair_key, _ in evidence_items]
        cached_hypotheses = [
            self.candidate_verification_hypotheses.get(
                pair_key, (self.candidate_verification_results.get(pair_key),))
            for pair_key, _ in evidence_items]
        evidence_timestamps = []
        for (evidence_own_key, evidence_peer_key), _ in evidence_items:
            candidate = self.evidence_candidates.get(
                (evidence_own_key, evidence_peer_key))
            own_entry = self.keyframes.get(evidence_own_key)
            peer_descriptor = None if candidate is None else candidate[2]
            if own_entry is None or peer_descriptor is None:
                self._record_diagnostic_event(
                    'EVIDENCE_DESCRIPTOR_METADATA_MISSING',
                    own_key=evidence_own_key, peer_key=evidence_peer_key)
                continue
            evidence_timestamps.append((
                self._stamp_ns(own_entry[0]),
                self._stamp_ns(peer_descriptor)))
        if len(evidence_timestamps) != len(evidence_items):
            self._request_next_candidate_verification()
            return
        consensus = self._run_registration(
            pairs, 'incremental_consensus', peer_key,
            individual_results=cached_results,
            hypothesis_sets=(cached_hypotheses if
                             self.registration_backend == 'mrpt' else None),
            evidence_timestamps=evidence_timestamps,
            source_viewpoints=[self._keyframe_viewpoint(pair_key[0])
                               for pair_key, _ in evidence_items],
            target_viewpoints=[self._descriptor_viewpoint(
                self.evidence_candidates[pair_key][2])
                for pair_key, _ in evidence_items])
        if consensus.accepted:
            self.evidence_acquisition_started = False
            self._publish_evidence_status(False)
            self.evidence_acquisition_deadline_wall = None
            self.verification_batches.mark_completed()
            self._publish_multi_constraint_proposal(result=consensus)
            self._clear_pending_registration_contexts('CONSENSUS_ACCEPTED')
            return
        self._write_physical_evidence_diagnostic(
            'INCREMENTAL_CONSENSUS_REJECTED',
            accepted_constraint_count=len(self.evidence_pairs),
            reason=str(consensus.reason))
        self._request_next_candidate_verification()

    def _run_registration(self, evidence_pairs, source, keyframe_id='',
                          individual_results=None, evidence_timestamps=None,
                          evidence_ids=None, source_viewpoints=None,
                          target_viewpoints=None, hypothesis_sets=None):
        self.counters['registrations'] += 1
        self.counters['registration_callback_entries'] += 1
        self.registration_callback_depth += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_ENTRY', source=source,
            keyframe_id=keyframe_id, constraint_count=len(evidence_pairs))
        try:
            evidence_ids = list(evidence_ids) if evidence_ids is not None else None
            accumulator = None
            if source == 'incremental_consensus':
                evidence_ids = list(evidence_ids or [
                    f'{own_key}|{peer_key}'
                    for (own_key, peer_key) in self.evidence_pairs])
                accumulator = self.hypothesis_accumulator
            mature_evidence = all(
                self._consensus_pair_maturity(source_crop, target_crop)[0]
                for source_crop, target_crop in evidence_pairs)
            if hypothesis_sets is not None:
                result = select_hypothesis_family(
                    evidence_pairs, hypothesis_sets,
                    target_map_radius_m=self.target_map_radius_m,
                    min_consistent_constraints=(
                        self.min_consistent_constraints),
                    max_projected_registration_error_m=(
                        self.max_projected_registration_error_m),
                    minimum_agreement=(0.0 if mature_evidence else 0.55),
                    evidence_timestamps=evidence_timestamps,
                    evidence_ids=evidence_ids,
                    source_viewpoints=source_viewpoints,
                    target_viewpoints=target_viewpoints)
            else:
                result = register_crop_set(
                    evidence_pairs,
                    target_map_radius_m=self.target_map_radius_m,
                    min_consistent_constraints=self.min_consistent_constraints,
                    max_projected_registration_error_m=(
                        self.max_projected_registration_error_m),
                    minimum_agreement=(0.0 if mature_evidence else 0.55),
                    individual_results=individual_results,
                    evidence_timestamps=evidence_timestamps,
                    evidence_ids=evidence_ids,
                    source_viewpoints=source_viewpoints,
                    target_viewpoints=target_viewpoints,
                    hypothesis_accumulator=accumulator)
        except Exception as exc:
            self.counters['registration_callback_exceptions'] += 1
            self.consensus_gate_rejection_counts['REGISTRATION_EXCEPTION'] += 1
            self._record_diagnostic_event(
                'REGISTRATION_CALLBACK_EXCEPTION', source=source,
                keyframe_id=keyframe_id, exception=repr(exc))
            raise
        finally:
            self.registration_callback_depth -= 1
            self.counters['registration_callback_exits'] += 1
        self._record_diagnostic_event(
            'REGISTRATION_CALLBACK_EXIT', source=source,
            keyframe_id=keyframe_id, constraint_count=len(evidence_pairs),
            accepted=bool(result.accepted), reason=str(result.reason),
            residual_m=float(result.residual_m),
            consistent_constraint_count=int(result.consistent_constraint_count))
        for diagnostic in getattr(result, 'consensus_diagnostics', ()):
            event_name = {
                'constraint': 'CONSENSUS_CONSTRAINT_DIAGNOSTIC',
                'pairwise_comparison': 'CONSENSUS_PAIRWISE_COMPARISON',
                'subset_comparison': 'CONSENSUS_SUBSET_COMPARISON',
            }.get(diagnostic.get('kind'), 'CONSENSUS_DIAGNOSTIC')
            fields = {key: value for key, value in diagnostic.items()
                      if key != 'kind'}
            self._record_diagnostic_event(
                event_name, source=source, keyframe_id=keyframe_id,
                **fields)
            self._write_consensus_diagnostic(
                event_name, source=source, keyframe_id=keyframe_id,
                **fields)
        self._write_consensus_diagnostic(
            'REGISTRATION_RESULT', source=source, keyframe_id=keyframe_id,
            accepted=bool(result.accepted), reason=str(result.reason),
            transform=[float(value) for value in result.transform],
            constraint_count=int(result.constraint_count),
            consistent_constraint_count=int(
                result.consistent_constraint_count),
            residual_m=float(result.residual_m),
            inlier_ratio=float(result.inlier_ratio),
            translation_uncertainty_m=float(
                result.translation_uncertainty_m),
            yaw_uncertainty_rad=float(result.yaw_uncertainty_rad),
            condition_number=float(result.condition_number),
            projected_error_m=float(result.projected_error_m),
            selector_status=str(getattr(result, 'selector_status', '')),
            selector_score=float(getattr(result, 'selector_score', 0.0)),
            selector_null_score=float(getattr(
                result, 'selector_null_score', 0.0)),
            selector_runner_up_score=float(getattr(
                result, 'selector_runner_up_score', -math.inf)),
            selector_runner_up_margin=float(getattr(
                result, 'selector_runner_up_margin', 0.0)),
            selector_inlier_probabilities=[float(value) for value in getattr(
                result, 'selector_inlier_probabilities', ())])
        if not result.accepted:
            self.counters['multi_constraint_rejections'] += 1
            self.consensus_gate_rejection_counts[str(result.reason)] += 1
        return result

    def _proposal_candidate_pool(self, result=None):
        """Return the candidate view of the evidence used for a proposal.

        When an incremental consensus result is supplied, its evidence has
        already been removed from ``pending_candidate_pairs`` by the crop
        verification path.  Reconstruct that bounded pool from the canonical
        hashable evidence-pair identities instead of silently losing the
        just-accepted constraints at proposal publication time.
        """
        if result is not None:
            pool = []
            for own_key, peer_key in self.evidence_pairs:
                retained = getattr(self, 'evidence_candidates', {}).get(
                    (own_key, peer_key))
                if retained is not None:
                    pool.append(retained)
                    continue
                own_entry = self.keyframes.get(own_key)
                peer_descriptor = self.peer_descriptors.get(peer_key)
                if own_entry is None or peer_descriptor is None:
                    continue
                pool.append((
                    peer_key, own_key, peer_descriptor, own_entry[0]))
            return pool
        candidate_pool = list(self.pending_candidate_pairs.values())
        if not candidate_pool:
            candidate_pool = list(self.active_candidate_pairs)
        return candidate_pool

    def _publish_multi_constraint_proposal(self, result=None):
        if self.batch_proposal_published:
            return
        if self.evidence_acquisition_started and result is None:
            return
        pairs = []
        selected_pairs = []
        physical_keys = set()
        content_pairs = set()
        source_content = set()
        peer_content = set()
        # Evidence can arrive over several selection callbacks.  The active
        # selection is only the latest snapshot; use the bounded pending pool
        # so accepted earlier pairs are reconsidered together.  A completed
        # incremental consensus must instead use the evidence pairs that were
        # just verified, because those candidates have already left pending.
        candidate_pool = self._proposal_candidate_pool(result)
        for candidate in evidence_candidates_for_pool(
                candidate_pool, self.evidence_pairs):
            pair_key = (candidate[1], candidate[0])
            evidence = self.evidence_pairs.get(pair_key)
            if evidence is None:
                continue
            physical_key = self.evidence_physical_keys.get(pair_key)
            if physical_key is None:
                physical_key = (
                    physical_crop_identity(
                        evidence[0], int(candidate[3].map_epoch),
                        int(candidate[3].checksum)),
                    physical_crop_identity(
                        evidence[1], int(candidate[2].map_epoch),
                        int(candidate[2].checksum)))
            if physical_key in physical_keys:
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                continue
            own_content = self._crop_content_identity(evidence[0])
            peer_content_id = self._crop_content_identity(evidence[1])
            if ((own_content, peer_content_id) in content_pairs or
                    (peer_content_id, own_content) in content_pairs or
                    own_content in source_content or
                    peer_content_id in peer_content):
                self.counters['physical_content_duplicates_suppressed'] += 1
                self.counters['physical_evidence_duplicates_suppressed'] += 1
                continue
            physical_keys.add(physical_key)
            content_pairs.add((own_content, peer_content_id))
            source_content.add(own_content)
            peer_content.add(peer_content_id)
            selected_pairs.append(candidate)
            pairs.append(evidence)
            if len(pairs) >= self.candidate_verification_budget:
                break
        if len(pairs) < self.min_consistent_constraints:
            return
        if not evidence_batch_is_spatially_diverse(
                [pair[0] for pair in pairs], min_spatial_baseline_m=0.75):
            self.counters['spatial_diversity_deferrals'] += 1
            self._record_diagnostic_event(
                'EVIDENCE_SET_DEFERRED',
                reason='INSUFFICIENT_SPATIAL_BASELINE',
                constraints_accumulated=len(pairs),
                minimum_spatial_baseline_m=0.75)
            return
        for index, (candidate, evidence) in enumerate(
                zip(selected_pairs, pairs)):
            peer_key, own_key, peer_descriptor, own_descriptor = candidate
            own_crop = self.keyframes[own_key][1]
            peer_crop = evidence[1]
            match = self.matches.get((peer_key, own_key))
            constraint_fields = {
                'index': index,
                'own_keyframe_id': own_key, 'peer_keyframe_id': peer_key,
                'own_timestamp_ns': self._stamp_ns(own_descriptor),
                'peer_timestamp_ns': self._stamp_ns(peer_descriptor),
                'own_map_epoch': int(own_descriptor.map_epoch),
                'peer_map_epoch': int(peer_descriptor.map_epoch),
                'own_descriptor_checksum': int(own_descriptor.checksum),
                'peer_descriptor_checksum': int(peer_descriptor.checksum),
                'descriptor_similarity': (
                    0.0 if match is None else float(match.similarity)),
                'descriptor_margin': (
                    0.0 if match is None else float(match.margin)),
                'own_crop_origin': [float(own_crop.origin_x),
                                 float(own_crop.origin_y),
                                 float(own_crop.origin_yaw)],
                'peer_crop_origin': [float(peer_crop.origin_x),
                                  float(peer_crop.origin_y),
                                  float(peer_crop.origin_yaw)],
                'own_crop_center': [float(value) for value in (
                    own_crop.origin_x + 0.5 * own_crop.values.shape[1] *
                    own_crop.resolution,
                    own_crop.origin_y + 0.5 * own_crop.values.shape[0] *
                    own_crop.resolution)],
                'peer_crop_center': [float(value) for value in (
                    peer_crop.origin_x + 0.5 * peer_crop.values.shape[1] *
                    peer_crop.resolution,
                    peer_crop.origin_y + 0.5 * peer_crop.values.shape[0] *
                    peer_crop.resolution)]
            }
            self._record_diagnostic_event(
                'CONSENSUS_INPUT_CONSTRAINT', **constraint_fields)
            self._write_consensus_diagnostic(
                'CONSENSUS_INPUT_CONSTRAINT', **constraint_fields)
        self.counters['multi_constraint_attempts'] += 1
        if result is None:
            result = self._run_registration(
                pairs, 'incremental_consensus', selected_pairs[0][0],
                evidence_timestamps=[
                    (self._stamp_ns(candidate[3]),
                     self._stamp_ns(candidate[2]))
                    for candidate in selected_pairs],
                source_viewpoints=[self._keyframe_viewpoint(candidate[1])
                                   for candidate in selected_pairs],
                target_viewpoints=[self._descriptor_viewpoint(candidate[2])
                                   for candidate in selected_pairs])
        if result.accepted:
            # The accumulated selector may reject some geometrically valid
            # observations as outliers.  Exchange/request only its winning
            # evidence IDs; sending the whole five/eight-candidate pool would
            # make the peer re-run the selector with known outliers included.
            winner_ids = set()
            for diagnostic in reversed(
                    getattr(result, 'consensus_diagnostics', ())):
                if diagnostic.get('kind') == \
                        'incremental_hypothesis_accumulator':
                    winner_ids = set(str(value) for value in diagnostic.get(
                        'winner_evidence_ids', ()))
                    break
            if winner_ids:
                filtered = [
                    (candidate, evidence)
                    for candidate, evidence in zip(selected_pairs, pairs)
                    if f'{candidate[1]}|{candidate[0]}' in winner_ids]
                if len(filtered) >= self.min_consistent_constraints:
                    selected_pairs = [item[0] for item in filtered]
                    pairs = [item[1] for item in filtered]
        own_key = selected_pairs[0][1]
        peer_key = selected_pairs[0][0]
        own_descriptor = self.keyframes[own_key][0]
        # The descriptor is retained in the selected candidate even when the
        # bounded live descriptor cache has since evicted its key.
        peer_descriptor = selected_pairs[0][2]
        source_ids = [pair[1] for pair in selected_pairs]
        target_ids = [pair[0] for pair in selected_pairs]
        self._publish_local_evidence_crops(
            source_ids, evidence_candidates=selected_pairs)
        proposal = self._hypothesis_message(
            own_descriptor, peer_descriptor, result,
            status='CANDIDATE' if result.accepted else 'REJECTED',
            accepted=False,
            rejection_reason='' if result.accepted else result.reason,
            evidence_source_keyframe_ids=source_ids,
            evidence_target_keyframe_ids=target_ids)
        self.hypothesis_pub.publish(proposal)
        if result.accepted:
            self.local_hypothesis_summary = proposal
            self.local_hypothesis_result = result
            self.counters['hypothesis_summaries_published'] += 1
            self.verification_batches.mark_completed()
            self._record_diagnostic_event(
                'LOCAL_HYPOTHESIS_SUMMARY_PUBLISHED',
                evidence_set_hash=str(proposal.evidence_set_hash),
                selector_score=float(proposal.selector_score),
                selector_runner_up_margin=float(
                    proposal.selector_runner_up_margin),
                consistent_constraint_count=int(
                    proposal.consistent_constraint_count))
            if (self.robot_id == min(self.robot_id, self.peer_robot_id) and
                    self._peer_hypothesis_agrees(proposal)):
                self._publish_canonical_proposal(proposal, result)
        else:
            self.counters['proposals_published'] += 1
            self.counters['rejected_hypotheses'] += 1
            self.negotiation_started = False
            self._write_physical_evidence_diagnostic(
                'VERIFICATION_BATCH_WAITING',
                acquisition_batch_id=self.verification_batches.batch_id,
                constraints_accumulated=len(self.evidence_physical_keys),
                reason='CONSENSUS_REJECTED',
                state='WAITING_FOR_NOVEL_EVIDENCE')

    @staticmethod
    def _canonical_evidence_hash(source_robot_id, target_robot_id,
                                  source_ids, target_ids):
        """Hash evidence pairs in a robot-independent direction."""
        entries = []
        low_first = str(source_robot_id) < str(target_robot_id)
        for source_id, target_id in zip(source_ids, target_ids):
            if low_first:
                entries.append(f'{source_robot_id}:{source_id}|'
                               f'{target_robot_id}:{target_id}')
            else:
                entries.append(f'{target_robot_id}:{target_id}|'
                               f'{source_robot_id}:{source_id}')
        return hashlib.sha256('|'.join(sorted(entries)).encode('utf-8')).hexdigest()[:16]

    @staticmethod
    def _canonical_evidence_id(source_robot_id, target_robot_id,
                               source_id, target_id):
        if str(source_robot_id) < str(target_robot_id):
            return f'{source_robot_id}:{source_id}|{target_robot_id}:{target_id}'
        return f'{target_robot_id}:{target_id}|{source_robot_id}:{source_id}'

    @staticmethod
    def _summary_transform(message):
        rotation = message.source_to_target.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z),
            1.0 - 2.0 * rotation.z ** 2)
        return (float(message.source_to_target.translation.x),
                float(message.source_to_target.translation.y), float(yaw))

    def _peer_hypothesis_agrees(self, local_summary):
        peer = self.peer_hypothesis_summary
        if peer is None:
            return False
        if str(peer.evidence_set_hash) != str(local_summary.evidence_set_hash):
            self._record_diagnostic_event(
                'HYPOTHESIS_SUMMARY_MISMATCH',
                local_evidence_set_hash=str(local_summary.evidence_set_hash),
                peer_evidence_set_hash=str(peer.evidence_set_hash))
            return False
        local = self._summary_transform(local_summary)
        peer_inverse = invert_se2(self._summary_transform(peer))
        translation_error = math.hypot(local[0] - peer_inverse[0],
                                       local[1] - peer_inverse[1])
        yaw_error = abs(math.atan2(math.sin(local[2] - peer_inverse[2]),
                                   math.cos(local[2] - peer_inverse[2])))
        agrees = (str(peer.selector_status) == 'ACCEPTED_HYPOTHESIS' and
                  int(peer.consistent_constraint_count) >=
                  int(self.min_consistent_constraints) and
                  translation_error <= 0.12 and yaw_error <= 0.04)
        self._record_diagnostic_event(
            'HYPOTHESIS_SUMMARY_VERIFIED',
            evidence_set_hash=str(local_summary.evidence_set_hash),
            translation_error_m=translation_error,
            yaw_error_rad=yaw_error, agrees=agrees)
        return agrees

    def _verify_peer_hypothesis_summary(self):
        summary = self.peer_hypothesis_summary
        if summary is None:
            return
        summary_hash = str(getattr(summary, 'evidence_set_hash', ''))
        if summary_hash and summary_hash == self._verified_peer_summary_hash:
            return
        source_ids = [str(value) for value in
                      getattr(summary, 'evidence_source_keyframe_ids', [])]
        target_ids = [str(value) for value in
                      getattr(summary, 'evidence_target_keyframe_ids', [])]
        if len(source_ids) < self.min_consistent_constraints or \
                len(source_ids) != len(target_ids):
            self._record_diagnostic_event(
                'PEER_HYPOTHESIS_SUMMARY_REJECTED',
                reason='INSUFFICIENT_EVIDENCE_IDS')
            return
        if any(source_id not in self.received_peer_crops or
               target_id not in self.keyframes
               for source_id, target_id in zip(source_ids, target_ids)):
            self._record_diagnostic_event(
                'PEER_HYPOTHESIS_SUMMARY_REJECTED',
                reason='INCOMPLETE_EVIDENCE_CROPS')
            return
        evidence_pairs = [
            (self.received_peer_crops[source_id],
             self.keyframes[target_id][1])
            for source_id, target_id in zip(source_ids, target_ids)]
        evidence_timestamps = [
            (self._stamp_ns(self.peer_descriptors[source_id]),
             self._stamp_ns(self.keyframes[target_id][0]))
            for source_id, target_id in zip(source_ids, target_ids)
            if source_id in self.peer_descriptors]
        if len(evidence_timestamps) != len(evidence_pairs):
            self._record_diagnostic_event(
                'PEER_HYPOTHESIS_SUMMARY_REJECTED',
                reason='MISSING_EVIDENCE_METADATA')
            return
        canonical_ids = [self._canonical_evidence_id(
            self.peer_robot_id, self.robot_id, source_id, target_id)
            for source_id, target_id in zip(source_ids, target_ids)]
        peer_hypothesis_sets = None
        if self.registration_backend == 'mrpt':
            peer_hypothesis_sets = [
                register_crop_hypotheses(
                    source, target, backend=self.registration_backend,
                    minimum_agreement=0.0,
                    mrpt_max_kld=self.mrpt_max_kld,
                    mrpt_max_modes=self.mrpt_max_modes_per_call,
                    mrpt_repetitions=self.mrpt_repetitions_per_pair,
                    max_distinct_modes=self.mrpt_max_distinct_modes_per_pair)
                for source, target in evidence_pairs]
        result = self._run_registration(
            evidence_pairs, 'peer_summary_verification', source_ids[0],
            evidence_timestamps=evidence_timestamps,
            source_viewpoints=[self._keyframe_viewpoint(target_id)
                               for target_id in target_ids],
            target_viewpoints=[self._descriptor_viewpoint(
                self.peer_descriptors.get(source_id))
                               for source_id in source_ids],
            evidence_ids=canonical_ids,
            hypothesis_sets=peer_hypothesis_sets)
        if not result.accepted:
            self._record_diagnostic_event(
                'PEER_HYPOTHESIS_SUMMARY_REJECTED',
                reason=str(result.reason),
                selector_status=str(result.selector_status),
                consistent_constraint_count=int(
                    result.consistent_constraint_count))
            return
        # Registration above is R2 -> R1; the canonical protocol publishes
        # R1 -> R2.  Invert exactly once at this protocol boundary.
        canonical_result = replace(
            result, transform=invert_se2(result.transform))
        own_descriptor = self.keyframes[target_ids[0]][0]
        peer_descriptor = self.peer_descriptors[source_ids[0]]
        summary_message = self._hypothesis_message(
            own_descriptor, peer_descriptor, canonical_result,
            status='CANDIDATE', accepted=False, rejection_reason='',
            evidence_source_keyframe_ids=target_ids,
            evidence_target_keyframe_ids=source_ids)
        self.local_hypothesis_summary = summary_message
        self.local_hypothesis_result = canonical_result
        self._verified_peer_summary_hash = summary_hash
        self.hypothesis_pub.publish(summary_message)
        self.counters['hypothesis_summaries_published'] += 1
        self._record_diagnostic_event(
            'LOCAL_PEER_SUMMARY_VERIFIED',
            evidence_set_hash=str(summary_message.evidence_set_hash),
            consistent_constraint_count=int(
                canonical_result.consistent_constraint_count))
        if self._peer_hypothesis_agrees(summary_message):
            self._publish_canonical_proposal(summary_message,
                                             canonical_result)

    def _publish_canonical_proposal(self, summary, result):
        """Publish exactly one canonical proposal after peer verification."""
        if self.batch_proposal_published or self.robot_id != min(
                self.robot_id, self.peer_robot_id):
            return
        proposal = copy.deepcopy(summary)
        proposal.status = 'PROPOSED'
        proposal.accepted = False
        key = (str(proposal.source_keyframe_id),
               str(proposal.target_keyframe_id))
        self.pending_proposals[key] = result
        self.pending_proposal_messages[key] = copy.deepcopy(proposal)
        self.pending_proposal_evidence_hashes[key] = str(
            proposal.evidence_set_hash)
        self.hypothesis_pub.publish(proposal)
        self.counters['proposals_published'] += 1
        self.batch_proposal_published = True
        self._record_diagnostic_event(
            'CANONICAL_PROPOSAL_PUBLISHED',
            evidence_set_hash=str(proposal.evidence_set_hash),
            source_keyframe_id=str(proposal.source_keyframe_id),
            target_keyframe_id=str(proposal.target_keyframe_id))

    def _publish_evidence_announcement(self, candidate, result,
                                       own_crop, peer_crop, hypotheses=None):
        """Advertise all accepted modes for one physical pair.

        This message is deliberately non-accepting.  The recipient requests
        the advertised source crop and runs the same geometric registration
        locally; only the resulting locally verified constraint enters its
        selector.  Thus this path shares evidence, never trust, and cannot
        trigger a TF or fusion handoff by itself.
        """
        peer_key, own_key, peer_descriptor, own_descriptor = (
            self._candidate_fields(candidate))
        evidence_id = self._canonical_evidence_id(
            self.robot_id, self.peer_robot_id, own_key, peer_key)
        modes = tuple(hypotheses or (result,))
        for mode in modes:
            mode_signature = (
                evidence_id,
                round(float(mode.transform[0]) / 0.05),
                round(float(mode.transform[1]) / 0.05),
                round(float(mode.transform[2]) / math.radians(0.5)))
            if mode_signature in self.evidence_announcements_published:
                continue
            message = self._hypothesis_message(
                own_descriptor, peer_descriptor, mode,
                status='EVIDENCE', accepted=False, rejection_reason='',
                evidence_source_keyframe_ids=[own_key],
                evidence_target_keyframe_ids=[peer_key])
            message.constraint_count = 1
            message.consistent_constraint_count = 1
            message.selector_status = 'INSUFFICIENT_EVIDENCE'
            message.selector_runner_up_margin = 0.0
            self.hypothesis_pub.publish(message)
            self.evidence_announcements_published.add(mode_signature)
            self.counters['evidence_announcements_published'] += 1
            self._record_diagnostic_event(
                'EVIDENCE_ANNOUNCEMENT_PUBLISHED',
                evidence_id=evidence_id,
                source_keyframe_id=str(own_key),
                target_keyframe_id=str(peer_key),
                mode_index=int(getattr(mode, 'mode_index', -1)),
                mode_support=int(getattr(mode, 'mode_support', 1)),
                transform=[float(value) for value in mode.transform])

    def _message_canonical_mode(self, message):
        """Decode one peer mode into the canonical R1->R2 convention."""
        raw = self._summary_transform(message)
        source_robot = str(message.source_robot_id)
        if source_robot == 'robot1':
            transform = raw
            forward = float(getattr(message, 'geometric_inlier_ratio', 0.0))
            reverse = float(getattr(message, 'reverse_inlier_ratio', forward))
        else:
            transform = invert_se2(raw)
            forward = float(getattr(message, 'reverse_inlier_ratio',
                                   getattr(message, 'geometric_inlier_ratio', 0.0)))
            reverse = float(getattr(message, 'geometric_inlier_ratio', 0.0))
        covariance = tuple(float(value) for value in getattr(
            message, 'covariance', (0.0,) * 36))
        if len(covariance) not in (9, 36):
            covariance = (0.0,) * 36
        return RegistrationResult(
            accepted=True, transform=tuple(float(value) for value in transform),
            covariance=covariance, inlier_ratio=forward,
            reverse_inlier_ratio=reverse,
            residual_m=float(getattr(message, 'registration_residual_m',
                                     math.inf)),
            occupied_free_agreement=float(getattr(
                message, 'occupied_free_agreement', 0.0)),
            overlap_fraction=float(getattr(message, 'overlap_fraction', 0.0)),
            reason='PEER_ANNOUNCED_MODE',
            backend='mrpt',
            mode_index=int(getattr(message, 'mode_index', -1)),
            mode_log_weight=float(getattr(message, 'mode_log_weight',
                                          -math.inf)),
            mode_support=max(1, int(getattr(message, 'mode_support', 1))))

    def _add_peer_canonical_mode(self, message, evidence_id, source_key,
                                 target_key):
        """Stage a peer mode; exact local re-registration remains mandatory."""
        mode = self._message_canonical_mode(message)
        source_is_robot1 = str(message.source_robot_id) == 'robot1'
        source_viewpoint = (
            (float(message.source_viewpoint_x),
             float(message.source_viewpoint_y),
             float(message.source_viewpoint_yaw))
            if source_is_robot1 and bool(getattr(
                message, 'source_viewpoint_available', False)) else
            ((float(message.target_viewpoint_x),
              float(message.target_viewpoint_y),
              float(message.target_viewpoint_yaw))
             if not source_is_robot1 and bool(getattr(
                 message, 'target_viewpoint_available', False)) else None))
        target_viewpoint = (
            (float(message.target_viewpoint_x),
             float(message.target_viewpoint_y),
             float(message.target_viewpoint_yaw))
            if source_is_robot1 and bool(getattr(
                message, 'target_viewpoint_available', False)) else
            ((float(message.source_viewpoint_x),
              float(message.source_viewpoint_y),
              float(message.source_viewpoint_yaw))
             if not source_is_robot1 and bool(getattr(
                 message, 'source_viewpoint_available', False)) else None))
        source_descriptor = self.keyframes.get(source_key, (None,))[0]
        if source_descriptor is None:
            source_descriptor = self.peer_descriptors.get(source_key)
        target_descriptor = self.keyframes.get(target_key, (None,))[0]
        if target_descriptor is None:
            target_descriptor = self.peer_descriptors.get(target_key)
        pair = None
        if self.robot_id == 'robot1':
            local = self.keyframes.get(source_key)
            remote = self.received_peer_crops.get(target_key)
            if local is not None and remote is not None:
                pair = (local[1], remote)
        else:
            local = self.keyframes.get(target_key)
            remote = self.received_peer_crops.get(source_key)
            if local is not None and remote is not None:
                pair = (remote, local[1])
        record = self.canonical_constraint_pool.get(evidence_id)
        if record is None:
            self.canonical_constraint_pool[evidence_id] = {
                'evidence_id': evidence_id,
                'source_key': source_key,
                'target_key': target_key,
                'pair': pair,
                'result': mode,
                'hypotheses': (mode,),
                'source_content': None,
                'target_content': None,
                'source_viewpoint': source_viewpoint,
                'target_viewpoint': target_viewpoint,
                'source_descriptor': source_descriptor,
                'target_descriptor': target_descriptor,
                'timestamps': (
                    None if source_descriptor is None else
                    self._stamp_ns(source_descriptor),
                    None if target_descriptor is None else
                    self._stamp_ns(target_descriptor)),
            }
        else:
            record['hypotheses'] = self._merge_canonical_modes(
                record.get('hypotheses', ()), (mode,))
            if record.get('source_viewpoint') is None:
                record['source_viewpoint'] = source_viewpoint
            if record.get('target_viewpoint') is None:
                record['target_viewpoint'] = target_viewpoint
            if record.get('source_descriptor') is None:
                record['source_descriptor'] = source_descriptor
            if record.get('target_descriptor') is None:
                record['target_descriptor'] = target_descriptor
        self._record_diagnostic_event(
            'CANONICAL_PEER_MODE_RECEIVED', evidence_id=evidence_id,
            source_keyframe_id=source_key, target_keyframe_id=target_key,
            mode_index=int(getattr(mode, 'mode_index', -1)),
            mode_support=int(getattr(mode, 'mode_support', 1)),
            requires_local_reregistration=True)

    def _queue_peer_evidence_reverification(self, message):
        """Request and independently re-register one peer-advertised pair."""
        source_key = str(message.source_keyframe_id)
        target_key = str(message.target_keyframe_id)
        peer_descriptor = self.peer_descriptors.get(source_key)
        own_entry = self.keyframes.get(target_key)
        if peer_descriptor is None or own_entry is None:
            self.pending_peer_evidence_announcements[
                str(message.evidence_set_hash) or
                f'{source_key}|{target_key}'] = message
            return False
        pair_key = (target_key, source_key)
        if pair_key in self.evidence_pairs:
            return True
        request_key = (source_key, int(peer_descriptor.checksum))
        if request_key in self.pending_requests:
            return True
        evidence_id = self._canonical_evidence_id(
            self.peer_robot_id, self.robot_id, source_key, target_key)
        # A single peer announcement is independent evidence, but duplicate
        # deliveries of the same announcement must not start duplicate crop
        # requests or registrations.
        if evidence_id in self.peer_evidence_announcements and evidence_id in getattr(
                self, '_peer_evidence_requested', set()):
            return True
        if not hasattr(self, '_peer_evidence_requested'):
            self._peer_evidence_requested = set()
        self._peer_evidence_requested.add(evidence_id)
        queued = self._queue_crop_request(
            source_key, target_key, peer_descriptor, own_entry[0],
            self.verification_batches.batch_id or 0,
            f'{self.robot_id}-peer-evidence-{source_key}-{target_key}')
        if queued:
            self.counters['evidence_reverification_requests'] += 1
            self._record_diagnostic_event(
                'PEER_EVIDENCE_REVERIFICATION_REQUESTED',
                evidence_id=evidence_id,
                source_keyframe_id=source_key,
                target_keyframe_id=target_key)
        return bool(queued)

    def _process_pending_peer_evidence(self):
        if not self.pending_peer_evidence_announcements:
            return
        for key, message in list(
                self.pending_peer_evidence_announcements.items()):
            if self._queue_peer_evidence_reverification(message):
                self.pending_peer_evidence_announcements.pop(key, None)

    def _handle_peer_evidence_announcement(self, message):
        if {str(message.source_robot_id), str(message.target_robot_id)} != {
                self.robot_id, self.peer_robot_id}:
            return
        source_ids = [str(value) for value in getattr(
            message, 'evidence_source_keyframe_ids', [])]
        target_ids = [str(value) for value in getattr(
            message, 'evidence_target_keyframe_ids', [])]
        if len(source_ids) != 1 or len(target_ids) != 1:
            self._record_diagnostic_event(
                'PEER_EVIDENCE_ANNOUNCEMENT_REJECTED',
                reason='EXPECTED_ONE_CONSTRAINT',
                source_keyframe_ids=source_ids,
                target_keyframe_ids=target_ids)
            return
        if (str(message.source_robot_id) != self.peer_robot_id or
                str(message.target_robot_id) != self.robot_id):
            self._record_diagnostic_event(
                'PEER_EVIDENCE_ANNOUNCEMENT_REJECTED',
                reason='DIRECTION_OR_SCOPE_MISMATCH')
            return
        evidence_id = self._canonical_evidence_id(
            self.peer_robot_id, self.robot_id, source_ids[0], target_ids[0])
        advertised_id = str(getattr(message, 'physical_evidence_id', ''))
        if advertised_id and advertised_id != evidence_id:
            self._record_diagnostic_event(
                'PEER_EVIDENCE_ANNOUNCEMENT_REJECTED',
                reason='PHYSICAL_EVIDENCE_ID_MISMATCH',
                advertised_evidence_id=advertised_id,
                canonical_evidence_id=evidence_id)
            return
        if evidence_id not in self.peer_evidence_announcements:
            self.peer_evidence_announcements[evidence_id] = message
        self.counters['evidence_announcements_received'] += 1
        self._add_peer_canonical_mode(
            message, evidence_id,
            target_ids[0] if str(message.target_robot_id) == 'robot1'
            else source_ids[0],
            source_ids[0] if str(message.target_robot_id) == 'robot1'
            else target_ids[0])
        self._record_diagnostic_event(
            'PEER_EVIDENCE_ANNOUNCEMENT_RECEIVED',
            evidence_id=evidence_id,
            source_keyframe_id=source_ids[0],
            target_keyframe_id=target_ids[0],
            advertised_transform=[
                float(message.source_to_target.translation.x),
                float(message.source_to_target.translation.y),
                float(self._summary_transform(message)[2])])
        # One request is sufficient for all alternatives belonging to this
        # physical pair.  Later mode announcements enrich the bounded pool
        # without consuming another crop/verification request.
        if evidence_id not in self._peer_evidence_requested:
            self._queue_peer_evidence_reverification(message)

    def _publish_local_evidence_crops(self, keyframe_ids,
                                      evidence_candidates=None):
        """Make the initiator's bounded evidence available for peer verification."""
        retained_by_own = {}
        for candidate in evidence_candidates or ():
            peer_key, own_key, _, _ = self._candidate_fields(candidate)
            evidence = self.evidence_pairs.get((own_key, peer_key))
            if evidence is not None:
                retained_by_own.setdefault(own_key, (
                    candidate[3], evidence[0]))
        for keyframe_id in keyframe_ids:
            stored = self.keyframes.get(keyframe_id)
            if stored is None:
                retained = retained_by_own.get(keyframe_id)
                if retained is None:
                    continue
                descriptor, crop = retained
            else:
                descriptor, crop = stored
            message = LocalMapCrop()
            message.header = descriptor.header
            message.source_robot_id = self.robot_id
            message.keyframe_id = keyframe_id
            message.map_epoch = descriptor.map_epoch
            message.descriptor_checksum = descriptor.checksum
            message.occupancy_checksum = self._occupancy_checksum(crop.values)
            message.occupancy_grid = self._crop_message(crop, descriptor.header)
            self.crop_pub.publish(message)
            self.counters['crops_sent'] += 1
            self._write_physical_evidence_diagnostic(
                'CROP_RESPONSE_SENT',
                keyframe_id=str(keyframe_id),
                map_epoch=int(descriptor.map_epoch),
                checksum=int(descriptor.checksum),
                crop=self._crop_geometry(
                    crop, keyframe_id, descriptor.map_epoch,
                    descriptor.checksum),
                status='SENT')

    @staticmethod
    def _grid_crop_from_message(message):
        if (int(message.info.width) <= 0 or int(message.info.height) <= 0 or
                not math.isfinite(float(message.info.resolution)) or
                float(message.info.resolution) <= 0.0 or
                len(message.data) != int(message.info.width) *
                int(message.info.height)):
            raise ValueError('invalid native occupancy crop metadata')
        values = np.asarray(message.data, dtype=np.int16).reshape(
            (message.info.height, message.info.width))
        if not np.isfinite(values.astype(np.float64)).all():
            raise ValueError('nonfinite native occupancy crop values')
        return GridCrop(
            values=values, resolution=float(message.info.resolution),
            origin_x=float(message.info.origin.position.x),
            origin_y=float(message.info.origin.position.y),
            origin_yaw=UnknownPoseFrontend._yaw(message.info.origin.orientation))

    @staticmethod
    def _transform_delta(first, second):
        translation = math.hypot(
            float(first.transform.translation.x - second.transform.translation.x),
            float(first.transform.translation.y - second.transform.translation.y))
        yaw_first = math.atan2(
            2.0 * (first.transform.rotation.w * first.transform.rotation.z),
            1.0 - 2.0 * first.transform.rotation.z ** 2)
        yaw_second = math.atan2(
            2.0 * (second.transform.rotation.w * second.transform.rotation.z),
            1.0 - 2.0 * second.transform.rotation.z ** 2)
        yaw = abs(math.atan2(math.sin(yaw_first - yaw_second),
                             math.cos(yaw_first - yaw_second)))
        return translation, yaw

    def _hypothesis_message(
            self, own, peer, result, status, accepted, rejection_reason,
            evidence_source_keyframe_ids=None,
            evidence_target_keyframe_ids=None,
            stationary_canonical_seed=None,
            stationary_canonical_source_snapshot_id='',
            stationary_canonical_target_snapshot_id='',
            stationary_canonical_source_map_hash='',
            stationary_canonical_target_map_hash=''):
        message = RelativePoseHypothesis()
        message.header = own.header
        message.source_robot_id = self.robot_id
        message.target_robot_id = self.peer_robot_id
        message.source_keyframe_id = own.keyframe_id
        message.target_keyframe_id = peer.keyframe_id
        message.source_to_target.translation.x = float(result.transform[0])
        message.source_to_target.translation.y = float(result.transform[1])
        message.source_to_target.rotation.z = math.sin(result.transform[2] / 2.0)
        message.source_to_target.rotation.w = math.cos(result.transform[2] / 2.0)
        message.covariance = list(result.covariance)
        match = self.matches.get((peer.keyframe_id, own.keyframe_id))
        message.descriptor_similarity = float(match.similarity if match else 0.0)
        message.descriptor_margin = float(match.margin if match else 0.0)
        message.geometric_inlier_ratio = float(result.inlier_ratio)
        message.reverse_inlier_ratio = float(
            getattr(result, 'reverse_inlier_ratio', result.inlier_ratio))
        message.registration_residual_m = float(result.residual_m)
        message.occupied_free_agreement = float(result.occupied_free_agreement)
        message.overlap_fraction = float(result.overlap_fraction)
        message.temporal_consistency = float(temporal_consistency(
            [own.header.stamp.sec * 1_000_000_000 + own.header.stamp.nanosec,
             peer.header.stamp.sec * 1_000_000_000 + peer.header.stamp.nanosec]))
        descriptor_confidence = (
            0.30 * message.descriptor_similarity +
            0.15 * min(1.0, message.descriptor_margin / 0.10) +
            0.20 * message.geometric_inlier_ratio +
            0.15 * message.occupied_free_agreement +
            0.10 * message.overlap_fraction +
            0.10 * message.temporal_consistency)
        geometric_confidence = float(result.final_confidence)
        message.final_confidence = float(max(0.0, min(1.0,
            0.45 * descriptor_confidence +
            0.55 * geometric_confidence
            if result.constraint_count > 1 else descriptor_confidence)))
        message.status = status
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        source_ids = list(evidence_source_keyframe_ids or [own.keyframe_id])
        target_ids = list(evidence_target_keyframe_ids or [peer.keyframe_id])
        message.evidence_set_hash = self._canonical_evidence_hash(
            self.robot_id, self.peer_robot_id, source_ids, target_ids)
        message.physical_evidence_id = self._canonical_evidence_id(
            self.robot_id, self.peer_robot_id, own.keyframe_id,
            peer.keyframe_id)
        message.mode_index = int(getattr(result, 'mode_index', -1))
        message.mode_support = max(1, int(getattr(result, 'mode_support', 1)))
        message.mode_log_weight = float(getattr(
            result, 'mode_log_weight', -math.inf))
        if self.robot_id == 'robot1':
            source_viewpoint = self._keyframe_pose(own.keyframe_id)
            target_viewpoint = self._descriptor_viewpoint(peer)
        else:
            source_viewpoint = self._descriptor_viewpoint(peer)
            target_viewpoint = self._keyframe_pose(own.keyframe_id)
        message.source_viewpoint_available = source_viewpoint is not None
        message.target_viewpoint_available = target_viewpoint is not None
        if source_viewpoint is not None:
            message.source_viewpoint_x = float(source_viewpoint[0])
            message.source_viewpoint_y = float(source_viewpoint[1])
            message.source_viewpoint_yaw = float(source_viewpoint[2])
        if target_viewpoint is not None:
            message.target_viewpoint_x = float(target_viewpoint[0])
            message.target_viewpoint_y = float(target_viewpoint[1])
            message.target_viewpoint_yaw = float(target_viewpoint[2])
        message.evidence_source_keyframe_ids = source_ids
        message.evidence_target_keyframe_ids = target_ids
        message.stationary_witness_scheme_version = (
            STATIONARY_WITNESS_SCHEME_VERSION
            if stationary_canonical_seed is not None else '')
        message.stationary_canonical_source_snapshot_id = str(
            stationary_canonical_source_snapshot_id)
        message.stationary_canonical_target_snapshot_id = str(
            stationary_canonical_target_snapshot_id)
        message.stationary_canonical_source_map_hash = str(
            stationary_canonical_source_map_hash)
        message.stationary_canonical_target_map_hash = str(
            stationary_canonical_target_map_hash)
        if stationary_canonical_seed is not None:
            message.stationary_canonical_seed = [float(value) for value in
                                                  stationary_canonical_seed]
        message.constraint_count = int(result.constraint_count)
        message.consistent_constraint_count = int(
            result.consistent_constraint_count)
        message.spatial_baseline_m = float(result.spatial_baseline_m)
        message.angular_spread_rad = float(result.angular_spread_rad)
        message.median_registration_residual_m = float(
            result.median_residual_m)
        message.p95_registration_residual_m = float(result.p95_residual_m)
        message.projected_error_m = float(result.projected_error_m)
        message.translation_uncertainty_m = float(
            result.translation_uncertainty_m)
        message.yaw_uncertainty_rad = float(result.yaw_uncertainty_rad)
        message.condition_number = float(result.condition_number)
        # Replicate the robust-selector model evidence.  The transport fields
        # are diagnostic/protocol evidence only; the accepted flag remains
        # gated by the existing two-sided handoff protocol.
        message.selector_status = str(getattr(
            result, 'selector_status', 'INSUFFICIENT_EVIDENCE'))
        message.selector_score = float(getattr(result, 'selector_score', 0.0))
        message.selector_null_score = float(getattr(
            result, 'selector_null_score', 0.0))
        message.selector_runner_up_score = float(getattr(
            result, 'selector_runner_up_score', -math.inf))
        message.selector_runner_up_margin = float(getattr(
            result, 'selector_runner_up_margin', 0.0))
        message.selector_inlier_probabilities = [float(value) for value in
            getattr(result, 'selector_inlier_probabilities', ())]
        return message

    def hypothesis_callback(self, message):
        if getattr(self, '_post_handoff_quiesced', False):
            return
        self._record_diagnostic_event(
            'HYPOTHESIS_RECEIVED',
            source_robot_id=str(message.source_robot_id),
            target_robot_id=str(message.target_robot_id),
            source_keyframe_id=str(message.source_keyframe_id),
            target_keyframe_id=str(message.target_keyframe_id),
            status=str(message.status),
            accepted=bool(message.accepted),
            final_confidence=float(message.final_confidence),
            pending_target_proposal=bool(self.pending_target_proposal),
            already_accepted=bool(self.accepted is not None))
        if {message.source_robot_id, message.target_robot_id} != {
                self.robot_id, self.peer_robot_id}:
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_SCOPE',
                source_robot_id=str(message.source_robot_id),
                target_robot_id=str(message.target_robot_id))
            return
        if self.accepted is not None:
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_ALREADY_ACCEPTED',
                status=str(message.status))
            return
        if self._is_full_map_hypothesis(message):
            if (message.status == 'PROPOSED' and
                    self.robot_id == message.target_robot_id):
                self._handle_full_map_proposal(message)
                return
            if (message.status in ('ACCEPTED', 'REJECTED') and
                    self.robot_id == message.source_robot_id):
                self._handle_full_map_ack(message)
                return
            self._record_diagnostic_event(
                'FULL_MAP_HYPOTHESIS_IGNORED_SCOPE',
                status=str(message.status))
            return
        if message.status == 'EVIDENCE':
            self._handle_peer_evidence_announcement(message)
            return
        if message.status == 'CANDIDATE':
            if self.robot_id != message.target_robot_id:
                self._record_diagnostic_event(
                    'HYPOTHESIS_SUMMARY_IGNORED_SCOPE',
                    source_robot_id=str(message.source_robot_id),
                    target_robot_id=str(message.target_robot_id))
                return
            self.peer_hypothesis_summary = message
            self.counters['hypothesis_summaries_received'] += 1
            incoming_hash = str(getattr(message, 'evidence_set_hash', ''))
            if incoming_hash and incoming_hash == self._verified_peer_summary_hash:
                self._record_diagnostic_event(
                    'HYPOTHESIS_SUMMARY_ALREADY_VERIFIED',
                    evidence_set_hash=incoming_hash)
                return
            if self.peer_summary_source_ids:
                self._record_diagnostic_event(
                    'HYPOTHESIS_SUMMARY_IGNORED_PENDING_VERIFICATION',
                    evidence_set_hash=str(message.evidence_set_hash))
                return
            self.peer_summary_source_ids = set(
                str(value) for value in getattr(
                    message, 'evidence_source_keyframe_ids', []))
            self.peer_summary_target_ids = {
                str(source): str(target)
                for source, target in zip(
                    getattr(message, 'evidence_source_keyframe_ids', []),
                    getattr(message, 'evidence_target_keyframe_ids', []))}
            self._request_source_for_confirmation(message)
            # The summary may arrive after ordinary verification traffic has
            # already delivered these same peer crops.  Verify immediately
            # when the bounded cache already contains the full requested set;
            # otherwise the normal response path triggers verification on the
            # last crop.  This is intentionally symmetric across robot IDs.
            if (not self.peer_summary_source_ids or all(
                    source_id in self.received_peer_crops
                    for source_id in self.peer_summary_source_ids)):
                self.peer_summary_source_ids.clear()
                self._verify_peer_hypothesis_summary()
            self._record_diagnostic_event(
                'PEER_HYPOTHESIS_SUMMARY_RECEIVED',
                evidence_set_hash=str(message.evidence_set_hash),
                selector_status=str(message.selector_status),
                consistent_constraint_count=int(
                    message.consistent_constraint_count))
            if (self.robot_id == min(self.robot_id, self.peer_robot_id) and
                    self.local_hypothesis_summary is not None and
                    self.local_hypothesis_result is not None and
                    self._peer_hypothesis_agrees(
                        self.local_hypothesis_summary)):
                self._publish_canonical_proposal(
                    self.local_hypothesis_summary,
                    self.local_hypothesis_result)
            return
        if message.status == 'PROPOSED' and self.robot_id == message.target_robot_id:
            if self.pending_target_proposal:
                self._record_diagnostic_event(
                    'HYPOTHESIS_IGNORED_PENDING_TARGET',
                    source_keyframe_id=str(message.source_keyframe_id),
                    target_keyframe_id=str(message.target_keyframe_id))
                return
            self.pending_target_proposal = True
            self.peer_proposals[message.source_keyframe_id] = message
            self._record_diagnostic_event(
                'HYPOTHESIS_PROPOSAL_ACCEPTED_FOR_CONFIRMATION',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id),
                evidence_source_keyframe_ids=[
                    str(value) for value in
                    getattr(message, 'evidence_source_keyframe_ids', [])],
                evidence_target_keyframe_ids=[
                    str(value) for value in
                    getattr(message, 'evidence_target_keyframe_ids', [])])
            self._request_source_for_confirmation(message)
            # The source crops may already be cached from the independent
            # peer-summary verification.  Do not wait for a duplicate crop
            # response to trigger the confirmation path.
            self._try_confirm_pending_proposal(message)
            return
        if message.status == 'REJECTED':
            if self.robot_id == message.source_robot_id:
                self.negotiation_started = False
                # The local registration may have passed while the peer's
                # independent confirmation failed.  This is not a completed
                # handoff: release the source latch and reopen the bounded
                # batch controller so only novel evidence can be tried next.
                self.batch_proposal_published = False
                self.pending_proposals.pop(
                    (str(message.source_keyframe_id),
                     str(message.target_keyframe_id)), None)
                getattr(self, 'pending_proposal_messages', {}).pop(
                    (str(message.source_keyframe_id),
                     str(message.target_keyframe_id)), None)
                getattr(self, 'pending_proposal_evidence_hashes', {}).pop(
                    (str(message.source_keyframe_id),
                     str(message.target_keyframe_id)), None)
                self.verification_batches.reopen_after_rejected_proposal()
            if self.robot_id == message.target_robot_id:
                # A rejected proposal terminates the responder's pending
                # confirmation too.  Without this reset, the responder
                # remains latched forever and ignores every later proposal,
                # including a valid multi-keyframe consensus from a new
                # acquisition batch.
                self.pending_target_proposal = False
                self.peer_proposals.pop(
                    str(message.source_keyframe_id), None)
            self._record_diagnostic_event(
                'HYPOTHESIS_REJECTED',
                source_robot_id=str(message.source_robot_id),
                target_robot_id=str(message.target_robot_id),
                rejection_reason=str(message.rejection_reason))
            return
        if not should_accept_hypothesis(
                self.accepted, message.status, message.accepted,
                message.final_confidence):
            self._record_diagnostic_event(
                'HYPOTHESIS_IGNORED_ACCEPTANCE_GATE',
                status=str(message.status), accepted=bool(message.accepted),
                final_confidence=float(message.final_confidence))
            return
        if self.robot_id != message.source_robot_id:
            self.accepted = message
            self.accepted_ros_time_s = (
                self.get_clock().now().nanoseconds * 1.0e-9)
            self.accepted_wall = time.monotonic()
            self._record_diagnostic_event(
                'HYPOTHESIS_TARGET_ACCEPTED',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id))
            self.publish_local_map(force=True)
            self._enter_post_handoff_quiescence()
            return
        proposal = self.pending_proposals.get(
            (message.source_keyframe_id, message.target_keyframe_id))
        if proposal is None or not proposal.accepted:
            self._record_diagnostic_event(
                'HYPOTHESIS_ACK_IGNORED_NO_PENDING_PROPOSAL',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id),
                pending_proposal_count=len(self.pending_proposals))
            return
        peer_selector_status = str(getattr(message, 'selector_status', ''))
        proposal_key = (str(message.source_keyframe_id),
                        str(message.target_keyframe_id))
        proposal_message = getattr(
            self, 'pending_proposal_messages', {}).get(proposal_key)
        proposal_evidence_set_hash = getattr(
            self, 'pending_proposal_evidence_hashes', {}).get(proposal_key, '')
        peer_evidence_set_hash = str(getattr(message, 'evidence_set_hash', ''))
        evidence_hash_matches = (
            not peer_evidence_set_hash or
            (bool(proposal_evidence_set_hash) and
             peer_evidence_set_hash == proposal_evidence_set_hash))
        peer_selector_accepted = (
            peer_selector_status in ('', 'ACCEPTED_HYPOTHESIS') and
            int(getattr(message, 'consistent_constraint_count', 0)) >=
            int(self.min_consistent_constraints))
        if not evidence_hash_matches or not peer_selector_accepted:
            self._record_diagnostic_event(
                'HYPOTHESIS_ACK_IGNORED_ROBUST_SELECTOR_MISMATCH',
                proposal_evidence_set_hash=str(proposal_evidence_set_hash),
                peer_evidence_set_hash=peer_evidence_set_hash,
                peer_selector_status=peer_selector_status,
                peer_consistent_constraint_count=int(getattr(
                    message, 'consistent_constraint_count', 0)))
            return
        if proposal_message is not None:
            # The proposal already contains the source-to-target transform
            # computed by the canonical source and independently verified by
            # the target.  Reuse that immutable envelope rather than
            # recomputing it from descriptors that may have been evicted.
            final = copy.deepcopy(proposal_message)
            final.status = 'ACCEPTED'
            final.accepted = True
            final.rejection_reason = ''
            final.evidence_set_hash = str(
                message.evidence_set_hash or
                proposal_message.evidence_set_hash)
            if getattr(message, 'evidence_source_keyframe_ids', []):
                final.evidence_source_keyframe_ids = list(
                    message.evidence_source_keyframe_ids)
            if getattr(message, 'evidence_target_keyframe_ids', []):
                final.evidence_target_keyframe_ids = list(
                    message.evidence_target_keyframe_ids)
            self._record_diagnostic_event(
                'HYPOTHESIS_ACK_FINALIZED_FROM_RETAINED_PROPOSAL',
                source_keyframe_id=str(message.source_keyframe_id),
                target_keyframe_id=str(message.target_keyframe_id),
                keyframe_evicted=bool(
                    message.source_keyframe_id not in self.keyframes or
                    message.target_keyframe_id not in self.peer_descriptors))
        else:
            # Defensive compatibility path for proposals created before the
            # retained-envelope field existed.  Keep the old safety check;
            # never synthesize a final message without both descriptors.
            own = self.keyframes.get(message.source_keyframe_id)
            peer = self.peer_descriptors.get(message.target_keyframe_id)
            if own is None or peer is None:
                self._record_diagnostic_event(
                    'HYPOTHESIS_ACK_IGNORED_MISSING_KEYFRAME',
                    source_keyframe_id=str(message.source_keyframe_id),
                    target_keyframe_id=str(message.target_keyframe_id))
                return
            final = self._hypothesis_message(
                own[0], peer, proposal, status='ACCEPTED', accepted=True,
                rejection_reason='',
                evidence_source_keyframe_ids=list(getattr(
                    message, 'evidence_source_keyframe_ids', [])),
                evidence_target_keyframe_ids=list(getattr(
                    message, 'evidence_target_keyframe_ids', [])))
        self.pending_proposals.pop(proposal_key, None)
        getattr(self, 'pending_proposal_messages', {}).pop(proposal_key, None)
        getattr(self, 'pending_proposal_evidence_hashes', {}).pop(
            proposal_key, None)
        self.hypothesis_pub.publish(final)
        self._record_diagnostic_event(
            'HYPOTHESIS_CANONICAL_ACCEPTED',
            source_keyframe_id=str(message.source_keyframe_id),
            target_keyframe_id=str(message.target_keyframe_id),
            evidence_set_hash=str(message.evidence_set_hash))
        self.counters['accepted_hypotheses'] += 1
        self.accepted = final
        self.accepted_ros_time_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.accepted_wall = time.monotonic()
        self.publish_accepted_tf()
        self.publish_local_map(force=True)
        self._enter_post_handoff_quiescence()

    def _ack_message(self, proposal, result, accepted, rejection_reason):
        message = RelativePoseHypothesis()
        message.header = proposal.header
        message.source_robot_id = proposal.source_robot_id
        message.target_robot_id = proposal.target_robot_id
        message.source_keyframe_id = proposal.source_keyframe_id
        message.target_keyframe_id = proposal.target_keyframe_id
        message.source_to_target.translation.x = float(result.transform[0])
        message.source_to_target.translation.y = float(result.transform[1])
        message.source_to_target.rotation.z = math.sin(result.transform[2] / 2.0)
        message.source_to_target.rotation.w = math.cos(result.transform[2] / 2.0)
        message.covariance = list(result.covariance)
        message.descriptor_similarity = proposal.descriptor_similarity
        message.descriptor_margin = proposal.descriptor_margin
        message.geometric_inlier_ratio = float(result.inlier_ratio)
        message.reverse_inlier_ratio = float(
            getattr(result, 'reverse_inlier_ratio', result.inlier_ratio))
        message.registration_residual_m = float(result.residual_m)
        message.occupied_free_agreement = float(result.occupied_free_agreement)
        message.overlap_fraction = float(result.overlap_fraction)
        message.temporal_consistency = proposal.temporal_consistency
        message.final_confidence = float(
            proposal.final_confidence if accepted else 0.0)
        message.status = 'ACCEPTED' if accepted else 'REJECTED'
        message.rejection_reason = rejection_reason
        message.accepted = bool(accepted)
        message.evidence_set_hash = proposal.evidence_set_hash
        message.physical_evidence_id = getattr(
            proposal, 'physical_evidence_id', '')
        message.mode_index = int(getattr(proposal, 'mode_index', -1))
        message.mode_support = int(getattr(proposal, 'mode_support', 1))
        message.mode_log_weight = float(getattr(
            proposal, 'mode_log_weight', -math.inf))
        for field in (
                'source_viewpoint_available', 'source_viewpoint_x',
                'source_viewpoint_y', 'source_viewpoint_yaw',
                'target_viewpoint_available', 'target_viewpoint_x',
                'target_viewpoint_y', 'target_viewpoint_yaw'):
            setattr(message, field, getattr(proposal, field, False if
                                            field.endswith('available') else 0.0))
        message.evidence_source_keyframe_ids = list(
            proposal.evidence_source_keyframe_ids)
        message.evidence_target_keyframe_ids = list(
            proposal.evidence_target_keyframe_ids)
        for field in (
                'stationary_witness_scheme_version',
                'stationary_canonical_source_snapshot_id',
                'stationary_canonical_target_snapshot_id',
                'stationary_canonical_source_map_hash',
                'stationary_canonical_target_map_hash'):
            setattr(message, field, str(getattr(proposal, field, '')))
        message.stationary_canonical_seed = list(getattr(
            proposal, 'stationary_canonical_seed', (0.0, 0.0, 0.0)))
        message.constraint_count = proposal.constraint_count
        message.consistent_constraint_count = proposal.consistent_constraint_count
        message.spatial_baseline_m = proposal.spatial_baseline_m
        message.angular_spread_rad = proposal.angular_spread_rad
        message.median_registration_residual_m = proposal.median_registration_residual_m
        message.p95_registration_residual_m = proposal.p95_registration_residual_m
        message.projected_error_m = proposal.projected_error_m
        message.translation_uncertainty_m = proposal.translation_uncertainty_m
        message.yaw_uncertainty_rad = proposal.yaw_uncertainty_rad
        message.condition_number = proposal.condition_number
        message.selector_status = getattr(
            result, 'selector_status', 'INSUFFICIENT_EVIDENCE')
        message.selector_score = getattr(result, 'selector_score', 0.0)
        message.selector_null_score = getattr(result, 'selector_null_score', 0.0)
        message.selector_runner_up_score = getattr(
            result, 'selector_runner_up_score', -math.inf)
        message.selector_runner_up_margin = getattr(
            result, 'selector_runner_up_margin', 0.0)
        message.selector_inlier_probabilities = list(getattr(
            result, 'selector_inlier_probabilities', []))
        return message

    def _request_source_for_confirmation(self, proposal):
        source_ids = list(getattr(
            proposal, 'evidence_source_keyframe_ids', []))
        if not source_ids:
            source_ids = [proposal.source_keyframe_id]
        for source_keyframe_id in source_ids:
            key = (source_keyframe_id, self.peer_robot_id)
            if key in self.pending_requests:
                self.counters['crop_request_duplicates_suppressed'] += 1
                continue
            self.pending_requests.add(key)
            self.counters['crop_requests_queued'] += 1
            request = LocalMapCropRequest()
            request.header = proposal.header
            request.requester_robot_id = self.robot_id
            request.source_robot_id = proposal.source_robot_id
            request.keyframe_id = source_keyframe_id
            request.descriptor_checksum = 0
            self.request_pub.publish(request)
            self.counters['crop_requests_sent'] += 1

    def publish_accepted_tf(self):
        if self.accepted is None or self.robot_id != min(self.robot_id, self.peer_robot_id):
            return
        now = self.get_clock().now().to_msg()
        identity = TransformStamped()
        identity.header.stamp = now
        identity.header.frame_id = self.shared_frame
        identity.child_frame_id = f'{self.robot_id}/local_world'
        identity.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(identity)
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.shared_frame
        transform.child_frame_id = f'{self.peer_robot_id}/local_world'
        # ``register_crops(source, target)`` returns the point transform that
        # maps source-crop coordinates into target-crop coordinates.  A ROS TF
        # with parent ``shared_map`` (the source/local frame) and child
        # ``peer_robot/local_world`` needs the inverse: the child pose
        # expressed in the parent frame.  Keep the protocol hypothesis in its
        # source->target convention and invert only at this TF boundary.
        registration = self.accepted.source_to_target
        yaw = math.atan2(
            2.0 * (registration.rotation.w * registration.rotation.z +
                   registration.rotation.x * registration.rotation.y),
            1.0 - 2.0 * (registration.rotation.y ** 2 +
                         registration.rotation.z ** 2))
        inverse_x, inverse_y, inverse_yaw = invert_se2(
            (registration.translation.x, registration.translation.y, yaw))
        transform.transform.translation.x = inverse_x
        transform.transform.translation.y = inverse_y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(inverse_yaw / 2.0)
        transform.transform.rotation.w = math.cos(inverse_yaw / 2.0)
        self.tf_broadcaster.sendTransform(transform)
        self.counters['tf_handoffs'] += 1

    def publish_local_map(self, force=False):
        if self.latest_map is None or self.accepted is None:
            return False
        now = self._ros_time_s()
        if not force:
            if now - self.last_export_ros_s < self.peer_map_publish_period_s:
                return False
            if self.latest_map_fingerprint == self.last_export_map_fingerprint:
                return False
        if not self.merge_handoff_logged:
            self.merge_handoff_logged = True
            self.counters['merge_handoff_started'] += 1
            self.get_logger().info(
                'UNKNOWN_POSE_MERGE_HANDOFF_START '
                f'confidence={self.accepted.final_confidence:.6f} '
                f'descriptor_similarity={self.accepted.descriptor_similarity:.6f} '
                f'geometric_inlier_ratio={self.accepted.geometric_inlier_ratio:.6f} '
                f'residual_m={self.accepted.registration_residual_m:.6f} '
                f'overlap_fraction={self.accepted.overlap_fraction:.6f} '
                f'source_keyframe={self.accepted.source_keyframe_id} '
                f'target_keyframe={self.accepted.target_keyframe_id} '
                f'robot={self.robot_id}')
        message = PeerMap()
        message.source_robot_id = self.robot_id
        message.revision = self.map_revision
        message.export_stamp = self.get_clock().now().to_msg()
        message.local_evidence_only = True
        message.occupancy_grid = self.latest_map
        self.peer_map_pub.publish(message)
        self.counters['peer_maps_published'] += 1
        self.last_export_ros_s = now
        self.last_export_map_fingerprint = self.latest_map_fingerprint
        return True

    def finalize(self):
        """Persist bounded diagnostics without affecting navigation behavior."""
        self._evidence_opportunity_deadline_wall = None
        self._publish_evidence_status(False)
        self._registration_shutdown = True
        if self._registration_future is not None:
            self._registration_future.cancel()
        if self._full_map_registration_future is not None:
            self._full_map_registration_future.cancel()
        # Never wait for a potentially expensive registration during ROS
        # teardown.  A running worker is intentionally abandoned; it owns
        # only immutable crop arrays and cannot publish or mutate frontend
        # state.  The campaign runner remains responsible for the process
        # group timeout and exact cleanup.
        self._registration_executor.shutdown(
            wait=False, cancel_futures=True)
        if not self.diagnostic_output:
            return
        path = Path(self.diagnostic_output)
        path.mkdir(parents=True, exist_ok=True)
        output = path / f'{self.robot_id}_unknown_pose_frontend.json'
        accepted_hypothesis = None
        if self.accepted is not None:
            accepted_hypothesis = {
                'source_robot_id': self.accepted.source_robot_id,
                'target_robot_id': self.accepted.target_robot_id,
                'source_keyframe_id': self.accepted.source_keyframe_id,
                'target_keyframe_id': self.accepted.target_keyframe_id,
                'transform_se2': [
                    float(self.accepted.source_to_target.translation.x),
                    float(self.accepted.source_to_target.translation.y),
                    float(self._yaw(self.accepted.source_to_target.rotation)),
                ],
                'covariance': [float(value) for value in self.accepted.covariance],
                'descriptor_similarity': float(
                    self.accepted.descriptor_similarity),
                'descriptor_margin': float(self.accepted.descriptor_margin),
                'temporal_consistency': float(
                    self.accepted.temporal_consistency),
                'geometric_inlier_ratio': float(
                    self.accepted.geometric_inlier_ratio),
                'registration_residual_m': float(
                    self.accepted.registration_residual_m),
                'occupied_free_agreement': float(
                    self.accepted.occupied_free_agreement),
                'overlap_fraction': float(self.accepted.overlap_fraction),
                'final_confidence': float(self.accepted.final_confidence),
                'evidence_set_hash': str(self.accepted.evidence_set_hash),
                'evidence_source_keyframe_ids': list(
                    self.accepted.evidence_source_keyframe_ids),
                'evidence_target_keyframe_ids': list(
                    self.accepted.evidence_target_keyframe_ids),
                'constraint_count': int(self.accepted.constraint_count),
                'consistent_constraint_count': int(
                    self.accepted.consistent_constraint_count),
                'spatial_baseline_m': float(self.accepted.spatial_baseline_m),
                'angular_spread_rad': float(self.accepted.angular_spread_rad),
                'median_registration_residual_m': float(
                    self.accepted.median_registration_residual_m),
                'p95_registration_residual_m': float(
                    self.accepted.p95_registration_residual_m),
                'projected_error_m': float(self.accepted.projected_error_m),
                'translation_uncertainty_m': float(
                    self.accepted.translation_uncertainty_m),
                'yaw_uncertainty_rad': float(
                    self.accepted.yaw_uncertainty_rad),
                'condition_number': float(self.accepted.condition_number),
                'status': str(self.accepted.status),
                'accepted': bool(self.accepted.accepted),
                'rejection_reason': str(self.accepted.rejection_reason),
                'accepted_ros_time_s': self.accepted_ros_time_s,
            }
        candidate_latency = None
        if self.first_candidate_wall is not None and self.accepted_wall is not None:
            candidate_latency = max(
                0.0, self.accepted_wall - self.first_candidate_wall)
        handoff_latency = None
        if self.first_map_wall is not None and self.accepted_wall is not None:
            handoff_latency = max(0.0, self.accepted_wall - self.first_map_wall)
        callback_timing = {}
        for name, stats in self.callback_stats.items():
            samples = sorted(stats['samples_ms'])
            p95 = samples[min(len(samples) - 1, int(0.95 * (len(samples) - 1)))] if samples else 0.0
            callback_timing[name] = {
                'count': stats['count'],
                'mean_ms': (stats['total_ms'] / stats['count']
                            if stats['count'] else 0.0),
                'p95_ms': p95,
                'max_ms': stats['max_ms'],
            }
        if self.consensus_diagnostics is not None:
            self.consensus_diagnostics.close()
        if self.physical_evidence_diagnostics is not None:
            self.physical_evidence_diagnostics.close()
        payload = {
            'robot_id': self.robot_id,
            'peer_robot_id': self.peer_robot_id,
            'map_revision': self.map_revision,
            'keyframes_retained': len(self.keyframes),
            'peer_descriptors_retained': len(self.peer_descriptors),
            'accepted': bool(self.accepted is not None),
            'accepted_ros_time_s': self.accepted_ros_time_s,
            'best_similarity': self.best_similarity,
            'best_margin': self.best_margin,
            'best_known_fraction': self.best_known_fraction,
            'accepted_confidence': (
                None if self.accepted is None else self.accepted.final_confidence),
            'accepted_descriptor_similarity': (
                None if self.accepted is None else self.accepted.descriptor_similarity),
            'accepted_geometric_inlier_ratio': (
                None if self.accepted is None else self.accepted.geometric_inlier_ratio),
            'accepted_registration_residual_m': (
                None if self.accepted is None else self.accepted.registration_residual_m),
            'accepted_overlap_fraction': (
                None if self.accepted is None else self.accepted.overlap_fraction),
            'accepted_hypothesis': accepted_hypothesis,
            'candidate_latency_wall_s': candidate_latency,
            'map_to_accept_latency_wall_s': handoff_latency,
            'descriptor_bytes': self.descriptor_bytes,
            'crop_cells_sent_max': self.crop_cells_sent,
            'crop_cells_received_max': self.crop_cells_received,
            'multi_constraint_attempts': self.counters[
                'multi_constraint_attempts'],
            'multi_constraint_rejections': self.counters[
                'multi_constraint_rejections'],
            'counters': self.counters,
            'diagnostics': {
                'descriptor_gate_rejection_reason_counts': dict(
                    self.gate_rejection_counts),
                'temporal_gate_rejection_reason_counts': dict(
                    self.temporal_gate_rejection_counts),
                'crop_response_rejection_reason_counts': dict(
                    self.crop_response_rejection_counts),
                'consensus_gate_rejection_reason_counts': dict(
                    self.consensus_gate_rejection_counts),
                'unique_descriptor_gate_survivors': len(
                    self.descriptor_gate_survivors),
                'unique_temporal_gate_survivors': len(
                    self.temporal_gate_survivors),
                'temporal_confirmation_clock': 'message_header_stamp_ros_clock',
                'configured_confirmation_window_s': (
                    self.confirmation_window_ns / 1.0e9),
                'effective_confirmation_window_s': (
                    self.effective_confirmation_window_ns / 1.0e9),
                'observed_own_descriptor_intervals_s': [
                    (right - left) / 1.0e9 for left, right in zip(
                        self.own_descriptor_stamps_ns,
                        list(self.own_descriptor_stamps_ns)[1:])],
                'observed_peer_descriptor_intervals_s': [
                    (right - left) / 1.0e9 for left, right in zip(
                        self.peer_descriptor_stamps_ns,
                        list(self.peer_descriptor_stamps_ns)[1:])],
                'callback_timing': callback_timing,
                'callback_started': self.callback_started,
                'callback_completed': self.callback_completed,
                'max_callback_inflight': self.max_callback_inflight,
                'max_executor_backlog_estimate': self.max_backlog_estimate,
                'registration_worker_queue_drops': int(
                    self._registration_queue_drops),
                'registration_worker_queue_capacity': int(
                    self._registration_pending_contexts.maxlen),
                'registration_worker_backpressure_depth': int(
                    self._registration_backpressure_depth),
                'registration_worker_backpressure_events': int(
                    self._registration_backpressure_events),
                'registration_worker_queue_enqueues': int(
                    self._registration_queue_enqueues),
                'registration_worker_queue_dequeues': int(
                    self._registration_queue_dequeues),
                'registration_worker_queue_max_depth': int(
                    self._registration_queue_max_depth),
                'registration_worker_queue_depth_at_finalize': int(
                    len(self._registration_pending_contexts)),
                'registration_worker_inflight': bool(
                    self._registration_future is not None and
                    not self._registration_future.done()),
                'cpu_samples': self.cpu_samples,
                'protocol_events': self.diagnostic_events,
                'protocol_event_drops': self.diagnostic_event_drops,
                'protocol_lifecycle_events': self.protocol_lifecycle_events,
                'protocol_lifecycle_event_drops': (
                    self.protocol_lifecycle_event_drops),
                'consensus_diagnostics_artifact': (
                    None if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.path.name),
                'consensus_diagnostic_records_written': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.records_written),
                'consensus_diagnostic_drops': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.dropped_records),
                'consensus_diagnostic_write_failures': (
                    0 if self.consensus_diagnostics is None else
                    self.consensus_diagnostics.write_failures),
                'physical_evidence_diagnostics_artifact': (
                    None if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.path.name),
                'physical_evidence_diagnostic_records_written': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.records_written),
                'physical_evidence_diagnostic_drops': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.dropped_records),
                'physical_evidence_diagnostic_write_failures': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.write_failures),
                'physical_evidence_record_counts': dict(
                    self._physical_diagnostic_summary),
                'physical_evidence_records_suppressed': dict(
                    self._physical_diagnostic_suppressed),
                'physical_evidence_bytes_written': (
                    0 if self.physical_evidence_diagnostics is None else
                    self.physical_evidence_diagnostics.bytes_written),
                'verification_batch_state': {
                    'acquisition_batch_id': int(
                        self.verification_batches.batch_id),
                    'batch_attempts': int(
                        self.verification_batches.batch_attempts),
                    'max_batches': int(
                        self.verification_batches.max_batches),
                    'budget': int(self.verification_batches.budget),
                    'waiting_for_novelty': bool(
                        self.verification_batches.waiting_for_novelty),
                    'lifetime_expired': bool(
                        self.verification_batches.lifetime_expired),
                    'completed': bool(self.verification_batches.completed),
                    'lifetime_s': float(
                        self.verification_batches.lifetime_s),
                    'novelty_spacing_m': float(
                        self.verification_batches.novelty_spacing_m),
                },
                'pending_candidate_pool_size': len(
                    self.pending_candidate_pairs),
                'finalized_wall_monotonic_s': time.monotonic(),
            },
        }
        output.write_text(json.dumps(payload, indent=2, sort_keys=True) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UnknownPoseFrontend()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
