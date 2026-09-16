"""Strictly passive structured observer for two-robot exploration experiments."""
from bisect import bisect_left, bisect_right
import csv, hashlib, json, math, os, re, signal, socket, statistics, subprocess, sys, threading, time, uuid
from collections import Counter, deque
from dataclasses import asdict
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from rcl_interfaces.msg import Log
from rclpy.duration import Duration
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener
from my_epuck_interfaces.msg import (
    DistributedExplorationEvent,
    DistributedExplorationStatus,
    ExplorationClaim,
    ExplorationEvent,
    ExplorationFailure,
    ExplorationStatus,
    FrontierCandidateArray,
    LocalMapDescriptor,
    PairDecision,
    PeerMap,
    TaskBidArray,
    TaskSnapshot,
)
from .experiment_metrics import CoverageAttribution, Grid, LocalTrajectory, MotionDetector, MotionSample, TrajectoryOverlap, WarningDeduplicator, allocate_run_directory, atomic_json, duplicate_goal, equivalent_frontiers, finite, known_counts, known_world_cells, rosout_diagnostic_category, utc_now, warning_category
from .forensic_evidence import ForensicEvidenceWriter
from .deferred_protocol import pair_decision_outcome
from .deferred_navigation import replay_navigation_evidence_from_bag
from .passive_rosbag import (
    export_index_with_bounded_retry,
    load_offloaded_timing,
    offloaded_sensor_topics,
    semantic_export_complete,
    start_recorder,
    stop_recorder,
)

SCHEMA='1.1.0'; STATES={0:'UNKNOWN',1:'PROPOSING',2:'NAVIGATING',3:'SUCCEEDED',4:'FAILED',5:'RELEASED',6:'CANCELED'}; STATUS_STATES={0:'STARTING',1:'ACTIVE',2:'NAVIGATING',3:'NO_ELIGIBLE_CANDIDATES',4:'COMPLETE',5:'STOPPED',6:'ERROR'}
TIME_FIELDS=['run_id','wall_time_utc','ros_time_sec','ros_time_nanosec','elapsed_s','wall_elapsed_s','event_sequence']
TELEMETRY=TIME_FIELDS+['robot_id','pose_x','pose_y','pose_yaw','linear_speed_mps','angular_speed_radps','commanded_linear_mps','commanded_angular_radps','cmd_vel_received','cmd_vel_age_s','cmd_vel_source','distance_travelled_m','claim_state','claim_id','frontier_id','goal_x','goal_y','goal_yaw','navigation_active','distance_remaining_m','recoveries','candidate_count','local_known_cells','shared_known_cells','local_costmap_obstacles','global_costmap_known','global_costmap_obstacles','odom_age_s','scan_age_s','map_age_s','shared_map_age_s','claim_age_s','feedback_age_s']
COVERAGE=TIME_FIELDS+['robot1_local_known','robot2_local_known','robot1_shared_known','robot2_shared_known','shared_free_cells','shared_occupied_cells','shared_unknown_cells','known_area_m2','coverage_gain_cells','coverage_gain_since_start_cells','unique_first_seen_robot1_cells','unique_first_seen_robot2_cells','later_duplicated_by_robot1_cells','later_duplicated_by_robot2_cells','simultaneously_observed_cells','total_known_union_cells','duplicated_known_fraction','shared_maps_equivalent']
HEALTH=TIME_FIELDS+['robot_id','topic_name','topic_rate_hz','topic_age_s','expected_min_rate_hz','stale']
GOAL_LEDGER_EVENTS={
    'CLAIM_PROPOSED', 'CLAIM_CONFLICT_DETECTED', 'EQUIVALENT_FRONTIER_DUPLICATE',
    'ARBITRATION_WON', 'ARBITRATION_LOST', 'NAV_GOAL_SENT',
    'NAV_GOAL_ACCEPTED', 'NAV_GOAL_REJECTED', 'NAVIGATION_SUCCEEDED',
    'NAVIGATION_FAILED', 'NAVIGATION_CANCELED', 'NAVIGATION_CANCELLED',
    'NAVIGATION_TIMEOUT', 'ROUND_INVALIDATED', 'DISTRIBUTED_TASK_FAILURE',
    'DISTRIBUTED_PAIR_DECISION', 'DISTRIBUTED_STATUS',
    'TRAFFIC_WAITING', 'TRAFFIC_RELEASED_FRESH_REALLOCATION',
    'MISSION_COMPLETE', 'MISSION_ABORTED',
}


def is_shutdown_conversion_error(error, shutdown_requested, context_valid):
    """Recognize only the known queued-take teardown signature."""
    return (shutdown_requested and not context_valid
            and isinstance(error, RuntimeError)
            and str(error).startswith('Unable to convert call argument'))

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def yaw_from_row(rotation_z, rotation_w):
    """Return planar yaw from raw CSV quaternion components."""
    return math.atan2(2.0 * float(rotation_w) * float(rotation_z),
                     1.0 - 2.0 * float(rotation_z) * float(rotation_z))
def as_grid(m): return Grid(m.info.width,m.info.height,m.info.resolution,m.info.origin.position.x,m.info.origin.position.y,yaw(m.info.origin.orientation),np.asarray(m.data,dtype=np.int8))
def stamp(m):
    s=getattr(getattr(m,'header',None),'stamp',None); return (int(s.sec),int(s.nanosec)) if s else (0,0)
def default_run_id(): return time.strftime('%Y-%m-%dT%H%M%SZ',time.gmtime())+'_'+uuid.uuid4().hex[:4]


def _wrap_planar_yaw(value):
    """Wrap a planar yaw to [-pi, pi)."""
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _compose_planar(first, second):
    """Compose ``T_A_B * T_B_C = T_A_C`` in the project convention."""
    tx, ty, heading = (float(value) for value in first)
    sx, sy, sheading = (float(value) for value in second)
    cosine, sine = math.cos(heading), math.sin(heading)
    return (
        tx + cosine * sx - sine * sy,
        ty + sine * sx + cosine * sy,
        _wrap_planar_yaw(heading + sheading),
    )


def _invert_planar(transform):
    """Invert a planar transform mapping source coordinates to target."""
    tx, ty, heading = (float(value) for value in transform)
    cosine, sine = math.cos(heading), math.sin(heading)
    return (
        -cosine * tx - sine * ty,
        sine * tx - cosine * ty,
        _wrap_planar_yaw(-heading),
    )


def _interpolate_planar(first, second, fraction):
    """Interpolate two timestamped planar transforms without yaw jumps."""
    fraction = min(1.0, max(0.0, float(fraction)))
    first_yaw = float(first[2])
    delta = _wrap_planar_yaw(float(second[2]) - first_yaw)
    return (
        float(first[0]) + fraction * (float(second[0]) - float(first[0])),
        float(first[1]) + fraction * (float(second[1]) - float(first[1])),
        _wrap_planar_yaw(first_yaw + fraction * delta),
    )


def webots_controller_host():
    """Resolve the Webots TCP host using the same WSL network contract.

    The passive forensic Supervisor is an external Webots controller.  It
    must use the same endpoint family as the robot controllers; hard-coding
    loopback leaves Webots waiting for the diagnostic controller in WSL NAT
    mode and consequently prevents /clock from advancing.
    """
    mode = os.environ.get('MY_EPUCK_WEBOTS_NETWORK_MODE', '').strip().lower()
    if mode in ('mirrored', 'loopback'):
        return '127.0.0.1'
    if mode in ('nat', 'subnet', 'wsl_nat'):
        try:
            result = subprocess.run(
                ['ip', 'route', 'show', 'default'], check=True,
                capture_output=True, text=True, timeout=1.0)
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError('Unable to resolve the WSL NAT gateway') from exc
        for line in result.stdout.splitlines():
            fields = line.split()
            if fields and fields[0] == 'default' and 'via' in fields:
                return fields[fields.index('via') + 1]
        raise RuntimeError('No default-route gateway found for Webots')
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '').strip().lower()
    discovery = os.environ.get(
        'ROS_AUTOMATIC_DISCOVERY_RANGE', '').strip().upper()
    if localhost_only in ('1', 'true', 'yes') or discovery != 'SUBNET':
        return '127.0.0.1'
    return webots_controller_host_for_nat()


def webots_controller_host_for_nat():
    """Return the default-route gateway for automatic subnet mode."""
    try:
        result = subprocess.run(
            ['ip', 'route', 'show', 'default'], check=True,
            capture_output=True, text=True, timeout=1.0)
    except (OSError, subprocess.SubprocessError) as exc:
        raise RuntimeError('Unable to resolve the Webots NAT gateway') from exc
    for line in result.stdout.splitlines():
        fields = line.split()
        if fields and fields[0] == 'default' and 'via' in fields:
            return fields[fields.index('via') + 1]
    raise RuntimeError('No default-route gateway found for Webots')


def supervisor_robot_arguments(robot_ids):
    """Return Supervisor robot arguments for the configured active robots."""
    arguments = []
    for robot in robot_ids:
        arguments.extend(['--robot-def', str(robot)])
    return arguments


class SupervisorTimestampIndex:
    """Stable nearest-time index for dense Supervisor trajectory rows."""

    def __init__(self, rows):
        # ``sorted`` is stable, preserving the legacy first-row tie behavior
        # for duplicate timestamps from unsorted input.
        self.values = tuple(sorted(rows, key=lambda item: item[0]))
        self.timestamps = tuple(item[0] for item in self.values)

    def nearest(self, query_ros):
        if not self.values:
            return None
        query_ros = float(query_ros)
        insertion = bisect_left(self.timestamps, query_ros)
        if insertion == 0:
            selected_timestamp = self.timestamps[0]
        elif insertion == len(self.timestamps):
            selected_timestamp = self.timestamps[-1]
        else:
            left_timestamp = self.timestamps[insertion - 1]
            right_timestamp = self.timestamps[insertion]
            if (abs(left_timestamp - query_ros) <=
                    abs(right_timestamp - query_ros)):
                selected_timestamp = left_timestamp
            else:
                selected_timestamp = right_timestamp
        # Select the first row at the chosen timestamp.  This preserves the
        # stable ``min`` result when multiple rows share that timestamp.
        first_at_timestamp = bisect_left(
            self.timestamps, selected_timestamp)
        return self.values[first_at_timestamp]

    def first_after(self, query_ros):
        """Return the legacy interpolation right-hand row index."""
        return bisect_right(self.timestamps, float(query_ros))


class RollingTimestampIndex:
    """Bounded, stable timestamp index for live odometry joins.

    Odometry arrives in timestamp order in normal operation, but retaining
    stable sorted insertion keeps the legacy behavior for delayed or
    out-of-order samples as well.  Queries return immutable snapshots so the
    callback and synchronized forensic timer can safely overlap.
    """

    def __init__(self, maxlen):
        self.maxlen = int(maxlen)
        self._entries = []
        self._arrival = deque()
        self._sequence = 0
        self._lock = threading.RLock()

    def append(self, timestamp, value):
        timestamp = float(timestamp)
        with self._lock:
            sequence = self._sequence
            self._sequence += 1
            entry = (timestamp, sequence, value)
            position = bisect_right(
                self._entries, (timestamp, sequence))
            self._entries.insert(position, entry)
            self._arrival.append((timestamp, sequence))
            if len(self._arrival) > self.maxlen:
                old_timestamp, old_sequence = self._arrival.popleft()
                old_position = bisect_left(
                    self._entries, (old_timestamp, old_sequence))
                del self._entries[old_position]

    def snapshot(self):
        with self._lock:
            return (
                tuple(entry[0] for entry in self._entries),
                tuple((entry[0], entry[2]) for entry in self._entries),
            )

    def lookup_bounds(self, query_timestamp):
        """Return the exact legacy interpolation bracket without copying it.

        The synchronized-map callback only needs the first row, the last row,
        or the two rows around a query.  ``snapshot()`` remains available for
        callers that need the complete historical view, but copying every
        odometry row for every forensic sample made that path unnecessarily
        expensive.  Entries are still searched in the same stable sorted
        order, so duplicate timestamps select the same rightmost row as
        ``bisect_right`` over the legacy timestamp tuple.
        """
        query_timestamp = float(query_timestamp)
        with self._lock:
            if not self._entries:
                return None
            first = self._entries[0]
            if query_timestamp < first[0]:
                return 'before', first[0], first[2]
            right_index = bisect_right(
                self._entries, (query_timestamp, math.inf))
            if right_index == len(self._entries):
                last = self._entries[-1]
                return 'after', last[0], last[2]
            left = self._entries[right_index - 1]
            right = self._entries[right_index]
            return 'between', left[0], left[2], right[0], right[2]


class RawTFSeriesIndex:
    """Bounded arrival history with a lazy stable timestamp index.

    Raw TF messages are appended at high frequency.  Sorting on every
    synchronized-map query is unnecessarily expensive, while inserting into
    a sorted structure on every message makes the hot subscription path
    expensive.  This index keeps ingestion O(1) and builds one stable sorted
    snapshot only when a query first needs it after an append.
    """

    def __init__(self, maxlen):
        self._arrival = deque(maxlen=int(maxlen))
        self._sorted_values = None
        self._timestamps = None
        self._monotonic = True
        self._last_timestamp = None
        self._lock = threading.RLock()

    def append(self, timestamp, value):
        with self._lock:
            timestamp = float(timestamp)
            if (self._last_timestamp is not None and
                    timestamp < self._last_timestamp):
                self._monotonic = False
            self._last_timestamp = timestamp
            self._arrival.append((timestamp, value))
            self._sorted_values = None
            self._timestamps = None

    def lookup_bounds(self, query_timestamp):
        """Return an exact legacy bracket without rebuilding a raw-TF graph.

        Webots/ROS TF samples are normally timestamp-monotonic per edge.  In
        that common case, the synchronized query is near the newest sample,
        so walking the bounded arrival deque backwards is cheaper than
        sorting/copying the whole series for each graph edge.  Any observed
        out-of-order sample permanently selects the existing stable sorted
        fallback, preserving the old duplicate/tie behavior exactly.
        """
        query_timestamp = float(query_timestamp)
        with self._lock:
            if not self._arrival:
                return None
            if not self._monotonic:
                timestamps, values = self.sorted_snapshot()
                if query_timestamp < timestamps[0]:
                    return 'before', values[0][0], values[0][1]
                right_index = bisect_right(timestamps, query_timestamp)
                if right_index == len(values):
                    return 'after', values[-1][0], values[-1][1]
                left = values[right_index - 1]
                right = values[right_index]
                return ('between', left[0], left[1], right[0], right[1])

            right = None
            for item in reversed(self._arrival):
                if item[0] > query_timestamp:
                    right = item
                    continue
                if right is None:
                    return 'after', item[0], item[1]
                return ('between', item[0], item[1], right[0], right[1])
            first = self._arrival[0]
            return 'before', first[0], first[1]

    def sorted_snapshot(self):
        with self._lock:
            if self._sorted_values is None:
                # Python's sort is stable.  The arrival order therefore
                # preserves the legacy tie behavior for duplicate timestamps.
                values = list(self._arrival)
                values.sort(key=lambda item: item[0])
                self._sorted_values = tuple(values)
                self._timestamps = tuple(item[0] for item in values)
            return self._timestamps, self._sorted_values

class CooperativeExperimentLogger(Node):
    def __init__(self, **node_kwargs):
        super().__init__('cooperative_experiment_logger', **node_kwargs)
        defaults={'run_id':'','output_root':'/home/arash/webots_ws/results','launch_file':'two_robots_observed_single_goal_launch.py','experiment_condition':'','seed_provenance_json':'{}','robot_ids':['robot1','robot2'],'global_frame':'shared_map','telemetry_rate_hz':1.,'coverage_rate_hz':.5,'topic_health_rate_hz':.2,'console_summary_period_s':5.,'warning_summary_period_s':30.,'progress_window_s':10.,'minimum_distance_remaining_improvement_m':.03,'minimum_robot_displacement_m':.02,'stuck_window_s':6.,'commanded_linear_threshold_mps':.02,'commanded_angular_threshold_radps':.15,'cmd_vel_zero_linear_epsilon_mps':.001,'cmd_vel_zero_angular_epsilon_radps':.001,'cmd_vel_no_command_timeout_s':1.5,'stuck_displacement_threshold_m':.015,'oscillation_window_s':10.,'angular_sign_change_threshold':4,'oscillation_displacement_threshold_m':.04,'simultaneous_coverage_window_s':2.,'trajectory_bin_size_m':.05,'initial_overlap_exclusion_radius_m':.15,'duplicate_goal_tolerance_m':.15,'shared_map_divergence_grace_s':3.,'enable_rosout_collection':True,'enable_coverage_attribution':True,'enable_trajectory_overlap':True,'enable_console_status':True,'odom_stale_s':2.,'scan_stale_s':2.,'map_stale_s':5.,'shared_map_stale_s':5.,'candidate_stale_s':5.,'claim_stale_s':4.,'status_stale_s':4.,'feedback_stale_s':3.,'costmap_stale_s':5.,'enable_forensic_capture':False,'enable_local_map_capture':True,'diagnostic_frontier_capture':False,'diagnostic_footprint_radius_m':.08,'forensic_snapshot_interval_s':5.,'forensic_sync_rate_hz':50.,'forensic_ground_truth_sample_period_s':0.02,'enable_contact_capture':False,'contact_sampling_period_ms':20,'webots_port':23000,'terminal_small_frontier_length_m':0.20}
        defaults.update({
            # Physical Robot 2 uses base_link; the verified simulation
            # observer default remains robotX/base_footprint.
            'robot_base_frame': '',
            'world_profile': 'small',
            'source_world_path': '',
            'installed_world_path': '',
            'world_dimensions': [0.0, 0.0],
            'robot_start_poses_json': '{}',
            # Direct/unit and physical deployments have no Webots world.
            # Simulation launch always overrides this with WORLD_DERIVED.
            'known_relative_transform': [-0.3, 0.0, -math.pi],
            'transform_source': 'EXPLICIT_PHYSICAL',
            'slam_resolution': 0.01,
            'fusion_resolution': 0.01,
            'global_costmap_resolution': 0.005,
            'local_costmap_resolution': 0.005,
            'lidar_maximum_range': 12.0,
            'initial_configuration_json': '{}',
            'world_sha256': '',
            'coverage_attribution_resolution': 0.01,
            # ``auto`` preserves the established contract: A uses its local
            # map and two-robot cooperative runs use the shared map.  The
            # independent two-robot baseline explicitly selects the passive
            # local-map union path.
            'coverage_source': 'auto',
            'enable_passive_rosbag': False,
            'enable_scientific_raw_capture': False,
        })
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}; self.robots=list(self.p['robot_ids']); self.start=time.monotonic(); self.start_ros=self.get_clock().now().nanoseconds*1e-9; self.start_utc=utc_now(); self.sequence=0; self.finalized=False; self._finalizing=False; self._closed=False; self.write_failures=0; self.dropped_samples=0
        if len(self.p['known_relative_transform']) != 3:
            raise ValueError('known_relative_transform must be explicit (physical) or world-derived')
        self._state_lock=threading.RLock(); self._io_lock=threading.RLock(); self._lifecycle_lock=threading.Lock(); self.internal_errors=Counter(); self._reporting_internal_error=False; self._observer_timers=[]
        self._map_cache={}; self._transformed_cache={}; self._last_attributed={}; self._cpu_samples=[]; self._rss_samples=[]; self._cpu_previous=None
        self._callback_timing_enabled = os.environ.get(
            'MY_EPUCK_CALLBACK_TIMING', '',
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self._callback_timing = {}
        self._high_rate_profile_enabled = os.environ.get(
            'MY_EPUCK_ODOM_TF_PROFILE', '',
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self._high_rate_profile = {}
        self._sync_map_profile_enabled = os.environ.get(
            'MY_EPUCK_SYNC_MAP_PROFILE', '',
        ).strip().lower() in ('1', 'true', 'yes', 'on')
        self._sync_map_profile = {}
        # Synchronized map frames are passive derived evidence.  When the
        # runner enables the offline path, retain the exact request metadata
        # live and perform the existing TF/odom reconstruction after the
        # scientific horizon.  Raw TF, odometry, maps, and all runtime event
        # streams remain captured through their existing subscriptions.
        self._defer_synchronized_map_frames = os.environ.get(
            'MY_EPUCK_DEFER_SYNC_MAP_FRAMES', '').strip().lower() in (
                '1', 'true', 'yes', 'on')
        self._deferred_sync_map_requests = []
        self.run_id,self.directory=allocate_run_directory(Path(self.p['output_root']),self.p['run_id'] or default_run_id())
        passive_bag_enabled = self.p.get('enable_passive_rosbag', False)
        if isinstance(passive_bag_enabled, str):
            passive_bag_enabled = passive_bag_enabled.lower() == 'true'
        self.passive_bag_enabled = bool(passive_bag_enabled)
        # The existing C passive-bag path owns the high-rate sensor evidence.
        # No live control component consumes these observer-only messages.
        self.passive_sensor_offload_enabled = self.passive_bag_enabled
        self._defer_health_history = bool(
            self.passive_bag_enabled and
            self.p.get('enable_scientific_raw_capture', False))
        self._deferred_health_rows = []
        self.passive_bag_process = None
        self.passive_bag_log = None
        self.passive_bag_command = None
        self.passive_bag_log_path = None
        self.passive_bag_export = None
        self._artifact_finalization = {
            'complete': False,
            'status': 'NOT_FINALIZED',
            'required': [],
            'missing': [],
        }
        self.robot_counts={r:Counter() for r in self.robots}; self.cycle_durations={r:[] for r in self.robots}; self.cycle_starts={}; self.region_attempts={r:Counter() for r in self.robots}; self.exhausted_since={r:None for r in self.robots}; self.exhausted_duration={r:0. for r in self.robots}; self.mission_completion_time=None; self.mission_terminal_reason=''; self.statuses={}
        self.files=[]; self.events=open(self.directory/'events.jsonl','a',encoding='utf-8',buffering=1); self.goal_decisions=open(self.directory/'goal_decision_ledger.jsonl','a',encoding='utf-8',buffering=1); self.files.append(self.goal_decisions); self.nav2_diagnostics=open(self.directory/'nav2_diagnostics.jsonl','a',encoding='utf-8',buffering=1); self.files.append(self.nav2_diagnostics); self.rosout_receipt_file=None; self._rosout_receipt_sequence=0; self.map_receipt_file=None; self._map_receipt_sequence=0; self.coverage_request_file=None; self.coverage_stream=None; self.frontier_regions_file=None; self.nav2_diagnostic_count=0; self._diagnostic_last={}; self.action_goal_states={}; self.follow_path_goal_lifecycle=set(); self.follow_path_terminal_pending=Counter(); self.warns=WarningDeduplicator(); self._deferred_warning_records=None; self._navigation_action_replay_failed=False; self.counts=Counter(); self.last={}; self.windows={}; self.stale={}; self.latest={r:{} for r in self.robots}; self.claims={}; self.distributed_last={}; self.frontier_metadata={r:{} for r in self.robots}; self.frontier_query_pending={}; self.frontier_query_forensics=bool(self.p.get('diagnostic_frontier_capture',False)); self.frontier_query_forensic_file=None; self.frontier_query_tf_file=None; self.frontier_query_crops={}
        if bool(self.p.get('enable_scientific_raw_capture', False)):
            self.rosout_receipt_file=open(
                self.directory / 'rosout_receipts.jsonl', 'a',
                encoding='utf-8', buffering=1)
            self.files.append(self.rosout_receipt_file)
            self.map_receipt_file=open(
                self.directory / 'map_receipts.jsonl', 'a', encoding='utf-8',
                buffering=1)
            self.files.append(self.map_receipt_file)
        if self.frontier_query_forensics:
            forensic_root=self.directory/'nav2_frontier_rejection_forensic'
            for name in ('local_costmap_crops','global_costmap_crops','shared_map_crops'):
                (forensic_root/name).mkdir(parents=True,exist_ok=True)
            self.frontier_query_crops={name: forensic_root/name for name in ('local_costmap_crops','global_costmap_crops','shared_map_crops')}
            self.frontier_query_forensic_file=(forensic_root/'frontier_query_diagnostics.jsonl').open('a',encoding='utf-8',buffering=1)
            self.frontier_query_tf_file=(forensic_root/'tf_query_provenance.jsonl').open('a',encoding='utf-8',buffering=1)
            self.files.extend([self.frontier_query_forensic_file,self.frontier_query_tf_file])
        # Protocol counters deliberately separate replicated publications from
        # unique decisions and local navigation outcomes.
        self.unique_agreed_rounds=set(); self.unique_agreed_decisions=set(); self.frontier_query_counts={}; self.frontier_query_last_time={}
        self.round_outcomes=Counter(); self.planner_query_counts=Counter()
        self.planner_query_duration_s=Counter()
        self.agreement_publications=0; self.dispatch_attempts=0; self.goals_terminal=0; self.goal_accounting=[]; self._accepted_before_send=set()
        self.detectors={r:MotionDetector(self.p['progress_window_s'],self.p['minimum_distance_remaining_improvement_m'],self.p['minimum_robot_displacement_m'],self.p['stuck_window_s'],self.p['commanded_linear_threshold_mps'],self.p['commanded_angular_threshold_radps'],self.p['stuck_displacement_threshold_m'],self.p['oscillation_window_s'],int(self.p['angular_sign_change_threshold']),self.p['oscillation_displacement_threshold_m']) for r in self.robots}
        self.attribution=CoverageAttribution(self.p['simultaneous_coverage_window_s']); self.trajectory=TrajectoryOverlap(self.p['trajectory_bin_size_m'],self.p['initial_overlap_exclusion_radius_m']); self.local_trajectory=LocalTrajectory(self.p['trajectory_bin_size_m'],self.p['initial_overlap_exclusion_radius_m']); self.initial_known=None; self.previous_known=None; self.writers={}
        configured_coverage_source = str(
            self.p.get('coverage_source', 'auto')).strip()
        if configured_coverage_source == 'auto':
            # Two-robot campaigns retain the established shared-map coverage
            # contract.  A is a true single-robot run, so its only valid
            # coverage source is robot1's local SLAM OccupancyGrid.
            self.coverage_source = (
                'local_map' if len(self.robots) == 1 else 'shared_map')
        elif configured_coverage_source in (
                'local_map', 'local_map_union', 'shared_map'):
            self.coverage_source = configured_coverage_source
        else:
            raise ValueError(
                'coverage_source must be auto, local_map, '
                'local_map_union, or shared_map')
        forensic_enabled = self.p['enable_forensic_capture']
        if isinstance(forensic_enabled, str):
            forensic_enabled = forensic_enabled.lower() == 'true'
        contact_enabled = self.p.get('enable_contact_capture', False)
        if isinstance(contact_enabled, str):
            contact_enabled = contact_enabled.lower() == 'true'
        self.contact_capture = bool(contact_enabled)
        local_capture_enabled = self.p.get('enable_local_map_capture', True)
        if isinstance(local_capture_enabled, str):
            local_capture_enabled = local_capture_enabled.lower() == 'true'
        self.local_map_capture = bool(local_capture_enabled)
        # Local occupancy/odom/TF capture is useful even when the optional
        # Supervisor observer is disabled.  Keeping these concerns separate
        # ensures a no-handoff run still has honest local-map artifacts without
        # starting a ground-truth process or changing the runtime graph.
        try:
            initial_configuration = json.loads(
                self.p['initial_configuration_json'])
            scan_matching_enabled = bool(
                initial_configuration.get('use_scan_matching', False) or
                initial_configuration.get(
                    'slam_runtime_parameters', {}).get(
                        'use_scan_matching', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            scan_matching_enabled = False
        self.scan_matching_enabled = scan_matching_enabled
        self.forensic = (ForensicEvidenceWriter(
            self.directory, self.robots, self.p['forensic_snapshot_interval_s'],
            scan_matching_enabled=scan_matching_enabled)
            if (forensic_enabled or self.local_map_capture) else None)
        if self.forensic is not None:
            # Preserve only the scalar distance state needed by live
            # telemetry/cycle rows.  Full bins/revisit state is reconstructed
            # from the authoritative forensic odometry at finalization.
            from .experiment_metrics import LiveDistanceAccumulator
            self.local_trajectory = LiveDistanceAccumulator()
        self.forensic_supervisor_enabled = bool(forensic_enabled or
                                                self.contact_capture)
        self.forensic_sync_enabled = bool(self.forensic_supervisor_enabled)
        self.latest_evidence = {robot: None for robot in self.robots}
        self.ground_truth_process = None
        self.ground_truth_log = None
        self.ground_truth_ready_file = None
        self.ground_truth_exit_reported = False
        if self.forensic_supervisor_enabled:
            self.start_forensic_ground_truth()
        self.stack_ready=False; self.divergence_since=None; self.divergence_reported=False; self.last_progress={}; self.tf_state={}; self.shared_map_seen=set()
        # Bounded scan-pipeline evidence.  These samples are passive and are
        # written once at shutdown so a campaign records what Slam Toolbox
        # actually received rather than only the configured value.
        self.scan_pipeline = {
            r: {
                'scan_d500_fixed_stamps': deque(maxlen=4096),
                'scan_d500_nav_stamps': deque(maxlen=4096),
                'scan_d500_nav_ages_s': deque(maxlen=4096),
                'map_stamps': deque(maxlen=4096),
                'scan_correction_records': 0,
            }
            for r in self.robots}
        self.scan_pipeline_warning_emitted = False
        self.tf_buffer=Buffer()
        # The passive raw-TF subscriptions below also feed this buffer.  A
        # separate TransformListener would subscribe to /tf and /tf_static a
        # second time, duplicating deserialization and callback dispatch while
        # adding no evidence.  Keep one subscription per TF channel and retain
        # the same tf2 buffer semantics in the combined callbacks.
        self.tf_listener = None
        # tf2's graph lookup is the preferred path.  Keep a bounded passive
        # copy of the actual /tf streams as a fallback for exact-time
        # forensic joins when a composed lookup reports an unconnected tree or
        # an extrapolation error.  These samples are never published and are
        # never visible to the estimator.
        self._direct_tf_samples = {}
        self._direct_tf_static = {}
        self._direct_tf_sample_limit = 4096
        self._direct_tf_graph_lock = threading.RLock()
        self._direct_tf_graph_keys = set()
        self._direct_tf_adjacency = {}
        self._direct_tf_adjacency_snapshot = None
        # Only these dynamic edges are queried by the logger's live passive
        # evidence paths.  Raw TF remains complete in the direct index and
        # forensic CSV; unrelated wheel/sensor edges do not need tf2 buffer
        # insertion for any live logger consumer.
        self._tf_buffer_live_dynamic_edges = {
            (f'{robot}/map', f'{robot}/odom') for robot in self.robots}
        self._tf_buffer_live_dynamic_edges.update(
            (f'{robot}/odom', self.robot_base_frame(robot))
            for robot in self.robots)
        self._odom_samples = {
            robot: RollingTimestampIndex(self._direct_tf_sample_limit)
            for robot in self.robots}
        for r in self.robots: self.writers[r]=self.csv_file(f'{r}_timeseries.csv',TELEMETRY)
        self.coverage=self.csv_file('coverage.csv',COVERAGE); self.coverage_stream=self.files[-1]; self.health=self.csv_file('topic_health.csv',HEALTH)
        if bool(self.p.get('enable_scientific_raw_capture', False)):
            self.coverage_request_file=open(
                self.directory / 'coverage_requests.jsonl', 'a',
                encoding='utf-8', buffering=1)
            self.files.append(self.coverage_request_file)
        (self.directory/'README.txt').write_text('Passive data; schema and formulas: my_epuck_project/docs/cooperative_experiment_logging.md\n',encoding='utf-8')
        self.write_manifest(False,'running'); self.event('RUN_START','experiment run started',console=True)
        if self.passive_bag_enabled:
            try:
                (self.passive_bag_process, self.passive_bag_log,
                 self.passive_bag_command,
                 self.passive_bag_log_path) = start_recorder(
                    self.directory / 'passive_rosbag', self.robots,
                    include_offloaded=self.passive_sensor_offload_enabled,
                    include_scientific_raw=bool(
                        self.p.get('enable_scientific_raw_capture', False)))
                self.event(
                    'PASSIVE_ROSBAG_STARTED',
                    'standard rosbag2 recorder started for observer-only topics',
                    command=self.passive_bag_command,
                    allow_during_shutdown=True,
                )
            except (OSError, RuntimeError) as exc:
                self.event('PASSIVE_ROSBAG_START_FAILED', str(exc),
                           severity='ERROR', allow_during_shutdown=True)
        self.subscribe()
        self._observer_timers.append(self.create_timer(1/self.p['telemetry_rate_hz'],lambda:self.safe_call('telemetry',self.sample_telemetry)))
        self._observer_timers.append(self.create_timer(1/self.p['coverage_rate_hz'],lambda:self.safe_call('coverage',self.sample_coverage)))
        self._observer_timers.append(self.create_timer(1/self.p['topic_health_rate_hz'],lambda:self.safe_call('topic_health',self.sample_health)))
        self._observer_timers.append(self.create_timer(self.p['console_summary_period_s'],lambda:self.safe_call('console_status',self.console)))
        self._observer_timers.append(self.create_timer(1.,lambda:self.safe_call('process_resources',self.sample_process_resources)))
        self._observer_timers.append(self.create_timer(5.,lambda:self.safe_call('file_flush',self.flush)))
        if self.ground_truth_process is not None:
            self._observer_timers.append(self.create_timer(
                1., lambda: self.safe_call(
                    'forensic_supervisor_monitor',
                    self.monitor_forensic_ground_truth)))
        if self.forensic is not None:
            self._observer_timers.append(self.create_timer(
                float(self.p['forensic_snapshot_interval_s']),
                lambda: self.safe_call('forensic_snapshot',
                                       self.forensic_snapshot)))
        if self.forensic is not None and self.forensic_sync_enabled:
            self._observer_timers.append(self.create_timer(
                1.0 / max(1.0, float(self.p['forensic_sync_rate_hz'])),
                lambda: self.safe_call(
                    'forensic_synchronized_map_frame',
                    self.forensic_synchronized_map_frame)))

    def csv_file(self,name,fields):
        f=open(self.directory/name,'w',newline='',encoding='utf-8'); self.files.append(f); w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); return w
    def csv_row(self,writer,row):
        with self._io_lock:
            if self._closed:self.dropped_samples+=1; return
            writer.writerow(finite(row))
    def qos(self,reliable=True,transient=False,depth=10): return QoSProfile(history=HistoryPolicy.KEEP_LAST,depth=depth,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
    def robot_base_frame(self, robot):
        configured = str(self.p.get('robot_base_frame', '') or '').strip()
        if configured:
            return configured if '/' in configured else f'{robot}/{configured}'
        return f'{robot}/base_footprint'
    def observe(self,message_type,topic,callback,qos,subsystem):
        return self.create_subscription(message_type,topic,lambda message:self.safe_call(subsystem,callback,message),qos)
    def start_forensic_ground_truth(self):
        """Start an external read-only Webots Supervisor observer.

        The observer is diagnostic-only and retries its controller connection
        while Webots is starting.  It never publishes ROS data or commands.
        """
        forensic_dir = self.directory / 'forensic'
        forensic_dir.mkdir(parents=True, exist_ok=True)
        output = forensic_dir / 'supervisor_ground_truth.csv'
        contact_output = forensic_dir / 'contact_points.csv'
        log_path = forensic_dir / 'supervisor_ground_truth.log'
        ready_file = forensic_dir / 'supervisor_ready.json'
        self.ground_truth_ready_file = ready_file
        environment = os.environ.copy()
        try:
            from ament_index_python.packages import get_package_prefix
            driver_prefix = get_package_prefix('webots_ros2_driver')
            python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
            controller_python = os.path.join(
                driver_prefix, 'lib', 'controller', 'python')
            driver_site = os.path.join(
                driver_prefix, 'lib', python_version, 'site-packages')
            environment['WEBOTS_HOME'] = driver_prefix
            environment['PYTHONPATH'] = os.pathsep.join(
                p for p in (controller_python, driver_site,
                            environment.get('PYTHONPATH', '')) if p)
        except Exception as exc:
            self.event('FORENSIC_SUPERVISOR_START_FAILED', str(exc),
                       severity='WARN', allow_during_shutdown=True)
            return
        environment['WEBOTS_CONTROLLER_URL'] = (
            f"tcp://{webots_controller_host()}:{self.p['webots_port']}/"
            'ForensicGroundTruthSupervisor')
        command = [sys.executable, '-m',
                   'my_epuck_project.cooperative_ground_truth_observer',
                   '--output', str(output)]
        command.extend(supervisor_robot_arguments(self.robots))
        command.extend(['--sample-period-s', str(float(
                       self.p['forensic_ground_truth_sample_period_s'])),
                   '--ready-file', str(ready_file),
                   '--controller-url', environment['WEBOTS_CONTROLLER_URL'],
                   '--runtime-directory', str(forensic_dir / 'runtime')])
        if self.contact_capture:
            command.extend([
                '--contact-output', str(contact_output),
                '--contact-sampling-period-ms', str(int(
                    self.p.get('contact_sampling_period_ms', 20))),
            ])
        try:
            self.ground_truth_log = log_path.open('w', encoding='utf-8')
            self.ground_truth_process = subprocess.Popen(
                command, env=environment, stdout=self.ground_truth_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            self.event('FORENSIC_SUPERVISOR_STARTED',
                       'external Webots ground-truth observer started',
                       supervisor_pid=self.ground_truth_process.pid,
                       output=str(output), allow_during_shutdown=True)
        except OSError as exc:
            self.event('FORENSIC_SUPERVISOR_START_FAILED', str(exc),
                       severity='WARN', allow_during_shutdown=True)

    def monitor_forensic_ground_truth(self):
        """Record Supervisor readiness and unexpected child termination.

        The Supervisor is an external diagnostic process.  A failed observer
        must be visible in the run artifact without taking down the passive
        ROS logger or changing the production control graph.
        """
        process = self.ground_truth_process
        if process is None:
            return
        if (self.ground_truth_ready_file is not None and
                self.ground_truth_ready_file.exists() and
                not self.counts['FORENSIC_SUPERVISOR_READY']):
            try:
                ready = json.loads(self.ground_truth_ready_file.read_text(
                    encoding='utf-8'))
            except (OSError, ValueError) as exc:
                self.event('FORENSIC_SUPERVISOR_READY_FILE_ERROR', str(exc),
                           severity='WARN', allow_during_shutdown=True)
            else:
                self.event('FORENSIC_SUPERVISOR_READY',
                           'external Webots Supervisor connected and found all robots',
                           supervisor_pid=process.pid, **ready,
                           allow_during_shutdown=True)
        return_code = process.poll()
        if return_code is not None and not self.ground_truth_exit_reported:
            self.ground_truth_exit_reported = True
            severity = 'INFO' if return_code == 0 else 'ERROR'
            self.event('FORENSIC_SUPERVISOR_EXITED',
                       'external Webots ground-truth observer exited',
                       severity=severity, return_code=return_code,
                       log_path=str(self.directory / 'forensic' /
                                    'supervisor_ground_truth.log'),
                       allow_during_shutdown=True)

    def stop_forensic_ground_truth(self):
        process = self.ground_truth_process
        if process is None:
            return
        if process.poll() is None:
            try:
                request_path = (self.directory / 'forensic' / 'runtime' /
                                'shutdown.requested')
                request_path.parent.mkdir(parents=True, exist_ok=True)
                request_path.write_text('shutdown_requested\n', encoding='utf-8')
                # Request an observer-owned shutdown before the launch group
                # is torn down.  The observer's monitor can then flush and
                # finalize even if its native Supervisor.step() call does
                # not return promptly; the signal below remains the normal
                # fallback for a responsive controller binding.
                process.wait(timeout=8.0)
            except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                try:
                    if process.poll() is None:
                        # Fallback for an observer whose Webots connection
                        # did not close during the normal launch shutdown.
                        # Keep the existing bounded signal/kill contract;
                        # the normal path above is the lifecycle fix.
                        process.send_signal(signal.SIGINT)
                        process.wait(timeout=3.0)
                except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                    try:
                        if process.poll() is None:
                            process.kill()
                        process.wait(timeout=3.0)
                    except (OSError, subprocess.TimeoutExpired, KeyboardInterrupt):
                        pass
        if self.ground_truth_log is not None:
            try:
                self.ground_truth_log.flush()
                self.ground_truth_log.close()
            except OSError:
                pass
        self.event('FORENSIC_SUPERVISOR_STOPPED',
                   'external ground-truth observer stopped',
                   return_code=process.returncode, allow_during_shutdown=True)

    def forensic_snapshot(self, force=False):
        if self.forensic is None:
            return
        now_ros = self.ros_seconds()
        now_wall = time.monotonic() - self.start
        if force:
            self.forensic.save_final_maps(self.latest, now_ros, now_wall)
        else:
            self.forensic.capture_maps(self.latest, now_ros, now_wall)
        for robot in self.robots:
            for target, source in (
                    (self.p['global_frame'], f'{robot}/map'),
                    (self.p['global_frame'], f'{robot}/odom'),
                    (f'{robot}/odom', self.robot_base_frame(robot))):
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target, source, Time(),
                        timeout=Duration(seconds=0.03))
                    self.forensic.record_transform(
                        now_ros, now_wall, target, source, transform=transform)
                except TransformException as exc:
                    self.forensic.record_transform(
                        now_ros, now_wall, target, source, error=str(exc))
        self.forensic.flush()

    @staticmethod
    def _normal_frame(frame):
        return str(frame or '').lstrip('/')

    def _record_direct_tf_message(self, message, static=False):
        """Cache bounded planar projections of the raw TF channels."""
        store = self._direct_tf_static if static else self._direct_tf_samples
        for item in getattr(message, 'transforms', ()):
            parent = self._normal_frame(item.header.frame_id)
            child = self._normal_frame(item.child_frame_id)
            if not parent or not child:
                continue
            stamp = item.header.stamp
            stamp_value = (int(stamp.sec) +
                           int(stamp.nanosec) * 1.0e-9)
            translation = item.transform.translation
            rotation = item.transform.rotation
            value = (float(translation.x), float(translation.y),
                     float(yaw(rotation)))
            if not all(math.isfinite(part) for part in value):
                continue
            key = (parent, child)
            # TF publishers repeat the same frame edge on nearly every
            # message.  The graph is immutable after an edge is registered;
            # avoid reacquiring its lock for those repeated evidence rows.
            # The raw sample is still appended below without alteration.
            if key not in self._direct_tf_graph_keys:
                self._register_direct_tf_key(key)
            if static:
                store[key] = (stamp_value, value)
                continue
            samples = store.setdefault(
                key, RawTFSeriesIndex(self._direct_tf_sample_limit))
            if isinstance(samples, RawTFSeriesIndex):
                samples.append(stamp_value, value)
            else:
                # Compatibility for a test or an older in-memory observer
                # object that still carries a plain deque.
                samples.append((stamp_value, value))

    def _register_direct_tf_key(self, key):
        """Add a TF edge to the cached undirected traversal graph once."""
        with self._direct_tf_graph_lock:
            if key in self._direct_tf_graph_keys:
                return
            self._direct_tf_graph_keys.add(key)
            parent, child = key
            self._direct_tf_adjacency.setdefault(child, []).append(
                (parent, False))
            self._direct_tf_adjacency.setdefault(parent, []).append(
                (child, True))
            self._direct_tf_adjacency[child].sort()
            self._direct_tf_adjacency[parent].sort()
            self._direct_tf_adjacency_snapshot = None

    def _direct_tf_graph(self):
        """Return an immutable cached graph snapshot for TF traversal."""
        with self._direct_tf_graph_lock:
            if self._direct_tf_adjacency_snapshot is None:
                # This compatibility path is used only by offline/unit-test
                # objects that populate the dictionaries directly rather than
                # through _record_direct_tf_message().
                for key in set(self._direct_tf_samples) | set(
                        self._direct_tf_static):
                    self._register_direct_tf_key(key)
                self._direct_tf_adjacency_snapshot = {
                    frame: tuple(neighbours)
                    for frame, neighbours in self._direct_tf_adjacency.items()
                }
            return self._direct_tf_adjacency_snapshot

    def _direct_tf_series(self, key):
        """Return sorted values/timestamps with legacy stable ordering."""
        samples = self._direct_tf_samples.get(key)
        if samples is None:
            return (), ()
        if isinstance(samples, RawTFSeriesIndex):
            timestamps, values = samples.sorted_snapshot()
            return values, timestamps
        values = list(samples)
        values.sort(key=lambda item: item[0])
        return tuple(values), tuple(item[0] for item in values)

    def _record_sync_map_timing(self, stage, elapsed):
        if not getattr(self, '_sync_map_profile_enabled', False):
            return
        entry = self._sync_map_profile.setdefault(str(stage), [])
        entry.append(float(max(0.0, elapsed)))

    def _sync_map_timing_summary(self):
        if not getattr(self, '_sync_map_profile_enabled', False):
            return {'enabled': False, 'stages': {}}
        summary = {}
        for stage, values in self._sync_map_profile.items():
            if not values:
                summary[stage] = {
                    'calls': 0, 'total_wall_s': 0.0,
                    'median_wall_s': 0.0, 'p95_wall_s': 0.0,
                    'max_wall_s': 0.0,
                }
                continue
            ordered = sorted(values)
            p95_index = min(len(ordered) - 1,
                            max(0, int(math.ceil(0.95 * len(ordered))) - 1))
            summary[stage] = {
                'calls': len(values),
                'total_wall_s': float(sum(values)),
                'median_wall_s': float(statistics.median(values)),
                'p95_wall_s': float(ordered[p95_index]),
                'max_wall_s': float(max(values)),
            }
        return {'enabled': True, 'stages': summary}

    def _record_high_rate_timing(self, component, elapsed):
        """Record opt-in sub-timings for the odom/TF evidence callbacks."""
        if not getattr(self, '_high_rate_profile_enabled', False):
            return
        with self._state_lock:
            entry = self._high_rate_profile.setdefault(
                str(component),
                {'calls': 0, 'total_wall_s': 0.0, 'max_wall_s': 0.0},
            )
            elapsed = max(0.0, float(elapsed))
            entry['calls'] += 1
            entry['total_wall_s'] += elapsed
            entry['max_wall_s'] = max(entry['max_wall_s'], elapsed)

    def _direct_tf_message(self, message):
        # This is the single /tf subscription for the logger.  Feed the
        # existing tf2 buffer before recording the identical raw message so
        # online lookup behavior and raw-TF evidence remain available without
        # a duplicate TransformListener subscription.
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        for transform in getattr(message, 'transforms', ()):
            key = (self._normal_frame(transform.header.frame_id),
                   self._normal_frame(transform.child_frame_id))
            live_edges = getattr(self, '_tf_buffer_live_dynamic_edges', None)
            if live_edges is None or key in live_edges:
                self.tf_buffer.set_transform(
                    transform, 'default_authority')
        if started is not None:
            self._record_high_rate_timing(
                'tf.tf2_buffer_update', time.perf_counter() - started)
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        if self.forensic is not None:
            self._record_direct_tf_message(message, static=False)
            if started is not None:
                self._record_high_rate_timing(
                    'tf.direct_index', time.perf_counter() - started)
            started = (time.perf_counter()
                       if getattr(self, '_high_rate_profile_enabled', False)
                       else None)
            self.forensic.record_raw_tf(
                '/tf', message, self.ros_seconds(),
                time.monotonic() - self.start, static=False)
            if started is not None:
                self._record_high_rate_timing(
                    'tf.csv_serialization', time.perf_counter() - started)
        else:
            self._record_direct_tf_message(message, static=False)
            if started is not None:
                self._record_high_rate_timing(
                    'tf.direct_index', time.perf_counter() - started)

    def _direct_tf_static_message(self, message):
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        for transform in getattr(message, 'transforms', ()):
            self.tf_buffer.set_transform_static(transform, 'default_authority')
        if started is not None:
            self._record_high_rate_timing(
                'tf_static.tf2_buffer_update', time.perf_counter() - started)
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        if self.forensic is not None:
            self._record_direct_tf_message(message, static=True)
            if started is not None:
                self._record_high_rate_timing(
                    'tf_static.direct_index', time.perf_counter() - started)
            started = (time.perf_counter()
                       if getattr(self, '_high_rate_profile_enabled', False)
                       else None)
            self.forensic.record_raw_tf(
                '/tf_static', message, self.ros_seconds(),
                time.monotonic() - self.start, static=True)
            if started is not None:
                self._record_high_rate_timing(
                    'tf_static.csv_serialization', time.perf_counter() - started)
        else:
            self._record_direct_tf_message(message, static=True)
            if started is not None:
                self._record_high_rate_timing(
                    'tf_static.direct_index', time.perf_counter() - started)

    def _direct_tf_edge(self, parent, child, query_ros,
                        allow_latest_before=False):
        """Return a tightly bracketed raw-TF edge at ``query_ros``."""
        key = (self._normal_frame(parent), self._normal_frame(child))
        if key in self._direct_tf_static:
            stamp_value, transform = self._direct_tf_static[key]
            return transform, {
                'sample_age_s': 0.0,
                'interpolation_used': False,
                'interpolation_age_s': 0.0,
                'interpolation_span_s': 0.0,
                'lookup_mode': 'raw_tf_static',
            }
        series = self._direct_tf_samples.get(key)
        if isinstance(series, RawTFSeriesIndex):
            bounds = series.lookup_bounds(query_ros)
            if bounds is None:
                return None
            kind = bounds[0]
            if allow_latest_before and kind != 'before':
                sample_timestamp, transform = bounds[1:3]
                return transform, {
                    'sample_age_s': max(0.0, float(query_ros) -
                                        float(sample_timestamp)),
                    'interpolation_used': False,
                    'interpolation_age_s': max(
                        0.0, float(query_ros) - float(sample_timestamp)),
                    'interpolation_span_s': 0.0,
                    'lookup_mode': 'raw_tf_latest_valid_before_query',
                }
            if kind == 'before':
                sample_timestamp, transform = bounds[1:3]
                if float(sample_timestamp) - float(query_ros) > 0.10:
                    return None
                return transform, {
                    'sample_age_s': abs(float(sample_timestamp) -
                                       float(query_ros)),
                    'interpolation_used': False,
                    'interpolation_age_s': abs(
                        float(sample_timestamp) - float(query_ros)),
                    'interpolation_span_s': 0.0,
                    'lookup_mode': 'raw_tf_bounded_nearest',
                }
            if kind == 'after':
                sample_timestamp, transform = bounds[1:3]
                delta = abs(float(sample_timestamp) - float(query_ros))
                if delta <= 1.0e-9:
                    mode = 'raw_tf_exact'
                elif delta > 0.10:
                    return None
                else:
                    mode = 'raw_tf_bounded_nearest'
                return transform, {
                    'sample_age_s': delta,
                    'interpolation_used': False,
                    'interpolation_age_s': delta,
                    'interpolation_span_s': 0.0,
                    'lookup_mode': mode,
                }
            _, left_timestamp, left_transform, right_timestamp, right_transform = bounds
            if abs(float(left_timestamp) - float(query_ros)) <= 1.0e-9:
                return left_transform, {
                    'sample_age_s': 0.0,
                    'interpolation_used': False,
                    'interpolation_age_s': 0.0,
                    'interpolation_span_s': 0.0,
                    'lookup_mode': 'raw_tf_exact',
                }
            span = float(right_timestamp - left_timestamp)
            if span <= 0.0 or span > 0.10:
                return None
            fraction = (float(query_ros) - float(left_timestamp)) / span
            return _interpolate_planar(left_transform, right_transform,
                                       fraction), {
                'sample_age_s': max(
                    abs(float(query_ros) - float(left_timestamp)),
                    abs(float(right_timestamp) - float(query_ros))),
                'interpolation_used': True,
                'interpolation_age_s': max(
                    abs(float(query_ros) - float(left_timestamp)),
                    abs(float(right_timestamp) - float(query_ros))),
                'interpolation_span_s': span,
                'lookup_mode': 'raw_tf_tightly_interpolated',
            }
        values, timestamps = self._direct_tf_series(key)
        if not values:
            return None
        query_ros = float(query_ros)
        if allow_latest_before:
            right_index = bisect_right(timestamps, query_ros)
            if right_index:
                latest = values[right_index - 1]
                return latest[1], {
                    'sample_age_s': max(0.0, query_ros - latest[0]),
                    'interpolation_used': False,
                    'interpolation_age_s': max(0.0, query_ros - latest[0]),
                    'interpolation_span_s': 0.0,
                    'lookup_mode': 'raw_tf_latest_valid_before_query',
                }
        insertion = bisect_left(timestamps, query_ros)
        if insertion == 0:
            nearest_index = 0
        elif insertion == len(values):
            nearest_index = len(values) - 1
        elif (abs(timestamps[insertion - 1] - query_ros) <=
              abs(timestamps[insertion] - query_ros)):
            nearest_index = insertion - 1
        else:
            nearest_index = insertion
        exact = values[nearest_index]
        if abs(exact[0] - query_ros) <= 1.0e-9:
            return exact[1], {
                'sample_age_s': 0.0,
                'interpolation_used': False,
                'interpolation_age_s': 0.0,
                'interpolation_span_s': 0.0,
                'lookup_mode': 'raw_tf_exact',
            }
        if query_ros < values[0][0] or query_ros > values[-1][0]:
            if abs(exact[0] - query_ros) > 0.10:
                return None
            return exact[1], {
                'sample_age_s': abs(exact[0] - query_ros),
                'interpolation_used': False,
                'interpolation_age_s': abs(exact[0] - query_ros),
                'interpolation_span_s': 0.0,
                'lookup_mode': 'raw_tf_bounded_nearest',
            }
        right_index = bisect_right(timestamps, query_ros)
        left = values[right_index - 1]
        right = values[right_index]
        span = float(right[0] - left[0])
        if span <= 0.0 or span > 0.10:
            return None
        fraction = (query_ros - left[0]) / span
        return _interpolate_planar(left[1], right[1], fraction), {
            'sample_age_s': max(abs(query_ros - left[0]),
                                abs(right[0] - query_ros)),
            'interpolation_used': True,
            'interpolation_age_s': max(abs(query_ros - left[0]),
                                       abs(right[0] - query_ros)),
            'interpolation_span_s': span,
            'lookup_mode': 'raw_tf_tightly_interpolated',
        }

    def _direct_sync_tf(self, target, source, query_ros,
                        allow_latest_before=False):
        """Compose a local raw-TF path without using latest wall-time data."""
        target = self._normal_frame(target)
        source = self._normal_frame(source)
        if target == source:
            return (0.0, 0.0, 0.0), {
                'sample_age_s': 0.0, 'interpolation_used': False,
                'interpolation_age_s': 0.0, 'interpolation_span_s': 0.0,
                'lookup_mode': 'raw_tf_identity', 'path': [source],
            }
        graph_started = (time.perf_counter()
                         if getattr(self, '_sync_map_profile_enabled', False)
                         else None)
        adjacency = self._direct_tf_graph()
        if graph_started is not None:
            self._record_sync_map_timing(
                'raw_tf_graph_snapshot', time.perf_counter() - graph_started)
        traversal_started = (time.perf_counter()
                             if getattr(self, '_sync_map_profile_enabled', False)
                             else None)
        queue = deque([(source, (0.0, 0.0, 0.0), 0.0, False, 0.0,
                        [source])])
        visited = {source}
        try:
            while queue:
                current, accumulated, max_age, interpolated, max_span, path = (
                    queue.popleft())
                neighbours = adjacency.get(current, ())
                for neighbour, inverse in neighbours:
                    if neighbour in visited:
                        continue
                    parent, child = ((neighbour, current) if not inverse else
                                     (current, neighbour))
                    edge = self._direct_tf_edge(
                        parent, child, query_ros,
                        allow_latest_before=allow_latest_before)
                    if edge is None:
                        continue
                    edge_transform, edge_meta = edge
                    if inverse:
                        edge_transform = _invert_planar(edge_transform)
                    compose_started = (time.perf_counter()
                                       if getattr(
                                           self, '_sync_map_profile_enabled',
                                           False) else None)
                    composed = _compose_planar(edge_transform, accumulated)
                    if compose_started is not None:
                        self._record_sync_map_timing(
                            'raw_tf_transform_composition',
                            time.perf_counter() - compose_started)
                    edge_age = float(edge_meta.get('sample_age_s', 0.0))
                    edge_span = float(edge_meta.get('interpolation_span_s', 0.0))
                    next_path = path + [neighbour]
                    if neighbour == target:
                        return composed, {
                            'sample_age_s': max(max_age, edge_age),
                            'interpolation_used': bool(
                                interpolated or edge_meta.get(
                                    'interpolation_used', False)),
                            'interpolation_age_s': max(max_age, edge_age),
                            'interpolation_span_s': max(max_span, edge_span),
                            'lookup_mode': 'raw_tf_bounded_composed',
                            'path': next_path,
                        }
                    visited.add(neighbour)
                    queue.append((
                        neighbour, composed, max(max_age, edge_age),
                        bool(interpolated or edge_meta.get(
                            'interpolation_used', False)),
                        max(max_span, edge_span), next_path))
            return None, {
                'lookup_mode': 'raw_tf_unavailable',
                'path': [source],
                'error': f'no bounded raw TF path {target} <- {source}',
            }
        finally:
            if traversal_started is not None:
                self._record_sync_map_timing(
                    'raw_tf_graph_traversal',
                    time.perf_counter() - traversal_started)

    @staticmethod
    def _planar_tf_observation(transform, target, source, query_ros, metadata):
        heading = float(transform[2])
        return {
            'available': True,
            'transform_stamp_s': float(query_ros),
            'age_s': float(metadata.get('sample_age_s', 0.0)),
            'lookup_mode': str(metadata.get('lookup_mode',
                                            'raw_tf_bounded_composed')),
            'requested_stamp_s': float(query_ros),
            'returned_stamp_delta_s': 0.0,
            'interpolation_used': bool(metadata.get('interpolation_used',
                                                   False)),
            'interpolation_age_s': float(metadata.get(
                'interpolation_age_s', 0.0)),
            'interpolation_span_s': float(metadata.get(
                'interpolation_span_s', 0.0)),
            'translation': {
                'x': float(transform[0]), 'y': float(transform[1]), 'z': 0.0},
            'quaternion': {
                'x': 0.0, 'y': 0.0, 'z': math.sin(heading / 2.0),
                'w': math.cos(heading / 2.0)},
            'target_frame': target,
            'source_frame': source,
            'path': list(metadata.get('path', ())),
            'error': '',
        }

    @staticmethod
    def _tf_observation(transform, query_ros):
        if transform is None:
            return {
                'available': False,
                'transform_stamp_s': None,
                'age_s': None,
                'lookup_mode': 'exact_ros_time',
                'requested_stamp_s': float(query_ros),
                'returned_stamp_delta_s': None,
                'interpolation_used': False,
                'translation': None,
                'quaternion': None,
                'error': '',
            }
        stamp_value = transform.header.stamp.sec + (
            transform.header.stamp.nanosec * 1e-9)
        t = transform.transform.translation
        q = transform.transform.rotation
        return {
            'available': True,
            'transform_stamp_s': float(stamp_value),
            'age_s': float(query_ros - stamp_value),
            'lookup_mode': 'exact_ros_time',
            'requested_stamp_s': float(query_ros),
            'returned_stamp_delta_s': float(stamp_value - query_ros),
            'interpolation_used': False,
            'translation': {
                'x': float(t.x), 'y': float(t.y), 'z': float(t.z)},
            'quaternion': {
                'x': float(q.x), 'y': float(q.y),
                'z': float(q.z), 'w': float(q.w)},
            'error': '',
        }

    def _lookup_odom_pose(self, robot, query_ros):
        """Return the local ^odom T_base from native odometry samples."""
        index = self._odom_samples.get(robot)
        if index is None:
            return self._tf_observation(None, query_ros)
        bounds = index.lookup_bounds(query_ros)
        if bounds is None:
            return self._tf_observation(None, query_ros)
        query_ros = float(query_ros)
        if bounds[0] == 'before':
            nearest_timestamp, pose = bounds[1:]
            if nearest_timestamp - query_ros > 0.10:
                return self._tf_observation(None, query_ros)
            metadata = {
                'sample_age_s': nearest_timestamp - query_ros,
                'interpolation_used': False,
                'interpolation_age_s': nearest_timestamp - query_ros,
                'interpolation_span_s': 0.0,
                'lookup_mode': 'native_odom_bounded_nearest',
                'path': [pose[3], pose[4]],
            }
        elif bounds[0] == 'after':
            sample_timestamp, pose = bounds[1:]
            age = max(0.0, query_ros - sample_timestamp)
            if age > 0.10:
                return self._tf_observation(None, query_ros)
            metadata = {
                'sample_age_s': age, 'interpolation_used': False,
                'interpolation_age_s': age, 'interpolation_span_s': 0.0,
                'lookup_mode': 'native_odom_latest_before_query',
                'path': [pose[3], pose[4]],
            }
        else:
            _, left_timestamp, left_pose, right_timestamp, right_pose = bounds
            span = float(right_timestamp - left_timestamp)
            if span <= 0.0 or span > 0.10:
                return self._tf_observation(None, query_ros)
            fraction = (query_ros - left_timestamp) / span
            interpolated = _interpolate_planar(
                left_pose[:3], right_pose[:3], fraction)
            pose = (*interpolated, left_pose[3], left_pose[4])
            metadata = {
                'sample_age_s': max(query_ros - left_timestamp,
                                   right_timestamp - query_ros),
                'interpolation_used': True,
                'interpolation_age_s': max(query_ros - left_timestamp,
                                           right_timestamp - query_ros),
                'interpolation_span_s': span,
                'lookup_mode': 'native_odom_tightly_interpolated',
                'path': [pose[3], pose[4]],
            }
        return self._planar_tf_observation(
            pose[:3], pose[3], pose[4],
            query_ros, metadata)

    def _lookup_sync_tf(self, target, source, query_ros,
                        allow_latest_before=False):
        lookup_error = ''
        tf2_started = (time.perf_counter()
                       if getattr(self, '_sync_map_profile_enabled', False)
                       else None)
        try:
            lookup_time = Time(
                seconds=float(query_ros), clock_type=ClockType.ROS_TIME)
            transform = self.tf_buffer.lookup_transform(
                # This callback is passive evidence collection.  The direct
                # raw-TF stream below is already maintained for exact-time
                # fallback, so waiting here can block the logger executor on
                # every synchronized sample without improving estimator or
                # navigation behavior.  Preserve the same tf2 result when it
                # is immediately available, then use the existing bounded raw
                # fallback when it is not.
                target, source, lookup_time, timeout=Duration(seconds=0.0))
            result = self._tf_observation(transform, query_ros)
            result['target_frame'] = target
            result['source_frame'] = source
            if (result.get('available') and
                    abs(float(result.get('returned_stamp_delta_s', 0.0)))
                    <= 0.10):
                result['synchronization_valid'] = True
                return result
            lookup_error = 'tf2 result outside 0.10 s synchronization bound'
        except TransformException as exc:
            lookup_error = str(exc)
        except (RuntimeError, TypeError, ValueError) as exc:
            lookup_error = str(exc)
        finally:
            if tf2_started is not None:
                self._record_sync_map_timing(
                    'tf2_lookup', time.perf_counter() - tf2_started)

        raw_started = (time.perf_counter()
                       if getattr(self, '_sync_map_profile_enabled', False)
                       else None)
        planar, metadata = self._direct_sync_tf(
            target, source, query_ros,
            allow_latest_before=allow_latest_before)
        if planar is not None:
            result = self._planar_tf_observation(
                planar, target, source, query_ros, metadata)
            result['synchronization_valid'] = bool(
                float(result.get('age_s', 0.0)) <= 0.10)
            if raw_started is not None:
                self._record_sync_map_timing(
                    'raw_tf_fallback_lookup', time.perf_counter() - raw_started)
            return result
        result = self._tf_observation(None, query_ros)
        result['target_frame'] = target
        result['source_frame'] = source
        result['synchronization_valid'] = False
        result['lookup_mode'] = str(metadata.get('lookup_mode',
                                                'unavailable'))
        result['path'] = list(metadata.get('path', ()))
        result['error'] = '; '.join(value for value in (
            lookup_error, metadata.get('error', '')) if value)
        if raw_started is not None:
            self._record_sync_map_timing(
                'raw_tf_fallback_lookup', time.perf_counter() - raw_started)
        return result

    def _lookup_sync_tf_direct_only(self, target, source, query_ros):
        """Capture only an immediately available tf2 result.

        This is the small live portion retained by deferred forensic capture.
        It preserves the legacy tf2 result when it exists, but deliberately
        does not construct the bounded raw-TF graph or compose fallback edges
        on the simulation executor.  A missing/unsynchronized result is
        reconstructed from the preserved raw stream after the run.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source,
                Time(seconds=float(query_ros), clock_type=ClockType.ROS_TIME),
                timeout=Duration(seconds=0.0))
            result = self._tf_observation(transform, query_ros)
            result['target_frame'] = target
            result['source_frame'] = source
            if (result.get('available') and
                    abs(float(result.get('returned_stamp_delta_s', 0.0)))
                    <= 0.10):
                result['synchronization_valid'] = True
                return result
        except (TransformException, RuntimeError, TypeError, ValueError):
            pass
        return None

    @staticmethod
    def _supervisor_pose_at(rows, query_ros, timestamp_index=None):
        """Join a dense Supervisor trajectory at one simulation timestamp."""
        index = timestamp_index or SupervisorTimestampIndex(rows)
        nearest = index.nearest(query_ros)
        if nearest is None:
            return None
        values = index.values
        query_ros = float(query_ros)
        if abs(nearest[0] - query_ros) <= 1.0e-9:
            return {
                'pose': nearest[1], 'alignment_error_s': 0.0,
                'interpolation_used': False, 'interpolation_span_s': 0.0,
            }
        if query_ros < values[0][0] or query_ros > values[-1][0]:
            if abs(nearest[0] - query_ros) > 0.10:
                return None
            return {
                'pose': nearest[1],
                'alignment_error_s': abs(nearest[0] - query_ros),
                'interpolation_used': False, 'interpolation_span_s': 0.0,
            }
        right_index = next(
            index for index, item in enumerate(values)
            if item[0] > query_ros)
        left, right = values[right_index - 1], values[right_index]
        span = float(right[0] - left[0])
        if span <= 0.0 or span > 0.10:
            return None
        fraction = (query_ros - left[0]) / span
        pose = _interpolate_planar(left[1], right[1], fraction)
        return {
            'pose': pose,
            'alignment_error_s': max(abs(query_ros - left[0]),
                                     abs(right[0] - query_ros)),
            'interpolation_used': True,
            'interpolation_span_s': span,
        }

    @staticmethod
    def _map_base_from_sync_row(row):
        """Extract M->B from one row, composing the local chain if needed."""
        def value(observation):
            if not isinstance(observation, dict) or not observation.get(
                    'available'):
                return None
            translation = observation.get('translation') or {}
            quaternion = observation.get('quaternion') or {}
            try:
                heading = math.atan2(
                    2.0 * (float(quaternion['w']) * float(quaternion['z']) +
                           float(quaternion['x']) * float(quaternion['y'])),
                    1.0 - 2.0 * (float(quaternion['y']) ** 2 +
                                 float(quaternion['z']) ** 2))
                transform = (float(translation['x']),
                             float(translation['y']), heading)
                if not all(math.isfinite(item) for item in transform):
                    return None
                return transform
            except (KeyError, TypeError, ValueError):
                return None

        # Prefer the actual local map->odom and odom->base chain.  A direct
        # composed TF lookup can be a convenience view with different TF
        # buffering semantics; the local chain is the evaluator's explicit
        # physical-map gauge and is sufficient before cross-robot handoff.
        map_to_odom = value(row.get('map_to_odom'))
        odom_to_base = value(row.get('odom_to_base'))
        if map_to_odom is not None and odom_to_base is not None:
            map_to_odom_observation = row.get('map_to_odom') or {}
            odom_to_base_observation = row.get('odom_to_base') or {}
            metadata = {
                'available': True,
                'lookup_mode': 'row_composed_map_to_odom_odom_to_base',
                'interpolation_used': bool(
                    map_to_odom_observation.get('interpolation_used', False) or
                    odom_to_base_observation.get('interpolation_used', False)),
                'age_s': max(float(map_to_odom_observation.get('age_s') or 0.0),
                            float(odom_to_base_observation.get('age_s') or 0.0)),
                'interpolation_age_s': max(
                    float(map_to_odom_observation.get(
                        'interpolation_age_s') or 0.0),
                    float(odom_to_base_observation.get(
                        'interpolation_age_s') or 0.0)),
                'map_to_odom_age_s': float(
                    map_to_odom_observation.get('age_s') or 0.0),
                'odom_to_base_age_s': float(
                    odom_to_base_observation.get('age_s') or 0.0),
            }
            return _compose_planar(map_to_odom, odom_to_base), metadata
        direct = value(row.get('map_to_base'))
        if direct is not None:
            direct_metadata = dict(row.get('map_to_base') or {})
            direct_metadata.setdefault('lookup_mode', 'direct_map_to_base_fallback')
            return direct, direct_metadata
        return None, None

    @classmethod
    def _fixed_map_anchor_from_rows(cls, robot, supervisor_rows, sync_rows):
        """Derive W<-M from the earliest valid pre-motion gauge row.

        This is evaluation-only.  A late map/odom observation after the
        robot has rotated can create a misleading physical yaw reference, so
        prefer an observation still near the initial Supervisor pose.  If a
        run has no such observation, reject the anchor explicitly rather than
        silently treating a post-motion pose as initialization.
        """
        del robot
        ordered = sorted(
            (row for row in sync_rows
             if isinstance(row.get('query_ros_time_s'), (int, float))),
            key=lambda row: float(row['query_ros_time_s']))
        supervisor_values = sorted(
            (row for row in supervisor_rows
             if isinstance(row, tuple) and len(row) == 2),
            key=lambda item: float(item[0]))
        supervisor_index = SupervisorTimestampIndex(supervisor_rows)
        initial_pose = supervisor_values[0][1] if supervisor_values else None
        pre_motion_candidates = []
        for row in ordered:
            query_ros = float(row['query_ros_time_s'])
            supervisor = cls._supervisor_pose_at(
                supervisor_rows, query_ros, supervisor_index)
            if supervisor is None or supervisor['alignment_error_s'] > 0.10:
                continue
            map_to_base, observation = cls._map_base_from_sync_row(row)
            if map_to_base is None:
                continue
            # Odometry is interpolated/tightly joined.  map->odom is allowed
            # to have an old message stamp when it is the latest valid TF
            # state; it is not a periodically re-published motion sample.
            motion_age = float((observation or {}).get(
                'odom_to_base_age_s') or 0.0)
            if motion_age > 0.10:
                continue
            world_map = _compose_planar(
                supervisor['pose'], _invert_planar(map_to_base))
            candidate = {
                'anchor_time_s': query_ros,
                'supervisor': supervisor,
                'map_to_base': map_to_base,
                'map_to_base_observation': observation,
                'world_to_map': world_map,
            }
            if initial_pose is not None:
                displacement = math.hypot(
                    float(supervisor['pose'][0]) - float(initial_pose[0]),
                    float(supervisor['pose'][1]) - float(initial_pose[1]),
                )
                heading_delta = abs(_wrap_planar_yaw(
                    float(supervisor['pose'][2]) - float(initial_pose[2])))
                candidate['motion_from_initial_m'] = displacement
                candidate['heading_from_initial_rad'] = heading_delta
                if displacement <= 0.10 and heading_delta <= 0.50:
                    pre_motion_candidates.append(candidate)
            else:
                pre_motion_candidates.append(candidate)
        if pre_motion_candidates:
            return pre_motion_candidates[0]
        return None

    def _write_physical_gt_evaluation(self):
        """Write post-run physical map-frame accuracy, never estimator input."""
        if self.forensic is None:
            return None
        output_path = self.directory / 'forensic' / 'physical_gt_evaluation.json'
        result = {
            'schema_version': 'physical_map_frame_gt_1.0',
            'evaluation_only': True,
            'estimator_input_connection': False,
            'formula': {
                'world_map_i': 'W_T_Bi * inverse(Mi_T_Bi)',
                'map2_map1': 'inverse(W_T_M2) * W_T_M1',
                'error': 'inverse(T_GT) * T_EST',
                'convention': 'p_target = R(theta) p_source + t',
            },
            'world_planar_convention': (
                'Supervisor observer world_x/world_y and heading_x/heading_y'),
            'physical_gt_valid': False,
            'reason': '',
        }
        try:
            frontend_directory = self.frontend_diagnostic_directory()
            summaries = []
            for robot in self.robots:
                path = frontend_directory / f'{robot}_unknown_pose_frontend.json'
                if not path.is_file():
                    continue
                payload = json.loads(path.read_text(encoding='utf-8'))
                accepted = payload.get('accepted_hypothesis')
                if accepted:
                    summaries.append((robot, payload, accepted))
            if not summaries:
                result['reason'] = 'NO_ACCEPTED_CANONICAL_HANDOFF'
                atomic_json(output_path, result)
                return result
            if len(self.robots) < 2:
                result.update({
                    'reason': 'SINGLE_ROBOT_NOT_APPLICABLE',
                    'robot_ids': list(self.robots),
                })
                atomic_json(output_path, result)
                return result
            accepted = next((item[2] for item in summaries
                             if item[2].get('source_robot_id') == 'robot1'),
                            summaries[0][2])
            query_ros = accepted.get('accepted_ros_time_s')
            if query_ros is None:
                result['reason'] = 'ACCEPTANCE_SIM_TIME_UNAVAILABLE'
                atomic_json(output_path, result)
                return result
            supervisor_path = self.directory / 'forensic' / \
                'supervisor_ground_truth.csv'
            synchronized_path = self.directory / 'forensic' / \
                'synchronized_map_frame.jsonl'
            if not supervisor_path.is_file() or not synchronized_path.is_file():
                result['reason'] = 'SUPERVISOR_OR_SYNCHRONIZED_TF_ARTIFACT_MISSING'
                atomic_json(output_path, result)
                return result
            supervisor_rows = {robot: [] for robot in self.robots}
            with supervisor_path.open(newline='', encoding='utf-8') as stream:
                for row in csv.DictReader(stream):
                    robot = str(row.get('robot_id', ''))
                    if robot not in supervisor_rows:
                        continue
                    try:
                        supervisor_rows[robot].append((
                            float(row['sim_time_s']),
                            (float(row['world_x_m']),
                             float(row['world_y_m']),
                             float(row['planar_yaw_rad']))))
                    except (KeyError, TypeError, ValueError):
                        continue
            sync_rows = {robot: [] for robot in self.robots}
            with synchronized_path.open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                    except (TypeError, ValueError, json.JSONDecodeError):
                        continue
                    robot = str(row.get('robot_id', ''))
                    if robot in sync_rows:
                        sync_rows[robot].append(row)

            # The physical map-frame reference is an initialization gauge,
            # not a time-varying reconstruction from later SLAM localization.
            # Freeze one anchor per map at the earliest synchronized local
            # map/odom/base observation and use it for the whole run.
            anchors = {}
            for robot in self.robots:
                anchor = self._fixed_map_anchor_from_rows(
                    robot, supervisor_rows[robot], sync_rows[robot])
                if anchor is None:
                    result['reason'] = f'{robot.upper()}_INITIALIZATION_ANCHOR_UNAVAILABLE'
                    atomic_json(output_path, result)
                    return result
                anchors[robot] = anchor
            gt = _compose_planar(
                _invert_planar(anchors['robot2']['world_to_map']),
                anchors['robot1']['world_to_map'])
            joins = {}
            supervisor_indexes = {
                robot: SupervisorTimestampIndex(supervisor_rows[robot])
                for robot in self.robots}
            for robot in self.robots:
                supervisor = self._supervisor_pose_at(
                    supervisor_rows[robot], query_ros,
                    supervisor_indexes[robot])
                joins[robot] = {
                    'supervisor_at_acceptance': supervisor,
                    'initialization_anchor': anchors[robot],
                }
            estimated = tuple(float(value) for value in
                              accepted.get('transform_se2', ()))
            if len(estimated) != 3 or not all(
                    math.isfinite(value) for value in estimated):
                result['reason'] = 'ACCEPTED_TRANSFORM_UNAVAILABLE'
                atomic_json(output_path, result)
                return result
            error_transform = _compose_planar(_invert_planar(gt), estimated)
            result.update({
                'physical_gt_valid': True,
                'accepted_robot_summary': summaries[0][0],
                'accepted_sim_time_s': float(query_ros),
                'gt_reference': 'FIXED_INITIALIZATION_ANCHOR_PHYSICAL_REFERENCE',
                'gt_canonical_r1_to_r2': {
                    'tx': gt[0], 'ty': gt[1],
                    'yaw_rad': gt[2], 'yaw_deg': math.degrees(gt[2])},
                'estimated_canonical_r1_to_r2': {
                    'tx': estimated[0], 'ty': estimated[1],
                    'yaw_rad': estimated[2],
                    'yaw_deg': math.degrees(estimated[2])},
                'error_transform': {
                    'tx': error_transform[0], 'ty': error_transform[1],
                    'yaw_rad': error_transform[2],
                    'yaw_deg': math.degrees(error_transform[2])},
                'physical_translation_error_m': math.hypot(
                    error_transform[0], error_transform[1]),
                'physical_yaw_error_deg': abs(math.degrees(
                    _wrap_planar_yaw(estimated[2] - gt[2]))),
                'gt_time_synchronization_error_s': max(
                    anchors[robot]['supervisor']['alignment_error_s']
                    for robot in self.robots),
                'map_tf_age_or_interpolation_s': {
                    robot: float((anchors[robot]['map_to_base_observation'] or {}).get(
                        'age_s') or (anchors[robot]['map_to_base_observation'] or {}).get(
                        'interpolation_age_s') or 0.0)
                    for robot in self.robots},
                'initialization_anchors': anchors,
                'joins': joins,
            })
        except (OSError, TypeError, ValueError, KeyError,
                json.JSONDecodeError) as exc:
            result['reason'] = f'EVALUATION_EXCEPTION:{type(exc).__name__}:{exc}'
        atomic_json(output_path, result)
        return result

    def _synchronized_map_frame_row(self, robot, query_ros, query_wall,
                                    sample_kind, request=None):
        map_started = (time.perf_counter()
                       if getattr(self, '_sync_map_profile_enabled', False)
                       else None)
        if request is None:
            evidence = self.latest_evidence.get(robot)
            odom_message = self.latest[robot].get('odom')
            map_message = self.latest[robot].get('map')
            map_frame = (str(map_message.header.frame_id)
                         if map_message is not None and
                         map_message.header.frame_id else f'{robot}/map')
            odom_frame = (str(odom_message.header.frame_id)
                          if odom_message is not None and
                          odom_message.header.frame_id else f'{robot}/odom')
            base_frame = (str(getattr(odom_message, 'child_frame_id', ''))
                          if odom_message is not None else '')
            base_frame = base_frame or self.robot_base_frame(robot)
        else:
            # These values are captured at the original timer/callback time;
            # using them during the post-run reconstruction preserves the
            # legacy frame/evidence join rather than consulting a later map
            # or odometry message.
            evidence = request.get('evidence')
            map_frame = str(request.get('map_frame') or f'{robot}/map')
            odom_frame = str(request.get('odom_frame') or f'{robot}/odom')
            base_frame = str(request.get('base_frame') or
                             self.robot_base_frame(robot))
        live_lookup = request.get('live_lookup') if request else None
        if map_started is not None:
            self._record_sync_map_timing(
                'map_acquisition_reference', time.perf_counter() - map_started)
        map_to_base = (
            live_lookup.get('map_to_base')
            if isinstance(live_lookup, dict) and
            live_lookup.get('map_to_base') is not None else
            self._lookup_sync_tf(map_frame, base_frame, query_ros))
        odom_started = (time.perf_counter()
                        if getattr(self, '_sync_map_profile_enabled', False)
                        else None)
        odom_to_base = (
            live_lookup.get('odom_to_base')
            if isinstance(live_lookup, dict) and
            'odom_to_base' in live_lookup else
            self._lookup_odom_pose(robot, query_ros))
        if odom_started is not None:
            self._record_sync_map_timing(
                'odometry_join', time.perf_counter() - odom_started)
        map_to_odom = (
            live_lookup.get('map_to_odom')
            if isinstance(live_lookup, dict) and
            live_lookup.get('map_to_odom') is not None else
            self._lookup_sync_tf(map_frame, odom_frame, query_ros,
                                 allow_latest_before=True))
        return {
            'schema_version': 'synchronized_map_frame_1.0',
            'sample_kind': sample_kind,
            'query_ros_time_s': float(query_ros),
            'query_wall_elapsed_s': float(query_wall),
            'robot_id': robot,
            'map_frame': map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'supervisor_join': {
                'source': 'forensic/supervisor_ground_truth.csv',
                'join_time_s': float(query_ros),
                'interpolation_required': True,
                'interpolation_method': 'offline_linear_pose_join',
                'interpolation_age_s': None,
                'interpolation_error_m': None,
            },
            # Keep the project's established TF naming convention used by
            # record_scan_correction_at_map_update: map->base, odom->base,
            # and map->odom are queried as (target map/odom, source child).
            # The target/source fields in each nested observation make the
            # tf2 direction explicit for the offline composition.
            'map_to_base': map_to_base,
            'odom_to_base': odom_to_base,
            'map_to_odom': map_to_odom,
            'evidence': evidence,
        }

    def forensic_synchronized_map_frame(self, sample_kind='periodic'):
        """Capture passive ROS/descriptor evidence for offline GT joining."""
        if self.forensic is None or not self.forensic_sync_enabled:
            return
        query_ros = self.ros_seconds()
        query_wall = time.monotonic() - self.start
        if self._defer_synchronized_map_frames:
            for robot in self.robots:
                odom_message = self.latest[robot].get('odom')
                map_message = self.latest[robot].get('map')
                map_frame = (
                    str(map_message.header.frame_id)
                    if map_message is not None and
                    map_message.header.frame_id else f'{robot}/map')
                odom_frame = (
                    str(odom_message.header.frame_id)
                    if odom_message is not None and
                    odom_message.header.frame_id else f'{robot}/odom')
                base_frame = (
                    str(getattr(odom_message, 'child_frame_id', ''))
                    if odom_message is not None else '') or \
                    self.robot_base_frame(robot)
                self._deferred_sync_map_requests.append({
                    'robot_id': robot,
                    'query_ros_time_s': float(query_ros),
                    'query_wall_elapsed_s': float(query_wall),
                    'sample_kind': str(sample_kind),
                    'map_frame': map_frame,
                    'odom_frame': odom_frame,
                    'base_frame': base_frame,
                    'evidence': self.latest_evidence.get(robot),
                    # Preserve the cheap, immediately available tf2/odom
                    # result exactly.  Only the expensive raw fallback graph
                    # is deferred to the post-run reconstruction.
                    'live_lookup': {
                        'map_to_base': self._lookup_sync_tf_direct_only(
                            map_frame, base_frame, query_ros),
                        'odom_to_base': self._lookup_odom_pose(
                            robot, query_ros),
                        'map_to_odom': self._lookup_sync_tf_direct_only(
                            map_frame, odom_frame, query_ros),
                    },
                })
            return
        for robot in self.robots:
            row = self._synchronized_map_frame_row(
                robot, query_ros, query_wall, sample_kind)
            write_started = (time.perf_counter()
                             if getattr(self, '_sync_map_profile_enabled', False)
                             else None)
            self.forensic.record_synchronized_map_frame(finite(row))
            if write_started is not None:
                self._record_sync_map_timing(
                    'serialization_write', time.perf_counter() - write_started)

    def _reconstruct_deferred_synchronized_map_frames(self):
        """Recreate the exact synchronized rows after live simulation.

        The live path only records the timer/evidence request and therefore
        avoids per-sample graph traversal and TF composition while Webots is
        advancing.  The existing bounded TF lookup methods, raw TF index, and
        odometry interpolation are deliberately reused here so row schemas,
        validity decisions, and tie behavior remain unchanged.
        """
        if not self._defer_synchronized_map_frames:
            return
        if self.forensic is None:
            return
        self.flush()
        raw_path = self.directory / 'forensic' / 'raw_tf.csv'
        if not raw_path.is_file():
            raise RuntimeError('deferred synchronized-map reconstruction is '
                               'missing forensic/raw_tf.csv')

        # Live bounded indexes are intentionally small.  Rebuild a complete
        # stable index from the already-preserved raw file for old samples so
        # the post-run computation is not limited by the live cache horizon.
        dynamic = {}
        static = {}
        raw_tf_events = []
        with raw_path.open(newline='', encoding='utf-8') as stream:
            for sequence, row in enumerate(csv.DictReader(stream)):
                try:
                    key = (self._normal_frame(row['parent_frame']),
                           self._normal_frame(row['child_frame']))
                    value = (
                        float(row['translation_x']),
                        float(row['translation_y']),
                        yaw_from_row(float(row['rotation_z']),
                                     float(row['rotation_w'])),
                    )
                    timestamp = float(row['transform_stamp'])
                    if not all(math.isfinite(part)
                               for part in (*value, timestamp)):
                        continue
                    received_wall = float(row['received_wall_elapsed_s'])
                    received_ros = float(row['received_ros_time_s'])
                    if not all(math.isfinite(part)
                               for part in (received_wall, received_ros)):
                        continue
                except (KeyError, TypeError, ValueError):
                    continue
                raw_tf_events.append((received_wall, received_ros, sequence,
                                      str(row.get('static', '')).lower() ==
                                      'true', key, timestamp, value))
                if str(row.get('static', '')).lower() == 'true':
                    static[key] = (timestamp, value)
                else:
                    dynamic.setdefault(key, []).append((timestamp, value))

        # Rebuild the stores empty and replay them in receipt order below.
        # This is important: a post-run query must not see a future TF sample
        # or odometry message that was not available to the live callback.
        raw_tf_events.sort(key=lambda item: (item[0], item[1], item[2]))
        self._deferred_raw_tf_events = raw_tf_events
        self._direct_tf_samples = {
            key: RawTFSeriesIndex(max(1, len(values)))
            for key, values in dynamic.items()}
        self._direct_tf_static = {}
        self._direct_tf_graph_keys = set()
        self._direct_tf_adjacency = {}
        self._direct_tf_adjacency_snapshot = None

        odom_events = []
        self._odom_samples = {}
        for robot in self.robots:
            path = self.directory / 'forensic' / f'{robot}_odom.csv'
            if not path.is_file():
                continue
            with path.open(newline='', encoding='utf-8') as stream:
                for sequence, row in enumerate(csv.DictReader(stream)):
                    try:
                        received_wall = float(row['received_wall_elapsed_s'])
                        received_ros = float(row['received_ros_time_s'])
                        timestamp = float(row['header_stamp'])
                        z = float(row['orientation_z'])
                        w = float(row['orientation_w'])
                        pose = (
                            float(row['pose_x']), float(row['pose_y']),
                            yaw_from_row(z, w), str(row['frame_id']),
                            self.robot_base_frame(robot))
                        if not all(math.isfinite(part) for part in
                                    (received_wall, received_ros,
                                     timestamp, pose[0], pose[1], pose[2])):
                            continue
                    except (KeyError, TypeError, ValueError):
                        continue
                    odom_events.append((received_wall, received_ros,
                                        sequence, robot, timestamp, pose))
            # The rolling index is populated causally as requests are replayed.
            self._odom_samples[robot] = RollingTimestampIndex(
                max(1, sum(1 for event in odom_events
                           if event[3] == robot)))
        odom_events.sort(key=lambda item: (item[0], item[1], item[2]))

        def received_before(event_wall, event_ros, request):
            request_wall = float(request.get('query_wall_elapsed_s', 0.0))
            request_ros = float(request.get('query_ros_time_s', 0.0))
            # Wall elapsed time is the causal ordering clock for callbacks;
            # retain ROS time as a fallback for old artifacts without it.
            if math.isfinite(request_wall) and math.isfinite(event_wall):
                return event_wall <= request_wall
            return event_ros <= request_ros

        indexed_requests = sorted(
            enumerate(self._deferred_sync_map_requests),
            key=lambda item: (float(item[1].get('query_wall_elapsed_s', 0.0)),
                              float(item[1].get('query_ros_time_s', 0.0)),
                              item[0]))
        rows = [None] * len(self._deferred_sync_map_requests)
        raw_index = 0
        odom_index = 0
        for request_index, request in indexed_requests:
            while raw_index < len(self._deferred_raw_tf_events):
                event = self._deferred_raw_tf_events[raw_index]
                if not received_before(event[0], event[1], request):
                    break
                _, _, _, is_static, key, timestamp, value = event
                if is_static:
                    self._direct_tf_static[key] = (timestamp, value)
                else:
                    self._direct_tf_samples[key].append(timestamp, value)
                self._register_direct_tf_key(key)
                raw_index += 1
            while odom_index < len(odom_events):
                event = odom_events[odom_index]
                if not received_before(event[0], event[1], request):
                    break
                _, _, _, robot, timestamp, pose = event
                self._odom_samples[robot].append(timestamp, pose)
                odom_index += 1
            robot = str(request['robot_id'])
            query_ros = float(request['query_ros_time_s'])
            rows[request_index] = self._synchronized_map_frame_row(
                robot, query_ros,
                float(request['query_wall_elapsed_s']),
                str(request['sample_kind']), request=request)

        for row in rows:
            self.forensic.record_synchronized_map_frame(finite(row))


    def evidence_descriptor(self, message):
        """Remember descriptor metadata and snapshot TF at its arrival.

        This callback is observer-only.  It stores no descriptor bytes and
        has no publisher or service client; the metadata is joined to the
        passive synchronized stream solely for post-run analysis.
        """
        robot = str(message.source_robot_id)
        if robot not in self.latest_evidence:
            return
        stamp_value = (int(message.header.stamp.sec) +
                       int(message.header.stamp.nanosec) * 1e-9)
        received_ros = self.ros_seconds()
        self.latest_evidence[robot] = {
            'keyframe_id': str(message.keyframe_id),
            'descriptor_stamp_s': stamp_value,
            'received_ros_time_s': received_ros,
            'evidence_age_s': received_ros - stamp_value,
            'map_epoch': int(message.map_epoch),
            'checksum': int(message.checksum),
            'resolution': float(message.resolution),
            'crop_width': int(message.crop_width),
            'crop_height': int(message.crop_height),
            'crop_origin_x': float(message.crop_origin_x),
            'crop_origin_y': float(message.crop_origin_y),
            'crop_origin_yaw': float(message.crop_origin_yaw),
            'frame_id': str(message.header.frame_id),
        }
        if self.forensic_sync_enabled:
            self.forensic_synchronized_map_frame('evidence_keyframe')

    def subscribe(self):
        for r in self.robots:
            self.observe(Odometry,f'/{r}/odom',lambda m,x=r:self.odom(x,m),qos_profile_sensor_data,f'{r}.odom')
            # Passive controller-health evidence; it never gates or commands
            # the running stack.
            # When the existing rosbag2 path is enabled, retain every raw
            # message in the bag and remove these passive entities from the
            # Python executor.  Finalization reconstructs their existing
            # cadence/age artifacts from the bag and /clock.
            if not self.passive_sensor_offload_enabled:
                self.observe(JointState,f'/{r}/joint_states',lambda m,x=r:self.mark(x,'joint_states',m),qos_profile_sensor_data,f'{r}.joint_states')
                for s in ('scan_d500_fixed','scan_d500_slam','scan_d500_nav'): self.observe(LaserScan,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),qos_profile_sensor_data,f'{r}.{s}')
            for s in ('map','shared_map','local_costmap/costmap','global_costmap/costmap'): self.observe(OccupancyGrid,f'/{r}/{s}',lambda m,x=r,k=s:self.mark(x,k,m),self.qos(True,True,1),f'{r}.{s}')
            self.observe(PeerMap,f'/cslam/{r}/local_map',lambda m,x=r:self.mark(x,'peer_map',m),self.qos(True,True,1),f'{r}.peer_map')
            self.observe(FrontierCandidateArray,f'/{r}/frontier_candidates',lambda m,x=r:self.candidates(x,m),self.qos(True,False,1),f'{r}.candidates')
            self.observe(ExplorationClaim,f'/cslam/{r}/exploration_claim',lambda m,x=r:self.claim(x,m),self.qos(True,False,10),f'{r}.claim')
            self.observe(ExplorationStatus,f'/cslam/{r}/exploration_status',lambda m,x=r:self.status(x,m),self.qos(True,False,10),f'{r}.status')
            self.observe(ExplorationEvent,f'/cslam/{r}/exploration_event',lambda m,x=r:self.coordinator_event(x,m),self.qos(True,False,50),f'{r}.event')
            self.observe(TaskSnapshot,f'/{r}/task_snapshot',lambda m,x=r:self.distributed_snapshot(x,m),self.qos(True,True,1),f'{r}.task_snapshot')
            self.observe(TaskBidArray,f'/{r}/task_bids',lambda m,x=r:self.distributed_bids(x,m),self.qos(True,True,1),f'{r}.task_bids')
            self.observe(PairDecision,f'/{r}/pair_decision',lambda m,x=r:self.distributed_decision(x,m),self.qos(True,True,1),f'{r}.pair_decision')
            self.observe(DistributedExplorationStatus,f'/{r}/distributed_status',lambda m,x=r:self.distributed_status(x,m),self.qos(True,True,1),f'{r}.distributed_status')
            self.observe(DistributedExplorationEvent,f'/{r}/distributed_event',lambda m,x=r:self.distributed_event(x,m),self.qos(True,False,50),f'{r}.distributed_event')
            self.observe(ExplorationFailure,f'/{r}/exploration_failure',lambda m,x=r:self.distributed_failure(x,m),self.qos(True,True,10),f'{r}.exploration_failure')
            if not self.passive_bag_enabled:
                self.observe(NavigateToPose_FeedbackMessage,f'/{r}/navigate_to_pose/_action/feedback',lambda m,x=r:self.feedback(x,m),self.qos(),f'{r}.feedback')
                self.observe(GoalStatusArray,f'/{r}/navigate_to_pose/_action/status',lambda m,x=r:self.mark(x,'navigate_status',m),self.qos(True,True,1),f'{r}.navigate_status')
                self.observe(GoalStatusArray,f'/{r}/follow_path/_action/status',lambda m,x=r:self.action_status(x,'FOLLOW_PATH',m),self.qos(True,True,1),f'{r}.follow_path_status')
                self.observe(GoalStatusArray,f'/{r}/compute_path_to_pose/_action/status',lambda m,x=r:self.action_status(x,'COMPUTE_PATH_TO_POSE',m),self.qos(True,True,1),f'{r}.compute_path_status')
                self.observe(NavPath,f'/{r}/plan',lambda m,x=r:self.plan(x,m),self.qos(),f'{r}.plan')
            self.observe(Twist,f'/{r}/cmd_vel_nav',lambda m,x=r:self.command(x,m,'cmd_vel_nav'),self.qos(),f'{r}.cmd_vel_nav')
            self.observe(TwistStamped,f'/{r}/cmd_vel',lambda m,x=r:self.command(x,m.twist,'cmd_vel'),self.qos(),f'{r}.cmd_vel')
        # Mirror the raw TF channels with their normal ROS QoS.  The listener
        # remains authoritative; these subscriptions only make the passive
        # evaluator able to reconstruct a tightly timestamped local chain
        # when Buffer.lookup_transform cannot compose it during a busy run.
        self.observe(TFMessage, '/tf', self._direct_tf_message,
                     self.qos(True, False, 1000), 'forensic.tf')
        self.observe(TFMessage, '/tf_static', self._direct_tf_static_message,
                     self.qos(True, True, 100), 'forensic.tf_static')
        # One low-rate authoritative startup marker.  It is control-neutral
        # evidence for the common two-robot release barrier.
        self.observe(
            String, '/cslam/unknown_pose/start_release',
            self.start_release, self.qos(True, True, 1),
            'startup.start_release')
        if self.forensic_sync_enabled:
            self.observe(
                LocalMapDescriptor, '/cslam/relative_pose/descriptors',
                self.evidence_descriptor, self.qos(True, False, 100),
                'relative_pose.descriptor')
        if self.p['enable_rosout_collection']: self.observe(Log,'/rosout',self.rosout,self.qos(True,True,1000),'rosout')

    def start_release(self, message):
        """Record the single simulation-time release marker verbatim."""
        try:
            payload = json.loads(str(message.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.event('START_RELEASE_INVALID', 'malformed START_RELEASE payload',
                       severity='ERROR', allow_during_shutdown=True)
            return
        if str(payload.get('event', '')) != 'START_RELEASE':
            return
        self.event(
            'START_RELEASE', 'common two-robot exploration release',
            source='/cslam/unknown_pose/start_release',
            release_sim_time_s=float(payload.get('release_sim_time_s', 0.0)),
            traffic_scheduler_ready=bool(
                payload.get('traffic_scheduler_ready', False)),
            accepted_handoff=bool(payload.get('accepted_handoff', False)),
            shared_nav2_ready=bool(payload.get('shared_nav2_ready', False)),
            ready_robots=payload.get('ready_robots', []),
            allow_during_shutdown=True,
        )
    def ros_seconds(self): return self.get_clock().now().nanoseconds*1e-9
    def ros_now(self): n=self.get_clock().now().nanoseconds; return n//1000000000,n%1000000000
    def common(self,source='/cooperative_experiment_logger',robot=None,source_stamp=None):
        with self._state_lock:
            self.sequence+=1; sequence=self.sequence
        sec,nsec=source_stamp or self.ros_now(); return {'schema_version':SCHEMA,'run_id':self.run_id,'event_sequence':sequence,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':self.ros_seconds()-self.start_ros,'wall_elapsed_s':time.monotonic()-self.start,'robot_id':robot,'source':source}
    def event(self,event_type,message,robot=None,source='/cooperative_experiment_logger',severity='INFO',source_stamp=None,console=False,allow_during_shutdown=False,write_goal_ledger=True,count_event=True,**extra):
        if (self._finalizing or self._closed) and not allow_during_shutdown:return None
        row=self.common(source,robot,source_stamp); row.update(severity=severity,event_type=event_type,message=message); row.update(finite(extra))
        if count_event:
            with self._state_lock:self.counts[event_type]+=1
        self._account_goal_event(row)
        try:
            encoded=json.dumps(finite(row),separators=(',',':'),allow_nan=False)+'\n'
            with self._io_lock:
                if self._closed:return None
                self.events.write(encoded)
                if write_goal_ledger and event_type in GOAL_LEDGER_EVENTS:
                    ledger = dict(row)
                    ledger['decision_stage'] = event_type
                    ledger['observed_navigation_active'] = bool(
                        robot is not None and
                        self.latest.get(robot, {}).get(
                            'navigation_active', False))
                    self.goal_decisions.write(
                        json.dumps(finite(ledger), separators=(',', ':'),
                                   allow_nan=False) + '\n')
        except (OSError,TypeError,ValueError) as exc:
            self.write_failures+=1; self.get_logger().error(f'event write failed: {exc}',throttle_duration_sec=10.)
        if console or severity=='ERROR' or event_type.endswith(('_STARTED','_CLEARED')):
            text=f"[{row['elapsed_s']:.1f}s][{robot or '-'}] {event_type} {message}"
            # Separate fixed call sites are required by rclpy's severity-per-call-site cache.
            if severity=='ERROR':self.get_logger().error(text)
            elif severity=='WARN':self.get_logger().warning(text)
            else:self.get_logger().info(text)
        return row
    @staticmethod
    def _goal_key(row):
        return (row.get('robot_id'), row.get('physical_task_signature') or '',
                row.get('canonical_task_id') or '', row.get('round_id') or '')
    def _account_goal_event(self,row):
        """Maintain one explicit lifecycle record per dispatched goal."""
        kind=row.get('event_type'); robot=row.get('robot_id')
        if kind == 'NAV_GOAL_SENT':
            self.dispatch_attempts += 1
            key=self._goal_key(row)
            self.goal_accounting.append({
                'dispatch_event_sequence': row['event_sequence'],
                'robot_id': robot,
                'round_id': row.get('round_id'),
                'canonical_task_id': row.get('canonical_task_id'),
                'physical_task_signature': row.get('physical_task_signature'),
                'sent_elapsed_s': row.get('elapsed_s'),
                'accepted': key in self._accepted_before_send,
                'terminal_category': None,
                'terminal_event_sequence': None,
            })
            self._accepted_before_send.discard(key)
            return
        if kind == 'NAV_GOAL_ACCEPTED':
            candidates=[item for item in reversed(self.goal_accounting)
                        if item['robot_id']==robot and
                        item['terminal_category'] is None]
            if candidates:
                candidates[0]['accepted']=True
            else:
                self._accepted_before_send.add(self._goal_key(row))
            return
        terminal={
            'NAVIGATION_SUCCEEDED':'SUCCEEDED',
            'NAVIGATION_FAILED':'NAV2_FAILED',
            'NAVIGATION_TIMEOUT':'CONTROLLER_TIMEOUT',
            'NAVIGATION_CANCELED':'CANCELLED',
            'NAVIGATION_CANCELLED':'CANCELLED',
        }.get(kind)
        if terminal is None:
            return
        candidates=[item for item in reversed(self.goal_accounting)
                    if item['robot_id']==robot and
                    item['terminal_category'] is None]
        if not candidates:
            return
        item=candidates[0]
        item['terminal_category']=terminal
        item['terminal_event_sequence']=row['event_sequence']
        item['terminal_elapsed_s']=row.get('elapsed_s')
        item['failure_class']=row.get('failure_class')
        self.goals_terminal += 1
    def goal_accounting_summary(self):
        records=[dict(item) for item in self.goal_accounting]
        for item in records:
            if item['terminal_category'] is None:
                active=self.latest.get(item['robot_id'],{}).get(
                    'navigation_active',False)
                item['terminal_category'] = (
                    'STILL_ACTIVE_AT_MISSION_END' if active else
                    'UNKNOWN_OR_UNACCOUNTED')
        categories=Counter(item['terminal_category'] for item in records)
        by_robot={}
        for robot in self.robots:
            own=[item for item in records if item['robot_id']==robot]
            by_robot[robot]={
                'dispatched':len(own),
                'terminal':sum(item['terminal_category'] not in
                               ('STILL_ACTIVE_AT_MISSION_END',
                                'UNKNOWN_OR_UNACCOUNTED') for item in own),
                'active_at_mission_end':sum(item['terminal_category']==
                                            'STILL_ACTIVE_AT_MISSION_END'
                                            for item in own),
                'unknown_or_unaccounted':sum(item['terminal_category']==
                                             'UNKNOWN_OR_UNACCOUNTED'
                                             for item in own),
                'categories':dict(Counter(item['terminal_category']
                                           for item in own)),
            }
        return {'dispatched':len(records),'terminal':self.goals_terminal,
                'categories':dict(categories),'by_robot':by_robot,
                'records':records}
    def safe_call(self,subsystem,operation,*args):
        if self._finalizing or self._closed:
            self.dropped_samples+=1; return None
        started = time.perf_counter() if self._callback_timing_enabled else None
        try:return operation(*args)
        except Exception as exc:
            self.record_internal_error(subsystem,exc); return None
        finally:
            if started is not None:
                elapsed = max(0.0, time.perf_counter() - started)
                with self._state_lock:
                    entry = self._callback_timing.setdefault(
                        str(subsystem),
                        {'calls': 0, 'total_wall_s': 0.0, 'max_wall_s': 0.0},
                    )
                    entry['calls'] += 1
                    entry['total_wall_s'] += elapsed
                    entry['max_wall_s'] = max(entry['max_wall_s'], elapsed)
    def record_internal_error(self,subsystem,exc):
        key=f'{subsystem}:{type(exc).__name__}:{exc}'
        with self._state_lock:
            self.internal_errors[key]+=1; occurrence=self.internal_errors[key]
        if self._reporting_internal_error:return
        self._reporting_internal_error=True
        try:
            if occurrence==1 or occurrence%100==0:
                self.event('LOGGER_INTERNAL_ERROR',f'{subsystem}: {type(exc).__name__}: {exc}',severity='ERROR',subsystem=subsystem,occurrence_count=occurrence)
        except Exception:
            self.get_logger().error(f'logger internal error reporting failed in {subsystem}',throttle_duration_sec=10.)
        finally:self._reporting_internal_error=False
    def _record_map_receipt(self, robot, key, message, received_ros,
                            received_wall):
        """Record only causal map identity metadata for exact replay joins."""
        if self.map_receipt_file is None:
            return
        info = message.info
        data = np.asarray(message.data, dtype=np.int8)
        stamp_sec, stamp_nanosec = stamp(message)
        with self._io_lock:
            self._map_receipt_sequence += 1
            row = {
                'sequence': self._map_receipt_sequence,
                'robot_id': str(robot),
                'map_key': str(key),
                'received_ros_time_s': float(received_ros),
                'received_wall_elapsed_s': float(received_wall),
                'header_stamp_s': float(stamp_sec) + float(stamp_nanosec) * 1e-9,
                'width': int(info.width),
                'height': int(info.height),
                'resolution': float(info.resolution),
                'origin_x': float(info.origin.position.x),
                'origin_y': float(info.origin.position.y),
                'origin_yaw': float(yaw(info.origin.orientation)),
                'data_sha256': hashlib.sha256(data.tobytes()).hexdigest(),
            }
            self.map_receipt_file.write(
                json.dumps(finite(row), separators=(',', ':')) + '\n')
    def mark(self,r,key,msg):
        now=self.ros_seconds()
        with self._state_lock:
            self.last[(r,key)]=now; self.windows.setdefault((r,key),deque()).append(now); self.latest[r][key]=msg
            if key == 'shared_map':
                self.shared_map_seen.add(r)
            if key == 'scan_d500_fixed':
                self.scan_pipeline[r]['scan_d500_fixed_stamps'].append(
                    stamp(msg)[0] + stamp(msg)[1] * 1e-9)
            elif key == 'scan_d500_nav':
                source_stamp = stamp(msg)[0] + stamp(msg)[1] * 1e-9
                self.scan_pipeline[r]['scan_d500_nav_stamps'].append(
                    source_stamp)
                self.scan_pipeline[r]['scan_d500_nav_ages_s'].append(
                    max(0.0, now - source_stamp))
            elif key == 'map':
                self.scan_pipeline[r]['map_stamps'].append(
                    stamp(msg)[0] + stamp(msg)[1] * 1e-9)
        if self.forensic is not None and key == 'peer_map':
            self.forensic.record_peer_map(
                r, msg, now, time.monotonic() - self.start)
        if key in ('map', 'shared_map'):
            self._record_map_receipt(
                r, key, msg, now, time.monotonic() - self.start)
        if self.forensic is not None and self.scan_matching_enabled and key == 'map':
            self.record_scan_correction_at_map_update(r, now)

    def record_scan_correction_at_map_update(self, robot, now_ros):
        """Record passive scan-match correction evidence at each local map update."""
        now_wall = time.monotonic() - self.start
        self.scan_pipeline[robot]['scan_correction_records'] += 1
        try:
            transform = self.tf_buffer.lookup_transform(
                f'{robot}/map', f'{robot}/odom', Time(),
                timeout=Duration(seconds=0.03))
            self.forensic.record_scan_correction(
                robot, now_ros, now_wall, self.latest[robot].get('odom'),
                map_to_odom=transform)
        except TransformException as exc:
            self.forensic.record_scan_correction(
                robot, now_ros, now_wall, self.latest[robot].get('odom'),
                error=str(exc))
    def age(self,r,key):
        value=self.last.get((r,key)); return self.ros_seconds()-value if value else None
    def odom(self,r,msg):
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        self.mark(r,'odom',msg)
        p=msg.pose.pose.position
        local_yaw=yaw(msg.pose.pose.orientation)
        # Local odometry is the reliable pre-handoff motion source.  The
        # shared_map transform is intentionally unavailable before handoff;
        # do not let that optional lookup erase the observer's local pose,
        # velocity, or travelled-distance accounting.
        self.latest[r]['pose']=(float(p.x),float(p.y),float(local_yaw))
        self.latest[r]['pose_frame']=str(msg.header.frame_id)
        self.latest[r]['speed']=(float(msg.twist.twist.linear.x),
                                 float(msg.twist.twist.angular.z))
        stamp_value = stamp(msg)[0] + stamp(msg)[1] * 1e-9
        if started is not None:
            self._record_high_rate_timing(
                f'{r}.odom.live_state', time.perf_counter() - started)
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        self._odom_samples[r].append(
            stamp_value,
            (float(p.x), float(p.y), float(local_yaw),
             str(msg.header.frame_id or f'{r}/odom'),
             str(getattr(msg, 'child_frame_id', '') or self.robot_base_frame(r))))
        if started is not None:
            self._record_high_rate_timing(
                f'{r}.odom.index_maintenance', time.perf_counter() - started)
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        self.local_trajectory.add(r,float(p.x),float(p.y))
        if started is not None:
            self._record_high_rate_timing(
                f'{r}.odom.live_motion_state', time.perf_counter() - started)
        if self.forensic is not None:
            started = (time.perf_counter()
                       if getattr(self, '_high_rate_profile_enabled', False)
                       else None)
            self.forensic.record_odom(
                r, msg, self.ros_seconds(), time.monotonic() - self.start)
            if started is not None:
                self._record_high_rate_timing(
                    f'{r}.odom.csv_serialization', time.perf_counter() - started)
        # The raw /tf callback receives the same authoritative transform
        # samples as tf2.  When an exact direct sample is already indexed,
        # use its planar projection for this passive trajectory evidence and
        # avoid a second tf2 wait-set lookup.  Any missing, interpolated, or
        # chained case retains the original tf2 path below.
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        direct = self._direct_tf_edge(
            self.p['global_frame'], msg.header.frame_id, stamp_value)
        if started is not None:
            self._record_high_rate_timing(
                f'{r}.odom.direct_tf_lookup', time.perf_counter() - started)
        if (direct is not None and
                direct[1].get('lookup_mode') == 'raw_tf_exact'):
            started = (time.perf_counter()
                       if getattr(self, '_high_rate_profile_enabled', False)
                       else None)
            transform_xyyaw = direct[0]
            t_x, t_y, heading = transform_xyyaw
            cosine, sine = math.cos(heading), math.sin(heading)
            shared_x = t_x + cosine * p.x - sine * p.y
            shared_y = t_y + sine * p.x + cosine * p.y
            shared_yaw = (heading + local_yaw + math.pi) % (2 * math.pi) - math.pi
            self.latest[r]['shared_pose'] = (shared_x, shared_y, shared_yaw)
            self.latest[r]['shared_pose_frame'] = self.p['global_frame']
            if self.p['enable_trajectory_overlap']:
                self.trajectory.add(r, shared_x, shared_y)
            if started is not None:
                self._record_high_rate_timing(
                    f'{r}.odom.shared_live_state', time.perf_counter() - started)
            return
        try:
            # This is passive evidence collection on the high-rate odometry
            # path.  Waiting up to 50 ms here can stall the logger executor
            # for every odometry message and therefore slow Webots/ROS time.
            # Preserve the same transform when it is immediately available;
            # the raw /tf stream is retained for offline cross-frame joins.
            started = (time.perf_counter()
                       if getattr(self, '_high_rate_profile_enabled', False)
                       else None)
            transform=self.tf_buffer.lookup_transform(
                self.p['global_frame'], msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.0))
            if started is not None:
                self._record_high_rate_timing(
                    f'{r}.odom.tf2_lookup', time.perf_counter() - started)
            t=transform.transform.translation
            heading=yaw(transform.transform.rotation)
            cosine, sine=math.cos(heading), math.sin(heading)
            shared_x=t.x+cosine*p.x-sine*p.y
            shared_y=t.y+sine*p.x+cosine*p.y
            shared_yaw=(heading+local_yaw+math.pi)%(2*math.pi)-math.pi
        except TransformException:
            if started is not None:
                self._record_high_rate_timing(
                    f'{r}.odom.tf2_lookup', time.perf_counter() - started)
            # Do not feed local-frame points to cross-robot metrics.
            return
        started = (time.perf_counter()
                   if getattr(self, '_high_rate_profile_enabled', False)
                   else None)
        self.latest[r]['shared_pose']=(shared_x,shared_y,shared_yaw)
        self.latest[r]['shared_pose_frame']=self.p['global_frame']
        if self.p['enable_trajectory_overlap']: self.trajectory.add(r,shared_x,shared_y)
        if started is not None:
            self._record_high_rate_timing(
                f'{r}.odom.shared_live_state', time.perf_counter() - started)
    def command(self,r,msg,source='cmd_vel'):
        self.mark(r,'cmd_vel',msg)
        now=self.ros_seconds(); linear=float(msg.linear.x); angular=float(msg.angular.z)
        self.latest[r].update(command=(linear,angular),cmd_vel_received_ros_s=now,cmd_vel_source=source)
        zero=(abs(linear)<=float(self.p['cmd_vel_zero_linear_epsilon_mps']) and
              abs(angular)<=float(self.p['cmd_vel_zero_angular_epsilon_radps']))
        previous=self.latest[r].get('cmd_vel_effectively_zero')
        if previous is not None and previous != zero:
            self.event('CMD_VEL_ZERO_CLEARED' if not zero else 'CMD_VEL_ZERO_STARTED',
                       'effective command state changed',r,f'/{r}/{source}',
                       linear_mps=linear,angular_radps=angular,
                       zero_linear_epsilon_mps=self.p['cmd_vel_zero_linear_epsilon_mps'],
                       zero_angular_epsilon_radps=self.p['cmd_vel_zero_angular_epsilon_radps'])
        self.latest[r]['cmd_vel_effectively_zero']=zero
    def plan(self,r,msg): self.mark(r,'plan',msg)
    def action_status(self,r,action,msg):
        self.mark(r,action.lower(),msg)
        names={0:'UNKNOWN',1:'ACCEPTED',2:'EXECUTING',3:'CANCELING',4:'SUCCEEDED',5:'CANCELED',6:'ABORTED'}
        for status in msg.status_list:
            goal_id=bytes(status.goal_info.goal_id.uuid).hex()
            value=int(status.status); key=(r,action,goal_id); previous=self.action_goal_states.get(key)
            if previous==value: continue
            self.action_goal_states[key]=value
            source=f'/{r}/{action.lower()}/_action/status'
            status_name=names.get(value,str(value))
            fields=dict(action=action, goal_uuid=goal_id,
                        status_value=value, status_name=status_name)
            self.event(f'{action}_STATUS', 'action goal status changed', r,
                       source, source_stamp=stamp(msg), **fields)
            if action != 'FOLLOW_PATH':
                continue
            # FollowPath is the physical solo allocator's navigation action.
            # Convert its passive status stream into the same goal ledger
            # lifecycle used by NavigateToPose/legacy claim reporting.
            lifecycle_key=(r, goal_id)
            if value in (1, 2):
                self.latest[r]['navigation_active']=True
                if lifecycle_key not in self.follow_path_goal_lifecycle:
                    self.follow_path_goal_lifecycle.add(lifecycle_key)
                    self.event('NAV_GOAL_SENT',
                               'FollowPath goal observed', r, source,
                               source_stamp=stamp(msg), **fields)
                    self.event('NAV_GOAL_ACCEPTED',
                               'FollowPath goal accepted', r, source,
                               source_stamp=stamp(msg), **fields)
            elif value in (4, 5, 6):
                self.latest[r]['navigation_active']=False
                terminal_kind={4:'NAVIGATION_SUCCEEDED',
                               5:'NAVIGATION_CANCELED',
                               6:'NAVIGATION_FAILED'}[value]
                self.follow_path_terminal_pending[r] += 1
                self.event(terminal_kind,
                           f'FollowPath terminal status: {status_name}',
                           r, source, source_stamp=stamp(msg),
                           severity='ERROR' if value == 6 else 'INFO',
                           failure_class='CONTROLLER_FAILURE' if value == 6 else None,
                           **fields)
    def candidates(self,r,msg):
        old=self.latest[r].get('candidate_count'); count=len(msg.candidates); self.mark(r,'frontier_candidates',msg); self.latest[r]['candidate_count']=count
        # Keep the observer passive, but retain the bounded candidate evidence
        # needed to audit one distributed-assignment round after a live run.
        candidates=[{'frontier_id':item.frontier_id,
                     'centroid':[item.centroid.x,item.centroid.y],
                     'bounds':[item.bounding_box_min.x,item.bounding_box_min.y,
                               item.bounding_box_max.x,item.bounding_box_max.y],
                     'approach':[item.approach_pose.pose.position.x,
                                 item.approach_pose.pose.position.y],
                     'visible_reveal_gain':item.information_gain,
                     'score':item.score,
                     'reachability_state':item.reachability_state,
                     'path_length_m':item.path_length_m,
                     'local_path_length_m':item.local_path_length_m,
                     # FrontierCandidate carries the generator's heading
                     # primitive as heading_change_rad.  The normalized
                     # task/bid representation uses path_heading_cost_rad;
                     # keep this passive observer compatible with both
                     # message generations without changing scoring.
                     'path_heading_cost_rad':getattr(
                         item, 'path_heading_cost_rad',
                         getattr(item, 'heading_change_rad', 0.0)),
                     'local_path_samples':len(item.local_path_samples)}
                    for item in msg.candidates]
        self.frontier_metadata[r] = {
            int(item['frontier_id']): item for item in candidates}
        self.latest[r]['candidate_frame'] = str(
            msg.header.frame_id or self.p['global_frame'])
        self.event('CANDIDATE_BATCH_RECEIVED',f'{count} reachable candidates',r,f'/{r}/frontier_candidates',source_stamp=stamp(msg),map_revision=msg.map_revision,candidate_count=count,detected_frontier_count=msg.detected_frontier_count,detected_not_queried_count=getattr(msg,'detected_not_queried_count',msg.unclassified_frontier_count),small_frontier_count=msg.small_frontier_count,out_of_range_frontier_count=msg.out_of_range_frontier_count,unreachable_frontier_count=msg.unreachable_frontier_count,planner_failure_count=msg.planner_failure_count,unclassified_frontier_count=msg.unclassified_frontier_count,candidates=candidates)
        diagnostic_regions = getattr(msg, 'diagnostic_regions_json', '')
        if diagnostic_regions:
            if self.frontier_regions_file is None:
                self.frontier_regions_file = open(
                    self.directory / 'frontier_regions.jsonl', 'a',
                    encoding='utf-8', buffering=1)
                self.files.append(self.frontier_regions_file)
            try:
                payload = json.loads(diagnostic_regions)
                payload.update({'capture_robot': r,
                                'capture_stamp': stamp(msg),
                                'capture_elapsed_s': self.ros_seconds() - self.start_ros})
                self.frontier_regions_file.write(
                    json.dumps(finite(payload), separators=(',', ':')) + '\n')
            except (TypeError, ValueError, OSError) as exc:
                self.record_internal_error('frontier_region_capture', exc)
        if old is not None and old!=count:self.event('CANDIDATE_COUNT_CHANGED',f'{old} -> {count}',r)
        if not count:self.event('NO_REACHABLE_CANDIDATES','candidate batch empty',r)
    @staticmethod
    def uuid_text(value): return bytes(value.uuid).hex()
    def distributed_changed(self,key,value):
        previous=self.distributed_last.get(key); self.distributed_last[key]=value; return previous!=value
    def distributed_snapshot(self,r,msg):
        self.mark(r,'task_snapshot',msg)
        if not self.distributed_changed((r,'snapshot'),(self.uuid_text(msg.source_session_id),msg.source_snapshot_epoch,msg.source_map_revision,msg.source_map_fingerprint)):return
        tasks=[{'physical_signature':task.physical_signature,'local_frontier_id':task.local_frontier_id,'centroid':[task.centroid.x,task.centroid.y],'bounds':[task.bounding_box_min.x,task.bounding_box_min.y,task.bounding_box_max.x,task.bounding_box_max.y],'approach':[task.approach_pose.pose.position.x,task.approach_pose.pose.position.y],'visible_reveal_gain':task.visible_reveal_gain,'local_ordering_score':task.local_ordering_score,'local_path_valid':task.local_path_valid,'local_path_length_m':task.local_path_length_m,'path_heading_cost_rad':task.path_heading_cost_rad,'local_path_samples':len(task.local_path_samples),'frontier_geometry_samples':len(task.frontier_geometry),'visible_cell_samples':len(task.visible_cells)} for task in msg.tasks]
        self.event('DISTRIBUTED_TASK_SNAPSHOT',f'{len(tasks)} bounded physical tasks',r,f'/{r}/task_snapshot',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),snapshot_epoch=msg.source_snapshot_epoch,source_map_revision=msg.source_map_revision,source_map_fingerprint=msg.source_map_fingerprint,tasks=tasks)
    def distributed_bids(self,r,msg):
        self.mark(r,'task_bids',msg)
        fingerprint=(msg.round_id,msg.union_hash,msg.source_snapshot_epoch,tuple((bid.canonical_task_id,bid.path_valid,round(bid.path_length_m,4)) for bid in msg.bids))
        if not self.distributed_changed((r,'bids'),fingerprint):return
        bids=[{'canonical_task_id':bid.canonical_task_id,'path_valid':bid.path_valid,'path_length_m':bid.path_length_m,'estimated_travel_cost':bid.estimated_travel_cost,'heading_cost':bid.heading_cost,'own_utility_contribution':bid.own_utility_contribution,'path_samples':[[point.x,point.y] for point in bid.path_samples]} for bid in msg.bids]
        self.event('DISTRIBUTED_BID_ARRAY',f'{len(bids)} bounded local bids',r,f'/{r}/task_bids',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,union_hash=msg.union_hash,source_snapshot_epoch=msg.source_snapshot_epoch,bids=bids)
    def distributed_decision(self,r,msg):
        self.mark(r,'pair_decision',msg)
        if not self.distributed_changed((r,'decision'),(msg.round_id,msg.union_hash,msg.decision_hash)):return
        outcome, diagnostics = pair_decision_outcome(msg)
        deferred_pair_enabled = (
            self.passive_bag_enabled and
            bool(self.p.get('enable_scientific_raw_capture', False)))
        if not deferred_pair_enabled:
            self.round_outcomes[outcome] += 1
        self.event('DISTRIBUTED_PAIR_DECISION','replicated complete pair decision',r,f'/{r}/pair_decision',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,union_hash=msg.union_hash,robot1_snapshot_epoch=msg.robot1_snapshot_epoch,robot2_snapshot_epoch=msg.robot2_snapshot_epoch,robot1_bid_fingerprint=msg.robot1_bid_fingerprint,robot2_bid_fingerprint=msg.robot2_bid_fingerprint,robot1_task=msg.robot1_canonical_task_id or 'IDLE',robot2_task=msg.robot2_canonical_task_id or 'IDLE',decision_hash=msg.decision_hash,total_team_score=msg.total_team_score,team_visible_gain=msg.team_visible_gain,combined_path_cost=msg.combined_path_cost,nearby_goal_penalty=msg.nearby_goal_penalty,route_overlap_penalty=msg.route_overlap_penalty,hard_failure_penalty=msg.hard_failure_penalty,sensing_overlap_penalty=msg.sensing_overlap_penalty,workload_imbalance_penalty=msg.workload_imbalance_penalty,coordinator_state=msg.coordinator_state,decision_diagnostics_json=msg.diagnostics_json,decision_outcome=outcome,decision_idle_reason=diagnostics.get('idle_reason'),decision_availability_reason=diagnostics.get('availability_reason'))
    def distributed_status(self,r,msg):
        self.mark(r,'distributed_status',msg)
        self.latest[r]['distributed_state']=msg.state
        distributed_states = {
            DistributedExplorationStatus.WAITING_FOR_INPUTS: 'WAITING_FOR_INPUTS',
            DistributedExplorationStatus.BIDDING: 'BIDDING',
            DistributedExplorationStatus.WAITING_FOR_MATCHING_DECISION: 'WAITING_FOR_MATCHING_DECISION',
            DistributedExplorationStatus.WAITING_FOR_TRAFFIC: 'WAITING_FOR_TRAFFIC',
            DistributedExplorationStatus.NAVIGATING: 'NAVIGATING',
            DistributedExplorationStatus.DEGRADED_SOLO: 'DEGRADED_SOLO',
            DistributedExplorationStatus.COMPLETE: 'COMPLETE',
            DistributedExplorationStatus.BLOCKED: 'BLOCKED',
        }
        self.latest[r]['claim_state'] = distributed_states.get(
            msg.state, str(msg.state),
        )
        self.latest[r]['navigation_active'] = bool(msg.local_nav_goal_active)
        self.latest[r].update(
            terminal=bool(msg.terminal),
            terminal_reason=msg.terminal_reason,
            terminal_epoch=int(msg.terminal_epoch),
            terminal_map_revision=int(msg.terminal_map_revision),
            remaining_frontier_count=int(msg.remaining_frontier_count),
            remaining_small_frontier_count=int(msg.remaining_small_frontier_count),
            remaining_out_of_range_count=int(msg.remaining_out_of_range_count),
            remaining_unreachable_count=int(msg.remaining_unreachable_count),
            planner_failure_count=int(msg.planner_failure_count),
            detected_not_queried_count=int(
                getattr(msg, 'detected_not_queried_count', 0)),
            below_minimum_gain_count=int(
                getattr(msg, 'below_minimum_gain_count', 0)),
            actionable_reachable_count=int(
                getattr(msg, 'actionable_reachable_count', 0)),
        )
        health=(msg.nav2_healthy,msg.tf_healthy,msg.candidate_source_healthy,msg.peer_communication_healthy)
        if not self.distributed_changed((r,'status'),(msg.state,msg.round_id,msg.decision_hash,msg.active_canonical_task_id,health,msg.reason)):return
        self.event('DISTRIBUTED_STATUS',msg.reason,r,f'/{r}/distributed_status',source_stamp=stamp(msg),source_session_id=self.uuid_text(msg.source_session_id),state=msg.state,round_id=msg.round_id,union_hash=msg.union_hash,decision_hash=msg.decision_hash,active_canonical_task_id=msg.active_canonical_task_id,local_nav_goal_active=msg.local_nav_goal_active,nav2_healthy=msg.nav2_healthy,tf_healthy=msg.tf_healthy,candidate_source_healthy=msg.candidate_source_healthy,peer_communication_healthy=msg.peer_communication_healthy,terminal=msg.terminal,terminal_reason=msg.terminal_reason,terminal_epoch=msg.terminal_epoch,remaining_frontier_count=msg.remaining_frontier_count,remaining_small_frontier_count=msg.remaining_small_frontier_count,remaining_out_of_range_count=msg.remaining_out_of_range_count,remaining_unreachable_count=msg.remaining_unreachable_count,planner_failure_count=msg.planner_failure_count,detected_not_queried_count=getattr(msg,'detected_not_queried_count',0),below_minimum_gain_count=getattr(msg,'below_minimum_gain_count',0),actionable_reachable_count=getattr(msg,'actionable_reachable_count',0))
        if msg.terminal:
            self.mission_terminal_reason = msg.terminal_reason or msg.reason
        if msg.state == DistributedExplorationStatus.COMPLETE and self.mission_completion_time is None:
            self.mission_completion_time = self.ros_seconds() - self.start_ros
    def distributed_event(self,r,msg):
        self.mark(r,'distributed_event',msg)
        deferred_agreement_enabled = (
            self.passive_bag_enabled and
            bool(self.p.get('enable_scientific_raw_capture', False)))
        if msg.event_type == 'DECISION_AGREED' and not deferred_agreement_enabled:
            self.agreement_publications += 1
            self.unique_agreed_rounds.add(msg.round_id)
            if msg.decision_hash:
                self.unique_agreed_decisions.add(msg.decision_hash)
        fields = dict(source_session_id=self.uuid_text(msg.source_session_id),
                      round_id=msg.round_id, union_hash=msg.union_hash,
                      decision_hash=msg.decision_hash,
                      canonical_task_id=msg.canonical_task_id,
                      physical_task_signature=msg.physical_task_signature,
                      previous_state=msg.previous_state,
                      next_state=msg.next_state,
                      path_length_m=msg.path_length_m,
                      travelled_distance_m=msg.travelled_distance_m,
                      navigation_duration_s=msg.navigation_duration_s,
                      newly_discovered_cells=msg.newly_discovered_cells,
                      peer_first_discovered_cells=msg.peer_first_discovered_cells,
                      duplicated_cells=msg.duplicated_cells,
                      route_overlap_score=msg.route_overlap_score,
                      sensing_overlap_estimate=msg.sensing_overlap_estimate,
                      result=msg.result, failure_class=msg.failure_class,
                      recoveries=msg.recoveries,
                      nav2_error_code=msg.nav2_error_code,
                      nav2_error_message=msg.nav2_error_message)
        source = f'/{r}/distributed_event'
        duplicate_follow_path_terminal = (
            msg.event_type in {
                'NAVIGATION_SUCCEEDED', 'NAVIGATION_FAILED',
                'NAVIGATION_CANCELED', 'NAVIGATION_CANCELLED',
                'NAVIGATION_TIMEOUT',
            } and self.follow_path_terminal_pending.get(r, 0) > 0)
        if duplicate_follow_path_terminal:
            self.follow_path_terminal_pending[r] -= 1
        self.event(msg.event_type, msg.reason, r, source,
                   source_stamp=stamp(msg),
                   write_goal_ledger=not duplicate_follow_path_terminal,
                   count_event=not duplicate_follow_path_terminal,
                   **fields)
        # The replicated executor reports the local action acceptance as a
        # state-transition event rather than using the legacy claim topic.
        # Normalize that protocol event so navigation telemetry retains the
        # accepted-goal count used by the existing report schema.
        if (msg.event_type == 'STATE_TRANSITION' and
                msg.reason == 'local agreed goal accepted for send'):
            self.event('NAV_GOAL_ACCEPTED', msg.reason, r, source,
                       source_stamp=stamp(msg), **fields)
    def distributed_failure(self,r,msg):
        self.mark(r,'exploration_failure',msg)
        self.event('DISTRIBUTED_TASK_FAILURE',msg.evidence,r,f'/{r}/exploration_failure',source_stamp=stamp(msg),severity='WARN',source_session_id=self.uuid_text(msg.source_session_id),round_id=msg.round_id,canonical_task_id=msg.canonical_task_id,physical_task_signature=msg.physical_task_signature,approach=[msg.approach_pose.pose.position.x,msg.approach_pose.pose.position.y],failure_class=msg.failure_class,path_length_m=msg.path_length_m,path_samples=[[point.x,point.y] for point in msg.path_samples],retry_count=msg.retry_count,nav2_error_code=msg.nav2_error_code,nav2_error_message=msg.nav2_error_message)
    def claim_fields(self,msg):
        return {'source_session_id':bytes(msg.source_session_id.uuid).hex(),'message_revision':msg.message_revision,'claim_id':msg.claim_id,'frontier_id':msg.frontier_id,'map_revision':msg.map_revision,'claim_state':STATES.get(msg.state,str(msg.state)),'frontier_centroid_x':msg.frontier_centroid.x,'frontier_centroid_y':msg.frontier_centroid.y,'approach_x':msg.approach_pose.pose.position.x,'approach_y':msg.approach_pose.pose.position.y,'approach_yaw':yaw(msg.approach_pose.pose.orientation),'path_length_m':msg.path_length_m,'information_gain':msg.information_gain,'utility_score':msg.utility_score,'release_reason':msg.state_reason}
    def claim(self,r,msg):
        self.mark(r,'exploration_claim',msg); old=self.claims.get(r); self.claims[r]=msg
        if old is not None and (old.state,old.claim_id,old.frontier_id,bytes(old.source_session_id.uuid))==(msg.state,msg.claim_id,msg.frontier_id,bytes(msg.source_session_id.uuid)):
            return
        state=STATES.get(msg.state,str(msg.state)); d=self.latest[r]; d.update(claim_state=state,claim_id=msg.claim_id,frontier_id=msg.frontier_id,goal=(msg.approach_pose.pose.position.x,msg.approach_pose.pose.position.y,yaw(msg.approach_pose.pose.orientation))); fields=self.claim_fields(msg); source=f'/cslam/{r}/exploration_claim'
        if old and bytes(old.source_session_id.uuid)!=bytes(msg.source_session_id.uuid):self.event('PEER_SESSION_CHANGED','source session changed',r,source,**fields)
        if msg.state==ExplorationClaim.PROPOSING:
            self.event('CLAIM_PROPOSED',msg.state_reason or 'frontier proposed',r,source,console=True,**fields)
            peer=next((p for name,p in self.claims.items() if name!=r and p.state in (ExplorationClaim.PROPOSING,ExplorationClaim.NAVIGATING)),None)
            if peer:
                a={'centroid_x':msg.frontier_centroid.x,'centroid_y':msg.frontier_centroid.y,'min_x':msg.bounding_box_min.x,'max_x':msg.bounding_box_max.x,'min_y':msg.bounding_box_min.y,'max_y':msg.bounding_box_max.y}; b={'centroid_x':peer.frontier_centroid.x,'centroid_y':peer.frontier_centroid.y,'min_x':peer.bounding_box_min.x,'max_x':peer.bounding_box_max.x,'min_y':peer.bounding_box_min.y,'max_y':peer.bounding_box_max.y}
                if equivalent_frontiers(a,b):
                    self.event('EQUIVALENT_FRONTIER_DUPLICATE','production-equivalent simultaneous claims',r,source,peer_robot_id=peer.source_robot_id); self.event('CLAIM_CONFLICT_DETECTED','equivalent active peer claim observed',r,source,peer_robot_id=peer.source_robot_id)
        elif msg.state==ExplorationClaim.NAVIGATING:
            self.event('ARBITRATION_WON',msg.state_reason or 'proposal proceeded',r,source,console=True,arbitration_result='WON',arbitration_reason=msg.state_reason,**fields); self.event('NAV_GOAL_SENT','dispatch inferred from coordinator transition',r,source,console=True,goal_x=fields['approach_x'],goal_y=fields['approach_y'],goal_yaw=fields['approach_yaw'],**fields); self.event('NAV_GOAL_ACCEPTED',msg.state_reason or 'goal accepted',r,source,console=True,**fields); d['navigation_active']=True
            for name,peer in self.claims.items():
                if name!=r and peer.state==ExplorationClaim.NAVIGATING and duplicate_goal((fields['approach_x'],fields['approach_y']),(peer.approach_pose.pose.position.x,peer.approach_pose.pose.position.y),self.p['duplicate_goal_tolerance_m']):self.event('DUPLICATE_GOAL_REGION','active goals within tolerance',r,source,peer_robot_id=name)
        elif msg.state in (ExplorationClaim.RELEASED,ExplorationClaim.CANCELED):
            kind='ARBITRATION_LOST' if 'arbitration' in msg.state_reason.lower() else ('NAVIGATION_CANCELED' if msg.state==ExplorationClaim.CANCELED else 'CLAIM_RELEASED'); self.event(kind,msg.state_reason or state,r,source,console=True,arbitration_result='LOST' if kind=='ARBITRATION_LOST' else None,**fields); d['navigation_active']=False
        elif msg.state in (ExplorationClaim.SUCCEEDED,ExplorationClaim.FAILED):
            kind='NAVIGATION_SUCCEEDED' if msg.state==ExplorationClaim.SUCCEEDED else ('NAV_GOAL_REJECTED' if 'reject' in msg.state_reason.lower() else ('NAVIGATION_TIMEOUT' if 'timeout' in msg.state_reason.lower() else 'NAVIGATION_FAILED')); self.event(kind,msg.state_reason or state,r,source,severity='ERROR' if msg.state==ExplorationClaim.FAILED else 'INFO',console=True,failure_reason=self.classify(msg.state_reason) if msg.state==ExplorationClaim.FAILED else None,**fields); d['navigation_active']=False
        if msg.state!=ExplorationClaim.NAVIGATING:
            for kind in self.detectors[r].reset():self.event(kind,'navigation no longer active',r)
    def classify(self,reason):
        text=reason.lower()
        for key,value in [('server','ACTION_SERVER_UNAVAILABLE'),('reject','GOAL_REJECTED'),('planner','PLANNER_FAILURE'),('controller','CONTROLLER_FAILURE'),('transform','TF_FAILURE'),('tf','TF_FAILURE'),('costmap','COSTMAP_FAILURE'),('collision','COLLISION_MONITOR_BLOCK'),('timeout','NAVIGATION_TIMEOUT'),('arbitration','CANCELED_BY_ARBITRATION')]:
            if key in text:return value
        return 'UNKNOWN_NAV2_FAILURE'
    def status(self,r,msg):
        self.mark(r,'exploration_status',msg); old=self.statuses.get(r); self.statuses[r]=msg; state=STATUS_STATES.get(msg.state,str(msg.state)); now=self.ros_seconds()
        self.latest[r]['exploration_status']=state
        if old is None or old.state!=msg.state:
            self.event('EXPLORATION_STATUS_CHANGED',state,r,f'/cslam/{r}/exploration_status',source_stamp=stamp(msg),status_state=state,status_reason=msg.reason,status_revision=msg.message_revision,candidate_count=msg.candidate_count,eligible_candidate_count=msg.eligible_candidate_count)
        if msg.state==ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is None:self.exhausted_since[r]=now
        elif msg.state!=ExplorationStatus.NO_ELIGIBLE_CANDIDATES and self.exhausted_since[r] is not None:
            self.exhausted_duration[r]+=now-self.exhausted_since[r]; self.exhausted_since[r]=None
        if msg.state==ExplorationStatus.COMPLETE and self.mission_completion_time is None:self.mission_completion_time=now-self.start_ros
    def coordinator_event(self,r,msg):
        self.mark(r,'exploration_event',msg); fields={'cycle_number':msg.cycle_number,'claim_id':msg.claim_id,'frontier_id':msg.frontier_id,'candidate_map_revision':msg.candidate_map_revision,'selected_rank':msg.selected_rank,'path_length_m':msg.path_length_m,'information_gain':msg.information_gain,'goal_x':msg.goal_pose.pose.position.x,'goal_y':msg.goal_pose.pose.position.y,'goal_yaw':yaw(msg.goal_pose.pose.orientation),'terminal_result':msg.terminal_result,'duration_s':msg.duration_s,'suppression_reason':msg.reason}
        if msg.event_type=='EXPLORATION_CYCLE_STARTED':
            self.cycle_starts[(r,msg.claim_id)]=(self.ros_seconds(),self.local_trajectory.total_distance.get(r,0.)); self.region_attempts[r][msg.frontier_id]+=1
        if msg.event_type=='EXPLORATION_CYCLE_ENDED':
            start=self.cycle_starts.pop((r,msg.claim_id),None)
            if start:
                fields['duration_s']=self.ros_seconds()-start[0]; fields['actual_travelled_distance_m']=self.local_trajectory.total_distance.get(r,0.)-start[1]; self.cycle_durations[r].append(fields['duration_s'])
        self.robot_counts[r][msg.event_type]+=1
        if msg.event_type=='MISSION_COMPLETE' and self.mission_completion_time is None:self.mission_completion_time=self.ros_seconds()-self.start_ros
        self.event(msg.event_type,msg.reason or msg.terminal_result or msg.event_type,r,f'/cslam/{r}/exploration_event',source_stamp=stamp(msg),**fields)
    def feedback(self,r,msg):
        self.mark(r,'navigate_feedback',msg); f=msg.feedback; old=self.latest[r].get('recoveries',0); self.latest[r].update(distance_remaining=float(f.distance_remaining),recoveries=int(f.number_of_recoveries))
        if old!=f.number_of_recoveries:self.event('RECOVERY_COUNT_CHANGED',f'{old} -> {f.number_of_recoveries}',r,f'/{r}/navigate_to_pose/_action/feedback',source_stamp=stamp(f.current_pose),recoveries=f.number_of_recoveries,distance_remaining_m=f.distance_remaining)

    @staticmethod
    def _query_stamp_seconds(source_stamp):
        return float(source_stamp[0]) + float(source_stamp[1]) * 1.0e-9

    def _query_event_time(self, message):
        """Return simulation time for a /rosout query event.

        Some early rosout publishers in this stack leave ``Log.stamp`` at
        zero even after /clock is live.  The observer receipt clock is the
        correct simulation-time fallback in that case; retaining a zero
        timestamp would incorrectly place every diagnostic query at t=0.
        """
        stamped = self._query_stamp_seconds(stamp(message))
        current = float(self.ros_seconds())
        if stamped <= 0.0 and current > 0.0:
            return current
        return stamped

    @staticmethod
    def _query_apply_tf(transform, x, y):
        """Apply a planar ``target <- source`` TF observation to a point."""
        if not transform or not transform.get('available'):
            return None
        translation = transform.get('translation') or {}
        quaternion = transform.get('quaternion') or {}
        heading = math.atan2(
            2.0 * (float(quaternion.get('w', 1.0)) *
                    float(quaternion.get('z', 0.0)) +
                    float(quaternion.get('x', 0.0)) *
                    float(quaternion.get('y', 0.0))),
            1.0 - 2.0 * (float(quaternion.get('y', 0.0)) ** 2 +
                         float(quaternion.get('z', 0.0)) ** 2),
        )
        cosine, sine = math.cos(heading), math.sin(heading)
        return (
            float(translation.get('x', 0.0)) + cosine * float(x) - sine * float(y),
            float(translation.get('y', 0.0)) + sine * float(x) + cosine * float(y),
        )

    def _query_tf(self, target, source, query_ros):
        """Capture the exact-time TF used by a diagnostic grid sample.

        The normal Buffer lookup is attempted at the query timestamp.  The
        observer's already-captured raw /tf stream is only a passive forensic
        fallback; neither path publishes data or affects the running stack.
        """
        target = self._normal_frame(target)
        source = self._normal_frame(source)
        if not target or not source:
            return {
                'available': False, 'target_frame': target,
                'source_frame': source, 'lookup_mode': 'invalid_frame',
                'requested_stamp_s': float(query_ros), 'error': 'empty frame',
            }
        if target == source:
            return {
                'available': True, 'target_frame': target,
                'source_frame': source, 'lookup_mode': 'identity',
                'requested_stamp_s': float(query_ros), 'transform_stamp_s': float(query_ros),
                'age_s': 0.0, 'returned_stamp_delta_s': 0.0,
                'interpolation_used': False, 'translation': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'quaternion': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'path': [source], 'error': '',
            }
        try:
            query_time = Time(
                nanoseconds=max(0, int(round(float(query_ros) * 1.0e9))),
                clock_type=ClockType.ROS_TIME)
            transform = self.tf_buffer.lookup_transform(
                target, source, query_time, timeout=Duration(seconds=0.05))
            observation = self._tf_observation(transform, query_ros)
            observation.update({'target_frame': target, 'source_frame': source})
            return observation
        except TransformException as exc:
            fallback, metadata = self._direct_sync_tf(
                target, source, float(query_ros), allow_latest_before=False)
            if fallback is not None:
                observation = self._planar_tf_observation(
                    fallback, target, source, query_ros, metadata)
                observation['error'] = str(exc)
                return observation
            return {
                'available': False, 'target_frame': target,
                'source_frame': source, 'lookup_mode': 'exact_ros_time_failed',
                'requested_stamp_s': float(query_ros), 'transform_stamp_s': None,
                'age_s': None, 'returned_stamp_delta_s': None,
                'interpolation_used': False, 'translation': None,
                'quaternion': None, 'path': list(metadata.get('path', ())),
                'error': str(exc), 'fallback_error': metadata.get('error', ''),
            }

    @staticmethod
    def _query_grid_cell(message, x, y):
        """Return the cell containing a point in the OccupancyGrid frame."""
        info = message.info
        resolution = float(info.resolution)
        if resolution <= 0.0 or int(info.width) <= 0 or int(info.height) <= 0:
            return None
        heading = yaw(info.origin.orientation)
        dx, dy = float(x) - float(info.origin.position.x), float(y) - float(info.origin.position.y)
        cosine, sine = math.cos(heading), math.sin(heading)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column, row = math.floor(local_x / resolution), math.floor(local_y / resolution)
        if column < 0 or row < 0 or column >= int(info.width) or row >= int(info.height):
            return None
        index = int(row) * int(info.width) + int(column)
        if index < 0 or index >= len(message.data):
            return None
        return int(column), int(row), index

    @staticmethod
    def _query_cell_center(message, column, row):
        info = message.info
        heading = yaw(info.origin.orientation)
        cosine, sine = math.cos(heading), math.sin(heading)
        local_x = (float(column) + 0.5) * float(info.resolution)
        local_y = (float(row) + 0.5) * float(info.resolution)
        return (
            float(info.origin.position.x) + cosine * local_x - sine * local_y,
            float(info.origin.position.y) + sine * local_x + cosine * local_y,
        )

    def _query_save_crop(self, kind, robot, query_id, label, message, x, y, cell):
        if message is None or cell is None:
            return None
        info = message.info
        width, height = int(info.width), int(info.height)
        values = np.asarray(message.data, dtype=np.int16)
        if width <= 0 or height <= 0 or values.size != width * height:
            return None
        values = values.reshape(height, width)
        radius_cells = max(1, int(math.ceil(0.5 / max(float(info.resolution), 1.0e-9))))
        column, row, _ = cell
        c0, c1 = max(0, column - radius_cells), min(width - 1, column + radius_cells)
        r0, r1 = max(0, row - radius_cells), min(height - 1, row + radius_cells)
        crop_directory = self.frontier_query_crops.get(kind)
        if crop_directory is None:
            crop_directory = self.frontier_query_crops.get(f'{kind}_crops')
        if crop_directory is None:
            return None
        path = crop_directory / (
            f'query_{int(query_id):06d}_{robot}_{label}.json')
        crop = values[r0:r1 + 1, c0:c1 + 1].tolist()
        payload = {
            'schema_version': 'nav2_frontier_rejection_crop_1.0',
            'kind': kind, 'robot_id': robot, 'query_id': int(query_id),
            'label': label, 'frame_id': str(message.header.frame_id),
            'header_stamp_s': self._query_stamp_seconds(stamp(message)),
            'resolution_m': float(info.resolution),
            'origin_x': float(info.origin.position.x),
            'origin_y': float(info.origin.position.y),
            'origin_yaw_rad': yaw(info.origin.orientation),
            'requested_point_xy': [float(x), float(y)],
            'cell_column': int(column), 'cell_row': int(row),
            'column_range': [int(c0), int(c1)], 'row_range': [int(r0), int(r1)],
            'values': crop,
        }
        path.write_text(json.dumps(finite(payload), separators=(',', ':')) + '\n', encoding='utf-8')
        return str(path)

    def _query_grid_sample(self, kind, robot, query_id, label, message,
                           point_xy, source_frame, query_ros, save_crop=False):
        if message is None:
            return {'available': False, 'reason': 'NO_MESSAGE',
                    'point_xy': list(point_xy) if point_xy is not None else None}
        if point_xy is None or len(point_xy) < 2 or not all(
                value is not None and math.isfinite(float(value))
                for value in point_xy[:2]):
            return {'available': False, 'reason': 'NO_POINT', 'point_xy': None}
        target_frame = str(message.header.frame_id or '')
        transform = self._query_tf(target_frame, source_frame, query_ros)
        transformed = self._query_apply_tf(transform, point_xy[0], point_xy[1])
        sample = {
            'available': transformed is not None,
            'kind': kind, 'label': label,
            'source_frame': str(source_frame), 'target_frame': target_frame,
            'point_source_xy': [float(point_xy[0]), float(point_xy[1])],
            'tf': transform,
            'point_target_xy': list(transformed) if transformed is not None else None,
            'header_stamp_s': self._query_stamp_seconds(stamp(message)),
            'sample_age_s': float(query_ros) - self._query_stamp_seconds(stamp(message)),
            'frame_id': target_frame,
            'resolution_m': float(message.info.resolution),
            'origin': {
                'x': float(message.info.origin.position.x),
                'y': float(message.info.origin.position.y),
                'yaw_rad': yaw(message.info.origin.orientation),
            },
            'width': int(message.info.width), 'height': int(message.info.height),
        }
        if transformed is None:
            sample.update({'classification': 'TF_FAILURE', 'cell': None})
            return sample
        cell = self._query_grid_cell(message, transformed[0], transformed[1])
        sample['cell'] = list(cell) if cell is not None else None
        values = np.asarray(message.data, dtype=np.int16)
        valid_shape = values.size == int(message.info.width) * int(message.info.height)
        if cell is None or not valid_shape:
            sample.update({
                'raw_cost': None,
                'classification': 'OUTSIDE_MAP' if cell is None else 'INVALID_GRID',
                'max_cost_within_footprint': None,
                'max_cost_within_radius': {str(radius): None for radius in (0.05, 0.10, 0.15, 0.25, 0.50)},
                'nearest_lethal_distance_m': None,
                'nearest_inflated_distance_m': None,
                'nearest_unknown_distance_m': None,
            })
            return sample
        values = values.reshape(int(message.info.height), int(message.info.width))
        column, row, index = cell
        raw = int(values[row, column])
        is_shared = kind == 'shared_map'
        occupied_threshold = 50 if is_shared else 100
        inflated_threshold = 50 if is_shared else 1
        if raw < 0:
            classification = 'UNKNOWN'
        elif raw >= occupied_threshold:
            classification = 'LETHAL_OR_OCCUPIED'
        elif raw >= inflated_threshold:
            classification = 'INFLATED' if not is_shared else 'FREE_COST_RANGE'
        else:
            classification = 'FREE'
        sample['raw_cost'] = raw
        sample['classification'] = classification
        sample['max_cost_within_radius'] = {}
        for radius in (0.05, 0.10, 0.15, 0.25, 0.50):
            radius_cells = int(math.ceil(radius / max(float(message.info.resolution), 1.0e-9)))
            window = values[max(0, row - radius_cells):min(values.shape[0], row + radius_cells + 1),
                            max(0, column - radius_cells):min(values.shape[1], column + radius_cells + 1)]
            sample['max_cost_within_radius'][str(radius)] = int(window.max()) if window.size else None
        footprint_radius = float(self.p.get('diagnostic_footprint_radius_m', 0.08))
        footprint_cells = int(math.ceil(footprint_radius / max(float(message.info.resolution), 1.0e-9)))
        footprint = values[max(0, row - footprint_cells):min(values.shape[0], row + footprint_cells + 1),
                           max(0, column - footprint_cells):min(values.shape[1], column + footprint_cells + 1)]
        sample['footprint_radius_m'] = footprint_radius
        sample['max_cost_within_footprint'] = int(footprint.max()) if footprint.size else None
        # Keep the observer callback bounded.  The previous Python loop over
        # every cell in every grid made a failed-query capture block on large
        # global costmaps, causing later /rosout records to be dropped.  The
        # same nearest-cell calculation is vectorized here without changing
        # any runtime navigation data.
        nearest = {'lethal': None, 'inflated': None, 'unknown': None}
        info = message.info
        cosine, sine = math.cos(yaw(info.origin.orientation)), math.sin(yaw(info.origin.orientation))
        for key, mask in (
                ('lethal', values >= occupied_threshold),
                ('inflated', values >= inflated_threshold),
                ('unknown', values < 0)):
            rows, columns = np.nonzero(mask)
            if rows.size == 0:
                continue
            local_x = (columns.astype(np.float64) + 0.5) * float(info.resolution)
            local_y = (rows.astype(np.float64) + 0.5) * float(info.resolution)
            centers_x = float(info.origin.position.x) + cosine * local_x - sine * local_y
            centers_y = float(info.origin.position.y) + sine * local_x + cosine * local_y
            nearest[key] = float(np.hypot(
                centers_x - float(transformed[0]),
                centers_y - float(transformed[1])).min())
        sample['nearest_lethal_distance_m'] = nearest['lethal']
        sample['nearest_inflated_distance_m'] = nearest['inflated']
        sample['nearest_unknown_distance_m'] = nearest['unknown']
        if save_crop:
            sample['crop_path'] = self._query_save_crop(
                kind, robot, query_id, label, message, transformed[0], transformed[1], cell)
        return finite(sample)

    def _query_frontier_robot(self, name):
        normalized = str(name or '').lstrip('/')
        for robot in self.robots:
            # rclpy /rosout uses the fully-qualified logger name with either
            # slash or dot separators depending on the emitting node and
            # launch composition.  The frontier generators in this runtime
            # emit e.g. ``robot1.local_frontier_candidate_generator``.
            if (normalized == robot or normalized.startswith(robot + '/') or
                    normalized.startswith(robot + '.')):
                return robot
        return None

    @staticmethod
    def _query_number(pattern, text, default=None, cast=float):
        match = re.search(pattern, text)
        if not match:
            return default
        try:
            return cast(match.group(1))
        except (TypeError, ValueError):
            return default

    def _query_capture_request(self, robot, message):
        if not self.frontier_query_forensics:
            return
        query_id = self._query_number(r'query_id=(\d+)', message.msg, None, int)
        frontier_id = self._query_number(r'\bid=(\d+)', message.msg, None, int)
        if query_id is None or frontier_id is None:
            return
        query_ros = self._query_event_time(message)
        global_costmap = self.latest[robot].get('global_costmap/costmap')
        start = self.latest[robot].get('shared_pose') or self.latest[robot].get('pose')
        start_frame = self.latest[robot].get('shared_pose_frame') or self.latest[robot].get('pose_frame') or self.p['global_frame']
        if start is None:
            start = (None, None, None)
        prior = int(self.frontier_query_counts.get((robot, frontier_id), 0))
        self.frontier_query_pending[(robot, query_id)] = {
            'query_id': int(query_id), 'frontier_id': int(frontier_id),
            'robot_id': robot, 'query_start_ros_s': query_ros,
            'map_revision': self._query_number(r'map_revision=(\d+)', message.msg, None, int),
            'costmap_revision': self._query_number(r'costmap_revision=(\d+)', message.msg, None, int),
            'candidate_generation_id': self._query_number(
                r'candidate_generation_id=(\d+)', message.msg, None, int),
            'map_stamp_s': self._query_number(
                r'map_stamp_s=([-+0-9.eE]+)', message.msg, None),
            'costmap_stamp_s': self._query_number(
                r'costmap_stamp_s=([-+0-9.eE]+)', message.msg, None),
            'query_count_before': prior,
            'previously_queried': prior > 0,
            'last_query_ros_s': self.frontier_query_last_time.get((robot, frontier_id)),
            'start_xy': [start[0], start[1]] if start[0] is not None else None,
            'start_yaw_rad': start[2] if start[2] is not None else None,
            'start_frame': str(start_frame),
            'planner_frame': str(global_costmap.header.frame_id) if global_costmap is not None else self.p['global_frame'],
            'goal_frame': (re.search(r'goal_frame=([^\s]+)', message.msg).group(1)
                          if re.search(r'goal_frame=([^\s]+)', message.msg)
                          else self.latest[robot].get('candidate_frame') or self.p['global_frame']),
            'selected_message': message.msg,
        }

    def _query_capture_result(self, robot, message):
        if not self.frontier_query_forensics:
            return
        query_id = self._query_number(r'query_id=(\d+)', message.msg, None, int)
        frontier_id = self._query_number(r'\bid=(\d+)', message.msg, None, int)
        if query_id is None or frontier_id is None:
            return
        query_ros = self._query_event_time(message)
        pending = self.frontier_query_pending.pop((robot, query_id), {})
        metadata = dict(self.frontier_metadata.get(robot, {}).get(frontier_id, {}))
        target_x = self._query_number(r'target_x=([-+0-9.eE]+)', message.msg, None)
        target_y = self._query_number(r'target_y=([-+0-9.eE]+)', message.msg, None)
        start_xy = pending.get('start_xy')
        if start_xy is None:
            pose = self.latest[robot].get('shared_pose') or self.latest[robot].get('pose')
            start_xy = [pose[0], pose[1]] if pose is not None else None
        start_frame = pending.get('start_frame') or self.p['global_frame']
        goal_frame_match = re.search(r'goal_frame=([^\s]+)', message.msg)
        goal_frame = (goal_frame_match.group(1) if goal_frame_match
                      else pending.get('goal_frame') or self.p['global_frame'])
        planner_frame = pending.get('planner_frame') or self.p['global_frame']
        if target_x is None or target_y is None:
            approach = metadata.get('approach')
            target_x, target_y = (approach if approach else (None, None))
        goal_xy = [target_x, target_y] if target_x is not None and target_y is not None else None
        target_yaw = self._query_number(
            r'target_yaw=([-+0-9.eE]+)', message.msg, None)
        error_name = re.search(r'error_name=([^\s]+)', message.msg)
        error_name = error_name.group(1) if error_name else 'UNKNOWN'
        action_result = re.search(r'action_result=([^\s]+)', message.msg)
        action_result = action_result.group(1) if action_result else 'UNKNOWN'
        status = re.search(r'\bstatus=([^\s]+)', message.msg)
        status = status.group(1) if status else 'UNKNOWN'
        failure_class = re.search(r'failure_class=([^\s]+)', message.msg)
        failure_class = failure_class.group(1) if failure_class else 'UNKNOWN'
        error_code = self._query_number(r'error_code=(-?\d+)', message.msg, None, int)
        duration = self._query_number(r'duration_s=([-+0-9.eE]+)', message.msg, None)
        path_length = self._query_number(r'path_length_m=([-+0-9.eE]+)', message.msg, None)
        error_message = re.search(r'error_message="(.*?)"', message.msg)
        error_message = error_message.group(1) if error_message else ''
        query_count = int(self.frontier_query_counts.get((robot, frontier_id), 0)) + 1
        self.frontier_query_counts[(robot, frontier_id)] = query_count
        self.frontier_query_last_time[(robot, frontier_id)] = query_ros
        if path_length is not None and path_length < 0.0:
            path_length = None
        save_crops = status != 'REACHABLE' and action_result != 'SUCCEEDED'
        grid_sources = {
            'local_costmap': self.latest[robot].get('local_costmap/costmap'),
            'global_costmap': self.latest[robot].get('global_costmap/costmap'),
            'shared_map': self.latest[robot].get('shared_map') or self.latest[robot].get('map'),
        }
        grid_samples = {}
        tf_rows = []
        for kind, grid in grid_sources.items():
            if grid is None:
                grid_samples[kind] = {'available': False, 'reason': 'NO_MESSAGE'}
                continue
            start_sample = self._query_grid_sample(
                kind, robot, query_id, 'start', grid, start_xy, start_frame,
                query_ros, save_crop=save_crops)
            goal_sample = self._query_grid_sample(
                kind, robot, query_id, 'goal', grid, goal_xy, goal_frame,
                query_ros, save_crop=save_crops) if goal_xy is not None else {
                    'available': False, 'reason': 'NO_GOAL_POINT'}
            grid_samples[kind] = {'start': start_sample, 'goal': goal_sample}
            for label, sample in (('start', start_sample), ('goal', goal_sample)):
                if sample.get('tf'):
                    tf_rows.append({
                        'query_id': query_id, 'robot_id': robot,
                        'frontier_id': frontier_id, 'sim_time_s': query_ros,
                        'purpose': f'{kind}_{label}', **sample['tf'],
                    })
        peer = next((candidate for candidate in self.robots
                     if candidate != robot), None)
        own_pose = self.latest[robot].get('shared_pose') or self.latest[robot].get('pose')
        peer_pose = (self.latest.get(peer, {}).get('shared_pose')
                     if peer is not None else None)
        peer_distance = peer_bearing = None
        if own_pose is not None and peer_pose is not None:
            dx, dy = float(peer_pose[0]) - float(own_pose[0]), float(peer_pose[1]) - float(own_pose[1])
            peer_distance = math.hypot(dx, dy)
            peer_bearing = math.atan2(dy, dx) - float(own_pose[2])
            peer_bearing = (peer_bearing + math.pi) % (2.0 * math.pi) - math.pi
        scan = self.latest[robot].get('scan_d500_nav')
        nearest_scan = None
        if scan is not None:
            finite_ranges = [(float(value), float(scan.angle_min) + index * float(scan.angle_increment))
                             for index, value in enumerate(scan.ranges)
                             if math.isfinite(float(value)) and float(scan.range_min) <= float(value) <= float(scan.range_max)]
            if finite_ranges:
                nearest_scan = min(finite_ranges, key=lambda item: item[0])
        local_start = grid_samples.get('local_costmap', {}).get('start', {})
        global_start = grid_samples.get('global_costmap', {}).get('start', {})
        if 'START_OCCUPIED' in error_name:
            primary = 'START_OCCUPIED'
        elif 'GOAL_OCCUPIED' in error_name:
            primary = 'GOAL_OCCUPIED'
        elif 'START_OUTSIDE_MAP' in error_name:
            primary = 'START_OUTSIDE_MAP'
        elif 'GOAL_OUTSIDE_MAP' in error_name:
            primary = 'GOAL_OUTSIDE_MAP'
        elif 'TF_ERROR' in error_name:
            primary = 'TF_FAILURE'
        elif 'TIMEOUT' in error_name or 'TIMEOUT' in failure_class:
            primary = 'TIMEOUT'
        elif 'NO_VALID_PATH' in error_name:
            start_classes = {local_start.get('classification'), global_start.get('classification')}
            if 'LETHAL_OR_OCCUPIED' in start_classes:
                primary = 'START_LETHAL'
            elif 'INFLATED' in start_classes:
                primary = 'START_INFLATED'
            elif 'UNKNOWN' in start_classes:
                primary = 'START_UNKNOWN'
            else:
                primary = 'NO_CONNECTED_FREE_PATH'
        elif status == 'REACHABLE' or action_result == 'SUCCEEDED':
            primary = 'SUCCESS'
        else:
            primary = 'ACTION_ABORT_WITH_OTHER_REASON'
        record = {
            'schema_version': 'nav2_frontier_query_forensic_1.0',
            'sim_time_s': query_ros, 'robot_id': robot,
            'query_id': int(query_id), 'frontier_id': int(frontier_id),
            'physical_signature': f'canonical_id:{int(frontier_id):016x}',
            'physical_signature_source': 'canonical_frontier_id_only',
            'centroid_xy': metadata.get('centroid'), 'approach_xy': goal_xy,
            'start_xy': start_xy, 'start_yaw_rad': pending.get('start_yaw_rad'),
            'goal_yaw_rad': target_yaw,
            'planner_frame': planner_frame, 'goal_frame': goal_frame,
            'start_frame': start_frame, 'map_revision': pending.get('map_revision'),
            'costmap_revision': pending.get('costmap_revision'),
            'candidate_generation_id': self._query_number(
                r'candidate_generation_id=(\d+)', message.msg,
                pending.get('candidate_generation_id'), int),
            'map_stamp_s': pending.get('map_stamp_s'),
            'costmap_stamp_s': pending.get('costmap_stamp_s'),
            'query_count': query_count, 'query_count_before': pending.get('query_count_before', query_count - 1),
            'previously_queried': bool(pending.get('previously_queried', query_count > 1)),
            'last_query_time_s': pending.get('last_query_ros_s'),
            'planner_action_result': action_result, 'planner_status': status,
            'planner_failure_class_raw': failure_class, 'error_code': error_code,
            'error_name': error_name, 'planner_result_text': error_message,
            'planner_duration_s': duration, 'returned_path_length_m': path_length,
            'classification': primary, 'costmaps': grid_samples,
            'scan': {
                'available': scan is not None,
                'frame_id': str(scan.header.frame_id) if scan is not None else None,
                'header_stamp_s': self._query_stamp_seconds(stamp(scan)) if scan is not None else None,
                'nearest_range_m': nearest_scan[0] if nearest_scan else None,
                'nearest_angle_rad': nearest_scan[1] if nearest_scan else None,
                'range_min_m': float(scan.range_min) if scan is not None else None,
                'range_max_m': float(scan.range_max) if scan is not None else None,
            },
            'peer': {'peer_robot_id': peer, 'distance_m': peer_distance,
                     'bearing_from_robot_heading_rad': peer_bearing},
            'collision_monitor_state': 'NOT_EXPOSED_TO_EXISTING_OBSERVER',
            'selected_message': pending.get('selected_message', ''),
            'result_message': message.msg,
        }
        with self._io_lock:
            if self.frontier_query_forensic_file is not None:
                self.frontier_query_forensic_file.write(
                    json.dumps(finite(record), separators=(',', ':'), allow_nan=False) + '\n')
            if self.frontier_query_tf_file is not None:
                for row in tf_rows:
                    self.frontier_query_tf_file.write(
                        json.dumps(finite(row), separators=(',', ':'), allow_nan=False) + '\n')

    def _capture_frontier_query_rosout(self, message):
        if not self.frontier_query_forensics:
            return
        robot = self._query_frontier_robot(message.name)
        if robot is None:
            return
        text = str(message.msg)
        if 'FRONTIER_QUERY_LIFECYCLE' in text and 'state=REQUEST_SENT' in text:
            self._query_capture_request(robot, message)
        elif ('FRONTIER_QUERY_RESULT' in text and
              'FRONTIER_QUERY_RESULT_PENDING' not in text):
            self._query_capture_result(robot, message)

    def _write_rosout_receipt(self, row, diagnostic=None,
                              warning_wall_time=None):
        if row is None or self.rosout_receipt_file is None:
            return
        if warning_wall_time is not None:
            row['warning_wall_time_utc'] = warning_wall_time
        if diagnostic is not None:
            for key in ('event_sequence', 'wall_time_utc', 'ros_time_sec',
                        'ros_time_nanosec', 'elapsed_s', 'wall_elapsed_s'):
                row[f'diagnostic_{key}'] = diagnostic.get(key)
        try:
            with self._io_lock:
                self.rosout_receipt_file.write(
                    json.dumps(finite(row), separators=(',', ':')) + '\n')
        except (OSError, TypeError, ValueError) as exc:
            self.write_failures += 1
            self.record_internal_error('rosout_receipt', exc)

    def rosout(self,msg):
        logger_name = msg.name.lstrip('/')
        if (logger_name == 'cooperative_experiment_logger' or
                logger_name.endswith('.cooperative_experiment_logger')):
            return
        self._capture_frontier_query_rosout(msg)
        receipt_row = None
        if self.rosout_receipt_file is not None:
            receipt_sec, receipt_nanosec = self.ros_now()
            receipt_row = {
                'schema_version': SCHEMA,
                'run_id': self.run_id,
                'receipt_sequence': self._rosout_receipt_sequence,
                'wall_time_utc': utc_now(),
                'ros_time_sec': receipt_sec,
                'ros_time_nanosec': receipt_nanosec,
                'elapsed_s': self.ros_seconds() - self.start_ros,
                'wall_elapsed_s': time.monotonic() - self.start,
                'source_stamp_sec': int(msg.stamp.sec),
                'source_stamp_nanosec': int(msg.stamp.nanosec),
                'level': int(msg.level),
                'name': str(msg.name),
                'message': str(msg.msg),
            }
            self._rosout_receipt_sequence += 1
        text=msg.name+' '+msg.msg
        lower=text.lower()
        diagnostic_category=rosout_diagnostic_category(text)
        diagnostic_record = None
        if 'COMPUTE_PATH_REUSED' in msg.msg:
            source_match = re.search(r'source=([A-Z0-9_]+)', msg.msg)
            source = source_match.group(1) if source_match else 'UNKNOWN'
            self.planner_query_counts[f'{source}.REUSED'] += 1
        elif 'COMPUTE_PATH_RESULT' in msg.msg or 'CANDIDATE_PATH_RESULT' in msg.msg:
            source_match = re.search(r'source=([A-Z0-9_]+)', msg.msg)
            source = source_match.group(1) if source_match else 'UNKNOWN'
            if re.search(r'\bok=true\b|\bvalid=true\b', msg.msg):
                outcome = 'SUCCESS'
            elif re.search(r'TIMEOUT|timeout', msg.msg):
                outcome = 'TIMEOUT'
            else:
                outcome = 'FAILURE'
            self.planner_query_counts[f'{source}.{outcome}'] += 1
            duration_match = re.search(r'duration_s=([0-9]+(?:\.[0-9]+)?)', msg.msg)
            if duration_match:
                self.planner_query_duration_s[source] += float(
                    duration_match.group(1))
        controller_or_planner=(re.search(r'controller|planner',lower) and
                               re.search(r'failed|failure|abort|progress checker|no valid control|timeout',lower))
        if controller_or_planner and diagnostic_category is None:
            diagnostic_category='CONTROLLER_OR_PLANNER_ERROR'
        if diagnostic_category is not None:
            source_stamp=(msg.stamp.sec,msg.stamp.nanosec)
            row=self.common('/rosout:'+msg.name,source_stamp=source_stamp)
            row.update(severity='ERROR' if msg.level>=Log.ERROR else ('WARN' if msg.level>=Log.WARN else 'INFO'),category=diagnostic_category,message=msg.msg,node=msg.name)
            diagnostic_record = row
            raw_replay_authority = (
                self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False)))
            if not raw_replay_authority:
                key=(msg.name,diagnostic_category,msg.msg); previous=self._diagnostic_last.get(key); now=row['elapsed_s']
                if (previous is None or now-previous>=0.25) and self.nav2_diagnostic_count<10000:
                    self._diagnostic_last[key]=now; self.nav2_diagnostic_count+=1
                    try:
                        with self._io_lock:self.nav2_diagnostics.write(json.dumps(finite(row),separators=(',',':'),allow_nan=False)+'\n')
                    except (OSError,TypeError,ValueError) as exc:self.write_failures+=1; self.get_logger().error(f'Nav2 diagnostic write failed: {exc}',throttle_duration_sec=10.)
        if msg.level<Log.WARN:
            self._write_rosout_receipt(receipt_row, diagnostic_record)
            return
        severity='ERROR' if msg.level>=Log.ERROR else 'WARN'; category=warning_category(text)
        warning_wall_time = utc_now()
        with self._state_lock:record,new=self.warns.add(msg.name,severity,msg.msg,warning_wall_time,category)
        if new:self.event(category,msg.msg,source='/rosout:'+msg.name,severity=severity,source_stamp=(msg.stamp.sec,msg.stamp.nanosec),occurrence_count=1)
        self._write_rosout_receipt(
            receipt_row, diagnostic_record, warning_wall_time)
    def row_time(self):
        sec,nsec=self.ros_now()
        with self._state_lock:self.sequence+=1; sequence=self.sequence
        return {'run_id':self.run_id,'wall_time_utc':utc_now(),'ros_time_sec':sec,'ros_time_nanosec':nsec,'elapsed_s':self.ros_seconds()-self.start_ros,'wall_elapsed_s':time.monotonic()-self.start,'event_sequence':sequence}
    def map_snapshot(self,r,key):
        with self._state_lock:msg=self.latest[r].get(key)
        if msg is None:return None
        cache_key=(r,key); identity=id(msg); cached=self._map_cache.get(cache_key)
        if cached is not None and cached[0]==identity:return cached
        grid=as_grid(msg); counts=known_counts(grid); digest=hashlib.sha256(grid.data.tobytes()).digest()
        cached=(identity,grid,counts,digest,msg); self._map_cache[cache_key]=cached; return cached
    def map_counts(self,r,key):
        snapshot=self.map_snapshot(r,key)
        if snapshot is None:return 0,0,0
        free,occupied,unknown=snapshot[2]; return free+occupied,free,occupied
    def sample_telemetry(self):
        if not self.stack_ready and all(all(k in self.latest[r] for k in ('odom','map','shared_map','frontier_candidates')) for r in self.robots):
            self.stack_ready=True
            self.event('STACK_READY','critical robot telemetry is available',console=True)
        for r in self.robots:
            d=self.latest[r]; pose=d.get('pose',(None,None,None)); speed=d.get('speed',(0.,0.)); command=d.get('command',(0.,0.)); goal=d.get('goal',(None,None,None)); cmd_age=self.age(r,'cmd_vel'); cmd_received=cmd_age is not None and cmd_age<=float(self.p['cmd_vel_no_command_timeout_s']); row=self.row_time(); row.update(robot_id=r,pose_x=pose[0],pose_y=pose[1],pose_yaw=pose[2],linear_speed_mps=speed[0],angular_speed_radps=speed[1],commanded_linear_mps=command[0],commanded_angular_radps=command[1],cmd_vel_received=cmd_received,cmd_vel_age_s=cmd_age,cmd_vel_source=d.get('cmd_vel_source'),distance_travelled_m=self.local_trajectory.total_distance.get(r,0.),claim_state=d.get('claim_state','UNKNOWN'),claim_id=d.get('claim_id'),frontier_id=d.get('frontier_id'),goal_x=goal[0],goal_y=goal[1],goal_yaw=goal[2],navigation_active=d.get('navigation_active',False),distance_remaining_m=d.get('distance_remaining'),recoveries=d.get('recoveries',0),candidate_count=d.get('candidate_count',0),local_known_cells=self.map_counts(r,'map')[0],shared_known_cells=self.map_counts(r,'shared_map')[0],local_costmap_obstacles=self.map_counts(r,'local_costmap/costmap')[2],global_costmap_known=self.map_counts(r,'global_costmap/costmap')[0],global_costmap_obstacles=self.map_counts(r,'global_costmap/costmap')[2],odom_age_s=self.age(r,'odom'),scan_age_s=self.age(r,'scan_d500_slam'),map_age_s=self.age(r,'map'),shared_map_age_s=self.age(r,'shared_map'),claim_age_s=self.age(r,'exploration_claim'),feedback_age_s=self.age(r,'navigate_feedback')); self.csv_row(self.writers[r],row)
            if pose[0] is not None:
                sample=MotionSample(row['elapsed_s'],pose[0],pose[1],d.get('distance_remaining'),command[0],command[1]); near=d.get('distance_remaining') is not None and d['distance_remaining']<.08
                for kind in self.detectors[r].update(sample,d.get('navigation_active',False),near_goal=near):
                    self.event(kind,'windowed passive detector changed state',r,distance_remaining_m=d.get('distance_remaining'),pose_x=pose[0],pose_y=pose[1])
                    if kind in ('STUCK_STARTED','OSCILLATION_STARTED') and self.map_counts(r,'local_costmap/costmap')[2]>0:self.event('CORNER_TRAP_SUSPECTED','motion anomaly with nearby costmap obstacles',r,severity='WARN')
                if d.get('navigation_active') and row['elapsed_s']-self.last_progress.get(r,-99)>=5:
                    self.last_progress[r]=row['elapsed_s']; self.event('NAVIGATION_PROGRESS','periodic low-rate progress sample',r,distance_remaining_m=d.get('distance_remaining'),pose_x=pose[0],pose_y=pose[1],distance_travelled_m=self.local_trajectory.total_distance.get(r,0.),recoveries=d.get('recoveries',0))
    def sample_coverage(self):
        if self.coverage_request_file is not None:
            # Raw-enabled canonical runs defer all map conversion, counting,
            # ownership, and attribution.  Keep only the exact timer request
            # boundary needed to reproduce the legacy CSV after shutdown.
            required_map_key = (
                'map' if self.coverage_source in ('local_map', 'local_map_union')
                else 'shared_map')
            with self._state_lock:
                maps_ready = all(
                    self.latest[robot].get(required_map_key) is not None
                    for robot in self.robots)
            # Preserve the legacy callback's early return exactly: a timer
            # callback before the selected map stream is complete is not a
            # coverage sample and must not become a deferred request row.
            if not maps_ready:
                return
            request = self.row_time()
            with self._io_lock:
                self.coverage_request_file.write(
                    json.dumps(finite(request), separators=(',', ':')) + '\n')
            return
        shared=[self.map_snapshot(r,'shared_map') for r in self.robots]
        local=[self.map_snapshot(r,'map') for r in self.robots]
        local_union = self.coverage_source == 'local_map_union'
        coverage_maps = (
            local if self.coverage_source in ('local_map', 'local_map_union')
            else shared)
        if not all(coverage_maps):
            return
        counts=[snapshot[2] for snapshot in coverage_maps]
        maps=[snapshot[4] for snapshot in coverage_maps]
        known=[a+b for a,b,_ in counts]
        current=max(known)
        if local_union:
            transforms={
                'robot1': (0., 0., 0.),
                'robot2': tuple(self.p['known_relative_transform']),
            }
            if self.p['enable_coverage_attribution']:
                for r,snapshot in zip(self.robots,local):
                    if snapshot and self._last_attributed.get(r)!=snapshot[0]:
                        transform=transforms.get(r,(0.,0.,0.)); transformed=self._transformed_cache.get(r)
                        if transformed is None or transformed[0]!=snapshot[0]:
                            transformed=(snapshot[0],known_world_cells(snapshot[1],self.p['coverage_attribution_resolution'],transform)); self._transformed_cache[r]=transformed
                        self.attribution.observe(r,transformed[1],time.monotonic()-self.start); self._last_attributed[r]=snapshot[0]
            current=self.attribution.summary()['total_known_union_cells']
        self.initial_known=current if self.initial_known is None else self.initial_known
        gain=current-(self.previous_known if self.previous_known is not None else current)
        self.previous_known=current
        if self.coverage_source in ('local_map', 'local_map_union'):
            equivalent = None
        else:
            equivalent=(maps[0].info.width,maps[0].info.height,maps[0].info.resolution,maps[0].info.origin)==(maps[1].info.width,maps[1].info.height,maps[1].info.resolution,maps[1].info.origin) and shared[0][3]==shared[1][3]
        if self.coverage_source in ('local_map', 'local_map_union'):
            self.divergence_since=None; self.divergence_reported=False
        elif equivalent:self.divergence_since=None; self.divergence_reported=False
        elif self.divergence_since is None:self.divergence_since=time.monotonic()
        elif not self.divergence_reported and time.monotonic()-self.divergence_since>=self.p['shared_map_divergence_grace_s']:
            self.divergence_reported=True; self.event('SHARED_MAP_DIVERGENCE','independent shared maps differ beyond grace period',severity='WARN')
        transforms={
            'robot1': (0., 0., 0.),
            'robot2': tuple(self.p['known_relative_transform']),
        }
        if self.p['enable_coverage_attribution']:
            for r,snapshot in zip(self.robots,local):
                if snapshot and self._last_attributed.get(r)!=snapshot[0]:
                    transform=transforms.get(r,(0.,0.,0.)); transformed=self._transformed_cache.get(r)
                    if transformed is None or transformed[0]!=snapshot[0]:
                        transformed=(snapshot[0],known_world_cells(snapshot[1],self.p['coverage_attribution_resolution'],transform)); self._transformed_cache[r]=transformed
                    self.attribution.observe(r,transformed[1],time.monotonic()-self.start); self._last_attributed[r]=snapshot[0]
        a=self.attribution.summary()
        if self.coverage_source == 'local_map':
            # A has no cross-robot ownership problem: report the raw local
            # grid count so coverage.csv agrees with robot1_map_final.npz.
            first_seen_robot1 = current
            first_seen_robot2 = 0
            later_duplicate_robot1 = 0
            later_duplicate_robot2 = 0
            simultaneous = 0
            total_known_union = current
            duplicate_fraction = 0.0
        else:
            first_seen_robot1 = a['unique_first_seen_cells'].get('robot1', 0)
            first_seen_robot2 = a['unique_first_seen_cells'].get('robot2', 0)
            later_duplicate_robot1 = a['later_duplicated_cells'].get('robot1', 0)
            later_duplicate_robot2 = a['later_duplicated_cells'].get('robot2', 0)
            simultaneous = a['simultaneously_observed_cells']
            total_known_union = a['total_known_union_cells']
            duplicate_fraction = a['duplicated_known_fraction']
        local_known={r: self.map_counts(r,'map')[0] for r in self.robots}
        row=self.row_time()
        row.update(
            robot1_local_known=local_known.get('robot1'),
            robot2_local_known=local_known.get('robot2'),
            robot1_shared_known=known[0] if len(known) > 0 else None,
            robot2_shared_known=known[1] if len(known) > 1 else None,
            shared_free_cells=counts[0][0],
            shared_occupied_cells=counts[0][1],
            shared_unknown_cells=counts[0][2],
            known_area_m2=(
                total_known_union * self.p['coverage_attribution_resolution']**2
                if local_union else known[0]*maps[0].info.resolution**2),
            coverage_gain_cells=gain,
            coverage_gain_since_start_cells=current-self.initial_known,
            unique_first_seen_robot1_cells=first_seen_robot1,
            unique_first_seen_robot2_cells=first_seen_robot2,
            later_duplicated_by_robot1_cells=later_duplicate_robot1,
            later_duplicated_by_robot2_cells=later_duplicate_robot2,
            simultaneously_observed_cells=simultaneous,
            total_known_union_cells=total_known_union,
            duplicated_known_fraction=duplicate_fraction,
            shared_maps_equivalent=equivalent)
        self.csv_row(self.coverage,row)
    def sample_health(self):
        limits={'odom':self.p['odom_stale_s'],'joint_states':self.p['odom_stale_s'],'scan_d500_fixed':self.p['scan_stale_s'],'scan_d500_slam':self.p['scan_stale_s'],'scan_d500_nav':self.p['scan_stale_s'],'map':self.p['map_stale_s'],'peer_map':self.p['map_stale_s'],'shared_map':self.p['shared_map_stale_s'],'frontier_candidates':self.p['candidate_stale_s'],'exploration_claim':self.p['claim_stale_s'],'exploration_status':self.p['status_stale_s'],'navigate_feedback':self.p['feedback_stale_s'],'local_costmap/costmap':self.p['costmap_stale_s'],'global_costmap/costmap':self.p['costmap_stale_s'],'cmd_vel':2.}
        now=time.monotonic()
        try:
            scan_parameters = json.loads(
                self.p['initial_configuration_json']).get(
                    'slam_runtime_parameters', {})
            configured_throttle = float(scan_parameters.get(
                'throttle_scans', 0))
        except (TypeError, ValueError, json.JSONDecodeError):
            configured_throttle = 0.0
        for r in self.robots:
            available=self.tf_buffer.can_transform(self.p['global_frame'],self.robot_base_frame(r),Time(),timeout=Duration(seconds=0.0))
            old_tf=self.tf_state.get(r)
            if self.stack_ready and old_tf!=available:
                self.event('TOPIC_RECOVERED' if available else 'TF_WARNING','shared-frame robot transform available' if available else 'shared-frame robot transform unavailable',r,'/tf',severity='INFO' if available else 'WARN',topic_name='/tf')
            self.tf_state[r]=available
            for key,limit in limits.items():
                window=self.windows.setdefault((r,key),deque())
                while window and window[0]<now-10:window.popleft()
                age=self.age(r,key); stale=age is None or age>limit; old=self.stale.get((r,key)); active=self.latest[r].get('navigation_active',False)
                if old is not None and stale!=old and (key!='navigate_feedback' or active):self.event('TOPIC_STALE' if stale else 'TOPIC_RECOVERED',f'{key} age={age}',r,f'/{r}/{key}',severity='WARN' if stale else 'INFO',topic_name=f'/{r}/{key}',topic_age_s=age)
                self.stale[(r,key)]=stale; row=self.row_time(); row.update(robot_id=r,topic_name=f'/{r}/{key}',topic_rate_hz=len(window)/10.,topic_age_s=age,expected_min_rate_hz=1/limit,stale=stale)
                if self._defer_health_history:
                    self._deferred_health_rows.append(row)
                else:
                    self.csv_row(self.health,row)
            if (self.scan_matching_enabled and configured_throttle > 1.0
                    and not self.scan_pipeline_warning_emitted):
                scan_window = self.windows.get((r, 'scan_d500_fixed'), ())
                if len(scan_window) >= 2 and len(scan_window) / 10.0 <= 2.0:
                    self.scan_pipeline_warning_emitted = True
                    self.event(
                        'SCAN_PIPELINE_THROTTLE_MISMATCH',
                        'corrected scan rate is too low for configured SLAM throttle',
                        r, severity='WARN', configured_throttle_scans=configured_throttle,
                        observed_scan_rate_hz=len(scan_window) / 10.0)
    def sample_process_resources(self):
        now=time.monotonic(); fields=Path('/proc/self/stat').read_text().split(); ticks=int(fields[13])+int(fields[14]); rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
        previous=self._cpu_previous; self._cpu_previous=(now,ticks); self._rss_samples.append(rss)
        if now-self.start<10. or previous is None:return
        elapsed=now-previous[0]
        if elapsed>0:self._cpu_samples.append((ticks-previous[1])/os.sysconf('SC_CLK_TCK')/elapsed*100.)
    def console(self):
        if not self.p['enable_console_status']:return
        for r in self.robots:
            d=self.latest[r]; p=d.get('pose',(None,None,None)); pose='?,?' if p[0] is None else f'{p[0]:.2f},{p[1]:.2f}'; self.get_logger().info(f'[{time.monotonic()-self.start:.1f}s][{r}] {d.get("claim_state","WAITING")} claim={d.get("claim_id","-")} frontier={d.get("frontier_id","-")} pose=({pose}) remaining={d.get("distance_remaining","-")} candidates={d.get("candidate_count",0)}')
    def flush(self):
        try:
            with self._io_lock:
                if self._closed:return
                self.events.flush()
                for f in self.files:f.flush()
        except OSError as e:self.write_failures+=1; self.get_logger().error(f'flush failed: {e}',throttle_duration_sec=10.)
        if self.forensic is not None:
            try:self.forensic.flush()
            except OSError as e:self.write_failures+=1; self.get_logger().error(f'forensic flush failed: {e}',throttle_duration_sec=10.)

    @staticmethod
    def _observed_rate(stamps):
        values = [float(value) for value in stamps if math.isfinite(float(value))]
        if len(values) < 2:
            return 0.0
        span = max(values) - min(values)
        return (len(values) - 1) / span if span > 0.0 else 0.0

    def write_scan_pipeline_diagnostic(self):
        """Persist effective SLAM and measured scan/map cadence evidence."""
        try:
            configuration = json.loads(self.p['initial_configuration_json'])
        except (TypeError, ValueError, json.JSONDecodeError):
            configuration = {}
        parameters = configuration.get('slam_runtime_parameters', {})
        throttle = parameters.get('throttle_scans')
        robots = {}
        mismatch = False
        for robot in self.robots:
            values = self.scan_pipeline[robot]
            scan_rate = self._observed_rate(values['scan_d500_fixed_stamps'])
            nav_stamps = list(values['scan_d500_nav_stamps'])
            nav_ages = sorted(float(value) for value in
                              values['scan_d500_nav_ages_s'])
            nav_gaps = [right - left for left, right in zip(
                nav_stamps, nav_stamps[1:]) if right >= left]
            def nav_percentile(fraction):
                if not nav_ages:
                    return None
                index = min(len(nav_ages) - 1, max(
                    0, math.ceil(len(nav_ages) * fraction) - 1))
                return nav_ages[index]
            map_rate = self._observed_rate(values['map_stamps'])
            robot_row = {
                'corrected_scan_messages': len(values['scan_d500_fixed_stamps']),
                'corrected_scan_rate_hz': scan_rate,
                # This is the scan stream consumed by collision_monitor and
                # Nav2 obstacle layers.  Ages are computed against the ROS
                # simulation clock at receipt time; no timestamp rewriting or
                # runtime gating is performed by the observer.
                'nav_scan_messages': len(nav_stamps),
                'nav_scan_rate_hz': self._observed_rate(nav_stamps),
                'nav_scan_max_source_gap_s': max(nav_gaps, default=None),
                'nav_scan_source_age_s': {
                    'samples': len(nav_ages),
                    'min': min(nav_ages, default=None),
                    'median': (statistics.median(nav_ages)
                               if nav_ages else None),
                    'p95': nav_percentile(0.95),
                    'max': max(nav_ages, default=None),
                },
                'map_messages': len(values['map_stamps']),
                'map_update_rate_hz': map_rate,
                'scan_correction_records': values['scan_correction_records'],
            }
            robots[robot] = robot_row
            if (self.scan_matching_enabled and throttle is not None
                    and float(throttle) > 1.0 and scan_rate <= 2.0):
                mismatch = True
        diagnostic = {
            'schema_version': SCHEMA,
            'scan_matching_enabled': self.scan_matching_enabled,
            'effective_slam_runtime_parameters': parameters,
            'configured_throttle_scans': throttle,
            'robots': robots,
            'preflight_status': 'FAIL_SCAN_THROTTLE_MISMATCH' if mismatch else 'PASS',
            'rate_definition': 'header-stamp span; (N-1)/(last-first)',
            'source_topics': {
                robot: f'/{robot}/scan_d500_fixed' for robot in self.robots},
            'map_topics': {
                robot: f'/{robot}/map' for robot in self.robots},
        }
        atomic_json(self.directory / 'scan_pipeline_diagnostic.json', diagnostic)
        if mismatch:
            self.event(
                'SCAN_PIPELINE_THROTTLE_MISMATCH',
                'scan pipeline preflight failed at finalization',
                severity='ERROR', configured_throttle_scans=throttle)
        return diagnostic

    def required_artifact_status(self, include_campaign_files=True):
        """Return the fail-closed artifact contract for this validation."""
        required=[]
        if self.passive_bag_enabled:
            required.extend([
                self.directory / 'passive_rosbag' / 'metadata.yaml',
                self.directory / 'passive_rosbag_export.jsonl',
                self.directory / 'passive_rosbag_export.json',
                self.directory / 'passive_rosbag_qos_overrides.yaml',
            ])
        if bool(self.p.get('enable_scientific_raw_capture', False)):
            required.extend([
                self.directory / 'map_receipts.jsonl',
                self.directory / 'coverage_requests.jsonl',
                self.directory / 'coverage.csv',
                self.directory / 'coverage_replay_parity.json',
                self.directory / 'pair_decision_replay_parity.json',
                self.directory / 'agreement_replay_parity.json',
                self.directory / 'rosout_receipts.jsonl',
                self.directory / 'warning_replay_parity.json',
                self.directory / 'nav2_diagnostic_replay_parity.json',
            ])
            if str(self.p.get('experiment_condition', '')).upper() == 'C':
                required.append(
                    self.directory / 'navigation_action_replay.json')
        if include_campaign_files:
            required.extend([self.directory/'summary.json',self.directory/'mission_result.json',self.directory/'run_manifest.json'])
        if self.scan_matching_enabled:
            required.append(self.directory / 'scan_pipeline_diagnostic.json')
        if self.forensic is not None:
            required.extend([
                self.directory/'forensic'/'transforms.csv',
            ])
            required.extend(
                self.directory/'forensic'/'maps'/f'{robot}_map_final.npz'
                for robot in self.robots)
            if self.forensic_supervisor_enabled:
                required.append(
                    self.directory/'forensic'/'supervisor_ground_truth.csv')
            if self.forensic_sync_enabled:
                required.append(
                    self.directory/'forensic'/'synchronized_map_frame.jsonl')
            if self.contact_capture:
                required.append(
                    self.directory/'forensic'/'contact_points.csv')
            # Shared-map exports are a post-handoff contract.  A valid
            # no-handoff run must not be marked incomplete merely because
            # those files correctly do not exist.  If either shared-map topic
            # was observed, require both final shared exports so a partial
            # handoff still fails closed.
            if self.shared_map_seen:
                required.extend(
                    self.directory/'forensic'/'maps'/
                    f'{robot}_shared_map_final.npz'
                    for robot in self.robots)
        try:
            unknown_pose = bool(json.loads(
                self.p['initial_configuration_json']).get(
                    'unknown_initial_pose', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            unknown_pose = False
        frontend_directory = None
        if unknown_pose:
            frontend_directory = self.frontend_diagnostic_directory()
            for robot in self.robots:
                required.extend([
                    frontend_directory / f'{robot}_unknown_pose_frontend.json',
                    frontend_directory / f'{robot}_consensus_diagnostics.jsonl',
                    frontend_directory / f'{robot}_physical_evidence_diagnostics.jsonl',
                ])
        def logical_name(path):
            if frontend_directory is not None and path.parent == frontend_directory:
                return f'frontend/{path.name}'
            try:
                return str(path.relative_to(self.directory))
            except ValueError:
                return str(path)
        missing=[logical_name(path) for path in required if not path.is_file()]
        if (self.passive_bag_enabled and
                not semantic_export_complete(
                    self.passive_bag_export,
                    include_offloaded=self.passive_sensor_offload_enabled)):
            missing.append('passive_rosbag_export:semantic-incomplete')
        result = {
            'complete': not missing,
            'status': 'COMPLETE' if not missing else 'MISSING_REQUIRED_ARTIFACTS',
            'required': [logical_name(path) for path in required],
            'missing': missing,
        }
        if frontend_directory is not None:
            try:
                result['frontend_directory'] = str(
                    frontend_directory.relative_to(self.directory.parent))
            except ValueError:
                result['frontend_directory'] = str(frontend_directory)
        if self.scan_matching_enabled and self.forensic is not None:
            required.extend([
                self.directory / 'forensic' / 'scan_matching' /
                f'{robot}_corrections.jsonl' for robot in self.robots])
            missing = [logical_name(path) for path in required
                       if not path.is_file()]
            if (self.passive_bag_enabled and
                    not semantic_export_complete(
                        self.passive_bag_export,
                        include_offloaded=self.passive_sensor_offload_enabled)):
                missing.append('passive_rosbag_export:semantic-incomplete')
            result['required'] = [logical_name(path) for path in required]
            result['missing'] = missing
            result['complete'] = not missing
            result['status'] = 'COMPLETE' if not missing else 'MISSING_REQUIRED_ARTIFACTS'
        if getattr(self, '_agreement_replay_failed', False):
            result['missing'].append(
                'agreement_replay_parity.json:deferred_replay_failed')
            result['complete'] = False
            result['status'] = 'MISSING_REQUIRED_ARTIFACTS'
        if getattr(self, '_pair_decision_replay_failed', False):
            result['missing'].append(
                'pair_decision_replay_parity.json:deferred_replay_failed')
            result['complete'] = False
            result['status'] = 'MISSING_REQUIRED_ARTIFACTS'
        if getattr(self, '_warning_replay_failed', False):
            result['missing'].append(
                'warning_replay_parity.json:deferred_replay_failed')
            result['complete'] = False
            result['status'] = 'MISSING_REQUIRED_ARTIFACTS'
        if getattr(self, '_nav2_diagnostic_replay_failed', False):
            result['missing'].append(
                'nav2_diagnostic_replay_parity.json:deferred_replay_failed')
            result['complete'] = False
            result['status'] = 'MISSING_REQUIRED_ARTIFACTS'
        if getattr(self, '_navigation_action_replay_failed', False):
            result['missing'].append(
                'navigation_action_replay.json:deferred_replay_failed')
            result['complete'] = False
            result['status'] = 'MISSING_REQUIRED_ARTIFACTS'
        return result

    def finalize_passive_rosbag(self):
        """Close the standard bag and materialize its deterministic index."""
        if not self.passive_bag_enabled:
            return None
        return_code = stop_recorder(
            self.passive_bag_process, self.passive_bag_log)
        bag_directory = self.directory / 'passive_rosbag'
        export_path = self.directory / 'passive_rosbag_export.jsonl'
        metadata = {
            'schema_version': 'passive_rosbag_finalization_1.0',
            'command': self.passive_bag_command,
            'recorder_return_code': return_code,
            'qos_profile_overrides_path': str(
                self.directory / 'passive_rosbag_qos_overrides.yaml'),
            'log_path': (None if self.passive_bag_log_path is None else
                         str(self.passive_bag_log_path)),
        }
        try:
            exported = export_index_with_bounded_retry(
                bag_directory, export_path, self.robots,
                include_offloaded=self.passive_sensor_offload_enabled,
                include_scientific_raw=bool(
                    self.p.get('enable_scientific_raw_capture', False)))
            metadata.update(exported)
            if return_code != 0:
                metadata['complete'] = False
                metadata['error'] = (
                    f'recorder exited with return code {return_code}')
            if (self.passive_sensor_offload_enabled and
                    metadata.get('complete')):
                timing = load_offloaded_timing(export_path)
                self._restore_offloaded_artifacts(timing)
                metadata['offloaded_reconstruction'] = {
                    'complete': True,
                    'topics': list(offloaded_sensor_topics(self.robots)),
                    'artifacts': [
                        'scan_pipeline_diagnostic.json',
                        'topic_health.csv',
                        'robot*_timeseries.csv',
                    ],
                }
            elif self.passive_sensor_offload_enabled:
                metadata['offloaded_reconstruction'] = {
                    'complete': False,
                    'reason': 'raw_export_incomplete',
                    'topics': list(offloaded_sensor_topics(self.robots)),
                }
        except Exception as exc:  # fail the artifact contract, never hide loss
            metadata.update({
                'complete': False,
                'error': f'{type(exc).__name__}:{exc}',
            })
            self.write_failures += 1
        atomic_json(self.directory / 'passive_rosbag_export.json', metadata)
        self.passive_bag_export = metadata
        if not metadata.get('complete', False):
            self.event('PASSIVE_ROSBAG_EXPORT_FAILED',
                       metadata.get('error', 'incomplete export'),
                       severity='ERROR', allow_during_shutdown=True)
        else:
            self.event('PASSIVE_ROSBAG_FINALIZED',
                       'raw passive navigation evidence indexed',
                       message_counts=metadata.get('message_counts', {}),
                       allow_during_shutdown=True)
        return metadata

    def _replay_navigation_actions(self):
        """Materialize normalized action/path evidence from the closed bag."""
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {'status': 'NO_SCIENTIFIC_RAW_BAG', 'complete': True}
        try:
            result = replay_navigation_evidence_from_bag(
                self.directory / 'passive_rosbag', self.robots)
        except (OSError, RuntimeError, ValueError) as exc:
            self.write_failures += 1
            self._navigation_action_replay_failed = True
            return {
                'schema_version': 'navigation_action_replay_1.0',
                'status': 'DEFERRED_REPLAY_FAILED',
                'complete': False,
                'error': f'{type(exc).__name__}:{exc}',
            }
        if not result.get('complete', False):
            self.write_failures += 1
            self._navigation_action_replay_failed = True
        return result

    @staticmethod
    def _offloaded_topic_records(timing, topic):
        values = [item for item in timing.get(topic, ())
                  if item.get('received_sim_s') is not None]
        values.sort(key=lambda item: float(item['received_sim_s']))
        return values

    @staticmethod
    def _offloaded_topic_indexes(timing):
        """Build the deferred receipt indexes once for finalization.

        The previous repair path filtered and sorted the complete receipt
        history for every health row.  Keep the same records and ordering, but
        materialize each topic once and retain a parallel timestamp sequence
        for binary-search joins.  This changes only finalization work, not the
        receipt evidence or any health boundary semantics.
        """
        records_by_topic = {}
        received_times_by_topic = {}
        for topic, values in timing.items():
            records = [item for item in values
                       if item.get('received_sim_s') is not None]
            records.sort(key=lambda item: float(item['received_sim_s']))
            records_by_topic[topic] = records
            received_times_by_topic[topic] = tuple(
                float(item['received_sim_s']) for item in records)
        return records_by_topic, received_times_by_topic

    @staticmethod
    def _record_received_times(records):
        return [float(item['received_sim_s']) for item in records]

    @staticmethod
    def _last_received_from_times(received_times, sim_time):
        index = bisect_right(received_times, float(sim_time)) - 1
        return (None if index < 0 else float(received_times[index]))

    @staticmethod
    def _last_received(records, sim_time):
        received = CooperativeExperimentLogger._record_received_times(records)
        return CooperativeExperimentLogger._last_received_from_times(
            received, sim_time)

    def _restore_offloaded_artifacts(self, timing):
        """Rebuild the old live-derived sensor fields from lossless bag rows."""
        topic_records, topic_times = self._offloaded_topic_indexes(timing)
        for robot in self.robots:
            fixed_topic = f'/{robot}/scan_d500_fixed'
            nav_topic = f'/{robot}/scan_d500_nav'
            fixed = topic_records.get(fixed_topic, ())
            nav = topic_records.get(nav_topic, ())
            if not fixed or not nav:
                raise ValueError(
                    f'missing reconstructed scan rows for {robot}')
            values = self.scan_pipeline[robot]
            values['scan_d500_fixed_stamps'].clear()
            values['scan_d500_fixed_stamps'].extend(
                float(item['header_stamp_s']) for item in fixed)
            values['scan_d500_nav_stamps'].clear()
            values['scan_d500_nav_stamps'].extend(
                float(item['header_stamp_s']) for item in nav)
            values['scan_d500_nav_ages_s'].clear()
            for item in nav:
                age = max(0.0, float(item['received_sim_s']) -
                          float(item['header_stamp_s']))
                values['scan_d500_nav_ages_s'].append(age)

        self._repair_offloaded_timeseries(timing, topic_records, topic_times)
        self._repair_offloaded_health(timing, topic_records, topic_times)

    def _repair_offloaded_timeseries(self, timing, topic_records=None,
                                     topic_times=None):
        if topic_records is None or topic_times is None:
            topic_records, topic_times = self._offloaded_topic_indexes(timing)
        for robot in self.robots:
            path = self.directory / f'{robot}_timeseries.csv'
            if not path.is_file():
                raise FileNotFoundError(path)
            topic = f'/{robot}/scan_d500_slam'
            records = topic_records.get(topic, ())
            rows = []
            with path.open(newline='', encoding='utf-8') as stream:
                reader = csv.DictReader(stream)
                fieldnames = list(reader.fieldnames or [])
                rows.extend(reader)
            if 'scan_age_s' not in fieldnames:
                raise ValueError('timeseries schema lacks scan_age_s')
            for row in rows:
                sim_time = (float(row['ros_time_sec']) +
                            float(row['ros_time_nanosec']) * 1.0e-9)
                received = self._last_received_from_times(
                    topic_times.get(topic, ()), sim_time)
                row['scan_age_s'] = '' if received is None else str(
                    max(0.0, sim_time - received))
            temporary = path.with_suffix('.csv.offload.tmp')
            with temporary.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            os.replace(temporary, path)

    def _repair_offloaded_health(self, timing, topic_records=None,
                                 topic_times=None):
        if topic_records is None or topic_times is None:
            topic_records, topic_times = self._offloaded_topic_indexes(timing)
        path = self.directory / 'topic_health.csv'
        if not path.is_file():
            raise FileNotFoundError(path)
        limits = {
            'odom': float(self.p['odom_stale_s']),
            'joint_states': float(self.p['odom_stale_s']),
            'scan_d500_fixed': float(self.p['scan_stale_s']),
            'scan_d500_slam': float(self.p['scan_stale_s']),
            'scan_d500_nav': float(self.p['scan_stale_s']),
            'map': float(self.p['map_stale_s']),
            'peer_map': float(self.p['map_stale_s']),
            'shared_map': float(self.p['shared_map_stale_s']),
            'frontier_candidates': float(self.p['candidate_stale_s']),
            'exploration_claim': float(self.p['claim_stale_s']),
            'exploration_status': float(self.p['status_stale_s']),
            'navigate_feedback': float(self.p['feedback_stale_s']),
            'cmd_vel': 2.0,
        }
        if self._defer_health_history:
            rows = [dict(row) for row in self._deferred_health_rows]
            fieldnames = list(HEALTH)
        else:
            rows = []
            with path.open(newline='', encoding='utf-8') as stream:
                reader = csv.DictReader(stream)
                fieldnames = list(reader.fieldnames or [])
                rows.extend(reader)
        source_topics = {
            'odom': lambda robot: f'/{robot}/odom',
            'joint_states': lambda robot: f'/{robot}/joint_states',
            'scan_d500_fixed': lambda robot: f'/{robot}/scan_d500_fixed',
            'scan_d500_slam': lambda robot: f'/{robot}/scan_d500_slam',
            'scan_d500_nav': lambda robot: f'/{robot}/scan_d500_nav',
            'map': lambda robot: f'/{robot}/map',
            'peer_map': lambda robot: f'/cslam/unknown_pose/{robot}/local_map',
            'shared_map': lambda robot: f'/{robot}/shared_map',
            'frontier_candidates': lambda robot: f'/{robot}/frontier_candidates',
            'exploration_claim': lambda robot: f'/cslam/{robot}/exploration_claim',
            'exploration_status': lambda robot: f'/cslam/{robot}/exploration_status',
            'navigate_feedback': lambda robot: f'/{robot}/navigate_to_pose/_action/feedback',
            'cmd_vel': lambda robot: f'/{robot}/cmd_vel',
        }
        for row in rows:
            topic = str(row.get('topic_name', ''))
            parts = topic.strip('/').split('/', 1)
            if len(parts) != 2 or parts[1] not in limits:
                continue
            key = parts[1]
            source_topic = source_topics[key](parts[0])
            received_times = topic_times.get(source_topic, ())
            sim_time = (float(row['ros_time_sec']) +
                        float(row['ros_time_nanosec']) * 1.0e-9)
            received = self._last_received_from_times(received_times, sim_time)
            age = None if received is None else max(0.0, sim_time - received)
            limit = limits[key]
            left = bisect_left(received_times, sim_time - 10.0)
            right = bisect_right(received_times, sim_time)
            rate = (right - left) / 10.0
            row['topic_rate_hz'] = str(rate)
            row['topic_age_s'] = '' if age is None else str(age)
            row['stale'] = str(age is None or age > limit)
        temporary = path.with_suffix('.csv.offload.tmp')
        for handle in list(self.files):
            if str(getattr(handle, 'name', '')) != str(path):
                continue
            with self._io_lock:
                try:
                    handle.flush()
                finally:
                    handle.close()
                self.files.remove(handle)
            break
        with temporary.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        os.replace(temporary, path)

    def frontend_diagnostic_directory(self):
        """Resolve frontend artifacts from the manifest-owned run directory.

        The launch graph can create the frontend output directory before the
        logger allocates its own collision-safe run directory.  In that case
        the logger receives ``run_id-01`` while frontend files remain under
        ``run_id/frontend``.  Resolve both locations within the same campaign
        root instead of manufacturing a missing-artifact failure.
        """
        direct = self.directory / 'frontend'
        candidates = [direct]
        parent = self.directory.parent
        # The launch runner deliberately creates the frontend directory before
        # the logger allocates its collision-safe run directory.  In that
        # normal case the files live directly under the observer root (for
        # example ``observer/frontend``), while ``self.directory`` is
        # ``observer/<run-id>``.  Include that root-level location explicitly;
        # treating only run-directory siblings as candidates makes a valid
        # frontend artifact set look missing at shutdown.
        candidates.append(parent / 'frontend')
        try:
            siblings = sorted(parent.iterdir(), key=lambda path: path.name)
        except OSError:
            siblings = []
        for sibling in siblings:
            # The launch runner may allocate the logger directory with a
            # collision suffix while the frontend keeps the manifest-owned
            # unsuffixed run directory.  Both are campaign-owned siblings
            # below this observer root; select only directories that actually
            # contain the required frontend contract.
            if sibling.is_dir() and sibling != self.directory:
                candidates.append(sibling / 'frontend')
        required_names = [
            f'{robot}_{suffix}'
            for robot in self.robots
            for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl')]
        return max(
            candidates,
            key=lambda path: (
                sum((path / name).is_file() for name in required_names),
                int(path == direct)),
        )

    def wait_for_frontend_diagnostics(self, timeout_s=10.0):
        """Allow frontend SIGINT handlers to finish before final validation."""
        try:
            unknown_pose = bool(json.loads(
                self.p['initial_configuration_json']).get(
                    'unknown_initial_pose', False))
        except (TypeError, ValueError, json.JSONDecodeError):
            unknown_pose = False
        if not unknown_pose:
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        names = [
            f'{robot}_{suffix}'
            for robot in self.robots
            for suffix in (
                'unknown_pose_frontend.json',
                'consensus_diagnostics.jsonl',
                'physical_evidence_diagnostics.jsonl')]
        while time.monotonic() < deadline:
            directory = self.frontend_diagnostic_directory()
            if all((directory / name).is_file() for name in names):
                return True
            time.sleep(0.25)
        return all(
            (self.frontend_diagnostic_directory() / name).is_file()
            for name in names)

    @staticmethod
    def runtime_worktree():
        """Find the checkout that supplied this running package."""
        # Physical migration runs are explicitly Git-free.  Do not inspect
        # repository metadata or spawn provenance subprocesses on the Pi.
        return None

    def git_value(self,args,default):
        try:
            root = self.runtime_worktree()
            if root is None:
                return default
            return subprocess.check_output(
                ['git', *args], cwd=str(root), text=True,
                stderr=subprocess.DEVNULL).strip()
        except Exception:
            return default
    def write_manifest(self,clean,status):
        try:
            seed_provenance = json.loads(self.p['seed_provenance_json'] or '{}')
        except (TypeError, ValueError):
            seed_provenance = {'invalid_seed_provenance_json': True}
        try:
            runtime_python_provenance = json.loads(
                os.getenv('MY_EPUCK_RUNTIME_PYTHON_PROVENANCE_JSON', '{}'))
        except (TypeError, ValueError):
            runtime_python_provenance = {
                'invalid_runtime_python_provenance_json': True}
        value={
            'schema_version':SCHEMA,'run_id':self.run_id,
            'utc_start_time':self.start_utc,
            'utc_end_time':utc_now() if status!='running' else None,
            'elapsed_duration_s':time.monotonic()-self.start,
            'runtime_worktree': str(self.runtime_worktree() or ''),
            'git_commit':self.git_value(['rev-parse','HEAD'],'unknown'),
            'git_branch':self.git_value(
                ['symbolic-ref','--short','-q','HEAD'], 'DETACHED'),
            'worktree_dirty':bool(self.git_value(['status','--porcelain'],'')),
            'runtime_python_provenance': runtime_python_provenance,
            'launch_file':self.p['launch_file'],
            'experiment_condition': self.p['experiment_condition'],
            'seed_provenance': seed_provenance,
            'launch_arguments':'recorded in logger parameters',
            'world_profile':self.p['world_profile'],
            'world_resource':self.p['installed_world_path'],
            'source_world_path':self.p['source_world_path'],
            'world_dimensions_m':list(self.p['world_dimensions']),
            'world_sha256':self.p['world_sha256'],
            'ros_distribution':os.getenv('ROS_DISTRO',''),
            'rmw_implementation':os.getenv('RMW_IMPLEMENTATION','default'),
            'ros_domain_id':os.getenv('ROS_DOMAIN_ID','0'),
            'hostname':socket.gethostname(),
            'logger_parameters':finite(self.p),
            'slam_resolution':self.p['slam_resolution'],
            'peer_export_resolution':self.p['slam_resolution'],
            'fusion_resolution':self.p['fusion_resolution'],
            'global_costmap_resolution':self.p['global_costmap_resolution'],
            'local_costmap_resolution':self.p['local_costmap_resolution'],
            'lidar_maximum_range':self.p['lidar_maximum_range'],
            'initial_map_costmap_configuration':json.loads(
                self.p['initial_configuration_json']),
            'robot_ids':self.robots,
            'initial_robot_poses':json.loads(
                self.p['robot_start_poses_json']),
            'known_initial_relative_transform':list(
                self.p['known_relative_transform']),
            'transform_source': self.p['transform_source'],
            'transform_frame_convention': {
                'source_frame': 'robot1_initial',
                'target_frame': 'robot2_initial',
                'meaning': 'robot2 pose expressed in robot1 initial frame',
                'transform_source': self.p['transform_source'],
                'world_sha256': self.p['world_sha256'],
            },
            'clean_shutdown':clean,'shutdown_status':status,
            'artifact_finalization':self._artifact_finalization,
        }
        atomic_json(self.directory/'run_manifest.json',value)
    def _replay_local_trajectory_for_parity(self):
        """Replay forensic odometry without changing the live authority.

        This is the first odometry salvage checkpoint: finalization computes
        the deferred result from the already-recorded forensic rows and keeps
        the existing live ``LocalTrajectory`` result authoritative until the
        two semantic summaries are equal on real artifacts.
        """
        live = (self.local_trajectory.summary()
                if hasattr(self.local_trajectory, 'summary') else None)
        if self.forensic is None:
            return {
                'live': live,
                'deferred': None,
                'equal': None,
                'status': 'NO_FORENSIC_ODOMETRY',
            }
        from .experiment_metrics import replay_local_trajectory_csv
        try:
            replayed = LocalTrajectory(
                self.p['trajectory_bin_size_m'],
                self.p['initial_overlap_exclusion_radius_m'])
            for robot in self.robots:
                source = self.directory / 'forensic' / f'{robot}_odom.csv'
                robot_replay = replay_local_trajectory_csv(
                    source, robot,
                    bin_size=self.p['trajectory_bin_size_m'],
                    exclusion_radius=self.p[
                        'initial_overlap_exclusion_radius_m'])
                for replay_robot, samples in robot_replay.samples.items():
                    replayed.samples[replay_robot] = samples
                replayed.bins.update(robot_replay.bins)
                replayed.starts.update(robot_replay.starts)
                replayed.repeated_distance.update(
                    robot_replay.repeated_distance)
                replayed.total_distance.update(robot_replay.total_distance)
                replayed.last.update(robot_replay.last)
            deferred = replayed.summary()
            self._deferred_local_trajectory = replayed
        except (OSError, ValueError) as exc:
            return {
                'live': live,
                'deferred': None,
                'equal': False,
                'status': 'DEFERRED_REPLAY_FAILED',
                'error': str(exc),
            }
        if live is None:
            return {
                'live': None,
                'deferred': deferred,
                'equal': None,
                'status': 'DEFERRED_AUTHORITATIVE',
            }
        return {
            'live': live,
            'deferred': deferred,
            'equal': live == deferred,
            'status': 'PARITY_PASS' if live == deferred else 'PARITY_FAIL',
        }

    def _replay_shared_trajectory_for_parity(self):
        """Compare causal TF2 replay with live overlap evidence.

        The native bag remains the complete payload source and is replayed as
        an integrity diagnostic.  The legacy metric's exact callback boundary
        is cross-topic causal order, which rosbag2 does not guarantee; the
        preserved forensic receipt-order rows are therefore the parity source
        for the authority comparison.
        """
        live = self.trajectory.summary()
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {
                'live': live,
                'deferred': None,
                'equal': None,
                'status': 'NO_SCIENTIFIC_RAW_BAG',
            }
        try:
            from .deferred_tf_trajectory import (
                replay_shared_trajectory_from_bag,
                replay_shared_trajectory_from_forensic_capture)
            bag_replay = replay_shared_trajectory_from_bag(
                self.directory / 'passive_rosbag', self.robots,
                self.p['global_frame'],
                bin_size=self.p['trajectory_bin_size_m'],
                exclusion_radius=self.p[
                    'initial_overlap_exclusion_radius_m'])
            causal_replay = replay_shared_trajectory_from_forensic_capture(
                self.directory / 'forensic', self.robots,
                self.p['global_frame'],
                bin_size=self.p['trajectory_bin_size_m'],
                exclusion_radius=self.p[
                    'initial_overlap_exclusion_radius_m'])
        except (OSError, RuntimeError, ValueError) as exc:
            return {
                'live': live,
                'deferred': None,
                'equal': False,
                'status': 'DEFERRED_REPLAY_FAILED',
                'error': f'{type(exc).__name__}:{exc}',
            }
        deferred = causal_replay['summary']
        if live == deferred:
            # Authority switches only after the complete causal replay has
            # matched the live semantic result.  The live object remains
            # available for the parity artifact and is not used as the final
            # summary source after this point.
            self._deferred_shared_trajectory_summary = deferred
        return {
            'live': live,
            'deferred': deferred,
            'equal': live == deferred,
            'status': 'PARITY_PASS' if live == deferred else 'PARITY_FAIL',
            'source': causal_replay['source'],
            'accepted_samples': causal_replay['accepted_samples'],
            'skipped_transform_samples': causal_replay[
                'skipped_transform_samples'],
            'causal_event_count': causal_replay['event_count'],
            'native_bag_integrity': {
                'summary': bag_replay['summary'],
                'accepted_samples': bag_replay['accepted_samples'],
                'skipped_transform_samples': bag_replay[
                    'skipped_transform_samples'],
                'deserialized_messages': bag_replay['deserialized_messages'],
                'errors': bag_replay['errors'],
            },
        }

    def _replay_coverage_for_parity(self):
        """Replay coverage and, when enabled, replace live map-derived rows.

        Scientific-raw runs retain only the timer request boundary during the
        mission.  The closed raw map bag and causal receipt ledger then
        regenerate the established ``coverage.csv`` schema during
        finalization.  The default non-raw path continues to compare against
        the live CSV exactly as before.
        """
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {
                'status': 'NO_SCIENTIFIC_RAW_BAG',
                'equal': None,
            }
        self.flush()
        request_path = self.directory / 'coverage_requests.jsonl'
        use_deferred_authority = request_path.is_file() and request_path.stat().st_size > 0
        if not use_deferred_authority:
            request_path = self.directory / 'coverage.csv'
        try:
            from .deferred_coverage import replay_coverage_from_bag
            replay = replay_coverage_from_bag(
                self.directory / 'passive_rosbag',
                self.directory / 'map_receipts.jsonl',
                request_path, self.robots,
                self.coverage_source,
                self.p['coverage_attribution_resolution'],
                self.p['known_relative_transform'],
                self.p['simultaneous_coverage_window_s'])
            if use_deferred_authority:
                self._write_deferred_coverage_csv(replay)
            with (self.directory / 'coverage.csv').open(
                    newline='', encoding='utf-8') as stream:
                live_rows = list(csv.DictReader(stream))
        except (OSError, RuntimeError, ValueError) as exc:
            return {
                'status': 'DEFERRED_REPLAY_FAILED',
                'equal': False,
                'error': f'{type(exc).__name__}:{exc}',
            }
        if use_deferred_authority:
            self._deferred_coverage_authority = replay['final_state']
            return {
                'status': 'DEFERRED_AUTHORITATIVE',
                'equal': None,
                'sample_count': replay['sample_count'],
                'source': 'native_bag_payload_plus_causal_map_receipts',
                'authority': 'deferred',
                'live_rows_during_mission': 0,
            }
        semantic_fields = (
            'robot1_local_known', 'robot2_local_known',
            'robot1_shared_known', 'robot2_shared_known',
            'shared_free_cells', 'shared_occupied_cells',
            'shared_unknown_cells', 'known_area_m2', 'coverage_gain_cells',
            'coverage_gain_since_start_cells',
            'unique_first_seen_robot1_cells',
            'unique_first_seen_robot2_cells',
            'later_duplicated_by_robot1_cells',
            'later_duplicated_by_robot2_cells',
            'simultaneously_observed_cells', 'total_known_union_cells',
            'duplicated_known_fraction', 'shared_maps_equivalent')

        def parse(field, value):
            if field == 'shared_maps_equivalent':
                return str(value).strip().lower() == 'true'
            if field in ('known_area_m2', 'duplicated_known_fraction'):
                return float(value)
            return int(float(value))

        differences = []
        if len(live_rows) != len(replay['rows']):
            differences.append({
                'kind': 'row_count', 'live': len(live_rows),
                'deferred': len(replay['rows'])})
        for live_row, deferred_row in zip(live_rows, replay['rows']):
            expected = deferred_row['semantic']
            for field in semantic_fields:
                try:
                    actual = parse(field, live_row[field])
                    proposed = expected[field]
                except (KeyError, TypeError, ValueError):
                    differences.append({
                        'row': deferred_row['row_number'],
                        'field': field, 'live': live_row.get(field),
                        'deferred': expected.get(field)})
                    continue
                if actual != proposed:
                    differences.append({
                        'row': deferred_row['row_number'],
                        'field': field, 'live': actual, 'deferred': proposed})
        result = {
            'status': 'PARITY_PASS' if not differences else 'PARITY_FAIL',
            'equal': not differences,
            'sample_count': replay['sample_count'],
            'differences': differences[:100],
            'difference_count': len(differences),
            'source': 'native_bag_payload_plus_causal_map_receipts',
        }
        if not differences:
            self._deferred_coverage_authority = replay['final_state']
        return result

    def _replay_pair_decisions_for_parity(self):
        """Compare raw pair decisions with the live outcome accounting."""
        live = dict(self.round_outcomes)
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {
                'status': 'NO_SCIENTIFIC_RAW_BAG',
                'equal': None,
                'live': live,
            }
        try:
            from .deferred_protocol import replay_pair_decisions_from_bag
            deferred = replay_pair_decisions_from_bag(
                self.directory / 'passive_rosbag', self.robots)
        except (OSError, RuntimeError, ValueError) as exc:
            return {
                'status': 'DEFERRED_REPLAY_FAILED',
                'equal': False,
                'live': live,
                'deferred': None,
                'error': f'{type(exc).__name__}:{exc}',
            }
        if (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {
                'status': 'DEFERRED_AUTHORITATIVE',
                'equal': None,
                'live': None,
                'deferred': deferred['round_outcomes'],
                'record_count': len(deferred['records']),
                'deserialized_messages': deferred['deserialized_messages'],
            }
        equal = live == deferred['round_outcomes']
        return {
            'status': 'PARITY_PASS' if equal else 'PARITY_FAIL',
            'equal': equal,
            'live': live,
            'deferred': deferred['round_outcomes'],
            'record_count': len(deferred['records']),
            'deserialized_messages': deferred['deserialized_messages'],
        }

    def _replay_agreement_counters_for_parity(self):
        """Compare raw event agreement counters with live accounting."""
        live = {
            'agreement_publications': self.agreement_publications,
            'unique_agreed_rounds': len(self.unique_agreed_rounds),
            'unique_agreed_decisions': len(self.unique_agreed_decisions),
        }
        deferred_mode = (self.passive_bag_enabled and
                         bool(self.p.get('enable_scientific_raw_capture', False)))
        if not deferred_mode:
            return {
                'status': 'NO_SCIENTIFIC_RAW_BAG',
                'equal': None,
                'live': live,
            }
        try:
            from .deferred_protocol import replay_agreement_counters_from_bag
            deferred = replay_agreement_counters_from_bag(
                self.directory / 'passive_rosbag', self.robots)
        except (OSError, RuntimeError, ValueError) as exc:
            return {
                'status': 'DEFERRED_REPLAY_FAILED',
                'equal': False,
                'live': live,
                'deferred': None,
                'error': f'{type(exc).__name__}:{exc}',
            }
        deferred_values = {
            key: deferred[key]
            for key in ('agreement_publications',
                        'unique_agreed_rounds',
                        'unique_agreed_decisions', 'round_ids',
                        'decision_hashes')}
        comparison_values = {
            key: deferred_values[key]
            for key in ('agreement_publications',
                        'unique_agreed_rounds',
                        'unique_agreed_decisions')}
        if deferred_mode:
            return {
                'status': 'DEFERRED_AUTHORITATIVE',
                'equal': None,
                'live': None,
                'deferred': deferred_values,
                'deserialized_messages': deferred['deserialized_messages'],
            }
        equal = live == comparison_values
        return {
            'status': 'PARITY_PASS' if equal else 'PARITY_FAIL',
            'equal': equal,
            'live': live,
            'deferred': deferred_values,
            'deserialized_messages': deferred['deserialized_messages'],
        }

    def _replay_warnings_for_parity(self):
        """Compare warning semantics with the observer receipt-ledger replay."""
        live_records = [asdict(record) for record in self.warns.records.values()]
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {
                'status': 'NO_SCIENTIFIC_RAW_BAG',
                'equal': None,
                'live': live_records,
                'timestamp_comparison': 'not_applicable',
            }
        try:
            from .deferred_protocol import (
                replay_warning_records_from_receipts,
                warning_record_semantics,
            )
            deferred = replay_warning_records_from_receipts(
                self.directory / 'rosout_receipts.jsonl')
        except (OSError, RuntimeError, ValueError) as exc:
            return {
                'status': 'DEFERRED_REPLAY_FAILED',
                'equal': False,
                'live': live_records,
                'deferred': None,
                'error': f'{type(exc).__name__}:{exc}',
            }
        live_semantics = warning_record_semantics(live_records)
        deferred_semantics = warning_record_semantics(deferred['records'])
        equal = live_semantics == deferred_semantics
        return {
            'status': 'PARITY_PASS' if equal else 'PARITY_FAIL',
            'equal': equal,
            'live': live_semantics,
            'deferred': deferred_semantics,
            'deferred_records': deferred['records'],
            'receipt_count': deferred['receipt_count'],
            'warning_receipt_count': deferred['warning_receipt_count'],
            'timestamp_comparison': 'semantic_only',
        }

    def _replay_nav2_diagnostics_for_parity(self):
        """Compare Nav2 diagnostic records with the receipt-ledger replay."""
        if not (self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False))):
            return {'status': 'NO_SCIENTIFIC_RAW_BAG', 'equal': None}
        try:
            from .deferred_protocol import (
                diagnostic_record_semantics,
                replay_nav2_diagnostics_from_receipts,
            )
            deferred = replay_nav2_diagnostics_from_receipts(
                self.directory / 'rosout_receipts.jsonl')
            raw_replay_authority = (
                self.passive_bag_enabled and
                bool(self.p.get('enable_scientific_raw_capture', False)))
            if raw_replay_authority:
                return {
                    'status': 'DEFERRED_AUTHORITATIVE',
                    'equal': None,
                    'live': None,
                    'deferred': diagnostic_record_semantics(
                        deferred['records']),
                    'deferred_records': deferred['records'],
                    'receipt_count': deferred['receipt_count'],
                    'diagnostic_record_count': len(deferred['records']),
                    'authority_ready': deferred['authority_ready'],
                    'timestamp_comparison': 'replayed_from_receipts',
                }
            self.nav2_diagnostics.flush()
            live = []
            path = self.directory / 'nav2_diagnostics.jsonl'
            if path.is_file():
                with path.open(encoding='utf-8') as stream:
                    live = [json.loads(line) for line in stream if line.strip()]
            live_semantics = diagnostic_record_semantics(live)
            deferred_semantics = diagnostic_record_semantics(deferred['records'])
            equal = live_semantics == deferred_semantics
            return {
                'status': 'PARITY_PASS' if equal else 'PARITY_FAIL',
                'equal': equal,
                'live': live_semantics,
                'deferred': deferred_semantics,
                'deferred_records': deferred['records'],
                'receipt_count': deferred['receipt_count'],
                'diagnostic_record_count': len(deferred['records']),
                'authority_ready': deferred['authority_ready'],
                'timestamp_comparison': 'semantic_only',
            }
        except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
            return {
                'status': 'DEFERRED_REPLAY_FAILED',
                'equal': False,
                'error': f'{type(exc).__name__}:{exc}',
            }

    def _write_replayed_jsonl(self, path, records, handle=None):
        """Write a final artifact from replayed records after closing its live handle."""
        if handle is not None:
            with self._io_lock:
                try:
                    handle.flush()
                finally:
                    handle.close()
                try:
                    self.files.remove(handle)
                except ValueError:
                    pass
        with path.open('w', encoding='utf-8') as stream:
            for record in records:
                stream.write(json.dumps(
                    finite(record), separators=(',', ':'),
                    allow_nan=False) + '\n')

    def _write_deferred_coverage_csv(self, replay):
        """Materialize the legacy coverage schema from deferred rows."""
        if self.coverage_stream is not None:
            with self._io_lock:
                self.coverage_stream.flush()
                self.coverage_stream.close()
            try:
                self.files.remove(self.coverage_stream)
            except ValueError:
                pass
            self.coverage_stream = None
        with (self.directory / 'coverage.csv').open(
                'w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=COVERAGE)
            writer.writeheader()
            for item in replay['rows']:
                request = item['request']
                row = {
                    'run_id': request.get('run_id', self.run_id),
                    'wall_time_utc': request.get('wall_time_utc', utc_now()),
                    'ros_time_sec': int(float(request['ros_time_sec'])),
                    'ros_time_nanosec': int(float(request['ros_time_nanosec'])),
                    'elapsed_s': float(request['elapsed_s']),
                    'wall_elapsed_s': float(request['wall_elapsed_s']),
                    'event_sequence': int(float(request['event_sequence'])),
                    **item['semantic'],
                }
                writer.writerow(finite(row))

    def summary(self,clean):
        elapsed=time.monotonic()-self.start
        deferred_coverage = getattr(self, '_deferred_coverage_authority', None)
        if deferred_coverage is None:
            a = self.attribution.summary()
            initial_known = self.initial_known
            previous_known = self.previous_known
        else:
            a = deferred_coverage['attribution']
            initial_known = deferred_coverage['initial_known']
            previous_known = deferred_coverage['previous_known']
        motion=self.local_trajectory.summary()
        shared_motion=getattr(
            self, '_deferred_shared_trajectory_summary',
            self.trajectory.summary())
        records = (
            list(self._deferred_warning_records)
            if self._deferred_warning_records is not None else
            list(self.warns.records.values()))
        rss=0
        try:rss=int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
        except OSError:pass
        cpu=sorted(self._cpu_samples); rss_values=self._rss_samples or [rss]
        percentile=lambda values,fraction: values[min(len(values)-1,max(0,math.ceil(len(values)*fraction)-1))] if values else 0.
        robot_states={r:{'claim_state':self.latest[r].get('claim_state','UNKNOWN'),'claim_id':self.latest[r].get('claim_id'),'frontier_id':self.latest[r].get('frontier_id'),'navigation_active':self.latest[r].get('navigation_active',False),'terminal':self.latest[r].get('terminal',False),'terminal_reason':self.latest[r].get('terminal_reason',''),'terminal_epoch':self.latest[r].get('terminal_epoch',0),'terminal_map_revision':self.latest[r].get('terminal_map_revision',0)} for r in self.robots}
        continuous={r:{'exploration_cycles':self.robot_counts[r]['EXPLORATION_CYCLE_STARTED'],'completed_goals':self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'failed_goals':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED'],'average_cycle_duration_s':statistics.fmean(self.cycle_durations[r]) if self.cycle_durations[r] else 0.,'suppression_creations':self.robot_counts[r]['FAILURE_SUPPRESSION_CREATED']+self.robot_counts[r]['SUCCESS_COOLDOWN_CREATED'],'repeated_region_attempts':sum(max(0,n-1) for n in self.region_attempts[r].values()),'maximum_equivalent_region_attempt_count':max(self.region_attempts[r].values(),default=0),'locally_exhausted_duration_s':self.exhausted_duration[r]+((time.monotonic()-self.exhausted_since[r]) if self.exhausted_since[r] is not None else 0.)} for r in self.robots}
        total_distance=sum(motion.get('distance_travelled_m',{}).values())
        shared_coverage_available=previous_known is not None
        coverage_gain=(previous_known-initial_known
                       if initial_known is not None and
                       previous_known is not None else None)
        mapping={
            'available': shared_coverage_available,
            'source': self.coverage_source,
            'reason': (f'runtime {self.coverage_source} samples'
                       if shared_coverage_available else
                       f'no runtime {self.coverage_source} samples; coverage is unavailable'),
            'initial_known_cells': initial_known,
            'final_known_cells': previous_known,
            'coverage_gain_cells': coverage_gain,
            'coverage_gain_per_metre_travelled': (
                coverage_gain/total_distance
                if coverage_gain is not None and total_distance > 0 else None),
            **a,
        }
        if self.coverage_source == 'local_map':
            first_robot = self.robots[0] if self.robots else None
            unique_first_seen = {'robot1': 0, 'robot2': 0}
            if first_robot in unique_first_seen:
                unique_first_seen[first_robot] = previous_known or 0
            mapping.update({
                'unique_first_seen_cells': unique_first_seen,
                'later_duplicated_cells': {'robot1': 0, 'robot2': 0},
                'simultaneously_observed_cells': 0,
                'total_known_union_cells': previous_known,
                'duplicated_known_fraction': 0.0,
            })
        warning_occurrences = sum(
            (record.get('occurrence_count', 1)
             if isinstance(record, dict)
             else record.occurrence_count)
            for record in records)
        return {'schema_version':SCHEMA,'run':{'run_id':self.run_id,'start_time':self.start_utc,'end_time':utc_now(),'elapsed_duration_s':elapsed,'clean_shutdown':clean},'frames':{'global_frame':self.p['global_frame'],'trajectory_source_frame':'per_robot_odom','shared_trajectory_source_frame':'global_frame (only when transform is available)','coverage_source_frame':'robot_local_map','coverage_target_frame':'robot1_initial','initial_transform_source':self.p['transform_source'],'known_initial_relative_transform':list(self.p['known_relative_transform'])},'mapping':mapping,'motion':motion,'shared_frame_motion':shared_motion,'events':dict(self.counts),'coordination':{'agreement_publications':self.agreement_publications,'unique_agreed_rounds':len(self.unique_agreed_rounds),'unique_agreed_decisions':len(self.unique_agreed_decisions),'dispatch_attempts':self.dispatch_attempts,'goals_terminal':self.goals_terminal,'goal_accounting':self.goal_accounting_summary(),'round_outcomes':dict(self.round_outcomes),'planner_query_attribution':dict(self.planner_query_counts),'planner_query_duration_s':dict(self.planner_query_duration_s)},'continuous_exploration':continuous,'mission':{'terminal':bool(self.mission_terminal_reason),'terminal_reason':self.mission_terminal_reason,'terminal_time_s':self.mission_completion_time,'shutdown_clean':clean},'mission_completion_time_s':self.mission_completion_time,'robot_terminal_state':robot_states,'navigation':{'goals_sent':self.counts['NAV_GOAL_SENT'],'goals_accepted':self.counts['NAV_GOAL_ACCEPTED'],'successes':self.counts['NAVIGATION_SUCCEEDED'],'failures':self.counts['NAVIGATION_FAILED'],'cancellations':self.counts['NAVIGATION_CANCELED'],'recoveries':self.counts['RECOVERY_COUNT_CHANGED'],'timeouts':self.counts['NAVIGATION_TIMEOUT']},'anomalies':{'no_progress_episodes':self.counts['NO_PROGRESS_STARTED'],'stuck_episodes':self.counts['STUCK_STARTED'],'stale_topic_episodes':self.counts['TOPIC_STALE'],'warning_occurrences':warning_occurrences},'system':{'logger_pid':os.getpid(),'cpu_measurement':{'scope':'logger process only','normalization':'one CPU core equals 100 percent','sampling_interval_s':1.,'warmup_s':10.,'sample_count':len(cpu),'mean_percent':statistics.fmean(cpu) if cpu else 0.,'median_percent':statistics.median(cpu) if cpu else 0.,'p95_percent':percentile(cpu,.95),'peak_percent':max(cpu,default=0.)},'logger_cpu_percent':statistics.fmean(cpu) if cpu else 0.,'logger_rss_bytes':rss,'rss_mean_bytes':statistics.fmean(rss_values),'rss_peak_bytes':max(rss_values,default=rss),'callback_timing_enabled':self._callback_timing_enabled,'callback_timing':self._callback_timing,'output_file_sizes':{p.name:p.stat().st_size for p in self.directory.iterdir() if p.is_file()},'dropped_logger_samples':self.dropped_samples,'write_failures':self.write_failures,'internal_logger_error_count':sum(self.internal_errors.values()),'internal_logger_errors':dict(self.internal_errors)},'artifact_finalization':self._artifact_finalization}

    def write_mission_result(self, clean):
        """Write one compact process-facing terminal result beside summary.json."""
        reason = self.mission_terminal_reason
        if reason.startswith('MISSION_COMPLETE_') and self._artifact_finalization.get('complete',False):
            status = 'SUCCEEDED'
            exit_code = 0
        elif reason.startswith('MISSION_COMPLETE_'):
            status = 'FAILED'
            exit_code = 1
        elif reason.startswith('MISSION_ABORT_'):
            status = 'FAILED'
            exit_code = 1
        else:
            status = 'INCOMPLETE'
            exit_code = 2
        robot_states = {
            robot: {
                'state': self.latest[robot].get('claim_state', 'UNKNOWN'),
                'terminal': self.latest[robot].get('terminal', False),
                'terminal_reason': self.latest[robot].get('terminal_reason', ''),
                'navigation_active': self.latest[robot].get(
                    'navigation_active', False),
            }
            for robot in self.robots
        }
        terminal_reasons = {
            self.latest[robot].get('terminal_reason', '')
            for robot in self.robots
            if self.latest[robot].get('terminal', False)
        }
        primary_robot = self.robots[0] if self.robots else None
        evidence = {
            key: self.latest.get(primary_robot, {}).get(key, 0)
            for key in (
                'remaining_frontier_count', 'remaining_small_frontier_count',
                'remaining_out_of_range_count', 'remaining_unreachable_count',
                'planner_failure_count', 'detected_not_queried_count',
                'below_minimum_gain_count',
                'actionable_reachable_count',
            )
        }
        mission_result = {
            'mission_status': status,
            'terminal_reason': reason or 'MISSION_NOT_TERMINATED',
            'simulated_duration_s': self.ros_seconds() - self.start_ros,
            'wall_duration_s': time.monotonic() - self.start,
            'accepted_goals': self.counts['NAV_GOAL_ACCEPTED'],
            'successful_goals': self.counts['NAVIGATION_SUCCEEDED'],
            'failed_goals': self.counts['NAVIGATION_FAILED'],
            'cancelled_goals': self.counts['NAVIGATION_CANCELED'] +
            self.counts['NAVIGATION_CANCELLED'],
            'goal_accounting': self.goal_accounting_summary(),
            'final_known_cells': self.previous_known,
            'final_known_cells_available': self.previous_known is not None,
            'mapping_metric_reason': (
                f'runtime {self.coverage_source} samples'
                if self.previous_known is not None else
                f'no runtime {self.coverage_source} samples; coverage is unavailable'),
            'semantic_agreement': bool(self.unique_agreed_rounds),
            'terminal_agreement': len(terminal_reasons) == 1,
            'robot_final_states': robot_states,
            'remaining_frontier_count': evidence['remaining_frontier_count'],
            'remaining_small_frontier_count': evidence[
                'remaining_small_frontier_count'],
            'remaining_out_of_range_count': evidence[
                'remaining_out_of_range_count'],
            'remaining_unreachable_count': evidence[
                'remaining_unreachable_count'],
            'planner_failed_count': evidence['planner_failure_count'],
            'detected_not_queried_count': evidence[
                'detected_not_queried_count'],
            'below_minimum_gain_count': evidence['below_minimum_gain_count'],
            'actionable_reachable_count': evidence[
                'actionable_reachable_count'],
            'terminal_small_frontier_length_m': float(
                self.p.get('terminal_small_frontier_length_m', 0.20)),
            'final_allocator_epoch': max(
                (self.latest[robot].get('terminal_epoch', 0)
                 for robot in self.robots), default=0,
            ),
            'final_map_revisions': {
                robot: self.latest[robot].get('terminal_map_revision', 0)
                for robot in self.robots
            },
            'terminal_time_s': self.mission_completion_time,
            'shutdown_clean': bool(clean),
            'recommended_exit_code': exit_code,
            'artifact_finalization': self._artifact_finalization,
        }
        # Keep the established two-robot field for B/C/D while allowing the
        # true single-robot A observer to finalize without manufacturing a
        # robot2 state.
        if 'robot1' in robot_states:
            mission_result['robot1_final_state'] = robot_states['robot1']
        if 'robot2' in robot_states:
            mission_result['robot2_final_state'] = robot_states['robot2']
        atomic_json(self.directory / 'mission_result.json', mission_result)
    def finalize(self,clean=True):
        with self._lifecycle_lock:
            if self.finalized or self._finalizing:return False
            self._finalizing=True
        for timer in self._observer_timers:
            try:timer.cancel()
            except Exception as exc:self.record_internal_error('timer_cancel',exc)
        if self.context.ok():
            self.event('RUN_END' if clean else 'RUN_INTERRUPTED','observer shutting down',console=True,allow_during_shutdown=True)
        if self.forensic is not None or self.contact_capture:
            if self.forensic is not None:
                self.forensic_snapshot(force=True)
                if self._defer_synchronized_map_frames:
                    # The live request ledger is complete before shutdown;
                    # reconstruct every derived row while the raw TF/odom
                    # streams are still open, then commit the normal writer
                    # manifest below.
                    self._reconstruct_deferred_synchronized_map_frames()
            self.stop_forensic_ground_truth()
            if self.forensic is not None:
                # Evidence streams must be closed before their manifest is
                # committed and before launch shutdown can terminate us.
                self.forensic.close()
                atomic_json(self.directory / 'forensic' / 'manifest.json',
                            self.forensic.manifest())
        if self.passive_bag_enabled:
            self.finalize_passive_rosbag()
        self._navigation_action_replay = self._replay_navigation_actions()
        if self._navigation_action_replay.get('status') not in (
                'NO_SCIENTIFIC_RAW_BAG',):
            atomic_json(
                self.directory / 'navigation_action_replay.json',
                self._navigation_action_replay)
        successful=False
        try:
            self._shared_trajectory_parity = (
                self._replay_shared_trajectory_for_parity())
            atomic_json(
                self.directory / 'shared_trajectory_parity.json',
                self._shared_trajectory_parity)
            self._coverage_parity = self._replay_coverage_for_parity()
            atomic_json(
                self.directory / 'coverage_replay_parity.json',
                self._coverage_parity)
            self._pair_decision_parity = (
                self._replay_pair_decisions_for_parity())
            atomic_json(
                self.directory / 'pair_decision_replay_parity.json',
                self._pair_decision_parity)
            self._agreement_parity = (
                self._replay_agreement_counters_for_parity())
            atomic_json(
                self.directory / 'agreement_replay_parity.json',
                self._agreement_parity)
            self._warning_parity = self._replay_warnings_for_parity()
            atomic_json(
                self.directory / 'warning_replay_parity.json',
                self._warning_parity)
            if self._warning_parity.get('status') not in (
                    'PARITY_PASS', 'NO_SCIENTIFIC_RAW_BAG'):
                self.write_failures += 1
                self._warning_replay_failed = True
            elif self._warning_parity.get('status') == 'PARITY_PASS':
                self._deferred_warning_records = list(
                    self._warning_parity.get('deferred_records', ()))
            self._nav2_diagnostic_parity = (
                self._replay_nav2_diagnostics_for_parity())
            atomic_json(
                self.directory / 'nav2_diagnostic_replay_parity.json',
                self._nav2_diagnostic_parity)
            nav2_status = self._nav2_diagnostic_parity.get('status')
            nav2_deferred_ready = (
                nav2_status == 'DEFERRED_AUTHORITATIVE' and
                self._nav2_diagnostic_parity.get('authority_ready', False))
            if (nav2_status not in ('PARITY_PASS', 'NO_SCIENTIFIC_RAW_BAG')
                    and not nav2_deferred_ready):
                self.write_failures += 1
                self._nav2_diagnostic_replay_failed = True
            elif (nav2_status in ('PARITY_PASS', 'DEFERRED_AUTHORITATIVE')
                  and (nav2_status == 'PARITY_PASS' or nav2_deferred_ready)):
                self._write_replayed_jsonl(
                    self.directory / 'nav2_diagnostics.jsonl',
                    self._nav2_diagnostic_parity.get('deferred_records', ()),
                    handle=self.nav2_diagnostics)
                self.nav2_diagnostics = None
            if self._agreement_parity.get('status') in (
                    'PARITY_PASS', 'DEFERRED_AUTHORITATIVE'):
                deferred_agreement = self._agreement_parity.get('deferred', {})
                self.agreement_publications = deferred_agreement.get(
                    'agreement_publications', self.agreement_publications)
                self.unique_agreed_rounds = set(
                    deferred_agreement.get('round_ids',
                                           self.unique_agreed_rounds))
                self.unique_agreed_decisions = set(
                    deferred_agreement.get('decision_hashes',
                                           self.unique_agreed_decisions))
            elif self._agreement_parity.get('status') not in (
                    'NO_SCIENTIFIC_RAW_BAG',):
                self.write_failures += 1
                self._agreement_replay_failed = True
            from .deferred_protocol import select_pair_decision_outcomes
            selected_round_outcomes, outcome_source = (
                select_pair_decision_outcomes(
                    self.round_outcomes, self._pair_decision_parity))
            if self._pair_decision_parity.get('status') not in (
                    'PARITY_PASS', 'DEFERRED_AUTHORITATIVE',
                    'NO_SCIENTIFIC_RAW_BAG'):
                self.write_failures += 1
                self._pair_decision_replay_failed = True
            self.round_outcomes = Counter(selected_round_outcomes)
            self._pair_decision_outcome_source = outcome_source
            self.flush()
            # The scan-age artifact is observer-owned and must not depend on
            # a frontend summary that may be delayed by ROS shutdown.  Emit it
            # first so a missing/late frontend file can never erase the
            # independent transport diagnostic.
            if self.scan_matching_enabled:
                self.write_scan_pipeline_diagnostic()
            if (self._callback_timing_enabled or
                    getattr(self, '_high_rate_profile_enabled', False)):
                atomic_json(
                    self.directory / 'callback_timing.json',
                    {
                        'enabled': True,
                        'scope': 'cooperative_experiment_logger safe_call callbacks',
                        'timers_and_subscriptions': self._callback_timing,
                        'high_rate_components': self._high_rate_profile,
                    },
                )
            if self._sync_map_profile_enabled:
                atomic_json(
                    self.directory / 'sync_map_timing.json',
                    self._sync_map_timing_summary(),
                )
            # ROS launch signals all children concurrently.  Frontend
            # finalizers therefore get a bounded opportunity to close their
            # JSON/JSONL streams before this observer freezes the artifact
            # contract; a missing summary is reported, never waited on
            # indefinitely.
            self.wait_for_frontend_diagnostics()
            if self.forensic is not None:
                # This is a post-run file join only.  It reads the passive
                # Supervisor/TF artifacts and never enters the ROS graph.
                self._write_physical_gt_evaluation()
            # First odometry salvage checkpoint: retain the existing live
            # summary as authority while recording a semantic comparison with
            # the deferred replay of the closed forensic odometry streams.
            self._odometry_trajectory_parity = (
                self._replay_local_trajectory_for_parity())
            atomic_json(
                self.directory / 'odometry_trajectory_parity.json',
                self._odometry_trajectory_parity)
            parity_status = self._odometry_trajectory_parity.get('status')
            if parity_status == 'DEFERRED_REPLAY_FAILED':
                self.write_failures += 1
                from .experiment_metrics import LocalTrajectory
                self.local_trajectory = LocalTrajectory(
                    self.p['trajectory_bin_size_m'],
                    self.p['initial_overlap_exclusion_radius_m'])
            elif parity_status in ('PARITY_PASS', 'DEFERRED_AUTHORITATIVE'):
                self._odometry_trajectory_live = self.local_trajectory
                self.local_trajectory = self._deferred_local_trajectory
            self._artifact_finalization=self.required_artifact_status(False)
            clean=bool(clean and self._artifact_finalization['complete'])
            with self._state_lock:
                warning_records = (
                    list(self._deferred_warning_records)
                    if self._deferred_warning_records is not None else
                    [asdict(r) for r in self.warns.records.values()])
            with open(self.directory/'warnings.jsonl','w',encoding='utf-8') as f:
                for record in warning_records:f.write(json.dumps(finite(record),allow_nan=False)+'\n')
            atomic_json(self.directory/'summary.json',self.summary(clean)); self.write_mission_result(clean)
            self.write_manifest(clean,'clean' if clean else 'interrupted')
            self._artifact_finalization=self.required_artifact_status(True)
            clean=bool(clean and self._artifact_finalization['complete'])
            # Re-emit the three campaign contracts with the final status, so a
            # missing artifact cannot be mistaken for a successful run.
            atomic_json(self.directory/'summary.json',self.summary(clean))
            self.write_mission_result(clean)
            atomic_json(self.directory/'artifact_finalization.json',self._artifact_finalization)
            self.write_manifest(clean,'clean' if clean else 'finalization_failed')
            successful=self._artifact_finalization['complete']
        except Exception as exc:
            self.write_failures+=1
            if self.context.ok():
                self.get_logger().error(f'final output failed: {exc}')
            try:
                atomic_json(self.directory/'summary.json',self.summary(False)); self.write_manifest(False,'finalization_failed')
            except Exception:
                if self.context.ok():
                    self.get_logger().error('failed to record finalization failure')
        finally:
            with self._io_lock:
                self._closed=True
                for stream in [self.events,*self.files]:
                    try:stream.flush(); stream.close()
                    except Exception as exc:
                        if self.context.ok():
                            self.get_logger().error(f'file close failed: {exc}')
            if self.forensic is not None:
                self.forensic.close()
            with self._lifecycle_lock:self.finalized=True; self._finalizing=False
        return successful

def create_logger_executor(context=None, diagnostic_frontier_capture=False):
    """Bind the logger executor to its dedicated ROS context.

    The normal observer remains single-threaded.  Opt-in frontier rejection
    capture uses a small multi-threaded executor so the passive /rosout query
    stream cannot be starved by serializing large costmap snapshots behind
    high-rate telemetry.  This branch is diagnostic-only and does not alter
    any navigation node or allocator behavior.
    """
    if diagnostic_frontier_capture:
        return MultiThreadedExecutor(num_threads=4, context=context)
    return SingleThreadedExecutor(context=context)


def main(args=None):
    context = Context()
    rclpy.init(args=args, context=context,
               signal_handler_options=SignalHandlerOptions.NO)
    node = None
    executor = None
    profiler = None
    profile_path = os.environ.get('MY_EPUCK_CPROFILE_PATH', '').strip()
    if profile_path:
        # Diagnostic-only hook.  It is deliberately opt-in and is never
        # enabled by campaign/acceptance launch configuration.
        import cProfile
        profiler = cProfile.Profile()
        profiler.enable()
    clean = True
    shutdown_requested = {'value': False}
    previous_handlers = {}

    def request_shutdown(signum, frame):
        del signum, frame
        shutdown_requested['value'] = True
        if executor is not None and context.ok():
            executor.wake()

    try:
        node = CooperativeExperimentLogger(context=context)
        executor = create_logger_executor(
            context, bool(node.p.get('diagnostic_frontier_capture', False)))
        executor.add_node(node)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_shutdown)
        while not shutdown_requested['value'] and context.ok():
            executor.spin_once(timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        shutdown_requested['value'] = True
    except RuntimeError as exc:
        if not is_shutdown_conversion_error(
                exc, shutdown_requested['value'], context.ok()):
            clean = False
            raise
    except BaseException: clean=False; raise
    finally:
        if node is not None:
            # Keep the shutdown handler installed until the complete artifact
            # finalizer returns.  The runner has a finite observer barrier and
            # may deliver a second SIGINT after that barrier expires; restoring
            # Python's default handler before finalize() turns that expected
            # escalation into KeyboardInterrupt inside native-bag replay.
            try:
                node.finalize(clean)
            finally:
                for signum, handler in previous_handlers.items():
                    signal.signal(signum, handler)
            if executor is not None:
                executor.remove_node(node)
                executor.shutdown()
            if node.context.ok():
                node.destroy_node()
        else:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
        if profiler is not None:
            profiler.disable()
            profiler.dump_stats(profile_path)
        if context.ok():
            context.shutdown()
