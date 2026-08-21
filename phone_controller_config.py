#!/usr/bin/env python3
"""Configuration for the Robot 1 phone controller."""

import os

WHEEL_RADIUS = 0.0350
WHEEL_SEPARATION = 0.22235
FORWARD_RPM = 50.0
SPIN_MAX_RADPS = 0.75
MIN_EFFECTIVE_COMMAND_RPM = 12.0

PUB_RATE_HZ = 20.0
COMMAND_TIMEOUT_S = 0.35
TRANSITION_REST_S = 0.50
HTTP_PORT = 8080
MAP_POLL_MS = 250
POSE_POLL_MS = 50
MAX_MAP_DISPLAY_DIM = 1200
LIDAR_GAP_THRESHOLD_S = 0.40

SLAM_PARAMS = os.path.expanduser(
    os.environ.get(
        "ROBOT1_SLAM_PARAMS",
        "~/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml",
    )
)
START_LIDAR_SCRIPT = os.path.expanduser(
    os.environ.get("ROBOT1_LIDAR_START_SCRIPT", "~/start_d500_with_recovery.sh")
)
START_MOTOR = os.environ.get("ROBOT1_PHONE_START_MOTOR", "1").strip().lower() not in {
    "0", "false", "no", "off"
}
START_LIDAR = os.environ.get("ROBOT1_PHONE_START_LIDAR", "1").strip().lower() not in {
    "0", "false", "no", "off"
}
STACK_LOG_DIR = os.path.expanduser(
    os.environ.get("ROBOT1_PHONE_STACK_LOG_DIR", "/tmp/robot1_phone_stack")
)
MAP_SAVE_DIR = os.path.expanduser(os.environ.get("ROBOT1_MAP_SAVE_DIR", "~/robot1_maps"))
MAP_CHECKPOINT_INTERVAL_S = 10.0
MAP_SAVE_TIMEOUT_S = 15.0

ODOM_RESULTS_DIR = os.path.expanduser(
    os.environ.get("ROBOT1_ODOM_RESULTS_DIR", "~/robot1_odom_drift_results")
)
ODOM_START_REFERENCE_SAMPLES = 10
ODOM_MODE_CONFIGURATION_GRACE_S = 10.0
ODOM_FINAL_REFERENCE_SAMPLES = 10
ODOM_FINALIZE_TIMEOUT_S = 1.5
ODOM_MIN_STATIONARY_LINEAR_MPS = 0.02
ODOM_MIN_STATIONARY_ANGULAR_RADPS = 0.05
ODOM_ANALYZER = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "odom_drift_analyzer.py"
)
SLAM_MATCHING_EXPERIMENT_ENABLED = os.environ.get(
    "ROBOT1_SLAM_MATCHING_EXPERIMENT", "1"
).strip().lower() not in {"0", "false", "no", "off"}
