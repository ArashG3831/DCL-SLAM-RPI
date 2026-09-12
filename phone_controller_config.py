#!/usr/bin/env python3
"""Configuration for the Robot 1 phone controller."""

import os

WHEEL_RADIUS = 0.0350
WHEEL_SEPARATION = 0.22235
FORWARD_RPM = 50.0
SPIN_MAX_RADPS = 0.75
MIN_EFFECTIVE_COMMAND_RPM = 12.0

PUB_RATE_HZ = 20.0
# The browser sends a motion heartbeat every 100 ms.  Allow a transient
# HTTP/Python scheduling delay without inserting a visible zero-velocity gap.
# The production motor node independently retains its existing 0.70 s command
# timeout and its separate 0.35 s feedback safety windows.
COMMAND_TIMEOUT_S = 0.70
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
# Keep the original combined selector as a compatibility fallback, but allow
# controlled mixed-backend runs (for example Python motor + C++ lidar).
legacy_hardware_backend = os.environ.get("ROBOT1_HARDWARE_BACKEND", "").strip().lower()
if legacy_hardware_backend and legacy_hardware_backend not in {"python", "cpp"}:
    raise ValueError(
        "ROBOT1_HARDWARE_BACKEND must be 'python' or 'cpp' "
        f"(got {legacy_hardware_backend!r})"
    )
default_motor_backend = legacy_hardware_backend or "cpp"
default_lidar_backend = legacy_hardware_backend or "cpp"
MOTOR_BACKEND = os.environ.get(
    "ROBOT1_MOTOR_BACKEND", default_motor_backend
).strip().lower()
LIDAR_BACKEND = os.environ.get(
    "ROBOT1_LIDAR_BACKEND", default_lidar_backend
).strip().lower()
for backend_name, backend in (
    ("ROBOT1_MOTOR_BACKEND", MOTOR_BACKEND),
    ("ROBOT1_LIDAR_BACKEND", LIDAR_BACKEND),
):
    if backend not in {"python", "cpp"}:
        raise ValueError(f"{backend_name} must be 'python' or 'cpp' (got {backend!r})")

# Retain a useful compatibility/display value for callers that imported the
# old combined constant.  New code should use MOTOR_BACKEND/LIDAR_BACKEND.
HARDWARE_BACKEND = (
    MOTOR_BACKEND if MOTOR_BACKEND == LIDAR_BACKEND
    else f"motor={MOTOR_BACKEND},lidar={LIDAR_BACKEND}"
)
START_SLAM = os.environ.get("ROBOT1_PHONE_START_SLAM", "1").strip().lower() not in {
    "0", "false", "no", "off"
}
# The validated native hardware plus live ON/OFF comparison are the normal
# phone-controller runtime.  The environment variable remains available for
# explicitly selecting the simpler single-mapper mode.
LIVE_SLAM_COMPARISON_ENABLED = os.environ.get(
    "ROBOT1_LIVE_SLAM_COMPARISON", "1"
).strip().lower() not in {"0", "false", "no", "off"}
STACK_LOG_DIR = os.path.expanduser(
    os.environ.get("ROBOT1_PHONE_STACK_LOG_DIR", "/tmp/robot1_phone_stack")
)
PHONE_RUNTIME_MONITOR_ENABLED = os.environ.get(
    "ROBOT1_PHONE_RUNTIME_MONITOR", "1"
).strip().lower() not in {"0", "false", "no", "off"}
PHONE_RUNTIME_MONITOR_INTERVAL_S = float(
    os.environ.get("ROBOT1_PHONE_RUNTIME_MONITOR_INTERVAL_S", "1.0")
)
PHONE_RUNTIME_PROFILE_DIR = os.path.expanduser(
    os.environ.get(
        "ROBOT1_PHONE_RUNTIME_PROFILE_DIR", "~/robot2_runtime_profiles/phone_runs"
    )
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
