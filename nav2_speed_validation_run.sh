#!/usr/bin/env bash
set -Ee -o pipefail

# Temporary, no-frontier Robot 2 physical Nav2 speed validation harness.
# It owns only the processes it starts and stops them as a group on exit.

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"
[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set -u

RUN_DIR="${ROBOT2_SPEED_VALIDATION_DIR:-$HOME/robot2_speed_validation_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_DIR"/{logs,monitor,gold,diagnostics}
exec > >(tee -a "$RUN_DIR/logs/validation_runner.log") 2>&1

declare -a GROUP_PIDS=()
stop_group() {
  local pid="${1:-}"
  [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 25); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.2
  done
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}
cleanup() {
  trap - INT TERM EXIT
  for ((index=${#GROUP_PIDS[@]}-1; index>=0; index--)); do
    stop_group "${GROUP_PIDS[index]}"
  done
}
trap cleanup INT TERM EXIT

start_group() {
  local log_file="$1"
  shift
  setsid "$@" > "$RUN_DIR/logs/$log_file" 2>&1 < /dev/null &
  local pid=$!
  GROUP_PIDS+=("$pid")
  printf '%s\n' "$pid" >> "$RUN_DIR/diagnostics/owned_group_pids.txt"
  echo "started pid=$pid command=$*"
}

wait_topic() {
  local topic="$1"
  local type="$2"
  local timeout_s="$3"
  echo "waiting for $topic ($type)"
  timeout "$timeout_s" ros2 topic echo --once "$topic" "$type" >/dev/null
  echo "ready: $topic"
}

echo "RUN_DIR=$RUN_DIR"
printf 'physical_robot=Robot 2\nrun_dir=%s\nstarted_utc=%s\n' \
  "$RUN_DIR" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$RUN_DIR/diagnostics/provenance.txt"
git -C "$HOME/webots_ws/src/my_epuck_project" status --short > "$RUN_DIR/diagnostics/source_git_status.txt" 2>&1 || true
git -C "$HOME/webots_ws/src/my_epuck_project" rev-parse HEAD > "$RUN_DIR/diagnostics/source_git_head.txt" 2>&1 || true

start_group hardware.log ros2 launch my_epuck_project_cpp hardware_backend_selector.launch.py motor_backend:=cpp lidar_backend:=cpp
start_group lidar_tf.log ros2 run tf2_ros static_transform_publisher \
  --x 0.0 --y 0.0 --z 0.07 --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link --child-frame-id d500_lidar
start_group slam.log ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$HOME/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml" \
  use_sim_time:=false autostart:=true use_lifecycle_manager:=false
start_group nav2.log ros2 launch my_epuck_project real_nav2_live_slam_launch.py

wait_topic /scan sensor_msgs/msg/LaserScan 90
wait_topic /odom nav_msgs/msg/Odometry 90
wait_topic /map nav_msgs/msg/OccupancyGrid 120
wait_topic /tf tf2_msgs/msg/TFMessage 30
wait_topic /tf_static tf2_msgs/msg/TFMessage 30

ros2 node list > "$RUN_DIR/diagnostics/node_list_ready.txt"
ros2 topic list -t > "$RUN_DIR/diagnostics/topic_list_ready.txt"
ros2 lifecycle get /controller_server > "$RUN_DIR/diagnostics/controller_lifecycle_ready.txt"
ros2 lifecycle get /planner_server > "$RUN_DIR/diagnostics/planner_lifecycle_ready.txt"
ros2 lifecycle get /bt_navigator > "$RUN_DIR/diagnostics/bt_navigator_lifecycle_ready.txt"
ros2 action list -t > "$RUN_DIR/diagnostics/action_list_ready.txt"
ros2 param dump /controller_server > "$RUN_DIR/diagnostics/controller_params_ready.yaml"
ros2 param dump /planner_server > "$RUN_DIR/diagnostics/planner_params_ready.yaml"
ros2 param dump /slam_toolbox > "$RUN_DIR/diagnostics/slam_params_ready.yaml"
ros2 run tf2_ros tf2_echo map base_link > "$RUN_DIR/diagnostics/map_base_link_tf.log" 2>&1 &
TF_PID=$!
sleep 3
kill "$TF_PID" 2>/dev/null || true

python3 -u "$HOME/robot_gold_logger.py" --output-dir "$RUN_DIR/gold" \
  > "$RUN_DIR/logs/gold_logger.log" 2>&1 &
GOLD_PID=$!
GROUP_PIDS+=("$GOLD_PID")
python3 -u "$HOME/robot2_runtime_monitor.py" --output-dir "$RUN_DIR/monitor" --interval 0.5 \
  > "$RUN_DIR/logs/runtime_monitor.log" 2>&1 &
MONITOR_PID=$!
GROUP_PIDS+=("$MONITOR_PID")

printf 'ready_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$RUN_DIR/diagnostics/provenance.txt"
echo "READY: no-frontier physical Nav2 speed validation stack is running."
echo "Robot is not commanded by this harness; send only the approved nearby Nav2 goal from another terminal."
echo "Keep this terminal open; Ctrl-C stops only the owned validation processes."
while true; do sleep 1; done
