#!/usr/bin/env bash
set -Ee -o pipefail

# One supervised Robot 2 C++ hardware + SLAM + Nav2 + frontier run.
# RViz is deliberately headless here; the WSL companion command runs local RViz.

SCRIPT_PATH="$(readlink -f "$0")"
MAX_RUNTIME_S="${ROBOT2_MAX_RUNTIME_S:-1800}"
RUNNER_PID_FILE="/tmp/robot2_full_visual_runner.pid"
ACTIVE_RUN_FILE="/tmp/robot2_full_visual_active_run"
READY_RUN_FILE="/tmp/robot2_full_visual_ready"
RUN_LOCK_FILE="/tmp/robot2_full_visual_run.lock"
STACK_PID_FILE="/tmp/robot2_full_visual_stack.pid"
BAG_PID_FILE="/tmp/robot2_full_visual_bag.pid"
GOLD_PID_FILE="/tmp/robot2_full_visual_gold.pid"
MONITOR_PID_FILE="/tmp/robot2_full_visual_monitor.pid"
BRIDGE_PID_FILE="/tmp/robot2_zenoh_bridge.pid"
BRIDGE_OWNER_FILE="/tmp/robot2_zenoh_bridge.owner"
BRIDGE_ROOT="$HOME/.local/zenoh-bridge-ros2dds/1.10.0"
BRIDGE_CONFIG="$HOME/.local/zenoh-bridge-ros2dds/robot2_rviz_readonly.json5"

process_is_runner() {
  local pid="${1:-}"
  local command_line=""
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  [ -r "/proc/$pid/cmdline" ] || return 1
  command_line="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  [[ "$command_line" == *"$SCRIPT_PATH"* ]]
}

active_run_dir() {
  local runner_pid=""
  local run_dir=""
  local stack_pid=""
  local bag_pid=""
  local gold_pid=""
  local monitor_pid=""
  local bridge_pid=""
  local ready_runner=""
  local ready_dir=""
  [ -s "$RUNNER_PID_FILE" ] || return 1
  runner_pid="$(<"$RUNNER_PID_FILE")"
  process_is_runner "$runner_pid" || return 1
  [ -s "$ACTIVE_RUN_FILE" ] || return 1
  run_dir="$(<"$ACTIVE_RUN_FILE")"
  [ -d "$run_dir" ] || return 1
  if [ -s "$STACK_PID_FILE" ]; then
    stack_pid="$(<"$STACK_PID_FILE")"
    kill -0 "$stack_pid" 2>/dev/null || return 1
    ps -p "$stack_pid" -o args= 2>/dev/null | grep -Fq "robot2_cpp_slam_frontier_launch.py" || return 1
  fi
  if [ -s "$BAG_PID_FILE" ]; then
    bag_pid="$(<"$BAG_PID_FILE")"
    kill -0 "$bag_pid" 2>/dev/null || return 1
    ps -p "$bag_pid" -o args= 2>/dev/null | grep -Eq "ros2 bag record|rosbag2_recorder" || return 1
  fi
  if [ -s "$GOLD_PID_FILE" ]; then
    gold_pid="$(<"$GOLD_PID_FILE")"
    kill -0 "$gold_pid" 2>/dev/null || return 1
    ps -p "$gold_pid" -o args= 2>/dev/null | grep -Fq "robot_gold_logger.py" || return 1
  fi
  if [ -s "$MONITOR_PID_FILE" ]; then
    monitor_pid="$(<"$MONITOR_PID_FILE")"
    kill -0 "$monitor_pid" 2>/dev/null || return 1
    ps -p "$monitor_pid" -o args= 2>/dev/null | grep -Fq "robot2_runtime_monitor.py" || return 1
  fi
  if [ -s "$READY_RUN_FILE" ]; then
    ready_runner="$(sed -n '1p' "$READY_RUN_FILE")"
    ready_dir="$(sed -n '2p' "$READY_RUN_FILE")"
    [ "$ready_runner" = "runner_pid=$runner_pid" ] || return 1
    [ "$ready_dir" = "run_dir=$run_dir" ] || return 1
  fi
  if [ -s "$BRIDGE_PID_FILE" ]; then
    bridge_pid="$(<"$BRIDGE_PID_FILE")"
    kill -0 "$bridge_pid" 2>/dev/null || return 1
    ps -p "$bridge_pid" -o args= 2>/dev/null | grep -Fq "$BRIDGE_ROOT/zenoh-bridge-ros2dds" || return 1
    grep -Fqx "runner_pid=$runner_pid" "$BRIDGE_OWNER_FILE" 2>/dev/null || return 1
  fi
  printf '%s\n' "$run_dir"
}

cleanup_stale_state() {
  local runner_pid=""
  local stack_pid=""
  if [ -s "$RUNNER_PID_FILE" ]; then
    runner_pid="$(<"$RUNNER_PID_FILE")"
    if ! process_is_runner "$runner_pid"; then
      echo "Removing stale runner state: $RUNNER_PID_FILE (pid=$runner_pid)"
      rm -f "$RUNNER_PID_FILE"
    fi
  fi
  if [ -s "$ACTIVE_RUN_FILE" ] && ! active_run_dir >/dev/null 2>&1; then
    echo "Removing stale active-run state: $ACTIVE_RUN_FILE"
    rm -f "$ACTIVE_RUN_FILE"
    rm -f "$READY_RUN_FILE"
  fi
  if [ -s "$STACK_PID_FILE" ]; then
    stack_pid="$(<"$STACK_PID_FILE")"
    if ! kill -0 "$stack_pid" 2>/dev/null; then
      echo "Removing stale stack state: $STACK_PID_FILE (pid=$stack_pid)"
      rm -f "$STACK_PID_FILE"
    fi
  fi
  if [ -s "$BRIDGE_PID_FILE" ]; then
    local bridge_pid="$(<"$BRIDGE_PID_FILE")"
    if ! kill -0 "$bridge_pid" 2>/dev/null; then
      echo "Removing stale bridge state: $BRIDGE_PID_FILE (pid=$bridge_pid)"
      rm -f "$BRIDGE_PID_FILE" "$BRIDGE_OWNER_FILE"
    fi
  fi
  for state_file in "$BAG_PID_FILE" "$GOLD_PID_FILE" "$MONITOR_PID_FILE"; do
    if [ -s "$state_file" ]; then
      local child_pid="$(<"$state_file")"
      if ! kill -0 "$child_pid" 2>/dev/null; then
        echo "Removing stale process state: $state_file (pid=$child_pid)"
        rm -f "$state_file"
      fi
    fi
  done
}

unmanaged_process_conflicts() {
  local pid=""
  local command_line=""
  local conflicts=""
  while read -r pid command_line; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    process_is_runner "$pid" && continue
    case "$command_line" in
      *real_diffdrive_node_cpp*|*d500_ros2_scan_cpp*|*real_diffdrive_node.py*|*d500_ros2_scan.py*|\
      *async_slam_toolbox_node*|*controller_server*|*planner_server*|*behavior_server*|\
      *bt_navigator*|*frontier_explorer*|*robot2_cpp_slam_frontier_launch.py*|\
      *ros2\ bag\ record*|*rosbag2_recorder*|*robot_gold_logger.py*|*robot2_runtime_monitor.py*)
        conflicts+="pid=$pid $command_line\n"
        ;;
    esac
  done < <(ps -eo pid=,args= 2>/dev/null || true)
  printf '%b' "$conflicts"
}

unmanaged_bridge_conflicts() {
  local pid=""
  local command_line=""
  while read -r pid command_line; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    process_is_runner "$pid" && continue
    case "$command_line" in
      *zenoh-bridge-ros2dds*)
        printf 'pid=%s %s\n' "$pid" "$command_line"
        ;;
    esac
  done < <(ps -eo pid=,args= 2>/dev/null || true)
}

check_start_conditions() {
  local active=""
  local conflicts=""
  local bridge_conflicts=""
  cleanup_stale_state
  if active="$(active_run_dir 2>/dev/null)"; then
    echo "Existing healthy supervised Robot 2 run detected; reusing: $active"
    exit 0
  fi
  conflicts="$(unmanaged_process_conflicts)"
  if [ -n "$conflicts" ]; then
    echo "ERROR: unmanaged Robot 2 stack processes already exist; refusing to launch duplicates."
    printf '%b' "$conflicts"
    echo "Stop only the identified old run explicitly, then retry. Nothing was killed."
    exit 2
  fi
  bridge_conflicts="$(unmanaged_bridge_conflicts)"
  if [ -n "$bridge_conflicts" ]; then
    echo "ERROR: an unmanaged Zenoh bridge already exists; refusing to launch a duplicate listener."
    printf '%s' "$bridge_conflicts"
    echo "Stop only the identified old bridge explicitly, then retry. Nothing was killed."
    exit 2
  fi
}

exec 9>"$RUN_LOCK_FILE"
if ! flock -n 9; then
  for _ in $(seq 1 30); do
    if active_run_dir >/dev/null 2>&1; then
      echo "Another supervised Robot 2 runner is starting; reusing its active run: $(active_run_dir)"
      exit 0
    fi
    sleep 1
  done
  echo "ERROR: another runner holds $RUN_LOCK_FILE but no healthy active run appeared."
  exit 2
fi
check_start_conditions

STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${ROBOT2_RUN_DIR:-$HOME/robot2_frontier_exploration_results/robot2_full_run_${STAMP}}"

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"
[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"

set -u

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p "$RUN_DIR"/{configs,diagnostics,gold,logs,map,monitor,raw}
exec > >(tee -a "$RUN_DIR/logs/runner.log") 2>&1
printf '%s\n' "$$" > "$RUNNER_PID_FILE"
printf '%s\n' "$RUN_DIR" > "$ACTIVE_RUN_FILE"

STACK_PID=""
BAG_PID=""
GOLD_PID=""
MONITOR_PID=""
WATCH_PID=""
BRIDGE_PID=""
FINISHED=0

copy_snapshot() {
  cp "$HOME/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml" "$RUN_DIR/configs/"
  cp "$HOME/webots_ws/src/my_epuck_project/resource/nav2_real_live_slam.yaml" "$RUN_DIR/configs/"
  cp "$HOME/webots_ws/src/my_epuck_project/resource/frontier_explorer_real_balanced.yaml" "$RUN_DIR/configs/"
  cp "$HOME/webots_ws/src/my_epuck_project/launch/robot2_cpp_slam_frontier_launch.py" "$RUN_DIR/configs/"
  cp "$HOME/robot_gold_logger.py" "$RUN_DIR/configs/"
  cp "$HOME/robot2_runtime_monitor.py" "$RUN_DIR/configs/"
  {
    echo "physical_robot=Robot 2"
    echo "linux_user=$USER"
    echo "started_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "git_head=$(git -C "$HOME" rev-parse HEAD 2>/dev/null || true)"
    echo "git_status:"
    git -C "$HOME" status --short 2>/dev/null || true
    echo "ros_domain_id=$ROS_DOMAIN_ID"
    echo "rmw_implementation=$RMW_IMPLEMENTATION"
    echo "launch=ros2 launch my_epuck_project robot2_cpp_slam_frontier_launch.py rviz:=false"
    echo "runner_pid_file=$RUNNER_PID_FILE"
    echo "bag_topics=/scan /odom /tf /tf_static /map /cmd_vel /cmd_vel_unstamped /plan /received_global_plan /global_costmap/costmap /local_costmap/costmap /frontier_explorer/frontiers /frontier_explorer/selected_frontier /frontier_explorer/optimized_map /frontier_explorer/exploration_complete /navigate_to_pose/_action/status"
  } > "$RUN_DIR/configs/provenance.txt"
}

start_bridge() {
  local bridge_pid=""
  if [ ! -x "$BRIDGE_ROOT/zenoh-bridge-ros2dds" ] || [ ! -f "$BRIDGE_CONFIG" ]; then
    echo "ERROR: Pi Zenoh bridge binary/config unavailable."
    return 1
  fi
  for denied_kind in subscribers service_servers service_clients action_servers action_clients; do
    if ! grep -Eq "^[[:space:]]*${denied_kind}:[[:space:]]*\[\][,]?[[:space:]]*$" "$BRIDGE_CONFIG"; then
      echo "ERROR: Zenoh bridge policy is not read-only: ${denied_kind} is not an empty allow-list."
      return 2
    fi
  done
  if grep -Eq '/cmd_vel|navigate_to_pose|control_exploration' "$BRIDGE_CONFIG"; then
    echo "ERROR: Zenoh bridge read-only policy unexpectedly mentions a physical control interface."
    return 2
  fi
  if [ -s "$BRIDGE_PID_FILE" ] && kill -0 "$(<"$BRIDGE_PID_FILE")" 2>/dev/null; then
    bridge_pid="$(<"$BRIDGE_PID_FILE")"
    if ! ps -p "$bridge_pid" -o args= | grep -Fq "$BRIDGE_ROOT/zenoh-bridge-ros2dds"; then
      echo "ERROR: Zenoh PID file does not identify the expected bridge."
      return 1
    fi
    if ! grep -Fqx "runner_pid=$$" "$BRIDGE_OWNER_FILE" 2>/dev/null; then
      echo "ERROR: Zenoh PID file identifies an existing bridge that is not owned by this supervised run."
      echo "pid=$bridge_pid"
      echo "owner_file=$BRIDGE_OWNER_FILE (missing or different runner)"
      echo "Nothing was killed. Stop that bridge explicitly before retrying."
      return 2
    fi
    echo "Pi Zenoh bridge already running and verified under this runner: pid=$bridge_pid"
  else
    rm -f "$BRIDGE_PID_FILE" "$BRIDGE_OWNER_FILE"
    if [ -n "$(unmanaged_bridge_conflicts)" ]; then
      echo "ERROR: an unmanaged Zenoh bridge appeared before startup; refusing duplicate listener."
      unmanaged_bridge_conflicts
      return 2
    fi
    setsid env ROS_DOMAIN_ID=0 "$BRIDGE_ROOT/zenoh-bridge-ros2dds" \
      -c "$BRIDGE_CONFIG" -d 0 -l tcp/0.0.0.0:7447 --no-multicast-scouting \
      > "$RUN_DIR/logs/zenoh_bridge.log" 2>&1 < /dev/null &
    bridge_pid=$!
    echo "$bridge_pid" > "$BRIDGE_PID_FILE"
    printf 'runner_pid=%s\nconfig=%s\nendpoint=tcp/0.0.0.0:7447\n' \
      "$$" "$BRIDGE_CONFIG" > "$BRIDGE_OWNER_FILE"
    echo "Started Pi Zenoh bridge: pid=$bridge_pid"
  fi
  BRIDGE_PID="$bridge_pid"
  for _ in $(seq 1 20); do
    if ss -ltn | grep -Eq ':7447[[:space:]]'; then
      printf 'pid=%s\nrunner_pid=%s\nendpoint=tcp/0.0.0.0:7447\nconfig=%s\npolicy=READ_ONLY_NO_SUBSCRIBERS_SERVICES_ACTIONS\nstatus=LISTENING\n' \
        "$bridge_pid" "$$" "$BRIDGE_CONFIG" > "$RUN_DIR/diagnostics/zenoh_bridge_status.txt"
      return 0
    fi
    sleep 0.5
  done
  echo "ERROR: Pi Zenoh bridge did not open TCP 7447."
  tail -80 "$RUN_DIR/logs/zenoh_bridge.log" 2>/dev/null || true
  return 1
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  echo "Waiting for $topic ..."
  if timeout "$timeout_s" ros2 topic echo --once "$topic" >/dev/null 2>&1; then
    echo "Received $topic"
    return 0
  fi
  echo "WARNING: did not receive $topic within ${timeout_s}s"
  return 1
}

stop_group() {
  local pid="$1"
  [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.2
  done
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

save_diagnostics() {
  echo "Saving final map and diagnostics..."
  timeout 25 ros2 run nav2_map_server map_saver_cli \
    -f "$RUN_DIR/map/final_map" \
    --ros-args -p map_topic:=/map \
    > "$RUN_DIR/logs/map_saver.log" 2>&1 || true
  ros2 topic list -t > "$RUN_DIR/diagnostics/topic_list.txt" 2>&1 || true
  ros2 node list > "$RUN_DIR/diagnostics/node_list.txt" 2>&1 || true
  ros2 action list -t > "$RUN_DIR/diagnostics/action_list.txt" 2>&1 || true
  ros2 param dump /slam_toolbox > "$RUN_DIR/diagnostics/slam_toolbox_params.yaml" 2>/dev/null || true
  ros2 param dump /controller_server > "$RUN_DIR/diagnostics/controller_server_params.yaml" 2>/dev/null || true
  ros2 param dump /planner_server > "$RUN_DIR/diagnostics/planner_server_params.yaml" 2>/dev/null || true
  ros2 param dump /frontier_explorer > "$RUN_DIR/diagnostics/frontier_explorer_params.yaml" 2>/dev/null || true
  ros2 lifecycle get /controller_server > "$RUN_DIR/diagnostics/controller_lifecycle.txt" 2>&1 || true
  ros2 lifecycle get /planner_server > "$RUN_DIR/diagnostics/planner_lifecycle.txt" 2>&1 || true
  ros2 lifecycle get /bt_navigator > "$RUN_DIR/diagnostics/bt_navigator_lifecycle.txt" 2>&1 || true
  ros2 run tf2_tools view_frames -o "$RUN_DIR/diagnostics/frames" \
    > "$RUN_DIR/logs/view_frames.log" 2>&1 || true
}

finish() {
  [ "$FINISHED" -eq 1 ] && return
  FINISHED=1
  trap - INT TERM EXIT
  echo "Finishing Robot 2 run: $RUN_DIR"
  [ -n "$WATCH_PID" ] && kill "$WATCH_PID" 2>/dev/null || true
  # Stop navigation immediately, then preserve the final map and artifacts.
  timeout 2 ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
    >/dev/null 2>&1 || true
  # Save while SLAM is still alive, then stop all owned processes.
  save_diagnostics
  stop_group "$GOLD_PID"
  stop_group "$BAG_PID"
  stop_group "$MONITOR_PID"
  stop_group "$STACK_PID"
  if [ -n "$BRIDGE_PID" ] && [ -s "$BRIDGE_OWNER_FILE" ] && \
     grep -Fqx "runner_pid=$$" "$BRIDGE_OWNER_FILE" 2>/dev/null; then
    stop_group "$BRIDGE_PID"
    rm -f "$BRIDGE_PID_FILE" "$BRIDGE_OWNER_FILE"
  fi
  echo "finished_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)" >> "$RUN_DIR/configs/provenance.txt"
  find "$RUN_DIR" -maxdepth 2 -type f -printf '%P\n' | sort > "$RUN_DIR/artifact_list.txt"
  if [ -s "$ACTIVE_RUN_FILE" ] && [ "$(<"$ACTIVE_RUN_FILE")" = "$RUN_DIR" ]; then
    rm -f "$ACTIVE_RUN_FILE"
    rm -f "$READY_RUN_FILE"
  fi
  if [ -s "$RUNNER_PID_FILE" ] && [ "$(<"$RUNNER_PID_FILE")" = "$$" ]; then
    rm -f "$RUNNER_PID_FILE" "$STACK_PID_FILE" "$BAG_PID_FILE" "$GOLD_PID_FILE" "$MONITOR_PID_FILE"
  fi
  echo "ARTIFACT_DIR=$RUN_DIR"
}
trap finish INT TERM EXIT

copy_snapshot
start_bridge

echo "Starting native Robot 2 stack..."
setsid ros2 launch my_epuck_project robot2_cpp_slam_frontier_launch.py rviz:=false \
  > "$RUN_DIR/logs/stack.log" 2>&1 < /dev/null &
STACK_PID=$!
echo "$STACK_PID" > "$STACK_PID_FILE"

wait_for_topic /scan 90 || true
wait_for_topic /odom 90 || true
wait_for_topic /map 120 || true

echo "Starting selected-topic rosbag recording..."
setsid ros2 bag record --storage sqlite3 -o "$RUN_DIR/raw/rosbag" \
  /scan /odom /tf /tf_static /map /cmd_vel /cmd_vel_unstamped \
  /plan /received_global_plan /global_costmap/costmap /local_costmap/costmap \
  /frontier_explorer/frontiers /frontier_explorer/selected_frontier \
  /frontier_explorer/optimized_map /frontier_explorer/exploration_complete \
  /navigate_to_pose/_action/status \
  > "$RUN_DIR/logs/rosbag_record.log" 2>&1 < /dev/null &
BAG_PID=$!
echo "$BAG_PID" > "$BAG_PID_FILE"

python3 -u "$HOME/robot_gold_logger.py" --output-dir "$RUN_DIR/gold" \
  > "$RUN_DIR/logs/gold_logger.log" 2>&1 &
GOLD_PID=$!
echo "$GOLD_PID" > "$GOLD_PID_FILE"

python3 -u "$HOME/robot2_runtime_monitor.py" --output-dir "$RUN_DIR/monitor" \
  > "$RUN_DIR/logs/runtime_monitor.log" 2>&1 &
MONITOR_PID=$!
echo "$MONITOR_PID" > "$MONITOR_PID_FILE"

printf 'runner_pid=%s\nrun_dir=%s\n' "$$" "$RUN_DIR" > "$READY_RUN_FILE"

echo "Monitoring frontier completion for up to ${MAX_RUNTIME_S}s..."
timeout "$MAX_RUNTIME_S" ros2 topic echo --once \
  /frontier_explorer/exploration_complete std_msgs/msg/Empty \
  > "$RUN_DIR/logs/exploration_complete.log" 2>&1 &
WATCH_PID=$!
wait "$WATCH_PID" || true
WATCH_PID=""
echo "Completion watcher ended; finalizing run."
