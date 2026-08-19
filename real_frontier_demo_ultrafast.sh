#!/usr/bin/env bash
# ULTRAFAST DEMO COPY: concurrently starts independent nodes and retains live-data, TF, lifecycle, motor-fault and completion checks.
set -o pipefail

# Usage examples:
#   bash ~/real_frontier_batch_test.sh
#   RUNS=5 RUN_TIMEOUT_SEC=600 bash ~/real_frontier_batch_test.sh
#   RUNS=5 RUN_TIMEOUT_SEC=600 MANUAL_RESET_BETWEEN_RUNS=1 bash ~/real_frontier_batch_test.sh
#
# RUN_TIMEOUT_SEC is max time per run. If exploration completes earlier, the run saves early.

RUNS="${RUNS:-5}"
RUN_TIMEOUT_SEC="${RUN_TIMEOUT_SEC:-60}"
MANUAL_RESET_BETWEEN_RUNS="${MANUAL_RESET_BETWEEN_RUNS:-0}"
SUPPRESSED_COMPLETE_SEC="${SUPPRESSED_COMPLETE_SEC:-15}"

SESSION_STAMP="$(date +%Y%m%d_%H%M%S)"
SAVE_ROOT="${SAVE_ROOT:-$HOME/robot_map_tests/session_$SESSION_STAMP}"

PORT=""
COMPLETION_WAIT_PID=""
MOTOR_FAULT_WAIT_PID=""

source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/nav2_ws/install/setup.bash" ] && source "$HOME/nav2_ws/install/setup.bash"
[ -f "$HOME/webots_ws/install/setup.bash" ] && source "$HOME/webots_ws/install/setup.bash"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_DISCOVERY_SERVER
export ROS_STATIC_PEERS=192.168.1.108
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

mkdir -p "$SAVE_ROOT"

stop_robot_cmd()
{
  timeout 2 ros2 topic pub --once /cmd_vel_unstamped geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true

  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" >/dev/null 2>&1 || true
}

kill_stack()
{
  echo "[batch] stopping robot and killing stack..."
  stop_robot_cmd

  # Kill launch/supervisor parents first so they cannot respawn children.
  pkill -INT -f '[r]eal_nav2_live_slam_launch.py' || true
  pkill -INT -f '[o]nline_async_launch.py' || true
  pkill -INT -f '[f]rontier_explorer.launch.py' || true
  pkill -INT -f '[s]tart_d500_with_recovery.sh' || true

  pkill -INT -f robot_gold_logger.py || true
  pkill -INT -f frontier_explorer || true
  pkill -INT -f controller_server || true
  pkill -INT -f planner_server || true
  pkill -INT -f bt_navigator || true
  pkill -INT -f behavior_server || true
  pkill -INT -f lifecycle_manager || true
  pkill -INT -f slam_toolbox || true
  pkill -INT -f d500_ros2_scan.py || true
  pkill -INT -f static_transform_publisher || true
  pkill -INT -f real_diffdrive_node || true

  sleep 2

  # Terminate any launch parents that ignored SIGINT.
  pkill -TERM -f '[r]eal_nav2_live_slam_launch.py' || true
  pkill -TERM -f '[o]nline_async_launch.py' || true
  pkill -TERM -f '[f]rontier_explorer.launch.py' || true
  pkill -TERM -f '[s]tart_d500_with_recovery.sh' || true

  pkill -TERM -f robot_gold_logger.py || true
  pkill -TERM -f frontier_explorer || true
  pkill -TERM -f controller_server || true
  pkill -TERM -f planner_server || true
  pkill -TERM -f bt_navigator || true
  pkill -TERM -f behavior_server || true
  pkill -TERM -f lifecycle_manager || true
  pkill -TERM -f slam_toolbox || true
  pkill -TERM -f d500_ros2_scan.py || true
  pkill -TERM -f static_transform_publisher || true
  pkill -TERM -f real_diffdrive_node || true

  sleep 1

  # Final bounded cleanup. Nothing should survive into the next run.
  pkill -KILL -f '[r]eal_nav2_live_slam_launch.py' || true
  pkill -KILL -f '[o]nline_async_launch.py' || true
  pkill -KILL -f '[f]rontier_explorer.launch.py' || true
  pkill -KILL -f '[s]tart_d500_with_recovery.sh' || true
  pkill -KILL -f robot_gold_logger.py || true
  pkill -KILL -f frontier_explorer || true
  pkill -KILL -f controller_server || true
  pkill -KILL -f planner_server || true
  pkill -KILL -f bt_navigator || true
  pkill -KILL -f behavior_server || true
  pkill -KILL -f lifecycle_manager || true
  pkill -KILL -f slam_toolbox || true
  pkill -KILL -f d500_ros2_scan.py || true
  pkill -KILL -f static_transform_publisher || true
  pkill -KILL -f real_diffdrive_node || true
}

on_exit()
{
  trap - INT TERM
  echo
  echo "[batch] interrupted/exiting — emergency motor stop"

  # Stop background topic watchers first.
  [ -n "${COMPLETION_WAIT_PID:-}" ] && kill "$COMPLETION_WAIT_PID" 2>/dev/null || true
  [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && kill "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true

  # Stop command sources and the motor node immediately. The motor node's
  # shutdown path directly sets both PWM outputs to zero.
  pkill -INT -f frontier_explorer || true
  pkill -INT -f controller_server || true
  pkill -INT -f behavior_server || true
  pkill -INT -f bt_navigator || true
  pkill -INT -f real_diffdrive_node || true
  sleep 0.4

  pkill -TERM -f frontier_explorer || true
  pkill -TERM -f controller_server || true
  pkill -TERM -f behavior_server || true
  pkill -TERM -f bt_navigator || true
  pkill -TERM -f real_diffdrive_node || true

  kill_stack
  exit 130
}
trap on_exit INT TERM

wait_for_message()
{
  local topic="$1"
  local timeout_s="$2"
  shift 2

  echo "[batch] waiting for a real message on $topic ..."

  if timeout "$timeout_s" ros2 topic echo --once "$@" "$topic" >/dev/null 2>&1; then
    echo "[batch] received a message on $topic"
    return 0
  fi

  echo "[batch] ERROR: no message received on $topic after ${timeout_s}s"
  return 1
}

wait_for_tf()
{
  local target_frame="$1"
  local source_frame="$2"
  local timeout_s="$3"

  echo "[batch] waiting for TF $target_frame -> $source_frame ..."

  if timeout "$timeout_s" bash -c \
    "ros2 run tf2_ros tf2_echo '$target_frame' '$source_frame' 2>&1 | grep -m1 -q 'Translation:'"
  then
    echo "[batch] TF $target_frame -> $source_frame is available"
    return 0
  fi

  echo "[batch] ERROR: TF $target_frame -> $source_frame unavailable after ${timeout_s}s"
  return 1
}

wait_lifecycle_active()
{
  local node="$1"
  local timeout_s="$2"
  local start_ms
  local now_ms
  local elapsed_ms
  local state=""
  local last_print_ms=-5000

  start_ms="$(date +%s%3N)"
  echo "[batch] waiting for $node active ..."

  while true; do
    state="$(timeout 1 ros2 lifecycle get "$node" 2>&1 || true)"

    if grep -q "active" <<< "$state"; then
      echo "[batch] $node is active"
      return 0
    fi

    now_ms="$(date +%s%3N)"
    elapsed_ms=$((now_ms - start_ms))

    if (( elapsed_ms >= timeout_s * 1000 )); then
      echo "[batch] ERROR: $node did not become active after ${timeout_s}s"
      echo "[batch] last lifecycle response: ${state:-<no response>}"
      return 1
    fi

    if (( elapsed_ms - last_print_ms >= 5000 )); then
      echo "[batch] still waiting for $node ($((elapsed_ms / 1000))s): ${state:-<no response>}"
      last_print_ms="$elapsed_ms"
    fi

    sleep 0.20
  done
}

start_stack()
{
  local run_dir="$1"
  local log_dir="$run_dir/logs"
  local start_ms
  local end_ms
  local scan_ok=0
  local frontier_ready=0

  local motor_pid
  local lidar_pid
  local tf_pid
  local slam_pid
  local logger_pid
  local nav2_pid
  local frontier_pid

  mkdir -p "$log_dir" "$run_dir/gold"
  start_ms="$(date +%s%3N)"

  echo "[batch] ultrafast startup: detecting D500 lidar..."

  if [ -n "${D500_PORT:-}" ]; then
    PORT="$D500_PORT"
    if [ ! -e "$PORT" ]; then
      echo "[batch] ERROR: requested D500_PORT does not exist: $PORT"
      return 1
    fi
  elif ! PORT="$("$HOME/find_d500_port.sh")"; then
    echo "[batch] ERROR: D500 lidar could not be identified."
    ls -l /dev/serial/by-id/ /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
    lsusb || true
    return 1
  fi

  echo "[batch] D500 detected at: $PORT"
  printf '%s
' "$PORT" > "$run_dir/detected_lidar_port.txt"

  sudo chmod a+rw "$PORT" || {
    echo "[batch] ERROR: could not set lidar serial permissions"
    return 1
  }

  # /scan readiness below is the definitive lidar-data test. The old raw-byte
  # probe is deliberately skipped because it opens the same serial port first.
  echo "[batch] launching motor, lidar, TF, SLAM, logger, Nav2 and explorer concurrently..."

  MOTOR_SAFETY_LOG_DIR="$run_dir/gold" \
  MOTOR_SAFETY_LOG_NAME="motor_safety.csv" \
    ros2 run my_epuck_project real_diffdrive_node \
    > "$log_dir/01_motor.log" 2>&1 &
  motor_pid=$!

  D500_SELECTED_PORT_FILE="$run_dir/detected_lidar_port.txt" \
    "$HOME/start_d500_with_recovery.sh" \
    > "$log_dir/02_lidar.log" 2>&1 &
  lidar_pid=$!

  ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.07 \
    --roll 0.0 --pitch 0.0 --yaw 0.0 \
    --frame-id base_link \
    --child-frame-id d500_lidar \
    > "$log_dir/03_static_tf.log" 2>&1 &
  tf_pid=$!

  ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$HOME/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml" \
    use_sim_time:=false \
    > "$log_dir/04_slam.log" 2>&1 &
  slam_pid=$!

  python3 -u "$HOME/robot_gold_logger.py" \
    --output-dir "$run_dir/gold" \
    > "$log_dir/00_gold_logger.log" 2>&1 &
  logger_pid=$!

  ros2 launch my_epuck_project real_nav2_live_slam_launch.py \
    > "$log_dir/05_nav2.log" 2>&1 &
  nav2_pid=$!

  ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py \
    params_file:="$HOME/webots_ws/src/my_epuck_project/resource/frontier_explorer_real_balanced.yaml" \
    > "$log_dir/08_frontier.log" 2>&1 &
  frontier_pid=$!

  cat > "$run_dir/startup_pids.txt" <<EOF
motor=$motor_pid
lidar=$lidar_pid
static_tf=$tf_pid
slam=$slam_pid
gold_logger=$logger_pid
nav2=$nav2_pid
frontier=$frontier_pid
EOF

  # Give exec failures a fraction of a second to become visible, not several seconds.
  sleep 0.25

  local name
  local pid
  local logfile

  while IFS=: read -r name pid logfile; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "[batch] ERROR: $name exited during concurrent startup."
      tail -120 "$logfile" 2>/dev/null || true
      return 1
    fi
  done <<EOF
motor:$motor_pid:$log_dir/01_motor.log
lidar:$lidar_pid:$log_dir/02_lidar.log
static_tf:$tf_pid:$log_dir/03_static_tf.log
slam:$slam_pid:$log_dir/04_slam.log
gold_logger:$logger_pid:$log_dir/00_gold_logger.log
nav2:$nav2_pid:$log_dir/05_nav2.log
frontier:$frontier_pid:$log_dir/08_frontier.log
EOF

  echo "[batch] waiting on readiness gates while all nodes continue starting..."

  if timeout 25 ros2 topic echo --once \
    --qos-reliability best_effort \
    --qos-durability volatile \
    /scan sensor_msgs/msg/LaserScan \
    >/dev/null 2>&1
  then
    scan_ok=1
    echo "[batch] /scan is live"
  fi

  if [ "$scan_ok" -ne 1 ]; then
    echo "[batch] ERROR: no live /scan message"
    tail -120 "$log_dir/02_lidar.log" 2>/dev/null || true
    return 1
  fi

  wait_for_message /odom 20 || {
    tail -120 "$log_dir/01_motor.log" 2>/dev/null || true
    return 1
  }

  wait_for_message /map 45 \
    --qos-durability transient_local \
    --qos-reliability reliable || {
      tail -160 "$log_dir/04_slam.log" 2>/dev/null || true
      return 1
    }

  wait_for_tf map base_link 45 || {
    tail -160 "$log_dir/04_slam.log" 2>/dev/null || true
    return 1
  }

  # Do not repeatedly launch the slow ROS CLI during heavy concurrent startup.
  # The lifecycle manager itself prints this only after every managed Nav2 node
  # has successfully activated.
  echo "[batch] waiting for Nav2 managed nodes to become active..."
  local nav2_ready=0

  for _ in $(seq 1 150); do
    if grep -q "Managed nodes are active" "$log_dir/05_nav2.log" 2>/dev/null; then
      nav2_ready=1
      break
    fi

    if ! kill -0 "$nav2_pid" 2>/dev/null; then
      echo "[batch] ERROR: Nav2 launch process exited during startup."
      tail -160 "$log_dir/05_nav2.log" 2>/dev/null || true
      return 1
    fi

    sleep 0.20
  done

  if [ "$nav2_ready" -ne 1 ]; then
    echo "[batch] ERROR: Nav2 did not activate after 30 seconds."
    tail -160 "$log_dir/05_nav2.log" 2>/dev/null || true
    return 1
  fi

  echo "[batch] Nav2 managed nodes are active"

  # The explorer requires the global costmap before it can dispatch a goal.
  wait_for_message /global_costmap/costmap 20 || {
    tail -160 "$log_dir/05_nav2.log" 2>/dev/null || true
    return 1
  }

  # Avoid another ROS CLI discovery loop. The node emits this line only
  # after its parameters, publishers, subscriptions and runtime are initialized.
  echo "[batch] waiting for frontier explorer initialization..."
  frontier_ready=0

  for _ in $(seq 1 100); do
    if grep -q "Frontier explorer initialized"       "$log_dir/08_frontier.log" 2>/dev/null
    then
      frontier_ready=1
      break
    fi

    if ! kill -0 "$frontier_pid" 2>/dev/null; then
      echo "[batch] ERROR: frontier explorer exited during startup."
      tail -160 "$log_dir/08_frontier.log" 2>/dev/null || true
      return 1
    fi

    sleep 0.10
  done

  if [ "$frontier_ready" -ne 1 ]; then
    echo "[batch] ERROR: frontier explorer did not initialize after 10 seconds."
    tail -160 "$log_dir/08_frontier.log" 2>/dev/null || true
    return 1
  fi

  echo "[batch] frontier explorer is initialized"

  end_ms="$(date +%s%3N)"
  echo "[batch] ULTRAFAST STACK READY in $(((end_ms - start_ms) / 1000)).$((((end_ms - start_ms) % 1000) / 100))s"
}

wait_for_run_end()
{
  local run_dir="$1"
  local completion_file="$run_dir/exploration_complete_msg.txt"
  local fault_file="$run_dir/motor_safety_fault_msg.txt"
  local start_s
  local now_s
  local completion_rc
  local fault_rc
  local frontier_log="$run_dir/logs/08_frontier.log"
  local suppression_since=0
  local tracked_suppression_line=0
  local current_suppression_line=0
  local current_goal_line=0
  local motor_idle=0

  : > "$completion_file"
  : > "$fault_file"

  timeout "$RUN_TIMEOUT_SEC" ros2 topic echo --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /frontier_explorer/exploration_complete std_msgs/msg/Empty \
    > "$completion_file" 2>&1 &
  COMPLETION_WAIT_PID=$!

  timeout "$RUN_TIMEOUT_SEC" ros2 topic echo --once \
    --qos-reliability reliable \
    --qos-durability transient_local \
    /motor_safety/fault std_msgs/msg/String \
    > "$fault_file" 2>&1 &
  MOTOR_FAULT_WAIT_PID=$!

  start_s="$(date +%s)"

  while true; do
    if [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && \
       ! kill -0 "$MOTOR_FAULT_WAIT_PID" 2>/dev/null; then
      wait "$MOTOR_FAULT_WAIT_PID"
      fault_rc=$?
      MOTOR_FAULT_WAIT_PID=""

      if [ "$fault_rc" -eq 0 ] && grep -q 'data:' "$fault_file"; then
        echo "[batch] MOTOR SAFETY FAULT received."
        sed 's/^/[batch]   /' "$fault_file" || true
        echo "motor safety fault" > "$run_dir/run_ended_by_motor_fault.txt"

        # The motor node has already latched both motors off. Stop exploration
        # immediately so Nav2 stops producing new goals while data is saved.
        pkill -INT -f frontier_explorer || true

        [ -n "${COMPLETION_WAIT_PID:-}" ] && \
          kill "$COMPLETION_WAIT_PID" 2>/dev/null || true
        [ -n "${COMPLETION_WAIT_PID:-}" ] && \
          wait "$COMPLETION_WAIT_PID" 2>/dev/null || true
        COMPLETION_WAIT_PID=""
        return 2
      fi
    fi

    if [ -n "${COMPLETION_WAIT_PID:-}" ] && \
       ! kill -0 "$COMPLETION_WAIT_PID" 2>/dev/null; then
      wait "$COMPLETION_WAIT_PID"
      completion_rc=$?
      COMPLETION_WAIT_PID=""

      if [ "$completion_rc" -eq 0 ]; then
        echo "[batch] exploration completion message received."
        [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && \
          kill "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true
        [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && \
          wait "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true
        MOTOR_FAULT_WAIT_PID=""
        return 0
      fi
    fi

    now_s="$(date +%s)"

    # The explorer waits indefinitely when all raw frontiers are suppressed.
    # Treat a sustained suppressed + stationary state as practical completion.
    if [ -f "$frontier_log" ]; then
      current_suppression_line="$(
        grep -n           'All currently detected frontiers are temporarily suppressed'           "$frontier_log" 2>/dev/null |
          tail -1 |
          cut -d: -f1
      )"

      current_goal_line="$(
        grep -n 'Sending frontier goal'           "$frontier_log" 2>/dev/null |
          tail -1 |
          cut -d: -f1
      )"

      current_suppression_line="${current_suppression_line:-0}"
      current_goal_line="${current_goal_line:-0}"

      if (( current_suppression_line > current_goal_line )); then
        if (( tracked_suppression_line != current_suppression_line )); then
          tracked_suppression_line="$current_suppression_line"
          suppression_since="$now_s"

          echo             "[batch] all current frontiers suppressed; "             "starting ${SUPPRESSED_COMPLETE_SEC}s completion hold..."

        elif (( suppression_since > 0 &&
                now_s - suppression_since >= SUPPRESSED_COMPLETE_SEC )); then

          motor_idle="$(
            tail -1 "$run_dir/gold/motor_safety.csv" 2>/dev/null |
              awk -F, '
                NF >= 17 {
                  left = $3 + 0.0
                  right = $17 + 0.0

                  if (left < 0) left = -left
                  if (right < 0) right = -right

                  print (left < 0.2 && right < 0.2) ? 1 : 0
                }
              '
          )"

          if [ "${motor_idle:-0}" = "1" ]; then
            echo "[batch] sustained suppressed-frontier state with motors idle."
            echo "[batch] treating exploration as complete and saving the map."

            printf '%s
'               "All detected frontiers remained suppressed for ${SUPPRESSED_COMPLETE_SEC}s while motors were idle."               > "$run_dir/exploration_complete_by_suppression.txt"

            [ -n "${COMPLETION_WAIT_PID:-}" ] &&               kill "$COMPLETION_WAIT_PID" 2>/dev/null || true

            [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] &&               kill "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true

            [ -n "${COMPLETION_WAIT_PID:-}" ] &&               wait "$COMPLETION_WAIT_PID" 2>/dev/null || true

            [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] &&               wait "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true

            COMPLETION_WAIT_PID=""
            MOTOR_FAULT_WAIT_PID=""
            return 3
          fi
        fi
      else
        tracked_suppression_line=0
        suppression_since=0
      fi
    fi

    if (( now_s - start_s >= RUN_TIMEOUT_SEC )); then
      echo "[batch] timeout reached. Saving map anyway."

      [ -n "${COMPLETION_WAIT_PID:-}" ] && \
        kill "$COMPLETION_WAIT_PID" 2>/dev/null || true
      [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && \
        kill "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true
      [ -n "${COMPLETION_WAIT_PID:-}" ] && \
        wait "$COMPLETION_WAIT_PID" 2>/dev/null || true
      [ -n "${MOTOR_FAULT_WAIT_PID:-}" ] && \
        wait "$MOTOR_FAULT_WAIT_PID" 2>/dev/null || true

      COMPLETION_WAIT_PID=""
      MOTOR_FAULT_WAIT_PID=""
      return 1
    fi

    sleep 0.20
  done
}

save_run_data()
{
  local run_dir="$1"
  local log_dir="$run_dir/logs"

  echo "[batch] stopping exploration motion before map save..."
  stop_robot_cmd
  pkill -INT -f frontier_explorer || true
  sleep 2

  echo "[batch] saving map to $run_dir/map ..."
  ros2 run nav2_map_server map_saver_cli \
    -f "$run_dir/map" \
    --ros-args \
    -p map_topic:=/map \
    > "$log_dir/09_map_saver.log" 2>&1 || {
      echo "[batch] ERROR: map_saver_cli failed. See $log_dir/09_map_saver.log"
    }

  echo "[batch] saving topic list and parameter dumps..."
  ros2 topic list -t > "$run_dir/topics.txt" 2>&1 || true

  ros2 param dump /slam_toolbox > "$run_dir/slam_toolbox_params_dump.yaml" 2>/dev/null || true
  ros2 param dump /controller_server > "$run_dir/controller_server_params_dump.yaml" 2>/dev/null || true
  ros2 param dump /planner_server > "$run_dir/planner_server_params_dump.yaml" 2>/dev/null || true
  ros2 param dump /frontier_explorer > "$run_dir/frontier_explorer_params_dump.yaml" 2>/dev/null || true

  cp "$HOME/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml" "$run_dir/" 2>/dev/null || true
  cp "$HOME/webots_ws/src/my_epuck_project/resource/nav2_real_live_slam.yaml" "$run_dir/" 2>/dev/null || true
  cp "$HOME/webots_ws/src/my_epuck_project/resource/frontier_explorer_real_balanced.yaml" "$run_dir/" 2>/dev/null || true
  cp "$HOME/robot_gold_logger.py" "$run_dir/" 2>/dev/null || true
  cp "$HOME/webots_ws/src/my_epuck_project/my_epuck_project/real_diffdrive_node.py"     "$run_dir/" 2>/dev/null || true

  if [ -s "$run_dir/gold/motor_safety.csv" ]; then
    echo "[batch] motor safety CSV saved: $run_dir/gold/motor_safety.csv"
  else
    echo "[batch] WARNING: motor safety CSV missing or empty"
    echo "missing or empty" > "$run_dir/motor_safety_csv_problem.txt"
  fi

  echo "[batch] saved run data:"
  ls -lh "$run_dir" | sed 's/^/[batch]   /'
}

echo "[batch] session directory: $SAVE_ROOT"
echo "[batch] runs: $RUNS"
echo "[batch] max seconds per run: $RUN_TIMEOUT_SEC"
echo "[batch] manual reset between runs: $MANUAL_RESET_BETWEEN_RUNS"
echo


if pgrep -f \
  '[r]eal_nav2_live_slam_launch.py|[o]nline_async_launch.py|[f]rontier_explorer.launch.py|[r]eal_diffdrive_node|[d]500_ros2_scan.py|[s]lam_toolbox|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[s]tatic_transform_publisher' \
  >/dev/null 2>&1
then
  echo "[batch] old robot stack detected; cleaning it first..."
  kill_stack
else
  echo "[batch] no old robot stack detected; skipping initial cleanup."
fi

# Ultrafast demo: preserve the current ROS daemon.
# Ultrafast demo: preserve the current ROS daemon.

for i in $(seq 1 "$RUNS"); do
  RUN_DIR="$SAVE_ROOT/run_$(printf "%02d" "$i")"
  mkdir -p "$RUN_DIR/logs"

  echo
  echo "============================================================"
  echo "[batch] RUN $i / $RUNS"
  echo "============================================================"

  if [ "$MANUAL_RESET_BETWEEN_RUNS" = "1" ]; then
    echo
    echo "[batch] Place robot at the same physical start pose, then press ENTER."
    read -r _
  fi

  if ! start_stack "$RUN_DIR"; then
    echo "[batch] ERROR: stack startup failed during run $i."
    echo "[batch] Nav2 diagnostic tail:"
    tail -120 "$RUN_DIR/logs/05_nav2.log" 2>/dev/null || true
    echo "stack startup failed" > "$RUN_DIR/startup_failed.txt"
    kill_stack
    exit 1
  fi

  echo "[batch] exploration running."
  echo "[batch] waiting for completion, motor fault, or ${RUN_TIMEOUT_SEC}s timeout..."

  wait_for_run_end "$RUN_DIR"
  RESULT=$?

  case "$RESULT" in
    0)
      echo "[batch] run ended normally: exploration complete"
      ;;
    1)
      echo "[batch] run ended normally: timeout"
      ;;
    2)
      echo "[batch] run aborted safely: motor fault"
      ;;
    3)
      echo "[batch] run ended normally: all remaining frontiers stayed suppressed"
      ;;
    *)
      echo "[batch] run wait ended unexpectedly with code $RESULT"
      ;;
  esac

  save_run_data "$RUN_DIR"
  kill_stack

  timeout 3 ros2 daemon stop >/dev/null 2>&1 || true
  timeout 3 ros2 daemon start >/dev/null 2>&1 || true

  sleep 0.5
done

echo
echo "[batch] ALL RUNS DONE"
echo "[batch] saved here: $SAVE_ROOT"
echo "[batch] map files:"
find "$SAVE_ROOT" -name "map.yaml" -o -name "map.pgm" | sort
