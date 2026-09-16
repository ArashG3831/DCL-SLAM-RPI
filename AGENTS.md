
## Persisted user directive: read-only latest-artifact Smac investigation

look dumbass. even with a crazy detailed 0.5cm local map, rpp was still not happy. very barely. until last second that it ACTUALLY HIT A WALL VERY SLIGHTLY. BEFORE RPP STOPPED IT. SO THERE IS A SERIOUS ISSUE WITH SMAC NOT DOING ITS JOB PROPERLY. INSPECT THE LATEST ARTIFACTS. DO NOT COME BACK UNTIL YOU HAVE A CONCRETE ANSWER. keep this read only. look up online when needed. spawn subagents set to luna high to even do deeper read only investigations for you. NO THE WALL DIDNT APPEAR OUT OF NO WHERE. THAT. IS. NOT. THE. ANSWER!!! BY THE END, I NEED A CONCRETE FINAL ANSWER ON WHY IT CHOOSES WRONG PATHS. logs: "arash@DESKTOP-OVP2LHB:~$ ~/robot2_visual
remote_runner=179941
[WARN] [1789512365.026967956] [rcl]: ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled. Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead.
[WARN] [1789512365.026997956] [rcl]: ROS_LOCALHOST_ONLY is deprecated but still honored if it is enabled. Use ROS_AUTOMATIC_DISCOVERY_RANGE and ROS_STATIC_PEERS instead.
[   0.1s] LIVE_REPORTER connected (read-only)
Robot 2 custom-frontier minimal-allocator run is remote and remains alive after RViz closes.
Use ~/robot2_stop for the explicit remote stop.
[  16.5s] FRONTIERS remaining=11 reachable=1
[  16.6s] STATUS bidding: solo startup spin active
[  17.1s] STATUS bidding: solo startup spin succeeded
[  17.2s] STATUS bidding: solo startup spin complete; updated snapshot received
[  17.2s] GOAL selected: id=11612572061505354901 approach=(0.158,-0.881) cost=2.643 path=2.643m
[  17.2s] GOAL navigating: id=11612572061505354901
[  25.0s] STATUS bidding: goal failure
[  30.1s] FRONTIERS remaining=0 reachable=1
[  30.2s] GOAL selected: id=15129409959263670948 approach=(-0.964,-0.318) cost=1.560 path=1.560m
[  30.2s] GOAL navigating: id=15129409959263670948
[  39.6s] STATUS bidding: goal failure
[  42.0s] FRONTIERS remaining=0 reachable=1
[  42.1s] GOAL selected: id=3113138530945688887 approach=(-1.003,0.186) cost=1.506 path=1.506m
[  42.1s] GOAL navigating: id=3113138530945688887
[  52.5s] STATUS bidding: goal success
[  56.2s] FRONTIERS remaining=11 reachable=1
[  56.2s] GOAL selected: id=125004559581254698 approach=(1.415,-0.429) cost=3.091 path=3.091m
[  56.2s] GOAL navigating: id=125004559581254698
[  62.1s] STATUS bidding: goal failure
[  67.5s] STATUS bidding: goal failure
[  71.1s] STATUS bidding: goal failure
[  71.5s] FRONTIERS remaining=1 reachable=0
[  71.5s] STATUS waiting: no eligible solo candidate after preflight
[  72.4s] STATUS waiting: IDLE
[  72.4s] STATUS waiting: no eligible solo candidate after preflight
[  74.3s] STATUS waiting: IDLE
[  74.3s] STATUS waiting: no eligible solo candidate after preflight
[  76.3s] STATUS waiting: IDLE
[  76.5s] STATUS complete: MISSION_COMPLETE_NO_REACHABLE_FRONTIERS"

# Robot 2 Project Guide for Coding Agents

minimal_frontier_allocator is the only true decentralized task allocator, all
else are either old or broken.
use the verified files in /home/robot1/cooperative_migration_source;

This file is durable project guidance for coding agents. It records project
context, non-obvious invariants, safe commands, and validation conventions. It
is not a ROS launch file and it does not change runtime behavior by itself.

## Physical and Linux identity

- Physical platform: **Robot 2**.
- Hostname: `robot2`.
- Linux login user: `robot2`.
- Home directory intentionally remains `/home/robot1` for compatibility.
- Do not rename or migrate `/home/robot1` without an explicit migration task.
- Historical files may still say Robot 1 or `robot1`; do not rewrite them
  unless they are active configuration or the task explicitly requests it.

## Production hardware and SLAM baseline

Use the native C++ hardware backend for the physical Robot 2 stack:

- C++ motor/encoder executable: `real_diffdrive_node_cpp`.
- C++ D500 executable: `d500_ros2_scan_cpp`.
- Do not launch the Python motor and C++ motor nodes together.
- Do not launch the Python D500 and C++ D500 nodes together.
- Do not use the old Python hardware paths or historical frontier launchers as
  the production launcher.

Frozen calibration and interfaces:

- wheel radius: `0.0350 m`
- encoder CPR: `4606` X4 transitions/revolution
- command and odometry wheel separation: `0.22235 m`
- D500: `720` bins, approximately `10 Hz`, frame `d500_lidar`
- static transform: `base_link -> d500_lidar`, `(x,y,z)=(0,0,0.07)` and zero
  rotation
- motor control: dedicated monotonic 20 Hz control thread
- keep the validated encoder signs, PID, safety timeout, GPIO mapping, lidar
  filtering, mirror convention, and timestamp semantics unchanged

Authoritative SLAM YAML:

`/home/robot1/webots_ws/src/my_epuck_project/resource/slam_toolbox_real_d500.yaml`

Required production SLAM settings:

```yaml
resolution: 0.03
map_update_interval: 1.0
use_scan_matching: true
do_loop_closing: false
correlation_search_space_dimension: 0.12
correlation_search_space_resolution: 0.01
correlation_search_space_smear_deviation: 0.015
distance_variance_penalty: 0.05
angle_variance_penalty: 0.05235987755982989
minimum_distance_penalty: 0.15
minimum_angle_penalty: 0.70
coarse_search_angle_offset: 0.05235987755982989
coarse_angle_resolution: 0.017453292519943295
fine_search_angle_offset: 0.003490658503988659
use_response_expansion: false
```

## Full autonomous exploration run

Run this on the Raspberry Pi over SSH. It starts the native C++ motor and
D500 nodes, finalized Slam Toolbox, Nav2, frontier exploration, rosbag
recording, and runtime monitoring:

```bash
ROBOT2_MAX_RUNTIME_S=1800 ~/robot2_full_visual_run.sh
```

The runner uses:

```text
robot2_cpp_slam_frontier_launch.py
  -> C++ hardware backend selector
  -> D500 static TF
  -> Slam Toolbox
  -> direct physical Nav2 launch
  -> frontier_explorer
```

The runner is supervised and idempotent. It refuses to create duplicate
hardware, SLAM, Nav2, frontier, recorder, monitor, or Zenoh processes when an
existing healthy run is detected. It also refuses to override conflicting
unmanaged processes.

Do not use these obsolete launchers as the production path:

- `real_frontier_batch_test.sh`
- `real_frontier_demo_ultrafast.sh`
- historical Python motor/D500 recovery launchers

## Visualization and stopping

`robot2_visual` and `robot2_stop` are **local WSL helper scripts**, not Pi
files. Run them from the WSL terminal on the Windows computer:

```bash
~/robot2_visual
```

This starts or reuses the supervised remote run, starts the read-only Zenoh
visualization bridge, and opens local RViz. Closing RViz does not stop Robot 2.

Stop the remote supervised run from WSL with:

```bash
~/robot2_stop
```

The WSL helper must SSH as `robot2@192.168.1.111`. Remote project paths still
use `/home/robot1`.

The frontier-only control command is different:

```bash
ros2 run frontier_exploration_ros2 frontier_exploration_ctl stop
ros2 run frontier_exploration_ros2 frontier_exploration_ctl start
```

`frontier_exploration_ctl stop` only disables frontier processing; it does not
stop the full hardware/SLAM/Nav2 run. `stop -q` additionally requests the
frontier node to exit, but still is not the full-run stop command.

If the WSL helper is unavailable, `Ctrl+C` in the Pi runner terminal causes
the supervised runner to finalize and stop its owned children. Never use broad
`pkill` patterns against the robot.

## Artifacts

Each supervised run creates a timestamped directory under:

```text
/home/robot1/robot2_frontier_exploration_results/robot2_full_run_YYYYMMDD_HHMMSS/
```

Expected contents include:

- `configs/`: effective YAML, launch, provenance, and copied monitoring code
- `raw/`: selected-topic rosbag
- `gold/`: event and telemetry logs
- `monitor/`: CPU, memory, temperature, and runtime samples
- `diagnostics/`: nodes, topics, lifecycle, parameters, TF, and Zenoh status
- `logs/`: stack, recorder, monitor, map saver, and completion logs
- `map/`: final `PGM` and map `YAML`

Do not commit bags, maps, logs, `build/`, `install/`, runtime result folders,
or large generated benchmark artifacts.

## Current completed capabilities

- Calibrated differential-drive odometry and validated D500 runtime exist.
- Native C++ motor/encoder and D500 ports exist and have automated/native
  regression validation plus hardware-side validation history.
- Conservative local Slam Toolbox scan matching was validated in a same-bag
  OFF/ON experiment; loop closure remains disabled.
- Production map resolution is 3 cm and the production map update interval is
  1 second.
- The physical Robot 2 Nav2 live-SLAM launch exists and uses Navfn, DWB,
  lifecycle management, live `/map`, and no AMCL/static map server.
- The existing frontier explorer is integrated through
  `robot2_cpp_slam_frontier_launch.py`.
- Current source configuration uses a maximum forward speed of `0.165 m/s`,
  frontier `max_linear_speed_vmax: 0.165`, and current global/local inflation
  radius `0.1620 m`; preserve these unless a new evidence-based task requests
  a change.
- The supervised launcher records reproducible run artifacts and provides a
  read-only WSL/RViz visualization path.
- Production promotion of conservative scan matching and 1 s map updates is
  recorded in commit `3a735c0`.

## Remaining/maintenance status

- The repository currently contains substantial uncommitted source, config,
  runtime, and generated artifacts. Inspect `git status` before editing or
  committing; do not assume every file in the workspace belongs in a commit.
- Before a new physical run, verify that no stale stack is active and that the
  effective installed package/config matches the source configuration.
- A full-house result should be judged from its timestamped artifacts and
  quantitative logs, not only from RViz appearance.
- Do not begin CSLAM, multi-robot coordination, or downstream exploration
  redesign while the single-robot baseline is under test.

## Agent working rules

- Start with read-only archaeology for unfamiliar runtime behavior.
- Preserve calibrated production behavior; avoid unrelated cleanup/refactors.
- Make backups before modifying production configuration or hardware code.
- Prefer existing launchers, analyzers, and artifact formats over duplicates.
- Run syntax checks, relevant tests, affected package builds, and `git diff
  --check` after implementation changes.
- Never deliberately command physical motion unless the user has explicitly
  authorized that test in the current task.
