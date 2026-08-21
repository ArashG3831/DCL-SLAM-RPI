# Robot 1 native-port behavior inventory

This document records the committed Python implementations used as the
behavioral oracle for `my_epuck_project_cpp`. It is intentionally factual; it
does not authorize changing any production default.

## Motor node

| Item | Python oracle |
|---|---|
| Node name | `real_diffdrive_node` |
| Subscriptions | `/cmd_vel` (`geometry_msgs/TwistStamped`), `/cmd_vel_unstamped` (`geometry_msgs/Twist`) |
| Publications | `/odom` (`nav_msgs/Odometry`), `/motor_safety/fault` (`std_msgs/String`) |
| Frames | `odom` -> `base_link`; TF is broadcast with the odometry pose |
| Control clock | dedicated `time.monotonic()` deadline loop, 20 Hz, 50 ms nominal period |
| Hardware callbacks | gpiozero X4 callbacks on both A/B channels; native active factory is `gpiozero.pins.lgpio.LGPIOFactory` |
| Geometry | radius 0.0350 m; CPR 4606; command/odom separation 0.22235 m |
| Wheel signs | right/M1 encoder +1 and motor +1; left/M2 encoder -1 and motor -1 |
| PID | PI, `KP=0.020`, `KI=0.020`, integral clamp [-30, 30], estimate ceiling 55 RPM |
| Command limits | target +/-50 RPM; zero epsilon 0.15 RPM; effective minimum 12 RPM |
| Command timeout | 0.70 s |
| Startup grace | 0.60 s after a stopped-to-moving or direction-changing wheel |
| Safety window | 0.35 s |
| Safety thresholds | target >=10 RPM, PWM >=0.22, A/B window >=20, channel ratio >=0.35 |
| No-pulse timeout | both A and B ages >=0.35 s |
| Additional safety | max encoder RPM 90; latch after 2 implausible cycles; severe underspeed ratio 0.20 at PWM 0.65 for 0.80 s; optional direction check disabled |
| Fault behavior | latch first-priority fault, command both wheels to zero, ignore later commands until process restart |
| Odom | midpoint differential-drive integration, yaw wrapped to [-pi, pi], covariance and frame IDs as in source |

## D500 node

| Item | Python oracle |
|---|---|
| Node name | `d500_ros2_scan` |
| Input | serial port, 230400 baud by production wrapper, timeout 50 ms |
| Packet | `0x54 0x2c`, 47 bytes, 12 points, CRC-8 polynomial 0x4d |
| Parser | stream resynchronization at header; partial reads and arbitrary chunking supported |
| Scan assembler | 720 bins, first observed angle starts a revolution, decreasing angle by >180 degrees closes it |
| Filtering | distance 30..12000 mm, intensity >=0, nearest valid range wins per bin |
| Handoff | one-slot latest-complete-scan handoff; replacement increments drop counter |
| ROS path | dedicated serial/parser thread; 5 ms nonblocking publication timer |
| Timestamp | host acquisition interval midpoint, not publication time |
| Output | `/scan`, `sensor_msgs/LaserScan`, production frame `d500_lidar`, mirror transform preserved |
| Timing fields | `scan_time` from acquisition interval; `time_increment=scan_time/720` |

## Native-port scope

The C++ implementations are opt-in executables. The Python executables remain
the production default until golden-vector, parser, live lidar-only, and
explicitly authorized motor validation are complete.
