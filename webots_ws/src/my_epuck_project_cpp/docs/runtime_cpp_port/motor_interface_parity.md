# Motor native-port parity inventory

Reference: `my_epuck_project.real_diffdrive_node` at baseline commit `3be3371`.

This package is opt-in. The Python executable remains the production default.

| Surface | Python reference | C++ port status |
|---|---|---|
| Node name | `real_diffdrive_node` | same |
| Subscriptions | `/cmd_vel` `TwistStamped`; `/cmd_vel_unstamped` `Twist`; depth 10 | same |
| Publications | `/odom` `nav_msgs/Odometry`; depth 10 | same |
| Safety publication | `/motor_safety/fault`, reliable/transient-local depth 1 | same |
| TF | `odom -> base_link`, same stamp as odometry | same |
| Control clock | dedicated 20 Hz monotonic deadline thread | same design |
| GPIO | gpiozero `LGPIOFactory`, chip 0 | native liblgpio API 0.2.2, chip 0 |
| Encoder | full X4, no debounce, pull-up inputs | full X4, no debounce, pull-up inputs |
| Python defaults | radius .0350, CPR 4606, command/odom separation .22235, safety defaults from source | same defaults are declared and applied to the testable C++ core; hardware validation remains pending |
| PID | KP .020, KI .020, 50 ms update, integral clamp [-30, 30], startup command behavior | golden-vector PASS; no tuning changes |
| Safety | channel/no-pulse/direction/implausible-RPM/underspeed candidates with priority 10/20/30/40/50 | synthetic safety tests PASS; live fault/latch behavior still requires authorized motor validation |
| Diagnostics | bounded timing/safety diagnostics and asynchronous Python CSV writer | C++ low-rate timing logs and fault topic are present; CSV schema parity is still pending |

The C++ node is deliberately not selected by the existing production launcher
until golden and hardware parity testing has been completed.
