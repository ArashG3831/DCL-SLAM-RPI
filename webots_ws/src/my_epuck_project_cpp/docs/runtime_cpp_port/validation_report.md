# Native hardware-node port validation

Reference baseline: commit `3be3371`, branch `main`.

This package is opt-in. The existing Python motor and D500 implementations
remain available and remain the production default.

## Completed safe validation

| Validation | Result |
|---|---|
| C++ package build with `-Wall -Wextra -Wpedantic` | PASS |
| C++ motor-core tests | 7 tests PASS |
| C++ D500 parser tests | 3 tests PASS |
| CTest total | 14 tests PASS, 0 failures |
| Python motor-vs-C++ golden vector | PASS, 5 deterministic 20 Hz steps; counts, valid/invalid, RPM, PI output, pose compared |
| Python D500-vs-C++ golden vector | PASS, 2 packets, 1 completed scan, all 720 bins compared |
| Existing relevant Python tests | 29 PASS |
| Python syntax checks | PASS |
| Installed ROS executables | PASS: motor, D500, and parser CLIs resolve |
| Backend selector | PASS; `none/none` default exercised, no hardware claimed |
| C++ D500 ROS path | PASS using pseudo-terminal fixture; `/scan`, 720 bins, `d500_lidar`, clean shutdown |
| `git diff --check` | PASS |

## Motor golden coverage

The test imports the committed Python `MotorPI` implementation but replaces
only the GPIO device classes with in-memory fakes and replaces monotonic time
with a deterministic clock. It feeds the same 192 signed transitions per
50-ms interval to both wheels and compares target RPM, measured RPM, counts,
valid/invalid transition totals, PI command, and midpoint odometry.

No GPIO line is claimed by this test.

## D500 golden coverage

The test constructs the same valid packet stream for the Python oracle and the
C++ parser, including garbage before the first header and arbitrary 17-byte
feed chunks in the C++ CLI. It compares packet accounting, CRC accounting,
scan completion, valid-bin count, and every range bin, including invalid
range representation.

The ROS smoke test uses a pseudo-terminal and an isolated ROS domain. It does
not open `/dev/ttyUSB0`, start the Python driver, or start the motor node.

## Authorized hardware validation

The following tests were completed in isolated ROS domains while the normal
phone-supervised stack was stopped. Python remained the reference/default
backend throughout.

| Validation | Result |
|---|---|
| C++ motor no-motion GPIO claim/shutdown | PASS; safe initialization and motor-zero shutdown |
| C++ motor lifted-wheel stress | PASS; 180 s at -50 RPM, 3,621 odometry samples, zero safety faults |
| Python motor lifted-wheel reference | PASS; 180 s at -50 RPM, 3,600 odometry samples, zero safety faults |
| C++ D500 real serial | PASS; 20 s, 211 scans, 720 bins, p95 interval 100.9 ms, CRC/parser/handoff errors 0 |
| Python D500 real serial reference | PASS; 20 s, 200 scans, 720 bins, p95 interval 104.3 ms |
| Four motor+D500 backend combinations | PASS; each 60 s lifted-wheel run, valid `/odom` and `/scan`, zero safety faults |

### Measured native-port performance

The one-process motor benchmark measured Python at 45.8% mean of one CPU core
and C++ at 21.3% in the 180-second runs. The comparable D500-only benchmark
measured Python at 17.6% and C++ at 3.5% of one core over 20 seconds. These are
process CPU values, not whole-Pi percentages; each result also retains the
whole-system and per-core samples in `/tmp/r1_*` and the generated benchmark
JSON files.

In the four-way 60-second matrix, the native/native combination measured about
21.7% motor plus 4.1% D500 process CPU, versus about 52.2% plus 35.6% for the
Python/Python combination. Scan timing stayed near 100 ms for native D500 and
near 103–105 ms for the Python reference. These runs did not change scan rate,
bin count, control rate, map resolution, calibration, or safety behavior.

## Remaining validation

1. Review the native encoder-count/edge diagnostics during a longer production
   workload containing the final SLAM and thesis nodes.
2. If desired, repeat the matrix with the final conservative scan-matching ON
   configuration once that configuration is deliberately selected; no SLAM
   parameters were changed by this port.
3. Keep the C++ backend opt-in until the physical results are reviewed and a
   separate production-default switch is approved.

No production launcher, Python node, calibration, safety threshold, SLAM
parameter, lidar geometry, or map resolution was changed in this port.
