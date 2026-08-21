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

## Remaining validation

The following require an explicitly authorized hardware window:

1. Physical D500-only A/B test. The current Python D500 process owns the real
   serial device, so it must be coordinated and stopped/restarted cleanly.
2. Motor no-motion GPIO initialization/claim test, followed by a stationary
   encoder observation test.
3. Controlled lifted-wheel motor parity test against Python.
4. Four runtime CPU benchmark configurations after safety parity is accepted.
5. Production-default switch, only after those results are reviewed.

No production launcher, Python node, calibration, safety threshold, SLAM
parameter, lidar geometry, or map resolution was changed in this port.
