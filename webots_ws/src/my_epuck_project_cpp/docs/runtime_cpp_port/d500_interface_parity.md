# D500 native-port parity inventory

Reference: `/home/robot1/d500_ros2_scan.py` at baseline commit `3be3371`.

The C++ node retains the repaired architecture: a dedicated blocking serial
reader/parser thread and a bounded latest-complete-scan handoff to a short ROS
publication timer. It does not share the serial device with the Python node.

| Surface | Python reference | C++ port status |
|---|---|---|
| Node name | `d500_ros2_scan` | same |
| Serial | 230400 baud, 50 ms read timeout | termios 230400, 50 ms poll |
| Packet | `0x54 0x2c`, 47 bytes, 12 points | same |
| CRC | CRC-8 polynomial `0x4d` | same |
| Scan | 720 bins, 30--12000 mm defaults | same |
| Handoff | one latest complete scan | same |
| Timestamp | host-observed acquisition midpoint | same convention |
| ROS topic | `/scan`, `LaserScan`, depth 10 | same |
| Geometry | post-assembly left/right mirror | same algorithm |
| Device selection | explicit port or first `/dev/serial/by-id` entry, then `/dev/ttyUSB0` fallback | same fallback behavior |
| Diagnostics | acquisition/publication gaps, age, CRC/parser/serial/handoff counters | low-rate C++ diagnostic log with equivalent counters |

The raw-byte Python/C++ comparison passes on a deterministic two-packet fixture
across all 720 bins. A ROS publication smoke test also passes through a
pseudo-terminal. A physical D500-only comparison remains pending because the
production Python driver currently owns `/dev/ttyUSB0`; it must be run as a
coordinated, explicitly authorized hardware test.
