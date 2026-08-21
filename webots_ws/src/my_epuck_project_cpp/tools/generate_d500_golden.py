#!/usr/bin/env python3
"""Cross-language D500 parser regression using the Python oracle."""

import importlib.util
import json
import math
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


def load_oracle():
    path = Path(__file__).parents[4] / "d500_ros2_scan.py"
    spec = importlib.util.spec_from_file_location("d500_python_oracle", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def packet(oracle, start, end, distance, intensity):
    p = bytearray(47)
    p[0:2] = bytes((oracle.HEADER0, oracle.VERLEN))
    struct.pack_into("<H", p, 2, 1200)
    struct.pack_into("<H", p, 4, start)
    for i in range(oracle.POINTS):
        off = 6 + 3 * i
        struct.pack_into("<H", p, off, distance + i)
        p[off + 2] = intensity + i
    struct.pack_into("<HH", p, 42, end, 7)
    p[-1] = oracle.crc8(bytes(p[:-1]))
    return bytes(p)


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: generate_d500_golden.py D500_PARSER_CLI")
    oracle = load_oracle()
    raw = (b"garbage" + packet(oracle, 35000, 35900, 1000, 10) +
           packet(oracle, 100, 1000, 1100, 20))
    with tempfile.TemporaryDirectory(prefix="d500_golden_") as tmp:
        raw_path = Path(tmp) / "stream.bin"
        raw_path.write_bytes(raw)
        cpp = json.loads(subprocess.check_output([sys.argv[1], str(raw_path)], text=True))

        args = type("Args", (), {
            "port": "/dev/null", "baud": 230400, "bins": 720,
            "min_mm": 30, "max_mm": 12000, "min_intensity": 0,
            "invert": False, "angle_offset_deg": 0.0,
        })()
        worker = oracle.SerialScanWorker(args)
        worker.buf.extend(raw)
        worker._parse_available()
        py_scan = worker.latest_scan
        if py_scan is None:
            raise AssertionError("Python oracle did not complete a scan")
        if cpp["packet_count"] != 2 or cpp["bad_crc_count"] != 0:
            raise AssertionError(f"C++ packet accounting mismatch: {cpp}")
        if cpp["completed_scan_count"] != 1 or cpp["valid_bins"] != py_scan.valid_bins:
            raise AssertionError(f"C++ scan accounting mismatch: {cpp}")
        for i, value in enumerate(py_scan.ranges):
            got = cpp["ranges"][i]
            if math.isfinite(value):
                if got is None or abs(got - value) > 1e-6:
                    raise AssertionError(f"range mismatch at {i}: {got} vs {value}")
            elif got is not None:
                raise AssertionError(f"invalid range became finite at {i}: {got}")
    print(json.dumps({"status": "PASS", "packets": 2, "completed_scans": 1,
                      "bins_compared": 720}, indent=2))


if __name__ == "__main__":
    main()
