#!/usr/bin/env python3
"""Fixed-first-scan lidar endpoint diagnostics.

This deliberately does not accumulate scan-to-scan rotations.  It compares
later scans with the first scan as a fixed geometric reference, which avoids
turning a tiny local matching bias into a large long-run angle error.
"""

import math

import numpy as np


def _profile(record):
    ranges = np.asarray(record["ranges"], dtype=float)
    valid = np.isfinite(ranges)
    valid &= ranges >= max(0.05, float(record.get("range_min", 0.05)))
    valid &= ranges <= min(8.0, float(record.get("range_max", 8.0)))
    values = np.where(valid, ranges, np.nan)
    return values, valid


def _score(reference, reference_valid, current, current_valid, shift):
    aligned = np.roll(current, shift)
    aligned_valid = np.roll(current_valid, shift)
    valid = reference_valid & aligned_valid
    if int(np.count_nonzero(valid)) < 100:
        return float("inf")
    delta = np.abs(reference[valid] - aligned[valid])
    return float(np.mean(np.minimum(delta, 0.50)))


def best_fixed_reference_shift(first, current):
    """Return the best circular range-profile shift against ``first``."""
    reference, reference_valid = _profile(first)
    values, valid = _profile(current)
    count = min(reference.size, values.size)
    reference = reference[:count]
    reference_valid = reference_valid[:count]
    values = values[:count]
    valid = valid[:count]
    increment = float(current.get("angle_increment", 0.0))
    if increment <= 0.0 or count < 100:
        return None
    candidates = [(_score(reference, reference_valid, values, valid, shift), shift)
                  for shift in range(count)]
    candidates = [(score, shift) for score, shift in candidates if math.isfinite(score)]
    if not candidates:
        return None
    candidates.sort()
    score, shift = candidates[0]
    second = candidates[1][0] if len(candidates) > 1 else float("inf")
    # Convert the circular bin shift to the signed smallest angular phase.
    signed_shift = shift if shift <= count // 2 else shift - count
    return {
        "shift_bins": int(signed_shift),
        "yaw_deg": math.degrees(signed_shift * increment),
        "score_m": float(score),
        "second_score_m": float(second),
        "valid_bins": int(np.count_nonzero(reference_valid & valid)),
        "confidence_margin_m": float(second - score) if math.isfinite(second) else None,
    }


def analyze_fixed_reference(records, hop_sizes=(5, 10, 20, 25)):
    """Analyze endpoint phase and agreement across several scan hops.

    The endpoint estimate is only valid when the first and final environment
    overlap sufficiently.  This is a lidar-referenced diagnostic, not external
    ground truth and not a replacement for the SLAM A/B comparison.
    """
    if len(records) < 2:
        return {"valid": False, "reason": "fewer than two scans"}
    first = records[0]
    final = records[-1]
    endpoint = best_fixed_reference_shift(first, final)
    if endpoint is None:
        return {"valid": False, "reason": "insufficient overlapping finite scan bins"}

    by_hop = {}
    for hop in hop_sizes:
        if hop <= 0 or hop >= len(records):
            continue
        sample = records[-1 - ((len(records) - 1) % hop)]
        result = best_fixed_reference_shift(first, sample)
        if result is not None:
            by_hop[str(hop)] = result
    phases = [item["yaw_deg"] for item in by_hop.values()]
    agreement = max(phases) - min(phases) if phases else None
    return {
        "valid": True,
        "method": "first-scan fixed-reference circular range-profile correlation",
        "scan_count": len(records),
        "final": endpoint,
        "hop_results": by_hop,
        "hop_phase_range_deg": agreement,
        "hop_agreement": agreement is None or agreement <= 3.0,
        "independence_warning": (
            "Uses lidar geometry and is not independent ground truth for judging "
            "a lidar scan-matching estimator."
        ),
    }
