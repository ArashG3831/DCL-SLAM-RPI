"""Shared frontier actionability semantics.

Planner reachability and exploration usefulness are deliberately separate
signals.  This small helper is used by both the allocator gate and terminal
evidence so the minimum visible-gain comparator cannot diverge.
"""

from __future__ import annotations

import math


BELOW_MINIMUM_GAIN = 'BELOW_MINIMUM_GAIN'


def gain_meets_minimum(gain: float, minimum_gain_m: float) -> bool:
    """Return the production eligibility test: finite gain >= threshold."""
    return math.isfinite(float(gain)) and float(gain) >= float(minimum_gain_m)


def is_actionable_reachable(
        planner_status: str,
        visible_reveal_gain: float | None,
        minimum_gain_m: float,
        *,
        hard_suppressed: bool = False) -> bool:
    """Return whether a planner-reachable frontier is worth assigning."""
    return (
        str(planner_status) == 'REACHABLE' and
        visible_reveal_gain is not None and
        gain_meets_minimum(visible_reveal_gain, minimum_gain_m) and
        not hard_suppressed
    )
