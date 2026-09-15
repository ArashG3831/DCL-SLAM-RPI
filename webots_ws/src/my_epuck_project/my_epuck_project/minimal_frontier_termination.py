"""Adapt existing mission evidence for the minimal coordinator.

This module owns only the stability gate and unreachable exception.
It reuses the established evidence taxonomy and matching semantics.
It has no lifecycle, acknowledgement, proof, or recovery behavior.
"""

from .mission_termination import (
    CandidateEvidence,
    TerminalReason,
    classify_empty_frontiers,
    matching_terminal_reason,
)


def classify(
        r1: CandidateEvidence, r2: CandidateEvidence, *,
        stable_for_s: float, stability_grace_s: float,
) -> TerminalReason | None:
    """Classify stable exhaustion, including only the authorized exception."""
    if stable_for_s < stability_grace_s:
        return None

    reason = classify_empty_frontiers(r1, r2)
    if reason is not None:
        return reason

    evidence = (r1, r2)
    if any(
            item.planner_failures or item.unclassified or
            item.meaningful_detected_not_queried or
            (item.detected_not_queried and not item.terminal_small) or
            (item.actionable_reachable if item.actionable_reachable is not None
             else item.reachable)
            for item in evidence
    ):
        return None
    if not any(item.unreachable > 0 for item in evidence):
        return None

    # Every detected region must have a terminally accounted-for class; a
    # reachable count may only coexist with unreachable evidence when all of
    # it is explicitly below the minimum gain.
    for item in evidence:
        terminal_count = (
            (item.terminal_small or item.small) + item.out_of_range +
            item.unreachable + item.below_minimum_gain
        )
        if terminal_count != item.detected or item.reachable > item.below_minimum_gain:
            return None
    return TerminalReason.NO_REACHABLE_FRONTIERS


def matching(
        local_terminal: bool, local_reason: str,
        peer_terminal: bool, peer_reason: str,
) -> str | None:
    """Return the existing matching terminal reason, if both peers agree."""
    reason = matching_terminal_reason(
        local_terminal, local_reason, peer_terminal, peer_reason,
    )
    return None if reason is None else str(getattr(reason, 'value', reason))
