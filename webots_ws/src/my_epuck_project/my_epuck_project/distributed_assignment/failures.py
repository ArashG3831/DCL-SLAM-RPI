"""Evidence-based failure classification and bounded team suppression."""

from dataclasses import dataclass
from typing import Iterable

from .models import FailureClass, FailureEvidence, FailureRecord, PhysicalTask


def classify_compute_path_result(
        error_code: int,
        *,
        timed_out: bool = False,
        tf_unavailable: bool = False,
        action_rejected: bool = False) -> FailureClass:
    """Classify only Nav2 ComputePath evidence exposed by its action result."""
    if tf_unavailable:
        return FailureClass.TF_OR_LIFECYCLE
    if timed_out:
        return FailureClass.TIMEOUT
    if action_rejected:
        return FailureClass.ACTION_REJECTION
    if int(error_code) in (203, 204, 205, 206, 208):
        # Nav2 Jazzy: start/goal outside map, start/goal occupied, no path.
        return FailureClass.HARD_UNREACHABLE
    if int(error_code) != 0:
        return FailureClass.PLANNER_FAILURE
    return FailureClass.UNKNOWN


def classify_failure(evidence: FailureEvidence) -> FailureClass:
    """Choose the most specific class supported by direct observable evidence."""
    if evidence.explicitly_cancelled:
        return FailureClass.EXPLICIT_CANCELLATION
    if evidence.tf_unavailable or evidence.lifecycle_inactive:
        return FailureClass.TF_OR_LIFECYCLE
    if evidence.dynamic_obstacle_confirmed:
        return FailureClass.DYNAMIC_BLOCKAGE
    if evidence.controller_no_progress:
        return FailureClass.CONTROLLER_NO_PROGRESS
    if evidence.compute_path_error == 'HARD_UNREACHABLE':
        return FailureClass.HARD_UNREACHABLE
    if evidence.compute_path_error == 'PLANNER_FAILURE':
        return FailureClass.PLANNER_FAILURE
    if evidence.timed_out:
        return FailureClass.TIMEOUT
    if evidence.action_rejected:
        return FailureClass.ACTION_REJECTION
    return FailureClass.UNKNOWN


HARD_FAILURES = frozenset({
    FailureClass.HARD_UNREACHABLE,
    FailureClass.PLANNER_FAILURE,
    FailureClass.CONTROLLER_NO_PROGRESS,
    FailureClass.DYNAMIC_BLOCKAGE,
})


def bounded_suppression_duration(
        failure_count: int, requested_ttl_s: float,
        maximum_duration_s: float = 120.0) -> float:
    """Return an escalating but bounded TTL for hard evidence."""
    count = max(1, int(failure_count))
    base = max(0.1, min(15.0, float(requested_ttl_s)))
    return min(float(maximum_duration_s), base * (2 ** (count - 1)))


@dataclass
class _Suppression:
    failures: int
    expires_steady_s: float


class FailureSuppressor:
    """Bounded escalating suppression keyed by physical signature."""

    def __init__(
            self, first_duration_s: float = 10.0,
            maximum_duration_s: float = 120.0,
            repeated_evidence_threshold: int = 2,
            maximum_records: int = 128):
        """Configure escalation timing and the bounded record capacity."""
        self._first_duration_s = first_duration_s
        self._maximum_duration_s = maximum_duration_s
        self._repeated_evidence_threshold = repeated_evidence_threshold
        self._maximum_records = maximum_records
        self._records: dict[str, _Suppression] = {}

    def observe(self, record: FailureRecord, now_steady_s: float) -> bool:
        """Create suppression only from hard, repeated evidence."""
        if record.failure_class not in HARD_FAILURES:
            return False
        previous = self._records.get(record.physical_signature)
        failures = 1 if previous is None else previous.failures + 1
        if failures < self._repeated_evidence_threshold:
            self._records[record.physical_signature] = _Suppression(
                failures, now_steady_s,
            )
            return False
        duration = min(
            self._maximum_duration_s,
            self._first_duration_s * (2 ** (failures - self._repeated_evidence_threshold)),
        )
        self._records[record.physical_signature] = _Suppression(
            failures, now_steady_s + duration,
        )
        if len(self._records) > self._maximum_records:
            oldest = min(self._records, key=lambda key: self._records[key].expires_steady_s)
            if oldest != record.physical_signature:
                self._records.pop(oldest)
        return True

    def suppressed(
            self, task: PhysicalTask, now_steady_s: float,
            failures: Iterable[FailureRecord] = ()) -> bool:
        """Check signature suppression, permitting documented alternate approaches."""
        local = self._records.get(task.physical_signature)
        if local is not None and local.expires_steady_s > now_steady_s:
            return True
        for record in failures:
            if (record.physical_signature == task.physical_signature and
                    record.failure_class in HARD_FAILURES and
                    not record.alternative_approach):
                return True
        return False
