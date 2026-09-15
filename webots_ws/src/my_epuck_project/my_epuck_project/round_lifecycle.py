"""Small, dependency-free generation gate for asynchronous allocator work."""

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class RoundLease:
    """Immutable identity captured by work that may complete later."""

    round_id: str
    generation: int


class RoundGeneration:
    """Monotonic round generation independent of ROS callback timing."""

    def __init__(self) -> None:
        self.generation = 0
        self.active_round_id: Optional[str] = None

    def activate(self, round_id: str) -> RoundLease:
        """Start or replace a round and return its immutable lease."""
        self.generation += 1
        self.active_round_id = round_id
        return RoundLease(round_id, self.generation)

    def invalidate(self) -> int:
        """Invalidate the current round and return the new generation."""
        self.generation += 1
        self.active_round_id = None
        return self.generation

    def is_current(self, lease: RoundLease) -> bool:
        """Return whether asynchronous work still belongs to this round."""
        return (
            self.active_round_id == lease.round_id and
            self.generation == lease.generation
        )
