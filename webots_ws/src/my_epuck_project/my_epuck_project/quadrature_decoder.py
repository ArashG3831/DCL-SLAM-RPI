"""Small, thread-safe full-quadrature decoder used by the real robot node."""

from threading import Lock


# Positive raw direction is the conventional sequence 00 -> 01 -> 11 -> 10 -> 00.
QUADRATURE_TRANSITIONS = {
    (0, 1): 1,
    (1, 3): 1,
    (3, 2): 1,
    (2, 0): 1,
    (0, 2): -1,
    (2, 3): -1,
    (3, 1): -1,
    (1, 0): -1,
}


class QuadratureDecoder:
    """Decode both edges of both channels into signed quadrature counts."""

    def __init__(self, initial_state=0, encoder_sign=1):
        if int(initial_state) not in (0, 1, 2, 3):
            raise ValueError("initial_state must be a two-bit value from 0 to 3")
        if int(encoder_sign) not in (-1, 1):
            raise ValueError("encoder_sign must be either -1 or 1")

        self._lock = Lock()
        self.encoder_sign = int(encoder_sign)
        self.previous_state = int(initial_state)
        self.current_state = int(initial_state)
        self.a_level = (self.current_state >> 1) & 1
        self.b_level = self.current_state & 1

        self.count = 0
        self.valid_transition_count = 0
        self.invalid_transition_count = 0
        self.a_edge_count = 0
        self.b_edge_count = 0
        self.last_delta = 0
        self.decoded_direction = 0
        self.last_transition_time = None
        self.last_a_edge_time = None
        self.last_b_edge_time = None

    def _process_state_locked(self, new_state, timestamp=None):
        previous_state = self.previous_state
        self.current_state = new_state
        self.previous_state = new_state

        if new_state == previous_state:
            # A duplicate notification is not motion and is not an invalid
            # quadrature transition.
            self.last_delta = 0
            return 0

        raw_delta = QUADRATURE_TRANSITIONS.get((previous_state, new_state))
        if raw_delta is None:
            self.invalid_transition_count += 1
            self.last_delta = 0
            self.decoded_direction = 0
            return 0

        delta = raw_delta * self.encoder_sign
        self.count += delta
        self.valid_transition_count += 1
        self.last_delta = delta
        self.decoded_direction = 1 if delta > 0 else -1
        self.last_transition_time = timestamp
        return delta

    def process_state(self, new_state, timestamp=None):
        """Process a complete two-bit state; useful for tests and calibration."""
        new_state = int(new_state)
        if new_state not in (0, 1, 2, 3):
            raise ValueError("new_state must be a two-bit value from 0 to 3")
        with self._lock:
            self.a_level = (new_state >> 1) & 1
            self.b_level = new_state & 1
            return self._process_state_locked(new_state, timestamp)

    def process_edge(self, channel, level, timestamp=None):
        """Process one GPIO edge without reading GPIO from the callback."""
        level = int(level)
        if level not in (0, 1):
            raise ValueError("level must be 0 or 1")
        if channel not in ("A", "B"):
            raise ValueError("channel must be 'A' or 'B'")

        with self._lock:
            if channel == "A":
                self.a_level = level
                self.a_edge_count += 1
                self.last_a_edge_time = timestamp
            else:
                self.b_level = level
                self.b_edge_count += 1
                self.last_b_edge_time = timestamp
            new_state = (self.a_level << 1) | self.b_level
            return self._process_state_locked(new_state, timestamp)

    def reset_counts(self):
        """Reset counters while preserving the currently observed GPIO state."""
        with self._lock:
            self.count = 0
            self.valid_transition_count = 0
            self.invalid_transition_count = 0
            self.a_edge_count = 0
            self.b_edge_count = 0
            self.last_delta = 0
            self.decoded_direction = 0
            self.last_transition_time = None
            self.last_a_edge_time = None
            self.last_b_edge_time = None
            self.previous_state = (self.a_level << 1) | self.b_level
            self.current_state = self.previous_state

    def snapshot(self):
        with self._lock:
            total_transitions = (
                self.valid_transition_count + self.invalid_transition_count
            )
            invalid_percentage = (
                100.0 * self.invalid_transition_count / total_transitions
                if total_transitions
                else 0.0
            )
            return {
                "count": self.count,
                "valid_transition_count": self.valid_transition_count,
                "invalid_transition_count": self.invalid_transition_count,
                "invalid_transition_percentage": invalid_percentage,
                "a_edge_count": self.a_edge_count,
                "b_edge_count": self.b_edge_count,
                "last_delta": self.last_delta,
                "decoded_direction": self.decoded_direction,
                "last_transition_time": self.last_transition_time,
                "last_a_edge_time": self.last_a_edge_time,
                "last_b_edge_time": self.last_b_edge_time,
                "a_level": self.a_level,
                "b_level": self.b_level,
                "current_state": self.current_state,
            }
