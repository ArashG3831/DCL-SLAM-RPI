from my_epuck_project.quadrature_decoder import QuadratureDecoder


POSITIVE_CYCLE = (0, 1, 3, 2)


def feed_cycle(decoder, start_state, cycle=POSITIVE_CYCLE):
    index = cycle.index(start_state)
    for offset in range(1, len(cycle) + 1):
        decoder.process_state(cycle[(index + offset) % len(cycle)])


def test_positive_cycle_is_four_valid_transitions():
    decoder = QuadratureDecoder(initial_state=0)

    feed_cycle(decoder, 0)

    snapshot = decoder.snapshot()
    assert snapshot["count"] == 4
    assert snapshot["valid_transition_count"] == 4
    assert snapshot["invalid_transition_count"] == 0


def test_reverse_cycle_is_negative_four_transitions():
    decoder = QuadratureDecoder(initial_state=0)

    for state in (2, 3, 1, 0):
        decoder.process_state(state)

    assert decoder.snapshot()["count"] == -4


def test_one_cycle_from_every_starting_state():
    for start_state in range(4):
        decoder = QuadratureDecoder(initial_state=start_state)

        feed_cycle(decoder, start_state)

        assert decoder.snapshot()["count"] == 4


def test_invalid_jump_does_not_change_position():
    decoder = QuadratureDecoder(initial_state=0)

    decoder.process_state(3)  # 00 -> 11 is impossible in one edge.

    snapshot = decoder.snapshot()
    assert snapshot["count"] == 0
    assert snapshot["valid_transition_count"] == 0
    assert snapshot["invalid_transition_count"] == 1
    assert snapshot["invalid_transition_percentage"] == 100.0


def test_repeated_state_and_reversal_preserve_count_continuity():
    decoder = QuadratureDecoder(initial_state=0)

    decoder.process_state(0)
    decoder.process_state(1)
    decoder.process_state(3)
    decoder.process_state(1)
    decoder.process_state(0)

    snapshot = decoder.snapshot()
    assert snapshot["count"] == 0
    assert snapshot["valid_transition_count"] == 4
    assert snapshot["invalid_transition_count"] == 0


def test_edge_callback_path_counts_a_and_b_edges_independently():
    decoder = QuadratureDecoder(initial_state=0)

    decoder.process_edge("B", 1)
    decoder.process_edge("A", 1)
    decoder.process_edge("B", 0)
    decoder.process_edge("A", 0)

    snapshot = decoder.snapshot()
    assert snapshot["count"] == 4
    assert snapshot["a_edge_count"] == 2
    assert snapshot["b_edge_count"] == 2


def test_encoder_sign_can_reverse_physical_convention():
    decoder = QuadratureDecoder(initial_state=0, encoder_sign=-1)

    feed_cycle(decoder, 0)

    assert decoder.snapshot()["count"] == -4
