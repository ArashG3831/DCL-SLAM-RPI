from my_epuck_project.robot2_resilient_mode_manager import (
    ResilientDecisionState,
    ResilientMode,
)


def _state():
    return ResilientDecisionState(peer_robot_id='robot1')


def test_no_peer_starts_solo_without_waiting():
    state = _state()
    assert state.mode == ResilientMode.SOLO_LOCAL_MAPPING
    assert not state.peer_fresh(1_000_000_000, 5_000_000_000)


def test_fresh_identity_valid_descriptor_enters_pending():
    state = _state()
    assert state.observe_descriptor(
        'robot1', 9_000_000_000, 'robot1-kf-1', 10_000_000_000,
        5_000_000_000)
    assert state.mode == ResilientMode.HANDOFF_PENDING
    assert 'PEER_DESCRIPTOR_VALID_HANDOFF_PENDING' in state.events


def test_wrong_identity_and_stale_descriptor_are_ignored():
    state = _state()
    assert not state.observe_descriptor(
        'robot2', 9_000_000_000, 'wrong-peer', 10_000_000_000,
        5_000_000_000)
    assert not state.observe_descriptor(
        'robot1', 1_000_000_000, 'stale', 10_000_000_000,
        5_000_000_000)
    assert state.mode == ResilientMode.SOLO_LOCAL_MAPPING


def test_handoff_failure_returns_to_solo():
    state = _state()
    state.observe_descriptor(
        'robot1', 9_000_000_000, 'robot1-kf-1', 10_000_000_000,
        5_000_000_000)
    state.mark_handoff_failed('PEER_DETECTED_HANDOFF_NOT_READY')
    assert state.mode == ResilientMode.HANDOFF_FAILED_SOLO
    state.return_to_solo()
    assert state.mode == ResilientMode.SOLO_LOCAL_MAPPING
    assert state.events[-1] == 'HANDOFF_RETURNED_TO_SOLO'


def test_shared_activation_requires_stationary_boundary():
    state = _state()
    state.observe_descriptor(
        'robot1', 9_000_000_000, 'robot1-kf-1', 10_000_000_000,
        5_000_000_000)
    state.mark_owner_terminal()
    assert state.mode == ResilientMode.STATIONARY_ALIGNMENT
    state.mark_shared_mapping_ready()
    state.mark_cooperative_active()
    assert state.mode == ResilientMode.COOPERATIVE_ACTIVE
