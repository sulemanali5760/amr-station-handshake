from amr_station_handshake.handshake import (
    AmrSignals, Handshake, Phase, RELEASED, StationSignals, Timeouts)

IDLE = StationSignals()
READY = StationSignals(ready=True)
ACTIVE = StationSignals(ready=True, transfer_active=True)
DONE = StationSignals(ready=True, transfer_done=True)
STALE_FAULT = StationSignals(fault=True, fault_code=2)

REQUESTING = AmrSignals(dock_request=True, in_position=False)
DOCKED = AmrSignals(dock_request=True, in_position=True)


def started(timeouts=Timeouts(), now=0.0):
    """Return a handshake that has already passed RESET and entered REQUEST at `now`."""
    hs = Handshake(timeouts, now)
    assert hs.step(IDLE, now) == REQUESTING
    assert hs.phase is Phase.REQUEST
    return hs


def test_happy_path_walks_every_phase_and_drives_the_right_signals():
    hs = Handshake(now=0.0)
    assert hs.phase is Phase.RESET

    assert hs.step(IDLE, 0.1) == REQUESTING
    assert hs.phase is Phase.REQUEST

    assert hs.step(READY, 0.2) == DOCKED
    assert hs.phase is Phase.DOCK

    assert hs.step(ACTIVE, 0.3) == DOCKED
    assert hs.phase is Phase.TRANSFER

    assert hs.step(DONE, 2.3) == RELEASED
    assert hs.phase is Phase.RELEASE

    assert hs.step(IDLE, 2.4) == RELEASED
    assert hs.phase is Phase.SUCCEEDED
    assert hs.done and hs.error == ''


def test_reset_keeps_signals_low_until_a_stale_fault_clears():
    hs = Handshake(now=0.0)
    assert hs.step(STALE_FAULT, 0.1) == RELEASED
    assert hs.phase is Phase.RESET
    assert hs.step(IDLE, 0.2) == REQUESTING
    assert hs.phase is Phase.REQUEST


def test_reset_waits_for_stale_ready_to_drop():
    hs = Handshake(now=0.0)
    assert hs.step(DONE, 0.1) == RELEASED
    assert hs.phase is Phase.RESET


def test_station_that_never_becomes_idle_times_out_in_reset_with_the_fault_code():
    hs = Handshake(Timeouts(reset=3.0), now=0.0)
    hs.step(STALE_FAULT, 2.9)
    assert hs.phase is Phase.RESET
    hs.step(STALE_FAULT, 3.1)
    assert hs.phase is Phase.TIMED_OUT
    assert 'fault code 2' in hs.error


def test_transfer_that_finishes_between_two_cycles_still_succeeds():
    hs = started()
    hs.step(READY, 0.1)
    hs.step(DONE, 0.2)       # TransferActive was never seen
    assert hs.phase is Phase.TRANSFER
    hs.step(DONE, 0.3)
    hs.step(IDLE, 0.4)
    assert hs.phase is Phase.SUCCEEDED


def test_station_that_never_answers_times_out_in_request():
    hs = started(Timeouts(request=5.0))
    assert hs.step(IDLE, 4.9) == REQUESTING
    assert hs.step(IDLE, 5.1) == RELEASED
    assert hs.phase is Phase.TIMED_OUT
    assert "'request'" in hs.error


def test_timeout_is_measured_from_entering_the_phase():
    hs = started(Timeouts(request=5.0, transfer=15.0))
    hs.step(READY, 4.0)
    hs.step(ACTIVE, 4.5)                 # TRANSFER entered at 4.5
    hs.step(ACTIVE, 19.0)                # 14.5 s in TRANSFER: still fine
    assert hs.phase is Phase.TRANSFER
    hs.step(ACTIVE, 19.6)                # 15.1 s in TRANSFER
    assert hs.phase is Phase.TIMED_OUT


def test_station_that_never_clears_ready_times_out_in_release():
    hs = started(Timeouts(release=3.0))
    hs.step(READY, 0.1)
    hs.step(ACTIVE, 0.2)
    hs.step(DONE, 0.3)
    hs.step(DONE, 3.2)
    assert hs.phase is Phase.RELEASE
    hs.step(DONE, 3.4)
    assert hs.phase is Phase.TIMED_OUT


def test_fault_ends_the_handover_and_reports_the_code():
    hs = started()
    hs.step(READY, 0.1)
    hs.step(ACTIVE, 0.2)
    out = hs.step(StationSignals(ready=True, fault=True, fault_code=1), 0.3)
    assert out == RELEASED
    assert hs.phase is Phase.FAULTED
    assert hs.error == 'station fault, code 1'


def test_fault_wins_over_a_transition_in_the_same_cycle():
    hs = started()
    hs.step(StationSignals(ready=True, fault=True, fault_code=7), 0.1)
    assert hs.phase is Phase.FAULTED


def test_ready_withdrawn_mid_transfer_is_a_fault():
    hs = started()
    hs.step(READY, 0.1)
    hs.step(ACTIVE, 0.2)
    hs.step(StationSignals(transfer_active=True), 0.3)
    assert hs.phase is Phase.FAULTED
    assert 'withdrew Ready' in hs.error


def test_cancel_drops_both_signals():
    hs = started()
    hs.step(READY, 0.1)
    hs.cancel()
    assert hs.phase is Phase.CANCELED
    assert hs.step(READY, 0.2) == RELEASED


def test_terminal_phase_is_final():
    hs = started()
    hs.step(StationSignals(fault=True, fault_code=1), 0.1)
    hs.cancel()
    hs.step(READY, 0.2)
    assert hs.phase is Phase.FAULTED
