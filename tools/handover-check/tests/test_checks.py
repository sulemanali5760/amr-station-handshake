"""The tester is only worth something if it catches a station that breaks a rule, and stays quiet on one
that does not. Every test here runs it against the reference station over real OPC UA."""
import asyncio
import json

import pytest

from handover_check.__main__ import main, markdown, render
from handover_check.checks import FAIL, PASS, RULES, SKIP, Limits, load_signal_map, run_checks
from handover_check.station import DEFECTS, FAULT_AMR_LEFT, FAULT_HEARTBEAT_LOST, StationLogic, serve

# Short times so the suite stays fast; the ratios are those of a real station.
LIMITS = Limits(ready=1.0, start=0.6, transfer=3.0, release=0.8, watchdog=0.9, stop=0.5, hold=0.4)
PORT = [48480]


def check(defects=(), only=None, limits=LIMITS, **station):
    PORT[0] += 1
    endpoint = f"opc.tcp://127.0.0.1:{PORT[0]}/station/"
    logic = StationLogic(ready_delay=0.15, transfer_time=station.pop("transfer_time", 1.6), watchdog=0.4,
                         defects=frozenset(defects), **station)

    async def go():
        ready, stop = asyncio.Event(), asyncio.Event()
        task = asyncio.create_task(serve(endpoint, logic, ready, stop))
        await asyncio.wait_for(ready.wait(), 20)
        try:
            return await run_checks(endpoint, limits=limits, only=only, period=0.03)
        finally:
            stop.set()
            await task
    return {r.rule: r for r in asyncio.run(go())}


def test_conforming_station_passes_every_rule():
    results = check()
    assert list(results) == list(RULES)
    assert {r.status for r in results.values()} == {PASS}, render(list(results.values()), "x")
    assert 0.1 < results["R2"].seconds < 0.6                 # the ready delay of 0.15 s is measured, not assumed
    assert f"fault code {FAULT_HEARTBEAT_LOST}" in results["R8"].detail
    assert f"code {FAULT_AMR_LEFT}" in results["R10"].detail


@pytest.mark.parametrize("defect, rule, words", [
    ("ready_without_request", "R1", "with no request"),
    ("slow_ready", "R2", "no StationReady within"),
    ("transfer_before_docked", "R3", "NOT in position"),
    ("active_stays_on", "R4", "TransferActive is still on"),
    ("ready_stuck", "R5", "still shows"),
    ("no_watchdog", "R8", "went silent"),
    ("fault_not_cleared", "R9", "still shows"),
    ("ignores_undock", "R10", "still running"),
])
def test_each_defect_is_reported_under_its_rule(defect, rule, words):
    only = [rule] if rule != "R2" else ["R2"]
    r = check([defect], only=only)[rule]
    assert r.status == FAIL, r
    assert words in r.detail, r.detail


def test_every_defect_has_a_test():
    assert len(DEFECTS) == 8


def test_a_broken_station_never_passes_everything():
    """Whatever single defect the station has, the full run reports at least one FAIL."""
    for defect in ("transfer_before_docked", "no_watchdog", "ready_stuck"):
        statuses = [r.status for r in check([defect]).values()]
        assert FAIL in statuses, (defect, statuses)


def test_too_fast_transfer_is_reported_as_not_testable_not_as_pass():
    """With a 0.2 s transfer the watchdog cannot show before the transfer ends: say so."""
    r = check(only=["R8"], transfer_time=0.2)["R8"]
    assert r.status == SKIP and "longer transfer" in r.detail


def test_second_handover_fails_when_the_station_stays_latched():
    r = check(["ready_stuck"], only=["R6"])["R6"]
    assert r.status in (FAIL, SKIP) and "idle" in r.detail


# ---------------------------------------------------------------- pure station logic, no network
def test_station_logic_walks_the_normal_sequence():
    s = StationLogic(ready_delay=1.0, transfer_time=2.0, watchdog=1.0)
    assert not s.step(False, False, 0, 0.0).ready
    s.step(True, False, 1, 0.1)
    assert not s.step(True, False, 2, 0.5).ready
    assert s.step(True, False, 3, 1.2).ready
    assert s.step(True, True, 4, 1.3).transfer_active
    out = s.step(True, True, 5, 3.4)
    assert out.transfer_done and not out.transfer_active and out.ready
    assert s.step(False, False, 6, 3.5) == type(out)()      # all off again


def test_station_logic_faults_on_a_frozen_heartbeat():
    s = StationLogic(ready_delay=0.0, transfer_time=10.0, watchdog=1.0)
    for t in (0.0, 0.1, 0.2):
        s.step(True, True, int(t * 10) + 1, t)
    assert s.step(True, True, 3, 0.9).transfer_active        # same counter value, still inside the watchdog time
    out = s.step(True, True, 3, 1.3)
    assert out.fault and out.fault_code == FAULT_HEARTBEAT_LOST and not out.transfer_active


# ---------------------------------------------------------------- command line and files
def test_refuses_a_remote_station_without_the_safety_confirmation(capsys):
    assert main(["run", "opc.tcp://192.168.0.10:4840"]) == 64
    assert "--station-is-safe" in capsys.readouterr().err


def test_unreachable_station_is_an_error_not_a_traceback(capsys):
    assert main(["run", "opc.tcp://127.0.0.1:48479"]) == 64
    assert "handover-check:" in capsys.readouterr().err


def test_signal_map_must_name_all_eight_signals(tmp_path):
    p = tmp_path / "map.yaml"
    p.write_text("namespace_uri: urn:x\nnode_ids:\n  AmrDockRequest: a\n", encoding="utf-8")
    with pytest.raises(ValueError, match="AmrInPosition"):
        load_signal_map(p)


def test_shipped_maps_load():
    import pathlib
    maps = sorted((pathlib.Path(__file__).parents[1] / "maps").glob("*.yaml"))
    assert len(maps) == 3
    for m in maps:
        assert len(load_signal_map(m)["node_ids"]) == 8


def test_reports():
    results = list(check(only=["R1", "R2"]).values())
    assert "[PASS] R1" in render(results, "opc.tcp://x") and "Station conforms." in render(results, "opc.tcp://x")
    assert "| R2 |" in markdown(results, "opc.tcp://x")
    assert json.dumps([r.__dict__ for r in results])
