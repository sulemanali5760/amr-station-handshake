"""handover-check run opc.tcp://plc:4840 --map station.yaml   |   handover-check station"""
import argparse
import asyncio
import json
import sys
import urllib.parse

from handover_check import __version__
from handover_check.checks import FAIL, PASS, SIM_MAP, SKIP, Limits, as_dicts, load_signal_map, run_checks
from handover_check.security import MODES, PASSWORD_ENV, POLICIES, from_args
from handover_check.station import DEFECTS, StationLogic, serve

MARK = {PASS: "[PASS]", FAIL: "[FAIL]", SKIP: "[SKIP]"}


def render(results, endpoint: str) -> str:
    lines = [f"Station handover check, {endpoint}", ""]
    for r in results:
        lines += [f"{MARK[r.status]} {r.rule:<3} {r.title}", f"           {r.detail}"]
    n = {s: sum(r.status == s for r in results) for s in (PASS, FAIL, SKIP)}
    verdict = "Station conforms." if not n[FAIL] and not n[SKIP] else \
        "Station does NOT conform." if n[FAIL] else "No rule failed, but not every rule could be tested."
    lines += ["", f"{verdict} {n[PASS]} passed, {n[FAIL]} failed, {n[SKIP]} not testable."]
    return "\n".join(lines)


def markdown(results, endpoint: str) -> str:
    rows = "\n".join(f"| {r.rule} | {r.title} | **{r.status}** | {r.detail} |" for r in results)
    return (f"# Station handover check\n\nStation: `{endpoint}`  \nTool: handover-check {__version__}\n\n"
            f"| Rule | Requirement | Result | Observed |\n|---|---|---|---|\n{rows}\n")


def _run(a) -> int:
    host = urllib.parse.urlparse(a.endpoint).hostname or ""
    if host not in ("127.0.0.1", "localhost", "::1") and not a.station_is_safe:
        print("handover-check writes request signals to the station. On a real station this can start a conveyor.\n"
              "Make sure the station is in a test state and clear of people and loads, then repeat the command "
              "with --station-is-safe.", file=sys.stderr)
        return 64
    signal_map = load_signal_map(a.map) if a.map else SIM_MAP
    limits = Limits(a.t_ready, a.t_start, a.t_transfer, a.t_release, a.t_watchdog, a.t_stop, a.t_hold)
    results = asyncio.run(run_checks(a.endpoint, signal_map, limits, a.only, security=from_args(a)))
    print(json.dumps(as_dicts(results), indent=2) if a.json else render(results, a.endpoint))
    if a.report:
        with open(a.report, "w", encoding="utf-8") as f:
            f.write(markdown(results, a.endpoint))
    failed = any(r.status == FAIL for r in results)
    skipped = any(r.status == SKIP for r in results)
    return 1 if failed else 2 if skipped and a.strict else 0


def _station(a) -> int:
    logic = StationLogic(a.ready_delay, a.transfer_time, a.watchdog, frozenset(a.defect or ()))
    endpoint = f"opc.tcp://{a.host}:{a.port}/station/"
    print(f"reference station on {endpoint}" + (f", defects: {', '.join(a.defect)}" if a.defect else ""), flush=True)
    try:
        asyncio.run(serve(endpoint, logic))
    except KeyboardInterrupt:
        pass
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="handover-check", description="Check a conveyor station's AMR handover "
                                 "handshake against rules R1-R10, before the robot arrives.")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="play the vehicle side against a station and report rule by rule")
    r.add_argument("endpoint", help="OPC UA endpoint of the station PLC, e.g. opc.tcp://192.168.0.10:4840")
    r.add_argument("--map", help="signal map YAML (namespace_uri, node_ids); default: the reference station")
    r.add_argument("--station-is-safe", action="store_true",
                   help="confirm the station is in a test state, clear of people and loads (needed for any "
                        "endpoint that is not this computer)")
    r.add_argument("--only", nargs="+", metavar="RULE", help="run only these rules, e.g. --only R2 R8")
    for name, default, text in (("ready", 5.0, "request -> StationReady"), ("start", 3.0, "in position -> TransferActive"),
                                ("transfer", 15.0, "TransferActive -> TransferDone"), ("release", 3.0, "release -> idle"),
                                ("watchdog", 2.0, "heartbeat stopped -> fault"), ("stop", 1.0, "vehicle left -> conveyor stopped"),
                                ("hold", 1.0, "how long 'must stay off' is observed")):
        r.add_argument(f"--t-{name}", type=float, default=default, metavar="S", help=f"limit in seconds: {text} (default {default:g})")
    sec = r.add_argument_group("security (a production PLC usually requires it)")
    sec.add_argument("--security-policy", choices=POLICIES, default="None")
    sec.add_argument("--security-mode", choices=MODES, default="SignAndEncrypt")
    sec.add_argument("--cert-dir", default="pki", metavar="DIR",
                     help="client certificate folder; a self-signed one is created on first use")
    sec.add_argument("--server-cert", default=None, metavar="FILE",
                     help="the PLC's certificate (.der); without it the server identity is not checked")
    sec.add_argument("--user", default=None, help=f"user name for login; the password is read from {PASSWORD_ENV}")
    r.add_argument("--json", action="store_true", help="machine-readable output")
    r.add_argument("--report", metavar="FILE.md", help="also write a Markdown report (for the commissioning record)")
    r.add_argument("--strict", action="store_true", help="exit code 2 when a rule could not be tested")
    r.set_defaults(func=_run)
    s = sub.add_parser("station", help="run the reference station (no PLC needed), optionally with a defect")
    s.add_argument("--host", default="127.0.0.1")
    s.add_argument("--port", type=int, default=4860)
    s.add_argument("--ready-delay", type=float, default=0.5)
    s.add_argument("--transfer-time", type=float, default=4.0)
    s.add_argument("--watchdog", type=float, default=1.0)
    s.add_argument("--defect", action="append", choices=DEFECTS, help="switch one rule off (repeatable)")
    s.set_defaults(func=_station)
    a = ap.parse_args(argv)
    try:
        return a.func(a)
    except (ValueError, OSError, asyncio.TimeoutError) as e:
        print(f"handover-check: {e or type(e).__name__}", file=sys.stderr)
        return 64


if __name__ == "__main__":
    sys.exit(main())
