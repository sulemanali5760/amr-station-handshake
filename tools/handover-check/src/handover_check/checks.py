"""Play the vehicle side of the handover against a station PLC and check the station against rules R1-R10.

The tester only needs the eight handshake signals over OPC UA (docs/SPEC.md). It does not need a robot,
ROS or the load: it sets the two request bits and the heartbeat like a vehicle would, also in the ways a
vehicle fails (goes silent, leaves, changes its mind), and watches what the station answers and how fast.

It WRITES to the station. On a real station that can start a conveyor. Run it with the station in a
test state, clear of people and loads.
"""
import asyncio
from dataclasses import asdict, dataclass

import yaml
from asyncua import Client, ua

SIGNALS = ("AmrDockRequest", "AmrInPosition", "AmrHeartbeat",
           "StationReady", "TransferActive", "TransferDone", "Fault", "FaultCode")
SIM_MAP = {"namespace_uri": "urn:amr-station-handshake:station",
           "node_ids": {name: f"Station.{name}" for name in SIGNALS}}
PASS, FAIL, SKIP = "PASS", "FAIL", "SKIP"


@dataclass(frozen=True)
class Limits:
    """Longest time the station may take, in seconds. Set them to what the plant specification says."""
    ready: float = 5.0        # request -> StationReady
    start: float = 3.0        # in position -> TransferActive
    transfer: float = 15.0    # TransferActive -> TransferDone
    release: float = 3.0      # request dropped -> station idle
    watchdog: float = 2.0     # heartbeat stopped -> Fault, conveyor stopped
    stop: float = 1.0         # vehicle left position -> conveyor stopped
    hold: float = 1.0         # how long "must stay off" conditions are observed


@dataclass
class Result:
    rule: str
    title: str
    status: str
    detail: str = ""
    seconds: float | None = None      # the measured response time, where the rule has one


@dataclass(frozen=True)
class State:
    ready: bool
    active: bool
    done: bool
    fault: bool
    code: int

    @property
    def idle(self) -> bool:
        return not (self.ready or self.active or self.done or self.fault)

    def __str__(self) -> str:
        on = [n for n, v in (("Ready", self.ready), ("TransferActive", self.active),
                             ("TransferDone", self.done), (f"Fault(code {self.code})", self.fault)) if v]
        return ", ".join(on) or "all outputs off"


def load_signal_map(path) -> dict:
    with open(path, encoding="utf-8") as f:
        signal_map = yaml.safe_load(f)
    missing = [n for n in SIGNALS if n not in (signal_map or {}).get("node_ids", {})]
    if not signal_map or "namespace_uri" not in signal_map or missing:
        raise ValueError(f"{path}: needs namespace_uri and node_ids for {', '.join(missing or SIGNALS)}")
    return signal_map


class Rig:
    """The vehicle side: two request bits, a heartbeat, and eyes on the station's five outputs."""

    def __init__(self, endpoint: str, signal_map: dict, period: float = 0.05, timeout: float = 4.0):
        self.endpoint, self.signal_map, self.period = endpoint, signal_map, period
        self._client = Client(endpoint, timeout=timeout)
        self._beating = True
        self._beat = 0
        self._task = None
        self._n = {}

    async def __aenter__(self) -> "Rig":
        await self._client.connect()
        try:
            ns = await self._client.get_namespace_index(self.signal_map["namespace_uri"])
        except ValueError:
            await self._client.disconnect()
            raise ValueError(f"namespace {self.signal_map['namespace_uri']} not found on {self.endpoint}") from None
        self._n = {name: self._client.get_node(ua.NodeId(self.signal_map["node_ids"][name], ns)) for name in SIGNALS}
        self._outputs = [self._n[k] for k in SIGNALS[3:]]
        await self.read()                                   # fails here if a node id is wrong
        self._task = asyncio.create_task(self._heartbeat())
        await asyncio.sleep(self.period * 4)                 # heartbeat is running before the first request
        return self

    async def __aexit__(self, *exc) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        try:                                                # never leave a request latched on the PLC
            await self.set(False, False)
        finally:
            await self._client.disconnect()

    async def _heartbeat(self) -> None:
        while True:
            if self._beating:
                self._beat = (self._beat + 1) % 32768       # PLC INT
                await self._n["AmrHeartbeat"].write_value(self._beat, ua.VariantType.Int16)
            await asyncio.sleep(self.period)

    def heartbeat(self, on: bool) -> None:
        self._beating = on

    async def set(self, request: bool, in_position: bool) -> None:
        await self._n["AmrDockRequest"].write_value(request, ua.VariantType.Boolean)
        await self._n["AmrInPosition"].write_value(in_position, ua.VariantType.Boolean)

    async def read(self) -> State:
        ready, active, done, fault, code = await self._client.read_values(self._outputs)   # one consistent snapshot
        return State(bool(ready), bool(active), bool(done), bool(fault), int(code or 0))

    async def wait(self, wanted, limit: float, forbidden=None):
        """Poll until wanted(state). Returns (reached, seconds, state, violation): violation is the first
        state in which forbidden(state) was true while waiting."""
        loop = asyncio.get_running_loop()
        start = loop.time()
        violation = None
        while True:
            s = await self.read()
            t = loop.time() - start
            if forbidden and violation is None and forbidden(s):
                violation = s
            if wanted(s):
                return True, t, s, violation
            if t > limit:
                return False, t, s, violation
            await asyncio.sleep(self.period / 2)

    async def observe(self, seconds: float, forbidden):
        """Watch for `seconds`; returns the first state that broke the rule, or None."""
        _, _, _, violation = await self.wait(lambda s: False, seconds, forbidden)
        return violation

    async def to_idle(self, limit: float) -> bool:
        self.heartbeat(True)
        await self.set(False, False)
        reached, *_ = await self.wait(lambda s: s.idle, limit)
        # A vehicle is alive before it asks. Give the station a few fresh heartbeats first, otherwise the
        # next request can meet a stale counter and a correct watchdog trips at once.
        await asyncio.sleep(self.period * 4)
        return reached


# ---------------------------------------------------------------- building blocks
async def _to_ready(rig: Rig, lim: Limits):
    await rig.set(True, False)
    return await rig.wait(lambda s: s.ready, lim.ready, forbidden=lambda s: s.active or s.done)


async def _to_transfer(rig: Rig, lim: Limits):
    reached, t, s, _ = await _to_ready(rig, lim)
    if not reached:
        return False, t, s
    await rig.set(True, True)
    reached, t, s, _ = await rig.wait(lambda s: s.active or s.done or s.fault, lim.start)
    return reached and s.active, t, s


# ---------------------------------------------------------------- the rules
async def r1_idle_at_rest(rig, lim):
    bad = await rig.observe(lim.hold, lambda s: not s.idle)
    if bad:
        return Result("R1", "", FAIL, f"with no request the station shows: {bad}")
    return Result("R1", "", PASS, f"all outputs off for {lim.hold:g} s without a request")


async def r2_ready_follows_request(rig, lim):
    reached, t, s, early = await _to_ready(rig, lim)
    if early:
        return Result("R2", "", FAIL, f"before the vehicle was in position the station showed: {early}", t)
    if not reached:
        return Result("R2", "", FAIL, f"no StationReady within {lim.ready:g} s of the request (station: {s})", t)
    return Result("R2", "", PASS, f"StationReady {t:.2f} s after the request (limit {lim.ready:g} s)", t)


async def r3_no_transfer_before_in_position(rig, lim):
    reached, _, s, _ = await _to_ready(rig, lim)
    if not reached:
        return Result("R3", "", SKIP, f"station never became ready (see R2), so this could not be tested: {s}")
    bad = await rig.observe(lim.hold, lambda s: s.active or s.done)
    if bad:
        return Result("R3", "", FAIL, f"the vehicle was NOT in position, but the station showed: {bad}")
    return Result("R3", "", PASS, f"no transfer for {lim.hold:g} s while the vehicle was not in position")


async def r4_transfer_completes(rig, lim):
    ok, t_start, s = await _to_transfer(rig, lim)
    if not ok:
        return Result("R4", "", FAIL, f"no TransferActive within {lim.start:g} s of AmrInPosition (station: {s})", t_start)
    reached, t, s, _ = await rig.wait(lambda s: s.done or s.fault, lim.transfer)
    if not reached or s.fault:
        return Result("R4", "", FAIL, f"no TransferDone within {lim.transfer:g} s (station: {s})", t)
    s = await rig.read()
    if s.active:
        return Result("R4", "", FAIL, "TransferDone is set but TransferActive is still on: the vehicle cannot tell "
                                      "whether the conveyor has stopped", t)
    return Result("R4", "", PASS, f"transfer started after {t_start:.2f} s and finished after {t:.2f} s", t)


async def r5_release(rig, lim):
    ok, _, s = await _to_transfer(rig, lim)
    reached, _, s, _ = await rig.wait(lambda s: s.done or s.fault, lim.transfer) if ok else (False, 0, s, None)
    if not reached or s.fault:
        return Result("R5", "", SKIP, f"no completed transfer to release from (see R4): {s}")
    await rig.set(False, False)
    reached, t, s, _ = await rig.wait(lambda s: s.idle, lim.release)
    if not reached:
        return Result("R5", "", FAIL, f"{lim.release:g} s after the vehicle released, the station still shows: {s}", t)
    return Result("R5", "", PASS, f"station idle {t:.2f} s after the release (limit {lim.release:g} s)", t)


async def r6_second_handover(rig, lim):
    for n in (1, 2):
        ok, _, s = await _to_transfer(rig, lim)
        reached, _, s, _ = await rig.wait(lambda s: s.done or s.fault, lim.transfer) if ok else (False, 0, s, None)
        if not reached or s.fault:
            return Result("R6", "", FAIL if n == 2 else SKIP, f"handover {n} did not complete: {s}")
        if not await rig.to_idle(lim.release):
            return Result("R6", "", FAIL if n == 2 else SKIP, f"station not idle after handover {n}: {await rig.read()}")
    return Result("R6", "", PASS, "two handovers in a row completed")


async def r7_withdrawn_request(rig, lim):
    reached, _, s, _ = await _to_ready(rig, lim)
    if not reached:
        return Result("R7", "", SKIP, f"station never became ready (see R2): {s}")
    await rig.set(False, False)
    reached, t, s, moved = await rig.wait(lambda s: s.idle, lim.release, forbidden=lambda s: s.active or s.done)
    if moved:
        return Result("R7", "", FAIL, f"the vehicle withdrew its request, yet the station showed: {moved}", t)
    if not reached:
        return Result("R7", "", FAIL, f"{lim.release:g} s after the request was withdrawn the station still shows: {s}", t)
    return Result("R7", "", PASS, f"station back to idle {t:.2f} s after the request was withdrawn", t)


async def r8_heartbeat_loss(rig, lim):
    ok, _, s = await _to_transfer(rig, lim)
    if not ok:
        return Result("R8", "", SKIP, f"no running transfer to interrupt (see R4): {s}")
    rig.heartbeat(False)
    reached, t, s, _ = await rig.wait(lambda s: s.fault or s.done, lim.watchdog)
    if s.done and not s.fault:
        return Result("R8", "", SKIP, f"the transfer finished {t:.2f} s after the heartbeat stopped, before a watchdog "
                                      "could show. Test with a longer transfer.", t)
    if not reached:
        return Result("R8", "", FAIL, f"the vehicle went silent for {lim.watchdog:g} s during the transfer and the "
                                      f"station carried on: {s}", t)
    if s.active:
        return Result("R8", "", FAIL, f"Fault is set but TransferActive is still on: {s}", t)
    if s.code == 0:
        return Result("R8", "", FAIL, "Fault is set but FaultCode is 0: nobody can tell what happened", t)
    return Result("R8", "", PASS, f"fault code {s.code} and conveyor stopped {t:.2f} s after the heartbeat stopped "
                                  f"(limit {lim.watchdog:g} s)", t)


async def r9_fault_acknowledge(rig, lim):
    ok, _, s = await _to_transfer(rig, lim)
    if ok:
        rig.heartbeat(False)
        ok, _, s, _ = await rig.wait(lambda s: s.fault or s.done, lim.watchdog)
    if not ok or not s.fault:
        return Result("R9", "", SKIP, f"could not bring the station into a fault to acknowledge (see R8): {s}")
    await rig.set(False, False)                             # the acknowledge; the heartbeat may still be dead
    reached, t, s, _ = await rig.wait(lambda s: s.idle, lim.release)
    if not reached:
        return Result("R9", "", FAIL, f"{lim.release:g} s after the request was dropped (the acknowledge) the station "
                                      f"still shows: {s}", t)
    return Result("R9", "", PASS, f"fault cleared {t:.2f} s after the acknowledge", t)


async def r10_vehicle_leaves(rig, lim):
    ok, _, s = await _to_transfer(rig, lim)
    if not ok:
        return Result("R10", "", SKIP, f"no running transfer to interrupt (see R4): {s}")
    await rig.set(True, False)                              # still requesting, but no longer in position
    reached, t, s, _ = await rig.wait(lambda s: not s.active, lim.stop)
    if not reached:
        return Result("R10", "", FAIL, f"the vehicle left its position and {lim.stop:g} s later the conveyor is "
                                       f"still running: {s}", t)
    if s.done and not s.fault:
        return Result("R10", "", SKIP, f"the transfer finished {t:.2f} s after the vehicle left, too fast to judge. "
                                       "Test with a longer transfer.", t)
    return Result("R10", "", PASS, f"conveyor stopped {t:.2f} s after the vehicle left its position "
                                   f"(limit {lim.stop:g} s; station: {s})", t)


RULES = {
    "R1": ("Idle at rest: no output is on without a request", r1_idle_at_rest),
    "R2": ("StationReady follows AmrDockRequest in time, and nothing moves yet", r2_ready_follows_request),
    "R3": ("No transfer before AmrInPosition", r3_no_transfer_before_in_position),
    "R4": ("Transfer starts after AmrInPosition and completes; TransferActive is off when TransferDone is on", r4_transfer_completes),
    "R5": ("Station returns to idle after the vehicle releases", r5_release),
    "R6": ("A second handover works straight after the first", r6_second_handover),
    "R7": ("A withdrawn request returns the station to idle without a transfer", r7_withdrawn_request),
    "R8": ("Heartbeat watchdog: a silent vehicle stops the transfer with a fault code", r8_heartbeat_loss),
    "R9": ("Dropping the request acknowledges and clears a fault", r9_fault_acknowledge),
    "R10": ("The conveyor stops when the vehicle leaves its position", r10_vehicle_leaves),
}


async def run_checks(endpoint: str, signal_map: dict = SIM_MAP, limits: Limits = Limits(),
                     only: list[str] | None = None, period: float = 0.05) -> list[Result]:
    """Run the rules in order against the station. Before each rule the station is brought back to idle;
    a rule that cannot start from idle is reported as SKIP with the reason."""
    results = []
    stuck = False
    async with Rig(endpoint, signal_map, period) as rig:
        for rule, (title, check) in RULES.items():
            if only and rule not in only:
                continue
            if rule != "R1" and not await rig.to_idle(limits.release * 2):
                # A station that keeps outputs on after the vehicle dropped both request bits has broken
                # the release rule. Say so once as a failure; the rules after it cannot be judged.
                detail = (f"the vehicle dropped its request and {limits.release * 2:g} s later the station still "
                          f"shows: {await rig.read()}")
                r = Result(rule, title, SKIP if stuck else FAIL,
                           ("not testable, " if stuck else "station does not return to idle (rule R5): ") + detail)
                stuck = True
            else:
                try:
                    r = await check(rig, limits)
                finally:
                    rig.heartbeat(True)
                r.title = title
            results.append(r)
        await rig.to_idle(limits.release)
    return results


def as_dicts(results: list[Result]) -> list[dict]:
    return [asdict(r) for r in results]
