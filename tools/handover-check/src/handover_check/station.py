"""Reference station: the station side of the handover as a pure, PLC-style step function, plus an
OPC UA server around it.

Two uses:
- a station to develop against when no PLC is at hand (`handover-check station`);
- the yardstick for the tester's own tests: each DEFECT switches one rule off, and the tester must
  report exactly that rule as broken.

The logic is written the way a PLC task works: read inputs, decide, write outputs, once per cycle.
"""
import asyncio
from dataclasses import dataclass, field

from asyncua import Server, ua

NAMESPACE = "urn:amr-station-handshake:station"
FAULT_CONVEYOR_BLOCKED, FAULT_HEARTBEAT_LOST, FAULT_AMR_LEFT = 1, 2, 3
DEFECTS = (
    "ready_without_request",      # R1: Ready is on although nobody asked
    "slow_ready",                 # R2: Ready comes far too late
    "transfer_before_docked",     # R3: conveyor starts before the vehicle is in position
    "active_stays_on",            # R4: TransferActive still on when TransferDone is set
    "ready_stuck",                # R5: Ready/Done not cleared after the vehicle released
    "no_watchdog",                # R8: a dead vehicle is not noticed
    "fault_not_cleared",          # R9: fault stays latched after the acknowledge
    "ignores_undock",             # R10: conveyor keeps running when the vehicle leaves
)


@dataclass
class Outputs:
    ready: bool = False
    transfer_active: bool = False
    transfer_done: bool = False
    fault: bool = False
    fault_code: int = 0


@dataclass
class StationLogic:
    ready_delay: float = 0.5      # s from request to Ready
    transfer_time: float = 2.0    # s the conveyor runs
    watchdog: float = 1.0         # s without a new heartbeat value before the station faults
    defects: frozenset = field(default_factory=frozenset)
    phase: str = "idle"
    code: int = 0
    _due: float = 0.0
    _last_beat: int | None = None
    _beat_seen: float = 0.0

    def step(self, request: bool, docked: bool, beat: int, now: float) -> Outputs:
        d = self.defects
        if beat != self._last_beat:
            self._last_beat, self._beat_seen = beat, now

        if not request:
            if self.phase == "fault" and "fault_not_cleared" in d:
                pass
            elif self.phase in ("ready", "done") and "ready_stuck" in d:
                pass
            else:
                self.phase, self.code = "idle", 0      # dropping the request also acknowledges a fault
        elif self.phase == "fault":
            pass                                         # latched until the request drops
        elif now - self._beat_seen > self.watchdog and "no_watchdog" not in d:
            self.phase, self.code = "fault", FAULT_HEARTBEAT_LOST
        elif self.phase == "idle":
            self.phase = "preparing"
            self._due = now + (self.ready_delay * 20 if "slow_ready" in d else self.ready_delay)
        elif self.phase == "preparing" and now >= self._due:
            self.phase = "ready"
        elif self.phase == "ready" and (docked or "transfer_before_docked" in d):
            self.phase, self._due = "transferring", now + self.transfer_time
        elif self.phase == "transferring":
            if not docked and not d & {"ignores_undock", "transfer_before_docked"}:
                self.phase, self.code = "fault", FAULT_AMR_LEFT
            elif now >= self._due:
                self.phase = "done"

        return Outputs(
            ready=self.phase in ("ready", "transferring", "done") or "ready_without_request" in d,
            transfer_active=self.phase == "transferring" or (self.phase == "done" and "active_stays_on" in d),
            transfer_done=self.phase == "done",
            fault=self.phase == "fault",
            fault_code=self.code,
        )


async def serve(endpoint: str, logic: StationLogic, ready: asyncio.Event | None = None,
                stop: asyncio.Event | None = None, cycle: float = 0.02) -> None:
    """Run the station as an OPC UA server (no security: a development aid, bind it to localhost)."""
    server = Server()
    await server.init()
    server.set_endpoint(endpoint)
    server.set_server_name("Reference conveyor station (handover-check)")
    server.set_security_policy([ua.SecurityPolicyType.NoSecurity])
    idx = await server.register_namespace(NAMESPACE)
    station = await server.nodes.objects.add_object(ua.NodeId("Station", idx), "Station")

    async def signal(name, value, vt):
        node = await station.add_variable(ua.NodeId(f"Station.{name}", idx), name, value, vt)
        await node.set_writable()
        return node
    boolean, int16 = ua.VariantType.Boolean, ua.VariantType.Int16
    inputs = [await signal(n, v, t) for n, v, t in (("AmrDockRequest", False, boolean),
                                                    ("AmrInPosition", False, boolean), ("AmrHeartbeat", 0, int16))]
    outs = [await signal(n, v, t) for n, v, t in (("StationReady", False, boolean), ("TransferActive", False, boolean),
                                                  ("TransferDone", False, boolean), ("Fault", False, boolean),
                                                  ("FaultCode", 0, int16))]
    loop = asyncio.get_running_loop()
    async with server:
        if ready:
            ready.set()
        while not (stop and stop.is_set()):
            request, docked, beat = [await n.read_value() for n in inputs]
            o = logic.step(bool(request), bool(docked), int(beat), loop.time())
            for node, value, vt in zip(outs, (o.ready, o.transfer_active, o.transfer_done, o.fault, o.fault_code),
                                       (boolean, boolean, boolean, boolean, int16)):
                await node.write_value(value, vt)
            await asyncio.sleep(cycle)
