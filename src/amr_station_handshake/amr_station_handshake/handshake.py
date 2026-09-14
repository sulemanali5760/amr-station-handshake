"""AMR-to-station handover handshake as a pure state machine.

No ROS, no OPC UA, no clock. The caller reads the station's signals, calls
step() once per cycle with the current time, and writes the returned AMR
signals back. That is the same read-inputs / execute / write-outputs cycle a
PLC task runs, so this logic ports 1:1 to a function block.
"""

from dataclasses import dataclass
from enum import Enum


@dataclass(frozen=True)
class StationSignals:
    """Signals the station PLC publishes."""

    ready: bool = False
    transfer_active: bool = False
    transfer_done: bool = False
    fault: bool = False
    fault_code: int = 0


@dataclass(frozen=True)
class AmrSignals:
    """Signals the AMR side writes to the station PLC."""

    dock_request: bool = False
    in_position: bool = False


RELEASED = AmrSignals(dock_request=False, in_position=False)


class Phase(Enum):
    REQUEST = 'request'      # DockRequest set, waiting for StationReady
    DOCK = 'dock'            # InPosition set, waiting for the transfer to start
    TRANSFER = 'transfer'    # waiting for TransferDone
    RELEASE = 'release'      # signals dropped, waiting for the station to clear Ready
    SUCCEEDED = 'succeeded'
    TIMED_OUT = 'timed_out'
    FAULTED = 'faulted'
    CANCELED = 'canceled'


TERMINAL = {Phase.SUCCEEDED, Phase.TIMED_OUT, Phase.FAULTED, Phase.CANCELED}


@dataclass(frozen=True)
class Timeouts:
    """Seconds allowed in each phase, measured from entering that phase."""

    request: float = 5.0
    dock: float = 3.0
    transfer: float = 15.0
    release: float = 3.0


class Handshake:

    def __init__(self, timeouts: Timeouts = Timeouts(), now: float = 0.0):
        self._timeouts = timeouts
        self.phase = Phase.REQUEST
        self.error = ''
        self._entered = now

    @property
    def done(self) -> bool:
        return self.phase in TERMINAL

    def cancel(self) -> None:
        if not self.done:
            self._end(Phase.CANCELED, 'canceled by client')

    def step(self, station: StationSignals, now: float) -> AmrSignals:
        if self.done:
            return RELEASED

        if station.fault:
            self._end(Phase.FAULTED, f'station fault, code {station.fault_code}')
        elif self.phase in (Phase.DOCK, Phase.TRANSFER) and not station.ready:
            self._end(Phase.FAULTED, 'station withdrew Ready during the handover')
        elif self.phase is Phase.REQUEST and station.ready:
            self._enter(Phase.DOCK, now)
        elif self.phase is Phase.DOCK and (station.transfer_active or station.transfer_done):
            self._enter(Phase.TRANSFER, now)
        elif self.phase is Phase.TRANSFER and station.transfer_done:
            self._enter(Phase.RELEASE, now)
        elif self.phase is Phase.RELEASE and not station.ready:
            self._end(Phase.SUCCEEDED, '')
        elif now - self._entered > self._limit():
            self._end(
                Phase.TIMED_OUT,
                f"no station response in phase '{self.phase.value}' "
                f'after {self._limit():.1f} s')

        return self._outputs()

    def _outputs(self) -> AmrSignals:
        if self.phase is Phase.REQUEST:
            return AmrSignals(dock_request=True, in_position=False)
        if self.phase in (Phase.DOCK, Phase.TRANSFER):
            return AmrSignals(dock_request=True, in_position=True)
        return RELEASED

    def _limit(self) -> float:
        return getattr(self._timeouts, self.phase.value)

    def _enter(self, phase: Phase, now: float) -> None:
        self.phase = phase
        self._entered = now

    def _end(self, phase: Phase, error: str) -> None:
        self.phase = phase
        self.error = error
