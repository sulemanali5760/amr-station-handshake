"""ROS 2 action server that runs the Dock handshake against a station PLC over OPC UA."""

from concurrent.futures import CancelledError
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from amr_handshake_interfaces.action import Dock
from amr_station_handshake.handshake import (
    AmrSignals, Handshake, Phase, RELEASED, StationSignals, Timeouts)
from asyncua import ua
from asyncua.sync import Client, ThreadLoop, ThreadLoopNotRunning
from asyncua.ua.uaerrors import UaError
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
import yaml

SIGNALS = (
    'AmrDockRequest', 'AmrInPosition', 'AmrHeartbeat',
    'StationReady', 'TransferActive', 'TransferDone', 'Fault', 'FaultCode',
)


class StationLinkError(Exception):
    """The station is reachable but does not expose what the signal map expects."""


# Everything that means "the link to the station is broken", and nothing that means "bug".
LINK_ERRORS = (OSError, TimeoutError, CancelledError, UaError, ThreadLoopNotRunning,
               StationLinkError)


def load_signal_map(path: str) -> dict:
    with open(path, encoding='utf-8') as f:
        signal_map = yaml.safe_load(f)
    missing = [name for name in SIGNALS if name not in signal_map.get('node_ids', {})]
    if 'namespace_uri' not in signal_map or missing:
        raise ValueError(f'{path}: needs namespace_uri and node_ids for {missing or SIGNALS}')
    return signal_map


class StationLink:
    """The station's handshake signals over one OPC UA session."""

    def __init__(self, endpoint: str, signal_map: dict, tloop: ThreadLoop):
        self._client = Client(endpoint, timeout=2, tloop=tloop)
        self._client.connect()
        uri = signal_map['namespace_uri']
        try:
            ns = self._client.get_namespace_index(uri)
        except ValueError:
            self._client.disconnect()
            raise StationLinkError(f'namespace {uri} not found on {endpoint}') from None
        self._nodes = {
            name: self._client.get_node(ua.NodeId(signal_map['node_ids'][name], ns))
            for name in SIGNALS
        }
        self._written = None
        self._heartbeat = 0

    def read(self) -> StationSignals:
        n = self._nodes
        return StationSignals(
            ready=n['StationReady'].read_value(),
            transfer_active=n['TransferActive'].read_value(),
            transfer_done=n['TransferDone'].read_value(),
            fault=n['Fault'].read_value(),
            fault_code=n['FaultCode'].read_value(),
        )

    def write(self, signals: AmrSignals) -> None:
        if signals == self._written:
            return
        self._nodes['AmrDockRequest'].write_value(signals.dock_request, ua.VariantType.Boolean)
        self._nodes['AmrInPosition'].write_value(signals.in_position, ua.VariantType.Boolean)
        self._written = signals

    def beat(self) -> None:
        """Increment the heartbeat so the station's watchdog knows this side is alive."""
        self._heartbeat = (self._heartbeat + 1) % 32768     # PLC INT is 16 bit
        self._nodes['AmrHeartbeat'].write_value(self._heartbeat, ua.VariantType.Int16)

    def close(self) -> None:
        self._client.disconnect()


class StationBridge(Node):

    def __init__(self):
        super().__init__('station_bridge')
        default_map = os.path.join(
            get_package_share_directory('amr_station_handshake'), 'config', 'station_sim.yaml')
        self._endpoint = self.declare_parameter(
            'endpoint', 'opc.tcp://127.0.0.1:4860/station/').value
        signal_map_path = self.declare_parameter('signal_map', default_map).value
        self._signal_map = load_signal_map(signal_map_path)
        self._cycle_s = self._seconds('cycle_s', 0.05)
        self._connect_timeout_s = self._seconds('connect_timeout_s', 10.0)
        defaults = Timeouts()
        self._timeouts = Timeouts(**{
            phase: self._seconds(f'timeout.{phase}', getattr(defaults, phase))
            for phase in ('reset', 'request', 'dock', 'transfer', 'release')
        })

        self._tloop = ThreadLoop(timeout=10)
        self._tloop.start()
        self._link = None
        self._busy = False
        self._lock = threading.Lock()

        self._server = ActionServer(
            self, Dock, 'station/dock',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f'Dock action ready, station at {self._endpoint}')

    def _seconds(self, name: str, default: float) -> float:
        # dynamic_typing accepts `-p timeout.request:=5` as well as `:=5.0`.
        value = self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
            raise ValueError(f'parameter {name} must be positive seconds, got {value!r}')
        return float(value)

    def _on_goal(self, goal):
        with self._lock:
            if self._busy:
                self.get_logger().warn(f'Rejecting {goal.amr_id}: a handover is already running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        try:
            result = self._handover(goal_handle)
            log = self.get_logger()
            amr_id = goal_handle.request.amr_id
            if result.success:
                goal_handle.succeed()
                log.info(f'{amr_id}: handover succeeded')
            elif result.phase == Phase.CANCELED.value:
                goal_handle.canceled()
                log.warn(f'{amr_id}: handover canceled')
            else:
                goal_handle.abort()
                log.error(f'{amr_id}: handover {result.phase}: {result.error}')
            return result
        finally:
            # Only after the goal status is set, so no second goal slips in before the result.
            with self._lock:
                self._busy = False

    def _handover(self, goal_handle) -> Dock.Result:
        amr_id = goal_handle.request.amr_id
        log = self.get_logger()
        log.info(f'{amr_id}: handover started')
        result = Dock.Result()
        try:
            link = self._connect()
            hs = Handshake(self._timeouts, time.monotonic())
            feedback = Dock.Feedback()
            reported = None
            try:
                while not hs.done:
                    if goal_handle.is_cancel_requested:
                        hs.cancel()
                    link.write(hs.step(link.read(), time.monotonic()))
                    link.beat()
                    if hs.phase is not reported:
                        reported = hs.phase
                        feedback.phase = hs.phase.value
                        goal_handle.publish_feedback(feedback)
                        log.info(f'{amr_id}: phase {hs.phase.value}')
                    time.sleep(self._cycle_s)
            finally:
                # Never leave a request latched on the PLC, whatever ended the loop.
                link.write(RELEASED)
            result.phase = hs.phase.value
            result.error = hs.error
            result.success = hs.phase is Phase.SUCCEEDED
        except LINK_ERRORS as exc:
            self._drop_link()
            result.phase = 'link_error'
            result.error = f'station link: {exc!r}'
        return result

    def _connect(self) -> StationLink:
        if self._link is not None:
            return self._link
        deadline = time.monotonic() + self._connect_timeout_s
        while True:
            try:
                self._link = StationLink(self._endpoint, self._signal_map, self._tloop)
                return self._link
            except LINK_ERRORS:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)

    def _drop_link(self):
        if self._link is not None:
            try:
                self._link.close()
            except LINK_ERRORS:
                pass   # the session is already broken; the next goal reconnects
            self._link = None

    def close(self):
        self._drop_link()
        self._tloop.stop()


def main(args=None):
    rclpy.init(args=args)
    node = StationBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
