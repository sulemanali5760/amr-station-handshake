"""ROS 2 action server that runs the Dock handshake against a station PLC over OPC UA."""

import threading
import time

from amr_handshake_interfaces.action import Dock
from asyncua.sync import Client, ThreadLoop
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from amr_station_handshake.handshake import (
    AmrSignals, Handshake, Phase, RELEASED, StationSignals, Timeouts)
from amr_station_handshake.station_sim import NAMESPACE


class StationLink:
    """The station's handshake signals over one OPC UA session."""

    def __init__(self, endpoint: str, tloop: ThreadLoop):
        self._client = Client(endpoint, timeout=2, tloop=tloop)
        self._client.connect()
        ns = self._client.get_namespace_index(NAMESPACE)

        def node(name):
            return self._client.get_node(f'ns={ns};s=Station.{name}')

        self._ready = node('StationReady')
        self._active = node('TransferActive')
        self._done = node('TransferDone')
        self._fault = node('Fault')
        self._fault_code = node('FaultCode')
        self._dock_request = node('AmrDockRequest')
        self._in_position = node('AmrInPosition')
        self._written = None

    def read(self) -> StationSignals:
        return StationSignals(
            ready=self._ready.read_value(),
            transfer_active=self._active.read_value(),
            transfer_done=self._done.read_value(),
            fault=self._fault.read_value(),
            fault_code=self._fault_code.read_value(),
        )

    def write(self, signals: AmrSignals) -> None:
        if signals == self._written:
            return
        self._dock_request.write_value(signals.dock_request)
        self._in_position.write_value(signals.in_position)
        self._written = signals

    def close(self) -> None:
        self._client.disconnect()


class StationBridge(Node):

    def __init__(self):
        super().__init__('station_bridge')
        self._endpoint = self.declare_parameter(
            'endpoint', 'opc.tcp://127.0.0.1:4860/station/').value
        self._cycle_s = self.declare_parameter('cycle_s', 0.05).value
        self._connect_timeout_s = self.declare_parameter('connect_timeout_s', 10.0).value
        defaults = Timeouts()
        self._timeouts = Timeouts(
            request=self.declare_parameter('timeout.request', defaults.request).value,
            dock=self.declare_parameter('timeout.dock', defaults.dock).value,
            transfer=self.declare_parameter('timeout.transfer', defaults.transfer).value,
            release=self.declare_parameter('timeout.release', defaults.release).value,
        )

        self._tloop = ThreadLoop()
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

    def _on_goal(self, goal):
        with self._lock:
            if self._busy:
                self.get_logger().warn(f'Rejecting {goal.amr_id}: a handover is already running')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _connect(self) -> StationLink:
        if self._link is not None:
            return self._link
        deadline = time.monotonic() + self._connect_timeout_s
        while True:
            try:
                self._link = StationLink(self._endpoint, self._tloop)
                return self._link
            except Exception:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.5)

    def _execute(self, goal_handle):
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
        except Exception as exc:  # any OPC UA or network error ends this handover
            self._drop_link()
            result.phase = 'connection_error'
            result.error = f'station link: {exc!r}'
        finally:
            with self._lock:
                self._busy = False

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

    def _drop_link(self):
        if self._link is not None:
            try:
                self._link.close()
            except Exception:
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
