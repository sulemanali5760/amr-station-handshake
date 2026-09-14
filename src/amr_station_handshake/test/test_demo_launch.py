"""
End-to-end: real station simulators, real bridges, a real AMR node.

Every failure path is checked from the PLC side as well as the ROS side:
after a handover ends badly, the station must see both request bits low.
"""

import time
import unittest

from action_msgs.msg import GoalStatus
from amr_handshake_interfaces.action import Dock
from asyncua import ua
from asyncua.sync import Client
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.action import ActionClient

PKG = 'amr_station_handshake'
NAMESPACE = 'urn:amr-station-handshake:station'


def station(port, *args):
    return launch_ros.actions.Node(
        package=PKG, executable='station_sim',
        arguments=['--port', str(port), *args],
        output='screen')


def bridge(port, namespace=None, **parameters):
    return launch_ros.actions.Node(
        package=PKG, executable='station_bridge', namespace=namespace,
        parameters=[{'endpoint': f'opc.tcp://127.0.0.1:{port}/station/', **parameters}],
        output='screen')


@pytest.mark.launch_test
def generate_test_description():
    amr = launch_ros.actions.Node(
        package=PKG, executable='amr_node',
        parameters=[{'missions': 2}],
        output='screen')
    return launch.LaunchDescription([
        station(4861), bridge(4861), amr,
        station(4862, '--scenario', 'fault_during_transfer'), bridge(4862, 'faulty'),
        station(4863, '--scenario', 'never_ready'),
        # timeout.request is an int on purpose: integer seconds must be accepted.
        bridge(4863, 'silent', **{'timeout.request': 1}),
        station(4864, '--transfer-time', '20'), bridge(4864, 'cancel'),
        station(4865, '--scenario', 'crash_during_transfer'), bridge(4865, 'crash'),
        station(4866),                           # no bridge: the test itself plays the AMR side
        launch_testing.actions.ReadyToTest(),
    ]), {'amr': amr}


class StationProbe:
    """Direct OPC UA access to a simulated station, bypassing the bridge."""

    def __init__(self, port):
        self.client = Client(f'opc.tcp://127.0.0.1:{port}/station/', timeout=2)
        deadline = time.monotonic() + 15.0
        while True:
            try:
                self.client.connect()
                break
            except (OSError, TimeoutError):
                if time.monotonic() > deadline:
                    self.client.tloop.stop()     # its thread would keep pytest alive
                    raise
                time.sleep(0.5)
        self.ns = self.client.get_namespace_index(NAMESPACE)

    def node(self, name):
        return self.client.get_node(ua.NodeId(f'Station.{name}', self.ns))

    def read(self, name):
        return self.node(name).read_value()

    def close(self):
        self.client.disconnect()


class TestHandover(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('handover_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def send_goal(self, action, feedback_callback=None):
        client = ActionClient(self.node, Dock, action)
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))
        sent = client.send_goal_async(Dock.Goal(amr_id='test_amr'), feedback_callback)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        handle = sent.result()
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)
        return client, handle

    def wait_result(self, handle, timeout=30.0):
        pending = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, pending, timeout_sec=timeout)
        response = pending.result()
        self.assertIsNotNone(response)
        return response

    def assert_released(self, port):
        probe = StationProbe(port)
        try:
            self.assertFalse(probe.read('AmrDockRequest'))
            self.assertFalse(probe.read('AmrInPosition'))
        finally:
            probe.close()

    def test_amr_completes_its_missions(self, proc_info, amr):
        proc_info.assertWaitForShutdown(process=amr, timeout=90)

    def test_faulty_station_aborts_and_releases(self):
        _client, handle = self.send_goal('/faulty/station/dock')
        response = self.wait_result(handle)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.phase, 'faulted')
        self.assertIn('code 1', response.result.error)
        self.assert_released(4862)

    def test_silent_station_times_out_and_releases(self):
        _client, handle = self.send_goal('/silent/station/dock')
        response = self.wait_result(handle)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.phase, 'timed_out')
        self.assertIn("'request'", response.result.error)
        self.assert_released(4863)

    def test_cancel_mid_transfer_releases(self):
        phases = []
        _client, handle = self.send_goal(
            '/cancel/station/dock', lambda msg: phases.append(msg.feedback.phase))
        deadline = time.monotonic() + 15.0
        while 'transfer' not in phases and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertIn('transfer', phases)

        cancel = handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel, timeout_sec=10.0)
        response = self.wait_result(handle)
        self.assertEqual(response.status, GoalStatus.STATUS_CANCELED)
        self.assertEqual(response.result.phase, 'canceled')
        self.assert_released(4864)

    def test_station_crash_mid_transfer_is_a_link_error(self):
        _client, handle = self.send_goal('/crash/station/dock')
        response = self.wait_result(handle)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(response.result.phase, 'link_error')

    def test_station_watchdog_faults_when_the_heartbeat_stops(self):
        probe = StationProbe(4866)
        try:
            request, heartbeat = probe.node('AmrDockRequest'), probe.node('AmrHeartbeat')
            # Like the bridge: count from 1 (the station starts at 0, so 0 would not count as a
            # beat) and beat a few cycles before raising the request.
            for count in range(1, 25):               # about one second of a live AMR side
                heartbeat.write_value(count, ua.VariantType.Int16)
                if count == 5:
                    request.write_value(True, ua.VariantType.Boolean)
                time.sleep(0.05)
            self.assertTrue(probe.read('StationReady'))
            self.assertFalse(probe.read('Fault'))

            deadline = time.monotonic() + 3.0         # watchdog is 1 s
            while not probe.read('Fault') and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(probe.read('Fault'))
            self.assertEqual(probe.read('FaultCode'), 2)
            self.assertFalse(probe.read('StationReady'))

            request.write_value(False, ua.VariantType.Boolean)   # acknowledge
            time.sleep(0.3)
            self.assertFalse(probe.read('Fault'))
        finally:
            probe.close()


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):

    def test_amr_exited_cleanly(self, proc_info, amr):
        launch_testing.asserts.assertExitCodes(proc_info, process=amr)
