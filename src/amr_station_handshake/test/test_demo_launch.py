"""End-to-end: real station simulators, real bridges, a real AMR node.

A normal station must let the AMR finish two full missions and exit 0.
A station that faults mid-transfer must make a Dock goal come back aborted
with the fault reported.
"""

import unittest

from action_msgs.msg import GoalStatus
from amr_handshake_interfaces.action import Dock
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.action import ActionClient

PKG = 'amr_station_handshake'


def station(port, scenario):
    return launch_ros.actions.Node(
        package=PKG, executable='station_sim',
        arguments=['--port', str(port), '--scenario', scenario],
        output='screen')


def bridge(port, namespace=None):
    return launch_ros.actions.Node(
        package=PKG, executable='station_bridge', namespace=namespace,
        parameters=[{'endpoint': f'opc.tcp://127.0.0.1:{port}/station/'}],
        output='screen')


@pytest.mark.launch_test
def generate_test_description():
    amr = launch_ros.actions.Node(
        package=PKG, executable='amr_node',
        parameters=[{'missions': 2}],
        output='screen')
    return launch.LaunchDescription([
        station(4861, 'normal'),
        bridge(4861),
        amr,
        station(4862, 'fault_during_transfer'),
        bridge(4862, namespace='faulty'),
        launch_testing.actions.ReadyToTest(),
    ]), {'amr': amr}


class TestHandover(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('handover_test')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_amr_completes_its_missions(self, proc_info, amr):
        proc_info.assertWaitForShutdown(process=amr, timeout=90)

    def test_faulty_station_aborts_the_handover(self):
        client = ActionClient(self.node, Dock, '/faulty/station/dock')
        self.assertTrue(client.wait_for_server(timeout_sec=30.0))

        sent = client.send_goal_async(Dock.Goal(amr_id='test_amr'))
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=10.0)
        handle = sent.result()
        self.assertIsNotNone(handle)
        self.assertTrue(handle.accepted)

        pending = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, pending, timeout_sec=30.0)
        response = pending.result()
        self.assertIsNotNone(response)
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.success)
        self.assertEqual(response.result.phase, 'faulted')
        self.assertIn('code 1', response.result.error)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):

    def test_amr_exited_cleanly(self, proc_info, amr):
        launch_testing.asserts.assertExitCodes(proc_info, process=amr)
