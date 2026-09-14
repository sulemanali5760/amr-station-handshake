"""Simulated AMR: drive to the station, dock through the Dock action, drive back.

Driving is a 1-D position ramp published on amr/position. The node exits with
code 0 after the configured number of missions, or 1 as soon as a handover
fails, so a launch test can judge a whole mission by its exit code.
"""

import sys

from action_msgs.msg import GoalStatus
from amr_handshake_interfaces.action import Dock
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64

TICK_S = 0.1


class AmrNode(Node):

    def __init__(self):
        super().__init__('amr')
        self._amr_id = self.declare_parameter('amr_id', 'amr_01').value
        self._station = self.declare_parameter('station_position', 5.0).value
        self._speed = self.declare_parameter('speed', 1.0).value
        self._missions = self.declare_parameter('missions', 1).value
        self._server_timeout_s = self.declare_parameter('server_timeout_s', 15.0).value

        self._position_pub = self.create_publisher(Float64, 'amr/position', 10)
        self._dock = ActionClient(self, Dock, 'station/dock')

        self._position = 0.0
        self._target = self._station
        self._state = 'drive_to_station'
        self._waited_s = 0.0
        self._completed = 0
        self.exit_code = None

        self.create_timer(TICK_S, self._tick)
        self.get_logger().info(f'{self._amr_id}: mission 1 of {self._missions}, driving to station')

    def _tick(self):
        if self._state in ('drive_to_station', 'drive_home'):
            self._drive()
        elif self._state == 'waiting_for_station':
            self._send_dock_goal()

    def _drive(self):
        step = self._speed * TICK_S
        delta = self._target - self._position
        self._position = self._target if abs(delta) <= step else self._position + step * (1 if delta > 0 else -1)
        self._position_pub.publish(Float64(data=self._position))
        if self._position != self._target:
            return

        if self._state == 'drive_to_station':
            self.get_logger().info(f'{self._amr_id}: at station, requesting handover')
            self._state = 'waiting_for_station'
            self._waited_s = 0.0
            self._send_dock_goal()
            return

        self._completed += 1
        if self._completed >= self._missions:
            self.get_logger().info(f'{self._amr_id}: all {self._missions} missions complete')
            self.exit_code = 0
            return
        self.get_logger().info(
            f'{self._amr_id}: mission {self._completed + 1} of {self._missions}, driving to station')
        self._state, self._target = 'drive_to_station', self._station

    def _send_dock_goal(self):
        if not self._dock.server_is_ready():
            self._waited_s += TICK_S
            if self._waited_s > self._server_timeout_s:
                self._fail(f'no Dock action server after {self._server_timeout_s:.0f} s')
            return
        self._state = 'docking'
        future = self._dock.send_goal_async(
            Dock.Goal(amr_id=self._amr_id), feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, msg):
        self.get_logger().info(f'{self._amr_id}: station phase {msg.feedback.phase}')

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self._fail('station rejected the handover')
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        response = future.result()
        result = response.result
        if response.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self._fail(f'handover {result.phase}: {result.error}')
            return
        self.get_logger().info(f'{self._amr_id}: load handed over, driving home')
        self._state, self._target = 'drive_home', 0.0

    def _fail(self, reason):
        self.get_logger().error(f'{self._amr_id}: {reason}')
        self._state = 'failed'
        self.exit_code = 1


def main(args=None):
    rclpy.init(args=args)
    node = AmrNode()
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=TICK_S)
    except KeyboardInterrupt:
        pass
    code = 0 if node.exit_code is None else node.exit_code
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
