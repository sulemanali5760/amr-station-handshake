"""
Station simulator, bridge and one AMR running three missions.

ros2 launch amr_station_handshake demo.launch.py scenario:=fault_during_transfer
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='normal',
            description='normal | never_ready | fault_during_transfer | crash_during_transfer'),
        Node(
            package='amr_station_handshake', executable='station_sim',
            arguments=['--scenario', LaunchConfiguration('scenario')],
            output='screen'),
        Node(
            package='amr_station_handshake', executable='station_bridge',
            output='screen'),
        Node(
            package='amr_station_handshake', executable='amr_node',
            parameters=[{'missions': 3}],
            output='screen'),
    ])
