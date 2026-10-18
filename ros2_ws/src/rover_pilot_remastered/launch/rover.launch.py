"""Rover: /cmd_vel + e-stop heartbeat -> drive ESP32.

    ros2 launch rover_pilot_remastered rover.launch.py [wheels:=wheel_28cm] [port:=/dev/rover-drive]
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('rover_pilot_remastered'))
    wheels = PathJoinSubstitution([str(share / 'config' / 'wheels'), [LaunchConfiguration('wheels'), '.yaml']])
    return LaunchDescription([
        DeclareLaunchArgument('wheels', default_value='wheel_28cm', description='wheel profile in config/wheels'),
        DeclareLaunchArgument('port', default_value='/dev/rover-drive', description='drive ESP32 serial port'),
        Node(package='rover_pilot_remastered', executable='drive_node', name='drive_node', output='screen',
             parameters=[str(share / 'config' / 'drive.yaml'), wheels, {'port': LaunchConfiguration('port')}]),
    ])
