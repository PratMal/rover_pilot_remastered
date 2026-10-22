"""Base station: PS5 controller -> /cmd_vel + e-stop heartbeat.

    ros2 launch rover_pilot_remastered base_station.launch.py [wheels:=wheel_28cm] [device_id:=0]
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
        DeclareLaunchArgument('device_id', default_value='0', description='joystick index for SDL'),
        # joy_node (the raw SDL joystick) reads this PS5's axes reliably; game_controller_node did not. deadzone 0 keeps
        # /joy flowing while the sticks rest (joy only autorepeats when no event arrives, and stick jitter never stops).
        Node(package='joy', executable='joy_node', name='joy', output='screen',
             parameters=[{'device_id': LaunchConfiguration('device_id'), 'deadzone': 0.0, 'autorepeat_rate': 20.0}]),
        Node(package='rover_pilot_remastered', executable='base_teleop', name='base_teleop', output='screen',
             parameters=[str(share / 'config' / 'drive.yaml'), wheels]),
    ])
