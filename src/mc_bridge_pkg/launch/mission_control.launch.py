"""Start the mission control bridge node and the Zenoh bridge to the rover."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rover_ip = LaunchConfiguration('rover_ip')
    zenoh_port = LaunchConfiguration('zenoh_port')

    return LaunchDescription([
        DeclareLaunchArgument('rover_ip', description='IP address of the rover'),
        DeclareLaunchArgument(
            'zenoh_port',
            default_value='7447',
            description='Port the rover Zenoh bridge listens on',
        ),
        # Connects out to the rover's Zenoh bridge; restarted if it exits
        ExecuteProcess(
            cmd=['zenoh-bridge-ros2dds', '-e', ['tcp/', rover_ip, ':', zenoh_port]],
            name='zenoh_bridge',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        Node(package='mc_bridge_pkg', executable='bridge_node', output='screen'),
    ])
