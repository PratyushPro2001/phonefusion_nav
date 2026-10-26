"""Bridge + EKF + Foxglove websocket in one command."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('record_path', default_value=''),
        Node(package='sensor_logger_bridge', executable='bridge', name='sensor_logger_bridge', output='screen',
             parameters=[{'port': LaunchConfiguration('port'), 'record_path': LaunchConfiguration('record_path')}]),
        Node(package='phone_ekf', executable='ekf_node', name='phone_ekf', output='screen'),
        Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge', output='screen',
             parameters=[{'port': 8765, 'address': '0.0.0.0'}]),
    ])
