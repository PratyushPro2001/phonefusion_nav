from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000'),
        DeclareLaunchArgument('record_path', default_value=''),
        DeclareLaunchArgument('time_mode', default_value='offset'),
        Node(package='sensor_logger_bridge', executable='bridge', name='sensor_logger_bridge',
             output='screen',
             parameters=[{'port': LaunchConfiguration('port'),
                          'record_path': LaunchConfiguration('record_path'),
                          'time_mode': LaunchConfiguration('time_mode')}]),
    ])
