from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_bridge',
            executable='gps_bridge',
            name='gps_bridge',
            output='screen',
            parameters=[{
                'udp_port': 6000,
                'frame_id': 'map',
                'publish_rate_hz': 10.0,
            }]
        )
    ])
