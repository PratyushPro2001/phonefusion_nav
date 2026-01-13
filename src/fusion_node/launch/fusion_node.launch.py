from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fusion_node',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'update_rate_hz': 50.0,
                'gps_alpha': 0.2,
            }]
        )
    ])
