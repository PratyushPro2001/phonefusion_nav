from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_orientation_viz',
            executable='imu_orientation_viz',
            name='imu_orientation_viz',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'update_rate_hz': 50.0
            }]
        )
    ])
