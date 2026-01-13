from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom_plotter',
            executable='odom_plotter',
            name='odom_plotter',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'max_points': 5000,
            }],
        )
    ])
