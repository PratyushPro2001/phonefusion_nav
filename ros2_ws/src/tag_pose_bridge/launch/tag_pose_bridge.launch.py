from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tag_pose_bridge',
            executable='tag_pose_bridge',  # from entry_points.txt
            name='tag_pose_bridge',
            parameters=[
                {
                    'detections_topic': '/detections',
                }
            ],
            output='screen',
        ),
    ])
