from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Laptop webcam camera node
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{'camera': 0}],
        ),

        # AprilTag detector (2D detections)
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            remappings=[
                ('image_rect', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),

        # Bridge detections -> /tag_pose (pixel-frame PoseStamped)
        Node(
            package='tag_pose_bridge',
            executable='tag_pose_bridge',
            name='tag_pose_bridge',
            output='screen',
            parameters=[{'detections_topic': '/detections'}],
        ),
    ])
