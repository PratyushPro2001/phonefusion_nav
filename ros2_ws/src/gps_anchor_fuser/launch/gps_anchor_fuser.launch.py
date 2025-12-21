from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_anchor_fuser',
            executable='gps_anchor_node',
            name='gps_anchor_fuser',
            output='screen',
            parameters=[{
                'gps_fix_topic': '/gps/fix',
                'imu_odom_topic': '/imu/odom',
                'out_odom_topic': '/fused/odom',
                'out_path_topic': '/fused/path',
                'gps_alpha': 0.02,
                'max_hacc_m': 20.0,
                'max_gps_step_m': 10.0,
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
            }]
        )
    ])
