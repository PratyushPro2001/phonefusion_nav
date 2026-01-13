from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='phone_imu_bridge',
            executable='phone_imu_bridge_node',
            name='phone_imu_bridge',
            output='screen',
            parameters=[{
                'bind_ip': '0.0.0.0',
                'port': 5555,
                'topic': '/imu/data',
                'frame_id': 'imu_link',
                'accel_scale': 1.0,
                'gyro_scale': 1.0,
                'swap_xy': False,
                'swap_xz': False,
                'swap_yz': False,
                'flip_x': False,
                'flip_y': False,
                'flip_z': False,
                'use_orientation_if_available': True,
                'orientation_from_rpy_if_available': True,
            }]
        ),

        Node(
            package='imu_deadreckoning',
            executable='imu_deadreckoning_node',
            name='imu_deadreckoning',
            output='screen'
        ),

        
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
        ),

Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/pratyush/ros2_ws/src/phonefusion_nav/phonefusion_nav_bringup/rviz/phase2_imu.rviz']
        ),

    ])
