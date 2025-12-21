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
        )
    ])
