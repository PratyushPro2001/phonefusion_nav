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

                'imu_topic': '/imu/data',
                'imu_frame_id': 'imu_link',

                'gps_topic': '/gps/fix',
                'gps_frame_id': 'gps_link',

                'accel_scale': 1.0,
                'gyro_scale': 1.0,

                'swap_xy': False,
                'swap_xz': False,
                'swap_yz': False,
                'flip_x': False,
                'flip_y': False,
                'flip_z': False,

                'use_orientation_if_available': True,
            }]
        )
    ])
