from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) Phone IMU UDP bridge
        Node(
            package='phone_imu_bridge',
            executable='imu_bridge',
            name='phone_imu_bridge',
            output='screen'
        ),

        # 2) 2D yaw-only "EKF" node
        Node(
            package='ekf2d',
            executable='ekf2d_node',
            name='ekf2d_node',
            output='screen'
        ),

        # 3) 2D heading arrow plotter
        Node(
            package='odom_plotter',
            executable='odom_plotter',
            name='odom_plotter',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'arrow_length': 0.5,
            }]
        ),

        # 4) 3D IMU orientation axes
        Node(
            package='imu_orientation_viz',
            executable='imu_orientation_viz',
            name='imu_orientation_viz',
            output='screen',
            parameters=[{
                'imu_topic': '/imu/data',
                'update_rate_hz': 50.0,
            }]
        ),
    ])
