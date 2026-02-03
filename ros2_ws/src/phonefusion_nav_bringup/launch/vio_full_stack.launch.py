from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    video_url = LaunchConfiguration('video_url')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([

        DeclareLaunchArgument(
            'video_url',
            default_value='http://192.168.1.114:4747/video'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=''
        ),

        # Camera
        Node(
            package='ipcam_bridge',
            executable='ipcam_pub',
            name='ipcam_pub',
            output='screen',
            parameters=[{'video_url': video_url}],
        ),

        # Feature tracker
        Node(
            package='feature_tracker',
            executable='track',
            name='feature_tracker',
            output='screen',
        ),

        # Visual Odometry
        Node(
            package='vo_estimator',
            executable='vo',
            name='vo_estimator',
            output='screen',
        ),

        # Phone IMU + GPS
        Node(
            package='phone_imu_bridge',
            executable='phone_imu_bridge_node',
            name='phone_imu_bridge',
            output='screen',
        ),

        # VIO fusion (VO + IMU)
        Node(
            package='vio_fuser',
            executable='fuse',
            name='vio_fuser',
            output='screen',
            parameters=[{
                'zupt_accel_thresh': 0.08,
                'zupt_gyro_thresh': 0.06,
                'zupt_count': 6,
                'zupt_mode': 'damp',
                'zupt_damp': 0.3,
            }],
        ),

        # GPS anchor for VIO
        Node(
            package='gps_anchor_fuser',
            executable='gps_anchor_node',
            name='gps_anchor',
            output='screen',
            parameters=[{
                'input_odom_topic': '/vio_fused/odom',
                'gps_fix_topic': '/gps/fix',
                'out_odom_topic': '/vio_gps/odom',
                'out_path_topic': '/vio_gps/path',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'gps_alpha': 0.03,
                'max_hacc_m': 15.0,
                'max_gps_step_m': 12.0,
                'correct_xy_only': True,
                'gate_gps_when_stationary': True,
                'min_speed_for_gps_update': 0.25,
                'min_dt_between_gps_updates': 0.2,
            }],
        ),

        # RViz (optional)
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
