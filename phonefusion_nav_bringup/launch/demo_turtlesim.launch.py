from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Turtlesim (just as a simple visual node in the system)
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim"
    )

    # Phone IMU bridge node (already implemented)
    imu_bridge_node = Node(
        package="phone_imu_bridge",
        executable="imu_bridge",
        name="phone_imu_bridge",
        output="screen"
    )

    # 2D IMU-based odometry node (new ekf2d package)
    ekf2d_node = Node(
        package="ekf2d",
        executable="ekf2d_node",
        name="ekf2d",
        output="screen",
        parameters=[{
            "update_rate_hz": 50.0,
            "use_accel_for_position": True,
            "frame_id": "odom",
            "child_frame_id": "base_link",
        }]
    )

    return LaunchDescription([
        turtlesim_node,
        imu_bridge_node,
        ekf2d_node,
    ])
