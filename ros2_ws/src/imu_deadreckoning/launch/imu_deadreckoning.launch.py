from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_deadreckoning',
            executable='imu_deadreckoning_node',
            name='imu_deadreckoning',
            output='screen'
        )
    ])
