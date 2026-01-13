from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(cmd=['ros2','bag','record','-o','bags/mvp',
                             '/imu/data','/camera/image_raw','/vision/pose','/odom','/plan','/cmd_vel'])
    ])
