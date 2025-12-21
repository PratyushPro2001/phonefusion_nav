#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler


class ImuDeadReckoning(Node):
    def __init__(self):
        super().__init__('imu_deadreckoning')

        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.last_time = None

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/imu/odom', 10)
        self.path_pub = self.create_publisher(Path, '/imu/path', 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time
        self.last_time = t

        wz = msg.angular_velocity.z
        self.yaw += wz * dt

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        c = np.cos(self.yaw)
        s = np.sin(self.yaw)
        ax_w = c * ax - s * ay
        ay_w = s * ax + c * ay

        self.vx += ax_w * dt
        self.vy += ay_w * dt

        self.x += self.vx * dt
        self.y += self.vy * dt

        self.publish_odometry(msg.header.stamp)
        self.publish_path(msg.header.stamp)

    def publish_odometry(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy

        self.odom_pub.publish(odom)

    def publish_path(self, stamp):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = ImuDeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
