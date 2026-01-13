#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_norm(q):
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if n < 1e-12:
        return None
    q.x /= n; q.y /= n; q.z /= n; q.w /= n
    return q


class ImuFuse(Node):
    """
    Minimal, sane fusion:
      - Position from VO odom (x,y,z)
      - Orientation from IMU (roll/pitch/yaw)
      - Publishes /vio_fused/odom and /vio_fused/path
      - Optionally broadcasts TF map -> base_link
    """
    def __init__(self):
        super().__init__('imu_fuse')

        # Topics
        self.declare_parameter('vo_odom_topic', '/vio/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('out_odom_topic', '/vio_fused/odom')
        self.declare_parameter('out_path_topic', '/vio_fused/path')

        # Frames
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        # Behavior
        self.declare_parameter('use_imu_orientation', True)   # assumes Imu.orientation is valid
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('path_stride', 1)

        self.vo_odom_topic = self.get_parameter('vo_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.out_odom_topic = self.get_parameter('out_odom_topic').value
        self.out_path_topic = self.get_parameter('out_path_topic').value

        self.world_frame = self.get_parameter('world_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.use_imu_orientation = bool(self.get_parameter('use_imu_orientation').value)
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)
        self.path_stride = int(self.get_parameter('path_stride').value)

        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 10)
        self.tf_br = TransformBroadcaster(self) if self.broadcast_tf else None

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self._count = 0

        # Sync VO odom + IMU
        self.vo_sub = Subscriber(self, Odometry, self.vo_odom_topic)
        self.imu_sub = Subscriber(self, Imu, self.imu_topic)
        self.sync = ApproximateTimeSynchronizer([self.vo_sub, self.imu_sub], queue_size=30, slop=0.10)
        self.sync.registerCallback(self.cb)

        self.get_logger().info(f"Fusing VO odom: {self.vo_odom_topic}")
        self.get_logger().info(f"With IMU:       {self.imu_topic}")
        self.get_logger().info(f"Publishing:     {self.out_odom_topic}, {self.out_path_topic}")

    def cb(self, vo: Odometry, imu: Imu):
        out = Odometry()
        out.header.stamp = vo.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.child_frame

        # Position: copy from VO
        out.pose.pose.position = vo.pose.pose.position

        # Orientation: replace with IMU orientation (normalized)
        if self.use_imu_orientation:
            q = imu.orientation
            q = quat_norm(q)
            if q is not None:
                out.pose.pose.orientation = q
            else:
                out.pose.pose.orientation = vo.pose.pose.orientation
        else:
            out.pose.pose.orientation = vo.pose.pose.orientation

        # Copy twist as-is (still zero in your VO; fine)
        out.twist = vo.twist

        self.odom_pub.publish(out)

        # Path
        self._count += 1
        if self.path_stride <= 1 or (self._count % self.path_stride == 0):
            ps = PoseStamped()
            ps.header = out.header
            ps.pose = out.pose.pose
            self.path.header.stamp = out.header.stamp
            self.path.poses.append(ps)
            self.path_pub.publish(self.path)

        # TF map -> base_link
        if self.tf_br is not None:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.child_frame
            t.transform.translation.x = float(out.pose.pose.position.x)
            t.transform.translation.y = float(out.pose.pose.position.y)
            t.transform.translation.z = float(out.pose.pose.position.z)
            t.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = ImuFuse()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
