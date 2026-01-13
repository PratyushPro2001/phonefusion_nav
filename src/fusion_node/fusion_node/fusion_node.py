#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_from_quaternion(qx, qy, qz, qw):
    """
    Extract yaw from quaternion, assuming Z-up (ENU).
    """
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class FusionNode(Node):
    """
    Minimal fusion node:

    - Subscribes:
        /imu/data   (sensor_msgs/Imu)           -> orientation / yaw
        /gps/pose   (geometry_msgs/PoseStamped) -> global XY in meters
        /tag_pose   (geometry_msgs/PoseStamped) -> optional indoor correction (future)

    - Publishes:
        /fusion/odom (nav_msgs/Odometry) with:
            position = filtered GPS XY
            orientation = IMU-based yaw
    """

    def __init__(self):
        super().__init__('fusion_node')

        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('gps_alpha', 0.1)  # smoothing factor for GPS position [0..1]

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        self.update_rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        self.gps_alpha = self.get_parameter('gps_alpha').get_parameter_value().double_value

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.gps_sub = self.create_subscription(PoseStamped, '/gps/pose', self.gps_callback, 10)
        self.tag_sub = self.create_subscription(PoseStamped, '/tag_pose', self.tag_callback, 10)  # not yet used

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/fusion/odom', 20)

        # State
        self.latest_imu = None
        self.have_imu = False

        self.latest_gps_x = 0.0
        self.latest_gps_y = 0.0
        self.have_gps = False

        self.fused_x = 0.0
        self.fused_y = 0.0
        self.have_fused_pos = False

        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        # Timer for periodic publish
        period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            f'FusionNode started: frame_id={self.frame_id}, child_frame_id={self.child_frame_id}, '
            f'update_rate_hz={self.update_rate_hz}, gps_alpha={self.gps_alpha}'
        )

    # ---------------- Callbacks ----------------

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        self.have_imu = True

    def gps_callback(self, msg: PoseStamped):
        self.latest_gps_x = msg.pose.position.x
        self.latest_gps_y = msg.pose.position.y
        self.have_gps = True

        if not self.have_fused_pos:
            self.fused_x = self.latest_gps_x
            self.fused_y = self.latest_gps_y
            self.have_fused_pos = True
            self.get_logger().info(
                f'Initialized fused position from GPS: x={self.fused_x:.2f}, y={self.fused_y:.2f}'
            )

    def tag_callback(self, msg: PoseStamped):
        # Placeholder for future AprilTag-based corrections.
        # For now we just store it or ignore; real logic can be added later.
        pass

    # ---------------- Main update loop ----------------

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0
        self.last_time = now

        # --- Orientation / yaw from IMU ---
        if self.have_imu:
            q = self.latest_imu.orientation
            if (
                abs(q.w) > 1e-6 or
                abs(q.x) > 1e-6 or
                abs(q.y) > 1e-6 or
                abs(q.z) > 1e-6
            ):
                # Use quaternion directly (ideal when phone provides fused orientation)
                self.yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            else:
                # Fallback: integrate gyro.z if orientation is identity / uninitialized
                gz = self.latest_imu.angular_velocity.z
                self.yaw += gz * dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # --- Position fusion (GPS only for now, smoothed) ---
        if self.have_gps:
            if not self.have_fused_pos:
                self.fused_x = self.latest_gps_x
                self.fused_y = self.latest_gps_y
                self.have_fused_pos = True
            else:
                a = self.gps_alpha
                self.fused_x = (1.0 - a) * self.fused_x + a * self.latest_gps_x
                self.fused_y = (1.0 - a) * self.fused_y + a * self.latest_gps_y

        if not self.have_fused_pos:
            # Nothing to publish yet
            return

        # --- Build Odometry message ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        # Position from fused state
        odom.pose.pose.position.x = self.fused_x
        odom.pose.pose.position.y = self.fused_y
        odom.pose.pose.position.z = 0.0

        # Orientation from yaw
        half_yaw = 0.5 * self.yaw
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)

        # Velocity left as zero for now
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('FusionNode shutting down (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
