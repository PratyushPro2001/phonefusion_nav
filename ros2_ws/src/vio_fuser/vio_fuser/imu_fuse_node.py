#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer


def quat_normalized(x, y, z, w):
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return None
    return (x/n, y/n, z/n, w/n)


def quat_to_rotmat(qx, qy, qz, qw):
    # v_world = R * v_body (assuming quaternion maps body->world)
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)

    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)

    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)

    return (r00, r01, r02,
            r10, r11, r12,
            r20, r21, r22)


def rot_apply(R, x, y, z):
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = R
    return (
        r00*x + r01*y + r02*z,
        r10*x + r11*y + r12*z,
        r20*x + r21*y + r22*z
    )


class ImuFuse(Node):
    """
    Minimal, controlled fusion:
      - Position from VO odom (x,y,z)
      - Orientation from IMU quaternion (normalized)
      - Velocity from IMU linear_acceleration rotated to world frame and integrated
      - ZUPT: when stationary, damp or clamp velocity to fight drift
      - IMPORTANT: VO-motion gate prevents ZUPT from sticking during motion
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
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('path_stride', 1)

        # Accel->vel integration
        self.declare_parameter('max_dt', 0.2)
        self.declare_parameter('accel_lpf_alpha', 0.2)        # 0..1, higher = less smoothing
        self.declare_parameter('use_world_accel', True)

        # ZUPT (stationary detection)
        self.declare_parameter('zupt_enable', True)
        self.declare_parameter('zupt_accel_thresh', 0.08)     # m/s^2
        self.declare_parameter('zupt_gyro_thresh', 0.06)      # rad/s
        self.declare_parameter('zupt_count', 6)               # consecutive samples required
        self.declare_parameter('zupt_mode', 'clamp')          # 'damp' or 'clamp'
        self.declare_parameter('zupt_damp', 0.6)              # per-update factor when stationary (0..1)

        # VO-motion gate (prevents ZUPT during motion)
        self.declare_parameter('zupt_use_vo_gate', True)
        self.declare_parameter('zupt_vo_speed_thresh', 0.03)  # m/s (VO-derived). If above -> NOT stationary.

        self.vo_odom_topic = self.get_parameter('vo_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.out_odom_topic = self.get_parameter('out_odom_topic').value
        self.out_path_topic = self.get_parameter('out_path_topic').value

        self.world_frame = self.get_parameter('world_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.use_imu_orientation = bool(self.get_parameter('use_imu_orientation').value)
        self.broadcast_tf = bool(self.get_parameter('broadcast_tf').value)
        self.path_stride = int(self.get_parameter('path_stride').value)

        self.max_dt = float(self.get_parameter('max_dt').value)
        self.accel_alpha = float(self.get_parameter('accel_lpf_alpha').value)
        self.use_world_accel = bool(self.get_parameter('use_world_accel').value)

        self.zupt_enable = bool(self.get_parameter('zupt_enable').value)
        self.zupt_accel_thresh = float(self.get_parameter('zupt_accel_thresh').value)
        self.zupt_gyro_thresh = float(self.get_parameter('zupt_gyro_thresh').value)
        self.zupt_count_req = int(self.get_parameter('zupt_count').value)
        self.zupt_mode = str(self.get_parameter('zupt_mode').value).strip().lower()
        self.zupt_damp = float(self.get_parameter('zupt_damp').value)

        self.zupt_use_vo_gate = bool(self.get_parameter('zupt_use_vo_gate').value)
        self.zupt_vo_speed_thresh = float(self.get_parameter('zupt_vo_speed_thresh').value)

        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 10)
        self.tf_br = TransformBroadcaster(self) if self.broadcast_tf else None

        self.path = Path()
        self.path.header.frame_id = self.world_frame
        self._count = 0

        # Integration state
        self.last_t = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # LPF accel state (world frame)
        self.ax_f = 0.0
        self.ay_f = 0.0
        self.az_f = 0.0
        self.have_accel_f = False

        # ZUPT state
        self.stationary_count = 0

        # VO motion estimation (from pose deltas)
        self.last_vo_p = None  # (x,y,z)

        # Sync VO odom + IMU
        self.vo_sub = Subscriber(self, Odometry, self.vo_odom_topic)
        self.imu_sub = Subscriber(self, Imu, self.imu_topic)
        self.sync = ApproximateTimeSynchronizer([self.vo_sub, self.imu_sub], queue_size=30, slop=0.10)
        self.sync.registerCallback(self.cb)

        self.get_logger().info(f"Fusing VO odom: {self.vo_odom_topic}")
        self.get_logger().info(f"With IMU:       {self.imu_topic}")
        self.get_logger().info(f"Publishing:     {self.out_odom_topic}, {self.out_path_topic}")
        self.get_logger().info(
            f"ZUPT: {self.zupt_enable} ({self.zupt_mode}), "
            f"VO-gate: {self.zupt_use_vo_gate} @ {self.zupt_vo_speed_thresh} m/s"
        )

    def cb(self, vo: Odometry, imu: Imu):
        out = Odometry()
        out.header.stamp = vo.header.stamp
        out.header.frame_id = self.world_frame
        out.child_frame_id = self.child_frame

        # Position: copy from VO
        out.pose.pose.position = vo.pose.pose.position

        # Orientation: replace with IMU orientation (normalized)
        q_msg = vo.pose.pose.orientation
        if self.use_imu_orientation:
            qn = quat_normalized(
                float(imu.orientation.x),
                float(imu.orientation.y),
                float(imu.orientation.z),
                float(imu.orientation.w),
            )
            if qn is not None:
                qx, qy, qz, qw = qn
                q_msg.x, q_msg.y, q_msg.z, q_msg.w = qx, qy, qz, qw
        out.pose.pose.orientation = q_msg

        # Time step
        if self.last_t is None:
            self.last_t = out.header.stamp
            self.last_vo_p = (
                float(out.pose.pose.position.x),
                float(out.pose.pose.position.y),
                float(out.pose.pose.position.z),
            )
            out.twist = vo.twist
            out.twist.twist.linear.x = float(self.vx)
            out.twist.twist.linear.y = float(self.vy)
            out.twist.twist.linear.z = float(self.vz)
            self.odom_pub.publish(out)
            return

        dt = (out.header.stamp.sec - self.last_t.sec) + (out.header.stamp.nanosec - self.last_t.nanosec) * 1e-9
        self.last_t = out.header.stamp

        if not (0.0 < dt < self.max_dt):
            out.twist = vo.twist
            out.twist.twist.linear.x = float(self.vx)
            out.twist.twist.linear.y = float(self.vy)
            out.twist.twist.linear.z = float(self.vz)
            self.odom_pub.publish(out)
            return

        # --- VO motion gate: compute VO speed from pose delta ---
        vo_speed = 0.0
        p = (
            float(out.pose.pose.position.x),
            float(out.pose.pose.position.y),
            float(out.pose.pose.position.z),
        )
        if self.last_vo_p is not None:
            dx = p[0] - self.last_vo_p[0]
            dy = p[1] - self.last_vo_p[1]
            dz = p[2] - self.last_vo_p[2]
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            vo_speed = dist / dt
        self.last_vo_p = p

        # Raw accel (assumed gravity-compensated from Sensagram)
        ax_b = float(imu.linear_acceleration.x)
        ay_b = float(imu.linear_acceleration.y)
        az_b = float(imu.linear_acceleration.z)

        # Gyro magnitude (for ZUPT)
        gx = float(imu.angular_velocity.x)
        gy = float(imu.angular_velocity.y)
        gz = float(imu.angular_velocity.z)
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)

        # Rotate accel into world frame if enabled and quaternion is valid
        ax_w, ay_w, az_w = ax_b, ay_b, az_b
        if self.use_world_accel and self.use_imu_orientation:
            qn = quat_normalized(
                float(out.pose.pose.orientation.x),
                float(out.pose.pose.orientation.y),
                float(out.pose.pose.orientation.z),
                float(out.pose.pose.orientation.w),
            )
            if qn is not None:
                qx, qy, qz, qw = qn
                R = quat_to_rotmat(qx, qy, qz, qw)
                ax_w, ay_w, az_w = rot_apply(R, ax_b, ay_b, az_b)

        # Low-pass filter accel in world frame
        a = self.accel_alpha
        if not self.have_accel_f:
            self.ax_f, self.ay_f, self.az_f = ax_w, ay_w, az_w
            self.have_accel_f = True
        else:
            self.ax_f = a * ax_w + (1.0 - a) * self.ax_f
            self.ay_f = a * ay_w + (1.0 - a) * self.ay_f
            self.az_f = a * az_w + (1.0 - a) * self.az_f

        # Stationary detection (IMU-only)
        accel_mag = math.sqrt(self.ax_f*self.ax_f + self.ay_f*self.ay_f + self.az_f*self.az_f)
        imu_stationary_now = (accel_mag < self.zupt_accel_thresh) and (gyro_mag < self.zupt_gyro_thresh)

        # VO gate: if VO says moving, force NOT stationary
        if self.zupt_use_vo_gate and (vo_speed > self.zupt_vo_speed_thresh):
            stationary_now = False
        else:
            stationary_now = imu_stationary_now

        if self.zupt_enable:
            if stationary_now:
                self.stationary_count += 1
            else:
                self.stationary_count = 0

        stationary = self.zupt_enable and (self.stationary_count >= self.zupt_count_req)

        # Integrate accel -> velocity, unless stationary (then damp/clamp)
        if stationary:
            if self.zupt_mode == 'clamp':
                self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            else:
                d = self.zupt_damp
                self.vx *= d
                self.vy *= d
                self.vz *= d
        else:
            self.vx += self.ax_f * dt
            self.vy += self.ay_f * dt
            self.vz += self.az_f * dt

        # Output twist: copy VO twist then override linear velocity
        out.twist = vo.twist
        out.twist.twist.linear.x = float(self.vx)
        out.twist.twist.linear.y = float(self.vy)
        out.twist.twist.linear.z = float(self.vz)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
