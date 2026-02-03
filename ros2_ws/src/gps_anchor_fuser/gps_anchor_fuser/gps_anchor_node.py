#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


def latlon_to_local_xy_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    R = 6378137.0
    lat0r = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = R * dlon * math.cos(lat0r)
    y = R * dlat
    return x, y


class GpsAnchorFuser(Node):
    """
    GPS anchor for an existing odometry stream.

    Learns a smooth XY offset that pulls odom into GPS local frame:
      target_offset = gps_xy - odom_xy
      offset <- (1-a)*offset + a*target_offset

    Adds a motion gate so GPS jitter does NOT walk pose while stationary:
      - compute odom speed from twist
      - only update offset when speed > min_speed_for_gps_update
    """
    def __init__(self):
        super().__init__('gps_anchor_fuser')

        # Topics
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('input_odom_topic', '/vio_fused/odom')
        self.declare_parameter('imu_odom_topic', '')  # legacy fallback
        self.declare_parameter('out_odom_topic', '/vio_gps/odom')
        self.declare_parameter('out_path_topic', '/vio_gps/path')

        # Frames
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')

        # GPS pull / gating
        self.declare_parameter('gps_alpha', 0.03)
        self.declare_parameter('max_hacc_m', 15.0)
        self.declare_parameter('max_gps_step_m', 12.0)

        # Motion gate (new)
        self.declare_parameter('gate_gps_when_stationary', True)
        self.declare_parameter('min_speed_for_gps_update', 0.08)  # m/s
        self.declare_parameter('min_dt_between_gps_updates', 0.2) # seconds (avoid overreacting)

        self.declare_parameter('correct_xy_only', True)

        self.gps_fix_topic = str(self.get_parameter('gps_fix_topic').value)

        input_odom = str(self.get_parameter('input_odom_topic').value)
        legacy_imu = str(self.get_parameter('imu_odom_topic').value)
        if (not input_odom) and legacy_imu:
            input_odom = legacy_imu
        self.input_odom_topic = input_odom if input_odom else '/vio_fused/odom'

        self.out_odom_topic = str(self.get_parameter('out_odom_topic').value)
        self.out_path_topic = str(self.get_parameter('out_path_topic').value)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        self.gps_alpha = float(self.get_parameter('gps_alpha').value)
        self.max_hacc_m = float(self.get_parameter('max_hacc_m').value)
        self.max_gps_step_m = float(self.get_parameter('max_gps_step_m').value)

        self.gate_gps_when_stationary = bool(self.get_parameter('gate_gps_when_stationary').value)
        self.min_speed_for_gps_update = float(self.get_parameter('min_speed_for_gps_update').value)
        self.min_dt_between_gps_updates = float(self.get_parameter('min_dt_between_gps_updates').value)

        self.correct_xy_only = bool(self.get_parameter('correct_xy_only').value)

        # State
        self.have_origin = False
        self.lat0 = 0.0
        self.lon0 = 0.0
        self.prev_gps_xy: Optional[Tuple[float, float]] = None

        self.latest_odom: Optional[Odometry] = None

        self.off_x = 0.0
        self.off_y = 0.0

        self.last_gps_update_time: Optional[float] = None  # seconds (node clock)

        self.create_subscription(Odometry, self.input_odom_topic, self.on_odom, 50)
        self.create_subscription(NavSatFix, self.gps_fix_topic, self.on_gps, 20)

        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 10)

        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.get_logger().info(
            f'GPS anchor fuser. odom={self.input_odom_topic} gps={self.gps_fix_topic} -> {self.out_odom_topic}'
        )

    def _hacc_m(self, fix: NavSatFix) -> Optional[float]:
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        var_x = float(fix.position_covariance[0])
        var_y = float(fix.position_covariance[4])
        if not math.isfinite(var_x) or not math.isfinite(var_y) or var_x < 0.0 or var_y < 0.0:
            return None
        return math.sqrt(max(var_x, var_y))

    def _odom_speed(self) -> Optional[float]:
        if self.latest_odom is None:
            return None
        vx = float(self.latest_odom.twist.twist.linear.x)
        vy = float(self.latest_odom.twist.twist.linear.y)
        vz = float(self.latest_odom.twist.twist.linear.z)
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(vz)):
            return None
        return math.sqrt(vx*vx + vy*vy + vz*vz)

    def on_gps(self, fix: NavSatFix):
        if not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude):
            return

        hacc = self._hacc_m(fix)
        if hacc is not None and hacc > self.max_hacc_m:
            return

        if not self.have_origin:
            self.lat0 = float(fix.latitude)
            self.lon0 = float(fix.longitude)
            self.have_origin = True
            self.get_logger().info(f'GPS origin set lat0={self.lat0:.8f} lon0={self.lon0:.8f}')
            return

        if self.latest_odom is None:
            return

        # Motion gate: don’t chase GPS jitter when stationary
        if self.gate_gps_when_stationary:
            spd = self._odom_speed()
            if spd is not None and spd < self.min_speed_for_gps_update:
                return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_gps_update_time is not None:
            if (now - self.last_gps_update_time) < self.min_dt_between_gps_updates:
                return

        gx, gy = latlon_to_local_xy_m(self.lat0, self.lon0, float(fix.latitude), float(fix.longitude))

        # Reject GPS teleport
        if self.prev_gps_xy is not None:
            dx = gx - self.prev_gps_xy[0]
            dy = gy - self.prev_gps_xy[1]
            step = math.hypot(dx, dy)
            if step > self.max_gps_step_m:
                return
        self.prev_gps_xy = (gx, gy)

        ox = float(self.latest_odom.pose.pose.position.x)
        oy = float(self.latest_odom.pose.pose.position.y)

        target_off_x = gx - ox
        target_off_y = gy - oy

        a = self.gps_alpha
        self.off_x = (1.0 - a) * self.off_x + a * target_off_x
        self.off_y = (1.0 - a) * self.off_y + a * target_off_y

        self.last_gps_update_time = now

    def on_odom(self, odom: Odometry):
        self.latest_odom = odom

        fused = Odometry()
        fused.header = odom.header
        fused.header.frame_id = self.frame_id
        fused.child_frame_id = self.child_frame_id

        fused.pose = odom.pose
        fused.twist = odom.twist

        fused.pose.pose.position.x = float(odom.pose.pose.position.x) + self.off_x
        fused.pose.pose.position.y = float(odom.pose.pose.position.y) + self.off_y

        if self.correct_xy_only:
            fused.pose.pose.position.z = float(odom.pose.pose.position.z)

        self.odom_pub.publish(fused)

        ps = PoseStamped()
        ps.header = fused.header
        ps.pose = fused.pose.pose

        self.path.header.stamp = fused.header.stamp
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    node = GpsAnchorFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
