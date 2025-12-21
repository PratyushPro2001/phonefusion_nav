#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


def latlon_to_local_xy_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    # local tangent approximation (good for small areas)
    R = 6378137.0
    lat0r = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = R * dlon * math.cos(lat0r)
    y = R * dlat
    return x, y


class GpsAnchorFuser(Node):
    def __init__(self):
        super().__init__('gps_anchor_fuser')

        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('imu_odom_topic', '/imu/odom')
        self.declare_parameter('out_odom_topic', '/fused/odom')
        self.declare_parameter('out_path_topic', '/fused/path')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # Pull strength (0..1). 0.02 = gentle; 0.1 = stronger.
        self.declare_parameter('gps_alpha', 0.03)

        # Gate updates if GPS too noisy
        self.declare_parameter('max_hacc_m', 15.0)

        # Reject GPS jumps
        self.declare_parameter('max_gps_step_m', 12.0)

        self.gps_fix_topic = self.get_parameter('gps_fix_topic').value
        self.imu_odom_topic = self.get_parameter('imu_odom_topic').value
        self.out_odom_topic = self.get_parameter('out_odom_topic').value
        self.out_path_topic = self.get_parameter('out_path_topic').value

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.gps_alpha = float(self.get_parameter('gps_alpha').value)
        self.max_hacc_m = float(self.get_parameter('max_hacc_m').value)
        self.max_gps_step_m = float(self.get_parameter('max_gps_step_m').value)

        self.have_origin = False
        self.lat0 = 0.0
        self.lon0 = 0.0

        self.latest_imu: Optional[Odometry] = None
        self.prev_gps_xy: Optional[Tuple[float, float]] = None

        # offset added to imu position
        self.off_x = 0.0
        self.off_y = 0.0

        self.create_subscription(Odometry, self.imu_odom_topic, self.on_imu, 50)
        self.create_subscription(NavSatFix, self.gps_fix_topic, self.on_gps, 20)

        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 10)

        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.get_logger().info(f'GPS anchor fuser running. imu={self.imu_odom_topic} gps={self.gps_fix_topic}')

    def _hacc(self, fix: NavSatFix) -> Optional[float]:
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        var_x = fix.position_covariance[0]
        var_y = fix.position_covariance[4]
        if var_x < 0.0 or var_y < 0.0:
            return None
        return math.sqrt(max(0.0, var_x + var_y))

    def on_gps(self, fix: NavSatFix):
        if not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude):
            return

        hacc = self._hacc(fix)
        if hacc is not None and hacc > self.max_hacc_m:
            return

        if not self.have_origin:
            self.lat0 = float(fix.latitude)
            self.lon0 = float(fix.longitude)
            self.have_origin = True
            self.get_logger().info(f'GPS origin set lat0={self.lat0:.8f} lon0={self.lon0:.8f}')
            return

        gx, gy = latlon_to_local_xy_m(self.lat0, self.lon0, float(fix.latitude), float(fix.longitude))

        # reject GPS teleport
        if self.prev_gps_xy is not None:
            dx = gx - self.prev_gps_xy[0]
            dy = gy - self.prev_gps_xy[1]
            step = math.hypot(dx, dy)
            if step > self.max_gps_step_m:
                return
        self.prev_gps_xy = (gx, gy)

        if self.latest_imu is None:
            return

        imu_x = float(self.latest_imu.pose.pose.position.x)
        imu_y = float(self.latest_imu.pose.pose.position.y)

        target_off_x = gx - imu_x
        target_off_y = gy - imu_y

        a = self.gps_alpha
        self.off_x = (1.0 - a) * self.off_x + a * target_off_x
        self.off_y = (1.0 - a) * self.off_y + a * target_off_y

    def on_imu(self, odom: Odometry):
        self.latest_imu = odom

        fused = Odometry()
        fused.header = odom.header
        fused.header.frame_id = self.frame_id
        fused.child_frame_id = self.child_frame_id

        fused.pose = odom.pose
        fused.twist = odom.twist

        fused.pose.pose.position.x = float(odom.pose.pose.position.x) + self.off_x
        fused.pose.pose.position.y = float(odom.pose.pose.position.y) + self.off_y

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
