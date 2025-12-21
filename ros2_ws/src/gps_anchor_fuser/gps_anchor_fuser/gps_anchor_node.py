#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped


def latlon_to_local_xy_m(lat0: float, lon0: float, lat: float, lon: float) -> Tuple[float, float]:
    """
    Simple local tangent plane approximation (equirectangular).
    Good for small areas (campus / neighborhood scale).
    """
    R = 6378137.0  # meters (WGS84 approx)
    lat0r = math.radians(lat0)
    latr = math.radians(lat)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = R * dlon * math.cos(lat0r)
    y = R * dlat
    return x, y


class GpsAnchorFuser(Node):
    def __init__(self):
        super().__init__('gps_anchor_fuser')

        # ---- params ----
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('imu_odom_topic', '/imu/odom')
        self.declare_parameter('out_odom_topic', '/fused/odom')
        self.declare_parameter('out_path_topic', '/fused/path')

        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # how strongly GPS pulls the IMU back (0.0 = never, 1.0 = snap hard)
        self.declare_parameter('gps_alpha', 0.02)

        # ignore GPS when accuracy is poor (meters). If your NavSatFix doesn't fill this, set negative to disable.
        self.declare_parameter('max_hacc_m', 20.0)

        # limit sudden GPS jumps (meters) to avoid teleporting when GPS glitches
        self.declare_parameter('max_gps_step_m', 10.0)

        self.gps_fix_topic = self.get_parameter('gps_fix_topic').value
        self.imu_odom_topic = self.get_parameter('imu_odom_topic').value
        self.out_odom_topic = self.get_parameter('out_odom_topic').value
        self.out_path_topic = self.get_parameter('out_path_topic').value

        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.gps_alpha = float(self.get_parameter('gps_alpha').value)
        self.max_hacc_m = float(self.get_parameter('max_hacc_m').value)
        self.max_gps_step_m = float(self.get_parameter('max_gps_step_m').value)

        # ---- state ----
        self.have_origin = False
        self.lat0 = 0.0
        self.lon0 = 0.0

        self.latest_imu_odom: Optional[Odometry] = None

        # GPS local position (m)
        self.gps_x: Optional[float] = None
        self.gps_y: Optional[float] = None
        self.prev_gps_x: Optional[float] = None
        self.prev_gps_y: Optional[float] = None

        # Offset that we add to IMU odom: fused = imu + offset
        self.off_x = 0.0
        self.off_y = 0.0

        # ---- pubs/subs ----
        self.create_subscription(Odometry, self.imu_odom_topic, self.on_imu_odom, 20)
        self.create_subscription(NavSatFix, self.gps_fix_topic, self.on_gps_fix, 20)

        self.odom_pub = self.create_publisher(Odometry, self.out_odom_topic, 20)
        self.path_pub = self.create_publisher(Path, self.out_path_topic, 10)

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.get_logger().info(
            f'GPS Anchor Fuser: imu={self.imu_odom_topic}, gps={self.gps_fix_topic} -> {self.out_odom_topic}, {self.out_path_topic}'
        )

    def gps_ok(self, msg: NavSatFix) -> bool:
        if msg.status.status < 0:
            return False

        if self.max_hacc_m < 0.0:
            return True

        # If covariance is known, use it
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            # horizontal accuracy approx sqrt(var_x + var_y)
            var_x = msg.position_covariance[0]
            var_y = msg.position_covariance[4]
            if var_x >= 0.0 and var_y >= 0.0:
                hacc = math.sqrt(max(0.0, var_x + var_y))
                return hacc <= self.max_hacc_m

        return True  # if unknown, let it through

    def on_gps_fix(self, msg: NavSatFix):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if not self.gps_ok(msg):
            return

        if not self.have_origin:
            self.lat0 = float(msg.latitude)
            self.lon0 = float(msg.longitude)
            self.have_origin = True
            self.get_logger().info(f'GPS origin set: lat0={self.lat0}, lon0={self.lon0}')
            return

        x, y = latlon_to_local_xy_m(self.lat0, self.lon0, float(msg.latitude), float(msg.longitude))

        # clamp GPS teleport/glitch
        if self.prev_gps_x is not None and self.prev_gps_y is not None:
            dx = x - self.prev_gps_x
            dy = y - self.prev_gps_y
            step = math.sqrt(dx*dx + dy*dy)
            if step > self.max_gps_step_m:
                # ignore this GPS update
                return

        self.prev_gps_x, self.prev_gps_y = x, y
        self.gps_x, self.gps_y = x, y

        # update offset if we also have IMU odom
        if self.latest_imu_odom is None:
            return

        imu_x = float(self.latest_imu_odom.pose.pose.position.x)
        imu_y = float(self.latest_imu_odom.pose.pose.position.y)

        target_off_x = x - imu_x
        target_off_y = y - imu_y

        a = self.gps_alpha
        self.off_x = (1.0 - a) * self.off_x + a * target_off_x
        self.off_y = (1.0 - a) * self.off_y + a * target_off_y

    def on_imu_odom(self, msg: Odometry):
        self.latest_imu_odom = msg

        # If no GPS origin yet, just pass-through IMU odom (still publish)
        fused = Odometry()
        fused.header = msg.header
        fused.header.frame_id = self.frame_id
        fused.child_frame_id = self.child_frame_id

        fused.pose = msg.pose
        fused.twist = msg.twist

        fused.pose.pose.position.x = float(msg.pose.pose.position.x) + self.off_x
        fused.pose.pose.position.y = float(msg.pose.pose.position.y) + self.off_y

        self.odom_pub.publish(fused)

        # Path
        ps = PoseStamped()
        ps.header = fused.header
        ps.pose = fused.pose.pose

        self.path_msg.header.stamp = fused.header.stamp
        self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)


def main():
    rclpy.init()
    node = GpsAnchorFuser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
