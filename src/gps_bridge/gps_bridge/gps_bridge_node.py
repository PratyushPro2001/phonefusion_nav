#!/usr/bin/env python3
import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import PoseStamped


class GpsBridge(Node):
    """
    Simple GPS bridge:

    - Listens on a UDP port for JSON lines like:
        {"lat": 36.886, "lon": -76.305, "alt": 5.2, "fix": 1}
    - Publishes:
        /gps/fix  (sensor_msgs/NavSatFix)
        /gps/pose (geometry_msgs/PoseStamped, local XY in meters)
    """

    def __init__(self):
        super().__init__('gps_bridge')

        # Parameters
        self.declare_parameter('udp_port', 6000)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 10.0)

        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # UDP socket (non-blocking)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.udp_port))
        self.sock.setblocking(False)

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/gps/pose', 10)

        # State for ENU conversion
        self.have_origin = False
        self.lat0 = None
        self.lon0 = None

        # Buffer for latest sample
        self.latest_sample = None

        # Timer to poll socket + publish
        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'GPS bridge listening on UDP port {self.udp_port}, '
            f'publishing /gps/fix and /gps/pose (frame_id={self.frame_id})'
        )

    def timer_callback(self):
        # Try to read all pending UDP packets (non-blocking)
        while True:
            try:
                data, addr = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().warn(f'Error receiving UDP data: {e}')
                break

            try:
                text = data.decode('utf-8').strip()
                if not text:
                    continue
                # Allow either single JSON object or one per line
                # If multiple lines, handle last non-empty
                if '\n' in text:
                    lines = [l for l in text.splitlines() if l.strip()]
                    text = lines[-1]
                sample = json.loads(text)
                self.latest_sample = sample
            except Exception as e:
                self.get_logger().warn(f'Failed to parse GPS JSON: {e}')
                continue

        if self.latest_sample is None:
            return

        # Parse fields
        try:
            lat = float(self.latest_sample.get('lat'))
            lon = float(self.latest_sample.get('lon'))
            alt = float(self.latest_sample.get('alt', 0.0))
            fix = int(self.latest_sample.get('fix', 1))
        except Exception as e:
            self.get_logger().warn(f'Invalid GPS sample fields: {e}')
            return

        # Initialize origin for ENU
        if not self.have_origin:
            self.lat0 = math.radians(lat)
            self.lon0 = math.radians(lon)
            self.have_origin = True
            self.get_logger().info(f'Set GPS origin at lat={lat:.6f}, lon={lon:.6f}')

        # Publish NavSatFix
        now = self.get_clock().now().to_msg()
        fix_msg = NavSatFix()
        fix_msg.header.stamp = now
        fix_msg.header.frame_id = self.frame_id
        fix_msg.status.status = NavSatStatus.STATUS_FIX if fix > 0 else NavSatStatus.STATUS_NO_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = alt
        # Covariances unknown for now; leave default
        self.fix_pub.publish(fix_msg)

        # Convert lat/lon to local XY (equirectangular approximation)
        # Reference: https://en.wikipedia.org/wiki/Equirectangular_projection
        if self.have_origin:
            R = 6378137.0  # Earth radius [m]
            lat_rad = math.radians(lat)
            lon_rad = math.radians(lon)
            dlon = lon_rad - self.lon0
            dlat = lat_rad - self.lat0
            x = R * dlon * math.cos(self.lat0)
            y = R * dlat
        else:
            x, y = 0.0, 0.0

        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = alt
        # Orientation unknown here; identity
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS bridge shutting down (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
