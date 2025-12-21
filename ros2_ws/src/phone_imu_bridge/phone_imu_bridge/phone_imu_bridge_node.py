#!/usr/bin/env python3

import json
import socket
import math
from typing import Any, Dict, Optional, Tuple, List

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import Quaternion


def _now_stamp(node: Node):
    return node.get_clock().now().to_msg()


def quat_from_android_rotation_vector(values: List[float]) -> Quaternion:
    x = float(values[0]) if len(values) > 0 else 0.0
    y = float(values[1]) if len(values) > 1 else 0.0
    z = float(values[2]) if len(values) > 2 else 0.0
    if len(values) > 3:
        w = float(values[3])
    else:
        t = 1.0 - (x*x + y*y + z*z)
        w = math.sqrt(max(0.0, t))
    q = Quaternion()
    q.x, q.y, q.z, q.w = x, y, z, w
    return q


class PhoneImuBridge(Node):
    def __init__(self):
        super().__init__('phone_imu_bridge')

        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 5555)

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('imu_frame_id', 'imu_link')

        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('gps_frame_id', 'gps_link')

        self.declare_parameter('accel_scale', 1.0)  # set 9.80665 if accel is in g
        self.declare_parameter('gyro_scale', 1.0)   # set pi/180 if gyro is deg/s

        self.declare_parameter('swap_xy', False)
        self.declare_parameter('swap_xz', False)
        self.declare_parameter('swap_yz', False)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('flip_z', False)

        self.declare_parameter('use_orientation_if_available', True)

        self.bind_ip = self.get_parameter('bind_ip').value
        self.port = int(self.get_parameter('port').value)

        self.imu_topic = self.get_parameter('imu_topic').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value

        self.gps_topic = self.get_parameter('gps_topic').value
        self.gps_frame_id = self.get_parameter('gps_frame_id').value

        self.accel_scale = float(self.get_parameter('accel_scale').value)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

        self.swap_xy = bool(self.get_parameter('swap_xy').value)
        self.swap_xz = bool(self.get_parameter('swap_xz').value)
        self.swap_yz = bool(self.get_parameter('swap_yz').value)
        self.flip_x = bool(self.get_parameter('flip_x').value)
        self.flip_y = bool(self.get_parameter('flip_y').value)
        self.flip_z = bool(self.get_parameter('flip_z').value)

        self.use_orientation_if_available = bool(self.get_parameter('use_orientation_if_available').value)

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 50)
        self.gps_pub = self.create_publisher(NavSatFix, self.gps_topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.setblocking(False)

        self.get_logger().info(f'Listening UDP on {self.bind_ip}:{self.port}')
        self.get_logger().info(f'Publishing IMU: {self.imu_topic} | GPS: {self.gps_topic}')

        self.create_timer(0.002, self.poll_socket)

        # Latest values (SensaGram streams send separate packets)
        self.last_accel: Optional[Tuple[float,float,float]] = None
        self.last_gyro: Optional[Tuple[float,float,float]] = None
        self.last_quat: Optional[Quaternion] = None

    def _apply_axis_map(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        if self.swap_xy: x, y = y, x
        if self.swap_xz: x, z = z, x
        if self.swap_yz: y, z = z, y
        if self.flip_x: x = -x
        if self.flip_y: y = -y
        if self.flip_z: z = -z
        return x, y, z

    def _parse_json(self, payload: bytes) -> Optional[Dict[str, Any]]:
        try:
            s = payload.decode('utf-8', errors='ignore').strip()
            if not s:
                return None
            if '\n' in s:
                parts = [p.strip() for p in s.splitlines() if p.strip()]
                s = parts[-1] if parts else s
            return json.loads(s)
        except Exception:
            return None

    def _publish_imu_if_ready(self):
        if self.last_accel is None or self.last_gyro is None:
            return

        ax, ay, az = self.last_accel
        gx, gy, gz = self.last_gyro

        imu = Imu()
        imu.header.stamp = _now_stamp(self)
        imu.header.frame_id = self.imu_frame_id

        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)

        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)

        if self.use_orientation_if_available and self.last_quat is not None:
            imu.orientation = self.last_quat
        else:
            imu.orientation.w = 1.0

        self.imu_pub.publish(imu)

    def _publish_gps(self, lat: float, lon: float, alt: Optional[float] = None, hacc: Optional[float] = None):
        fix = NavSatFix()
        fix.header.stamp = _now_stamp(self)
        fix.header.frame_id = self.gps_frame_id

        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = float(lat)
        fix.longitude = float(lon)
        fix.altitude = float(alt) if alt is not None else 0.0

        # If we have horizontal accuracy, populate covariance
        if hacc is not None and hacc >= 0.0:
            # simple: var_x = var_y = (hacc^2)/2 so sqrt(var_x+var_y)=hacc
            var = (float(hacc) ** 2) / 2.0
            fix.position_covariance = [0.0]*9
            fix.position_covariance[0] = var
            fix.position_covariance[4] = var
            fix.position_covariance[8] = max(1.0, var)  # altitude unknown-ish
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.gps_pub.publish(fix)

    def poll_socket(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(8192)
            except BlockingIOError:
                return
            except Exception as e:
                self.get_logger().warn(f'UDP recv error: {e}')
                return

            d = self._parse_json(data)
            if not isinstance(d, dict):
                continue

            # ---- SensaGram format: {"type": "...", "values": [...] } ----
            t = str(d.get('type', ''))
            vals = d.get('values', None)

            if isinstance(vals, list) and t:
                # IMU
                if t in ('android.sensor.linear_acceleration', 'android.sensor.accelerometer'):
                    if len(vals) >= 3:
                        ax, ay, az = float(vals[0]), float(vals[1]), float(vals[2])
                        ax *= self.accel_scale; ay *= self.accel_scale; az *= self.accel_scale
                        ax, ay, az = self._apply_axis_map(ax, ay, az)
                        self.last_accel = (ax, ay, az)
                        self._publish_imu_if_ready()
                    continue

                if t == 'android.sensor.gyroscope':
                    if len(vals) >= 3:
                        gx, gy, gz = float(vals[0]), float(vals[1]), float(vals[2])
                        gx *= self.gyro_scale; gy *= self.gyro_scale; gz *= self.gyro_scale
                        gx, gy, gz = self._apply_axis_map(gx, gy, gz)
                        self.last_gyro = (gx, gy, gz)
                        self._publish_imu_if_ready()
                    continue

                if t in ('android.sensor.game_rotation_vector', 'android.sensor.rotation_vector'):
                    if self.use_orientation_if_available and len(vals) >= 3:
                        self.last_quat = quat_from_android_rotation_vector([float(v) for v in vals])
                    continue

                # GPS (SensaGram varies; accept several likely type strings)
                if ('gps' in t.lower()) or ('location' in t.lower()):
                    # common convention: [lat, lon, alt, speed, bearing, hacc, vacc, ...]
                    if len(vals) >= 2:
                        lat = float(vals[0])
                        lon = float(vals[1])
                        alt = float(vals[2]) if len(vals) >= 3 else None
                        hacc = float(vals[5]) if len(vals) >= 6 else None
                        self._publish_gps(lat, lon, alt=alt, hacc=hacc)
                    continue

                continue

            # ---- Alternative GPS formats (keyed JSON) ----
            if 'latitude' in d and 'longitude' in d:
                lat = float(d['latitude'])
                lon = float(d['longitude'])
                alt = float(d['altitude']) if 'altitude' in d else None
                hacc = float(d.get('accuracy', -1.0)) if 'accuracy' in d else None
                self._publish_gps(lat, lon, alt=alt, hacc=hacc)
                continue

            # ignore unknown packets


def main():
    rclpy.init()
    node = PhoneImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
