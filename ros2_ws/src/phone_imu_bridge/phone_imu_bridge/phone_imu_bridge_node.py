#!/usr/bin/env python3

import json
import socket
import math
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def _now_stamp(node: Node):
    return node.get_clock().now().to_msg()


def _get_nested(d: Dict[str, Any], keys) -> Optional[Any]:
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


def _as_float(x) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None


def quat_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class PhoneImuBridge(Node):
    def __init__(self):
        super().__init__('phone_imu_bridge')

        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 5555)
        self.declare_parameter('topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')

        self.declare_parameter('accel_scale', 1.0)
        self.declare_parameter('gyro_scale', 1.0)

        self.declare_parameter('swap_xy', False)
        self.declare_parameter('swap_xz', False)
        self.declare_parameter('swap_yz', False)
        self.declare_parameter('flip_x', False)
        self.declare_parameter('flip_y', False)
        self.declare_parameter('flip_z', False)

        self.declare_parameter('use_orientation_if_available', True)
        self.declare_parameter('orientation_from_rpy_if_available', True)

        self.bind_ip = self.get_parameter('bind_ip').value
        self.port = int(self.get_parameter('port').value)
        self.topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.accel_scale = float(self.get_parameter('accel_scale').value)
        self.gyro_scale = float(self.get_parameter('gyro_scale').value)

        self.swap_xy = bool(self.get_parameter('swap_xy').value)
        self.swap_xz = bool(self.get_parameter('swap_xz').value)
        self.swap_yz = bool(self.get_parameter('swap_yz').value)
        self.flip_x = bool(self.get_parameter('flip_x').value)
        self.flip_y = bool(self.get_parameter('flip_y').value)
        self.flip_z = bool(self.get_parameter('flip_z').value)

        self.use_orientation_if_available = bool(self.get_parameter('use_orientation_if_available').value)
        self.orientation_from_rpy_if_available = bool(self.get_parameter('orientation_from_rpy_if_available').value)

        self.pub = self.create_publisher(Imu, self.topic, 50)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.bind_ip, self.port))
        self.sock.setblocking(False)

        self.get_logger().info(f'Listening UDP on {self.bind_ip}:{self.port} -> publishing {self.topic}')
        self.create_timer(0.002, self.poll_socket)

    def _apply_axis_map(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        if self.swap_xy:
            x, y = y, x
        if self.swap_xz:
            x, z = z, x
        if self.swap_yz:
            y, z = z, y
        if self.flip_x:
            x = -x
        if self.flip_y:
            y = -y
        if self.flip_z:
            z = -z
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

    def _extract_vec3(self, d: Dict[str, Any], style_keys) -> Optional[Tuple[float, float, float]]:
        for path in style_keys:
            sub = _get_nested(d, path)
            if isinstance(sub, dict):
                x = _as_float(sub.get('x', None))
                y = _as_float(sub.get('y', None))
                z = _as_float(sub.get('z', None))
                if x is not None and y is not None and z is not None:
                    return x, y, z
        return None

    def _extract_short(self, d: Dict[str, Any], kx, ky, kz) -> Optional[Tuple[float, float, float]]:
        x = _as_float(d.get(kx, None))
        y = _as_float(d.get(ky, None))
        z = _as_float(d.get(kz, None))
        if x is None or y is None or z is None:
            return None
        return x, y, z

    def _extract_quat(self, d: Dict[str, Any]) -> Optional[Quaternion]:
        if not self.use_orientation_if_available:
            return None
        for key in ('quat', 'q', 'quaternion', 'orientation'):
            sub = d.get(key, None)
            if isinstance(sub, dict):
                qx = _as_float(sub.get('x', None))
                qy = _as_float(sub.get('y', None))
                qz = _as_float(sub.get('z', None))
                qw = _as_float(sub.get('w', None))
                if None not in (qx, qy, qz, qw):
                    q = Quaternion()
                    q.x, q.y, q.z, q.w = qx, qy, qz, qw
                    return q
        return None

    def _extract_rpy_quat(self, d: Dict[str, Any]) -> Optional[Quaternion]:
        if not self.orientation_from_rpy_if_available:
            return None

        roll = _as_float(d.get('roll', d.get('r', None)))
        pitch = _as_float(d.get('pitch', d.get('p', None)))
        yaw = _as_float(d.get('yaw', d.get('y', None)))

        if roll is None or pitch is None or yaw is None:
            rpy = d.get('rpy', None)
            if isinstance(rpy, dict):
                roll = _as_float(rpy.get('roll', None))
                pitch = _as_float(rpy.get('pitch', None))
                yaw = _as_float(rpy.get('yaw', None))

        if roll is None or pitch is None or yaw is None:
            return None

        return quat_from_euler(roll, pitch, yaw)

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

            accel = self._extract_vec3(d, [('accel',), ('linear_acceleration',), ('linacc',), ('a',)])
            if accel is None:
                accel = self._extract_short(d, 'ax', 'ay', 'az')

            gyro = self._extract_vec3(d, [('gyro',), ('angular_velocity',), ('angvel',), ('g',)])
            if gyro is None:
                gyro = self._extract_short(d, 'gx', 'gy', 'gz')

            if accel is None or gyro is None:
                continue

            ax, ay, az = accel
            gx, gy, gz = gyro

            ax *= self.accel_scale
            ay *= self.accel_scale
            az *= self.accel_scale
            gx *= self.gyro_scale
            gy *= self.gyro_scale
            gz *= self.gyro_scale

            ax, ay, az = self._apply_axis_map(ax, ay, az)
            gx, gy, gz = self._apply_axis_map(gx, gy, gz)

            imu = Imu()
            imu.header.stamp = _now_stamp(self)
            imu.header.frame_id = self.frame_id

            imu.linear_acceleration.x = float(ax)
            imu.linear_acceleration.y = float(ay)
            imu.linear_acceleration.z = float(az)

            imu.angular_velocity.x = float(gx)
            imu.angular_velocity.y = float(gy)
            imu.angular_velocity.z = float(gz)

            q = self._extract_quat(d)
            if q is None:
                q = self._extract_rpy_quat(d)
            if q is not None:
                imu.orientation = q
            else:
                imu.orientation.w = 1.0

            self.pub.publish(imu)


def main():
    rclpy.init()
    node = PhoneImuBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
