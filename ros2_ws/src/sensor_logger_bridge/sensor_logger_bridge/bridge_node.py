#!/usr/bin/env python3
"""Sensor Logger HTTP push -> ROS 2 topics.

Publishes
  /imu/data                 sensor_msgs/Imu          100 Hz  gyro + linear accel (gravity removed) + iOS orientation
  /gps/fix                  sensor_msgs/NavSatFix    ~1 Hz   covariance from reported horizontal/vertical accuracy
  /mag/uncalibrated         sensor_msgs/MagneticField        Tesla, raw (hard-iron offset included)
  /barometer/pressure       sensor_msgs/FluidPressure        Pa
  /barometer/relative_altitude std_msgs/Float64              m
  /compass/heading          std_msgs/Float64                 deg, magnetic bearing (clockwise from north), published right after its IMU tick

Phone axes are kept as delivered (iOS device frame: x right, y up, z out of screen); frame_id 'imu_link'.
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, MagneticField, FluidPressure
from std_msgs.msg import Float64

from sensor_logger_bridge.http_ingest import IngestServer
from sensor_logger_bridge.parser import (ClockOffset, ImuAssembler, iter_samples,
                                         parse_body, quat_normalized)


def _stamp(ns):
    return {'sec': int(ns // 1_000_000_000), 'nanosec': int(ns % 1_000_000_000)}


class Bridge(Node):
    def __init__(self):
        super().__init__('sensor_logger_bridge')
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('auth_token', '')          # must equal the app's "Auth Header" if set
        self.declare_parameter('record_path', '')         # append raw POST bodies here (.jsonl)
        self.declare_parameter('time_mode', 'offset')     # 'offset' | 'phone' | 'arrival'
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('gps_frame_id', 'gps_link')
        # noise from static analysis (phonedata/.../static_summary.csv)
        self.declare_parameter('gyro_std', 0.0007)        # rad/s
        self.declare_parameter('accel_std', 0.008)        # m/s^2
        self.declare_parameter('orient_std', 0.02)        # rad (placeholder; not measured yet)

        g = lambda n: self.get_parameter(n).value
        self.time_mode = g('time_mode')
        self.imu_frame, self.gps_frame = g('imu_frame_id'), g('gps_frame_id')
        self.gyro_var, self.accel_var, self.orient_var = g('gyro_std') ** 2, g('accel_std') ** 2, g('orient_std') ** 2

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 200)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/mag/uncalibrated', 50)
        self.baro_pub = self.create_publisher(FluidPressure, '/barometer/pressure', 10)
        self.alt_pub = self.create_publisher(Float64, '/barometer/relative_altitude', 10)
        self.compass_pub = self.create_publisher(Float64, '/compass/heading', 200)

        self.assembler, self.clock = ImuAssembler(), ClockOffset()
        self.server = IngestServer(g('bind_ip'), g('port'), g('auth_token'), g('record_path')).start()
        self.counts = {'imu': 0, 'gps': 0, 'mag': 0, 'baro': 0, 'compass': 0, 'bad': 0}
        self._last_report = time.time()
        self.create_timer(0.02, self.drain)
        self.get_logger().info(
            f"listening on {g('bind_ip')}:{self.server.port} (set the app's Push URL to http://<this-ip>:{self.server.port}/data)")

    def _t(self, phone_ns, recv_ns):
        if self.time_mode == 'phone':
            ns = phone_ns
        elif self.time_mode == 'arrival':
            ns = recv_ns
        else:
            ns = self.clock.to_pc_ns(phone_ns)
        return _stamp(ns)

    def drain(self):
        while not self.server.queue.empty():
            recv_ns, body = self.server.queue.get_nowait()
            try:
                msg = parse_body(body)
                samples = list(iter_samples(msg))
            except Exception as e:  # noqa: BLE001
                self.counts['bad'] += 1
                self.get_logger().warn(f'bad payload: {e}', throttle_duration_sec=5.0)
                continue
            if not samples:
                continue
            self.clock.update(recv_ns, max(t for _, t, _ in samples))
            compass = {t: v for n, t, v in samples if n == 'compass' and 'magneticBearing' in v}
            for imu in self.assembler.feed(samples):
                self.pub_imu(imu, recv_ns)
                if imu['t_ns'] in compass:     # keep heading ordered with the IMU tick it belongs to
                    self.pub_compass(compass.pop(imu['t_ns']))
            for v in compass.values():
                self.pub_compass(v)
            for name, t, v in samples:
                if name == 'location':
                    self.pub_gps(t, v, recv_ns)
                elif name == 'magnetometeruncalibrated':
                    self.pub_mag(t, v, recv_ns)
                elif name == 'barometer':
                    self.pub_baro(t, v, recv_ns)
        if time.time() - self._last_report > 5.0:
            self._last_report = time.time()
            self.get_logger().info(
                f"published {self.counts}  clock_offset={self.clock.offset_ns / 1e6:.1f} ms  "
                f"queue_overflow={self.server.overflow} dropped_ticks={self.assembler.dropped}")

    def pub_imu(self, d, recv_ns):
        m = Imu()
        s = self._t(d['t_ns'], recv_ns)
        m.header.stamp.sec, m.header.stamp.nanosec = s['sec'], s['nanosec']
        m.header.frame_id = self.imu_frame
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(d['gyro'][k]) for k in 'xyz')
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(d['accel'][k]) for k in 'xyz')
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quat_normalized(d['orient'])
        for cov, var in ((m.angular_velocity_covariance, self.gyro_var),
                         (m.linear_acceleration_covariance, self.accel_var),
                         (m.orientation_covariance, self.orient_var)):
            cov[0] = cov[4] = cov[8] = var
        self.imu_pub.publish(m)
        self.counts['imu'] += 1

    def pub_gps(self, t, v, recv_ns):
        hacc = float(v.get('horizontalAccuracy', -1))
        if hacc < 0:
            return
        m = NavSatFix()
        s = self._t(t, recv_ns)
        m.header.stamp.sec, m.header.stamp.nanosec = s['sec'], s['nanosec']
        m.header.frame_id = self.gps_frame
        m.status.status = NavSatStatus.STATUS_FIX
        m.status.service = NavSatStatus.SERVICE_GPS
        m.latitude, m.longitude = float(v['latitude']), float(v['longitude'])
        m.altitude = float(v.get('altitude', 0.0))   # ellipsoidal height
        vacc = float(v.get('verticalAccuracy', hacc))
        m.position_covariance[0] = m.position_covariance[4] = hacc ** 2
        m.position_covariance[8] = max(vacc, 0.0) ** 2
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.gps_pub.publish(m)
        self.counts['gps'] += 1

    def pub_compass(self, v):
        m = Float64()
        m.data = float(v['magneticBearing'])
        self.compass_pub.publish(m)
        self.counts['compass'] += 1

    def pub_mag(self, t, v, recv_ns):
        m = MagneticField()
        s = self._t(t, recv_ns)
        m.header.stamp.sec, m.header.stamp.nanosec = s['sec'], s['nanosec']
        m.header.frame_id = self.imu_frame
        m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z = (float(v[k]) * 1e-6 for k in 'xyz')
        self.mag_pub.publish(m)
        self.counts['mag'] += 1

    def pub_baro(self, t, v, recv_ns):
        m = FluidPressure()
        s = self._t(t, recv_ns)
        m.header.stamp.sec, m.header.stamp.nanosec = s['sec'], s['nanosec']
        m.header.frame_id = self.imu_frame
        m.fluid_pressure = float(v['pressure']) * 100.0   # hPa -> Pa
        self.baro_pub.publish(m)
        a = Float64()
        a.data = float(v.get('relativeAltitude', 0.0))
        self.alt_pub.publish(a)
        self.counts['baro'] += 1


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
