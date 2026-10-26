#!/usr/bin/env python3
"""ROS 2 wrapper for PhoneEKF.

Subscribes : /imu/data, /compass/heading, /barometer/pressure   (from sensor_logger_bridge)
Publishes  : /ekf/odom (Odometry, frame map)  /ekf/pose  /ekf/path (rolling trail)  TF map->base_link
             /ekf/status (JSON string, 1 Hz: still?, aligned?, update counts, biases, NIS)
Service    : /ekf/reset (std_srvs/Trigger) clears the trajectory and filter
"""
import json
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from phone_ekf.ekf import EkfConfig, PhoneEKF, quat_to_R, R_to_quat


class EkfNode(Node):
    def __init__(self):
        super().__init__('phone_ekf')
        cfg = EkfConfig()
        for name, default in cfg.__dict__.items():
            self.declare_parameter(name, default)
            setattr(cfg, name, self.get_parameter(name).value)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('path_hz', 10.0)
        self.declare_parameter('path_max_points', 3000)
        self.frame = self.get_parameter('frame_id').value
        self.child = self.get_parameter('child_frame_id').value
        self.path_period = 1.0 / float(self.get_parameter('path_hz').value)
        self.path_max = int(self.get_parameter('path_max_points').value)

        self.cfg = cfg
        self.ekf = PhoneEKF(cfg)
        self.t0 = None
        self.last_path_t = -1e9
        self.path = deque(maxlen=self.path_max)

        self.odom_pub = self.create_publisher(Odometry, '/ekf/odom', 20)
        self.pose_pub = self.create_publisher(PoseStamped, '/ekf/pose', 20)
        self.path_pub = self.create_publisher(Path, '/ekf/path', 5)
        self.status_pub = self.create_publisher(String, '/ekf/status', 5)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)   # direct, avoids the tf2_ros python dependency
        self.create_subscription(Imu, '/imu/data', self.on_imu, 200)
        self.create_subscription(Float64, '/compass/heading', self.on_compass, 200)
        self.create_subscription(FluidPressure, '/barometer/pressure', self.on_baro, 10)
        self.create_service(Trigger, '/ekf/reset', self.on_reset)
        self.create_timer(1.0, self.on_status)
        self.create_timer(self.path_period, self.on_path)
        self.n_imu = 0
        self.get_logger().info('phone_ekf up: waiting for /imu/data, /compass/heading, /barometer/pressure')

    # ------------------------------------------------------------------ inputs
    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        q = m.orientation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        a = (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
        w = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        self.ekf.imu(t - self.t0, R, a, w)
        self.n_imu += 1
        self.publish_state(m.header.stamp)

    def on_compass(self, m):
        self.ekf.compass(m.data)

    def on_baro(self, m):
        self.ekf.baro(m.fluid_pressure)

    def on_reset(self, req, resp):
        self.ekf.reset()
        self.t0 = None
        self.path.clear()
        resp.success, resp.message = True, 'EKF and trajectory reset (hold the phone still for ~1 s to re-align heading)'
        return resp

    # ------------------------------------------------------------------ outputs
    def publish_state(self, stamp):
        e = self.ekf
        qx, qy, qz, qw = R_to_quat(e.R_world())
        p, v = e.p, e.v
        o = Odometry()
        o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, self.frame, self.child
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = map(float, p)
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z = map(float, v)  # world frame
        for i in range(3):
            o.pose.covariance[i * 7] = float(e.P[i, i])
        self.odom_pub.publish(o)
        ps = PoseStamped()
        ps.header, ps.pose = o.header, o.pose.pose
        self.pose_pub.publish(ps)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = o.header, self.child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p)
        tf.transform.rotation = o.pose.pose.orientation
        self.tf_pub.publish(TFMessage(transforms=[tf]))
        self._last_ps = ps

    def on_path(self):
        ps = getattr(self, '_last_ps', None)
        if ps is None or self.t0 is None:
            return
        self.path.append(ps)
        m = Path()
        m.header = ps.header
        m.poses = list(self.path)
        self.path_pub.publish(m)

    def on_status(self):
        e = self.ekf
        s = {
            'imu_msgs': self.n_imu, 'still': e.still, 'compass_aligned': e.aligned,
            'psi_deg': round(float(np.degrees(e.psi)), 2),
            'pos_m': [round(float(x), 3) for x in e.p], 'speed_mps': round(float(np.linalg.norm(e.v)), 3),
            'accel_bias': [round(float(x), 4) for x in e.ba],
            'updates': e.stats, 'last_nis': {k: round(v, 2) for k, v in e.last_nis.items()},
        }
        self.status_pub.publish(String(data=json.dumps(s)))
        self.get_logger().info(
            f"pos=({e.p[0]:.2f},{e.p[1]:.2f},{e.p[2]:.2f}) m  speed={np.linalg.norm(e.v):.2f}  still={e.still} "
            f"aligned={e.aligned}  zupt={e.stats['zupt']} compass ok/rej={e.stats['compass']}/{e.stats['compass_rej']}",
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
