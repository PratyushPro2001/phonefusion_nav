#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def quat_multiply(q1, q2):
    """Quaternion multiply: q = q1 * q2, both as (w, x, y, z)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return (w, x, y, z)


def quat_normalize(q):
    w, x, y, z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/n, x/n, y/n, z/n)


def quat_to_rot_matrix(q):
    """Quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    w, x, y, z = q
    ww, xx, yy, zz = w*w, x*x, y*y, z*z
    wx, wy, wz = w*x, w*y, w*z
    xy, xz, yz = x*y, x*z, y*z

    r00 = ww + xx - yy - zz
    r01 = 2*(xy - wz)
    r02 = 2*(xz + wy)

    r10 = 2*(xy + wz)
    r11 = ww - xx + yy - zz
    r12 = 2*(yz - wx)

    r20 = 2*(xz - wy)
    r21 = 2*(yz + wx)
    r22 = ww - xx - yy + zz

    return [
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ]


class ImuOrientationViz(Node):
    """
    Subscribe to /imu/data and visualize phone orientation as 3D RGB axes.

    - Red   = body X axis
    - Green = body Y axis
    - Blue  = body Z axis

    This uses simple gyro integration (no gravity / mag correction),
    so orientation will drift slowly, but looks good over short intervals.
    """

    def __init__(self):
        super().__init__('imu_orientation_viz')

        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('update_rate_hz', 50.0)

        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.update_rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value

        self.sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            50
        )

        # Current orientation as quaternion (w, x, y, z)
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.last_stamp = None
        self.have_imu = False

        # Matplotlib 3D setup
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title('PhoneFusion-Nav: IMU 3D Orientation Axes')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        # Keep axes roughly unit-size
        self.ax.set_xlim([-1.2, 1.2])
        self.ax.set_ylim([-1.2, 1.2])
        self.ax.set_zlim([-1.2, 1.2])

        # Lines for X, Y, Z axes (origin to rotated unit vectors)
        (self.line_x,) = self.ax.plot([0, 1], [0, 0], [0, 0])
        (self.line_y,) = self.ax.plot([0, 0], [0, 1], [0, 0])
        (self.line_z,) = self.ax.plot([0, 0], [0, 0], [0, 1])

        # Timer to refresh plot (use create_timer, not create_wall_timer)
        period = 1.0 / self.update_rate_hz
        self.timer = self.create_timer(
            period,
            self.update_plot
        )

        self.get_logger().info(f'IMU orientation viz subscribed to: {imu_topic}')

    def imu_callback(self, msg: Imu):
        """
        Integrate angular velocity to update quaternion orientation.
        """
        # Extract angular velocity in rad/s
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Time step: use message stamps if available
        if self.last_stamp is None:
            self.last_stamp = msg.header.stamp
            self.have_imu = True
            return

        dt = (msg.header.stamp.sec - self.last_stamp.sec) + \
             1e-9 * (msg.header.stamp.nanosec - self.last_stamp.nanosec)
        self.last_stamp = msg.header.stamp

        if dt <= 0.0 or dt > 0.5:
            # Ignore crazy gaps
            return

        # Small-angle quaternion from angular velocity: dq = [1, 0.5*w*dt]
        half_dt = 0.5 * dt
        dq = (
            1.0,
            wx * half_dt,
            wy * half_dt,
            wz * half_dt
        )

        # q <- q * dq
        self.q = quat_normalize(quat_multiply(self.q, dq))

    def update_plot(self):
        if not self.have_imu:
            return

        R = quat_to_rot_matrix(self.q)

        # Basis vectors in world frame: R * e_i
        ex = [R[0][0], R[1][0], R[2][0]]  # body X
        ey = [R[0][1], R[1][1], R[2][1]]  # body Y
        ez = [R[0][2], R[1][2], R[2][2]]  # body Z

        # Origin
        ox, oy, oz = 0.0, 0.0, 0.0

        # Update line data
        self.line_x.set_xdata([ox, ex[0]])
        self.line_x.set_ydata([oy, ex[1]])
        self.line_x.set_3d_properties([oz, ex[2]])

        self.line_y.set_xdata([ox, ey[0]])
        self.line_y.set_ydata([oy, ey[1]])
        self.line_y.set_3d_properties([oz, ey[2]])

        self.line_z.set_xdata([ox, ez[0]])
        self.line_z.set_ydata([oy, ez[1]])
        self.line_z.set_3d_properties([oz, ez[2]])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            plt.ioff()
            plt.close('all')
        except Exception:
            pass


if __name__ == '__main__':
    main()
