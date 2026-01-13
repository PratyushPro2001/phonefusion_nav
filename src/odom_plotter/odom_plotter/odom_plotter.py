#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        # Parameters
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('arrow_length', 0.5)

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.arrow_length = self.get_parameter('arrow_length').get_parameter_value().double_value

        # Last pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.arrow_line,) = self.ax.plot([0, 1], [0, 0], linewidth=3)

        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('PhoneFusion-Nav: Heading Arrow (Yaw Only)')
        self.ax.grid(True)
        self.ax.axis('equal')
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)

        # Timer to refresh plot
        self.timer = self.create_timer(0.05, self.update_plot)

        self.get_logger().info(f'Subscribing to odom topic: {odom_topic}')
        self.get_logger().info('Heading arrow plotter started.')

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.have_odom = True

    def update_plot(self):
        if not self.have_odom:
            return

        x0, y0 = self.x, self.y
        x1 = x0 + self.arrow_length * math.cos(self.yaw)
        y1 = y0 + self.arrow_length * math.sin(self.yaw)

        self.arrow_line.set_xdata([x0, x1])
        self.arrow_line.set_ydata([y0, y1])

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPlotter()

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
        except:
            pass


if __name__ == '__main__':
    main()
