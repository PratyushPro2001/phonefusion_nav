#!/usr/bin/env python3
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LKFeatureTracker(Node):
    def __init__(self):
        super().__init__('feature_tracker')

        # Topics
        self.declare_parameter('in_topic', '/camera/image_raw')
        self.declare_parameter('out_topic', '/camera/features')

        # Seeding params (Shi-Tomasi)
        self.declare_parameter('max_corners', 400)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance', 8)
        self.declare_parameter('block_size', 7)

        # Reseed policy
        self.declare_parameter('reseed_min_tracked', 120)   # reseed if tracked points drop below this
        self.declare_parameter('reseed_every_n', 0)          # 0 disables periodic reseed

        # LK optical flow params
        self.declare_parameter('win_size', 21)
        self.declare_parameter('max_level', 3)
        self.declare_parameter('term_count', 30)
        self.declare_parameter('term_eps', 0.01)

        # Drawing
        self.declare_parameter('trail_len', 15)
        self.declare_parameter('publish_gray', False)

        self.in_topic = self.get_parameter('in_topic').value
        self.out_topic = self.get_parameter('out_topic').value

        self.max_corners = int(self.get_parameter('max_corners').value)
        self.quality_level = float(self.get_parameter('quality_level').value)
        self.min_distance = int(self.get_parameter('min_distance').value)
        self.block_size = int(self.get_parameter('block_size').value)

        self.reseed_min_tracked = int(self.get_parameter('reseed_min_tracked').value)
        self.reseed_every_n = int(self.get_parameter('reseed_every_n').value)

        self.win_size = int(self.get_parameter('win_size').value)
        self.max_level = int(self.get_parameter('max_level').value)
        self.term_count = int(self.get_parameter('term_count').value)
        self.term_eps = float(self.get_parameter('term_eps').value)

        self.trail_len = int(self.get_parameter('trail_len').value)
        self.publish_gray = bool(self.get_parameter('publish_gray').value)

        self.lk_params = dict(
            winSize=(self.win_size, self.win_size),
            maxLevel=self.max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, self.term_count, self.term_eps),
        )

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.out_topic, 10)
        self.sub = self.create_subscription(Image, self.in_topic, self.cb, 10)

        # State
        self.prev_gray = None
        self.prev_pts = None          # Nx1x2 float32
        self.trails = []              # list[list[(x,y)]], same order as prev_pts
        self.frame_count = 0

        self.get_logger().info(f"LK tracker subscribing: {self.in_topic}")
        self.get_logger().info(f"LK tracker publishing:  {self.out_topic}")

    def _to_gray(self, frame):
        if frame.ndim == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame

    def _reseed(self, gray):
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=self.block_size
        )
        if pts is None:
            self.prev_pts = None
            self.trails = []
            return 0

        pts = pts.astype(np.float32)
        self.prev_pts = pts
        self.trails = [[tuple(p[0])] for p in pts]
        return len(pts)

    def cb(self, msg: Image):
        try:
            # Convert ROS -> OpenCV
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                frame = self.bridge.imgmsg_to_cv2(msg)

            gray = self._to_gray(frame)
            self.frame_count += 1

            # First frame: seed
            if self.prev_gray is None or self.prev_pts is None or len(self.trails) == 0:
                reseeded = self._reseed(gray)
                self.prev_gray = gray
                out = self._draw(frame, msg.header)
                self.pub.publish(out)
                if self.frame_count % 5 == 0:
                    self.get_logger().info(f"tracked={reseeded} reseed={reseeded}")
                return

            # Track
            next_pts, status, _err = cv2.calcOpticalFlowPyrLK(
                self.prev_gray, gray, self.prev_pts, None, **self.lk_params
            )

            if next_pts is None or status is None:
                reseeded = self._reseed(gray)
                self.prev_gray = gray
                out = self._draw(frame, msg.header)
                self.pub.publish(out)
                self.get_logger().warn(f"LK failed → reseed={reseeded}")
                return

            status = status.reshape(-1)
            good_idx = np.where(status == 1)[0]
            tracked = int(good_idx.size)

            new_pts = []
            new_trails = []
            for i in good_idx:
                p = next_pts[i, 0]  # (x,y)
                new_pts.append(p)
                trail = self.trails[i]
                trail.append((float(p[0]), float(p[1])))
                if len(trail) > self.trail_len:
                    trail.pop(0)
                new_trails.append(trail)

            # Reseed policy
            need_reseed = (tracked < self.reseed_min_tracked) or (
                self.reseed_every_n > 0 and (self.frame_count % self.reseed_every_n == 0)
            )

            if need_reseed:
                reseeded = self._reseed(gray)
                tracked = reseeded
            else:
                self.prev_pts = np.array(new_pts, dtype=np.float32).reshape(-1, 1, 2)
                self.trails = new_trails
                reseeded = 0

            self.prev_gray = gray

            out = self._draw(frame, msg.header)
            self.pub.publish(out)

            if self.frame_count % 5 == 0:
                self.get_logger().info(f"tracked={tracked} reseed={reseeded}")

        except Exception as e:
            self.get_logger().warn(f"Tracker error: {e}")

    def _draw(self, frame, header):
        # Draw on BGR canvas
        if frame.ndim == 2:
            vis = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            vis = frame.copy()

        # Trails: green polylines, current point: red dot
        for trail in self.trails:
            if len(trail) >= 2:
                pts = np.array(trail, dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(vis, [pts], False, (0, 255, 0), 2)
            if len(trail) >= 1:
                x, y = trail[-1]
                cv2.circle(vis, (int(x), int(y)), 3, (0, 0, 255), -1)

        if self.publish_gray:
            mono = cv2.cvtColor(vis, cv2.COLOR_BGR2GRAY)
            out_msg = self.bridge.cv2_to_imgmsg(mono, encoding='mono8')
        else:
            out_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')

        out_msg.header = header
        return out_msg


def main():
    rclpy.init()
    node = LKFeatureTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
