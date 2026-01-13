#!/usr/bin/env python3
import math
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion


def quat_from_yaw(yaw: float) -> Quaternion:
    # yaw about +Z in map frame
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class VOEstimator(Node):
    """
    Minimal monocular VO:
      - Track points with LK
      - Estimate Essential matrix (RANSAC)
      - Recover relative pose (R,t) (scale-ambiguous)
      - Integrate a 2D pose with heavy motion gating so stationary stays stationary
    """
    def __init__(self):
        super().__init__('vo_estimator')

        # Topics
        self.declare_parameter('in_topic', '/camera/image_raw')
        self.declare_parameter('odom_topic', '/vio/odom')
        self.declare_parameter('path_topic', '/vio/path')
        self.declare_parameter('debug_topic', '/vio/debug_image')

        # Frames
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        # Camera intrinsics (defaults are rough; tune later)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        # Tracking params
        self.declare_parameter('max_corners', 500)
        self.declare_parameter('quality_level', 0.01)
        self.declare_parameter('min_distance', 8)
        self.declare_parameter('block_size', 7)

        # LK params
        self.declare_parameter('win_size', 21)
        self.declare_parameter('max_level', 3)
        self.declare_parameter('term_count', 30)
        self.declare_parameter('term_eps', 0.01)

        # Gating (THIS is what prevents fake motion)
        self.declare_parameter('min_tracked', 120)          # require enough tracks
        self.declare_parameter('min_inlier_ratio', 0.55)    # RANSAC inlier ratio
        self.declare_parameter('min_median_flow_px', 1.25)  # median pixel motion threshold
        self.declare_parameter('step_scale', 0.05)          # arbitrary unit step per accepted update

        # Output settings
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('path_stride', 1)            # add to path every N accepted steps

        self.in_topic = self.get_parameter('in_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value

        self.world_frame = self.get_parameter('world_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        self.max_corners = int(self.get_parameter('max_corners').value)
        self.quality_level = float(self.get_parameter('quality_level').value)
        self.min_distance = int(self.get_parameter('min_distance').value)
        self.block_size = int(self.get_parameter('block_size').value)

        self.win_size = int(self.get_parameter('win_size').value)
        self.max_level = int(self.get_parameter('max_level').value)
        self.term_count = int(self.get_parameter('term_count').value)
        self.term_eps = float(self.get_parameter('term_eps').value)

        self.min_tracked = int(self.get_parameter('min_tracked').value)
        self.min_inlier_ratio = float(self.get_parameter('min_inlier_ratio').value)
        self.min_median_flow_px = float(self.get_parameter('min_median_flow_px').value)
        self.step_scale = float(self.get_parameter('step_scale').value)

        self.publish_debug = bool(self.get_parameter('publish_debug').value)
        self.path_stride = int(self.get_parameter('path_stride').value)

        self.K = np.array([[self.fx, 0.0, self.cx],
                           [0.0, self.fy, self.cy],
                           [0.0, 0.0, 1.0]], dtype=np.float64)

        self.lk_params = dict(
            winSize=(self.win_size, self.win_size),
            maxLevel=self.max_level,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, self.term_count, self.term_eps),
        )

        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, self.in_topic, self.cb, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10) if self.publish_debug else None

        # State
        self.prev_gray = None
        self.prev_pts = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.accepted_steps = 0

        self.path = Path()
        self.path.header.frame_id = self.world_frame

        self.get_logger().info(f"VO subscribing: {self.in_topic}")
        self.get_logger().info(f"Publishing odom: {self.odom_topic}, path: {self.path_topic}")
        self.get_logger().warn("Monocular VO is scale-ambiguous. We gate motion to avoid fake drift.")

    def _to_gray(self, bgr_or_gray):
        if bgr_or_gray.ndim == 3:
            return cv2.cvtColor(bgr_or_gray, cv2.COLOR_BGR2GRAY)
        return bgr_or_gray

    def _seed(self, gray):
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_corners,
            qualityLevel=self.quality_level,
            minDistance=self.min_distance,
            blockSize=self.block_size
        )
        if pts is None:
            self.prev_pts = None
            return 0
        self.prev_pts = pts.astype(np.float32)
        return len(self.prev_pts)

    def _publish(self, stamp, inlier_ratio=None, med_flow=None, tracked=None, accepted=False, debug=None):
        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = float(self.pose_x)
        odom.pose.pose.position.y = float(self.pose_y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(self.pose_yaw)

        self.odom_pub.publish(odom)

        # Path
        if accepted and (self.path_stride <= 1 or (self.accepted_steps % self.path_stride == 0)):
            ps = PoseStamped()
            ps.header = odom.header
            ps.pose = odom.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(ps)
            self.path_pub.publish(self.path)

        # Debug image
        if self.publish_debug and debug is not None:
            out = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            out.header.stamp = stamp
            out.header.frame_id = self.child_frame
            self.debug_pub.publish(out)

        # Periodic log
        if tracked is not None:
            msg = f"tracked={tracked}"
            if med_flow is not None:
                msg += f" med_flow_px={med_flow:.2f}"
            if inlier_ratio is not None:
                msg += f" inlier_ratio={inlier_ratio:.2f}"
            msg += " ACCEPT" if accepted else " hold"
            self.get_logger().info(msg)

    def cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = self._to_gray(frame)

            # First frame
            if self.prev_gray is None or self.prev_pts is None:
                seeded = self._seed(gray)
                self.prev_gray = gray
                stamp = msg.header.stamp
                dbg = frame.copy()
                if self.prev_pts is not None:
                    for p in self.prev_pts.reshape(-1, 2).astype(int):
                        cv2.circle(dbg, (int(p[0]), int(p[1])), 2, (0, 255, 0), -1)
                self._publish(stamp, tracked=seeded, accepted=False, debug=dbg)
                return

            # Track with LK
            next_pts, status, _err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None, **self.lk_params)
            if next_pts is None or status is None:
                self._seed(gray)
                self.prev_gray = gray
                self._publish(msg.header.stamp, tracked=0, accepted=False, debug=frame)
                return

            status = status.reshape(-1)
            good_idx = np.where(status == 1)[0]
            tracked = int(good_idx.size)

            if tracked < max(8, self.min_tracked // 2):
                # too few, reseed and hold
                self._seed(gray)
                self.prev_gray = gray
                self._publish(msg.header.stamp, tracked=tracked, accepted=False, debug=frame)
                return

            p0 = self.prev_pts[good_idx].reshape(-1, 2)
            p1 = next_pts[good_idx].reshape(-1, 2)

            # Motion gating: median pixel flow
            flows = np.linalg.norm(p1 - p0, axis=1)
            med_flow = float(np.median(flows))

            # Essential matrix with RANSAC
            E, mask = cv2.findEssentialMat(p1, p0, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            if E is None or mask is None:
                self.prev_gray = gray
                self.prev_pts = next_pts
                self._publish(msg.header.stamp, tracked=tracked, med_flow=med_flow, accepted=False, debug=frame)
                return

            inliers = int(mask.sum())
            inlier_ratio = float(inliers) / float(max(1, tracked))

            # Decide accept vs hold (prevents fake motion)
            accept = (tracked >= self.min_tracked) and (inlier_ratio >= self.min_inlier_ratio) and (med_flow >= self.min_median_flow_px)

            dbg = frame.copy()
            # draw inliers in green, outliers in red
            m = mask.reshape(-1).astype(bool)
            for i in range(p1.shape[0]):
                x, y = int(p1[i, 0]), int(p1[i, 1])
                if m[i]:
                    cv2.circle(dbg, (x, y), 2, (0, 255, 0), -1)
                else:
                    cv2.circle(dbg, (x, y), 2, (0, 0, 255), -1)

            if accept:
                # Recover relative pose (up to scale)
                _, R, t, _ = cv2.recoverPose(E, p1, p0, self.K, mask=mask)

                # Convert R to a small yaw update around Z in map frame (approx)
                # This is intentionally conservative; main goal is "move when moved, freeze when still"
                yaw_delta = math.atan2(R[1, 0], R[0, 0])

                # Translation direction in camera coords (x right, y down, z forward)
                # Map approximate: forward-> +X, right-> +Y (simple 2D)
                tx, tz = float(t[0, 0]), float(t[2, 0])
                step_x = self.step_scale * tz
                step_y = self.step_scale * tx

                self.pose_x += step_x * math.cos(self.pose_yaw) - step_y * math.sin(self.pose_yaw)
                self.pose_y += step_x * math.sin(self.pose_yaw) + step_y * math.cos(self.pose_yaw)
                self.pose_yaw += yaw_delta

                self.accepted_steps += 1

                cv2.putText(dbg, "ACCEPT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.putText(dbg, f"med_flow={med_flow:.2f} inlier={inlier_ratio:.2f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(dbg, "HOLD", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.putText(dbg, f"med_flow={med_flow:.2f} inlier={inlier_ratio:.2f}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Update tracker state
            self.prev_gray = gray
            self.prev_pts = next_pts

            # If we’re losing points, reseed quietly (but don't force motion)
            if tracked < self.min_tracked:
                self._seed(gray)

            self._publish(msg.header.stamp, inlier_ratio=inlier_ratio, med_flow=med_flow, tracked=tracked, accepted=accept, debug=dbg)

        except Exception as e:
            self.get_logger().warn(f"VO error: {e}")


def main():
    rclpy.init()
    node = VOEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
