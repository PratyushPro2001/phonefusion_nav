#!/usr/bin/env python3
import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def mat_to_quat(R):
    # Convert rotation matrix to quaternion (x,y,z,w)
    q = np.empty(4, dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        q[3] = 0.25 * S
        q[0] = (R[2,1] - R[1,2]) / S
        q[1] = (R[0,2] - R[2,0]) / S
        q[2] = (R[1,0] - R[0,1]) / S
    else:
        if (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
            S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            q[3] = (R[2,1] - R[1,2]) / S
            q[0] = 0.25 * S
            q[1] = (R[0,1] + R[1,0]) / S
            q[2] = (R[0,2] + R[2,0]) / S
        elif R[1,1] > R[2,2]:
            S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            q[3] = (R[0,2] - R[2,0]) / S
            q[0] = (R[0,1] + R[1,0]) / S
            q[1] = 0.25 * S
            q[2] = (R[1,2] + R[2,1]) / S
        else:
            S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            q[3] = (R[1,0] - R[0,1]) / S
            q[0] = (R[0,2] + R[2,0]) / S
            q[1] = (R[1,2] + R[2,1]) / S
            q[2] = 0.25 * S
    # normalize
    q /= np.linalg.norm(q) + 1e-12
    return q


class IPCamVOLite(Node):
    """
    Phase 3B-Lite:
      - Read frames from IP Webcam (MJPEG)
      - Track features (optical flow)
      - If flow ~0 => stationary => no motion
      - If flow > threshold => estimate relative pose (recoverPose) and integrate
    Output:
      /vio/odom, /vio/path, TF: odom -> vio_base
    """
    def __init__(self):
        super().__init__('ipcam_vo_lite')

        self.declare_parameter('video_url', 'http://10.0.0.190:8080/video')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('min_features', 200)
        self.declare_parameter('max_features', 800)
        self.declare_parameter('flow_stationary_thresh_px', 0.8)   # median flow (pixels)
        self.declare_parameter('scale_per_update', 0.03)           # meters per update when moving (demo scale)
        self.declare_parameter('reset_every_n', 0)                 # 0 = never

        self.video_url = self.get_parameter('video_url').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.min_features = int(self.get_parameter('min_features').value)
        self.max_features = int(self.get_parameter('max_features').value)
        self.flow_stationary_thresh = float(self.get_parameter('flow_stationary_thresh_px').value)
        self.scale_per_update = float(self.get_parameter('scale_per_update').value)
        self.reset_every_n = int(self.get_parameter('reset_every_n').value)

        self.odom_pub = self.create_publisher(Odometry, '/vio/odom', 10)
        self.path_pub = self.create_publisher(Path, '/vio/path', 10)
        self.tf_br = TransformBroadcaster(self)

        # IMU is optional here; we subscribe so later we can use it (Phase 3B-full),
        # but VO-lite works even without it.
        self.latest_imu = None
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 50)

        # Pose state: world(odom) -> vio_base
        self.R_wb = np.eye(3, dtype=np.float64)
        self.t_wb = np.zeros((3,1), dtype=np.float64)

        self.path = Path()
        self.path.header.frame_id = 'odom'

        # OpenCV capture
        self.cap = cv2.VideoCapture(self.video_url)
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video stream: {self.video_url}')
        else:
            self.get_logger().info(f'Connected to IP Webcam: {self.video_url}')

        self.prev_gray = None
        self.prev_pts = None
        self.frame_count = 0

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.step)

    def _imu_cb(self, msg: Imu):
        self.latest_imu = msg

    def _detect_features(self, gray):
        pts = cv2.goodFeaturesToTrack(
            gray,
            maxCorners=self.max_features,
            qualityLevel=0.01,
            minDistance=7,
            blockSize=7
        )
        return pts

    def step(self):
        if not self.cap or not self.cap.isOpened():
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn('Frame read failed (IP Webcam).')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            self.prev_pts = self._detect_features(gray)
            return

        # if too few features, re-detect
        if self.prev_pts is None or len(self.prev_pts) < self.min_features:
            self.prev_pts = self._detect_features(self.prev_gray)
            if self.prev_pts is None:
                self.prev_gray = gray
                return

        next_pts, st, err = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.prev_pts, None,
            winSize=(21, 21), maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        if next_pts is None or st is None:
            self.prev_gray = gray
            self.prev_pts = self._detect_features(gray)
            return

        st = st.reshape(-1)
        p0 = self.prev_pts[st == 1]
        p1 = next_pts[st == 1]

        if len(p0) < 30:
            self.prev_gray = gray
            self.prev_pts = self._detect_features(gray)
            return

        flow = np.linalg.norm((p1 - p0).reshape(-1,2), axis=1)
        med_flow = float(np.median(flow))

        moving = med_flow > self.flow_stationary_thresh

        if moving:
            # Essential matrix (assume roughly pinhole, unknown intrinsics → use normalized with fake focal)
            h, w = gray.shape[:2]
            f = 0.9 * max(w, h)
            K = np.array([[f, 0, w/2],
                          [0, f, h/2],
                          [0, 0, 1]], dtype=np.float64)

            E, mask = cv2.findEssentialMat(p1, p0, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
            if E is not None:
                _, R, t, _ = cv2.recoverPose(E, p1, p0, K)

                # integrate pose with demo scale
                s = self.scale_per_update
                self.t_wb = self.t_wb + (self.R_wb @ (s * t))
                self.R_wb = self.R_wb @ R

        # publish odom/path (even when stationary; pose remains constant)
        self.publish_outputs()

        # reset logic (optional)
        self.frame_count += 1
        if self.reset_every_n > 0 and (self.frame_count % self.reset_every_n == 0):
            self.R_wb = np.eye(3, dtype=np.float64)
            self.t_wb = np.zeros((3,1), dtype=np.float64)
            self.path.poses = []
            self.get_logger().warn('Reset VO pose + path.')

        # advance
        self.prev_gray = gray
        self.prev_pts = p1.reshape(-1,1,2)

    def publish_outputs(self):
        now = self.get_clock().now().to_msg()

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'vio_base'

        odom.pose.pose.position.x = float(self.t_wb[0,0])
        odom.pose.pose.position.y = float(self.t_wb[1,0])
        odom.pose.pose.position.z = float(self.t_wb[2,0])

        q = mat_to_quat(self.R_wb)
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])

        self.odom_pub.publish(odom)

        # Path
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = 'odom'
        ps.pose = odom.pose.pose

        self.path.header.stamp = now
        self.path.poses.append(ps)
        self.path_pub.publish(self.path)

        # TF
        tfm = TransformStamped()
        tfm.header.stamp = now
        tfm.header.frame_id = 'odom'
        tfm.child_frame_id = 'vio_base'
        tfm.transform.translation.x = odom.pose.pose.position.x
        tfm.transform.translation.y = odom.pose.pose.position.y
        tfm.transform.translation.z = odom.pose.pose.position.z
        tfm.transform.rotation = odom.pose.pose.orientation
        self.tf_br.sendTransform(tfm)


def main():
    rclpy.init()
    node = IPCamVOLite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
