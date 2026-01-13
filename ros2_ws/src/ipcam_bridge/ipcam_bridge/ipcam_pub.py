#!/usr/bin/env python3
import time
import cv2
import numpy as np
import requests

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class IPCamPublisher(Node):
    def __init__(self):
        super().__init__('ipcam_pub')

        self.declare_parameter('video_url', 'http://127.0.0.1:4747/video')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('connect_timeout_s', 5.0)
        self.declare_parameter('reconnect_backoff_s', 0.5)

        self.url = self.get_parameter('video_url').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_hz = float(self.get_parameter('publish_hz').value)
        self.connect_timeout_s = float(self.get_parameter('connect_timeout_s').value)
        self.reconnect_backoff_s = float(self.get_parameter('reconnect_backoff_s').value)

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.get_logger().info(f"Streaming MJPEG from: {self.url}")

        self._session = requests.Session()
        self._headers = {
            "User-Agent": "ROS2-IPCamBridge",
            "Accept": "multipart/x-mixed-replace,image/jpeg,*/*",
            "Connection": "keep-alive",
            "Cache-Control": "no-cache",
            "Pragma": "no-cache",
        }

        self._resp = None
        self._iter = None
        self._buf = bytearray()

        self._last_frame_t = 0.0
        self._last_reconnect_t = 0.0

        period = 1.0 / max(self.publish_hz, 1e-6)
        self.timer = self.create_timer(period, self._tick)

    def _close(self):
        try:
            if self._resp is not None:
                self._resp.close()
        except Exception:
            pass
        self._resp = None
        self._iter = None
        self._buf.clear()

    def _connect(self):
        self._close()
        self.get_logger().warn("Connecting to MJPEG stream...")

        # connect timeout only (read can be infinite for streaming)
        self._resp = self._session.get(
            self.url,
            stream=True,
            headers=self._headers,
            timeout=(self.connect_timeout_s, None),
        )
        self._resp.raise_for_status()
        self._iter = self._resp.iter_content(chunk_size=16384)
        self.get_logger().info("MJPEG stream connected")

    def _extract_jpeg(self):
        start = self._buf.find(b'\xff\xd8')
        if start == -1:
            return None
        end = self._buf.find(b'\xff\xd9', start)
        if end == -1:
            return None
        jpg = bytes(self._buf[start:end + 2])
        del self._buf[:end + 2]
        return jpg

    def _tick(self):
        if self._iter is None:
            try:
                self._connect()
                self._last_reconnect_t = time.time()
            except Exception as e:
                self.get_logger().error(f"Connect failed: {e}")
                time.sleep(self.reconnect_backoff_s)
                return

        try:
            # pull a few chunks each tick; decode at most 1 frame per tick
            for _ in range(4):
                chunk = next(self._iter)  # may raise StopIteration / requests errors
                if not chunk:
                    continue
                self._buf.extend(chunk)

                jpg = self._extract_jpeg()
                if jpg is None:
                    continue

                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue

                msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)

                self._last_frame_t = time.time()
                return

            # If no frame for a while, reconnect (but be chill)
            if self._last_frame_t and (time.time() - self._last_frame_t > 5.0):
                raise RuntimeError("No decoded frames for >5s")

        except Exception as e:
            now = time.time()
            if now - self._last_reconnect_t < self.reconnect_backoff_s:
                return
            self._last_reconnect_t = now
            self.get_logger().warn(f"Stream error: {e} — reconnecting")
            self._close()
            time.sleep(self.reconnect_backoff_s)


def main():
    rclpy.init()
    node = IPCamPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
