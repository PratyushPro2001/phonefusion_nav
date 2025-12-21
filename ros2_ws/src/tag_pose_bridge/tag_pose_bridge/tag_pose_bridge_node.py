#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped


class TagPoseBridge(Node):
    """
    Bridge AprilTagDetectionArray (/detections) -> PoseStamped (/tag_pose)

    NOTE: Your apriltag_msgs/AprilTagDetection message does NOT include 3D pose.
    It only provides pixel-space detections: centre, corners, homography.
    So we publish a simple "image-frame pose":
      position.x = centre.x
      position.y = centre.y
      position.z = 0
      orientation = identity (w=1)
    """

    def __init__(self):
        super().__init__('tag_pose_bridge')

        self.declare_parameter('detections_topic', '/detections')
        det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value

        self.pub = self.create_publisher(PoseStamped, '/tag_pose', 10)
        self.sub = self.create_subscription(AprilTagDetectionArray, det_topic, self.detections_callback, 10)

        self.get_logger().info(f"Listening to {det_topic}, publishing /tag_pose (PoseStamped, pixel-frame)")

    def detections_callback(self, msg: AprilTagDetectionArray):
        # If no detections, do nothing (keep topic quiet)
        if not msg.detections:
            return

        det = msg.detections[0]  # take first detection
        out = PoseStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = "camera"

        # pixel-space "pose"
        out.pose.position.x = float(det.centre.x)
        out.pose.position.y = float(det.centre.y)
        out.pose.position.z = 0.0

        out.pose.orientation.x = 0.0
        out.pose.orientation.y = 0.0
        out.pose.orientation.z = 0.0
        out.pose.orientation.w = 1.0

        self.pub.publish(out)


def main():
    rclpy.init()
    node = TagPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
