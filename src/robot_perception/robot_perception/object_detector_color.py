#!/usr/bin/env python3
"""Detect a single colored object using RGB + depth images.

This node uses simple HSV color thresholding and estimates the 3D position
of the detected object in the camera frame using depth at the centroid.
"""

from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


class ColorObjectDetector(Node):
    """Detects a single colored object and publishes its pose in camera frame."""

    def __init__(self) -> None:
        super().__init__('color_object_detector')
        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_encoding = None

        self.declare_parameter('hsv_lower', [20, 100, 100])
        self.declare_parameter('hsv_upper', [40, 255, 255])
        self.declare_parameter('min_contour_area', 500.0)
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('fx', 615.0)
        self.declare_parameter('fy', 615.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('camera_frame', 'camera_link')

        self.object_pub = self.create_publisher(PoseStamped, '/arm/object_pose_camera', 10)

        self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10
        )
        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        self.get_logger().info('ColorObjectDetector node started.')

    def depth_callback(self, msg: Image) -> None:
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return
        self.depth_image = np.array(depth_image, dtype=np.float32)
        self.depth_encoding = msg.encoding

    def color_callback(self, msg: Image) -> None:
        if self.depth_image is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert color image: {exc}')
            return

        mask = self._threshold_color(color_image)
        contour = self._largest_contour(mask)
        if contour is None:
            return

        cx, cy = self._contour_centroid(contour)
        if cx is None:
            return

        depth_m = self._depth_at_pixel(cx, cy)
        if depth_m is None:
            return

        intrinsics = self._camera_intrinsics()
        x = (cx - intrinsics.cx) * depth_m / intrinsics.fx
        y = (cy - intrinsics.cy) * depth_m / intrinsics.fy
        z = depth_m

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.get_parameter('camera_frame').value
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        pose_msg.pose.orientation.w = 1.0

        self.object_pub.publish(pose_msg)

    def _threshold_color(self, color_image: np.ndarray) -> np.ndarray:
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower = np.array(self.get_parameter('hsv_lower').value, dtype=np.uint8)
        upper = np.array(self.get_parameter('hsv_upper').value, dtype=np.uint8)
        mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.medianBlur(mask, 5)
        return mask

    def _largest_contour(self, mask: np.ndarray) -> np.ndarray | None:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        min_area = float(self.get_parameter('min_contour_area').value)
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_area:
            return None
        return largest

    def _contour_centroid(self, contour: np.ndarray) -> tuple[int | None, int | None]:
        moments = cv2.moments(contour)
        if moments['m00'] == 0:
            return None, None
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        return cx, cy

    def _depth_at_pixel(self, cx: int, cy: int) -> float | None:
        if self.depth_image is None:
            return None

        window = int(self.get_parameter('depth_window').value)
        half = max(window // 2, 1)
        height, width = self.depth_image.shape[:2]
        x_min = max(cx - half, 0)
        x_max = min(cx + half + 1, width)
        y_min = max(cy - half, 0)
        y_max = min(cy + half + 1, height)

        depth_patch = self.depth_image[y_min:y_max, x_min:x_max]
        if self.depth_encoding == '16UC1':
            depth_patch = depth_patch / 1000.0

        depth_values = depth_patch[depth_patch > 0.0]
        if depth_values.size == 0:
            return None
        return float(np.median(depth_values))

    def _camera_intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(
            fx=float(self.get_parameter('fx').value),
            fy=float(self.get_parameter('fy').value),
            cx=float(self.get_parameter('cx').value),
            cy=float(self.get_parameter('cy').value),
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ColorObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

