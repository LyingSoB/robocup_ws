#!/usr/bin/env python3
"""Detect a dominant horizontal surface (e.g., a table) from depth images.

This node is intentionally simple and uses geometry-only logic (no ML).
It looks for a dominant depth band in the central region of the image and
reports whether a large planar surface is present.
"""

from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge


@dataclass
class DepthImageInfo:
    image_m: np.ndarray
    valid_mask: np.ndarray


class TablePlaneDetector(Node):
    """Detects a dominant horizontal plane using depth-only cues."""

    def __init__(self) -> None:
        super().__init__('table_plane_detector')
        self.bridge = CvBridge()

        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 3.0)
        self.declare_parameter('roi_scale', 0.5)
        self.declare_parameter('min_valid_points', 500)
        self.declare_parameter('plane_tolerance_m', 0.03)
        self.declare_parameter('min_plane_ratio', 0.4)

        self.table_detected_pub = self.create_publisher(
            Bool, '/navigation/table_detected', 10
        )
        self.table_distance_pub = self.create_publisher(
            Float32, '/navigation/table_distance', 10
        )

        self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        self.get_logger().info('TablePlaneDetector node started.')

    def depth_callback(self, msg: Image) -> None:
        depth_info = self._convert_depth(msg)
        if depth_info is None:
            self._publish(False, 0.0)
            return

        depth_m, valid_mask = depth_info.image_m, depth_info.valid_mask
        roi = self._extract_roi(depth_m, valid_mask)
        if roi is None:
            self._publish(False, 0.0)
            return

        roi_depths = roi[roi > 0.0]
        if roi_depths.size < self.get_parameter('min_valid_points').value:
            self._publish(False, 0.0)
            return

        median_depth = float(np.median(roi_depths))
        tolerance = float(self.get_parameter('plane_tolerance_m').value)
        plane_mask = np.abs(roi_depths - median_depth) <= tolerance
        plane_ratio = float(np.count_nonzero(plane_mask)) / float(roi_depths.size)

        detected = plane_ratio >= float(self.get_parameter('min_plane_ratio').value)
        self._publish(detected, median_depth if detected else 0.0)

    def _convert_depth(self, msg: Image) -> DepthImageInfo | None:
        """Convert incoming depth image to meters."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert depth image: {exc}')
            return None

        depth_image = np.array(depth_image, dtype=np.float32)
        if msg.encoding == '16UC1':
            depth_m = depth_image / 1000.0
        else:
            depth_m = depth_image

        min_depth = float(self.get_parameter('min_depth_m').value)
        max_depth = float(self.get_parameter('max_depth_m').value)
        valid_mask = (depth_m > min_depth) & (depth_m < max_depth)
        return DepthImageInfo(image_m=depth_m, valid_mask=valid_mask)

    def _extract_roi(self, depth_m: np.ndarray, valid_mask: np.ndarray) -> np.ndarray | None:
        """Extract a central region of interest for table detection."""
        height, width = depth_m.shape[:2]
        roi_scale = float(self.get_parameter('roi_scale').value)
        roi_scale = np.clip(roi_scale, 0.1, 1.0)

        roi_w = int(width * roi_scale)
        roi_h = int(height * roi_scale)
        start_x = (width - roi_w) // 2
        start_y = (height - roi_h) // 2

        roi_depth = depth_m[start_y:start_y + roi_h, start_x:start_x + roi_w]
        roi_valid = valid_mask[start_y:start_y + roi_h, start_x:start_x + roi_w]
        roi = np.where(roi_valid, roi_depth, 0.0)
        if np.count_nonzero(roi) == 0:
            return None
        return roi

    def _publish(self, detected: bool, distance: float) -> None:
        self.table_detected_pub.publish(Bool(data=detected))
        self.table_distance_pub.publish(Float32(data=float(distance)))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TablePlaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
