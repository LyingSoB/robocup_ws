#!/usr/bin/env python3
"""Transform object pose from camera frame to base_link frame."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs


class ObjectPoseTransformer(Node):
    """Listens for object poses in camera frame and republishes in base_link."""

    def __init__(self) -> None:
        super().__init__('object_pose_transformer')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('transform_timeout_sec', 0.2)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, '/arm/object_pose_base', 10)
        self.create_subscription(
            PoseStamped, '/arm/object_pose_camera', self.pose_callback, 10
        )

        self.get_logger().info('ObjectPoseTransformer node started.')

    def pose_callback(self, msg: PoseStamped) -> None:
        target_frame = self.get_parameter('target_frame').value
        timeout = Duration(seconds=float(self.get_parameter('transform_timeout_sec').value))

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout,
            )
        except Exception as exc:
            self.get_logger().warn(f'Transform unavailable: {exc}')
            return

        transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
        transformed.header.frame_id = target_frame
        self.pose_pub.publish(transformed)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ObjectPoseTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
