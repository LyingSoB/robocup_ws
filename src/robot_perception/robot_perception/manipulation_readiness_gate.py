#!/usr/bin/env python3
"""Gate manipulation based on navigation status and object pose availability."""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool


class ManipulationReadinessGate(Node):
    """Publishes whether manipulation is allowed based on nav and pose status."""

    def __init__(self) -> None:
        super().__init__('manipulation_readiness_gate')
        self.declare_parameter('pose_timeout_sec', 1.0)
        self.declare_parameter('navigation_active_is_true', True)

        self.nav_active = True
        self.last_pose_time = None

        self.object_reachable_pub = self.create_publisher(
            Bool, '/arm/object_reachable', 10
        )

        self.create_subscription(Bool, '/navigation/status', self.nav_callback, 10)
        self.create_subscription(
            PoseStamped, '/arm/object_pose_base', self.pose_callback, 10
        )

        self.timer = self.create_timer(0.1, self.publish_readiness)
        self.get_logger().info('ManipulationReadinessGate node started.')

    def nav_callback(self, msg: Bool) -> None:
        navigation_active_is_true = bool(self.get_parameter('navigation_active_is_true').value)
        self.nav_active = msg.data if navigation_active_is_true else not msg.data

    def pose_callback(self, msg: PoseStamped) -> None:
        self.last_pose_time = self.get_clock().now()

    def publish_readiness(self) -> None:
        navigation_stopped = not self.nav_active
        pose_valid = self._pose_is_recent()
        ready = navigation_stopped and pose_valid
        self.object_reachable_pub.publish(Bool(data=ready))

    def _pose_is_recent(self) -> bool:
        if self.last_pose_time is None:
            return False
        timeout = Duration(seconds=float(self.get_parameter('pose_timeout_sec').value))
        return (self.get_clock().now() - self.last_pose_time) <= timeout


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ManipulationReadinessGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
