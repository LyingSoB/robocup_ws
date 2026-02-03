#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ArmJointBridge(Node):

    def __init__(self):
        super().__init__('arm_joint_bridge')

        self.latest_state = None

        self.sub = self.create_subscription(
            JointState,
            '/arm/joint_targets',
            self.cb,
            10
        )

        self.pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_state)

        self.get_logger().info('Arm Joint Bridge running (continuous mode)')

    def cb(self, msg: JointState):
        self.latest_state = msg

    def publish_state(self):
        if self.latest_state is None:
            return

        self.latest_state.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.latest_state)


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

