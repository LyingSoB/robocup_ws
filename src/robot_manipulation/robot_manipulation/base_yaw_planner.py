#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Bool


class BaseYawPlanner(Node):

    def __init__(self):
        super().__init__('base_yaw_planner')

        # ---- parameters ----
        self.declare_parameter('j1_min', -1.57)
        self.declare_parameter('j1_max',  2.27)
        self.declare_parameter('max_arm_reach', 0.60)

        self.j1_min = self.get_parameter('j1_min').value
        self.j1_max = self.get_parameter('j1_max').value
        self.max_arm_reach = self.get_parameter('max_arm_reach').value

        # ---- pubs/subs ----
        self.sub = self.create_subscription(
            Point,
            '/detected_object_position',
            self.cb_object,
            10
        )

        self.yaw_pub = self.create_publisher(
            Float64,
            '/arm/desired_base_yaw',
            10
        )

        self.reach_pub = self.create_publisher(
            Bool,
            '/arm/object_reachable',
            10
        )

        self.get_logger().info('BaseYawPlanner ready')

    def cb_object(self, msg: Point):

        x, y = msg.x, msg.y

        # ---- yaw computation ----
        yaw = math.atan2(y, x)

        # ---- clamp ----
        yaw_clamped = max(self.j1_min, min(self.j1_max, yaw))

        # ---- reachability ----
        r = math.sqrt(x*x + y*y)
        reachable = r <= self.max_arm_reach

        # ---- publish ----
        self.yaw_pub.publish(Float64(data=yaw_clamped))
        self.reach_pub.publish(Bool(data=reachable))

        self.get_logger().info(
            f'Object (x={x:.2f}, y={y:.2f}) → yaw={yaw_clamped:.2f} rad, reachable={reachable}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BaseYawPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
