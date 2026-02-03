#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import JointState

import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ArmStateMachine(Node):

    def __init__(self):
        super().__init__('arm_state_machine')

        # =====================================================
        # Load arm poses (Phase 3.1)
        # =====================================================
        pkg_path = get_package_share_directory('robot_manipulation')
        poses_file = os.path.join(pkg_path, 'config', 'arm_poses.yaml')

        with open(poses_file, 'r') as f:
            self.poses = yaml.safe_load(f)

        self.get_logger().info('Arm poses loaded successfully')

        # =====================================================
        # Internal state
        # =====================================================
        self.state = 'IDLE'
        self.current_base_yaw = None
        self.object_reachable = False

        # =====================================================
        # Publishers
        # =====================================================
        # (command to hardware later, NOT RViz)
        self.joint_pub = self.create_publisher(
            JointState,
            '/arm/joint_targets',
            10
        )

        # Decision output (Phase 3.4)
        self.decision_pub = self.create_publisher(
            String,
            '/arm/command_decision',
            10
        )

        # =====================================================
        # Subscribers
        # =====================================================
        # Manual commands (testing only)
        self.create_subscription(
            String,
            '/arm/command',
            self.command_callback,
            10
        )

        # Perception → yaw
        self.create_subscription(
            Float64,
            '/arm/desired_base_yaw',
            self.cb_desired_yaw,
            10
        )

        # Perception → reachability
        self.create_subscription(
            Bool,
            '/arm/object_reachable',
            self.cb_object_reachable,
            10
        )

        self.get_logger().info('Arm State Machine initialized (Phase 3.4)')

    # =====================================================
    # Manual command handler (Phase 3.2 support)
    # =====================================================
    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        self.get_logger().info(f'Received manual command: {command}')

        if command == 'home':
            self.go_to_pose('home')

        elif command == 'pre_grasp':
            self.go_to_pose('pre_grasp')

        else:
            self.get_logger().warn(f'Unknown manual command: {command}')

    # =====================================================
    # Perception callbacks (Phase 3.4)
    # =====================================================
    def cb_desired_yaw(self, msg: Float64):
        self.current_base_yaw = msg.data
        self.get_logger().info(
            f'Perception update: desired base yaw = {msg.data:.2f} rad'
        )

    def cb_object_reachable(self, msg: Bool):
        self.object_reachable = msg.data
        self.get_logger().info(
            f'Perception update: object_reachable = {msg.data}'
        )

        # Every time reachability updates, re-evaluate decision
        self.evaluate_pick_decision()

    # =====================================================
    # Decision logic (CORE of Phase 3.4)
    # =====================================================
    def evaluate_pick_decision(self):

        if not self.object_reachable:
            self.publish_decision('abort_pick')
            self.state = 'IDLE'
            return

        if self.current_base_yaw is None:
            self.get_logger().warn('Base yaw not available yet')
            return

        # At this point:
        # - object is reachable
        # - base yaw is known
        self.publish_decision('prepare_pre_grasp')
        self.state = 'PRE_GRASP_READY'

    def publish_decision(self, decision: str):
        msg = String()
        msg.data = decision
        self.decision_pub.publish(msg)
        self.get_logger().info(f'DECISION → {decision}')

    # =====================================================
    # Pose execution (joint-space, hardware-oriented)
    # =====================================================
    def go_to_pose(self, pose_name: str):

        if pose_name not in self.poses:
            self.get_logger().error(f'Pose "{pose_name}" not found')
            return

        pose = self.poses[pose_name]

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()

        for joint, value in pose.items():
            js.name.append(joint)
            js.position.append(float(value))

        self.joint_pub.publish(js)

        self.state = pose_name.upper()
        self.get_logger().info(f'Arm command sent → pose: {self.state}')


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMachine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

