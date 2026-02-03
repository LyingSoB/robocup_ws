#!/usr/bin/env python3
"""
Manipulation Executor Node
Executes manipulation steps based on arm_state_machine decisions

This node is the ONLY execution layer.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
import time


class ManipulationExecutor(Node):

    def __init__(self):
        super().__init__('manipulation_executor')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud').value

        # ---------------- SERIAL ----------------
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            time.sleep(2.0)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().fatal(f"Serial connection failed: {e}")
            raise e

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            String,
            '/arm/command_decision',
            self.decision_cb,
            10
        )

        self.create_subscription(
            Float32,
            '/arm/desired_base_yaw',
            self.base_yaw_cb,
            10
        )

        # ---------------- INTERNAL STATE ----------------
        self.desired_base_yaw = None
        self.busy = False

        self.get_logger().info("Manipulation Executor ready.")

    # ------------------------------------------------

    def base_yaw_cb(self, msg):
        self.desired_base_yaw = msg.data

    def decision_cb(self, msg):
        if self.busy:
            self.get_logger().warn("Executor busy, ignoring decision.")
            return

        decision = msg.data
        self.get_logger().info(f"Decision received: {decision}")

        if decision == "prepare_pre_grasp":
            self.execute_pick_sequence()

        elif decision == "abort_pick":
            self.send_home()

    # ------------------------------------------------

    def send_cmd(self, cmd):
        self.get_logger().info(f"→ Arduino: {cmd}")
        self.ser.write((cmd + '\n').encode())
        time.sleep(0.1)

    # ------------------------------------------------

    def execute_pick_sequence(self):
        self.busy = True

        try:
            # 1️⃣ Ensure PRE_GRASP (state machine already commanded it)
            self.get_logger().info("Assuming PRE_GRASP pose reached.")
            time.sleep(1.5)

            # 2️⃣ Align base yaw
            if self.desired_base_yaw is None:
                self.get_logger().warn("No base yaw received. Aborting.")
                self.send_home()
                return

            self.get_logger().info(f"Aligning base to {self.desired_base_yaw:.2f} deg")
            self.send_cmd(f"SET J1 {self.desired_base_yaw}")
            time.sleep(2.0)

            # 3️⃣ DESCEND (stub – no Z motion yet)
            self.get_logger().info("DESCEND (stub)")
            time.sleep(1.0)

            # 4️⃣ GRASP (stub – gripper control later)
            self.get_logger().info("GRASP (stub)")
            time.sleep(1.0)

            # 5️⃣ LIFT (stub)
            self.get_logger().info("LIFT (stub)")
            time.sleep(1.0)

            # 6️⃣ Return HOME
            self.send_home()

        finally:
            self.busy = False

    # ------------------------------------------------

    def send_home(self):
        self.get_logger().info("Returning HOME")
        self.send_cmd("HOME")
        time.sleep(2.0)


def main():
    rclpy.init()
    node = ManipulationExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

