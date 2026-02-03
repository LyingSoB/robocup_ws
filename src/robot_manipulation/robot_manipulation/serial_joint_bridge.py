#!/usr/bin/env python3
"""
ROS2 Serial Joint Bridge
Reads joint data from Arduino over serial
Publishes sensor_msgs/JointState

Author: Final working version
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math
import time


class SerialJointBridge(Node):

    def __init__(self):
        super().__init__('serial_joint_bridge')

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate', 10.0)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        rate = self.get_parameter('rate').get_parameter_value().double_value

        # ---------------- SERIAL ----------------
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            time.sleep(2.0)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to open serial port: {e}")
            raise e

        # ---------------- PUBLISHER ----------------
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # ---------------- TIMER ----------------
        self.timer = self.create_timer(1.0 / rate, self.update)

        # ---------------- JOINT STATE ----------------
        self.msg = JointState()
        self.msg.name = [
            'joint_base_yaw',
            'joint_shoulder',
            'joint_elbow',
            'joint_wrist_pitch'
        ]

        self.get_logger().info("Serial Joint Bridge started.")

    # ------------------------------------------------

    def update(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            # Expected format:
            # J1(deg): 45.23 | J2(adc): -120 | J3(adc): 230 | J4(adc): -15

            parts = line.replace('|', '').split()

            j1_deg = float(parts[1])
            j2_adc = int(parts[4])
            j3_adc = int(parts[7])
            j4_adc = int(parts[10])

            # ---------------- CONVERSIONS ----------------
            j1_rad = math.radians(j1_deg)

            # NOTE:
            # ADC -> rad scaling is temporary.
            # You will refine this in Phase C.
            ADC_TO_RAD = 0.001  # placeholder scale

            j2_rad = j2_adc * ADC_TO_RAD
            j3_rad = j3_adc * ADC_TO_RAD
            j4_rad = j4_adc * ADC_TO_RAD

            # ---------------- PUBLISH ----------------
            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.msg.position = [j1_rad, j2_rad, j3_rad, j4_rad]

            self.pub.publish(self.msg)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")


def main():
    rclpy.init()
    node = SerialJointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
