#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time


class SerialJointReader(Node):

    def __init__(self):
        super().__init__('serial_joint_reader')

        # ---- PARAMETERS ----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # ---- SERIAL ----
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        time.sleep(2.0)

        # ---- ROS ----
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.read_serial)

        self.joint_names = [
            'joint_base_yaw',
            'joint_shoulder_pitch',
            'joint_elbow_pitch',
            'joint_wrist_pitch'
        ]

        self.get_logger().info(f'SerialJointReader connected to {port}')

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return

            # Expected format:
            # J1:x,J2:y,J3:z,J4:w
            parts = line.split(',')

            angles = []
            for p in parts:
                _, value = p.split(':')
                angles.append(float(value))

            if len(angles) != 4:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = angles

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Serial parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialJointReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
