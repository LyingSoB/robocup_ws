#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray

import serial


class WheelBridge(Node):

    def __init__(self):
        super().__init__('wheel_bridge')

        # ---------------- Publishers ----------------
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/wheel_ticks',
            10
        )

        # ---------------- Subscribers ----------------
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_commands',
            self.cmd_callback,
            10
        )

        # ---------------- Serial ----------------
        try:
            self.ser = serial.Serial(
                '/dev/ttyACM0',
                115200,
                timeout=0.01
            )
            self.get_logger().info("Connected to Arduino on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            raise

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

    # ==================================================
    # ============== SEND MOTOR COMMANDS ===============
    # ==================================================
    def cmd_callback(self, msg: Float32MultiArray):
        """
        Receives wheel velocities from mecanum_base
        Format: [fl, fr, rl, rr]
        Sends to Arduino as:
        fl,fr,rl,rr\n
        """

        if len(msg.data) != 4:
            self.get_logger().warn("wheel_commands must have 4 elements")
            return

        cmd_str = "{:.4f},{:.4f},{:.4f},{:.4f}\n".format(
            msg.data[0],
            msg.data[1],
            msg.data[2],
            msg.data[3]
        )

        try:
            self.ser.write(cmd_str.encode())
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    # ==================================================
    # ============== READ ENCODERS =====================
    # ==================================================
    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return

            # Expect: ENC,fl,fr,rl,rr
            if not line.startswith("ENC"):
                return

            parts = line.split(',')
            if len(parts) != 5:
                return

            _, fl, fr, rl, rr = parts

            msg = Int32MultiArray()
            msg.data = [
                int(fl),
                int(fr),
                int(rl),
                int(rr)
            ]

            self.encoder_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")


def main():
    rclpy.init()
    node = WheelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

