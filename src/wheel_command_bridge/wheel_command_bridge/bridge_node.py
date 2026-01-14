import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
import serial


class WheelBridge(Node):

    def __init__(self):
        super().__init__('wheel_bridge')

        # -------- Publisher --------
        self.pub = self.create_publisher(
            Int32MultiArray,
            '/wheel_ticks',
            10
        )

        # -------- Serial --------
        try:
            self.ser = serial.Serial(
                '/dev/ttyACM1',
                115200,
                timeout=0.1
            )
            self.get_logger().info("Serial port opened: /dev/ttyACM1")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # -------- Timer --------
        self.timer = self.create_timer(0.02, self.read_serial)  # 50 Hz

        self.get_logger().info("Wheel command bridge started")

    # --------------------------------------------------
    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            self.get_logger().info(f"RAW SERIAL: '{line}'")
            if not line:
                return

            # Expect: ENC,<delta>
            if not line.startswith("ENC"):
                return

            _, delta = line.split(',')
            delta = int(delta)

            msg = Int32MultiArray()
            # ONE wheel for now: FL only
            msg.data = [delta, 0, 0, 0]

            self.pub.publish(msg)

            # DEBUG (important)
            self.get_logger().info(f"Published wheel_ticks: {msg.data}")

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

