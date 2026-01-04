import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MecanumBase(Node):

    def __init__(self):
        super().__init__('mecanum_base')

        # ---------------- ROBOT PARAMETERS ----------------
        self.wheel_radius = 0.05   # meters (100 mm diameter)
        self.base_length  = 0.30   # meters (front-back distance / 2)
        self.base_width   = 0.30   # meters (left-right distance / 2)

        # ---------------- ROS INTERFACES ------------------
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.wheel_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_commands',
            10
        )

        self.get_logger().info("Mecanum base controller started")

    def cmd_vel_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        L = self.base_length
        W = self.base_width
        R = self.wheel_radius

        # -------- Mecanum inverse kinematics --------
        w_fl = (1.0 / R) * (vx - vy - (L + W) * omega)
        w_fr = (1.0 / R) * (vx + vy + (L + W) * omega)
        w_rl = (1.0 / R) * (vx + vy - (L + W) * omega)
        w_rr = (1.0 / R) * (vx - vy + (L + W) * omega)

        wheel_msg = Float32MultiArray()
        wheel_msg.data = [w_fl, w_fr, w_rl, w_rr]

        self.wheel_pub.publish(wheel_msg)


def main():
    rclpy.init()
    node = MecanumBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
