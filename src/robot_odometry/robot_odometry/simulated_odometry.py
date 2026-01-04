import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import time


class SimulatedOdometry(Node):

    def __init__(self):
        super().__init__('simulated_odometry')

        # ---------------- ROBOT PARAMETERS ----------------
        self.R = 0.05     # wheel radius (m)
        self.L = 0.30     # half-length (m)
        self.W = 0.30     # half-width (m)

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        # ---------------- ROS INTERFACES ----------------
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_commands',
            self.wheel_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.get_logger().info("Simulated odometry started")
        self.tf_broadcaster = TransformBroadcaster(self)

    def wheel_callback(self, msg: Float32MultiArray):
        w_fl, w_fr, w_rl, w_rr = msg.data

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # -------- Mecanum forward kinematics --------
        vx = self.R / 4.0 * (w_fl + w_fr + w_rl + w_rr)
        vy = self.R / 4.0 * (-w_fl + w_fr + w_rl - w_rr)
        omega = self.R / (4.0 * (self.L + self.W)) * (
            -w_fl + w_fr - w_rl + w_rr
        )

        # -------- Integrate pose --------
        self.x += (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        self.y += (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.theta += omega * dt

        # -------- Quaternion --------
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=qz, w=qw
        )

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)
                # -------- TF: odom -> base_link --------
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = SimulatedOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
