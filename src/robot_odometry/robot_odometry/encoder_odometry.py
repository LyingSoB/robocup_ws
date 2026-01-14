import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

import math
import time


class EncoderOdometry(Node):

    def __init__(self):
        super().__init__('encoder_odometry')

        # ---------------- ROBOT PARAMETERS ----------------
        self.wheel_radius = 0.05        # meters
        self.base_length = 0.30         # meters
        self.base_width = 0.30          # meters

        self.encoder_ticks_per_rev = 2500.0  # NOE2-05 + 5:1 gearbox

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.last_time = self.get_clock().now()

        # ---------------- ROS INTERFACES ----------------
        self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("📐 Encoder-based odometry started")

    # --------------------------------------------------
    def encoder_callback(self, msg: Int32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn("wheel_ticks must have 4 values")
            return

        # Time
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # Ticks
        fl, fr, rl, rr = msg.data

        # Convert ticks → wheel angular displacement (rad)
        def ticks_to_rad(ticks):
            return (2.0 * math.pi * ticks) / self.encoder_ticks_per_rev

        w_fl = ticks_to_rad(fl) / dt
        w_fr = ticks_to_rad(fr) / dt
        w_rl = ticks_to_rad(rl) / dt
        w_rr = ticks_to_rad(rr) / dt

        R = self.wheel_radius
        L = self.base_length
        W = self.base_width

        # -------- MECANUM FORWARD KINEMATICS --------
        vx = (R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
        vy = (R / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
        omega = (R / (4.0 * (L + W))) * (-w_fl + w_fr - w_rl + w_rr)

        # -------- INTEGRATE --------
        dx = vx * math.cos(self.yaw) - vy * math.sin(self.yaw)
        dy = vx * math.sin(self.yaw) + vy * math.cos(self.yaw)

        self.x += dx * dt
        self.y += dy * dt
        self.yaw += omega * dt

        # -------- PUBLISH ODOM --------
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = self.yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # -------- TF --------
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    # --------------------------------------------------
    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main():
    rclpy.init()
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
