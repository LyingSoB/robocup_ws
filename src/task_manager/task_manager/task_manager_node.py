import json
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose


class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')

        # ------------------------
        # State & task management
        # ------------------------
        self.state = "IDLE"

        # Priority-aware task queue
        # Each item is a dict with injected metadata
        self.task_queue = []

        self.current_task = None
        self.current_task_id = None

        # ------------------------
        # Subscribers / Publishers
        # ------------------------
        self.sub = self.create_subscription(
            String,
            '/atwork/task',
            self.task_callback,
            10
        )

        self.feedback_pub = self.create_publisher(
            String,
            '/atwork/task_feedback',
            10
        )

        # Emergency stop publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ------------------------
        # Nav2 action client
        # ------------------------
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.nav_goal_handle = None

        # ------------------------
        # Timeout configuration
        # ------------------------
        self.DEFAULT_NAV_TIMEOUT_SEC = 30.0
        self.NAV_TIMEOUT_SEC = self.DEFAULT_NAV_TIMEOUT_SEC

        self.nav_start_time = None
        self.nav_timeout_timer = None

        self.get_logger().info("🧠 Task Manager READY (waiting for tasks)")

    # ==========================================================
    # Feedback helper
    # ==========================================================
    def publish_feedback(self, task_id, status):
        msg = String()
        msg.data = json.dumps({
            "task_id": task_id,
            "status": status
        })
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"📤 Feedback sent: {msg.data}")

    # ==========================================================
    # Task reception (Phase 7.2: priority queue)
    # ==========================================================
    def task_callback(self, msg: String):
        try:
            task = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON task")
            return

        # -------- Inject defaults --------
        task["priority"] = task.get("priority", 0)
        task["retries_left"] = task.get("retries", 0)

        # Insert task by priority (higher = earlier)
        self.task_queue.append(task)
        self.task_queue.sort(
            key=lambda t: t["priority"],
            reverse=True
        )

        self.get_logger().info(f"📥 Task queued: {task}")

        if self.state == "IDLE":
            self.execute_next_task()

    # ==========================================================
    # Task execution
    # ==========================================================
    def execute_next_task(self):
        if not self.task_queue:
            self.get_logger().info("🟢 Task queue empty, robot idle")
            self.state = "IDLE"
            return

        self.current_task = self.task_queue.pop(0)
        self.current_task_id = self.current_task.get("task_id", "unknown")

        # Phase 7.1 — task-specific timeout
        self.NAV_TIMEOUT_SEC = self.current_task.get(
            "timeout",
            self.DEFAULT_NAV_TIMEOUT_SEC
        )

        self.get_logger().info(f"▶️ Executing task: {self.current_task}")
        self.publish_feedback(self.current_task_id, "STARTED")

        self.state = "BUSY"

        task_type = self.current_task.get("task_type")

        if task_type == "NAVIGATE":
            self.send_navigation_goal(
                self.current_task.get("target_area")
            )
        else:
            self.handle_task_failure("Unsupported task type")

    # ==========================================================
    # Navigation
    # ==========================================================
    def send_navigation_goal(self, target_area: str):
        
        self.state = "NAVIGATING"

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()

        if target_area == "WS1":
            pose.pose.position.x = 1.0
            pose.pose.position.y = 0.5
        elif target_area == "WS2":
            pose.pose.position.x = 2.0
            pose.pose.position.y = -0.5
        else:
            self.handle_task_failure("Unknown target_area")
            return

        pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.handle_task_failure("Nav2 unavailable")
            return

        self.publish_feedback(self.current_task_id, "IN_PROGRESS")

        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

        # Start timeout watchdog
        self.nav_start_time = self.get_clock().now()
        self.start_nav_timeout()

    # ==========================================================
    # Nav2 callbacks
    # ==========================================================
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.handle_task_failure("Goal rejected")
            return

        self.get_logger().info("🚀 Navigation goal accepted")
        self.nav_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.nav_result)

    def nav_result(self, future):
        self.cancel_nav_timeout()
        self.nav_start_time = None

        self.get_logger().info("✅ Goal reached")
        self.publish_feedback(self.current_task_id, "DONE")

        self.current_task = None
        self.state = "IDLE"
        self.execute_next_task()

    # ==========================================================
    # Failure & retry policy (Phase 7.3)
    # ==========================================================
    def handle_task_failure(self, reason: str):
        self.get_logger().error(f"❌ Task failed: {reason}")

        self.cancel_nav_timeout()
        self.nav_start_time = None

        if self.current_task and self.current_task["retries_left"] > 0:
            self.current_task["retries_left"] -= 1
            self.get_logger().warn(
                f"🔁 Retrying task {self.current_task_id} "
                f"({self.current_task['retries_left']} retries left)"
            )
            # Reinsert task with same priority
            self.task_queue.append(self.current_task)
            self.task_queue.sort(
                key=lambda t: t["priority"],
                reverse=True
            )
        else:
            self.publish_feedback(self.current_task_id, "FAILED")

        self.current_task = None
        self.state = "IDLE"
        self.execute_next_task()

    # ==========================================================
    # Timeout logic
    # ==========================================================
    def start_nav_timeout(self):
        self.cancel_nav_timeout()
        self.nav_timeout_timer = self.create_timer(
            0.2,
            self.nav_timeout_callback
        )
        self.get_logger().warn(
            f"⏱ Navigation timeout armed ({self.NAV_TIMEOUT_SEC}s)"
        )

    def cancel_nav_timeout(self):
        if self.nav_timeout_timer:
            self.nav_timeout_timer.cancel()
            self.nav_timeout_timer = None

    def nav_timeout_callback(self):
        if self.nav_start_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.nav_start_time
        ).nanoseconds / 1e9

        if elapsed < self.NAV_TIMEOUT_SEC:
            return

        self.get_logger().error(
            f"⏱ Navigation TIMEOUT after {elapsed:.1f}s"
        )

        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
            self.nav_goal_handle = None

        self.cmd_vel_pub.publish(Twist())
        self.handle_task_failure("Timeout")


def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    rclpy.shutdown()

