import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')

        self.task_sub = self.create_subscription(
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

        self.task_queue = []
        self.state = "IDLE"

        self.current_task = None
        self.current_task_id = None

        self.dummy_timer = None

        self.get_logger().info("🧠 Task Manager READY (waiting for tasks)")

    def task_callback(self, msg):
        task = json.loads(msg.data)
        self.task_queue.append(task)
        self.get_logger().info(f"📥 Task queued: {task}")
        self.execute_next_task()

    def execute_next_task(self):
        if self.state != "IDLE":
            return

        if not self.task_queue:
            return

        self.current_task = self.task_queue.pop(0)
        self.current_task_id = self.current_task["task_id"]
        self.state = "BUSY"

        self.publish_feedback("STARTED")

        task_type = self.current_task.get("task_type")

        if task_type == "NAVIGATE":
            self.handle_navigate()

        elif task_type in ["PICK", "PLACE"]:
            self.get_logger().info(f"🧪 Simulating {task_type} for task {self.current_task_id}")
            self.dummy_timer = self.create_timer(1.0, self.finish_dummy_task)

        else:
            self.finish_task("FAILED")

    def handle_navigate(self):
        self.get_logger().info(f"🚀 Navigating to {self.current_task.get('target_area')}")
        time.sleep(0.5)
        self.finish_task("DONE")

    def finish_dummy_task(self):
        if self.dummy_timer is not None:
            self.dummy_timer.cancel()
            self.dummy_timer = None

        self.finish_task("DONE")

    def finish_task(self, status):
        self.publish_feedback(status)
        self.current_task = None
        self.current_task_id = None
        self.state = "IDLE"
        self.execute_next_task()

    def publish_feedback(self, status):
        msg = String()
        msg.data = json.dumps({
            "task_id": self.current_task_id,
            "status": status
        })
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"📤 Feedback sent: {msg.data}")


def main():
    rclpy.init()
    node = TaskManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

