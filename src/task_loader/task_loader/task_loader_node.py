import json
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


WORKSTATION_MAP = {
    "WORKSTATION_1": "WS1",
    "WORKSTATION_2": "WS2",
    "WORKSTATION_3": "WS3",
}


class TaskLoader(Node):
    def __init__(self, json_file):
        super().__init__('task_loader')

        self.task_pub = self.create_publisher(
            String,
            '/atwork/task',
            10
        )

        self.feedback_sub = self.create_subscription(
            String,
            '/atwork/task_feedback',
            self.feedback_callback,
            10
        )

        if not os.path.isfile(json_file):
            self.get_logger().error(f"❌ Task file not found: {json_file}")
            rclpy.shutdown()
            return

        self.get_logger().info(f"📂 Loading task file: {json_file}")

        with open(json_file, 'r') as f:
            self.task_data = json.load(f)

        self.atomic_tasks = self.expand_tasks(self.task_data)

        self.current_index = 0
        self.waiting_for_feedback = False

        # 🔒 WAIT until TaskManager is connected
        self.get_logger().info("⏳ Waiting for task_manager to subscribe...")
        while self.task_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.publish_next_task()

    def expand_tasks(self, data):
        tasks = []

        source_raw = data.get("source_location", {}).get("name")
        source = WORKSTATION_MAP.get(source_raw, source_raw)

        for obj in data.get("objects", []):
            target_raw = obj.get("target")
            target = WORKSTATION_MAP.get(target_raw, target_raw)

            tasks.append({
                "task_type": "NAVIGATE",
                "target_area": source
            })

            tasks.append({
                "task_type": "PICK",
                "object_id": obj.get("id")
            })

            tasks.append({
                "task_type": "NAVIGATE",
                "target_area": target
            })

            tasks.append({
                "task_type": "PLACE",
                "object_id": obj.get("id")
            })

        return tasks

    def publish_next_task(self):
        if self.current_index >= len(self.atomic_tasks):
            self.get_logger().info("✅ All tasks executed")
            return

        task = self.atomic_tasks[self.current_index]
        task["task_id"] = str(self.current_index + 1)

        msg = String()
        msg.data = json.dumps(task)

        self.waiting_for_feedback = True
        self.task_pub.publish(msg)

        self.get_logger().info(f"📤 Published task: {msg.data}")

    def feedback_callback(self, msg):
        if not self.waiting_for_feedback:
            return

        try:
            feedback = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if feedback.get("task_id") != str(self.current_index + 1):
            return

        if feedback.get("status") in ["DONE", "FAILED"]:
            self.waiting_for_feedback = False
            self.current_index += 1
            self.publish_next_task()


def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run task_loader task_loader_node <task_file.json>")
        return

    node = TaskLoader(sys.argv[1])
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

