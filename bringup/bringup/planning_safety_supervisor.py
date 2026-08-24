#!/usr/bin/env python3

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Bool


class PlanningSafetySupervisor(Node):
    def __init__(self):
        super().__init__("planning_safety_supervisor")
        self.path_timeout_sec = self.declare_parameter("path_timeout_sec", 3.0).value
        self.waiting_for_path = False
        self.goal_time = None
        self.stop_active = False

        self.stop_pub = self.create_publisher(Bool, "planning/stop_requested", 10)
        self.create_subscription(PoseStamped, "goal_pose", self.goal_callback, 10)
        self.create_subscription(Path, "global_planner/path", self.path_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def publish_stop(self, stop):
        msg = Bool()
        msg.data = bool(stop)
        self.stop_pub.publish(msg)
        self.stop_active = bool(stop)

    def goal_callback(self, _msg):
        self.goal_time = self.get_clock().now()
        self.waiting_for_path = True
        self.publish_stop(True)
        self.get_logger().info("New goal received; stopping old trajectory until a fresh global path arrives")

    def path_callback(self, msg):
        if not self.waiting_for_path or self.goal_time is None:
            return
        path_time = rclpy.time.Time.from_msg(msg.header.stamp)
        if path_time < self.goal_time:
            return
        if not msg.poses:
            return
        self.waiting_for_path = False
        self.publish_stop(False)
        self.get_logger().info(
            f"Fresh global path received; releasing controller stop, path_points={len(msg.poses)}"
        )

    def timer_callback(self):
        if not self.waiting_for_path or self.goal_time is None:
            return
        elapsed = (self.get_clock().now() - self.goal_time).nanoseconds * 1e-9
        if elapsed > self.path_timeout_sec:
            self.publish_stop(True)
            self.waiting_for_path = False
            self.get_logger().error(
                f"No fresh global path within {self.path_timeout_sec:.1f}s; keeping controller stopped"
            )


def main():
    rclpy.init()
    node = PlanningSafetySupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
