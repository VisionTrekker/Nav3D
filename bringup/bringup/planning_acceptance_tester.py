#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PlanningAcceptanceTester(Node):
    def __init__(self):
        super().__init__("planning_acceptance_tester")
        self.frame_id = "map"
        self.start = (0.0, 0.0, 0.0)
        self.goals = [(5.0, 3.0, 1.0), (1.0, 1.0, 0.3)]
        self.unreachable_goal = (100.0, 100.0, 10.0)
        self.phase = "first_goal"
        self.phase_started_at = time.monotonic()
        self.goal_sent = False
        self.last_goal_publish_time = 0.0
        self.global_paths = 0
        self.bsplines = 0
        self.cmd_after_bspline = 0
        self.stop_after_unreachable = False
        self.max_cmd_after_unreachable = 0.0
        self.bspline_at_unreachable = 0

        self.odom_pub = self.create_publisher(Odometry, "/lio/localization/odom", 10)
        self.sensor_pub = self.create_publisher(Odometry, "/lio/localization/odom_imu", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/lio/mapping/clouds_lidar", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.create_subscription(Path, "/global_planner/path", self.path_callback, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.bspline_callback, 10)
        self.create_subscription(Twist, "/local_planner/cmd_vel", self.cmd_callback, 10)

        self.io_timer = self.create_timer(0.05, self.publish_io)
        self.step_timer = self.create_timer(0.2, self.step)

    def header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def publish_io(self):
        odom = Odometry()
        odom.header = self.header()
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = self.start[0]
        odom.pose.pose.position.y = self.start[1]
        odom.pose.pose.position.z = self.start[2]
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        self.sensor_pub.publish(odom)
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(self.header(), []))

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header = self.header()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        msg.pose.position.z = goal[2]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.last_goal_publish_time = time.monotonic()
        self.goal_sent = True
        self.get_logger().info(f"Published {self.phase} goal: {goal}")

    def step(self):
        now = time.monotonic()
        if self.phase == "first_goal":
            if not self.goal_sent or now - self.last_goal_publish_time > 2.0:
                self.publish_goal(self.goals[0])
            if self.global_paths >= 1 and self.bsplines >= 1 and self.cmd_after_bspline > 0:
                self.next_phase("second_goal")
        elif self.phase == "second_goal":
            if not self.goal_sent or now - self.last_goal_publish_time > 2.0:
                self.publish_goal(self.goals[1])
            if self.global_paths >= 2 and self.bsplines >= 2:
                self.next_phase("unreachable_goal")
        elif self.phase == "unreachable_goal":
            if not self.goal_sent:
                self.bspline_at_unreachable = self.bsplines
                self.publish_goal(self.unreachable_goal)
            if now - self.phase_started_at > 5.0:
                if self.stop_after_unreachable and self.bsplines == self.bspline_at_unreachable:
                    self.get_logger().info("Formal planning acceptance PASS")
                    raise SystemExit(0)
                self.get_logger().error(
                    "Formal planning acceptance FAIL: "
                    f"stop_after_unreachable={self.stop_after_unreachable}, "
                    f"bsplines_before={self.bspline_at_unreachable}, bsplines_after={self.bsplines}, "
                    f"max_cmd_after_unreachable={self.max_cmd_after_unreachable:.3f}"
                )
                raise SystemExit(2)

        if now - self.phase_started_at > 95.0:
            self.get_logger().error(f"Timeout in phase {self.phase}")
            raise SystemExit(2)

    def next_phase(self, phase):
        self.phase = phase
        self.phase_started_at = time.monotonic()
        self.goal_sent = False
        self.get_logger().info(f"Switching to phase: {phase}")

    def path_callback(self, msg):
        if msg.poses:
            self.global_paths += 1
            self.get_logger().info(f"Observed global path #{self.global_paths}: poses={len(msg.poses)}")

    def bspline_callback(self, msg):
        self.bsplines += 1
        self.get_logger().info(
            f"Observed bspline #{self.bsplines}: traj_id={msg.traj_id}, control_points={len(msg.pos_pts)}"
        )

    def cmd_callback(self, msg):
        speed = math.hypot(msg.linear.x, msg.linear.y) + abs(msg.angular.z)
        if self.bsplines > 0 and self.phase != "unreachable_goal":
            self.cmd_after_bspline += 1
        if self.phase == "unreachable_goal":
            self.max_cmd_after_unreachable = max(self.max_cmd_after_unreachable, speed)
            if speed < 1e-3:
                self.stop_after_unreachable = True


def main():
    rclpy.init()
    node = PlanningAcceptanceTester()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exc.code)


if __name__ == "__main__":
    main()
