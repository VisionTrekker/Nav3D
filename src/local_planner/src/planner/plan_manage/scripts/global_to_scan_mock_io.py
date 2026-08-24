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


class GlobalToScanMockIO(Node):
    def __init__(self):
        super().__init__("global_to_scan_mock_io")
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.start = (
            self.declare_parameter("start_x", 0.0).value,
            self.declare_parameter("start_y", 0.0).value,
            self.declare_parameter("start_z", 0.0).value,
        )
        self.goal = (
            self.declare_parameter("goal_x", 5.0).value,
            self.declare_parameter("goal_y", 3.0).value,
            self.declare_parameter("goal_z", 1.0).value,
        )
        self.goal_publish_count = 0
        self.last_goal_publish_time = 0.0
        self.global_path_count = 0
        self.bspline_count = 0
        self.cmd_vel_count = 0
        self.first_global_path_z = None

        self.odom_pub = self.create_publisher(Odometry, "/lio/localization/odom", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/global_to_scan/cloud", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.create_subscription(Path, "/global_planner/path", self.global_path_callback, 10)
        self.create_subscription(Bspline, "/planning/bspline", self.bspline_callback, 10)
        self.create_subscription(Twist, "/global_to_scan/cmd_vel", self.cmd_vel_callback, 10)

        self.publish_timer = self.create_timer(0.05, self.publish_mock_state)
        self.goal_timer = self.create_timer(0.5, self.maybe_publish_goal)
        self.done_timer = self.create_timer(0.5, self.check_done)
        self.started_at = time.monotonic()

    def now_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def publish_mock_state(self):
        odom = Odometry()
        odom.header = self.now_header()
        odom.child_frame_id = "base"
        odom.pose.pose.position.x = self.start[0]
        odom.pose.pose.position.y = self.start[1]
        odom.pose.pose.position.z = self.start[2]
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        cloud = point_cloud2.create_cloud_xyz32(self.now_header(), [])
        self.cloud_pub.publish(cloud)

    def maybe_publish_goal(self):
        if self.global_path_count > 0:
            return
        if time.monotonic() - self.started_at < 3.0:
            return
        if time.monotonic() - self.last_goal_publish_time < 2.0:
            return

        goal = PoseStamped()
        goal.header = self.now_header()
        goal.pose.position.x = self.goal[0]
        goal.pose.position.y = self.goal[1]
        goal.pose.position.z = self.goal[2]
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.goal_publish_count += 1
        self.last_goal_publish_time = time.monotonic()
        self.get_logger().info(
            f"Published goal_pose #{self.goal_publish_count}: "
            f"({self.goal[0]:.2f}, {self.goal[1]:.2f}, {self.goal[2]:.2f})"
        )

    def global_path_callback(self, msg):
        self.global_path_count += 1
        if msg.poses:
            self.first_global_path_z = msg.poses[0].pose.position.z
        self.get_logger().info(
            f"Observed /global_planner/path #{self.global_path_count}: "
            f"poses={len(msg.poses)}, frame={msg.header.frame_id}, first_z={self.first_global_path_z}"
        )

    def bspline_callback(self, msg):
        self.bspline_count += 1
        first_z = msg.pos_pts[0].z if msg.pos_pts else float("nan")
        self.get_logger().info(
            f"Observed /planning/bspline #{self.bspline_count}: "
            f"traj_id={msg.traj_id}, control_points={len(msg.pos_pts)}, first_cp_z={first_z:.3f}"
        )

    def cmd_vel_callback(self, msg):
        if self.bspline_count == 0:
            return
        self.cmd_vel_count += 1
        if self.cmd_vel_count == 1:
            speed = math.hypot(msg.linear.x, msg.linear.y)
            self.get_logger().info(
                f"Observed cmd_vel: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, "
                f"wz={msg.angular.z:.3f}, speed={speed:.3f}"
            )

    def check_done(self):
        if self.global_path_count > 0 and self.bspline_count > 0 and self.cmd_vel_count > 0:
            self.get_logger().info("global_planner/path -> SCAN -> cmd_vel validation PASS")
            raise SystemExit(0)
        if time.monotonic() - self.started_at > 90.0:
            self.get_logger().error(
                "global to SCAN validation timeout: "
                f"goal_publish_count={self.goal_publish_count}, "
                f"global_path_count={self.global_path_count}, "
                f"bspline_count={self.bspline_count}, cmd_vel_count={self.cmd_vel_count}"
            )
            raise SystemExit(2)


def main():
    rclpy.init()
    node = GlobalToScanMockIO()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exc.code)


if __name__ == "__main__":
    main()
