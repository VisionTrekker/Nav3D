#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scan_planner_msgs.msg import Bspline
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from geometry_msgs.msg import Twist


class ScanMode3MockIO(Node):
    def __init__(self):
        super().__init__("scan_mode3_mock_io")
        self.frame_id = self.declare_parameter("frame_id", "world").value
        self.start = (
            self.declare_parameter("start_x", -5.5).value,
            self.declare_parameter("start_y", 5.5).value,
            self.declare_parameter("start_z", 0.5).value,
        )
        self.path_points = [
            (-5.5, 5.5, 0.10),
            (-5.0, 4.5, 0.10),
            (-4.5, 3.5, 0.10),
            (-4.0, 2.5, 0.10),
        ]
        self.path_sent = False
        self.bspline_count = 0
        self.cmd_vel_count = 0

        self.body_pose_pub = self.create_publisher(Odometry, "body_pose", 10)
        self.sensor_pose_pub = self.create_publisher(Odometry, "sensor_pose", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "cloud", 10)
        self.path_pub = self.create_publisher(Path, "initial_path", 10)

        self.create_subscription(Bspline, "planning/bspline", self.bspline_callback, 10)
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        self.publish_timer = self.create_timer(0.05, self.publish_mock_state)
        self.path_timer = self.create_timer(0.5, self.maybe_publish_path)
        self.stop_timer = self.create_timer(0.5, self.check_done)
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
        self.body_pose_pub.publish(odom)
        self.sensor_pose_pub.publish(odom)

        cloud = point_cloud2.create_cloud_xyz32(self.now_header(), [])
        self.cloud_pub.publish(cloud)

    def maybe_publish_path(self):
        if self.path_sent:
            return
        if self.path_pub.get_subscription_count() < 1:
            return
        if time.monotonic() - self.started_at < 2.0:
            return

        path = Path()
        path.header = self.now_header()
        for x, y, z in self.path_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.path_sent = True
        self.get_logger().info(
            f"Published mock initial_path with {len(path.poses)} points; "
            "z is route/ground height before SCAN adds body_height"
        )

    def bspline_callback(self, msg):
        self.bspline_count += 1
        self.get_logger().info(
            f"Observed planning/bspline #{self.bspline_count}: "
            f"traj_id={msg.traj_id}, control_points={len(msg.pos_pts)}"
        )

    def cmd_vel_callback(self, msg):
        self.cmd_vel_count += 1
        if self.cmd_vel_count == 1:
            speed = math.hypot(msg.linear.x, msg.linear.y)
            self.get_logger().info(
                f"Observed cmd_vel: vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, "
                f"wz={msg.angular.z:.3f}, speed={speed:.3f}"
            )

    def check_done(self):
        if self.bspline_count > 0 and self.cmd_vel_count > 0:
            self.get_logger().info("SCAN Mode 3 mock validation PASS")
            raise SystemExit(0)
        if time.monotonic() - self.started_at > 20.0:
            self.get_logger().error(
                f"SCAN Mode 3 mock validation timeout: "
                f"path_sent={self.path_sent}, bspline_count={self.bspline_count}, "
                f"cmd_vel_count={self.cmd_vel_count}"
            )
            raise SystemExit(2)


def main():
    rclpy.init()
    node = ScanMode3MockIO()
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exc.code)


if __name__ == "__main__":
    main()
