"""Publish one synchronized /initialpose from the first valid raw LIO odom."""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AutoInitialPoseOnce(Node):
    """Test helper for binding /initialpose to the current raw odometry stamp."""

    def __init__(self) -> None:
        super().__init__('auto_initialpose_once')
        self._published = False
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/lio/mapping/odom_body', self._on_odom, 10)
        self.get_logger().info(
            'Waiting for first valid /lio/mapping/odom_body to publish /initialpose once')

    def _on_odom(self, msg: Odometry) -> None:
        if self._published:
            return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().warning(
                f'Ignoring raw odom with frames {msg.header.frame_id!r} -> '
                f'{msg.child_frame_id!r}; expected odom -> base_link')
            return

        initialpose = PoseWithCovarianceStamped()
        initialpose.header.stamp = msg.header.stamp
        initialpose.header.frame_id = 'map'
        initialpose.pose.pose = msg.pose.pose
        initialpose.pose.covariance = list(msg.pose.covariance)

        self._initialpose_pub.publish(initialpose)
        self._published = True

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.get_logger().info(
            'Published /initialpose from raw odom stamp '
            f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}; '
            f'position=({p.x:.9f}, {p.y:.9f}, {p.z:.9f}); '
            f'quaternion=({q.x:.9f}, {q.y:.9f}, {q.z:.9f}, {q.w:.9f})')
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoInitialPoseOnce()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
