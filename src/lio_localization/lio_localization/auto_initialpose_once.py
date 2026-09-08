"""Publish one map-frame /initialpose at the first valid raw LIO timestamp."""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AutoInitialPoseOnce(Node):
    """Bind a configured map pose to the first raw odometry sample.

    The raw LIO pose is expressed in ``odom`` and must not be relabeled as a
    map pose.  The configured map pose is therefore required explicitly.
    """

    def __init__(self) -> None:
        super().__init__('auto_initialpose_once')
        self._map_pose = self._parse_map_pose(
            str(self.declare_parameter('map_pose', '').value))
        self._repeat_count = int(
            self.declare_parameter('publish_repetitions', 5).value)
        repeat_interval = float(
            self.declare_parameter('publish_interval_sec', 0.2).value)
        if self._map_pose is None:
            raise ValueError('map_pose is required as four map-frame values: x,y,z,yaw')
        if self._repeat_count <= 0 or not math.isfinite(repeat_interval) or repeat_interval <= 0.0:
            raise ValueError('publish_repetitions and publish_interval_sec must be positive')

        self._initialpose = None
        self._publish_count = 0
        self._completed = False
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/lio/mapping/odom_body', self._on_odom, 10)
        self._publish_timer = self.create_timer(
            repeat_interval, self._publish_pending_initialpose)
        self.get_logger().info(
            'Waiting for first valid /lio/mapping/odom_body to bind configured '
            f'map pose: {self._map_pose}')

    @staticmethod
    def _parse_map_pose(value):
        if not value.strip():
            return None
        try:
            fields = [float(field.strip()) for field in value.split(',')]
        except ValueError as exc:
            raise ValueError('map_pose must be x,y,z,yaw') from exc
        if len(fields) != 4 or not all(math.isfinite(field) for field in fields):
            raise ValueError('map_pose must contain four finite values: x,y,z,yaw')
        return tuple(fields)

    def _on_odom(self, msg: Odometry) -> None:
        if self._initialpose is not None:
            return
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.get_logger().warning(
                f'Ignoring raw odom with frames {msg.header.frame_id!r} -> '
                f'{msg.child_frame_id!r}; expected odom -> base_link')
            return

        raw_values = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        if (not all(math.isfinite(value) for value in raw_values) or
                math.sqrt(sum(value * value for value in raw_values[3:])) <= 1.0e-12):
            self.get_logger().warning('Ignoring non-finite or denormalized raw odometry')
            return

        initialpose = PoseWithCovarianceStamped()
        initialpose.header.stamp = msg.header.stamp
        initialpose.header.frame_id = 'map'
        x, y, z, yaw = self._map_pose
        initialpose.pose.pose.position.x = x
        initialpose.pose.pose.position.y = y
        initialpose.pose.pose.position.z = z
        initialpose.pose.pose.orientation.z = math.sin(0.5 * yaw)
        initialpose.pose.pose.orientation.w = math.cos(0.5 * yaw)
        initialpose.pose.covariance = list(msg.pose.covariance)

        self._initialpose = initialpose
        self._publish_pending_initialpose()

    def _publish_pending_initialpose(self) -> None:
        if self._initialpose is None or self._completed:
            return

        self._initialpose_pub.publish(self._initialpose)
        self._publish_count += 1

        if self._publish_count == 1:
            stamp = self._initialpose.header.stamp
            p = self._initialpose.pose.pose.position
            q = self._initialpose.pose.pose.orientation
            self.get_logger().info(
                'Published configured map-frame /initialpose at first raw odom stamp '
                f'{stamp.sec}.{stamp.nanosec:09d}; '
                f'position=({p.x:.9f}, {p.y:.9f}, {p.z:.9f}); '
                f'quaternion=({q.x:.9f}, {q.y:.9f}, {q.z:.9f}, {q.w:.9f})')

        if self._publish_count >= self._repeat_count:
            self._completed = True
            self.get_logger().info(
                f'Automatic /initialpose delivery completed after '
                f'{self._publish_count} publications')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AutoInitialPoseOnce()
        while rclpy.ok() and not node._completed:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
