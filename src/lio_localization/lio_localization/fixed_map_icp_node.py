"""ROS 2 boundary node for fixed-PCD KISS-ICP pose corrections."""

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from lio_localization.fixed_map_registration import FixedMapRegistration
from lio_localization.localization_composer import (
    BODY_FRAME,
    MAP_FRAME,
    ODOM_FRAME,
    _inverse_se3,
    _matrix_to_pose,
    _pose_to_matrix,
)
from lio_localization.pcd_io import load_pcd_xyz, resolve_map_path


class FixedMapIcpNode(Node):
    """Align synchronized base-frame scans to an immutable map-frame PCD."""

    def __init__(self) -> None:
        super().__init__('fixed_map_icp')

        map_path_value = self.declare_parameter('map_path', '').value
        voxel_size = float(self.declare_parameter('voxel_size', 0.20).value)
        max_correspondence_distance = float(
            self.declare_parameter('max_correspondence_distance', 1.0).value)
        max_iterations = int(self.declare_parameter('max_iterations', 100).value)
        convergence_criterion = float(
            self.declare_parameter('convergence_criterion', 1.0e-4).value)
        kernel = float(self.declare_parameter('kernel', 0.5).value)
        fitness_threshold = float(
            self.declare_parameter('fitness_threshold', 0.35).value)
        min_correspondences = int(
            self.declare_parameter('min_correspondences', 100).value)
        max_translation_jump = float(
            self.declare_parameter('max_translation_jump', 1.5).value)
        max_rotation_jump_degrees = float(
            self.declare_parameter('max_rotation_jump_degrees', 30.0).value)
        max_points_per_voxel = int(
            self.declare_parameter('max_points_per_voxel', 20).value)
        self._every_n_frames = int(
            self.declare_parameter('every_n_frames', 5).value)
        self._voxel_size = voxel_size
        sync_queue_size = int(self.declare_parameter('sync_queue_size', 30).value)
        sync_slop = float(self.declare_parameter('sync_slop', 0.10).value)
        if self._every_n_frames <= 0 or sync_queue_size <= 0 or sync_slop < 0.0:
            raise ValueError('every_n_frames/queue must be positive and sync_slop nonnegative')

        try:
            map_path = resolve_map_path(str(map_path_value))
            map_points = load_pcd_xyz(map_path)
            self._registration = FixedMapRegistration(
                map_points,
                voxel_size=voxel_size,
                max_correspondence_distance=max_correspondence_distance,
                max_iterations=max_iterations,
                convergence_criterion=convergence_criterion,
                kernel=kernel,
                fitness_threshold=fitness_threshold,
                min_correspondences=min_correspondences,
                max_translation_jump=max_translation_jump,
                max_rotation_jump_degrees=max_rotation_jump_degrees,
                max_points_per_voxel=max_points_per_voxel,
            )
        except (FileNotFoundError, ImportError, OSError, ValueError) as exc:
            self.get_logger().fatal(f'Cannot initialize fixed-map ICP: {exc}')
            raise RuntimeError(f'Cannot initialize fixed-map ICP: {exc}') from exc

        self._correction_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/lio/localization/pose_correction',
            10,
        )
        self._cloud_sub = message_filters.Subscriber(
            self, PointCloud2, '/lio/mapping/clouds_lidar',
            qos_profile=qos_profile_sensor_data)
        self._raw_odom_sub = message_filters.Subscriber(
            self, Odometry, '/lio/mapping/odom_body',
            qos_profile=qos_profile_sensor_data)
        self._predicted_odom_sub = message_filters.Subscriber(
            self, Odometry, '/lio/localization/odom',
            qos_profile=qos_profile_sensor_data)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._cloud_sub, self._raw_odom_sub, self._predicted_odom_sub],
            queue_size=sync_queue_size,
            slop=sync_slop,
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._on_inputs)
        self._frame_count = 0
        self._input_contract_logged = False

        self.get_logger().info(
            f'Loaded immutable map {map_path} with {map_points.shape[0]} finite points; '
            f'{self._registration.reference_point_count} remain in the KISS voxel map')
        self.get_logger().info(
            'Fixed-map ICP is waiting for initialized /lio/localization/odom; '
            f'registration runs every {self._every_n_frames} synchronized frames')
        self.get_logger().info(
            'Localization geometry diagnostics: map_frame=map body_frame=base_link '
            f'icp_voxel_size={self._voxel_size:.3f} m '
            f'sync_slop={sync_slop:.3f} s')

    def _on_inputs(
        self,
        cloud: PointCloud2,
        raw_odom: Odometry,
        predicted_odom: Odometry,
    ) -> None:
        self._frame_count += 1
        if self._frame_count % self._every_n_frames != 0:
            return
        if cloud.header.frame_id != BODY_FRAME:
            self.get_logger().warning(
                f'Ignoring cloud in {cloud.header.frame_id!r}; expected base_link')
            return
        if (raw_odom.header.frame_id != ODOM_FRAME or
                raw_odom.child_frame_id != BODY_FRAME):
            self.get_logger().warning(
                'Ignoring raw odometry whose frames are not odom -> base_link')
            return
        if (predicted_odom.header.frame_id != MAP_FRAME or
                predicted_odom.child_frame_id != BODY_FRAME):
            self.get_logger().warning(
                'Ignoring localization prediction whose frames are not map -> base_link')
            return

        if not self._input_contract_logged:
            cloud_stamp = self._stamp_to_seconds(cloud.header.stamp)
            raw_stamp = self._stamp_to_seconds(raw_odom.header.stamp)
            predicted_stamp = self._stamp_to_seconds(predicted_odom.header.stamp)
            self.get_logger().info(
                'ICP input diagnostics: '
                f'cloud_frame={cloud.header.frame_id} '
                f'raw_odom={raw_odom.header.frame_id}->{raw_odom.child_frame_id} '
                f'predicted_odom={predicted_odom.header.frame_id}->{predicted_odom.child_frame_id} '
                f'stamp_delta_cloud_raw={abs(cloud_stamp - raw_stamp):.6f} s '
                f'stamp_delta_predicted_raw={abs(predicted_stamp - raw_stamp):.6f} s')
            self._input_contract_logged = True

        try:
            odom_to_base = _pose_to_matrix(raw_odom.pose.pose)
            composed_map_to_base = _pose_to_matrix(predicted_odom.pose.pose)
            map_to_odom = composed_map_to_base @ _inverse_se3(odom_to_base)
            map_to_base_prediction = map_to_odom @ odom_to_base
            points = point_cloud2.read_points_numpy(
                cloud, field_names=['x', 'y', 'z'], skip_nans=True)
            points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        except (AssertionError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring invalid ICP inputs: {exc}')
            return

        result = self._registration.register(points, map_to_base_prediction)
        if not result.accepted:
            self.get_logger().warning(
                f'Rejected fixed-map ICP correction: {result.reason}')
            return

        correction = PoseWithCovarianceStamped()
        correction.header.stamp = raw_odom.header.stamp
        correction.header.frame_id = MAP_FRAME
        try:
            correction.pose.pose = _matrix_to_pose(result.transform)
        except ValueError as exc:
            self.get_logger().warning(f'Cannot publish ICP correction: {exc}')
            return

        # The KISS Python binding does not expose a covariance estimate.  Keep
        # the ROS covariance at its default rather than inventing a frame or
        # uncertainty rotation.  Acceptance is controlled by the gates above.
        self._correction_pub.publish(correction)
        self.get_logger().info(
            f'Published fixed-map ICP correction: fitness={result.fitness:.4f}, '
            f'correspondences={result.correspondence_count}, '
            f'jump={result.translation_jump:.3f} m/'
            f'{result.rotation_jump_degrees:.2f} deg')

    @staticmethod
    def _stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FixedMapIcpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
