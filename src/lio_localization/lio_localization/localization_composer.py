"""
Compose a map-relative pose from an Elevator-LIO body odometry stream.

This node deliberately contains no scan matching, map loading, loop closure,
or estimator logic. It stores one SE(3) alignment initialized by
``/initialpose``, accepts validated map-frame pose corrections, and uses the
alignment to convert raw LIO odometry into the formal localization output.
"""

from copy import deepcopy
from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


MAP_FRAME = 'map'
ODOM_FRAME = 'odom'
BODY_FRAME = 'base_link'


def _pose_to_matrix(pose: Pose) -> np.ndarray:
    """Convert a ROS pose to a normalized quaternion SE(3) matrix."""
    translation = np.array(
        [pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    quaternion = np.array(
        [pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w], dtype=np.float64)

    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise ValueError('pose contains NaN or Inf')

    norm = np.linalg.norm(quaternion)
    if norm < 1.0e-12:
        raise ValueError('pose quaternion has zero norm')
    x, y, z, w = quaternion / norm

    rotation = np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def _inverse_se3(transform: np.ndarray) -> np.ndarray:
    """Invert an SE(3) matrix without changing its coordinate meaning."""
    inverse = np.eye(4, dtype=np.float64)
    rotation = transform[:3, :3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def _yaw_from_rotation(rotation: np.ndarray) -> float:
    """Return the ZYX yaw of a rotation matrix."""
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def _planar_map_to_odom(
        map_to_base: np.ndarray, odom_to_base: np.ndarray) -> np.ndarray:
    """Align map and odom with translation and yaw, without tilting map."""
    yaw_delta = _yaw_from_rotation(map_to_base[:3, :3]) - _yaw_from_rotation(
        odom_to_base[:3, :3])
    yaw_delta = math.atan2(math.sin(yaw_delta), math.cos(yaw_delta))
    cosine = math.cos(yaw_delta)
    sine = math.sin(yaw_delta)

    map_to_odom = np.eye(4, dtype=np.float64)
    map_to_odom[:3, :3] = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    map_to_odom[:3, 3] = (
        map_to_base[:3, 3]
        - map_to_odom[:3, :3] @ odom_to_base[:3, 3]
    )
    return map_to_odom


def _matrix_to_pose(transform: np.ndarray) -> Pose:
    """Convert an SE(3) matrix to a ROS pose with a normalized quaternion."""
    rotation = transform[:3, :3]
    trace = float(np.trace(rotation))

    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(
            max(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2], 0.0))
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
        w = (rotation[2, 1] - rotation[1, 2]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(
            max(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2], 0.0))
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
        w = (rotation[0, 2] - rotation[2, 0]) / scale
    else:
        scale = 2.0 * np.sqrt(
            max(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1], 0.0))
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
        w = (rotation[1, 0] - rotation[0, 1]) / scale

    quaternion = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if norm < 1.0e-12 or not np.all(np.isfinite(quaternion)):
        raise ValueError('SE(3) rotation produced an invalid quaternion')
    x, y, z, w = quaternion / norm

    pose = Pose()
    pose.position.x = float(transform[0, 3])
    pose.position.y = float(transform[1, 3])
    pose.position.z = float(transform[2, 3])
    pose.orientation.x = float(x)
    pose.orientation.y = float(y)
    pose.orientation.z = float(z)
    pose.orientation.w = float(w)
    return pose


class LocalizationComposer(Node):
    """Minimal initial-pose to map/odom alignment node."""

    def __init__(self) -> None:
        super().__init__('localization_composer')

        history_size = int(self.declare_parameter('raw_odom_history_size', 500).value)
        self._correction_tolerance = float(
            self.declare_parameter('correction_timestamp_tolerance', 0.15).value)
        self._max_map_to_odom_translation_jump = float(
            self.declare_parameter('max_map_to_odom_translation_jump', 0.5).value)
        self._max_map_to_odom_rotation_jump_degrees = float(
            self.declare_parameter('max_map_to_odom_rotation_jump_degrees', 15.0).value)
        self._planar_initial_alignment = bool(
            self.declare_parameter('planar_initial_alignment', False).value)
        if (history_size <= 0 or self._correction_tolerance < 0.0 or
                self._max_map_to_odom_translation_jump <= 0.0 or
                self._max_map_to_odom_rotation_jump_degrees <= 0.0):
            raise ValueError(
                'raw odom history/tolerances and map-to-odom jump limits must be positive')

        self._latest_raw_odom = None
        self._latest_raw_transform = None
        self._map_to_odom = None
        self._raw_history = deque(maxlen=history_size)
        self._raw_contract_logged = False
        self._last_published_stamp = None
        self._last_initialpose_stamp = None

        self._odom_sub = self.create_subscription(
            Odometry,
            '/lio/mapping/odom_body',
            self._on_raw_odom,
            10,
        )
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initialpose,
            10,
        )
        self._correction_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/lio/localization/pose_correction',
            self._on_pose_correction,
            10,
        )
        self._localization_pub = self.create_publisher(
            Odometry,
            '/lio/localization/odom',
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'Localization composer ready; waiting for /initialpose and raw body odometry')
        self.get_logger().info(
            'Localization geometry diagnostics: raw=odom->base_link, output=map->base_link, '
            f'tf=map->odom, correction_timestamp_tolerance={self._correction_tolerance:.3f} s, '
            f'max_map_to_odom_jump={self._max_map_to_odom_translation_jump:.3f} m/'
            f'{self._max_map_to_odom_rotation_jump_degrees:.1f} deg, '
            f'initial_alignment={"planar" if self._planar_initial_alignment else "full_se3"}')

    @staticmethod
    def _validate_raw_odom(msg: Odometry) -> bool:
        return msg.header.frame_id == ODOM_FRAME and msg.child_frame_id == BODY_FRAME

    def _on_raw_odom(self, msg: Odometry) -> None:
        if not self._validate_raw_odom(msg):
            self.get_logger().warning(
                f'Ignoring raw odometry with frames {msg.header.frame_id!r} -> '
                f'{msg.child_frame_id!r}; expected odom -> base_link',
            )
            return

        try:
            raw_transform = _pose_to_matrix(msg.pose.pose)
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring invalid raw odometry: {exc}')
            return

        self._latest_raw_odom = msg
        self._latest_raw_transform = raw_transform
        self._raw_history.append((
            self._stamp_to_nanoseconds(msg.header.stamp), msg, raw_transform))

        if not self._raw_contract_logged:
            self.get_logger().info(
                'Localization input diagnostics: '
                f'frame={msg.header.frame_id}->{msg.child_frame_id} '
                f'stamp={self._stamp_to_seconds(msg):.9f} '
                f'raw_position=({msg.pose.pose.position.x:.3f}, '
                f'{msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f})')
            self._raw_contract_logged = True

        if self._map_to_odom is not None:
            self._publish_composed(msg, raw_transform)

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        if msg.header.frame_id != MAP_FRAME:
            self.get_logger().warning(
                f'Ignoring /initialpose with frame {msg.header.frame_id!r}; expected map')
            return

        initialpose_stamp = self._stamp_to_nanoseconds(msg.header.stamp)
        if (initialpose_stamp > 0 and
                initialpose_stamp == self._last_initialpose_stamp):
            return

        if self._latest_raw_odom is None or self._latest_raw_transform is None:
            self.get_logger().warning(
                'Ignoring /initialpose: no valid /lio/mapping/odom_body has been received')
            return

        raw_odom = self._latest_raw_odom
        odom_to_base = self._latest_raw_transform
        stamp_error = 0.0
        if initialpose_stamp > 0:
            raw_stamp, raw_odom, odom_to_base = min(
                self._raw_history,
                key=lambda entry: abs(entry[0] - initialpose_stamp),
            )
            stamp_error = abs(raw_stamp - initialpose_stamp) * 1.0e-9
            if stamp_error > self._correction_tolerance:
                self.get_logger().warning(
                    'Ignoring /initialpose: nearest raw odometry differs by '
                    f'{stamp_error:.6f} s '
                    f'(limit {self._correction_tolerance:.6f} s)')
                return

        try:
            map_to_base = _pose_to_matrix(msg.pose.pose)
            if self._planar_initial_alignment:
                self._map_to_odom = _planar_map_to_odom(map_to_base, odom_to_base)
            else:
                self._map_to_odom = map_to_base @ _inverse_se3(odom_to_base)
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring invalid /initialpose: {exc}')
            return

        rotation = self._map_to_odom[:3, :3]
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        pitch = math.atan2(
            -float(rotation[2, 0]),
            math.hypot(float(rotation[2, 1]), float(rotation[2, 2])),
        )
        yaw = _yaw_from_rotation(rotation)
        if initialpose_stamp > 0:
            self._last_initialpose_stamp = initialpose_stamp
        self.get_logger().info(
            'Initialized map -> odom from /initialpose using raw odometry stamp '
            f'{self._stamp_to_seconds(raw_odom):.9f} '
            f'({stamp_error:.6f} s stamp error, '
            f'{"planar" if self._planar_initial_alignment else "full_se3"} alignment); '
            f'translation=({self._map_to_odom[0, 3]:.3f}, '
            f'{self._map_to_odom[1, 3]:.3f}, {self._map_to_odom[2, 3]:.3f}); '
            f'rpy_deg=({math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, '
            f'{math.degrees(yaw):.3f})',
        )
        self._publish_composed(self._latest_raw_odom, self._latest_raw_transform)

    def _on_pose_correction(self, msg: PoseWithCovarianceStamped) -> None:
        if self._map_to_odom is None:
            self.get_logger().warning(
                'Ignoring pose correction: localization has not been initialized')
            return
        if msg.header.frame_id != MAP_FRAME:
            self.get_logger().warning(
                f'Ignoring pose correction in {msg.header.frame_id!r}; expected map')
            return
        if not self._raw_history:
            self.get_logger().warning(
                'Ignoring pose correction: no raw odometry history is available')
            return

        correction_stamp = self._stamp_to_nanoseconds(msg.header.stamp)
        raw_stamp, _, odom_to_base = min(
            self._raw_history,
            key=lambda entry: abs(entry[0] - correction_stamp),
        )
        stamp_error = abs(raw_stamp - correction_stamp) * 1.0e-9
        if stamp_error > self._correction_tolerance:
            self.get_logger().warning(
                f'Ignoring pose correction: nearest raw odometry differs by '
                f'{stamp_error:.6f} s (limit {self._correction_tolerance:.6f} s)')
            return

        try:
            corrected_map_to_base = _pose_to_matrix(msg.pose.pose)
            if self._planar_initial_alignment:
                candidate_map_to_odom = _planar_map_to_odom(
                    corrected_map_to_base, odom_to_base)
            else:
                candidate_map_to_odom = corrected_map_to_base @ _inverse_se3(odom_to_base)
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring invalid pose correction: {exc}')
            return

        delta = _inverse_se3(self._map_to_odom) @ candidate_map_to_odom
        translation_jump = float(np.linalg.norm(delta[:3, 3]))
        cosine = float(np.clip((np.trace(delta[:3, :3]) - 1.0) * 0.5, -1.0, 1.0))
        rotation_jump_degrees = float(np.degrees(np.arccos(cosine)))
        if (translation_jump > self._max_map_to_odom_translation_jump or
                rotation_jump_degrees > self._max_map_to_odom_rotation_jump_degrees):
            self.get_logger().warning(
                'Ignoring pose correction: map -> odom jump '
                f'{translation_jump:.3f} m/{rotation_jump_degrees:.2f} deg exceeds '
                f'limit {self._max_map_to_odom_translation_jump:.3f} m/'
                f'{self._max_map_to_odom_rotation_jump_degrees:.2f} deg')
            return

        self._map_to_odom = candidate_map_to_odom
        rotation = self._map_to_odom[:3, :3]
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        pitch = math.atan2(
            -float(rotation[2, 0]),
            math.hypot(float(rotation[2, 1]), float(rotation[2, 2])),
        )
        yaw = _yaw_from_rotation(rotation)

        self.get_logger().info(
            'Updated map -> odom from fixed-map pose correction using matched '
            f'raw odometry ({stamp_error:.6f} s stamp error); '
            f'{"planar" if self._planar_initial_alignment else "full_se3"} alignment; '
            f'translation=({self._map_to_odom[0, 3]:.3f}, '
            f'{self._map_to_odom[1, 3]:.3f}, {self._map_to_odom[2, 3]:.3f}); '
            f'rpy_deg=({math.degrees(roll):.3f}, {math.degrees(pitch):.3f}, '
            f'{math.degrees(yaw):.3f})')
        if self._latest_raw_odom is not None and self._latest_raw_transform is not None:
            self._publish_composed(self._latest_raw_odom, self._latest_raw_transform)

    def _publish_composed(self, raw: Odometry, odom_to_base: np.ndarray) -> None:
        if self._map_to_odom is None:
            return

        stamp = self._stamp_to_nanoseconds(raw.header.stamp)
        if self._last_published_stamp is not None:
            if stamp < self._last_published_stamp:
                self.get_logger().warning(
                    'Ignoring non-monotonic localization output: '
                    f'last={self._last_published_stamp} current={stamp}')
                return
            if stamp == self._last_published_stamp:
                return

        map_to_base = self._map_to_odom @ odom_to_base
        try:
            pose = _matrix_to_pose(map_to_base)
        except ValueError as exc:
            self.get_logger().warning(f'Cannot publish localization pose: {exc}')
            return

        output = Odometry()
        output.header.stamp = raw.header.stamp
        output.header.frame_id = MAP_FRAME
        output.child_frame_id = BODY_FRAME
        output.pose.pose = pose
        output.pose.covariance = list(raw.pose.covariance)

        # The raw body twist is copied without rotation or reinterpretation.
        # This first-stage composer only guarantees the pose and TF contract.
        output.twist = deepcopy(raw.twist)
        self._localization_pub.publish(output)
        self._last_published_stamp = stamp
        self._publish_tf(raw.header.stamp)

    def _publish_tf(self, stamp) -> None:
        if self._map_to_odom is None:
            return

        try:
            pose = _matrix_to_pose(self._map_to_odom)
        except ValueError as exc:
            self.get_logger().warning(f'Cannot publish map -> odom TF: {exc}')
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = MAP_FRAME
        transform.child_frame_id = ODOM_FRAME
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    @staticmethod
    def _stamp_to_seconds(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1.0e-9

    @staticmethod
    def _stamp_to_nanoseconds(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationComposer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
