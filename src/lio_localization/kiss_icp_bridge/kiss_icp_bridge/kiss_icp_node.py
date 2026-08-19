# kiss_icp_bridge/kiss_icp_bridge/kiss_icp_node.py
import numpy as np
import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from geometry_msgs.msg import PoseStamped
import tf2_ros
import os

from kiss_icp_bridge.voxels import VoxelMap


def _load_pcd_open3d(pcd_path: str) -> np.ndarray:
    """Load PCD using open3d (primary method). Raises ImportError if open3d unavailable."""
    import open3d as o3d  # noqa: F401

    if not os.path.exists(pcd_path):
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")

    pcd = o3d.io.read_point_cloud(pcd_path)
    return np.asarray(pcd.points, dtype=np.float32)


def _load_pcd_numpy_fallback(pcd_path: str) -> np.ndarray:
    """Fallback PCD loader using numpy — handles ASCII and binary PCD formats."""
    if not os.path.exists(pcd_path):
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")

    with open(pcd_path, "rb") as f:
        header_lines = []
        while True:
            line = f.readline().decode("utf-8", errors="replace")
            if not line or line.startswith("\n") or line.strip() == "":
                break
            header_lines.append(line)
            if line.startswith("DATA"):
                break

        has_ascii = any("ASCII" in hl.upper() for hl in header_lines)

        # Read past header
        f.seek(0)
        for line in header_lines:
            if line.startswith("DATA"):
                break

        if has_ascii:
            data = np.loadtxt(f)
            return data[:, :3].astype(np.float32)
        else:
            # Binary PCD: each point is float32 x, y, z
            data = np.frombuffer(f.read(), dtype=np.float32)
            n_points = len(data) // 3
            return data[: n_points * 3].reshape(-1, 3).astype(np.float32)


def load_pcd_points(pcd_path: str) -> np.ndarray:
    """Load XYZ points from a PCD file using open3d (handles binary format with header).

    Falls back to numpy-based PCD parser if open3d is unavailable.
    """
    try:
        return _load_pcd_open3d(pcd_path)
    except ImportError:
        return _load_pcd_numpy_fallback(pcd_path)


class KISSIcpBridge(Node):
    def __init__(self):
        super().__init__("kiss_icp_node")

        self.declare_parameter("map_pcd_path", "/media/lenovo/disk/planner_ws/maps/campus3.pcd")
        pcd_path = self.get_parameter("map_pcd_path").value

        # Load map points from PCD (graceful missing-file handling)
        try:
            self.map_points_ = load_pcd_points(pcd_path)
            self.get_logger().info(f"Loaded {len(self.map_points_)} map points from {pcd_path}")
        except FileNotFoundError:
            self.get_logger().error(f"Map PCD not found: {pcd_path} — will operate without prior map")
            self.map_points_ = np.empty((0, 3), dtype=np.float32)

        self.voxel_map_ = VoxelMap(voxel_size=0.5)

        # Subscriptions
        self.lidar_sub_ = self.create_subscription(
            CustomMsg,
            "/livox/lidar",
            self.on_lidar,
            10,
        )

        # Publishers
        self.pose_pub_ = self.create_publisher(
            PoseStamped,
            "/lio/localization/scan_to_map_pose",
            10,
        )

        self.get_logger().info("KISSIcpBridge node initialized")

    def on_lidar(self, msg: CustomMsg):
        # Extract XYZ from CustomMsg
        pts = np.array(
            [[p.x, p.y, p.z] for p in msg.points], dtype=np.float32
        )
        if pts.shape[0] == 0:
            return

        pose = self.voxel_map_.update(pts)

        out = PoseStamped()
        out.header.frame_id = "map"
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = float(pose[0, 3])
        out.pose.position.y = float(pose[1, 3])
        out.pose.position.z = float(pose[2, 3])
        out.pose.orientation.w = 1.0
        out.pose.orientation.x = 0.0
        out.pose.orientation.y = 0.0
        out.pose.orientation.z = 0.0

        self.pose_pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KISSIcpBridge())
    finally:
        rclpy.shutdown()
