# kiss_icp_bridge/kiss_icp_bridge/voxels.py
"""Standalone VoxelMap wrapper — no ROS dependencies, testable outside the node."""

import numpy as np
from kiss_icp.mapping import VoxelHashMap


class VoxelMap:
    """VoxelHashMap wrapper that exposes a simple update() -> 4x4 pose interface."""

    def __init__(self, voxel_size: float = 0.5):
        # KISS-ICP VoxelHashMap: voxel_size, max_distance, max_points_per_voxel
        self.vm = VoxelHashMap(
            voxel_size=voxel_size,
            max_distance=7.0,
            max_points_per_voxel=100,
        )
        self._last_pose = np.eye(4, dtype=np.float64)

    def update(self, points_xyz: np.ndarray) -> np.ndarray:
        """Add points to the map and return the last known pose (4x4 homogeneous)."""
        self.vm.update(points_xyz)
        # First frame returns identity; downstream lio_localization overrides with
        # its coarse scan-context relocation pose.
        return self._last_pose
