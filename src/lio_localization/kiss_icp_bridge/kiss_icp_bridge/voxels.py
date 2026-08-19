# kiss_icp_bridge/kiss_icp_bridge/voxels.py
"""Standalone VoxelMap wrapper — no ROS dependencies, testable outside the node."""

import numpy as np
from kiss_icp.mapping import VoxelHashMap
from kiss_icp.registration import Registration


class VoxelMap:
    """VoxelHashMap + Registration wrapper for scan-to-map ICP refinement.

    Uses kiss_icp.mapping.VoxelHashMap to accumulate map points and
    kiss_icp.registration.Registration.align_points_to_map() to compute
    the ICP-refined pose between each incoming scan and the current map.
    """

    def __init__(self, voxel_size: float = 0.5):
        self.vm = VoxelHashMap(
            voxel_size=voxel_size,
            max_distance=7.0,
            max_points_per_voxel=100,
        )
        self.reg = Registration(max_num_iterations=20, convergence_criterion=0.001)
        self._last_pose = np.eye(4, dtype=np.float64)

    def update(self, points_xyz: np.ndarray) -> np.ndarray:
        """Add points to the map and run scan-to-map ICP.

        Returns a 4x4 homogeneous transformation matrix (np.ndarray).
        Identity is returned until enough frames accumulate for ICP to converge.
        """
        self.vm.update(points_xyz)

        # Run scan-to-map ICP refinement using the current map
        # initial_guess = identity; Registration.refine() handles convergence
        try:
            pose = self.reg.align_points_to_map(
                points=points_xyz,
                voxel_map=self.vm,
                initial_guess=np.eye(4, dtype=np.float64),
                max_correspondance_distance=1.0,
                kernel=0.5,
            )
            self._last_pose = np.asarray(pose, dtype=np.float64)
        except Exception:
            # If ICP fails (e.g., insufficient overlap), keep last known pose
            pass

        return self._last_pose
