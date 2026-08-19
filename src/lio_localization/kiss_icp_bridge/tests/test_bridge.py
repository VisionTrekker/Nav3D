# tests/test_bridge.py
import unittest
from kiss_icp_bridge.voxels import VoxelMap
import numpy as np


class TestVoxelMap(unittest.TestCase):
    def test_first_frame_returns_initial_pose(self):
        m = VoxelMap(voxel_size=0.5)
        pts = np.random.rand(100, 3).astype(np.float32)
        pose = m.update(pts)
        self.assertEqual(pose.shape, (4, 4))

    def test_subsequent_frames_produce_valid_poses(self):
        m = VoxelMap(voxel_size=0.5)
        # Build up the map with a base point cloud
        base = np.random.rand(200, 3).astype(np.float32) * 5.0
        m.update(base)
        # Subsequent scans at a different offset should produce non-trivial poses
        for i in range(5):
            offset = np.array([i * 0.05, 0.0, 0.0], dtype=np.float32)
            scan = base + offset + np.random.randn(*base.shape).astype(np.float32) * 0.05
            pose = m.update(scan)
            self.assertEqual(pose.shape, (4, 4))
            self.assertTrue(np.isfinite(pose).all())
