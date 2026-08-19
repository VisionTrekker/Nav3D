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
