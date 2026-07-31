#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_localizer.py — unit tests for localization.icp_localizer.

Run without ROS:
    python3 test_localizer.py
"""

import os
import sys
import unittest

import numpy as np

_AMR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("localization", "lidar_odometry", "lidar_processing",
             "dataset_loader"):
    _p = os.path.join(_AMR, _pkg, "src")
    if _p not in sys.path:
        sys.path.insert(0, _p)

from localization.icp_localizer import IcpLocalizer, transform_points  # noqa: E402


def make_scene(seed=0, n=3000, size=30.0):
    """A box-like scene with walls + floor (points on surfaces)."""
    rng = np.random.default_rng(seed)

    def wall(axis, value, lo, hi):
        pts = np.zeros((n // 5, 3))
        pts[:, axis] = value
        other = [0, 1, 2]
        other.remove(axis)
        pts[:, other[0]] = rng.uniform(lo, hi, n // 5)
        pts[:, other[1]] = rng.uniform(lo, hi, n // 5)
        return pts

    floor = np.column_stack([
        rng.uniform(-size, size, n),
        rng.uniform(-size, size, n),
        rng.uniform(-1.8, -1.6, n)])
    walls = np.vstack([
        wall(0, -size, -1.8, 0.0),
        wall(0, size, -1.8, 0.0),
        wall(1, -size, -1.8, 0.0),
        wall(1, size, -1.8, 0.0),
    ])
    return np.vstack([floor, walls]).astype(np.float32)


class TestLocalizer(unittest.TestCase):
    def test_mapping_phase_transitions(self):
        localizer = IcpLocalizer(voxel_size=0.5, min_map_points=2000,
                                 max_map_points=100_000, max_mapping_frames=10)
        scene = make_scene()
        for i in range(15):
            T = np.eye(4)
            T[0, 3] = i * 0.5                      # slowly moving robot
            localizer.add_scan_to_map(scene, T)
        self.assertFalse(localizer.mapping)         # ended (frames or size)
        self.assertGreaterEqual(localizer.map_points.shape[0], 2000)

    def test_localize_recovers_offset(self):
        """The localizer must correct a deliberately wrong odometry pose."""
        scene = make_scene(seed=1)
        localizer = IcpLocalizer(voxel_size=0.5, min_map_points=2000,
                                 max_map_points=100_000, max_mapping_frames=5)
        # mapping: robot at identity
        for _ in range(5):
            localizer.add_scan_to_map(scene, np.eye(4))
        self.assertTrue(localizer.map_ready)

        # ground truth pose: robot 1.2 m in x (so the scene, fixed in the
        # world, is seen at -1.2 in the robot's base frame)
        T_truth = np.eye(4)
        T_truth[0, 3] = 1.2
        # odometry prior: wrong by 0.5 m
        T_odom = np.eye(4)
        T_odom[0, 3] = 0.7

        scan_at_truth = transform_points(scene, np.linalg.inv(T_truth))
        T_corr, T_map_base, error = localizer.localize(scan_at_truth, T_odom)
        # refined pose should recover the ground truth (within voxel size)
        self.assertLess(abs(T_map_base[0, 3] - 1.2), 0.5)
        # correction should compensate the 0.5 m odometry error
        self.assertLess(abs(T_corr[0, 3] - 0.5), 0.5)

    def test_rejects_implausible_jump(self):
        localizer = IcpLocalizer(voxel_size=0.5, min_map_points=2000,
                                 max_map_points=100_000, max_mapping_frames=5,
                                 max_translation_jump=0.5)
        scene = make_scene(seed=2)
        for _ in range(5):
            localizer.add_scan_to_map(scene, np.eye(4))

        T_truth = np.eye(4)
        T_truth[0, 3] = 10.0                        # way outside the map
        scan = transform_points(scene, T_truth)
        T_corr, _, _ = localizer.localize(scan, np.eye(4))
        # the jump guard should keep the correction bounded (no divergence)
        self.assertLess(np.linalg.norm(T_corr[:3, 3]), 5.0)

    def test_reset_restarts_mapping(self):
        localizer = IcpLocalizer(voxel_size=0.5, min_map_points=2000,
                                 max_mapping_frames=3)
        scene = make_scene(seed=3)
        for _ in range(5):
            localizer.add_scan_to_map(scene, np.eye(4))
        self.assertFalse(localizer.mapping)
        localizer.reset()
        self.assertTrue(localizer.mapping)
        self.assertEqual(0, localizer.map_points.shape[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
