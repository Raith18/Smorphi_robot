#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_kitti_gt.py — unit tests for KITTI odometry ground-truth parsers.

Run without ROS:
    python3 test_kitti_gt.py
"""

import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                "..", "..", "dataset_loader", "src"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dataset_loader.kitti_parsers import (KittiOdometryPaths,  # noqa: E402
                                          read_odometry_poses,
                                          read_odometry_times)


class TestOdometryPoses(unittest.TestCase):
    def test_read_12_float_lines(self):
        with tempfile.NamedTemporaryFile("w", suffix=".txt",
                                         delete=False) as handle:
            handle.write("1 0 0 0 0 1 0 0 0 0 1 0\n")
            handle.write("1 0 0 1 0 1 0 0 0 0 1 0\n")
            path = handle.name
        try:
            poses = read_odometry_poses(path)
            self.assertEqual(2, len(poses))
            self.assertEqual((4, 4), poses[0].shape)
            np.testing.assert_allclose(np.eye(4), poses[0], atol=1e-9)
            self.assertAlmostEqual(1.0, poses[1][0, 3], places=9)
        finally:
            os.unlink(path)

    def test_bad_line_raises(self):
        with tempfile.NamedTemporaryFile("w", suffix=".txt",
                                         delete=False) as handle:
            handle.write("1 2 3\n")
            path = handle.name
        try:
            with self.assertRaises(ValueError):
                read_odometry_poses(path)
        finally:
            os.unlink(path)

    def test_empty_raises(self):
        with tempfile.NamedTemporaryFile("w", suffix=".txt",
                                         delete=False) as handle:
            handle.write("\n\n")
            path = handle.name
        try:
            with self.assertRaises(ValueError):
                read_odometry_poses(path)
        finally:
            os.unlink(path)


class TestOdometryTimes(unittest.TestCase):
    def test_read(self):
        with tempfile.NamedTemporaryFile("w", suffix=".txt",
                                         delete=False) as handle:
            handle.write("0.0\n0.1\n0.2\n")
            path = handle.name
        try:
            times = read_odometry_times(path)
            self.assertEqual([0.0, 0.1, 0.2], times)
        finally:
            os.unlink(path)


class TestOdometryPaths(unittest.TestCase):
    def test_layout(self):
        paths = KittiOdometryPaths("/data/kitti", "5")
        self.assertTrue(paths.sequence, "05")
        self.assertTrue(paths.poses_path.endswith("odometry/poses/05.txt"))
        self.assertTrue(paths.image_path(3, 0).endswith("image_00/000003.png"))
        self.assertTrue(paths.velodyne_path(3).endswith("velodyne/000003.bin"))
        self.assertTrue(paths.calib_path.endswith("sequences/05/calib.txt"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
