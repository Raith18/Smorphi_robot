#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_lidar_processing.py — unit tests for the pure LiDAR algorithms.

Run without ROS:
    python3 test_lidar_processing.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from lidar_processing.clustering import (AxisAlignedBoundingBox,  # noqa: E402
                                         EuclideanClusterExtraction,
                                         OrientedBoundingBox)
from lidar_processing.filters import PassthroughFilter, VoxelGrid  # noqa: E402
from lidar_processing.ground_segmentation import (GroundRemover,  # noqa: E402
                                                  RansacPlaneSegmenter,
                                                  fit_plane_svd)


def make_ground_plane(n=3000, size=30.0, z=0.0, noise=0.02, seed=42):
    """Uniform ground plane at height z with small noise."""
    rng = np.random.default_rng(seed)
    xy = (rng.random((n, 2)) - 0.5) * size
    zs = np.full((n, 1), z) + rng.normal(0.0, noise, (n, 1))
    return np.hstack([xy, zs]).astype(np.float32)


def make_sphere(center, radius, n=300, seed=7):
    """Random points on the surface of a sphere."""
    rng = np.random.default_rng(seed)
    dirs = rng.normal(size=(n, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    return (center + radius * dirs).astype(np.float32)


class TestPassthrough(unittest.TestCase):
    def test_axis_limits(self):
        points = np.array([[1.0, 0, 0], [-1.0, 0, 0], [0, 5, 0], [0, 0, 2]])
        filt = PassthroughFilter({"x": (-0.5, 0.5), "z": (0, 1)})
        out = filt.filter(points)
        self.assertEqual(1, out.shape[0])
        np.testing.assert_array_equal([[0, 5, 0]], out)

    def test_range_limits(self):
        points = np.array([[0.1, 0, 0], [2.0, 0, 0], [0, 5, 0]])
        filt = PassthroughFilter(min_range=0.5, max_range=3.0)
        out = filt.filter(points)
        self.assertEqual(1, out.shape[0])
        np.testing.assert_array_equal([[2.0, 0, 0]], out)


class TestVoxelGrid(unittest.TestCase):
    def test_downsample_reduces_points(self):
        rng = np.random.default_rng(0)
        points = rng.random((5000, 3)) * 2.0  # 2 m cube
        voxel = VoxelGrid(leaf_size=1.0)
        centroids, _ = voxel.downsample(points)
        self.assertLessEqual(centroids.shape[0], 8)   # 2x2x2 voxels max
        self.assertEqual(3, centroids.shape[1])

    def test_invalid_leaf(self):
        with self.assertRaises(ValueError):
            VoxelGrid(leaf_size=0.0)

    def test_feature_averaging(self):
        points = np.zeros((10, 3), dtype=np.float32)
        features = np.ones((10, 1), dtype=np.float32) * 5.0
        voxel = VoxelGrid(0.5)
        centroids, feats = voxel.downsample(points, features)
        self.assertEqual(1, centroids.shape[0])
        self.assertAlmostEqual(5.0, feats[0, 0], places=5)


class TestRansac(unittest.TestCase):
    def test_find_flat_plane(self):
        plane = make_ground_plane(n=2000, z=0.0, noise=0.01)
        obstacles = make_sphere(np.array([2.0, 1.0, 0.8]), 0.5, n=200)
        points = np.vstack([plane, obstacles])
        normal, d, _ = RansacPlaneSegmenter(0.05, 60, seed=3).fit(points)
        # normal should point up
        self.assertGreater(normal[2], 0.95)
        self.assertAlmostEqual(d, 0.0, places=1)

    def test_svd_plane(self):
        points = make_ground_plane(n=500, z=1.5, noise=0.0)
        normal, d = fit_plane_svd(points)
        self.assertGreater(normal[2], 0.99)
        self.assertAlmostEqual(d, -1.5, places=4)

    def test_ground_remover(self):
        plane = make_ground_plane(n=2000, z=0.0, noise=0.01)
        car = make_sphere(np.array([3.0, 0.0, 0.8]), 0.7, n=400, seed=5)
        points = np.vstack([plane, car])
        remover = GroundRemover(RansacPlaneSegmenter(0.05, 60, seed=3))
        ground, obstacles, _ = remover.separate(points)
        # all ground-plane points should stay ground
        self.assertGreaterEqual(ground.shape[0], 1900)
        # most car points should be obstacles
        self.assertGreaterEqual(obstacles.shape[0], 300)


class TestClustering(unittest.TestCase):
    def test_two_clusters(self):
        blob_a = make_sphere(np.array([0.0, 0.0, 0.0]), 0.3, n=100, seed=1)
        blob_b = make_sphere(np.array([5.0, 0.0, 0.0]), 0.3, n=100, seed=2)
        points = np.vstack([blob_a, blob_b])
        clusterer = EuclideanClusterExtraction(tolerance=0.6, min_cluster_size=10)
        clusters = clusterer.extract(points)
        self.assertEqual(2, len(clusters))
        sizes = sorted(len(c) for c in clusters)
        self.assertTrue(all(s >= 90 for s in sizes))

    def test_min_cluster_size(self):
        blob_a = make_sphere(np.array([0.0, 0.0, 0.0]), 0.3, n=100, seed=1)
        blob_b = make_sphere(np.array([5.0, 0.0, 0.0]), 0.3, n=20, seed=2)
        points = np.vstack([blob_a, blob_b])
        clusterer = EuclideanClusterExtraction(tolerance=0.6, min_cluster_size=50)
        clusters = clusterer.extract(points)
        self.assertEqual(1, len(clusters))
        self.assertGreaterEqual(len(clusters[0]), 90)


class TestBoundingBoxes(unittest.TestCase):
    def test_axis_aligned(self):
        rng = np.random.default_rng(0)
        points = rng.uniform(low=[1, 2, 3], high=[3, 4, 5], size=(500, 3))
        box = AxisAlignedBoundingBox(points)
        np.testing.assert_allclose(box.min, [1, 2, 3], atol=0.05)
        np.testing.assert_allclose(box.max, [3, 4, 5], atol=0.05)

    def test_oriented_box_dims(self):
        # box aligned with axes -> PCA OBB recovers the dimensions
        rng = np.random.default_rng(1)
        points = rng.uniform(low=[0, 0, 0], high=[2, 1, 0.5], size=(800, 3))
        box = OrientedBoundingBox(points)
        np.testing.assert_allclose(sorted(box.extents), [0.5, 1.0, 2.0], atol=0.1)
        # axes orthonormal
        np.testing.assert_allclose(box.axes.T @ box.axes, np.eye(3), atol=1e-6)

    def test_oriented_box_rotated(self):
        # rotate a 2x1x0.5 box by 30 deg around z -> extents preserved
        # (a cube would have degenerate covariance, so PCA axes are ambiguous;
        #  a box with distinct dimensions recovers the true axes reliably)
        theta = np.pi / 6
        rot = np.array([[np.cos(theta), -np.sin(theta), 0],
                        [np.sin(theta), np.cos(theta), 0],
                        [0, 0, 1.0]])
        rng = np.random.default_rng(2)
        box_points = rng.uniform(low=[0, 0, 0], high=[2, 1, 0.5], size=(600, 3))
        rotated = box_points @ rot.T
        box = OrientedBoundingBox(rotated)
        np.testing.assert_allclose(sorted(box.extents), [0.5, 1.0, 2.0], atol=0.12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
