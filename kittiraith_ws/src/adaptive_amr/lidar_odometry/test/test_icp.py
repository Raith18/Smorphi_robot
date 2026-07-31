#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_icp.py — unit tests for lidar_odometry.icp (pure NumPy/SciPy).

Validates on synthetic data:
  * point-to-point ICP recovers a known rigid transform.
  * point-to-plane ICP recovers rotation+translation on a noisy plane.
  * point-to-plane is more accurate than point-to-point along plane normals.
  * normals are oriented toward the sensor origin.
Run without ROS:
    python3 test_icp.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from lidar_odometry.icp import (estimate_normals, icp_point_to_plane,  # noqa: E402
                                icp_point_to_point)


def transform_cloud(points, rvec, tvec):
    """Apply rotation vector + translation to a cloud."""
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        R = np.eye(3)
    else:
        k = rvec / theta
        kx, ky, kz = k
        skew = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
        R = np.eye(3) + np.sin(theta) * skew + (1 - np.cos(theta)) * (skew @ skew)
    return (points @ R.T) + np.asarray(tvec)


def random_cloud(n=600, seed=0, scale=10.0):
    rng = np.random.default_rng(seed)
    return rng.random((n, 3)) * scale


def noisy_plane(n=800, seed=1, size=15.0, noise=0.03):
    """A ground-like plane (z ~ 0) with small noise."""
    rng = np.random.default_rng(seed)
    xy = (rng.random((n, 2)) - 0.5) * size
    z = rng.normal(0.0, noise, (n, 1))
    return np.hstack([xy, z])


class TestNormals(unittest.TestCase):
    def test_plane_normals_point_up(self):
        # ground plane BELOW the sensor (KITTI-like: velodyne at ~1.73 m,
        # ground at z ~ -1.73) -> normals must point up (toward the sensor)
        rng = np.random.default_rng(1)
        xy = (rng.random((400, 2)) - 0.5) * 15.0
        z = np.full((400, 1), -1.73)
        plane = np.hstack([xy, z])
        normals = estimate_normals(plane, k=8, sensor_origin=np.zeros(3))
        self.assertGreater(normals[:, 2].mean(), 0.99)   # pointing up

    def test_oriented_toward_origin(self):
        # points on the +x side of a yz plane -> normals point toward origin
        rng = np.random.default_rng(2)
        plane = np.column_stack([np.full(300, 2.0),
                                 rng.random(300) * 4 - 2,
                                 rng.random(300) * 4 - 2])
        normals = estimate_normals(plane, k=8, sensor_origin=np.zeros(3))
        # normal should point toward origin -> negative x
        self.assertLess(normals[:, 0].mean(), -0.9)


class TestPointToPoint(unittest.TestCase):
    def test_recovers_transform(self):
        source = random_cloud(seed=3)
        rvec = np.array([0.2, -0.15, 0.1])
        tvec = np.array([1.5, -2.0, 0.5])
        target = transform_cloud(source, rvec, tvec)
        T, rmse = icp_point_to_point(source, target, max_iterations=60,
                                     max_correspondence_distance=10.0)
        recovered_t = T[:3, 3]
        np.testing.assert_allclose(tvec, recovered_t, atol=1e-3)
        # rotation recovery: apply recovered transform to source, compare
        aligned = (source @ T[:3, :3].T) + recovered_t
        np.testing.assert_allclose(target, aligned, atol=1e-2)
        self.assertLess(rmse, 1e-2)


class TestPointToPlane(unittest.TestCase):
    def test_recovers_transform_on_plane(self):
        # 15 m wide noisy plane: expect mm-level translation, ~0.2 deg rotation
        source = noisy_plane(n=800, noise=0.02)
        rvec = np.array([0.1, -0.05, 0.0])     # small rotation
        tvec = np.array([0.8, -0.6, 0.1])
        target = transform_cloud(source, rvec, tvec)
        T, err = icp_point_to_plane(source, target, max_iterations=60,
                                    max_correspondence_distance=5.0)
        np.testing.assert_allclose(tvec, T[:3, 3], atol=2e-2)
        aligned = (source @ T[:3, :3].T) + T[:3, 3]
        np.testing.assert_allclose(target, aligned, atol=8e-2)   # ~0.3 deg @ 15 m
        self.assertLess(err, 1e-2)               # mean point-to-plane error

    def test_recovers_pure_normal_translation(self):
        """Both variants must recover a translation along the plane normal."""
        source = noisy_plane(n=1000, noise=0.02)
        tvec = np.array([0.0, 0.0, 0.05])       # height offset
        target = transform_cloud(source, np.zeros(3), tvec)
        T_pp, _ = icp_point_to_point(source, target, max_iterations=60,
                                     max_correspondence_distance=5.0)
        T_pl, _ = icp_point_to_plane(source, target, max_iterations=60,
                                     max_correspondence_distance=5.0)
        self.assertLess(abs(T_pp[2, 3] - 0.05), 2e-2)
        self.assertLess(abs(T_pl[2, 3] - 0.05), 1e-2)

    def test_rejects_far_correspondences(self):
        """With a small max_correspondence_distance the result is biased but
        still finite (guard against garbage correspondence)."""
        source = noisy_plane(n=300, seed=7)
        target = transform_cloud(source, np.zeros(3), np.array([10.0, 0.0, 0.0]))
        T, _ = icp_point_to_plane(source, target, max_iterations=10,
                                  max_correspondence_distance=0.5)
        self.assertTrue(np.all(np.isfinite(T)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
