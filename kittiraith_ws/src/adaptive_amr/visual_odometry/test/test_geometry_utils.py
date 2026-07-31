#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_geometry_utils.py — unit tests for visual odometry geometry helpers.

Run without ROS:
    python3 test_geometry_utils.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from visual_odometry.geometry_utils import (  # noqa: E402
    axis_angle_from_rotation, compose_poses, invert_pose,
    matrix_to_rvec_tvec, rotation_matrix_from_axis_angle,
    rvec_tvec_to_matrix, transform_points, triangulate_dlt, triangulate_many,
)

# KITTI-like projection matrices (cam 2 / cam 3, baseline ~0.54 m apart in
# rectified cam0 coordinates).
FX, FY, CX, CY = 721.5377, 721.5377, 609.5593, 172.854
P2 = np.array([[FX, 0, CX, -3.895575e2],
               [0, FY, CY, 0],
               [0, 0, 1, 0]])
P3 = np.array([[FX, 0, CX, -7.806575e2],
               [0, FY, CY, 0],
               [0, 0, 1, 0]])


def project(P, point3d):
    """Project a 3D point with a 3x4 matrix -> (u, v)."""
    p_h = P @ np.array([point3d[0], point3d[1], point3d[2], 1.0])
    return p_h[0] / p_h[2], p_h[1] / p_h[2]


class TestTriangulation(unittest.TestCase):
    def test_known_point(self):
        point = np.array([5.0, -1.0, 10.0])
        u1, v1 = project(P2, point)
        u2, v2 = project(P3, point)
        recovered = triangulate_dlt(P2, P3, (u1, v1), (u2, v2))
        np.testing.assert_allclose(point, recovered, atol=1e-6)

    def test_many_matches_single(self):
        point = np.array([2.0, 0.5, 15.0])
        u1, v1 = project(P2, point)
        u2, v2 = project(P3, point)
        pts = triangulate_many(P2, P3,
                               np.array([[u1, v1]]), np.array([[u2, v2]]))
        np.testing.assert_allclose(point, pts[0], atol=1e-6)

    def test_disparity_increases_with_depth(self):
        """Nearer points must have larger disparity (u1 - u2)."""
        near = np.array([0.0, 0.0, 5.0])
        far = np.array([0.0, 0.0, 30.0])
        d_near = project(P2, near)[0] - project(P3, near)[0]
        d_far = project(P2, far)[0] - project(P3, far)[0]
        self.assertGreater(d_near, d_far)
        self.assertGreater(d_near, 0.0)


class TestPoseConversions(unittest.TestCase):
    def test_rvec_roundtrip(self):
        rvec = np.array([0.3, -0.2, 0.7])
        tvec = np.array([1.0, 2.0, 3.0])
        T = rvec_tvec_to_matrix(rvec, tvec)
        r2, t2 = matrix_to_rvec_tvec(T)
        np.testing.assert_allclose(tvec, t2, atol=1e-9)
        # rotation vectors can differ by 2pi — compare matrices instead
        T2 = rvec_tvec_to_matrix(r2, t2)
        np.testing.assert_allclose(T, T2, atol=1e-9)

    def test_pure_numpy_rodrigues_matches_cv2(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("cv2 not installed")
        rvec = np.array([0.5, -0.1, 0.2])
        r_cv, _ = cv2.Rodrigues(rvec)
        r_np = rotation_matrix_from_axis_angle(rvec)
        np.testing.assert_allclose(r_cv, r_np, atol=1e-9)

    def test_axis_angle_roundtrip(self):
        rvec = np.array([0.1, 0.4, -0.3])
        R = rotation_matrix_from_axis_angle(rvec)
        r2 = axis_angle_from_rotation(R)
        R2 = rotation_matrix_from_axis_angle(r2)
        np.testing.assert_allclose(R, R2, atol=1e-9)


class TestPoseHelpers(unittest.TestCase):
    def test_invert_compose_roundtrip(self):
        T = rvec_tvec_to_matrix(np.array([0.2, -0.1, 0.5]),
                                np.array([1.0, 2.0, 3.0]))
        np.testing.assert_allclose(np.eye(4),
                                   compose_poses(T, invert_pose(T)), atol=1e-9)

    def test_transform_points(self):
        T = np.eye(4)
        T[:3, 3] = [10, 20, 30]
        pts = np.array([[0, 0, 0], [1, 1, 1]], dtype=float)
        out = transform_points(pts, T)
        np.testing.assert_allclose([[10, 20, 30], [11, 21, 31]], out, atol=1e-9)


if __name__ == "__main__":
    unittest.main(verbosity=2)
