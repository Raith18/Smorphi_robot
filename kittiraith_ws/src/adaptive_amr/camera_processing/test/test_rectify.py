#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_rectify.py — unit tests for camera_processing.rectify (pure NumPy).

Run without ROS:
    python3 test_rectify.py
    python3 -m unittest test_rectify
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from camera_processing.rectify import (  # noqa: E402
    RectifyMapper, adjust_intrinsics, adjust_projection_matrix,
    build_crop_resize_maps_numpy, build_rectify_maps_numpy,
    distort_point_norm, remap_numpy,
)

FX, FY, CX, CY = 500.0, 500.0, 320.0, 240.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1.0]])
D_ZERO = np.zeros(5)
R_ID = np.eye(3)
P = np.array([[FX, 0, CX, 0], [0, FY, CY, 0], [0, 0, 1, 0.0]])


class TestDistortion(unittest.TestCase):
    def test_center_undistorted(self):
        xd, yd = distort_point_norm(0.0, 0.0, 0.1, -0.2, 0.0, 0.0, 0.05)
        self.assertAlmostEqual(0.0, xd, places=9)
        self.assertAlmostEqual(0.0, yd, places=9)

    def test_radial_pulls_toward_center(self):
        # positive k1 with x=1 -> radial factor > 1 -> farther from center
        xd, _ = distort_point_norm(1.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0)
        self.assertGreater(xd, 1.0)

    def test_known_value(self):
        # hand-computed: x=0.2, y=0.1, k1=0.1, k2=0.01, k3=0.001, p1=0.01, p2=0.02
        xd, yd = distort_point_norm(0.2, 0.1, 0.1, 0.01, 0.01, 0.02, 0.001)
        r2 = 0.2 ** 2 + 0.1 ** 2
        radial = 1 + 0.1 * r2 + 0.01 * r2 ** 2 + 0.001 * r2 ** 3
        exp_x = 0.2 * radial + 2 * 0.01 * 0.2 * 0.1 + 0.02 * (r2 + 2 * 0.2 ** 2)
        exp_y = 0.1 * radial + 0.01 * (r2 + 2 * 0.1 ** 2) + 2 * 0.02 * 0.2 * 0.1
        self.assertAlmostEqual(exp_x, xd, places=9)
        self.assertAlmostEqual(exp_y, yd, places=9)


class TestProjectionAdjustment(unittest.TestCase):
    def test_crop_and_scale(self):
        new_p = adjust_projection_matrix(P, (100, 50, 0, 0), 0.5)
        self.assertAlmostEqual(FX * 0.5, new_p[0, 0], places=9)
        self.assertAlmostEqual(FY * 0.5, new_p[1, 1], places=9)
        self.assertAlmostEqual((CX - 100) * 0.5, new_p[0, 2], places=9)
        self.assertAlmostEqual((CY - 50) * 0.5, new_p[1, 2], places=9)

    def test_translation_scales(self):
        P_tx = P.copy()
        P_tx[0, 3] = -389.5575          # KITTI-style stereo baseline term
        new_p = adjust_projection_matrix(P_tx, (0, 0, 0, 0), 2.0)
        self.assertAlmostEqual(-389.5575 * 2.0, new_p[0, 3], places=6)

    def test_intrinsics_adjustment(self):
        new_k = adjust_intrinsics(K, (10, 20, 0, 0), 0.25)
        self.assertAlmostEqual((CX - 10) * 0.25, new_k[0, 2], places=9)
        self.assertAlmostEqual((CY - 20) * 0.25, new_k[1, 2], places=9)
        self.assertAlmostEqual(FX * 0.25, new_k[0, 0], places=9)


class TestMaps(unittest.TestCase):
    def test_crop_resize_maps(self):
        map_x, map_y = build_crop_resize_maps_numpy((10, 20), 2.0, (8, 6))
        self.assertEqual((6, 8), map_x.shape)
        # output (row 0, col 4) <- input (4/2 + 10, 0/2 + 20) = (12, 20)
        self.assertAlmostEqual(12.0, map_x[0, 4], places=5)
        self.assertAlmostEqual(20.0, map_y[0, 4], places=5)

    def test_undistort_identity_when_clean(self):
        # no distortion, no rectification -> maps ~ identity
        map_x, map_y = build_rectify_maps_numpy(K, D_ZERO, R_ID, P, (32, 24))
        yy, xx = np.mgrid[0:24, 0:32].astype(np.float64)
        np.testing.assert_allclose(map_x, xx, atol=1e-4)
        np.testing.assert_allclose(map_y, yy, atol=1e-4)

    def test_undistort_against_cv2(self):
        try:
            import cv2
        except ImportError:
            self.skipTest("OpenCV not installed")
        k1, k2, p1, p2, k3 = 0.05, -0.01, 0.001, 0.002, 0.0
        D = np.array([k1, k2, p1, p2, k3])
        map_x_np, map_y_np = build_rectify_maps_numpy(K, D, R_ID, P, (64, 48))
        map_x_cv, map_y_cv = cv2.initUndistortRectifyMap(
            K, D, R_ID, P, (64, 48), cv2.CV_32FC1)
        np.testing.assert_allclose(map_x_np, map_x_cv, atol=1e-3)
        np.testing.assert_allclose(map_y_np, map_y_cv, atol=1e-3)

    def test_remap_numpy(self):
        image = np.arange(6 * 8).reshape(6, 8)
        map_x, map_y = build_crop_resize_maps_numpy((0, 0), 1.0, (8, 6))
        out = remap_numpy(image, map_x, map_y)
        np.testing.assert_array_equal(out, image)


class TestRectifyMapper(unittest.TestCase):
    def test_mapper_none_mode(self):
        mapper = RectifyMapper(K, D_ZERO, R_ID, P, (640, 480),
                               mode="none", crop=(0, 0, 0, 0), scale=1.0)
        self.assertEqual((640, 480), mapper.output_size)
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[100, 200] = [255, 0, 0]
        out = mapper.apply(image)
        np.testing.assert_array_equal(out, image)

    def test_mapper_crop_scale_output_size(self):
        mapper = RectifyMapper(K, D_ZERO, R_ID, P, (640, 480),
                               mode="none", crop=(40, 30, 200, 150), scale=0.5)
        self.assertEqual((100, 75), mapper.output_size)

    def test_mapper_rejects_wrong_size(self):
        mapper = RectifyMapper(K, D_ZERO, R_ID, P, (640, 480),
                               mode="none", scale=1.0)
        with self.assertRaises(ValueError):
            mapper.apply(np.zeros((10, 10, 3), dtype=np.uint8))


if __name__ == "__main__":
    unittest.main(verbosity=2)
