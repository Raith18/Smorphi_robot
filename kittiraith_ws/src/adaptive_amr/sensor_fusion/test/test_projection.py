#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_projection.py — unit tests for sensor_fusion.projection (pure NumPy).

Run without ROS:
    python3 test_projection.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sensor_fusion.projection import (LidarCameraProjection,  # noqa: E402
                                      combined_projection_matrix)

FX, FY, CX, CY = 500.0, 500.0, 320.0, 240.0
P = np.array([[FX, 0, CX, 0], [0, FY, CY, 0], [0, 0, 1, 0.0]])


class TestCombinedMatrix(unittest.TestCase):
    def test_identity_chain(self):
        m = combined_projection_matrix(P, np.eye(3), np.eye(4))
        np.testing.assert_allclose(m, P, atol=1e-9)

    def test_translation_included(self):
        t = np.eye(4)
        t[:3, 3] = [1.0, 2.0, 3.0]
        m = combined_projection_matrix(P, np.eye(3), t)
        # point at origin in velodyne -> camera coords (1,2,3), then to pixels:
        #   m @ p = [fx*1 + cx*3, fy*2 + cy*3, 3] = [1460, 1720, 3]
        p_h = np.array([0.0, 0.0, 0.0, 1.0])
        proj = m @ p_h
        self.assertAlmostEqual(1460.0, proj[0], places=6)
        self.assertAlmostEqual(1720.0, proj[1], places=6)
        self.assertAlmostEqual(3.0, proj[2], places=6)
        # normalized pixel coordinates
        self.assertAlmostEqual(1460.0 / 3.0, proj[0] / proj[2], places=6)
        self.assertAlmostEqual(1720.0 / 3.0, proj[1] / proj[2], places=6)


class TestProjection(unittest.TestCase):
    def setUp(self):
        self.proj = LidarCameraProjection(P, np.eye(3), np.eye(4),
                                          width=640, height=480)

    def test_center_point(self):
        # (1, 0, 10) -> u = 500*1/10 + 320 = 370, v = 240, depth 10
        u, v, depth, valid = self.proj.project(np.array([[1.0, 0.0, 10.0]]))
        self.assertAlmostEqual(370.0, u[0], places=5)
        self.assertAlmostEqual(240.0, v[0], places=5)
        self.assertAlmostEqual(10.0, depth[0], places=5)
        self.assertTrue(valid[0])

    def test_behind_camera_invalid(self):
        _, _, _, valid = self.proj.project(np.array([[0.0, 0.0, -5.0]]))
        self.assertFalse(valid[0])

    def test_negative_u(self):
        # left of the image -> u < 0, still "valid" depth-wise
        u, _, depth, valid = self.proj.project(np.array([[-10.0, 0.0, 10.0]]))
        self.assertTrue(valid[0])
        self.assertLess(u[0], 0.0)
        self.assertGreater(depth[0], 0.0)

    def test_kitti_like_extrinsic(self):
        # mimic calib_velo_to_cam translation: velodyne is behind/right/down of cam
        t = np.eye(4)
        t[:3, 3] = [-0.004, -0.076, -0.27]
        proj = LidarCameraProjection(P, np.eye(3), t, width=640, height=480)
        u, v, depth, valid = proj.project(np.array([[5.0, 0.0, 0.0]]))
        # camera coords: x=4.996, y=-0.076, z=-0.27 -> behind camera -> invalid
        self.assertFalse(valid[0])
        # a point 10 m ahead of the vehicle: camera z = 10 - t_z = 9.73
        u2, v2, depth2, valid2 = proj.project(np.array([[0.0, 0.0, 10.0]]))
        self.assertTrue(valid2[0])
        self.assertAlmostEqual(9.73, depth2[0], places=4)


class TestDepthImage(unittest.TestCase):
    def setUp(self):
        self.proj = LidarCameraProjection(P, np.eye(3), np.eye(4),
                                          width=640, height=480)

    def test_sparse_depth_values(self):
        points = np.array([[1.0, 0.0, 10.0],      # pixel (370, 240), depth 10
                           [0.0, 0.0, 20.0],      # pixel (320, 240), depth 20
                           [100.0, 0.0, 10.0]])   # outside image
        u, v, depth, valid = self.proj.project(points)
        depth_img = self.proj.build_sparse_depth(u, v, depth, valid)
        self.assertAlmostEqual(10.0, depth_img[240, 370], places=4)
        self.assertAlmostEqual(20.0, depth_img[240, 320], places=4)
        self.assertTrue(np.isnan(depth_img[0, 0]))        # unobserved
        self.assertTrue(np.isnan(depth_img[240, 500]))    # outside point dropped

    def test_colorize(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        image[240, 370] = [10, 20, 30]
        points = np.array([[1.0, 0.0, 10.0]])
        u, v, depth, valid = self.proj.project(points)
        colors = self.proj.colorize(points, image, u, v, valid)
        np.testing.assert_array_equal([10, 20, 30], colors[0])
        # out-of-image points get the default color
        points2 = np.array([[100.0, 0.0, 10.0]])
        u2, v2, d2, valid2 = self.proj.project(points2)
        colors2 = self.proj.colorize(points2, image, u2, v2, valid2, (255, 0, 0))
        np.testing.assert_array_equal([255, 0, 0], colors2[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
