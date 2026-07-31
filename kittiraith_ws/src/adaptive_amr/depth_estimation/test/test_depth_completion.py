#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_depth_completion.py — unit tests for depth_completion (pure math).

Run without ROS:
    python3 test_depth_completion.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from depth_estimation.depth_completion import (colorize_depth,  # noqa: E402
                                               nearest_fill, smooth_depth)


class TestNearestFill(unittest.TestCase):
    def test_fill_simple(self):
        sparse = np.full((5, 5), np.nan, dtype=np.float32)
        sparse[2, 2] = 5.0
        dense = nearest_fill(sparse)
        self.assertEqual(0.0, np.isnan(dense).sum())
        np.testing.assert_allclose(5.0, dense, atol=1e-6)

    def test_fill_nearest_not_mean(self):
        sparse = np.full((7, 7), np.nan, dtype=np.float32)
        sparse[3, 0] = 1.0
        sparse[3, 6] = 100.0
        dense = nearest_fill(sparse)
        # pixel (3,1) must be 1.0 (nearest), NOT 50.5 (mean)
        self.assertAlmostEqual(1.0, dense[3, 1], places=4)
        # pixel (3,5) must be 100.0
        self.assertAlmostEqual(100.0, dense[3, 5], places=4)
        # middle pixel gets either, but must be one of them
        self.assertIn(dense[3, 3], (1.0, 100.0))

    def test_max_fill_distance(self):
        sparse = np.full((9, 9), np.nan, dtype=np.float32)
        sparse[4, 4] = 3.0
        dense = nearest_fill(sparse, max_fill_distance=2)
        # within 2 px -> filled
        self.assertAlmostEqual(3.0, dense[4, 6], places=4)
        # 3 px away -> still NaN
        self.assertTrue(np.isnan(dense[4, 7]))

    def test_all_valid(self):
        sparse = np.ones((4, 4), dtype=np.float32)
        np.testing.assert_array_equal(sparse, nearest_fill(sparse))

    def test_no_valid_raises(self):
        sparse = np.full((3, 3), np.nan, dtype=np.float32)
        with self.assertRaises(ValueError):
            nearest_fill(sparse)


class TestSmooth(unittest.TestCase):
    def test_sigma_zero_identity(self):
        depth = np.ones((5, 5), dtype=np.float32)
        np.testing.assert_array_equal(depth, smooth_depth(depth, 0.0))

    def test_smoothing_reduces_variance(self):
        rng = np.random.default_rng(0)
        depth = rng.random((20, 20)).astype(np.float32)
        smoothed = smooth_depth(depth, sigma=2.0, use_cv2=False)
        self.assertLess(np.std(smoothed), np.std(depth))


class TestColorize(unittest.TestCase):
    def test_shape_and_dtype(self):
        depth = np.ones((10, 12), dtype=np.float32) * 40.0
        colored = colorize_depth(depth, max_depth=80.0)
        self.assertEqual((10, 12, 3), colored.shape)
        self.assertEqual(np.uint8, colored.dtype)

    def test_nan_black(self):
        depth = np.full((4, 4), np.nan, dtype=np.float32)
        colored = colorize_depth(depth)
        np.testing.assert_array_equal(np.zeros((4, 4, 3), dtype=np.uint8), colored)

    def test_near_far_different(self):
        near = colorize_depth(np.array([[5.0]], dtype=np.float32), max_depth=80.0)
        far = colorize_depth(np.array([[75.0]], dtype=np.float32), max_depth=80.0)
        self.assertFalse(np.array_equal(near, far))


if __name__ == "__main__":
    unittest.main(verbosity=2)
