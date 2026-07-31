#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_detection_utils.py — unit tests for object_detection.detection_utils.

Run without ROS:
    python3 test_detection_utils.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from object_detection.detection_utils import (  # noqa: E402
    KITTI_RELEVANT, backproject_to_laser, box_area, box_center, box_iou,
    clip_box, coco_index, median_depth_in_box, normalize_label, points_in_box,
)


class TestClasses(unittest.TestCase):
    def test_coco_index(self):
        self.assertEqual(2, coco_index("car"))
        self.assertEqual(0, coco_index("person"))
        self.assertIsNone(coco_index("not_a_class"))

    def test_kitti_relevant(self):
        self.assertIn("car", KITTI_RELEVANT)
        self.assertIn("pedestrian".replace("pedestrian", "person"), KITTI_RELEVANT)

    def test_normalize_label(self):
        self.assertEqual("pedestrian", normalize_label("person"))
        self.assertEqual("car", normalize_label("car"))
        self.assertEqual("unknown_thing", normalize_label("unknown_thing"))


class TestBoxGeometry(unittest.TestCase):
    def test_area(self):
        self.assertEqual(12.0, box_area([0, 0, 3, 4]))

    def test_iou_identical(self):
        self.assertAlmostEqual(1.0, box_iou([0, 0, 10, 10], [0, 0, 10, 10]))

    def test_iou_half_overlap(self):
        # 10x10 boxes shifted by 5 -> inter 5x10=50, union 100+100-50=150
        self.assertAlmostEqual(50.0 / 150.0, box_iou([0, 0, 10, 10], [5, 0, 15, 10]))

    def test_iou_disjoint(self):
        self.assertEqual(0.0, box_iou([0, 0, 1, 1], [5, 5, 6, 6]))

    def test_center(self):
        cx, cy = box_center([10, 20, 30, 40])
        self.assertEqual(20.0, cx)
        self.assertEqual(30.0, cy)

    def test_clip(self):
        clipped = clip_box([-5, -5, 700, 500], 640, 480)
        np.testing.assert_array_equal([0.0, 0.0, 639.0, 479.0], clipped)


class TestFrustumFusion(unittest.TestCase):
    def test_points_in_box(self):
        u = np.array([1.0, 5.0, 10.0])
        v = np.array([1.0, 5.0, 10.0])
        mask = points_in_box(u, v, [2, 2, 8, 8])
        np.testing.assert_array_equal([False, True, False], mask)

    def test_median_depth_in_box(self):
        depth = np.array([1.0, 2.0, 100.0, -1.0])
        mask = np.array([True, True, True, True])
        # filters out >120 (keeps 100), <=0 (-1): median of [1,2,100] = 2
        self.assertAlmostEqual(2.0, median_depth_in_box(depth, mask, 120.0))
        # no valid points
        self.assertIsNone(median_depth_in_box(np.array([-5.0]), np.array([True]), 120.0))

    def test_backproject_identity(self):
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=float)
        T = np.eye(4)
        # pixel (320, 240) -> ray [0,0,1] -> laser point (0,0,depth)
        pos = backproject_to_laser(320.0, 240.0, 10.0, K, T)
        np.testing.assert_allclose([0.0, 0.0, 10.0], pos, atol=1e-6)

    def test_backproject_offset(self):
        K = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1]], dtype=float)
        T = np.eye(4)
        T[:3, 3] = [1.0, 2.0, 3.0]        # laser -> cam translation
        pos = backproject_to_laser(320.0, 240.0, 10.0, K, T)
        # camera point (0,0,10) -> laser (0-1, 0-2, 10-3)
        np.testing.assert_allclose([-1.0, -2.0, 7.0], pos, atol=1e-6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
