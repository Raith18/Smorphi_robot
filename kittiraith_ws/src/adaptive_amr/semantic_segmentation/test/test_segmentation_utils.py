#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_segmentation_utils.py — unit tests for segmentation label/color helpers.

Run without ROS:
    python3 test_segmentation_utils.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from semantic_segmentation.segmentation_utils import (  # noqa: E402
    CLASS_COLORS, build_label_image, class_color, colorize_labels,
)


class TestColors(unittest.TestCase):
    def test_known_class(self):
        self.assertEqual((255, 0, 0), class_color("car"))

    def test_unknown_class_default(self):
        self.assertEqual((200, 200, 200), class_color("ufo"))

    def test_background_black(self):
        self.assertEqual((0, 0, 0), CLASS_COLORS["background"])


class TestLabelImage(unittest.TestCase):
    def test_single_mask(self):
        mask = np.zeros((4, 5), dtype=bool)
        mask[1:3, 1:4] = True
        label = build_label_image([mask], [2], 4, 5)
        self.assertEqual(3, label[2, 2])      # class 2 -> value 3
        self.assertEqual(0, label[0, 0])      # background

    def test_overlap_later_wins(self):
        mask_a = np.zeros((4, 4), dtype=bool)
        mask_a[:, :] = True
        mask_b = np.zeros((4, 4), dtype=bool)
        mask_b[0:2, 0:2] = True
        label = build_label_image([mask_a, mask_b], [1, 7], 4, 4)
        self.assertEqual(8, label[1, 1])      # later instance wins
        self.assertEqual(2, label[3, 3])

    def test_shape(self):
        mask = np.ones((10, 20), dtype=bool)
        label = build_label_image([mask], [0], 10, 20)
        self.assertEqual((10, 20), label.shape)
        self.assertEqual(np.uint8, label.dtype)


class TestColorize(unittest.TestCase):
    def test_colorize(self):
        label = np.zeros((4, 4), dtype=np.uint8)
        label[0:2, 0:2] = 3     # class 2 ("car")
        label[2:4, 2:4] = 1     # class 0 ("person")
        names = ["person", "x", "car"]
        colored = colorize_labels(label, names)
        np.testing.assert_array_equal([255, 0, 0], colored[1, 1])      # car blue
        np.testing.assert_array_equal([80, 80, 255], colored[3, 3])    # person
        np.testing.assert_array_equal([0, 0, 0], colored[0, 3])        # bg


if __name__ == "__main__":
    unittest.main(verbosity=2)
