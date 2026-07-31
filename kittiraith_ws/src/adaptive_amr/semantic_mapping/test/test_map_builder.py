#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_map_builder.py — unit tests for semantic_mapping.map_builder.

Run without ROS:
    python3 test_map_builder.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from semantic_mapping.map_builder import SemanticMapBuilder, transform_points  # noqa: E402


class TestMapBuilder(unittest.TestCase):
    def test_identity_pose_adds_points(self):
        builder = SemanticMapBuilder(voxel_size=0.5)
        xyz = np.array([[0.1, 0.1, 0.1], [1.0, 1.0, 1.0]], dtype=np.float32)
        rgb = np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8)
        added = builder.add_frame(xyz, rgb, np.eye(4))
        self.assertEqual(2, added)
        map_xyz, map_rgb = builder.get_map()
        self.assertEqual(2, map_xyz.shape[0])
        self.assertIn(255, map_rgb[:, 0])      # red present

    def test_translation_pose(self):
        builder = SemanticMapBuilder(voxel_size=0.5)
        xyz = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
        T = np.eye(4)
        T[0, 3] = 10.0
        builder.add_frame(xyz, np.array([[1, 2, 3]], np.uint8), T)
        map_xyz, _ = builder.get_map()
        self.assertAlmostEqual(10.0, map_xyz[0, 0], places=1)   # moved by T

    def test_voxel_merging(self):
        builder = SemanticMapBuilder(voxel_size=1.0)
        # two points in the same voxel -> one merged voxel
        xyz = np.array([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]], dtype=np.float32)
        rgb = np.array([[10, 20, 30], [10, 20, 30]], dtype=np.uint8)
        builder.add_frame(xyz, rgb, np.eye(4))
        map_xyz, _ = builder.get_map()
        self.assertEqual(1, map_xyz.shape[0])

    def test_dominant_color(self):
        builder = SemanticMapBuilder(voxel_size=1.0)
        xyz = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0],
                        [0.2, 0.0, 0.0]], dtype=np.float32)
        # two red, one blue -> voxel must end up red
        rgb = np.array([[255, 0, 0], [255, 0, 0], [0, 0, 255]], dtype=np.uint8)
        builder.add_frame(xyz, rgb, np.eye(4))
        _, map_rgb = builder.get_map()
        np.testing.assert_array_equal([255, 0, 0], map_rgb[0])

    def test_transform_points(self):
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        out = transform_points(np.array([[0, 0, 0]]), T)
        np.testing.assert_allclose([[1, 2, 3]], out, atol=1e-9)

    def test_clear(self):
        builder = SemanticMapBuilder(voxel_size=0.5)
        builder.add_frame(np.zeros((3, 3), np.float32),
                          np.zeros((3, 3), np.uint8), np.eye(4))
        builder.clear()
        self.assertEqual(0, builder.get_map()[0].shape[0])


if __name__ == "__main__":
    unittest.main(verbosity=2)
