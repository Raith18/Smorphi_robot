#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_costmap.py — unit tests for navigation_layer.costmap.

Run without ROS:
    python3 test_costmap.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from navigation_layer.costmap import CostmapBuilder  # noqa: E402


def grid_with_wall(size=21):
    """Empty grid with a wall at column 10 (rows 0..size-1)."""
    grid = np.zeros((size, size), dtype=np.int8)
    grid[:, 10] = 100
    return grid


class TestCostmapBuilder(unittest.TestCase):
    def test_obstacle_is_lethal(self):
        builder = CostmapBuilder(inflation_radius=0.0)
        cost = builder.build(grid_with_wall(), 0.2)
        self.assertEqual(100, cost[10, 10])      # the wall

    def test_free_area_is_zero(self):
        builder = CostmapBuilder(inflation_radius=0.0)
        cost = builder.build(grid_with_wall(), 0.2)
        self.assertEqual(0, cost[0, 0])

    def test_inflation_decays(self):
        builder = CostmapBuilder(inflation_radius=1.0)
        cost = builder.build(grid_with_wall(), 0.5)    # 1.0 m / 0.5 = 2 cells
        # adjacent to the wall -> inflated > 0
        self.assertGreater(cost[10, 9], 0)
        # 3+ cells away -> free
        self.assertEqual(0, cost[10, 6])

    def test_unknown_preserved(self):
        grid = np.full((10, 10), -1, dtype=np.int8)
        grid[5, 5] = 100
        builder = CostmapBuilder(inflation_radius=2.0)
        cost = builder.build(grid, 0.5)
        self.assertEqual(-1, cost[0, 0])        # unknown stays unknown
        self.assertEqual(100, cost[5, 5])

    def test_dynamic_cells_marked(self):
        builder = CostmapBuilder(inflation_radius=0.0)
        grid = np.zeros((10, 10), dtype=np.int8)
        cost = builder.build(grid, 0.5, dynamic_cells=np.array([[3, 3]]))
        self.assertEqual(100, cost[3, 3])

    def test_world_cell_roundtrip(self):
        row, col = CostmapBuilder.world_to_cell(5.0, -3.0, 0.0, 0.0, 0.5)
        x, y = CostmapBuilder.cell_to_world(row, col, 0.0, 0.0, 0.5)
        # cell centers are within half a resolution of the world point
        self.assertLessEqual(abs(x - 5.0), 0.25)
        self.assertLessEqual(abs(y + 3.0), 0.25)


if __name__ == "__main__":
    unittest.main(verbosity=2)
