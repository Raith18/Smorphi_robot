#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_planner.py — unit tests for navigation_layer.planner (A*).

Run without ROS:
    python3 test_planner.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from navigation_layer.planner import AStarPlanner, cell_cost_factor  # noqa: E402


class TestCellCost(unittest.TestCase):
    def test_values(self):
        self.assertEqual(0.0, cell_cost_factor(0))
        self.assertEqual(0.5, cell_cost_factor(50))
        self.assertEqual(float("inf"), cell_cost_factor(100))
        self.assertEqual(0.5, cell_cost_factor(-1))
        self.assertEqual(0.2, cell_cost_factor(-1, unknown_penalty=0.2))


class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.size = 31
        self.res = 1.0
        self.origin = (-(self.size // 2), -(self.size // 2))
        self.free = np.zeros((self.size, self.size), dtype=np.int8)

    def test_straight_line(self):
        planner = AStarPlanner(self.free, self.res, *self.origin)
        path = planner.plan((0.0, 0.0), (5.0, 0.0))
        self.assertIsNotNone(path)
        np.testing.assert_allclose(path[0], [0.0, 0.0], atol=0.1)
        np.testing.assert_allclose(path[-1], [5.0, 0.0], atol=0.1)
        # path length ~ 5 (straight, no detour)
        length = np.sum(np.hypot(np.diff(path[:, 0]), np.diff(path[:, 1])))
        self.assertLess(length, 6.0)

    def test_goes_around_obstacle(self):
        grid = self.free.copy()
        # wall from (0,-2) to (0,2) — blocks the direct path
        for y in range(-2, 3):
            row, col = self._cell(0.0, y)
            grid[row, col] = 100
        planner = AStarPlanner(grid, self.res, *self.origin)
        path = planner.plan((0.0, 5.0), (0.0, -5.0))
        self.assertIsNotNone(path)
        # path must NOT cross the wall cells
        for x, y in path:
            row, col = self._cell(x, y)
            self.assertNotEqual(100, grid[row, col])

    def test_unreachable(self):
        grid = self.free.copy()
        row, col = self._cell(5.0, 0.0)
        grid[row, col] = 100
        planner = AStarPlanner(grid, self.res, *self.origin)
        self.assertIsNone(planner.plan((0.0, 0.0), (5.0, 0.0)))

    def test_goal_on_obstacle(self):
        grid = self.free.copy()
        row, col = self._cell(5.0, 0.0)
        grid[row, col] = 100
        planner = AStarPlanner(grid, self.res, *self.origin)
        self.assertIsNone(planner.plan((0.0, 0.0), (5.0, 0.0)))

    def _cell(self, x, y):
        col = int(np.floor((x - self.origin[0]) / self.res))
        row = int(np.floor((y - self.origin[1]) / self.res))
        return row, col


if __name__ == "__main__":
    unittest.main(verbosity=2)
