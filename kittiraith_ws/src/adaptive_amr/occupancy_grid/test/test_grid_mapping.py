#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_grid_mapping.py — unit tests for occupancy_grid.grid_mapping.

Run without ROS:
    python3 test_grid_mapping.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from occupancy_grid.grid_mapping import (OccupancyGridMapper,  # noqa: E402
                                         bresenham)


class TestBresenham(unittest.TestCase):
    def test_horizontal_line(self):
        cells = list(bresenham(0, 0, 5, 0))
        self.assertEqual(6, len(cells))
        self.assertEqual((5, 0), cells[-1])

    def test_diagonal(self):
        cells = list(bresenham(0, 0, 3, 3))
        self.assertEqual(4, len(cells))
        self.assertEqual((3, 3), cells[-1])

    def test_reverse(self):
        cells = list(bresenham(5, 5, 2, 2))
        self.assertEqual((2, 2), cells[-1])
        self.assertEqual((5, 5), cells[0])


class TestGridMapper(unittest.TestCase):
    def test_world_cell_roundtrip(self):
        mapper = OccupancyGridMapper(resolution=0.5, width_m=10, height_m=10)
        # origin maps to the nearest cell; its center is within half a cell
        row, col = mapper.world_to_cell(0.0, 0.0)
        x, y = mapper.cell_to_world(row, col)
        self.assertLessEqual(abs(x), mapper.resolution / 2)
        self.assertLessEqual(abs(y), mapper.resolution / 2)
        # and the roundtrip world->cell->world is idempotent
        row2, col2 = mapper.world_to_cell(x, y)
        self.assertEqual((row, col), (row2, col2))

    def test_free_ray_and_occupied_endpoint(self):
        """Robot at origin looking at a wall at x=5: cells along the ray are
        free, the endpoint is occupied."""
        mapper = OccupancyGridMapper(resolution=0.5, width_m=20, height_m=20)
        mapper.add_scan(np.array([[5.0, 0.0]]), 0.0, 0.0, max_range=10.0)
        occ = mapper.get_occupancy()

        r_origin, c_origin = mapper.world_to_cell(0.0, 0.0)
        r_wall, c_wall = mapper.world_to_cell(5.0, 0.0)
        # midpoint along the ray: one free observation -> p≈0.40 -> ~40
        r_mid, c_mid = mapper.world_to_cell(2.5, 0.0)
        self.assertLess(occ[r_mid, c_mid], 50)      # more free than occupied
        self.assertGreater(occ[r_mid, c_mid], 0)
        # the wall cell: one occupied hit -> p≈0.70 -> ~70
        self.assertGreater(occ[r_wall, c_wall], 50)
        # origin cell itself: gets the free-ray update (harmless, standard)
        self.assertLess(occ[r_origin, c_origin], 50)

    def test_unknown_elsewhere(self):
        mapper = OccupancyGridMapper(resolution=1.0, width_m=10, height_m=10)
        mapper.add_scan(np.array([[4.0, 0.0]]), 0.0, 0.0, max_range=10.0)
        occ = mapper.get_occupancy()
        r_far, c_far = mapper.world_to_cell(-4.0, -4.0)   # never observed
        self.assertEqual(-1, occ[r_far, c_far])

    def test_out_of_bounds_point_ignored(self):
        mapper = OccupancyGridMapper(resolution=1.0, width_m=10, height_m=10)
        mapper.add_scan(np.array([[1000.0, 1000.0]]), 0.0, 0.0, max_range=5.0)
        occ = mapper.get_occupancy()
        # nothing was observed -> all unknown
        self.assertTrue((occ == -1).all())

    def test_incremental_evidence(self):
        """Repeated free observations push a cell's probability down."""
        mapper = OccupancyGridMapper(resolution=0.5, width_m=20, height_m=20)
        r, c = mapper.world_to_cell(3.0, 0.0)
        for _ in range(10):
            mapper.add_scan(np.array([[6.0, 0.0]]), 0.0, 0.0, max_range=10.0)
        occ = mapper.get_occupancy()
        # 10 free updates -> log-odds clamped at -3.5 -> p≈0.03 -> value ~2
        self.assertLess(occ[r, c], 10)          # consistently (very) free

    def test_max_range_limits(self):
        mapper = OccupancyGridMapper(resolution=0.5, width_m=20, height_m=20)
        mapper.add_scan(np.array([[8.0, 0.0]]), 0.0, 0.0, max_range=5.0)
        occ = mapper.get_occupancy()
        r, c = mapper.world_to_cell(8.0, 0.0)
        self.assertEqual(-1, occ[r, c])         # beyond range -> unobserved


if __name__ == "__main__":
    unittest.main(verbosity=2)
