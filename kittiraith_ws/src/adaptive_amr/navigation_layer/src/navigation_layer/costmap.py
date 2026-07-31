#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
costmap.py — costmap inflation (pure NumPy/SciPy, unit-tested).

Builds a navigation costmap from an occupancy grid by inflating obstacles:

    cost(cell) = 100 * (1 - d / R_inflate)     for d < R_inflate
                 0                              otherwise
                 -1 (unknown)                   for never-observed cells

where d = Euclidean distance to the nearest lethal cell, computed in O(n)
with the scipy Euclidean distance transform (costmap_2d's inflation layer
uses the same idea). Dynamic obstacles can be marked as lethal before
inflation so the planner avoids them too.
"""

from typing import Optional, Sequence, Tuple

import numpy as np

try:
    from scipy import ndimage
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    HAVE_SCIPY = False


class CostmapBuilder:
    """
    Inflates an occupancy grid into a costmap.

    Args:
        inflation_radius: cost decay radius [m] (0 = no inflation).
        lethal_threshold: occupancy value at which a cell is lethal.
    """

    def __init__(self, inflation_radius: float = 1.0,
                 lethal_threshold: int = 50):
        self.inflation_radius = float(inflation_radius)
        self.lethal_threshold = int(lethal_threshold)

    # ------------------------------------------------------------------ #
    def build(self, occupancy: np.ndarray, resolution: float,
              dynamic_cells: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Build the costmap.

        Args:
            occupancy: (H, W) int8 with -1 unknown, 0..100 occupancy.
            resolution: meters per cell (for the inflation radius).
            dynamic_cells: optional (M, 2) cell coordinates to mark lethal
                           (current/predicted dynamic obstacles).

        Returns:
            (H, W) int8 costmap: -1 unknown, 0 free, 1..99 inflated,
            100 lethal.
        """
        if not HAVE_SCIPY:
            raise RuntimeError("scipy is required for costmap inflation")
        occupancy = np.asarray(occupancy)
        h, w = occupancy.shape
        obstacle = occupancy >= self.lethal_threshold

        if dynamic_cells is not None and len(dynamic_cells) > 0:
            rows = np.clip(np.asarray(dynamic_cells)[:, 0].astype(int), 0, h - 1)
            cols = np.clip(np.asarray(dynamic_cells)[:, 1].astype(int), 0, w - 1)
            obstacle[rows, cols] = True

        cost = np.zeros((h, w), dtype=np.float32)
        if self.inflation_radius > 0.0 and obstacle.any():
            # distance to nearest obstacle = EDT on the complement mask
            dist = ndimage.distance_transform_edt(~obstacle)
            cost = 100.0 * np.clip(1.0 - dist / (self.inflation_radius / resolution),
                                   0.0, 1.0)
        cost[obstacle] = 100.0

        costmap = cost.astype(np.int8)
        costmap[occupancy == -1] = -1          # unknown stays unknown
        costmap[obstacle] = 100
        return costmap

    # ------------------------------------------------------------------ #
    @staticmethod
    def world_to_cell(x: float, y: float, origin_x: float, origin_y: float,
                      resolution: float) -> Tuple[int, int]:
        """World coordinates -> (row, col)."""
        col = int(np.floor((x - origin_x) / resolution))
        row = int(np.floor((y - origin_y) / resolution))
        return row, col

    @staticmethod
    def cell_to_world(row: int, col: int, origin_x: float, origin_y: float,
                      resolution: float) -> Tuple[float, float]:
        """Cell center -> world coordinates."""
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        return x, y
