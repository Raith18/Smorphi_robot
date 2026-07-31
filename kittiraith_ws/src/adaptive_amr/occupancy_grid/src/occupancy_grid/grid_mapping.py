#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
grid_mapping.py — probabilistic 2D occupancy grid mapping (pure NumPy, tested).

The classic log-odds occupancy framework (Moravec / Elfes):

    belief  l(x) = log( p(x occupied) / p(x free) )
    update  l(x) <- l(x) + l_occ          (sensor hit cell)
            l(x) <- l(x) + l_free         (every cell along the ray)

    p = 1 - 1/(1 + exp(l))   ->  0..100 OccupancyGrid values

Rays are rasterized with the integer Bresenham line algorithm (O(L) per ray,
no floating-point per-cell cost). Sensor origin = the robot position in the
grid frame; points come from the obstacle cloud in the same frame.
"""

from typing import Optional, Tuple

import numpy as np

L_OCC = 0.85        # log-odds added for an occupied cell (p≈0.70)
L_FREE = -0.4       # log-odds added for a free cell     (p≈0.40)
L_CLAMP = 3.5       # log-odds clamp ([-L_CLAMP, +L_CLAMP])


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """
    Integer line from (x0, y0) to (x1, y1) (inclusive) — Bresenham's
    algorithm. Yields cell coordinates.
    """
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


class OccupancyGridMapper:
    """
    Incremental log-odds occupancy grid.

    Args:
        resolution:  meters per cell.
        width_m, height_m: grid size in meters (centered on the origin).
        l_occ, l_free, l_clamp: log-odds update parameters.
    """

    def __init__(self, resolution: float = 0.2,
                 width_m: float = 60.0, height_m: float = 60.0,
                 l_occ: float = L_OCC, l_free: float = L_FREE,
                 l_clamp: float = L_CLAMP):
        if resolution <= 0.0:
            raise ValueError("resolution must be > 0")
        self.resolution = float(resolution)
        self.width = int(round(width_m / resolution))
        self.height = int(round(height_m / resolution))
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_clamp = float(l_clamp)
        # world x -> column: col = (x / res) + width//2
        self.half_w = self.width // 2
        self.half_h = self.height // 2
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    # ------------------------------------------------------------------ #
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """World coordinates -> cell (row, col)."""
        col = int(np.floor(x / self.resolution)) + self.half_w
        row = int(np.floor(y / self.resolution)) + self.half_h
        return row, col

    def cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Cell center -> world coordinates."""
        x = (col - self.half_w) * self.resolution + self.resolution / 2.0
        y = (row - self.half_h) * self.resolution + self.resolution / 2.0
        return x, y

    # ------------------------------------------------------------------ #
    def add_scan(self, points: np.ndarray, robot_x: float, robot_y: float,
                 max_range: Optional[float] = None) -> None:
        """
        Integrate one scan.

        Args:
            points: (N, 2) obstacle points in the grid frame.
            robot_x, robot_y: sensor/robot position in the grid frame.
            max_range: skip points farther than this [m].
        """
        points = np.asarray(points, dtype=float)
        if points.ndim == 1:
            points = points.reshape(1, -1)
        if max_range is not None:
            d = np.hypot(points[:, 0] - robot_x, points[:, 1] - robot_y)
            points = points[d <= max_range]
        if points.shape[0] == 0:
            return

        rx, ry = self.world_to_cell(robot_x, robot_y)
        if not (0 <= rx < self.height and 0 <= ry < self.width):
            return

        # ---- free cells along each ray (Bresenham) ---------------------------
        for px, py in points:
            cx, cy = self.world_to_cell(px, py)
            if not (0 <= cx < self.height and 0 <= cy < self.width):
                # still mark the visible part of the ray
                cx = min(max(cx, 0), self.height - 1)
                cy = min(max(cy, 0), self.width - 1)
            for row, col in bresenham(rx, ry, cx, cy):
                if not (0 <= row < self.height and 0 <= col < self.width):
                    break
                if row == cx and col == cy:
                    continue          # endpoint handled as occupied below
                self.log_odds[row, col] = np.clip(
                    self.log_odds[row, col] + self.l_free,
                    -self.l_clamp, self.l_clamp)

        # ---- occupied cells (endpoints) ---------------------------------------
        for px, py in points:
            cx, cy = self.world_to_cell(px, py)
            if 0 <= cx < self.height and 0 <= cy < self.width:
                self.log_odds[cx, cy] = np.clip(
                    self.log_odds[cx, cy] + self.l_occ,
                    -self.l_clamp, self.l_clamp)

    # ------------------------------------------------------------------ #
    def get_occupancy(self) -> np.ndarray:
        """
        Log-odds -> int8 occupancy in the nav_msgs/OccupancyGrid convention:
            -1 = unknown, 0 = free, 100 = occupied.
        """
        p = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        occ = (p * 100.0).astype(np.int8)
        # cells never observed stay unknown
        unknown = self.log_odds == 0.0
        occ[unknown] = -1
        return occ

    # ------------------------------------------------------------------ #
    def get_log_odds(self) -> np.ndarray:
        """Raw log-odds array (H, W)."""
        return self.log_odds

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        self.log_odds[:] = 0.0
