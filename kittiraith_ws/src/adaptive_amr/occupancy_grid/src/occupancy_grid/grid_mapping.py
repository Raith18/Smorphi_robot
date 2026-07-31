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
        Integrate one scan (VECTORIZED ray casting, Phase 8 optimization).

        Instead of a per-point Bresenham Python loop, every ray is sampled at
        steps of <= 0.5 cells (so no grid cell is skipped) and the free/occ
        updates are applied with np.add.at on a flattened grid. Points are
        processed in chunks to bound memory.

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

        origin = np.array([robot_x, robot_y], dtype=float)
        step = 0.5 * self.resolution
        chunk = 2048                       # points per vectorized chunk
        flat_log = self.log_odds.ravel()

        for start in range(0, points.shape[0], chunk):
            pts = points[start:start + chunk]
            dirs = pts[:, :2] - origin
            dists = np.linalg.norm(dirs, axis=1)
            n_samples = np.maximum(np.ceil(dists / step).astype(int), 1) + 1
            max_n = int(n_samples.max())
            if max_n < 1:
                continue

            t = np.linspace(0.0, 1.0, max_n)                     # (max_n,)
            samples = (origin[None, :]
                       + dirs[:, None, :] * t[None, :, None])    # (N, max_n, 2)
            cols = np.floor(samples[..., 0] / self.resolution).astype(int) \
                + self.half_w
            rows = np.floor(samples[..., 1] / self.resolution).astype(int) \
                + self.half_h

            valid = np.arange(max_n)[None, :] < n_samples[:, None]
            in_bounds = (valid
                         & (rows >= 0) & (rows < self.height)
                         & (cols >= 0) & (cols < self.width))

            # last in-bounds sample of each ray = the occupied hit; everything
            # before it along the ray is free
            idx = np.arange(max_n)
            last_in = np.where(in_bounds.any(axis=1),
                               (in_bounds * idx[None, :]).max(axis=1), -1)
            is_last = np.zeros_like(in_bounds)
            valid_last = last_in >= 0
            is_last[valid_last, last_in[valid_last]] = True

            free_flat = (rows[in_bounds & ~is_last] * self.width
                         + cols[in_bounds & ~is_last])
            occ_flat = (rows[is_last] * self.width + cols[is_last])

            # np.bincount is much faster than np.add.at for large scatters
            if free_flat.size:
                flat_log += np.bincount(free_flat,
                                        minlength=self.height * self.width) \
                    * self.l_free
            if occ_flat.size:
                flat_log += np.bincount(occ_flat,
                                        minlength=self.height * self.width) \
                    * self.l_occ

        np.clip(flat_log, -self.l_clamp, self.l_clamp, out=flat_log)

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
