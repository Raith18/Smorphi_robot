#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
filters.py — point-cloud filtering (pure NumPy).

  * PassthroughFilter — axis-aligned range limits (+ horizontal-range limit).
  * VoxelGrid         — uniform downsampling via grid quantization.
"""

from typing import Dict, Optional, Tuple

import numpy as np


class PassthroughFilter:
    """
    Keeps points whose coordinates lie inside axis-aligned bounds.

    Args:
        limits:      {"x": (min, max), "y": ..., "z": ...} — optional axes.
        min_range:   minimum horizontal range sqrt(x^2 + y^2).
        max_range:   maximum horizontal range (LiDAR max ~ 120 m).
    """

    def __init__(self, limits: Optional[Dict[str, Tuple[float, float]]] = None,
                 min_range: float = 0.0, max_range: float = float("inf")):
        self.limits = limits or {}
        self.min_range = float(min_range)
        self.max_range = float(max_range)

    def filter(self, points: np.ndarray) -> np.ndarray:
        """Return the filtered (N', 3) array."""
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points must be (N, >=3), got {}".format(points.shape))
        mask = np.ones(points.shape[0], dtype=bool)
        axis_index = {"x": 0, "y": 1, "z": 2}
        for axis, (lo, hi) in self.limits.items():
            idx = axis_index[axis]
            mask &= (points[:, idx] >= lo) & (points[:, idx] <= hi)
        horizontal_range = np.hypot(points[:, 0], points[:, 1])
        mask &= (horizontal_range >= self.min_range) & (horizontal_range <= self.max_range)
        return points[mask]


class VoxelGrid:
    """
    Uniform voxel downsampling: quantize points into a grid of `leaf_size`
    cells and keep the centroid of each cell.

    Complexity O(n); memory O(n). Preserves spatial distribution (unlike
    random sampling) while shrinking 120k-point scans by ~20-50x.
    """

    def __init__(self, leaf_size: float = 0.3):
        if leaf_size <= 0.0:
            raise ValueError("leaf_size must be > 0")
        self.leaf_size = float(leaf_size)

    def downsample(self, points: np.ndarray,
                   features: Optional[np.ndarray] = None) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Downsample `points` (N, 3). Optionally aggregate `features` (N, F) by
        averaging per voxel. Returns (centroids, averaged_features|None).
        """
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points must be (N, >=3), got {}".format(points.shape))
        voxel = np.floor(points[:, :3] / self.leaf_size).astype(np.int64)
        _, inverse, counts = np.unique(voxel, axis=0, return_inverse=True,
                                       return_counts=True)

        centroids = np.zeros((len(counts), points.shape[1]), dtype=np.float64)
        np.add.at(centroids, inverse, points)
        centroids /= counts[:, None]
        centroids = centroids.astype(np.float32)

        if features is None:
            return centroids, None
        feats = np.zeros((len(counts), features.shape[1]), dtype=np.float64)
        np.add.at(feats, inverse, features)
        feats /= counts[:, None]
        return centroids, feats.astype(np.float32)
