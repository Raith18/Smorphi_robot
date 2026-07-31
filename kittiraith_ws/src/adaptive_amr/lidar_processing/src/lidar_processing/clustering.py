#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
clustering.py — Euclidean clustering and bounding boxes (pure NumPy/SciPy).

  * EuclideanClusterExtraction — PCL-style radius clustering with a KD-tree:
      - build KD-tree:        O(n log n)
      - BFS over neighbors:   O(n * avg. neighbors within tolerance)
  * OrientedBoundingBox      — PCA box: eigenvectors of the covariance matrix
      are the box axes, extents = 2 * sqrt(eigenvalues).
  * AxisAlignedBoundingBox   — cheap min/max box.
"""

from typing import List, Optional

import numpy as np

try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    HAVE_SCIPY = False


class EuclideanClusterExtraction:
    """
    Clusters points whose mutual distance is below `tolerance`.

    Args:
        tolerance:        radius for neighborhood queries [m].
        min_cluster_size: drop clusters with fewer points.
        max_cluster_size: split/ignore huge clusters (far walls etc.).
    """

    def __init__(self, tolerance: float = 0.5,
                 min_cluster_size: int = 10,
                 max_cluster_size: int = 200_000):
        if tolerance <= 0.0:
            raise ValueError("tolerance must be > 0")
        self.tolerance = float(tolerance)
        self.min_cluster_size = int(min_cluster_size)
        self.max_cluster_size = int(max_cluster_size)

    def extract(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster `points` (N, 3). Returns a list of index arrays (one per
        cluster), sorted by cluster size (largest first).
        """
        n = points.shape[0]
        if n == 0:
            return []
        if not HAVE_SCIPY:
            raise RuntimeError("scipy is required for Euclidean clustering")
        tree = cKDTree(points)
        unvisited = set(range(n))
        clusters: List[List[int]] = []

        while unvisited:
            seed = unvisited.pop()
            queue = [seed]
            cluster: List[int] = []
            while queue:
                idx = queue.pop()
                cluster.append(idx)
                for neighbor in tree.query_ball_point(points[idx], self.tolerance):
                    if neighbor in unvisited:
                        unvisited.remove(neighbor)
                        queue.append(neighbor)
            if self.min_cluster_size <= len(cluster) <= self.max_cluster_size:
                clusters.append(cluster)

        clusters.sort(key=len, reverse=True)
        return [np.asarray(c, dtype=np.int64) for c in clusters]


class OrientedBoundingBox:
    """
    PCA-based oriented bounding box (PCL-style tight box).

    Axes come from the eigenvectors of the covariance matrix; the extents are
    the min/max of the point projections onto those axes (which gives the
    exact tight box for any point distribution).

    Attributes:
        center : (3,) box center.
        axes   : (3, 3) orthonormal box axes (columns, largest variance first).
        extents: (3,) box dimensions along the axes.
        corners: (8, 3) box corners.
    """

    def __init__(self, points: np.ndarray):
        if points.shape[0] < 3:
            raise ValueError("need at least 3 points for an OBB")
        covariance = np.cov(points.T)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)  # ascending

        order = np.argsort(eigenvalues)[::-1]                    # descending
        axes = eigenvectors[:, order]
        if np.linalg.det(axes) < 0:                              # right-handed
            axes[:, 2] *= -1.0
        self.axes = axes

        projections = points @ axes
        box_min = projections.min(axis=0)
        box_max = projections.max(axis=0)
        self.extents = box_max - box_min
        self.center = axes @ (0.5 * (box_min + box_max))

        # ---- corners: center +- half extents along axes -----------------------
        half = self.extents / 2.0
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                          for sz in (-1, 1)], dtype=float)
        self.corners = self.center + (signs * half) @ axes.T

    @property
    def volume(self) -> float:
        return float(np.prod(self.extents))

    def __repr__(self):
        return ("OrientedBoundingBox(center={}, extents={})".format(
            np.round(self.center, 3), np.round(self.extents, 3)))


class AxisAlignedBoundingBox:
    """Min/max axis-aligned box."""

    def __init__(self, points: np.ndarray):
        self.min = points.min(axis=0)
        self.max = points.max(axis=0)
        self.center = 0.5 * (self.min + self.max)
        self.extents = self.max - self.min

    @property
    def corners(self) -> np.ndarray:
        mins, maxs = self.min, self.max
        return np.array([[x, y, z]
                         for x in (mins[0], maxs[0])
                         for y in (mins[1], maxs[1])
                         for z in (mins[2], maxs[2])])

    def __repr__(self):
        return "AxisAlignedBoundingBox(min={}, max={})".format(
            np.round(self.min, 3), np.round(self.max, 3))
