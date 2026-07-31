#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ground_segmentation.py — RANSAC plane fitting and ground removal (pure NumPy).

The ground is modeled as a plane  n . p + d = 0. RANSAC finds it robustly
even with 50%+ obstacle points:

  1. Sample 3 random points, compute the plane normal n = (p1-p0) x (p2-p0).
  2. Count inliers: |n . p + d| < distance_threshold.
  3. Repeat N times, keep the model with the most inliers.
  4. Refit the winner with total least squares (SVD) on its inliers.
"""

from typing import Tuple

import numpy as np


def fit_plane_svd(points: np.ndarray) -> Tuple[np.ndarray, float]:
    """
    Fit a plane to `points` (N,3) via SVD (total least squares).

    Returns (normal, d) with the plane n.p + d = 0. The normal is
    canonicalized to point up (positive z) — the sensor frame convention.
    """
    if points.shape[0] < 3:
        raise ValueError("need at least 3 points to fit a plane")
    centroid = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - centroid)
    normal = vt[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal *= -1.0
    d = -float(np.dot(normal, centroid))
    return normal, d


class RansacPlaneSegmenter:
    """
    RANSAC plane fitting.

    Args:
        distance_threshold: inlier distance [m] (e.g. 0.2).
        max_iterations:     RANSAC iteration budget.
        seed:               RNG seed for reproducibility (tests).
    """

    def __init__(self, distance_threshold: float = 0.2,
                 max_iterations: int = 50, seed: int = None):
        self.distance_threshold = float(distance_threshold)
        self.max_iterations = int(max_iterations)
        self.seed = seed

    def fit(self, points: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
        """
        Fit the plane. Returns (normal, d, inlier_mask).
        """
        n = points.shape[0]
        if n < 3:
            raise ValueError("need at least 3 points for RANSAC")
        rng = np.random.default_rng(self.seed)

        best_inliers = None
        best_model = None
        for _ in range(self.max_iterations):
            sample = rng.choice(n, 3, replace=False)
            p0, p1, p2 = points[sample]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(normal)
            if norm < 1e-12:
                continue
            normal /= norm
            if normal[2] < 0.0:
                normal *= -1.0        # canonical: normal points up
            d = -float(np.dot(normal, p0))
            dist = np.abs(points @ normal + d)
            inliers = dist < self.distance_threshold
            if best_inliers is None or int(inliers.sum()) > int(best_inliers.sum()):
                best_inliers = inliers
                best_model = (normal, d)

        if best_inliers is None or int(best_inliers.sum()) < 3:
            raise ValueError("RANSAC could not find a plane (bad point cloud?)")

        # ---- refit on the winner with SVD (more accurate than 3-point) --------
        normal, d = fit_plane_svd(points[best_inliers])
        dist = np.abs(points @ normal + d)
        best_inliers = dist < self.distance_threshold
        return normal, d, best_inliers


class GroundRemover:
    """
    Separates ground from obstacles.

    Args:
        segmenter:       RansacPlaneSegmenter instance.
        min_normal_z:    keep only planes pointing up (normal z > this).
        ground_offset:   extra margin above the fitted plane still "ground".
    """

    def __init__(self, segmenter: RansacPlaneSegmenter,
                 min_normal_z: float = 0.6, ground_offset: float = 0.05):
        self.segmenter = segmenter
        self.min_normal_z = float(min_normal_z)
        self.ground_offset = float(ground_offset)

    def separate(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns (ground, obstacles, plane) where plane = (normal, d).
        Points whose signed distance to the plane is <= ground_offset are
        ground; the rest are obstacles.
        """
        normal, d, inliers = self.segmenter.fit(points)
        if normal[2] < self.min_normal_z:
            # The dominant plane is not the floor (e.g. a wall) — fall back to
            # treating everything above the lowest points as obstacles.
            inliers = np.zeros(points.shape[0], dtype=bool)
        signed_dist = points @ normal + d
        inliers |= signed_dist <= self.ground_offset
        ground = points[inliers]
        obstacles = points[~inliers]
        return ground, obstacles, (normal, d)
