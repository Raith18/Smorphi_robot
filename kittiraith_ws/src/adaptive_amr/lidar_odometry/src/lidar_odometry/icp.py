#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
icp.py — Iterative Closest Point registration (pure NumPy/SciPy, unit-tested).

Two classic variants, both operating on (N, 3) point clouds:

  * icp_point_to_point — SVD/Kabsch closed-form solution. Robust baseline;
    sensitive to noise along surface normals.
  * icp_point_to_plane — linearized least squares against target surface
    normals (Chen & Medioni / Low). Converges faster and more accurately for
    LiDAR (structured surfaces) — the DEFAULT for this stack.

Both iterate: find nearest neighbors (cKDTree) -> solve for the incremental
rigid transform -> apply -> repeat, rejecting correspondences farther than
`max_correspondence_distance`.

estimate_normals computes PCA normals (smallest eigenvector of the k-NN
covariance), oriented toward the sensor origin (LiDAR convention).
"""

from typing import Optional, Tuple

import numpy as np

try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    HAVE_SCIPY = False


# --------------------------------------------------------------------------- #
# Normals
# --------------------------------------------------------------------------- #
def estimate_normals(points: np.ndarray, k: int = 10,
                     sensor_origin: Optional[np.ndarray] = None) -> np.ndarray:
    """
    PCA normals for `points` (N, 3): smallest eigenvector of the k-NN
    covariance, oriented toward `sensor_origin` (default: origin).

    Vectorized (batched SVD), O(n log n) for the KD-tree + O(n k^2).
    """
    points = np.asarray(points, dtype=float)
    n = points.shape[0]
    if n == 0:
        return np.zeros((0, 3))
    if not HAVE_SCIPY:
        raise RuntimeError("scipy is required for normal estimation")
    kk = min(k + 1, n)
    tree = cKDTree(points)
    _, idx = tree.query(points, k=kk)
    nbr = points[idx]                        # (n, kk, 3)
    centered = nbr - nbr.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / kk
    _, eigenvectors = np.linalg.eigh(covariance)   # ascending eigenvalues
    normals = eigenvectors[:, :, 0]                # smallest eigenvector
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.maximum(norms, 1e-12)

    if sensor_origin is not None:
        vec = points - np.asarray(sensor_origin, dtype=float)
        # Keep normals that point TOWARD the sensor (dot < 0); flip the rest.
        # (For a ground plane below the sensor this yields up-facing normals;
        #  for walls it yields normals facing the scanner — both required for
        #  stable point-to-plane ICP.)
        flip = np.einsum("ni,ni->n", normals, vec) > 0.0
        normals[flip] *= -1.0
    return normals


# --------------------------------------------------------------------------- #
# Closed-form helpers
# --------------------------------------------------------------------------- #
def _kabsch(source: np.ndarray, target: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Rotation + translation aligning source -> target (SVD)."""
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    sc = source - mu_s
    tc = target - mu_t
    h = sc.T @ tc
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1] *= -1.0
        r = vt.T @ u.T
    t = mu_t - r @ mu_s
    return r, t


def _compose(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _apply_transform(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    hom = np.hstack([points, np.ones((points.shape[0], 1))])
    return (T @ hom.T).T[:, :3]


# --------------------------------------------------------------------------- #
# ICP variants
# --------------------------------------------------------------------------- #
def icp_point_to_point(source: np.ndarray, target: np.ndarray,
                       init: Optional[np.ndarray] = None,
                       max_iterations: int = 50, tolerance: float = 1e-6,
                       max_correspondence_distance: float = 2.0
                       ) -> Tuple[np.ndarray, float]:
    """
    Classic point-to-point ICP (Besl & McKay 1992) with SVD alignment.

    Returns (T, rmse): T maps source points into the target frame.
    """
    if not HAVE_SCIPY:
        raise RuntimeError("scipy is required for ICP")
    if source.shape[0] < 3 or target.shape[0] < 3:
        raise ValueError("need at least 3 points in source and target")
    tree = cKDTree(target)
    T = np.eye(4) if init is None else np.asarray(init, dtype=float).copy()
    prev_rmse = float("inf")

    for _ in range(max_iterations):
        transformed = _apply_transform(source, T)
        dist, idx = tree.query(transformed, k=1)
        valid = dist < max_correspondence_distance
        if valid.sum() < 3:
            break
        s = transformed[valid]
        t = target[idx[valid]]
        r, tv = _kabsch(s, t)
        dT = _compose(r, tv)
        T = dT @ T
        rmse = float(np.sqrt(((t - _apply_transform(s, dT)) ** 2).sum(axis=1).mean()))
        if abs(prev_rmse - rmse) < tolerance:
            break
        prev_rmse = rmse
    return T, prev_rmse


def icp_point_to_plane(source: np.ndarray, target: np.ndarray,
                       target_normals: Optional[np.ndarray] = None,
                       init: Optional[np.ndarray] = None,
                       max_iterations: int = 40, tolerance: float = 1e-7,
                       max_correspondence_distance: float = 2.0,
                       min_correspondences: int = 6
                       ) -> Tuple[np.ndarray, float]:
    """
    Point-to-plane ICP (Chen & Medioni 1992, Low 2004).

    Minimizes  sum_i ( n_i . (R s_i + t - t_i) )^2
    Linearized around the current estimate (small-angle rotation):

        error_i ~= n_i.(s_i - t_i) + (s_i x n_i).w + n_i.t
        a_i = [ s_i x n_i , n_i ]   (6-vector)
        solve  (sum a_i a_i^T) x = sum a_i (n_i.(t_i - s_i))

    Returns (T, mean_point_to_plane_error).
    """
    if not HAVE_SCIPY:
        raise RuntimeError("scipy is required for ICP")
    if source.shape[0] < 3 or target.shape[0] < 3:
        raise ValueError("need at least 3 points in source and target")
    if target_normals is None:
        target_normals = estimate_normals(target)
    if target_normals.shape[0] != target.shape[0]:
        raise ValueError("target_normals must match target shape")

    tree = cKDTree(target)
    T = np.eye(4) if init is None else np.asarray(init, dtype=float).copy()
    prev_error = float("inf")

    for _ in range(max_iterations):
        transformed = _apply_transform(source, T)
        dist, idx = tree.query(transformed, k=1)
        valid = dist < max_correspondence_distance
        if valid.sum() < min_correspondences:
            break

        s = transformed[valid]                       # (m, 3)
        t = target[idx[valid]]
        n = target_normals[idx[valid]]

        cross = np.cross(s, n)                        # (m, 3) = s x n
        A = np.hstack([cross, n])                     # (m, 6)
        b = np.einsum("ni,ni->n", n, t - s)           # (m,)

        AtA = A.T @ A
        Atb = A.T @ b
        try:
            x = np.linalg.solve(AtA + 1e-9 * np.eye(6), Atb)
        except np.linalg.LinAlgError:
            x = np.linalg.lstsq(AtA, Atb, rcond=None)[0]

        alpha, beta, gamma, tx, ty, tz = x
        dT = np.array([
            [1.0, -gamma, beta, tx],
            [gamma, 1.0, -alpha, ty],
            [-beta, alpha, 1.0, tz],
            [0.0, 0.0, 0.0, 1.0],
        ])
        T = dT @ T

        # point-to-plane error after the update (for convergence)
        transformed = _apply_transform(source, T)
        dist, idx = tree.query(transformed, k=1)
        valid = dist < max_correspondence_distance
        if valid.sum() < min_correspondences:
            break
        err = np.abs(np.einsum("ni,ni->n",
                               target_normals[idx[valid]],
                               transformed[valid] - target[idx[valid]]))
        mean_error = float(err.mean())
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    return T, prev_error
