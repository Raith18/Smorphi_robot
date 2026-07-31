#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry_utils.py — pure-NumPy geometry for visual odometry (unit-tested).

  * triangulate_dlt / triangulate_many — homogeneous (DLT) stereo
    triangulation: solves A x = 0 with A built from two 3x4 projection
    matrices, via SVD.
  * rvec_tvec_to_matrix / matrix_to_rvec_tvec — cv2-style conversions.
  * invert_pose, compose_poses — 4x4 homogeneous transform helpers.
  * matrix_to_quat_xyzw — reuse dataset_loader's quaternion conversion.
"""

from typing import Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# Stereo triangulation (DLT)
# --------------------------------------------------------------------------- #
def triangulate_dlt(P1: np.ndarray, P2: np.ndarray,
                    pt1, pt2) -> np.ndarray:
    """
    Triangulate one point from two projection matrices (Direct Linear Transform).

    For a 3x4 projection P, a pixel (u, v) contributes two rows
        u * P[2] - P[0]  and  v * P[2] - P[1]
    to the homogeneous system A x = 0 (x = [X, Y, Z, 1]). The solution is the
    last right-singular vector of A (SVD). Returns the 3D point in the common
    reference frame of P1/P2.
    """
    P1 = np.asarray(P1, dtype=float)
    P2 = np.asarray(P2, dtype=float)
    u1, v1 = float(pt1[0]), float(pt1[1])
    u2, v2 = float(pt2[0]), float(pt2[1])

    A = np.array([
        u1 * P1[2] - P1[0],
        v1 * P1[2] - P1[1],
        u2 * P2[2] - P2[0],
        v2 * P2[2] - P2[1],
    ])
    _, _, vt = np.linalg.svd(A)
    x = vt[-1]
    if abs(x[3]) < 1e-12:
        raise ValueError("Degenerate triangulation (w ~ 0)")
    return x[:3] / x[3]


def triangulate_many(P1: np.ndarray, P2: np.ndarray,
                     pts1: np.ndarray, pts2: np.ndarray) -> np.ndarray:
    """
    Vectorized DLT triangulation for many points.

    Args:
        P1, P2: 3x4 projection matrices.
        pts1, pts2: (N, 2) pixel coordinates.

    Returns:
        (N, 3) points in the common reference frame.
    """
    pts1 = np.asarray(pts1, dtype=float)
    pts2 = np.asarray(pts2, dtype=float)
    if pts1.shape != pts2.shape or pts1.ndim != 2 or pts1.shape[1] != 2:
        raise ValueError("pts1/pts2 must be (N, 2) with equal shape")
    n = pts1.shape[0]

    A = np.empty((n, 4, 4), dtype=float)
    A[:, 0] = pts1[:, 0:1] * P1[2] - P1[0]
    A[:, 1] = pts1[:, 1:2] * P1[2] - P1[1]
    A[:, 2] = pts2[:, 0:1] * P2[2] - P2[0]
    A[:, 3] = pts2[:, 1:2] * P2[2] - P2[1]

    _, _, vt = np.linalg.svd(A)            # batched SVD -> (n, 4, 4)
    x = vt[:, -1]                           # last right-singular vector
    with np.errstate(divide="ignore", invalid="ignore"):
        out = x[:, :3] / x[:, 3:4]
    return out.astype(np.float64)


# --------------------------------------------------------------------------- #
# Pose conversions (cv2-compatible conventions)
# --------------------------------------------------------------------------- #
def rvec_tvec_to_matrix(rvec, tvec) -> np.ndarray:
    """Rotation vector + translation -> 4x4 homogeneous matrix."""
    rvec = np.asarray(rvec, dtype=float).reshape(3)
    tvec = np.asarray(tvec, dtype=float).reshape(3)
    try:
        import cv2
        rmat, _ = cv2.Rodrigues(rvec)
    except ImportError:  # pragma: no cover
        rmat = rotation_matrix_from_axis_angle(rvec)
    T = np.eye(4)
    T[:3, :3] = rmat
    T[:3, 3] = tvec
    return T


def rotation_matrix_from_axis_angle(rvec) -> np.ndarray:
    """Rodrigues formula in pure NumPy (fallback when cv2 is unavailable)."""
    rvec = np.asarray(rvec, dtype=float)
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        return np.eye(3)
    k = rvec / theta
    kx, ky, kz = k
    skew = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    return (np.eye(3) + np.sin(theta) * skew
            + (1.0 - np.cos(theta)) * (skew @ skew))


def matrix_to_rvec_tvec(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """4x4 matrix -> (rotation vector, translation)."""
    T = np.asarray(T, dtype=float)
    try:
        import cv2
        rvec, _ = cv2.Rodrigues(T[:3, :3])
    except ImportError:  # pragma: no cover
        rvec = axis_angle_from_rotation(T[:3, :3])
    return rvec.reshape(3), T[:3, 3].reshape(3)


def axis_angle_from_rotation(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> rotation vector (pure NumPy fallback)."""
    R = np.asarray(R, dtype=float)
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if theta < 1e-12:
        return np.zeros(3)
    r = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]])
    return r * (theta / (2.0 * np.sin(theta)))


# --------------------------------------------------------------------------- #
# 4x4 helpers
# --------------------------------------------------------------------------- #
def invert_pose(T: np.ndarray) -> np.ndarray:
    """Inverse of a rigid 4x4 transform."""
    T = np.asarray(T, dtype=float)
    R = T[:3, :3].T
    t = -R @ T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = t
    return out


def compose_poses(T_a_b: np.ndarray, T_b_c: np.ndarray) -> np.ndarray:
    """Compose two rigid transforms: T_a_c = T_a_b @ T_b_c."""
    return np.asarray(T_a_b, dtype=float) @ np.asarray(T_b_c, dtype=float)


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 rigid transform to (N, 3) points."""
    points = np.asarray(points, dtype=float)
    hom = np.hstack([points, np.ones((points.shape[0], 1))])
    return (np.asarray(T, dtype=float) @ hom.T).T[:, :3]


def matrix_to_quat_xyzw(T: np.ndarray) -> np.ndarray:
    """4x4 matrix -> quaternion [x, y, z, w] (stable Shepperd method)."""
    from dataset_loader.kitti_parsers import matrix_to_quaternion
    return matrix_to_quaternion(np.asarray(T, dtype=float)[:3, :3])
