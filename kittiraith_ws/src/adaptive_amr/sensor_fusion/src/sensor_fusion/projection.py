#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
projection.py — camera-LiDAR projection (pure NumPy).

The core fusion equation (KITTI convention):

    p_img = P_rect . R_rect4x4 . T_velo_cam . p_velo
    u = p_img[0] / p_img[2] ,  v = p_img[1] / p_img[2] ,  depth = p_img[2]

  * T_velo_cam : 4x4, velodyne -> cam0            (from calib_velo_to_cam.txt)
  * R_rect4x4  : 4x4, cam0 -> rectified cam0      (R_rect_00 padded)
  * P_rect     : 3x4, rectified cam0 -> pixels    (P_rect_00, includes K)

Projecting n points is one matrix multiply: O(n), ~100k points in ~1 ms.
"""

from typing import Optional, Tuple

import numpy as np


def combined_projection_matrix(P: np.ndarray, R_rect: np.ndarray,
                               T_velo_cam: np.ndarray) -> np.ndarray:
    """Build the 3x4 matrix mapping velodyne points straight to pixels."""
    r_rect4 = np.eye(4)
    r_rect4[:3, :3] = R_rect
    return np.asarray(P, dtype=float).dot(r_rect4).dot(np.asarray(T_velo_cam, dtype=float))


class LidarCameraProjection:
    """
    Projects LiDAR points into the camera image and builds depth maps.

    Args:
        P:         3x4 rectified projection matrix of the camera.
        R_rect:    3x3 rectification rotation (identity for rectified cams).
        T_velo_cam:4x4 extrinsic velodyne -> camera (identity if same frame).
        width, height: image dimensions (for depth map bounds).
    """

    def __init__(self, P: np.ndarray, R_rect: Optional[np.ndarray] = None,
                 T_velo_cam: Optional[np.ndarray] = None,
                 width: int = 1242, height: int = 376):
        self.P = np.asarray(P, dtype=float)
        self.R_rect = np.eye(3) if R_rect is None else np.asarray(R_rect, float)
        self.T_velo_cam = (np.eye(4) if T_velo_cam is None
                           else np.asarray(T_velo_cam, float))
        self.combined = combined_projection_matrix(self.P, self.R_rect,
                                                   self.T_velo_cam)
        self.width = int(width)
        self.height = int(height)

    # ------------------------------------------------------------------ #
    def project(self, points: np.ndarray) -> Tuple[np.ndarray, np.ndarray,
                                                   np.ndarray, np.ndarray]:
        """
        Project `points` (N, 3) in the velodyne frame.

        Returns (u, v, depth, valid):
            u, v    : float pixel coordinates (can be outside the image).
            depth   : z in the rectified camera frame [m].
            valid   : depth > 0 (in front of the camera).
        """
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points must be (N, >=3), got {}".format(points.shape))
        ones = np.ones((points.shape[0], 1), dtype=np.float64)
        hom = np.hstack([points[:, :3], ones])
        projected = hom @ self.combined.T                    # (N, 3)
        depth = projected[:, 2]
        valid = depth > 1e-6
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.where(valid, projected[:, 0] / depth, -1.0)
            v = np.where(valid, projected[:, 1] / depth, -1.0)
        return u, v, depth, valid

    # ------------------------------------------------------------------ #
    def in_image_mask(self, u: np.ndarray, v: np.ndarray) -> np.ndarray:
        """Boolean mask of pixels inside the image bounds."""
        return ((u >= 0) & (u < self.width - 1) & (v >= 0) & (v < self.height - 1))

    # ------------------------------------------------------------------ #
    def build_sparse_depth(self, u: np.ndarray, v: np.ndarray, depth: np.ndarray,
                           valid: np.ndarray) -> np.ndarray:
        """
        Rasterize projected points into a (H, W) float32 depth image.
        Unobserved pixels are NaN (so downstream interpolation can mask them).
        """
        depth_image = np.full((self.height, self.width), np.nan, dtype=np.float32)
        inside = self.in_image_mask(u, v) & valid
        uu = u[inside].astype(np.int32)
        vv = v[inside].astype(np.int32)
        depth_image[vv, uu] = depth[inside].astype(np.float32)
        return depth_image

    # ------------------------------------------------------------------ #
    def colorize(self, points: np.ndarray, image: np.ndarray,
                 u: np.ndarray, v: np.ndarray, valid: np.ndarray,
                 default_color=(0, 0, 0)) -> np.ndarray:
        """
        Color every LiDAR point with the pixel it falls on.

        Returns (N, 3) uint8 RGB colors.
        """
        colors = np.zeros((points.shape[0], 3), dtype=np.uint8)
        colors[:] = default_color
        inside = self.in_image_mask(u, v) & valid
        uu = u[inside].astype(np.int32)
        vv = v[inside].astype(np.int32)
        colors[inside] = image[vv, uu, :3]
        return colors
