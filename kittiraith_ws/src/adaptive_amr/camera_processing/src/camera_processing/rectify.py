#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rectify.py — camera undistortion/rectification math (pure NumPy, OpenCV-optional).

Implements the standard pinhole-camera correction pipeline so it is testable
on any host and serves as a transparent reference to the OpenCV
implementation used at runtime:

  output pixel (u,v)
    -> normalized rectified ray  x = (u - c'x)/f'x , y = (v - c'y)/f'y
    -> original camera ray       p_cam = R_rect^T * [x, y, 1]
    -> distorted coordinates     (plumb_bob model: k1 k2 p1 p2 k3)
    -> source pixel via K

Key correctness rule implemented here and used by the node:
  * If the image is cropped/resized, P and K must be adjusted:
      f' = s*f , c' = s*(c - crop) , t' = s*t
    Publishing a processed image with stale intrinsics silently breaks every
    downstream projection — this module prevents that.
"""

from typing import Optional, Sequence, Tuple

import numpy as np


# --------------------------------------------------------------------------- #
# Distortion model
# --------------------------------------------------------------------------- #
def distort_point_norm(xn, yn, k1, k2, p1, p2, k3):
    """Plumb-bob (Brown-Conrady) distortion of normalized coordinates."""
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    xd = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    yd = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    return xd, yd


def build_rectify_maps_numpy(K: np.ndarray, D: np.ndarray, R: np.ndarray,
                             P: np.ndarray, size: Tuple[int, int],
                             interpolation: str = "linear") -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute remap tables (map_x, map_y) for undistort+rectify+crop+resize.

    Args:
        K:    3x3 intrinsics of the ORIGINAL (unrectified) image.
        D:    5 distortion coefficients (k1 k2 p1 p2 k3).
        R:    3x3 rectification rotation (R_rect); identity if already rectified.
        P:    3x4 projection of the OUTPUT image (already adjusted for any
              crop/resize by adjust_projection_matrix).
        size: (width, height) of the OUTPUT image.

    Returns:
        map_x, map_y float32 arrays of shape (height, width) mapping each
        output pixel to its source pixel in the ORIGINAL image. Equivalent to
        cv2.initUndistortRectifyMap (validated in the unit tests).
    """
    width, height = size
    # --- 1. output pixel -> normalized rectified ray --------------------------
    fx = P[0, 0]
    fy = P[1, 1]
    cx = P[0, 2]
    cy = P[1, 2]
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("Projection matrix has non-positive focal length: fx={}, fy={}".format(fx, fy))

    yy, xx = np.mgrid[0:height, 0:width].astype(np.float64)
    xn = (xx - cx) / fx
    yn = (yy - cy) / fy

    # --- 2. rotate back to the original camera frame ---------------------------
    if R is not None and not np.allclose(R, np.eye(3)):
        rays = np.stack([xn, yn, np.ones_like(xn)], axis=-1)      # (H,W,3)
        rays_orig = rays @ R.T                                     # R^T * ray
        xn, yn = rays_orig[..., 0], rays_orig[..., 1]

    # --- 3. apply distortion ------------------------------------------------
    if D is None or len(D) == 0:
        k1 = k2 = p1 = p2 = k3 = 0.0
    else:
        k1 = D[0]
        k2 = D[1] if len(D) > 1 else 0.0
        p1 = D[2] if len(D) > 2 else 0.0
        p2 = D[3] if len(D) > 3 else 0.0
        k3 = D[4] if len(D) > 4 else 0.0
    xd, yd = distort_point_norm(xn, yn, k1, k2, p1, p2, k3)

    # --- 4. source pixel via K --------------------------------------------------
    map_x = (K[0, 0] * xd + K[0, 2]).astype(np.float32)
    map_y = (K[1, 1] * yd + K[1, 2]).astype(np.float32)
    return map_x, map_y


def build_crop_resize_maps_numpy(crop: Sequence[int], scale: float,
                                 size: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray]:
    """
    Pure crop+resize remap tables for already-corrected images
    (rectify_mode == "none"): output pixel (i, j) maps to source
    (i/scale + crop_y, j/scale + crop_x).
    """
    width, height = size
    crop_x, crop_y = crop[0], crop[1]
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float32)
    map_x = xx / scale + crop_x
    map_y = yy / scale + crop_y
    return map_x, map_y


# --------------------------------------------------------------------------- #
# Camera info adjustment
# --------------------------------------------------------------------------- #
def adjust_projection_matrix(P: np.ndarray, crop: Sequence[int],
                             scale: float) -> np.ndarray:
    """
    Adjust a 3x4 projection matrix for cropping and resizing:

        f' = s*f , c' = s*(c - crop) , t' = s*t

    Keeps the projection matrix consistent with the processed image so
    downstream consumers (fusion, mapping) can trust it.
    """
    new_p = np.array(P, dtype=float, copy=True)
    crop_x, crop_y = crop[0], crop[1]
    new_p[0, 0] *= scale
    new_p[1, 1] *= scale
    new_p[0, 2] = (new_p[0, 2] - crop_x) * scale
    new_p[1, 2] = (new_p[1, 2] - crop_y) * scale
    new_p[0, 3] *= scale
    new_p[1, 3] *= scale
    return new_p


def adjust_intrinsics(K: np.ndarray, crop: Sequence[int], scale: float) -> np.ndarray:
    """Adjust a 3x3 intrinsics matrix for crop/resize (same rule as P)."""
    new_k = np.array(K, dtype=float, copy=True)
    crop_x, crop_y = crop[0], crop[1]
    new_k[0, 0] *= scale
    new_k[1, 1] *= scale
    new_k[0, 2] = (new_k[0, 2] - crop_x) * scale
    new_k[1, 2] = (new_k[1, 2] - crop_y) * scale
    return new_k


# --------------------------------------------------------------------------- #
# Sampling (remap)
# --------------------------------------------------------------------------- #
def remap_numpy(image: np.ndarray, map_x: np.ndarray, map_y: np.ndarray) -> np.ndarray:
    """
    Nearest-neighbor remap (pure NumPy reference implementation of cv2.remap).
    The node prefers cv2.remap (bilinear) when OpenCV is available.
    """
    height, width = map_x.shape
    src_h, src_w = image.shape[:2]
    u = np.clip(np.rint(map_x), 0, src_w - 1).astype(np.int32)
    v = np.clip(np.rint(map_y), 0, src_h - 1).astype(np.int32)
    return image[v, u]


# --------------------------------------------------------------------------- #
# RectifyMapper — one object per (camera_info, crop, scale, mode)
# --------------------------------------------------------------------------- #
class RectifyMapper:
    """
    Builds and applies the rectification/processing maps for one camera.

    Modes:
      * "none"               — already rectified (KITTI _sync): crop+resize only.
      * "undistort_rectify"  — full undistortion + rectification (raw cameras).
    """

    def __init__(self, K: np.ndarray, D: Optional[np.ndarray], R: Optional[np.ndarray],
                 P: np.ndarray, input_size: Tuple[int, int],
                 mode: str = "none",
                 crop: Sequence[int] = (0, 0, 0, 0),
                 scale: float = 1.0,
                 use_cv2: bool = True):
        if mode not in ("none", "undistort_rectify"):
            raise ValueError("mode must be 'none' or 'undistort_rectify', got '{}'".format(mode))
        self.mode = mode
        self.use_cv2 = use_cv2 and _HAVE_CV2
        self.crop = tuple(int(v) for v in crop[:4])
        self.scale = float(scale)
        self.input_size = tuple(int(v) for v in input_size)
        self._cv2 = _import_cv2()

        # ---- adjusted projection & output size -------------------------------
        self.output_P = adjust_projection_matrix(P, self.crop, self.scale)
        crop_w = self.crop[2] if self.crop[2] > 0 else self.input_size[0] - self.crop[0]
        crop_h = self.crop[3] if self.crop[3] > 0 else self.input_size[1] - self.crop[1]
        self.output_size = (int(round(crop_w * self.scale)), int(round(crop_h * self.scale)))

        # ---- build maps ---------------------------------------------------------
        if mode == "undistort_rectify":
            self.map_x, self.map_y = build_rectify_maps_numpy(
                K, D, R, self.output_P, self.output_size)
        else:
            self.map_x, self.map_y = build_crop_resize_maps_numpy(
                self.crop, self.scale, self.output_size)

    # ------------------------------------------------------------------ #
    def apply(self, image: np.ndarray) -> np.ndarray:
        """Remap an input image (H,W,C or H,W) into the output image."""
        # camera_info size is (width, height); image shape is (height, width)
        if image.shape[:2] != (self.input_size[1], self.input_size[0]):
            raise ValueError(
                "Input image {} does not match camera_info size {}".format(
                    image.shape[:2], self.input_size))
        if self.use_cv2 and self._cv2 is not None:
            return self._cv2.remap(image, self.map_x, self.map_y,
                                   self._cv2.INTER_LINEAR)
        return remap_numpy(image, self.map_x, self.map_y)


# --------------------------------------------------------------------------- #
# Lazy cv2 import
# --------------------------------------------------------------------------- #
_HAVE_CV2 = False
try:
    import cv2  # noqa: F401
    _HAVE_CV2 = True
except ImportError:  # pragma: no cover
    _HAVE_CV2 = False


def _import_cv2():
    if _HAVE_CV2:
        import cv2
        return cv2
    return None
