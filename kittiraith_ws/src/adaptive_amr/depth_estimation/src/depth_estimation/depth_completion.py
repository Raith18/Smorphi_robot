#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
depth_completion.py — sparse-to-dense depth completion (pure NumPy/SciPy).

The classic non-learned baseline:

  1. nearest_fill: for every pixel without depth, copy the depth of the
     nearest valid pixel. Computed in O(n) with the Euclidean distance
     transform (scipy.ndimage.distance_transform_edt, return_indices=True).
  2. smooth: optional Gaussian blur (a box blur fallback when OpenCV is
     absent) to remove the blocky look of nearest-fill edges.

This is the standard baseline against which learned depth-completion models
(KBNet, S2D, etc.) are compared — a strong, cheap, deterministic reference.
"""

from typing import Optional

import numpy as np

try:
    from scipy import ndimage
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    HAVE_SCIPY = False


def nearest_fill(sparse: np.ndarray, max_fill_distance: Optional[int] = None
                 ) -> np.ndarray:
    """
    Fill missing (NaN) depth with the nearest valid pixel's depth.

    Args:
        sparse:           (H, W) float32 depth image, NaN = unobserved.
        max_fill_distance: optional cap [px]: pixels farther than this from any
                          valid sample stay NaN (prevents over-painting sky).

    Returns:
        (H, W) dense float32 depth image (still NaN beyond max_fill_distance).

    Implementation note: scipy's distance_transform_edt(return_indices=True)
    returns, for every pixel, the index of the nearest *background* element.
    To find the nearest VALID pixel we therefore pass the complement mask
    (~valid): valid pixels are the background and map to themselves (distance
    0), while invalid pixels map to their nearest valid neighbor.
    """
    sparse = np.asarray(sparse, dtype=np.float32)
    if sparse.ndim != 2:
        raise ValueError("sparse depth must be 2D, got {}".format(sparse.shape))
    if not HAVE_SCIPY:
        raise RuntimeError("scipy is required for depth completion")
    valid = ~np.isnan(sparse)
    if not valid.any():
        raise ValueError("sparse depth image has no valid pixels")
    if valid.all():
        return sparse.copy()

    distances, indices = ndimage.distance_transform_edt(
        ~valid, return_distances=True, return_indices=True)

    filled = sparse[tuple(indices)]           # nearest valid value everywhere
    if max_fill_distance is not None:
        filled = filled.copy()
        filled[distances > max_fill_distance] = np.nan
    return filled.astype(np.float32)


def smooth_depth(depth: np.ndarray, sigma: float = 2.0,
                 use_cv2: bool = True) -> np.ndarray:
    """
    Smooth a dense depth image.

    Args:
        depth:   (H, W) float32 depth (NaNs preserved).
        sigma:   Gaussian kernel size (px). 0 disables smoothing.
        use_cv2: prefer cv2.GaussianBlur when available.
    """
    if sigma <= 0.0:
        return depth
    valid = ~np.isnan(depth)
    if use_cv2:
        try:
            import cv2
            smoothed = cv2.GaussianBlur(depth, (0, 0), sigmaX=sigma)
            smoothed[~valid] = np.nan
            return smoothed.astype(np.float32)
        except ImportError:
            pass
    # scipy fallback
    from scipy import ndimage
    smoothed = ndimage.gaussian_filter(depth, sigma=sigma)
    smoothed[~valid] = np.nan
    return smoothed.astype(np.float32)


def colorize_depth(depth: np.ndarray, max_depth: float = 80.0
                   ) -> np.ndarray:
    """
    Depth -> BGR jet image for visualization (pure NumPy, no OpenCV needed).

    Returns (H, W, 3) uint8. NaN pixels become black.
    """
    h, w = depth.shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    valid = ~np.isnan(depth)
    clipped = np.clip(depth[valid] / max_depth, 0.0, 1.0)
    t = clipped * 4.0
    r = np.clip(np.minimum(t - 1.5, 4.5 - t) * 255.0, 0, 255)
    g = np.clip(np.minimum(t - 0.5, 3.5 - t) * 255.0, 0, 255)
    b = np.clip(np.minimum(t + 0.5, 2.5 - t) * 255.0, 0, 255)
    # BGR ordering
    out[valid, 0] = b.astype(np.uint8)
    out[valid, 1] = g.astype(np.uint8)
    out[valid, 2] = r.astype(np.uint8)
    return out
