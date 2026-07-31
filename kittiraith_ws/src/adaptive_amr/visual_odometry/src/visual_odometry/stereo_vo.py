#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
stereo_vo.py — classic stereo visual odometry (viso2-style).

Pipeline per frame pair (left, right), all in the *rectified* camera frame
(KITTI: images are already rectified, distortion = 0):

  1. Detect Shi-Tomasi corners in the left image.
  2. KLT track:  left_prev -> left_cur (temporal),
                 right_prev -> right_cur (temporal),
                 left_cur -> right_cur (stereo, epipolar ~ horizontal).
  3. Triangulate the stereo pair with DLT -> metric 3D points.
  4. PnP (RANSAC) from the PREVIOUS frame's 3D points to the CURRENT left 2D
     points -> relative motion T_cur_prev.
  5. Accumulate: T_w_cur = T_w_prev * inv(T_cur_prev).
  6. Replenish features so the track count stays healthy.

The scale is metric because stereo triangulation is metric (fixed baseline).

This class only depends on numpy + OpenCV (no ROS), so the math is testable
on any host with synthetic projections.
"""

from typing import Dict, Optional

import numpy as np

try:
    import cv2
    HAVE_CV2 = True
except ImportError:  # pragma: no cover
    HAVE_CV2 = False

from visual_odometry.geometry_utils import (invert_pose, rvec_tvec_to_matrix,
                                            triangulate_many)


class StereoVisualOdometry:
    """
    Stereo VO engine.

    Args:
        P_left, P_right: 3x4 rectified projection matrices (KITTI P_rect_0X).
        max_features:   corner budget per frame.
        quality_level:  Shi-Tomasi quality (0.0-1.0).
        min_distance:   minimum px spacing between corners.
        lk_win:         KLT search window size.
        lk_max_level:   KLT pyramid levels.
        min_inliers:    minimum PnP inliers; below this the frame is skipped.
        ransac_reproj:  PnP RANSAC reprojection threshold [px].
    """

    def __init__(self, P_left: np.ndarray, P_right: np.ndarray,
                 max_features: int = 1500, quality_level: float = 0.01,
                 min_distance: int = 10,
                 lk_win: int = 21, lk_max_level: int = 3,
                 min_inliers: int = 15, ransac_reproj: float = 3.0):
        if not HAVE_CV2:
            raise RuntimeError("OpenCV (cv2) is required for stereo VO")
        self.P_left = np.asarray(P_left, dtype=float)
        self.P_right = np.asarray(P_right, dtype=float)
        self.max_features = int(max_features)
        self.quality_level = float(quality_level)
        self.min_distance = int(min_distance)
        self.lk_win = (int(lk_win), int(lk_win))
        self.lk_max_level = int(lk_max_level)
        self.min_inliers = int(min_inliers)
        self.ransac_reproj = float(ransac_reproj)

        self.lk_params = dict(winSize=self.lk_win,
                              maxLevel=self.lk_max_level,
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                                        30, 0.01))
        self.lk_params.setdefault("minEigThreshold", 1e-4)

        # state: previous frame data (None until the first frame)
        self.prev_left: Optional[np.ndarray] = None
        self.prev_right: Optional[np.ndarray] = None
        self.prev_pts_left: Optional[np.ndarray] = None
        self.prev_pts_right: Optional[np.ndarray] = None
        self.prev_3d: Optional[np.ndarray] = None
        self.T_w_prev: Optional[np.ndarray] = None     # world -> camera (4x4)
        self.frames = 0
        self.last_inliers = 0

    # ------------------------------------------------------------------ #
    def process(self, left: np.ndarray, right: np.ndarray) -> Optional[np.ndarray]:
        """
        Process one stereo pair (uint8 grayscale or BGR).

        Returns the camera pose in the world frame T_w_cam (4x4) or None on
        the first frame / degenerate frame.
        """
        left = self._to_gray(left)
        right = self._to_gray(right)

        if self.prev_left is None:
            self._initialize(left, right)
            return None

        # ---- 1. temporal tracking ------------------------------------------
        pts_l_cur, status_l = self._klt(self.prev_left, left, self.prev_pts_left)
        pts_r_cur, status_r = self._klt(self.prev_right, right, self.prev_pts_right)
        temporal_ok = status_l & status_r

        # ---- 2. stereo matching in the current frame --------------------------
        pts_r_stereo, status_s = self._klt(left, right, pts_l_cur)
        stereo_ok = status_s

        keep = temporal_ok & stereo_ok
        if keep.sum() < self.min_inliers:
            # degenerate frame: keep pose, re-initialize from the current pair
            self._initialize(left, right)
            return self.T_w_prev

        p3d_prev = self.prev_3d[keep]                 # 3D from previous pair
        pts2d_cur = pts_l_cur[keep]                    # matched in current left

        # ---- 3. PnP: previous 3D -> current 2D --------------------------------
        K = self.P_left[:3, :3]
        obj = p3d_prev.reshape(-1, 1, 3).astype(np.float64)
        img = pts2d_cur.reshape(-1, 1, 2).astype(np.float64)
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj, img, K, None, reprojectionError=self.ransac_reproj,
            iterationsCount=200, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success or (inliers is not None and len(inliers) < self.min_inliers):
            self._initialize(left, right)
            return self.T_w_prev
        self.last_inliers = len(inliers) if inliers is not None else 0

        # T_cur_prev: maps previous camera coords to current camera coords
        T_cur_prev = rvec_tvec_to_matrix(rvec, tvec)
        T_w_cur = self.T_w_prev @ invert_pose(T_cur_prev)

        # ---- 4. triangulate the current pair (for the NEXT frame) ---------------
        pts_r_stereo = pts_r_stereo.reshape(-1, 2)
        p3d_cur = triangulate_many(self.P_left, self.P_right,
                                   pts_l_cur, pts_r_stereo)
        valid3d = (p3d_cur[:, 2] > 0.1) & np.all(np.isfinite(p3d_cur), axis=1)

        # ---- 5. replenish features -----------------------------------------------
        new_l, new_r, new_3d = self._detect_and_match(left, right)
        if new_l is not None:
            pts_l_cur = np.vstack([pts_l_cur, new_l])
            pts_r_stereo = np.vstack([pts_r_stereo, new_r])
            p3d_cur = np.vstack([p3d_cur, new_3d])
            valid3d = np.concatenate([valid3d, np.ones(new_3d.shape[0], dtype=bool)])

        self.prev_left, self.prev_right = left, right
        self.prev_pts_left = pts_l_cur[valid3d]
        self.prev_pts_right = pts_r_stereo[valid3d]
        self.prev_3d = p3d_cur[valid3d]
        self.T_w_prev = T_w_cur
        self.frames += 1
        return T_w_cur

    # ------------------------------------------------------------------ #
    def _initialize(self, left, right) -> None:
        """First frame: detect + stereo match + triangulate."""
        pts_l, pts_r, p3d = self._detect_and_match(left, right)
        self.prev_left, self.prev_right = left, right
        self.prev_pts_left = pts_l
        self.prev_pts_right = pts_r
        self.prev_3d = p3d
        self.T_w_prev = np.eye(4)
        self.frames = 0

    # ------------------------------------------------------------------ #
    def _detect_and_match(self, left, right):
        """Detect Shi-Tomasi corners in left, KLT-match to right, triangulate."""
        corners = cv2.goodFeaturesToTrack(
            left, maxCorners=self.max_features, qualityLevel=self.quality_level,
            minDistance=self.min_distance)
        if corners is None or len(corners) < 4:
            return None, None, None
        pts_l = corners.reshape(-1, 2)
        pts_r, status = self._klt(left, right, pts_l)
        ok = status
        if ok.sum() < 4:
            return None, None, None
        pts_l, pts_r = pts_l[ok], pts_r[ok]
        p3d = triangulate_many(self.P_left, self.P_right, pts_l, pts_r)
        valid = (p3d[:, 2] > 0.1) & np.all(np.isfinite(p3d), axis=1)
        if valid.sum() < 4:
            return None, None, None
        return pts_l[valid], pts_r[valid], p3d[valid]

    # ------------------------------------------------------------------ #
    def _klt(self, prev_img, cur_img, prev_pts):
        """Lucas-Kanade optical flow; returns (tracked_pts, ok_mask)."""
        if prev_pts is None or prev_pts.shape[0] == 0:
            return (np.zeros((0, 2), dtype=np.float32),
                    np.zeros(0, dtype=bool))
        pts, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_img, cur_img, prev_pts.reshape(-1, 1, 2).astype(np.float32),
            None, **self.lk_params)
        if pts is None:
            return prev_pts, np.zeros(prev_pts.shape[0], dtype=bool)
        ok = status.reshape(-1).astype(bool)
        return pts.reshape(-1, 2), ok

    # ------------------------------------------------------------------ #
    @staticmethod
    def _to_gray(image: np.ndarray) -> np.ndarray:
        """Convert BGR/RGB to grayscale; pass through already-gray images."""
        if image.ndim == 2:
            return image
        if image.shape[2] == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image[:, :, 0]
