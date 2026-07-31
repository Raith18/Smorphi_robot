#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
icp_localizer.py — ICP scan-to-map localization (pure NumPy/SciPy, unit-tested).

Design (per the project spec: ICP-based, NO EKF):

  * Mapping phase: accumulate downsampled scans (transformed to the odom
    frame via the LiDAR odometry poses) into a global voxel map.
  * Localization phase: register each new scan against the map with
    point-to-plane ICP, using the odometry pose as the initial guess.

Frames:
    map  <--(T_map_odom, ICP correction)-->  odom  <--(T_odom_base, odometry)-->  base

    During mapping,      T_map_odom = I
    After mapping,       T_map_odom = T_corr        (ICP correction)
    Localization pose:   T_map_base = T_map_odom @ T_odom_base
"""

from typing import Optional, Tuple

import numpy as np

from lidar_odometry.icp import estimate_normals, icp_point_to_plane
from lidar_processing.filters import VoxelGrid


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 rigid transform to (N, 3) points."""
    hom = np.hstack([points, np.ones((points.shape[0], 1))])
    return (np.asarray(T, dtype=float) @ hom.T).T[:, :3]


class IcpLocalizer:
    """
    Incremental map building + point-to-plane ICP localization.

    Args:
        voxel_size:          map and scan downsampling leaf [m].
        min_map_points:      mapping phase ends when the map has this many pts.
        max_map_points:      hard cap: map is re-downsampled above this.
        max_mapping_frames:  mapping also ends after this many frames.
        max_corr_dist:       ICP correspondence distance threshold [m].
        max_iterations:      ICP iteration budget.
        max_translation_jump: reject ICP corrections larger than this [m].
    """

    def __init__(self, voxel_size: float = 0.5,
                 min_map_points: int = 60_000,
                 max_map_points: int = 400_000,
                 max_mapping_frames: int = 200,
                 max_corr_dist: float = 2.0,
                 max_iterations: int = 40,
                 max_translation_jump: float = 5.0):
        self.voxel = VoxelGrid(voxel_size)
        self.min_map_points = int(min_map_points)
        self.max_map_points = int(max_map_points)
        self.max_mapping_frames = int(max_mapping_frames)
        self.max_corr_dist = float(max_corr_dist)
        self.max_iterations = int(max_iterations)
        self.max_translation_jump = float(max_translation_jump)

        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.map_normals = np.empty((0, 3), dtype=np.float32)
        self.mapping = True
        self.mapping_frames = 0
        self.T_map_odom = np.eye(4)
        self.last_correction = np.eye(4)
        self.last_error = float("nan")
        self.frames_localized = 0

    # ------------------------------------------------------------------ #
    @property
    def map_ready(self) -> bool:
        """True once mapping finished (map large enough)."""
        return not self.mapping

    # ------------------------------------------------------------------ #
    def reset(self) -> None:
        """Restart mapping from scratch."""
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.map_normals = np.empty((0, 3), dtype=np.float32)
        self.mapping = True
        self.mapping_frames = 0
        self.T_map_odom = np.eye(4)
        self.last_correction = np.eye(4)

    # ------------------------------------------------------------------ #
    def add_scan_to_map(self, scan: np.ndarray, T_odom_base: np.ndarray) -> None:
        """
        Mapping phase: transform the scan into the odom frame and merge it
        into the voxel map.
        """
        if not self.mapping:
            return
        scan_world = transform_points(scan, T_odom_base).astype(np.float32)
        merged = np.vstack([self.map_points, scan_world])
        # re-downsample when the raw map grows past the cap
        if merged.shape[0] > self.max_map_points:
            merged, _ = self.voxel.downsample(merged)
        self.map_points = merged
        self.mapping_frames += 1
        if (self.map_points.shape[0] >= self.min_map_points
                or self.mapping_frames >= self.max_mapping_frames):
            self._finish_mapping()

    # ------------------------------------------------------------------ #
    def _finish_mapping(self) -> None:
        """Final downsample + normals; switch to localization."""
        if self.map_points.shape[0] > 0:
            self.map_points, _ = self.voxel.downsample(self.map_points)
            self.map_normals = estimate_normals(self.map_points,
                                                sensor_origin=np.zeros(3))
        self.mapping = False
        self.T_map_odom = np.eye(4)
        self.last_correction = np.eye(4)

    # ------------------------------------------------------------------ #
    def localize(self, scan: np.ndarray,
                 T_odom_base: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Localization phase: ICP the scan (in the odom frame via the odometry
        prior) against the map.

        Returns:
            T_map_odom: correction transform.
            T_map_base: refined pose of the base in the map frame.
            error:      mean point-to-plane error of the registration.
        """
        if self.mapping or self.map_points.shape[0] < 100:
            # not localized yet -> identity correction
            self.frames_localized += 1
            return np.eye(4), T_odom_base, float("nan")

        scan_odom = transform_points(scan, T_odom_base)
        T_corr, error = icp_point_to_plane(
            scan_odom, self.map_points, self.map_normals,
            init=self.last_correction,          # warm start
            max_iterations=self.max_iterations,
            max_correspondence_distance=self.max_corr_dist)

        # ---- sanity: reject implausible jumps ----------------------------------
        delta_t = np.linalg.norm(T_corr[:3, 3] - self.last_correction[:3, 3])
        if delta_t > self.max_translation_jump:
            T_corr = self.last_correction.copy()
            rospy_log_warn("localizer rejected implausible ICP jump (%.2f m)",
                           delta_t)

        self.last_correction = T_corr
        self.T_map_odom = T_corr
        self.last_error = error
        self.frames_localized += 1
        T_map_base = T_corr @ T_odom_base
        return T_corr, T_map_base, error

    # ------------------------------------------------------------------ #
    def get_map(self) -> np.ndarray:
        """Current map points (N, 3)."""
        return self.map_points


def rospy_log_warn(message: str, *args) -> None:
    """Logging hook (no-op outside ROS; node injects rospy.logwarn)."""
    try:
        import rospy
        rospy.logwarn_throttle(5.0, message, *args)
    except ImportError:  # pragma: no cover
        pass
