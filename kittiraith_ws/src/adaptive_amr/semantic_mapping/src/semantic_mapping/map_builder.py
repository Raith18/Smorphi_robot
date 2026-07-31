#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
map_builder.py — global semantic point-cloud map builder (pure NumPy, tested).

Accumulates semantic-colored LiDAR frames into a global voxel map:

    p_map = T_map_base . p_base
    voxel = floor(p_map / leaf)
    per voxel: average position, dominant (most frequent) color

The dominant-color rule keeps the map stable: transient mislabels (a frame
where a car pixel flickers) cannot repaint a persistent voxel.

Frames:
    map  <-- T_map_base (localization pose) --  base  <-- ... --  sensor
"""

from typing import Tuple

import numpy as np


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 rigid transform to (N, 3) points."""
    hom = np.hstack([points, np.ones((points.shape[0], 1))])
    return (np.asarray(T, dtype=float) @ hom.T).T[:, :3]


class SemanticMapBuilder:
    """
    Incremental voxel map of semantic-colored points.

    Args:
        voxel_size:  map voxel leaf [m].
        max_voxels:  hard cap: when exceeded, the map is re-downsampled
                     (doubles the leaf) so memory stays bounded.
    """

    def __init__(self, voxel_size: float = 0.2, max_voxels: int = 400_000):
        if voxel_size <= 0.0:
            raise ValueError("voxel_size must be > 0")
        self.voxel_size = float(voxel_size)
        self.max_voxels = int(max_voxels)
        self._xyz = np.empty((0, 3), dtype=np.float32)
        self._rgb = np.empty((0, 3), dtype=np.uint8)
        self._voxel_key = np.empty((0, 3), dtype=np.int64)

    # ------------------------------------------------------------------ #
    def add_frame(self, xyz: np.ndarray, rgb: np.ndarray,
                  T_map_base: np.ndarray) -> int:
        """
        Add one semantic-colored cloud (in the base frame) to the map.

        Args:
            xyz: (N, 3) points in the base frame.
            rgb: (N, 3) uint8 colors (semantic labels already painted).
            T_map_base: 4x4 pose of the base in the map frame.

        Returns:
            number of new voxels added.
        """
        xyz = np.asarray(xyz, dtype=np.float32)
        rgb = np.asarray(rgb, dtype=np.uint8)
        if xyz.shape[0] == 0:
            return 0
        if xyz.shape[0] != rgb.shape[0]:
            raise ValueError("xyz and rgb must have the same length")

        world = transform_points(xyz, T_map_base).astype(np.float32)
        keys = np.floor(world / self.voxel_size).astype(np.int64)

        all_keys = np.vstack([self._voxel_key, keys])
        all_xyz = np.vstack([self._xyz, world])
        all_rgb = np.vstack([self._rgb, rgb])

        # unique voxel keys -> first-occurrence index per voxel
        _, first_idx, inverse = np.unique(
            all_keys, axis=0, return_index=True, return_inverse=True)

        voxel_xyz = np.zeros((len(first_idx), 3), dtype=np.float64)
        np.add.at(voxel_xyz, inverse, all_xyz)
        counts = np.bincount(inverse, minlength=len(first_idx))
        voxel_xyz = (voxel_xyz / counts[:, None]).astype(np.float32)

        # dominant color per voxel (most frequent rgb value)
        voxel_rgb = self._dominant_color(all_rgb, inverse, len(first_idx))

        self._xyz = voxel_xyz
        self._rgb = voxel_rgb
        self._voxel_key = all_keys[first_idx]

        if self._xyz.shape[0] > self.max_voxels:
            self._coarsen()
        return len(first_idx)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _dominant_color(rgb: np.ndarray, inverse: np.ndarray,
                        n_voxels: int) -> np.ndarray:
        """Most frequent color per voxel (ties -> lowest index, deterministic)."""
        packed = (rgb[:, 0].astype(np.uint32) << 16 |
                  rgb[:, 1].astype(np.uint32) << 8 |
                  rgb[:, 2].astype(np.uint32))
        order = np.argsort(packed, kind="stable")
        packed_sorted = packed[order]
        inverse_sorted = inverse[order]
        # first occurrence of each packed color per voxel group
        unique_key = np.stack([inverse_sorted, packed_sorted], axis=1)
        _, first_idx = np.unique(unique_key, axis=0, return_index=True)
        best_packed = np.full(n_voxels, -1, dtype=np.int64)
        best_packed[inverse_sorted[first_idx]] = packed_sorted[first_idx]
        r = (best_packed >> 16) & 0xFF
        g = (best_packed >> 8) & 0xFF
        b = best_packed & 0xFF
        return np.stack([r, g, b], axis=1).astype(np.uint8)

    # ------------------------------------------------------------------ #
    def _coarsen(self) -> None:
        """Double the voxel size and re-voxelize (memory bound)."""
        self.voxel_size *= 2.0
        self._voxel_key = np.floor(self._xyz / self.voxel_size).astype(np.int64)
        _, first_idx, inverse = np.unique(
            self._voxel_key, axis=0, return_index=True, return_inverse=True)
        xyz = np.zeros((len(first_idx), 3), dtype=np.float64)
        np.add.at(xyz, inverse, self._xyz)
        counts = np.bincount(inverse, minlength=len(first_idx))
        self._xyz = (xyz / counts[:, None]).astype(np.float32)
        self._rgb = self._dominant_color(self._rgb, inverse, len(first_idx))
        self._voxel_key = self._voxel_key[first_idx]

    # ------------------------------------------------------------------ #
    def get_map(self) -> Tuple[np.ndarray, np.ndarray]:
        """(xyz (N,3) float32, rgb (N,3) uint8) of the current map."""
        return self._xyz, self._rgb

    # ------------------------------------------------------------------ #
    def clear(self) -> None:
        """Reset the map."""
        self._xyz = np.empty((0, 3), dtype=np.float32)
        self._rgb = np.empty((0, 3), dtype=np.uint8)
        self._voxel_key = np.empty((0, 3), dtype=np.int64)
