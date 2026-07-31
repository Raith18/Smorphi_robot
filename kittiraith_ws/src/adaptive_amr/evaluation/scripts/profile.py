#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
profile.py — CPU latency profiler for the pure algorithm cores.

Measures wall-clock latency of every CPU-heavy primitive on synthetic data
(no ROS, no GPU), printing a markdown table and writing a CSV. Run it on ANY
machine — including this development sandbox — to get honest numbers:

    python3 evaluation/scripts/profile.py [--repeats 5] [--csv profile.csv]

Results feed the Phase 8 optimization report (docs/phase8_benchmarking.md).
"""

import argparse
import csv
import os
import sys
import time
from typing import Callable, Dict, List, Tuple

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src"))
_AMR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "..", "..", "..", ".."))
for _pkg in ("lidar_odometry", "lidar_processing", "sensor_fusion",
             "depth_estimation", "object_tracking", "occupancy_grid",
             "navigation_layer", "semantic_mapping", "motion_prediction"):
    _p = os.path.join(_AMR, "src", "adaptive_amr", _pkg, "src")
    if _p not in sys.path:
        sys.path.insert(0, _p)


def timed(fn: Callable, repeats: int = 5) -> float:
    """Median wall-clock latency of fn() over `repeats` runs [ms]."""
    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    return float(np.median(times))


# --------------------------------------------------------------------------- #
# Synthetic data builders
# --------------------------------------------------------------------------- #
def make_cloud(n: int, seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.random((n, 3)) * 30.0


def make_sparse_depth(h=376, w=1242, points=15000):
    rng = np.random.default_rng(1)
    depth = np.full((h, w), np.nan, dtype=np.float32)
    u = rng.integers(0, w, points)
    v = rng.integers(0, h, points)
    depth[v, u] = rng.uniform(1.0, 80.0, points).astype(np.float32)
    return depth


def make_grid(size=300):
    grid = np.zeros((size, size), dtype=np.int8)
    grid[:, size // 2] = 100          # a wall
    grid[size // 2 - 5:size // 2 + 5, ::7] = 100
    return grid


# --------------------------------------------------------------------------- #
# Benchmark definitions: (label, setup, call)
# --------------------------------------------------------------------------- #
def collect() -> List[Tuple[str, float]]:
    results: List[Tuple[str, float]] = []

    # --- LiDAR: voxel downsample -------------------------------------------------
    cloud100k = make_cloud(100_000)
    from lidar_processing.filters import VoxelGrid
    voxel = VoxelGrid(0.3)
    results.append(("voxel_downsample_100k",
                    timed(lambda: voxel.downsample(cloud100k))))

    # --- LiDAR: RANSAC ground -----------------------------------------------------
    from lidar_processing.ground_segmentation import RansacPlaneSegmenter
    seg = RansacPlaneSegmenter(0.2, 50)
    results.append(("ransac_ground_30k",
                    timed(lambda: seg.fit(make_cloud(30_000, seed=2)))))

    # --- LiDAR: Euclidean clustering -----------------------------------------------
    from lidar_processing.clustering import EuclideanClusterExtraction
    clusterer = EuclideanClusterExtraction(0.5, 10)
    blobs = np.vstack([make_cloud(4000, seed=i) + np.array([i * 4, 0, 0])
                       for i in range(5)])
    results.append(("euclidean_cluster_20k",
                    timed(lambda: clusterer.extract(blobs))))

    # --- ICP point-to-plane ---------------------------------------------------------
    from lidar_odometry.icp import icp_point_to_plane
    src = make_cloud(20_000, seed=3)
    tgt = src + np.array([0.3, -0.2, 0.05])
    results.append(("icp_point_to_plane_20k",
                    timed(lambda: icp_point_to_plane(src, tgt,
                                                     max_iterations=20))))

    # --- Sensor fusion projection -----------------------------------------------------
    from sensor_fusion.projection import LidarCameraProjection
    P = np.array([[721.5, 0, 609.5, -389.5], [0, 721.5, 172.8, 0],
                  [0, 0, 1, 0]])
    proj = LidarCameraProjection(P, np.eye(3), np.eye(4), 1242, 376)
    results.append(("projection_100k",
                    timed(lambda: proj.project(make_cloud(100_000, seed=4)))))

    # --- Depth completion (EDT fill) ---------------------------------------------------
    from depth_estimation.depth_completion import nearest_fill
    sparse = make_sparse_depth()
    results.append(("depth_completion_1242x376",
                    timed(lambda: nearest_fill(sparse))))

    # --- SORT update ------------------------------------------------------------------
    from object_tracking.sort import SortTracker
    tracker = SortTracker(max_age=4, min_hits=1, iou_threshold=0.3)
    dets = np.array([[100 + i * 2, 100, 120 + i * 2, 140] for i in range(10)])

    def sort_update():
        tracker.update(dets + np.random.randn(10, 4) * 0.5)
    results.append(("sort_update_10_tracks", timed(sort_update, 10)))

    # --- Occupancy grid ray-cast ---------------------------------------------------------
    from occupancy_grid.grid_mapping import OccupancyGridMapper
    mapper = OccupancyGridMapper(0.2, 60, 60)
    pts = make_cloud(20_000, seed=5)[:, :2] * 1.0
    results.append(("occupancy_raycast_20k",
                    timed(lambda: mapper.add_scan(pts, 0.0, 0.0))))

    # --- Costmap inflation ------------------------------------------------------------------
    from navigation_layer.costmap import CostmapBuilder
    builder = CostmapBuilder(1.0)
    occ = make_grid()
    results.append(("costmap_inflation_300x300",
                    timed(lambda: builder.build(occ, 0.2))))

    # --- A* planning ---------------------------------------------------------------------------
    from navigation_layer.planner import AStarPlanner
    planner = AStarPlanner(make_grid(), 0.2, -30.0, -30.0)
    results.append(("astar_300x300",
                    timed(lambda: planner.plan((0.0, 0.0), (20.0, -10.0)))))

    # --- Semantic map merge ----------------------------------------------------------------------
    from semantic_mapping.map_builder import SemanticMapBuilder
    builder_map = SemanticMapBuilder(0.2)
    pts = make_cloud(20_000, seed=6)
    rgb = np.random.default_rng(7).integers(0, 255, (20_000, 3)).astype(np.uint8)
    results.append(("semantic_map_merge_20k",
                    timed(lambda: builder_map.add_frame(pts, rgb, np.eye(4)))))

    # --- Motion prediction (20 tracks) -----------------------------------------------------------------
    from motion_prediction.predictor import MotionPredictor
    predictor = MotionPredictor()
    t = 0.0

    def pred_update():
        nonlocal t
        t += 0.1
        for tid in range(20):
            predictor.update(tid, [t * 0.5, tid * 0.1, 0.0], t)
    results.append(("motion_prediction_20_tracks", timed(pred_update, 10)))

    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description="CPU latency profiler")
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--csv", default="")
    args = parser.parse_args(argv)

    print("# kittiraith_ws — CPU latency profile")
    print("({} repeats, median ms)\n".format(args.repeats))
    print("| module / algorithm | latency [ms] |")
    print("|---|---|")

    results = collect()
    rows = []
    for label, ms in results:
        print("| {} | {:.2f} |".format(label, ms))
        rows.append({"module": label, "latency_ms": ms})

    if args.csv:
        with open(args.csv, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["module", "latency_ms"])
            writer.writeheader()
            writer.writerows(rows)
        print("\nCSV written to {}".format(args.csv))
    return 0


if __name__ == "__main__":
    sys.exit(main())
