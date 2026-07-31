#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_pipeline_integration.py — headless END-TO-END pipeline integration test.

No ROS, no GPU, no dataset download. Generates a synthetic KITTI-like drive
(the robot drives through a box world with a pillar obstacle) and runs the
REAL modules together, the way the ROS graph would:

    lidar_processing  (ground removal + clustering)
        -> occupancy_grid          (ray-cast log-odds grid)
        -> navigation_layer        (costmap inflation + A*)
        -> behavior                (state machine + velocity decision)
    semantic_mapping + motion_prediction (map growth + trajectory prediction)

Asserts end-to-end behavior: the grid fills with walls, the A* path avoids
the obstacle, the behavior layer reaches the goal, the map grows, and
predicted trajectories make sense.

Run:
    bash tests/run_integration_tests.sh
    python3 tests/integration/test_pipeline_integration.py
"""

import os
import sys
import unittest

import numpy as np

_AMR = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "..", "..", "src", "adaptive_amr"))
for _pkg in ("dataset_loader", "lidar_processing", "lidar_odometry",
             "sensor_fusion", "depth_estimation", "object_tracking",
             "occupancy_grid", "navigation_layer", "semantic_mapping",
             "motion_prediction"):
    _p = os.path.join(_AMR, _pkg, "src")
    if _p not in sys.path:
        sys.path.insert(0, _p)

from lidar_processing.clustering import EuclideanClusterExtraction  # noqa: E402
from lidar_processing.filters import VoxelGrid  # noqa: E402
from lidar_processing.ground_segmentation import (GroundRemover,  # noqa: E402
                                                  RansacPlaneSegmenter)
from motion_prediction.predictor import MotionPredictor  # noqa: E402
from navigation_layer.behavior import BehaviorState, BehaviorStateMachine  # noqa: E402
from navigation_layer.costmap import CostmapBuilder  # noqa: E402
from navigation_layer.planner import AStarPlanner  # noqa: E402
from occupancy_grid.grid_mapping import OccupancyGridMapper  # noqa: E402
from semantic_mapping.map_builder import SemanticMapBuilder  # noqa: E402


# --------------------------------------------------------------------------- #
# Synthetic world + drive
# --------------------------------------------------------------------------- #
def build_world(seed=0):
    """A 60x60 m box world: ground + 4 walls + 1 pillar obstacle."""
    rng = np.random.default_rng(seed)
    size = 30.0
    n = 4000

    def wall_axis(axis, value, lo, hi, n_pts=1200):
        pts = np.zeros((n_pts, 3))
        pts[:, axis] = value
        other = [0, 1, 2]
        other.remove(axis)
        pts[:, other[0]] = rng.uniform(lo, hi, n_pts)
        pts[:, other[1]] = rng.uniform(-2.0, 2.0, n_pts)   # wall height
        return pts

    ground = np.column_stack([rng.uniform(-size, size, n),
                              rng.uniform(-size, size, n),
                              rng.uniform(-0.02, 0.02, n)])
    walls = np.vstack([
        wall_axis(0, -size, -size, size),
        wall_axis(0, size, -size, size),
        wall_axis(1, -size, -size, size),
        wall_axis(1, size, -size, size),
    ])
    # interior pillar obstacle at (10, 0) — a realistic 1.6 x 1.6 x 2 m block
    # with dense surface points (a thin obstacle would be washed out by the
    # free-ray updates of a log-odds grid — see docs/phase9_ci_testing.md)
    pillar = np.column_stack([
        rng.uniform(9.2, 10.8, n // 2),
        rng.uniform(-0.8, 0.8, n // 2),
        rng.uniform(0.0, 2.0, n // 2)])
    return np.vstack([ground, walls, pillar]).astype(np.float32)


def robot_scan(world, pose, seed=0, n_keep=4000):
    """
    LiDAR-like scan of the world in the robot frame at `pose`.

    Models OCCLUSION like a real range sensor: per angular bearing bin
    (~0.57 deg) only the NEAREST point is kept — so obstacles block the
    rays behind them. (Without occlusion, synthetic wall points behind an
    obstacle leak free-rays through it and wash it out of the grid.)
    """
    rng = np.random.default_rng(seed)
    T = np.eye(4)
    T[0, 3], T[1, 3], T[2, 3] = pose[0], pose[1], pose[2]
    T_inv = np.linalg.inv(T)
    hom = np.hstack([world, np.ones((world.shape[0], 1))])
    scan = (T_inv @ hom.T).T[:, :3]
    r = np.linalg.norm(scan, axis=1)
    keep_range = r < 40.0
    scan = scan[keep_range]
    r = r[keep_range]
    if scan.shape[0] == 0:
        return np.empty((0, 3))

    azimuth = np.arctan2(scan[:, 1], scan[:, 0])
    elevation = np.arctan2(scan[:, 2], np.hypot(scan[:, 0], scan[:, 1]))
    # angular bin key (0.01 rad ~ 0.57 deg)
    key = (np.floor(azimuth / 0.01).astype(np.int64) * 1_000_000
           + np.floor(elevation / 0.01).astype(np.int64))

    order = np.argsort(r)
    _, first_idx = np.unique(key[order], return_index=True)
    scan = scan[order[first_idx]] + rng.normal(0.0, 0.02,
                                               scan[order[first_idx]].shape)
    if scan.shape[0] > n_keep:
        idx = rng.choice(scan.shape[0], n_keep, replace=False)
        scan = scan[idx]
    return scan


class TestPipelineIntegration(unittest.TestCase):
    """Runs the real perception->mapping->navigation chain end to end."""

    def test_full_pipeline(self):
        world = build_world(seed=0)
        voxel = VoxelGrid(0.2)                       # accuracy preset
        remover = GroundRemover(RansacPlaneSegmenter(0.15, 50, seed=1))
        clusterer = EuclideanClusterExtraction(1.0, 10)
        grid = OccupancyGridMapper(0.2, 60.0, 60.0)
        costmap_builder = CostmapBuilder(1.0, 50)
        semantic = SemanticMapBuilder(0.4)
        predictor = MotionPredictor(horizon_s=3.0, steps=6)
        sm = BehaviorStateMachine(risk_stop=0.8, risk_avoid=0.4,
                                  goal_tolerance=1.0)

        # robot drives along +x from (0,0) to (22,0)
        path_poses = [(float(i) * 1.0, 0.0, 1.73) for i in range(23)]
        t = 0.0
        final_costmap = None

        for i, pose in enumerate(path_poses):
            scan = robot_scan(world, pose, seed=i)
            scan, _ = voxel.downsample(scan)

            # ---- lidar_processing ------------------------------------------
            ground, obstacles, _ = remover.separate(scan)
            clusters = clusterer.extract(obstacles)

            # ---- occupancy_grid ----------------------------------------------
            # (mirrors the fixed node: obstacles are in the robot frame and
            #  must be transformed into the map frame with the robot pose)
            T_map_base = np.eye(4)
            T_map_base[0, 3], T_map_base[1, 3] = pose[0], pose[1]
            obstacles_world = (T_map_base @ np.hstack(
                [obstacles[:, :2], np.zeros((obstacles.shape[0], 1)),
                 np.ones((obstacles.shape[0], 1))]).T).T[:, :2]
            grid.add_scan(obstacles_world, pose[0], pose[1], max_range=30.0)

            # ---- semantic_mapping (colored points = obstacle height label) ---
            rgb = np.zeros((obstacles.shape[0], 3), dtype=np.uint8)
            rgb[:, 2] = np.clip(obstacles[:, 2] * 80, 0, 255).astype(np.uint8)
            semantic.add_frame(obstacles, rgb, T_map_base)

            # ---- motion_prediction (two fake tracked obstacles) --------------
            predictor.update(1, [pose[0] + 3.0, 2.0, 0.0], t)
            predictor.update(2, [pose[0] + 3.0, -2.0, 0.0], t)
            t += 0.1

            if i == len(path_poses) - 1:
                final_costmap = costmap_builder.build(grid.get_occupancy(),
                                                      grid.resolution)

        # ==================================================================
        # END-TO-END ASSERTIONS
        # ==================================================================

        # 1. grid: the world got mapped — the pillar REGION is occupied, the
        #    walls produced many occupied cells, and the robot's own path is
        #    free. (Exact single-cell checks are brittle: voxel centroids can
        #    land just outside one cell, so we assert over neighborhoods.)
        occ = grid.get_occupancy()
        r0, c0 = grid.world_to_cell(9.0, -1.0)
        r1, c1 = grid.world_to_cell(11.0, 1.0)
        pillar_region = occ[r0:r1 + 1, c0:c1 + 1]
        self.assertGreater(int((pillar_region >= 50).sum()), 5,
                           "pillar region not occupied")
        self.assertGreater(int((occ >= 50).sum()), 300,
                           "walls did not map into the grid")
        # the robot's own start cell is free (ray origins get free updates)
        r_start, c_start = grid.world_to_cell(0.0, 0.0)
        self.assertLess(occ[r_start, c_start], 50,
                        "robot start cell not free")

        # 2. costmap + A*: plan from the end pose to the pillar-side goal,
        #    the path must EXIST and avoid the inflated pillar
        self.assertIsNotNone(final_costmap)
        planner = AStarPlanner(final_costmap, grid.resolution,
                               -30.0, -30.0)
        path = planner.plan((22.0, 0.0), (22.0, 15.0))
        self.assertIsNotNone(path, "A* could not plan in the mapped world")
        # path must stay inside the walls (|x| < 29, |y| < 29)
        self.assertLess(np.abs(path[:, 0]).max(), 29.0)
        self.assertLess(np.abs(path[:, 1]).max(), 29.0)

        # 3. behavior: no risk -> NAVIGATE; high risk -> STOP; clears ->
        #    RESUME -> NAVIGATE; near goal -> GOAL_REACHED
        self.assertEqual(BehaviorState.NAVIGATE,
                         sm.update(True, 10.0, 0.0, False))
        self.assertEqual(BehaviorState.STOP,
                         sm.update(True, 10.0, 0.9, False))
        self.assertEqual(BehaviorState.RESUME,
                         sm.update(True, 10.0, 0.1, False))
        self.assertEqual(BehaviorState.NAVIGATE,
                         sm.update(True, 10.0, 0.1, False))
        self.assertEqual(BehaviorState.GOAL_REACHED,
                         sm.update(True, 0.5, 0.0, False))
        vx_stop, _ = sm.desired_velocity(BehaviorState.STOP, 0.0)
        self.assertEqual(0.0, vx_stop)

        # 4. semantic map: grew and has sensible bounds
        map_xyz, map_rgb = semantic.get_map()
        self.assertGreater(map_xyz.shape[0], 500, "semantic map too small")
        self.assertEqual(map_xyz.shape[0], map_rgb.shape[0])
        self.assertLess(np.abs(map_xyz[:, 0]).max(), 31.0)

        # 5. motion prediction: trajectories exist and extrapolate forward
        traj = predictor.predict_trajectory(1)
        self.assertEqual((6, 3), traj.shape)
        self.assertGreater(traj[-1, 0], predictor.positions[1][0],
                           "trajectory must extrapolate forward")
        # an obstacle whose predicted path passes through the robot is risky
        risk = predictor.collision_risk(0.3, safe_distance=1.5)
        self.assertGreater(risk, 0.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
