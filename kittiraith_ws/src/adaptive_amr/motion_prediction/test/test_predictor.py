#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_predictor.py — unit tests for motion_prediction.predictor.

Run without ROS:
    python3 test_predictor.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from motion_prediction.predictor import (ConstantVelocityFilter,  # noqa: E402
                                         MotionPredictor)


class TestConstantVelocityFilter(unittest.TestCase):
    def test_constant_velocity_extrapolation(self):
        filt = ConstantVelocityFilter([0.0, 0.0, 0.0], dt=0.1)
        filt.x[3:] = [2.0, 0.0, 0.0]          # 2 m/s along x
        traj, closest = filt.trajectory(2.0, 5)
        # at t=2 s the object should be at x=4
        self.assertAlmostEqual(4.0, traj[-1, 0], places=3)
        self.assertAlmostEqual(0.0, traj[-1, 1], places=3)
        # closest approach is at t=0 (0 m)
        self.assertAlmostEqual(0.0, closest, places=3)

    def test_update_pulls_toward_measurement(self):
        filt = ConstantVelocityFilter([0.0, 0.0, 0.0], dt=0.1)
        for _ in range(5):
            filt.update([10.0, 0.0, 0.0])
        self.assertGreater(filt.x[0], 5.0)     # pulled toward x=10


class TestMotionPredictor(unittest.TestCase):
    def test_track_velocity_estimate(self):
        predictor = MotionPredictor(horizon_s=2.0, steps=5)
        # object moving at 1 m/s in x, measured every 0.5 s
        t = 0.0
        for i in range(5):
            pos = [i * 0.5, 0.0, 0.0]
            _, vel, _ = predictor.update(1, pos, t)
            t += 0.5
        # after a few updates the velocity estimate should be ~1 m/s
        self.assertAlmostEqual(1.0, vel[0], delta=0.5)

    def test_predicted_trajectory_matches_motion(self):
        predictor = MotionPredictor(horizon_s=2.0, steps=4)
        t = 0.0
        for i in range(6):
            predictor.update(7, [i * 0.5, 0.0, 0.0], t)
            t += 0.5
        traj = predictor.predict_trajectory(7)
        self.assertEqual((4, 3), traj.shape)
        # last predicted point should be roughly 2 s ahead of the last meas.
        self.assertGreater(traj[-1, 0], 2.5)

    def test_collision_risk(self):
        predictor = MotionPredictor()
        # a trajectory passing through the origin -> risk 1
        self.assertAlmostEqual(1.0, predictor.collision_risk(0.0, 1.5), places=6)
        # far away -> risk 0
        self.assertAlmostEqual(0.0, predictor.collision_risk(10.0, 1.5), places=6)
        # exactly at the boundary -> 0
        self.assertAlmostEqual(0.0, predictor.collision_risk(1.5, 1.5), places=6)

    def test_prune_removes_stale(self):
        predictor = MotionPredictor()
        predictor.update(1, [0, 0, 0], 0.0)
        predictor.update(2, [0, 0, 0], 0.0)
        predictor.prune(active_ids={1}, keep_age=1.0)   # track 2 is stale
        self.assertIn(1, predictor.filters)
        self.assertNotIn(2, predictor.filters)

    def test_clear(self):
        predictor = MotionPredictor()
        predictor.update(1, [0, 0, 0], 0.0)
        predictor.clear()
        self.assertEqual(0, len(predictor.filters))


if __name__ == "__main__":
    unittest.main(verbosity=2)
