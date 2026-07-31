#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_metrics.py — unit tests for evaluation.metrics.

Run without ROS:
    python3 test_metrics.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from evaluation.metrics import (absolute_trajectory_error,  # noqa: E402
                                grid_precision_recall, mot_metrics,
                                relative_pose_error, segmentation_iou,
                                umeyama_alignment)


def straight_trajectory(n=200, step=0.1):
    """GT: robot driving along +x."""
    poses = []
    for i in range(n):
        m = np.eye(4)
        m[0, 3] = i * step
        poses.append(m)
    return poses


class TestUmeyama(unittest.TestCase):
    def test_recovers_transform(self):
        rng = np.random.default_rng(0)
        src = rng.random((50, 3)) * 10.0
        R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])   # 90 deg z
        t = np.array([5.0, -3.0, 2.0])
        dst = (R @ src.T).T + t
        r, tv, s = umeyama_alignment(src, dst, with_scale=False)
        np.testing.assert_allclose(R, r, atol=1e-9)
        np.testing.assert_allclose(t, tv, atol=1e-9)
        self.assertAlmostEqual(1.0, s, places=9)


class TestAte(unittest.TestCase):
    def test_perfect_zero(self):
        gt = straight_trajectory(100)
        ate = absolute_trajectory_error(gt, gt)
        self.assertLess(ate["rmse_m"], 1e-9)

    def test_drift_detected_without_scale(self):
        # LINEAR drift is absorbed by Umeyama scale alignment (classic TUM
        # behavior) — use with_scale=False to expose it, or nonlinear drift.
        gt = straight_trajectory(100)
        est = []
        for i, m in enumerate(gt):
            e = m.copy()
            e[0, 3] += 0.01 * i          # growing 1 cm/frame drift
            est.append(e)
        ate = absolute_trajectory_error(est, gt, with_scale=False)
        # rigid alignment splits the 0..1 m linear drift -> mean ~ 0.25 m
        self.assertGreater(ate["mean_m"], 0.2)
        self.assertLess(ate["mean_m"], 0.4)

    def test_scale_alignment_absorbs_linear_drift(self):
        # teaching check: with with_scale=True the linear drift is absorbed
        gt = straight_trajectory(100)
        est = []
        for i, m in enumerate(gt):
            e = m.copy()
            e[0, 3] += 0.01 * i
            est.append(e)
        ate = absolute_trajectory_error(est, gt, with_scale=True)
        self.assertLess(ate["rmse_m"], 0.1)


class TestRpe(unittest.TestCase):
    def test_perfect_zero(self):
        gt = straight_trajectory(300)
        rpe = relative_pose_error(gt, gt, length=100)
        self.assertLess(rpe["t_mean_m"], 1e-9)
        self.assertLess(rpe["r_mean_deg"], 1e-9)

    def test_translation_drift(self):
        gt = straight_trajectory(300)
        est = []
        for i, m in enumerate(gt):
            e = m.copy()
            e[0, 3] += 0.05 * i
            est.append(e)
        rpe = relative_pose_error(est, gt, length=100)
        self.assertGreater(rpe["t_mean_m"], 1.0)

    def test_rotation_error(self):
        # growing yaw error per frame: R_i = Rz(0.01 * i) @ M_i
        gt = straight_trajectory(200)
        est = []
        for i, m in enumerate(gt):
            e = m.copy()
            theta = 0.02 * i
            rz = np.array([[np.cos(theta), -np.sin(theta), 0],
                           [np.sin(theta), np.cos(theta), 0],
                           [0, 0, 1.0]])
            e[:3, :3] = rz @ e[:3, :3]
            est.append(e)
        rpe = relative_pose_error(est, gt, length=50)
        self.assertGreater(rpe["r_mean_deg"], 20.0)


class TestMot(unittest.TestCase):
    def test_perfect_tracking(self):
        # one car tracked perfectly for 5 frames
        gt_frames = [[[0, 0, 10, 10]]] * 5
        gt_ids = [[1]] * 5
        pred_frames = [[[0.5, 0.5, 10.5, 10.5]]] * 5
        pred_ids = [[7]] * 5
        m = mot_metrics(gt_frames, gt_ids, pred_frames, pred_ids)
        self.assertAlmostEqual(1.0, m["mota"], places=6)
        self.assertEqual(0, m["idsw"])
        self.assertEqual(0, m["fp"] + m["fn"])

    def test_false_positive_and_negative(self):
        gt_frames = [[[0, 0, 10, 10]]]      # one GT box
        gt_ids = [[1]]
        pred_frames = [[[0, 0, 10, 10], [20, 20, 30, 30]]]   # one FP
        pred_ids = [[7, 8]]
        m = mot_metrics(gt_frames, gt_ids, pred_frames, pred_ids)
        self.assertEqual(1, m["fp"])
        # MOTA = 1 - (1 + 0 + 0)/1 = 0
        self.assertAlmostEqual(0.0, m["mota"], places=6)

    def test_identity_switch_counted(self):
        # GT: car A and car B; predictions swap IDs between frame 1 and 2
        gt_frames = [[[0, 0, 10, 10], [20, 0, 30, 10]],
                     [[0, 0, 10, 10], [20, 0, 30, 10]]]
        gt_ids = [[1, 2], [1, 2]]
        pred_frames = [[[0, 0, 10, 10], [20, 0, 30, 10]],
                       [[0, 0, 10, 10], [20, 0, 30, 10]]]
        pred_ids = [[10, 11], [11, 10]]     # swap!
        m = mot_metrics(gt_frames, gt_ids, pred_frames, pred_ids)
        # a swap counts as TWO identity switches (both tracks switched)
        self.assertEqual(2, m["idsw"])


class TestSegmentation(unittest.TestCase):
    def test_perfect(self):
        gt = np.zeros((10, 10), dtype=np.int32)
        gt[5:, :] = 1
        m = segmentation_iou(gt, gt, num_classes=2)
        self.assertAlmostEqual(1.0, m["miou"], places=6)

    def test_partial(self):
        gt = np.zeros((10, 10), dtype=np.int32)
        pred = np.zeros((10, 10), dtype=np.int32)
        gt[:5, :] = 1
        pred[:5, :5] = 1            # pred class-1 only in the top-left quadrant
        m = segmentation_iou(pred, gt, num_classes=2)
        # class 0: pred covers 75 cells, gt 50, inter 50 -> 50/75 = 0.667
        # class 1: pred 25 cells, gt 50, inter 25 -> 25/50 = 0.5
        self.assertAlmostEqual((0.5 + 50 / 75) / 2, m["miou"], places=3)


class TestGrid(unittest.TestCase):
    def test_perfect(self):
        gt = np.zeros((10, 10), dtype=np.int8)
        gt[5, 5] = 100
        m = grid_precision_recall(gt, gt)
        self.assertAlmostEqual(1.0, m["precision"], places=6)
        self.assertAlmostEqual(1.0, m["recall"], places=6)

    def test_ignores_unknown(self):
        gt = np.full((10, 10), -1, dtype=np.int8)
        gt[5, 5] = 100
        pred = np.full((10, 10), -1, dtype=np.int8)
        pred[5, 5] = 100
        pred[0, 0] = 100            # in unknown area -> ignored
        m = grid_precision_recall(pred, gt)
        self.assertAlmostEqual(1.0, m["precision"], places=6)
        self.assertAlmostEqual(1.0, m["recall"], places=6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
