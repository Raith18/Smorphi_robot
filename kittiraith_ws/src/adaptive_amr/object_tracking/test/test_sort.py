#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_sort.py — unit tests for object_tracking.sort (pure NumPy/SciPy).

Run without ROS:
    python3 test_sort.py
"""

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from object_tracking.sort import (KalmanBoxFilter, KalmanBoxTracker,  # noqa: E402
                                  SortTracker, box_to_xyah, iou_batch,
                                  xyah_to_box)


class TestBoxConversions(unittest.TestCase):
    def test_roundtrip(self):
        box = np.array([10.0, 20.0, 30.0, 40.0])
        xyah = box_to_xyah(box)
        back = xyah_to_box(xyah)
        np.testing.assert_allclose(box, back, atol=1e-6)

    def test_xyah_values(self):
        xyah = box_to_xyah([0, 0, 10, 5])
        np.testing.assert_allclose([5.0, 2.5, 50.0, 2.0], xyah, atol=1e-6)


class TestIou(unittest.TestCase):
    def test_identical(self):
        boxes = np.array([[0, 0, 10, 10]])
        self.assertAlmostEqual(1.0, iou_batch(boxes, boxes)[0, 0])

    def test_disjoint(self):
        a = np.array([[0, 0, 10, 10]])
        b = np.array([[20, 20, 30, 30]])
        self.assertEqual(0.0, iou_batch(a, b)[0, 0])

    def test_empty(self):
        self.assertEqual((0, 3), iou_batch(np.empty((0, 4)), np.zeros((3, 4))).shape)


class TestKalmanBoxFilter(unittest.TestCase):
    def test_predict_constant_velocity(self):
        kf = KalmanBoxFilter(box_to_xyah([0, 0, 10, 10]))
        kf.x[4] = 2.0   # vx = 2 px/frame
        kf.predict()
        # center x = 5.0, moves +2 -> 7.0
        self.assertAlmostEqual(7.0, kf.x[0], places=6)
        self.assertAlmostEqual(5.0, kf.x[1], places=6)

    def test_update_approaches_measurement(self):
        kf = KalmanBoxFilter(box_to_xyah([0, 0, 10, 10]))
        for _ in range(5):
            kf.update([100, 100, 110, 110])
        self.assertGreater(kf.x[0], 50.0)   # pulled toward the measurement
        self.assertGreater(kf.x[1], 50.0)

    def test_predict_grows_covariance(self):
        kf = KalmanBoxFilter(box_to_xyah([0, 0, 10, 10]))
        p_before = np.trace(kf.P)
        kf.predict()
        self.assertGreater(np.trace(kf.P), p_before)


class TestSortTracker(unittest.TestCase):
    def test_single_object_keeps_id(self):
        tracker = SortTracker(max_age=4, min_hits=1, iou_threshold=0.3)
        box = np.array([[100.0, 100.0, 120.0, 140.0]])
        first = tracker.update(box)
        self.assertEqual(1, len(first))
        track_id = first[0, 4]
        for i in range(10):
            shifted = box + i * 1.0   # slow drift
            out = tracker.update(shifted)
            self.assertEqual(1, len(out))
            self.assertEqual(track_id, out[0, 4])   # same ID throughout

    def test_track_removed_after_max_age(self):
        tracker = SortTracker(max_age=2, min_hits=1, iou_threshold=0.3)
        tracker.update([[100, 100, 120, 140]])
        # no detections for max_age+1 frames -> track deleted
        for _ in range(5):
            out = tracker.update(np.empty((0, 4)))
        self.assertEqual(0, len(out))

    def test_new_track_gets_new_id(self):
        # min_hits=0 -> new tracks are output immediately
        tracker = SortTracker(max_age=4, min_hits=0, iou_threshold=0.3)
        first = tracker.update([[100, 100, 120, 140]])
        second = tracker.update([[400, 400, 420, 440]])   # far away -> new track
        ids = set()
        for out in (first, second):
            for row in out:
                ids.add(int(row[4]))
        self.assertEqual(2, len(ids))

    def test_two_objects_swap_cleanly(self):
        tracker = SortTracker(max_age=4, min_hits=1, iou_threshold=0.3)
        a = np.array([100.0, 100.0, 120.0, 140.0])
        b = np.array([300.0, 100.0, 320.0, 140.0])
        out1 = tracker.update(np.vstack([a, b]))
        out2 = tracker.update(np.vstack([a + 1, b + 1]))
        self.assertEqual(2, len(out1))
        self.assertEqual(2, len(out2))
        # both ids must persist
        ids1 = {int(r[4]) for r in out1}
        ids2 = {int(r[4]) for r in out2}
        self.assertEqual(ids1, ids2)

    def test_far_detection_creates_new_track(self):
        tracker = SortTracker(max_age=4, min_hits=0, iou_threshold=0.3)
        tracker.update([[100, 100, 120, 140]])
        # same-frame second object far away -> must be a NEW track (different id)
        out = tracker.update([[400, 400, 420, 440]])
        self.assertEqual(1, len(out))
        self.assertGreaterEqual(int(out[0, 4]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
