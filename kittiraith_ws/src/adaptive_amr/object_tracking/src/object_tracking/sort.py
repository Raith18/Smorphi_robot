#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sort.py — Simple Online and Realtime Tracking (SORT), pure NumPy/SciPy.

Clean-room implementation of the classic SORT tracker (Bewley et al., 2016),
the standard lightweight baseline used in industrial perception stacks:

  * KalmanBoxFilter — a 7-state constant-velocity linear Kalman filter
    (implemented here in NumPy, no external KF library):
        state x = [x, y, s, r, vx, vy, vs]
        (x,y) = box center, s = area, r = aspect ratio (kept constant),
        (vx,vy,vs) = velocities.
        predict:  x' = F x ,  P' = F P F^T + Q
        update:   K = P H^T (H P H^T + R)^{-1}
                  x = x + K (z - H x) ,  P = (I - K H) P
  * SortTracker — association via the Hungarian algorithm on IoU cost,
    with track creation/deletion rules (min_hits, max_age, iou_threshold).

No ROS imports: unit-testable on any host.
"""

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
    HAVE_SCIPY = True
except ImportError:  # pragma: no cover
    HAVE_SCIPY = False


# --------------------------------------------------------------------------- #
# Box conversions and IoU
# --------------------------------------------------------------------------- #
def box_to_xyah(box) -> np.ndarray:
    """[x1,y1,x2,y2] -> [x_center, y_center, area, aspect_ratio]."""
    x1, y1, x2, y2 = box
    w = max(float(x2) - float(x1), 1e-6)
    h = max(float(y2) - float(y1), 1e-6)
    return np.array([(x1 + x2) / 2.0, (y1 + y2) / 2.0, w * h, w / h])


def xyah_to_box(state) -> np.ndarray:
    """[x, y, s, r] -> [x1, y1, x2, y2]."""
    x, y, s, r = state
    w = np.sqrt(max(s * r, 1e-6))
    h = s / w
    return np.array([x - w / 2.0, y - h / 2.0, x + w / 2.0, y + h / 2.0])


def iou_batch(bboxes_a: np.ndarray, bboxes_b: np.ndarray) -> np.ndarray:
    """
    Pairwise IoU between two sets of boxes [N,4] and [M,4] -> (N, M).
    """
    bboxes_a = np.asarray(bboxes_a, dtype=float)
    bboxes_b = np.asarray(bboxes_b, dtype=float)
    if bboxes_a.size == 0 or bboxes_b.size == 0:
        return np.zeros((bboxes_a.shape[0], bboxes_b.shape[0]))
    inter_x1 = np.maximum(bboxes_a[:, None, 0], bboxes_b[None, :, 0])
    inter_y1 = np.maximum(bboxes_a[:, None, 1], bboxes_b[None, :, 1])
    inter_x2 = np.minimum(bboxes_a[:, None, 2], bboxes_b[None, :, 2])
    inter_y2 = np.minimum(bboxes_a[:, None, 3], bboxes_b[None, :, 3])
    inter = np.maximum(inter_x2 - inter_x1, 0.0) * np.maximum(inter_y2 - inter_y1, 0.0)
    area_a = (bboxes_a[:, 2] - bboxes_a[:, 0]) * (bboxes_a[:, 3] - bboxes_a[:, 1])
    area_b = (bboxes_b[:, 2] - bboxes_b[:, 0]) * (bboxes_b[:, 3] - bboxes_b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(union > 0.0, inter / np.maximum(union, 1e-12), 0.0)


# --------------------------------------------------------------------------- #
# Kalman filter (self-contained)
# --------------------------------------------------------------------------- #
class KalmanBoxFilter:
    """
    Constant-velocity linear Kalman filter over [x, y, s, r, vx, vy, vs].

    State transition (dt = 1 frame):
        F = [ I3   I3 ]        (position/area integrate velocity)
            [ 0    I3 ]
    """

    def __init__(self, xyah):
        dim_x, dim_z = 7, 4
        self.F = np.array([
            [1, 0, 0, 0, 1, 0, 0],
            [0, 1, 0, 0, 0, 1, 0],
            [0, 0, 1, 0, 0, 0, 1],
            [0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1]], dtype=float)
        self.H = np.zeros((dim_z, dim_x), dtype=float)
        self.H[:4, :4] = np.eye(4)
        self.R = np.eye(dim_z) * 10.0            # measurement noise
        self.Q = np.eye(dim_x) * 0.01
        self.Q[4:, 4:] *= 0.01 / 0.01            # velocity process noise
        self.P = np.eye(dim_x) * 10.0
        self.P[4:, 4:] *= 1000.0                 # uncertain initial velocity
        self.x = np.zeros(dim_x)
        self.x[:4] = np.asarray(xyah, dtype=float)

    # ------------------------------------------------------------------ #
    def predict(self) -> None:
        """Time update: x = F x ; P = F P F^T + Q."""
        self.x = self.F.dot(self.x)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q

    # ------------------------------------------------------------------ #
    def update(self, xyah) -> None:
        """Measurement update with a detection [x, y, s, r]."""
        z = np.asarray(xyah, dtype=float)
        y = z - self.H.dot(self.x)
        s = self.H.dot(self.P).dot(self.H.T) + self.R
        k = self.P.dot(self.H.T).dot(np.linalg.inv(s))
        self.x = self.x + k.dot(y)
        self.P = (np.eye(self.P.shape[0]) - k.dot(self.H)).dot(self.P)


# --------------------------------------------------------------------------- #
# Track + SORT tracker
# --------------------------------------------------------------------------- #
class KalmanBoxTracker:
    """A single tracked object: one KalmanBoxFilter + lifecycle bookkeeping."""

    _next_id = 1  # class-level counter (shared across all trackers)

    def __init__(self, bbox, dt: float = 1.0):
        self.dt = float(dt)
        self.kf = KalmanBoxFilter(box_to_xyah(bbox))
        self.time_since_update = 0
        self.id = KalmanBoxTracker._next_id
        KalmanBoxTracker._next_id += 1
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def update(self, bbox) -> None:
        """Correct the filter with a matched detection."""
        self.time_since_update = 0
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(box_to_xyah(bbox))

    def predict(self) -> np.ndarray:
        """Advance one step; returns the predicted box [x1, y1, x2, y2]."""
        if self.kf.x[6] + self.kf.x[2] <= 0.0:
            self.kf.x[6] *= 0.0          # guard: area must stay positive
        self.kf.predict()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        return self.get_state()

    def get_state(self) -> np.ndarray:
        return xyah_to_box(self.kf.x[:4])


class SortTracker:
    """
    SORT tracker.

    Args:
        max_age:       frames to keep a track without detections before deleting.
        min_hits:      detections required before a track is "confirmed".
        iou_threshold: minimum IoU for a detection-track match.
    """

    def __init__(self, max_age: int = 4, min_hits: int = 1,
                 iou_threshold: float = 0.3, dt: float = 1.0):
        self.max_age = int(max_age)
        self.min_hits = int(min_hits)
        self.iou_threshold = float(iou_threshold)
        self.dt = float(dt)
        self.trackers = []
        self.frame_count = 0

    # ------------------------------------------------------------------ #
    def update(self, dets) -> np.ndarray:
        """
        Feed detections [N, 4] (x1,y1,x2,y2) -> tracked boxes [M, 5]
        (x1, y1, x2, y2, track_id).
        """
        if not HAVE_SCIPY:
            raise RuntimeError("scipy is required for SORT")
        dets = np.atleast_2d(np.asarray(dets, dtype=float))
        self.frame_count += 1

        # ---- predict all existing tracks ------------------------------------
        for tracker in self.trackers:
            tracker.predict()

        # ---- build track matrix, drop NaN states ------------------------------
        trks = np.zeros((len(self.trackers), 5))
        to_del = []
        for t, tracker in enumerate(self.trackers):
            state = tracker.get_state()
            trks[t, :4] = state
            if np.any(np.isnan(state)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            self.trackers.pop(t)

        # ---- associate (Hungarian on IoU) -------------------------------------
        matched, unmatched_dets, _ = self._associate(dets, trks)

        for t, d in matched:
            self.trackers[t].update(dets[d])

        for d in unmatched_dets:
            self.trackers.append(KalmanBoxTracker(dets[d], dt=self.dt))

        # ---- prune dead tracks ---------------------------------------------------
        self.trackers = [t for t in self.trackers
                         if t.time_since_update < self.max_age]

        # ---- output: only tracks updated this frame & confirmed -------------------
        outputs = []
        for tracker in self.trackers:
            if (tracker.time_since_update < 1 and
                    (tracker.hit_streak >= self.min_hits or
                     self.frame_count <= self.min_hits)):
                box = tracker.get_state()
                outputs.append(np.concatenate([box, [float(tracker.id)]]))
        if not outputs:
            return np.empty((0, 5))
        return np.asarray(outputs)

    # ------------------------------------------------------------------ #
    def _associate(self, dets, trks):
        """Hungarian assignment maximizing IoU; returns matched/unmatched."""
        if len(trks) == 0:
            return [], list(range(len(dets))), list(range(len(trks)))
        iou = iou_batch(dets, trks)
        row_idx, col_idx = linear_sum_assignment(1.0 - iou)
        matched = [(int(col), int(row)) for row, col in zip(row_idx, col_idx)
                   if iou[row, col] >= self.iou_threshold]
        matched_rows = {m[1] for m in matched}
        matched_cols = {m[0] for m in matched}
        unmatched_dets = [d for d in range(len(dets)) if d not in matched_rows]
        unmatched_trks = [t for t in range(len(trks)) if t not in matched_cols]
        return matched, unmatched_dets, unmatched_trks
