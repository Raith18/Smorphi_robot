#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
metrics.py — pure-NumPy evaluation metrics (unit-tested).

Implements the standard robotics benchmarks:

  * ATE  (Absolute Trajectory Error, TUM style)      — odometry/localization
  * RPE  (Relative Pose Error, KITTI style)          — odometry/localization
  * MOTA / MOTP (CLEAR MOT)                          — multi-object tracking
  * IoU / mIoU                                       — segmentation
  * grid_precision_recall                            — occupancy grids

No ROS imports: testable on any host.
"""

from typing import Dict, Optional, Sequence

import numpy as np


# --------------------------------------------------------------------------- #
# Trajectory alignment (Umeyama)
# --------------------------------------------------------------------------- #
def umeyama_alignment(src: np.ndarray, dst: np.ndarray,
                      with_scale: bool = True):
    """
    Least-squares similarity alignment of two (N, 3) point sets
    (Umeyama 1991). Returns (R, t, scale) with  dst ~= scale·R·src + t.
    """
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    if src.shape != dst.shape or src.shape[1] != 3 or src.shape[0] < 3:
        raise ValueError("src/dst must be (N, 3) with N >= 3")
    n = src.shape[0]
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    sc = src - mu_s
    dc = dst - mu_d
    cov = (dc.T @ sc) / n
    u, d, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0.0:
        s[2, 2] = -1.0
    r = u @ s @ vt
    if with_scale:
        var_s = sc.var(axis=0).sum()
        scale = np.trace(np.diag(d) @ s) / var_s if var_s > 1e-12 else 1.0
    else:
        scale = 1.0
    t = mu_d - scale * r @ mu_s
    return r, t, scale


def _to_translations(poses) -> np.ndarray:
    poses = np.asarray(poses, dtype=float)
    if poses.ndim == 3 and poses.shape[1:] == (4, 4):
        return poses[:, :3, 3]
    return poses


def absolute_trajectory_error(est, gt, with_scale: bool = True
                              ) -> Dict[str, float]:
    """
    ATE (TUM): align the estimate to the ground truth (Umeyama), then
    RMSE/mean/median of the per-pose translation error [m].
    """
    p_est = _to_translations(est)
    p_gt = _to_translations(gt)
    if p_est.shape != p_gt.shape:
        raise ValueError("est and gt must have the same number of poses")
    r, t, scale = umeyama_alignment(p_est, p_gt, with_scale)
    aligned = scale * (r @ p_est.T).T + t
    errors = np.linalg.norm(aligned - p_gt, axis=1)
    return {
        "rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "mean_m": float(np.mean(errors)),
        "median_m": float(np.median(errors)),
        "max_m": float(np.max(errors)),
    }


def relative_pose_error(est, gt, length: int = 100) -> Dict[str, float]:
    """
    RPE (KITTI odometry benchmark style): for each start i, compare the
    relative motion over `length` frames:

        P_rel = inv(est[i]) @ est[i+length]      Q_rel = inv(gt[i]) @ gt[i+length]
        t_err = || P_rel[:3,3] - Q_rel[:3,3] ||
        r_err = angle( P_rel[:3,:3].T @ Q_rel[:3,:3] )   [deg]

    Returns mean/median translation error and mean rotation error.
    """
    est = np.asarray(est, dtype=float)
    gt = np.asarray(gt, dtype=float)
    if est.shape != gt.shape or est.ndim != 3 or est.shape[1:] != (4, 4):
        raise ValueError("est/gt must be lists of 4x4 matrices with equal length")
    n = est.shape[0] - length
    if n <= 0:
        raise ValueError("length {} exceeds trajectory ({} frames)".format(
            length, est.shape[0]))
    t_errs = np.empty(n)
    r_errs = np.empty(n)
    for i in range(n):
        p_rel = np.linalg.inv(est[i]) @ est[i + length]
        q_rel = np.linalg.inv(gt[i]) @ gt[i + length]
        t_errs[i] = np.linalg.norm(p_rel[:3, 3] - q_rel[:3, 3])
        d_r = p_rel[:3, :3].T @ q_rel[:3, :3]
        cos_angle = np.clip((np.trace(d_r) - 1.0) / 2.0, -1.0, 1.0)
        r_errs[i] = np.degrees(np.arccos(cos_angle))
    return {
        "length": int(length),
        "t_mean_m": float(t_errs.mean()),
        "t_median_m": float(np.median(t_errs)),
        "t_max_m": float(t_errs.max()),
        "r_mean_deg": float(r_errs.mean()),
        "r_max_deg": float(r_errs.max()),
    }


# --------------------------------------------------------------------------- #
# CLEAR MOT (MOTA / MOTP)
# --------------------------------------------------------------------------- #
def _iou_boxes(a: Sequence[float], b: Sequence[float]) -> float:
    inter_x1 = max(a[0], b[0])
    inter_y1 = max(a[1], b[1])
    inter_x2 = min(a[2], b[2])
    inter_y2 = min(a[3], b[3])
    inter = max(inter_x2 - inter_x1, 0.0) * max(inter_y2 - inter_y1, 0.0)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0


def mot_metrics(gt_frames, gt_ids, pred_frames, pred_ids,
                iou_threshold: float = 0.5) -> Dict[str, float]:
    """
    CLEAR MOT metrics (MOTA / MOTP / FP / FN / IDSW).

    Args:
        gt_frames:   per frame, list of boxes [x1,y1,x2,y2].
        gt_ids:      per frame, list of ground-truth track ids.
        pred_frames: per frame, predicted boxes.
        pred_ids:    per frame, predicted track ids.
        iou_threshold: match threshold.
    """
    fp_total = 0
    fn_total = 0
    idsw_total = 0
    iou_sum = 0.0
    matches_total = 0
    mota_den = 0
    pred_to_gt: Dict[int, int] = {}

    n_frames = len(gt_frames)
    for f in range(n_frames):
        gts = gt_frames[f] if f < len(gt_frames) else []
        gids = gt_ids[f] if f < len(gt_ids) else []
        prs = pred_frames[f] if f < len(pred_frames) else []
        pids = pred_ids[f] if f < len(pred_ids) else []
        mota_den += len(gts)

        pairs = []
        for p, pbox in enumerate(prs):
            for g, gbox in enumerate(gts):
                iou = _iou_boxes(pbox, gbox)
                if iou >= iou_threshold:
                    pairs.append((iou, p, g))
        pairs.sort(reverse=True, key=lambda x: x[0])

        matched_p = set()
        matched_g = set()
        for iou, p, g in pairs:
            if p in matched_p or g in matched_g:
                continue
            matched_p.add(p)
            matched_g.add(g)
            pid = pids[p]
            gid = gids[g]
            if pid in pred_to_gt and pred_to_gt[pid] != gid:
                idsw_total += 1
            pred_to_gt[pid] = gid
            iou_sum += iou
            matches_total += 1

        fp_total += len(prs) - len(matched_p)
        fn_total += len(gts) - len(matched_g)

    mota = 1.0 - (fp_total + fn_total + idsw_total) / mota_den \
        if mota_den > 0 else 0.0
    motp = iou_sum / matches_total if matches_total > 0 else 0.0
    return {
        "mota": float(mota),
        "motp": float(motp),
        "fp": int(fp_total),
        "fn": int(fn_total),
        "idsw": int(idsw_total),
        "num_gt": int(mota_den),
        "num_matches": int(matches_total),
    }


# --------------------------------------------------------------------------- #
# Segmentation IoU
# --------------------------------------------------------------------------- #
def segmentation_iou(pred: np.ndarray, gt: np.ndarray,
                     num_classes: Optional[int] = None) -> Dict[str, float]:
    """
    Per-class IoU + mean IoU for label images (values 0..num_classes-1).
    """
    pred = np.asarray(pred)
    gt = np.asarray(gt)
    if pred.shape != gt.shape:
        raise ValueError("pred and gt must have the same shape")
    if num_classes is None:
        num_classes = int(max(pred.max(), gt.max())) + 1
    ious = []
    for c in range(num_classes):
        p = pred == c
        g = gt == c
        inter = int(np.logical_and(p, g).sum())
        union = int(np.logical_or(p, g).sum())
        if union == 0:
            continue
        ious.append(inter / union)
    miou = float(np.mean(ious)) if ious else 0.0
    return {"miou": miou, "per_class": ious, "num_classes": int(num_classes)}


# --------------------------------------------------------------------------- #
# Occupancy grid accuracy
# --------------------------------------------------------------------------- #
def grid_precision_recall(pred: np.ndarray, gt: np.ndarray,
                          occupied_threshold: int = 50) -> Dict[str, float]:
    """
    Precision/recall of occupied cells vs a ground-truth grid.

    pred/gt: int grids (-1 unknown, 0..100). Unknown cells are ignored.
    """
    pred = np.asarray(pred)
    gt = np.asarray(gt)
    if pred.shape != gt.shape:
        raise ValueError("pred and gt must have the same shape")
    valid = gt != -1
    p_occ = (pred >= occupied_threshold) & valid
    g_occ = (gt >= occupied_threshold) & valid
    tp = int((p_occ & g_occ).sum())
    fp = int((p_occ & ~g_occ).sum())
    fn = int((~p_occ & g_occ).sum())
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    return {"precision": float(precision), "recall": float(recall),
            "tp": tp, "fp": fp, "fn": fn}
