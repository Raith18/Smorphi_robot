#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
detection_utils.py — detection helpers (pure NumPy, no ROS, unit-tested).

  * COCO class handling: filter COCO classes to the KITTI-relevant subset.
  * Bounding-box geometry: center, area, IoU, clipping.
  * LiDAR frustum fusion: points inside a bbox -> median depth, and
    back-projection of the bbox center to the laser frame.
"""

from typing import List, Optional, Sequence, Tuple

import numpy as np

# COCO 80-class names (index -> name), as used by YOLOv8/ultralytics.
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
    "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
    "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
    "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table",
    "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
]

# KITTI-relevant COCO classes (driving scenes) kept by default.
KITTI_RELEVANT = ["person", "bicycle", "car", "motorcycle", "bus", "truck",
                  "traffic light", "stop sign"]

# Human-readable label normalization (COCO -> KITTI-style labels).
LABEL_MAP = {"person": "pedestrian", "bicycle": "cyclist", "car": "car",
             "motorcycle": "motorcyclist", "bus": "bus", "truck": "truck",
             "traffic light": "traffic_light", "stop sign": "stop_sign"}


def coco_index(name: str) -> Optional[int]:
    """COCO class index for a name, or None."""
    try:
        return COCO_NAMES.index(name)
    except ValueError:
        return None


def normalize_label(name: str) -> str:
    """COCO name -> KITTI-style label (identity if unmapped)."""
    return LABEL_MAP.get(name, name)


# --------------------------------------------------------------------------- #
# Bounding box geometry (boxes are [x1, y1, x2, y2])
# --------------------------------------------------------------------------- #
def clip_box(box: Sequence[float], width: int, height: int) -> np.ndarray:
    """Clip a box to the image bounds and enforce non-degenerate size."""
    x1, y1, x2, y2 = (float(v) for v in box[:4])
    x1 = min(max(x1, 0.0), width - 1.0)
    y1 = min(max(y1, 0.0), height - 1.0)
    x2 = min(max(x2, 0.0), width - 1.0)
    y2 = min(max(y2, 0.0), height - 1.0)
    if x2 - x1 < 1.0:
        x2 = x1 + 1.0
    if y2 - y1 < 1.0:
        y2 = y1 + 1.0
    return np.array([x1, y1, x2, y2])


def box_area(box: Sequence[float]) -> float:
    return max(float(box[2] - box[0]), 0.0) * max(float(box[3] - box[1]), 0.0)


def box_iou(a: Sequence[float], b: Sequence[float]) -> float:
    """Intersection-over-union of two boxes [x1,y1,x2,y2]."""
    inter_x1 = max(float(a[0]), float(b[0]))
    inter_y1 = max(float(a[1]), float(b[1]))
    inter_x2 = min(float(a[2]), float(b[2]))
    inter_y2 = min(float(a[3]), float(b[3]))
    inter = max(inter_x2 - inter_x1, 0.0) * max(inter_y2 - inter_y1, 0.0)
    union = box_area(a) + box_area(b) - inter
    return inter / union if union > 0.0 else 0.0


def box_center(box: Sequence[float]) -> Tuple[float, float]:
    return (float(box[0]) + float(box[2])) / 2.0, (float(box[1]) + float(box[3])) / 2.0


# --------------------------------------------------------------------------- #
# LiDAR frustum fusion
# --------------------------------------------------------------------------- #
def points_in_box(u: np.ndarray, v: np.ndarray, box: Sequence[float]) -> np.ndarray:
    """Boolean mask of projected pixels that fall inside `box`."""
    x1, y1, x2, y2 = box
    return (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)


def median_depth_in_box(depth: np.ndarray, in_box: np.ndarray,
                        max_depth: float = 120.0) -> Optional[float]:
    """
    Median depth of valid points inside a bbox (robust to outliers).
    Returns None when no valid depth is available.
    """
    values = depth[in_box]
    values = values[(values > 0.0) & (values < max_depth)]
    if values.size == 0:
        return None
    return float(np.median(values))


def backproject_to_laser(u: float, v: float, depth: float, K: np.ndarray,
                         T_velo_cam: np.ndarray) -> np.ndarray:
    """
    Back-project a pixel (u, v) at `depth` meters to a 3D point in the
    laser frame:

        p_cam  = depth * K^{-1} * [u, v, 1]
        p_laser = T_velo_cam^{-1} * p_cam
    """
    ray = np.linalg.inv(K).dot(np.array([u, v, 1.0]))
    ray /= np.linalg.norm(ray)
    p_cam = ray * depth
    t_inv = np.linalg.inv(np.asarray(T_velo_cam, dtype=float))
    p_h = t_inv.dot(np.array([p_cam[0], p_cam[1], p_cam[2], 1.0]))
    return p_h[:3]
