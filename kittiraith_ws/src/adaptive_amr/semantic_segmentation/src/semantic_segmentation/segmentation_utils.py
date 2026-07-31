#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
segmentation_utils.py — segmentation label/color helpers (pure NumPy).

  * CLASS_COLORS — a fixed BGR palette per class name (visualization only).
  * build_label_image — combine instance masks into a single class-id image
    (0 = background, class_id+1 = class) with instance-overlap resolution
    (later/higher-score instances win).
  * colorize_labels — label image -> BGR colored image.
"""

from typing import Dict, List, Tuple

import numpy as np

# BGR colors (visualization) keyed by class name.
CLASS_COLORS: Dict[str, Tuple[int, int, int]] = {
    "person": (80, 80, 255),      # red-ish (BGR: blue=80)
    "bicycle": (80, 200, 255),
    "car": (255, 0, 0),           # blue
    "motorcycle": (0, 200, 255),
    "bus": (0, 128, 255),
    "truck": (0, 255, 255),       # yellow
    "traffic light": (255, 128, 0),
    "stop sign": (0, 0, 255),
    "background": (0, 0, 0),
}

DEFAULT_COLOR: Tuple[int, int, int] = (200, 200, 200)


def class_color(name: str) -> Tuple[int, int, int]:
    """Fixed BGR color for a class name (deterministic)."""
    if name in CLASS_COLORS:
        return CLASS_COLORS[name]
    return DEFAULT_COLOR


def build_label_image(masks: List[np.ndarray], class_ids: List[int],
                      height: int, width: int) -> np.ndarray:
    """
    Combine instance masks into a single label image (H, W) uint8.

    Args:
        masks:     list of boolean instance masks (H, W), in detection order.
        class_ids: integer class id per mask (0-based COCO index).
        height/width: image size.

    Returns:
        label image where 0 = background and pixel = class_id + 1.
        Later instances in the list overwrite earlier ones (tie-break by
        detection order, which YOLOv8 returns by confidence).
    """
    label = np.zeros((height, width), dtype=np.uint8)
    for mask, cls in zip(masks, class_ids):
        binary = np.asarray(mask, dtype=bool)
        label[binary] = cls + 1
    return label


def colorize_labels(label: np.ndarray, names: List[str]) -> np.ndarray:
    """
    label image (H, W) with values class_id+1 -> BGR image (H, W, 3).

    names: class names indexed by class_id (e.g. YOLOv8 model.names).
    """
    h, w = label.shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    unique = np.unique(label)
    for value in unique:
        if value == 0:
            continue
        cls_id = int(value) - 1
        if 0 <= cls_id < len(names):
            color = class_color(names[cls_id])
        else:
            color = DEFAULT_COLOR
        out[label == value] = color
    return out
