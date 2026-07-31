#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pacing.py — dataset replay pacing (pure Python, no ROS imports).

The driver nodes publish each sensor frame at the cadence dictated by the
KITTI timestamps file. Real robots never "pace" — data arrives from hardware —
but a dataset player must, so downstream nodes observe a realistic stream.
"""

import time
from typing import Sequence


class RatePacer:
    """
    Replays a sequence of timestamps at a configurable speed.

    Args:
        stamps_ns:   per-frame timestamps (integer nanoseconds), ascending.
        rate_factor: 1.0 = real time; >1 = faster; 0 = as fast as possible.
        sleep_fn:    injectable sleep (default time.sleep) for unit tests.
    """

    MAX_SLEEP_SECONDS = 2.0  # cap: a corrupt timestamp must never stall playback

    def __init__(self, stamps_ns: Sequence[int], rate_factor: float = 1.0,
                 sleep_fn=time.sleep):
        self.stamps_ns = list(stamps_ns)
        self.rate_factor = float(rate_factor)
        self.sleep_fn = sleep_fn

    def sleep_until_frame(self, index: int) -> None:
        """
        Sleep so frame `index` is published at the right cadence.
        No-op for index 0, for rate_factor <= 0 (fast mode) or for
        non-increasing stamps (defensive).
        """
        if self.rate_factor <= 0.0 or index <= 0 or index >= len(self.stamps_ns):
            return
        dt_ns = self.stamps_ns[index] - self.stamps_ns[index - 1]
        if dt_ns <= 0:
            return
        delay = (dt_ns / 1e9) / self.rate_factor
        self.sleep_fn(min(delay, self.MAX_SLEEP_SECONDS))
