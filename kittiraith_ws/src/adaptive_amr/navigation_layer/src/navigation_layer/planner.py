#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
planner.py — A* grid path planner (pure Python, unit-tested).

Standard A* on an 8-connected grid:

    f = g + h ,   h = Euclidean distance to the goal (admissible)
    g = step length * cell_cost_factor (0 free .. 1 max, unknown = penalty)

Lethal cells (cost >= 100) are blocked; unknown cells (-1) get a small
penalty so the planner prefers observed free space without refusing to
explore. The open list is a min-heap -> O(E log V).
"""

import heapq
from typing import Optional, Sequence, Tuple

import numpy as np

STEP_COST = [1.0, np.sqrt(2.0)]      # cardinal / diagonal
NEIGHBORS = [(-1, 0), (1, 0), (0, -1), (0, 1),
             (-1, -1), (-1, 1), (1, -1), (1, 1)]


def cell_cost_factor(value: int, unknown_penalty: float = 0.5) -> float:
    """Cost factor for a cell value: 0 free .. 1.0 max, penalty for unknown."""
    if value < 0:
        return float(unknown_penalty)
    if value >= 100:
        return float("inf")
    return value / 100.0


class AStarPlanner:
    """
    A* planner over a costmap.

    Args:
        costmap:     (H, W) int8 (-1 unknown, 0..100).
        resolution:  meters per cell.
        origin_x, origin_y: world coordinates of the grid's (0,0) corner.
        unknown_penalty: cost factor for unknown cells.
    """

    def __init__(self, costmap: np.ndarray, resolution: float,
                 origin_x: float = 0.0, origin_y: float = 0.0,
                 unknown_penalty: float = 0.5):
        self.costmap = np.asarray(costmap)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.unknown_penalty = float(unknown_penalty)
        self.h, self.w = self.costmap.shape

    # ------------------------------------------------------------------ #
    def _world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        col = int(np.floor((x - self.origin_x) / self.resolution))
        row = int(np.floor((y - self.origin_y) / self.resolution))
        return row, col

    def _cell_to_world(self, row: int, col: int) -> Tuple[float, float]:
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    # ------------------------------------------------------------------ #
    def plan(self, start: Sequence[float],
             goal: Sequence[float]) -> Optional[np.ndarray]:
        """
        Plan a path from start to goal (world coordinates).

        Returns an (N, 2) world path (smoothed) or None if unreachable.
        """
        sr, sc = self._world_to_cell(start[0], start[1])
        gr, gc = self._world_to_cell(goal[0], goal[1])
        if not (0 <= sr < self.h and 0 <= sc < self.w):
            return None
        if not (0 <= gr < self.h and 0 <= gc < self.w):
            return None
        if cell_cost_factor(self.costmap[gr, gc]) == float("inf"):
            return None

        start_world = np.asarray(start, dtype=float)[:2]
        goal_world = np.asarray(goal, dtype=float)[:2]

        open_heap = [(0.0, 0.0, (sr, sc))]
        g_score = {(sr, sc): 0.0}
        came_from = {}

        while open_heap:
            _, g, current = heapq.heappop(open_heap)
            if g > g_score.get(current, float("inf")):
                continue
            if current == (gr, gc):
                return self._reconstruct_path(came_from, current,
                                              start_world, goal_world)

            r, c = current
            for i, (dr, dc) in enumerate(NEIGHBORS):
                nr, nc = r + dr, c + dc
                if not (0 <= nr < self.h and 0 <= nc < self.w):
                    continue
                factor = cell_cost_factor(self.costmap[nr, nc],
                                          self.unknown_penalty)
                if factor == float("inf"):
                    continue
                step = STEP_COST[1 if dr != 0 and dc != 0 else 0]
                tentative = g + step * (0.1 + factor)   # small base cost
                if tentative < g_score.get((nr, nc), float("inf")):
                    g_score[(nr, nc)] = tentative
                    came_from[(nr, nc)] = current
                    h = float(np.hypot((nr - gr) * self.resolution,
                                       (nc - gc) * self.resolution))
                    heapq.heappush(open_heap, (tentative + h, tentative,
                                               (nr, nc)))
        return None

    # ------------------------------------------------------------------ #
    def _reconstruct_path(self, came_from, current, start_world, goal_world):
        """Backtrack the A* tree into a world-coordinate path."""
        cells = [current]
        while current in came_from:
            current = came_from[current]
            cells.append(current)
        cells.reverse()

        path = np.array([[self._cell_to_world(r, c)[0],
                          self._cell_to_world(r, c)[1]] for r, c in cells],
                        dtype=float)
        # replace the first/last cell with the exact start/goal
        if path.shape[0] > 0:
            path[0] = start_world
            path[-1] = goal_world
        return self._simplify(path)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _simplify(path: np.ndarray, min_step: float = 0.2) -> np.ndarray:
        """Drop collinear intermediate points (keeps the path compact)."""
        if path.shape[0] < 3:
            return path
        kept = [path[0]]
        for i in range(1, path.shape[0] - 1):
            a = path[i - 1]
            b = path[i]
            c = path[i + 1]
            cross = abs((b[0] - a[0]) * (c[1] - a[1])
                        - (b[1] - a[1]) * (c[0] - a[0]))
            if cross > 1e-6:
                kept.append(b)
        kept.append(path[-1])
        return np.asarray(kept, dtype=float)
