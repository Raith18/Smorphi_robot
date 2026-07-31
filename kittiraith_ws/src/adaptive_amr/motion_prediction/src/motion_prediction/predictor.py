#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
predictor.py — constant-velocity Kalman trajectory prediction (pure NumPy).

For each tracked object we maintain a 6-state CV (constant-velocity) Kalman
filter over [x, y, z, vx, vy, vz]:

    F = [ I3  dt*I3 ]        H = [ I3  0 ]
        [ 0      I3 ]
    predict:  x' = F x ,  P' = F P F^T + Q
    update:   K = P' H^T (H P' H^T + R)^{-1} ,  x = x' + K (z - H x')

Future positions along the horizon are the CV extrapolation
    p(t) = p0 + v * t      (with covariance growth from P)

collision_risk: a heuristic based on how close the predicted trajectory
passes to the ego origin (the sensor), normalized by the horizon.
"""

from typing import Dict, List, Tuple

import numpy as np


class ConstantVelocityFilter:
    """6-state constant-velocity Kalman filter (pure NumPy)."""

    def __init__(self, position, dt: float = 0.1,
                 process_noise: float = 0.5, measurement_noise: float = 0.1):
        self.dt = float(dt)
        self.F = np.eye(6)
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        self.H = np.zeros((3, 6))
        self.H[:3, :3] = np.eye(3)
        self.Q = np.eye(6) * float(process_noise)
        self.Q[3:, 3:] *= 0.1
        self.R = np.eye(3) * float(measurement_noise)
        self.P = np.eye(6) * 10.0
        self.x = np.zeros(6)
        self.x[:3] = np.asarray(position, dtype=float)

    # ------------------------------------------------------------------ #
    def predict(self) -> None:
        # dt can change between frames (irregular sensor rate) — refresh F
        self.F[0, 3] = self.F[1, 4] = self.F[2, 5] = self.dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    # ------------------------------------------------------------------ #
    def update(self, position) -> None:
        z = np.asarray(position, dtype=float).reshape(3)
        y = z - self.H @ self.x
        s = self.H @ self.P @ self.H.T + self.R
        k = self.P @ self.H.T @ np.linalg.inv(s)
        self.x = self.x + k @ y
        self.P = (np.eye(6) - k @ self.H) @ self.P

    # ------------------------------------------------------------------ #
    def trajectory(self, horizon_s: float, steps: int) -> Tuple[np.ndarray, float]:
        """
        Predicted positions at `steps` points up to `horizon_s` seconds.

        Returns (trajectory (steps, 3), distance of closest approach to the
        origin along the trajectory [m]).
        """
        times = np.linspace(0.0, horizon_s, steps)
        p0 = self.x[:3]
        v = self.x[3:]
        traj = p0[None, :] + np.outer(times, v)
        closest = float(np.min(np.linalg.norm(traj, axis=1)))
        return traj, closest


class MotionPredictor:
    """
    Maintains one ConstantVelocityFilter per track_id.

    Args:
        horizon_s: prediction horizon [s].
        steps:     trajectory samples.
        process_noise / measurement_noise: Kalman tuning.
    """

    def __init__(self, horizon_s: float = 3.0, steps: int = 10,
                 process_noise: float = 0.5, measurement_noise: float = 0.1):
        self.horizon_s = float(horizon_s)
        self.steps = int(steps)
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.filters: Dict[int, ConstantVelocityFilter] = {}
        self.last_time: Dict[int, float] = {}
        self.velocity_out: Dict[int, np.ndarray] = {}
        self.positions: Dict[int, np.ndarray] = {}

    # ------------------------------------------------------------------ #
    def update(self, track_id: int, position, stamp_s: float
               ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Feed one measurement. Returns (position, velocity, closest_approach).
        """
        position = np.asarray(position, dtype=float)
        if track_id not in self.filters:
            self.filters[track_id] = ConstantVelocityFilter(
                position, process_noise=self.process_noise,
                measurement_noise=self.measurement_noise)
            self.last_time[track_id] = stamp_s
            self.positions[track_id] = position
            self.velocity_out[track_id] = np.zeros(3)
            return position, np.zeros(3), float("inf")

        dt = max(stamp_s - self.last_time.get(track_id, stamp_s), 1e-3)
        filt = self.filters[track_id]
        filt.dt = dt
        filt.predict()
        filt.update(position)

        self.last_time[track_id] = stamp_s
        self.positions[track_id] = filt.x[:3].copy()
        self.velocity_out[track_id] = filt.x[3:].copy()
        _, closest = filt.trajectory(self.horizon_s, self.steps)
        return self.positions[track_id], self.velocity_out[track_id], closest

    # ------------------------------------------------------------------ #
    def predict_trajectory(self, track_id: int) -> np.ndarray:
        """Predicted (steps, 3) trajectory for a track."""
        filt = self.filters.get(track_id)
        if filt is None:
            return np.empty((0, 3))
        traj, _ = filt.trajectory(self.horizon_s, self.steps)
        return traj

    # ------------------------------------------------------------------ #
    def prune(self, active_ids, keep_age: float = 5.0) -> None:
        """Drop filters not seen recently (bounded memory)."""
        for track_id in list(self.filters.keys()):
            if track_id not in active_ids and \
                    self.last_time.get(track_id, 0.0) < keep_age:
                self.filters.pop(track_id, None)
                self.last_time.pop(track_id, None)
                self.velocity_out.pop(track_id, None)
                self.positions.pop(track_id, None)

    # ------------------------------------------------------------------ #
    def collision_risk(self, closest: float, safe_distance: float = 1.5) -> float:
        """0..1 heuristic: 1 = predicted path passes through the ego zone."""
        if not np.isfinite(closest):
            return 0.0
        return float(np.clip(1.0 - closest / safe_distance, 0.0, 1.0))

    # ------------------------------------------------------------------ #
    def clear(self) -> None:
        self.filters.clear()
        self.last_time.clear()
        self.velocity_out.clear()
        self.positions.clear()
