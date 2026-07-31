#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
behavior.py — safety behavior state machine (pure Python, unit-tested).

States:
    INIT -> NAVIGATE -> AVOID -> STOP -> RESUME -> NAVIGATE -> GOAL_REACHED
                     \-> STOP   \-> NAVIGATE

Transitions are driven by:
    * has_goal / distance_to_goal
    * max_risk  (collision risk 0..1 from the motion predictor)
    * path_blocked (no feasible path or next waypoint lethal)

The state machine only *decides*; the node turns the decision into a
velocity command (cruise speed, angular P-control on the heading error).
"""

from typing import Tuple


class BehaviorState:
    INIT = "INIT"
    NAVIGATE = "NAVIGATE"
    AVOID = "AVOID"
    STOP = "STOP"
    RESUME = "RESUME"
    GOAL_REACHED = "GOAL_REACHED"


class BehaviorStateMachine:
    """
    Safety-gated navigation state machine.

    Args:
        risk_stop:      risk >= this -> STOP.
        risk_avoid:     risk >= this -> AVOID.
        goal_tolerance: distance [m] within which the goal is reached.
    """

    def __init__(self, risk_stop: float = 0.8, risk_avoid: float = 0.4,
                 goal_tolerance: float = 1.0):
        self.risk_stop = float(risk_stop)
        self.risk_avoid = float(risk_avoid)
        self.goal_tolerance = float(goal_tolerance)
        self.state = BehaviorState.INIT

    # ------------------------------------------------------------------ #
    def update(self, has_goal: bool, distance_to_goal: float,
               max_risk: float, path_blocked: bool) -> str:
        """
        Advance the state machine and return the new state.
        """
        distance_to_goal = float(distance_to_goal)
        max_risk = float(max_risk)

        # INIT: wait for a goal
        if self.state == BehaviorState.INIT:
            if has_goal:
                self.state = BehaviorState.NAVIGATE
            return self.state

        # GOAL_REACHED: a new goal restarts navigation
        if self.state == BehaviorState.GOAL_REACHED:
            if has_goal and distance_to_goal > self.goal_tolerance:
                self.state = BehaviorState.NAVIGATE
            return self.state

        if not has_goal:
            self.state = BehaviorState.STOP
            return self.state

        # NAVIGATE
        if self.state == BehaviorState.NAVIGATE:
            if distance_to_goal <= self.goal_tolerance:
                self.state = BehaviorState.GOAL_REACHED
            elif max_risk >= self.risk_stop:
                self.state = BehaviorState.STOP
            elif max_risk >= self.risk_avoid or path_blocked:
                self.state = BehaviorState.AVOID
            return self.state

        # AVOID
        if self.state == BehaviorState.AVOID:
            if max_risk >= self.risk_stop:
                self.state = BehaviorState.STOP
            elif max_risk < self.risk_avoid and not path_blocked:
                self.state = BehaviorState.NAVIGATE
            return self.state

        # STOP
        if self.state == BehaviorState.STOP:
            if max_risk < self.risk_stop:
                self.state = BehaviorState.RESUME
            return self.state

        # RESUME
        if self.state == BehaviorState.RESUME:
            if max_risk >= self.risk_stop:
                self.state = BehaviorState.STOP
            elif max_risk < self.risk_avoid and not path_blocked:
                self.state = BehaviorState.NAVIGATE
            return self.state

        return self.state

    # ------------------------------------------------------------------ #
    def desired_velocity(self, state: str, heading_error: float,
                         cruise_speed: float = 0.5,
                         kp_angular: float = 1.0) -> Tuple[float, float]:
        """
        Map a behavior state to a (linear_vel, angular_vel) command.

        heading_error: signed angle [rad] to the next waypoint.
        """
        if state == BehaviorState.NAVIGATE:
            return cruise_speed, float(kp_angular) * _clamp_angle(heading_error)
        if state == BehaviorState.AVOID:
            return cruise_speed * 0.3, 0.6
        if state == BehaviorState.RESUME:
            return cruise_speed * 0.5, float(kp_angular) * _clamp_angle(heading_error)
        # STOP / INIT / GOAL_REACHED
        return 0.0, 0.0


def _clamp_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    import math
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
