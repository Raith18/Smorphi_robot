#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_behavior.py — unit tests for navigation_layer.behavior state machine.

Run without ROS:
    python3 test_behavior.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from navigation_layer.behavior import (BehaviorState,  # noqa: E402
                                       BehaviorStateMachine)


class TestStateMachine(unittest.TestCase):
    def setUp(self):
        self.sm = BehaviorStateMachine(risk_stop=0.8, risk_avoid=0.4,
                                       goal_tolerance=1.0)

    def test_init_to_navigate(self):
        self.assertEqual(BehaviorState.INIT, self.sm.update(False, 10.0, 0.0,
                                                            False))
        self.assertEqual(BehaviorState.NAVIGATE, self.sm.update(True, 10.0,
                                                                0.0, False))

    def test_navigate_to_stop_on_high_risk(self):
        self.sm.update(True, 10.0, 0.0, False)
        state = self.sm.update(True, 10.0, 0.9, False)
        self.assertEqual(BehaviorState.STOP, state)

    def test_navigate_to_avoid_on_medium_risk(self):
        self.sm.update(True, 10.0, 0.0, False)
        state = self.sm.update(True, 10.0, 0.5, False)
        self.assertEqual(BehaviorState.AVOID, state)

    def test_avoid_back_to_navigate_when_clear(self):
        self.sm.update(True, 10.0, 0.0, False)
        self.sm.update(True, 10.0, 0.5, False)          # AVOID
        state = self.sm.update(True, 10.0, 0.1, False)
        self.assertEqual(BehaviorState.NAVIGATE, state)

    def test_stop_to_resume_to_navigate(self):
        self.sm.update(True, 10.0, 0.0, False)
        self.sm.update(True, 10.0, 0.9, False)          # STOP
        state = self.sm.update(True, 10.0, 0.2, False)  # clear
        self.assertEqual(BehaviorState.RESUME, state)
        state = self.sm.update(True, 10.0, 0.1, False)
        self.assertEqual(BehaviorState.NAVIGATE, state)

    def test_navigate_to_goal_reached(self):
        self.sm.update(True, 10.0, 0.0, False)
        state = self.sm.update(True, 0.5, 0.0, False)
        self.assertEqual(BehaviorState.GOAL_REACHED, state)

    def test_no_goal_stops(self):
        self.sm.update(True, 10.0, 0.0, False)
        state = self.sm.update(False, 10.0, 0.0, False)
        self.assertEqual(BehaviorState.STOP, state)

    def test_path_blocked_triggers_avoid(self):
        self.sm.update(True, 10.0, 0.0, False)
        state = self.sm.update(True, 10.0, 0.1, True)   # path blocked
        self.assertEqual(BehaviorState.AVOID, state)


class TestVelocity(unittest.TestCase):
    def setUp(self):
        self.sm = BehaviorStateMachine()

    def test_stop_zero(self):
        vx, wz = self.sm.desired_velocity(BehaviorState.STOP, 0.0)
        self.assertEqual((0.0, 0.0), (vx, wz))

    def test_navigate_cruise(self):
        vx, wz = self.sm.desired_velocity(BehaviorState.NAVIGATE, 0.0)
        self.assertGreater(vx, 0.0)
        self.assertAlmostEqual(0.0, wz, places=6)

    def test_navigate_turns_with_heading_error(self):
        vx, wz = self.sm.desired_velocity(BehaviorState.NAVIGATE, 0.5,
                                          kp_angular=2.0)
        self.assertAlmostEqual(1.0, wz, places=6)

    def test_avoid_slows_down(self):
        vx_cruise, _ = self.sm.desired_velocity(BehaviorState.NAVIGATE, 0.0,
                                                cruise_speed=0.5)
        vx_avoid, _ = self.sm.desired_velocity(BehaviorState.AVOID, 0.0,
                                               cruise_speed=0.5)
        self.assertLess(vx_avoid, vx_cruise)


if __name__ == "__main__":
    unittest.main(verbosity=2)
