#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
behavior_node.py — safety behavior & decision node.

Subscribes:
    /dynamic_obstacles   (adaptive_amr_msgs/DynamicObstacleArray)
    /localization_pose   (geometry_msgs/PoseWithCovarianceStamped)
    /planned_path        (nav_msgs/Path)
    /goal                (geometry_msgs/PoseStamped)
Publishes:
    /behavior/state            (std_msgs/String)
    /behavior/velocity_command (geometry_msgs/Twist)
    /behavior/statistics       (DiagnosticArray)

The state machine (NAVIGATE/AVOID/STOP/RESUME/GOAL_REACHED) is driven by the
max collision risk from /dynamic_obstacles and the distance to the goal; the
decision layer turns the state into a (v, w) command with an angular
P-controller on the heading error to the next waypoint.

Run:
    rosrun navigation_layer behavior_node.py
"""

import math

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String

from adaptive_amr_msgs.msg import DynamicObstacleArray
from navigation_layer.behavior import BehaviorState, BehaviorStateMachine


class BehaviorNode:
    """Decides the robot's state and velocity command."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.obstacles_topic = rospy.get_param("~obstacles_topic",
                                               "/dynamic_obstacles")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization_pose")
        self.path_topic = rospy.get_param("~path_topic", "/planned_path")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.risk_stop = rospy.get_param("~risk_stop", 0.8)
        self.risk_avoid = rospy.get_param("~risk_avoid", 0.4)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.0)
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.5)
        self.kp_angular = rospy.get_param("~kp_angular", 1.0)
        self.control_rate = rospy.get_param("~control_rate", 10.0)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.sm = BehaviorStateMachine(risk_stop=self.risk_stop,
                                       risk_avoid=self.risk_avoid,
                                       goal_tolerance=self.goal_tolerance)

        # ---- state ---------------------------------------------------------------
        self.robot_pose = None            # (x, y, yaw)
        self.goal = None                  # (x, y)
        self.path = None                  # nav_msgs/Path (latest)
        self.max_risk = 0.0
        self.state = BehaviorState.INIT

        # ---- ROS -------------------------------------------------------------------
        self.pub_state = rospy.Publisher("/behavior/state", String, queue_size=5)
        self.pub_cmd = rospy.Publisher("/behavior/velocity_command", Twist,
                                       queue_size=5)
        self.pub_stats = rospy.Publisher("/behavior/statistics",
                                         DiagnosticArray, queue_size=5)

        rospy.Subscriber(self.obstacles_topic, DynamicObstacleArray,
                         self._on_obstacles)
        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped,
                         self._on_pose)
        rospy.Subscriber(self.path_topic, Path, self._on_path)
        rospy.Subscriber(self.goal_topic, PoseStampedType(), self._on_goal)

        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("behavior: %s + %s + %s (risk_stop=%.2f, risk_avoid=%.2f)",
                      self.obstacles_topic, self.pose_topic, self.path_topic,
                      self.risk_stop, self.risk_avoid)

    # ------------------------------------------------------------------ #
    def _on_obstacles(self, msg: DynamicObstacleArray) -> None:
        self.max_risk = max((o.collision_risk for o in msg.obstacles),
                            default=0.0)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw)

    def _on_path(self, msg: Path) -> None:
        self.path = msg

    def _on_goal(self, msg) -> None:
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        rospy.loginfo("behavior: goal set (%.2f, %.2f)", *self.goal)

    # ------------------------------------------------------------------ #
    def _control_loop(self, _event=None) -> None:
        has_goal = self.goal is not None
        distance = float("inf")
        if has_goal and self.robot_pose is not None:
            distance = math.hypot(self.goal[0] - self.robot_pose[0],
                                  self.goal[1] - self.robot_pose[1])
        path_blocked = self._path_blocked()

        self.state = self.sm.update(has_goal, distance, self.max_risk,
                                    path_blocked)

        # heading error to the next waypoint (if any)
        heading_error = 0.0
        if self.robot_pose is not None and self.path is not None and \
                len(self.path.poses) > 0:
            wp = self.path.poses[0].pose.position
            heading_error = math.atan2(wp.y - self.robot_pose[1],
                                       wp.x - self.robot_pose[0]) - \
                            self.robot_pose[2]
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi

        vx, wz = self.sm.desired_velocity(self.state, heading_error,
                                          self.cruise_speed, self.kp_angular)

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        self.pub_cmd.publish(cmd)

        state_msg = String(data=self.state)
        self.pub_state.publish(state_msg)

    # ------------------------------------------------------------------ #
    def _path_blocked(self) -> bool:
        """True when no feasible path exists (planner failed or empty)."""
        if self.path is None:
            return True
        return len(self.path.poses) == 0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.state != BehaviorState.STOP
                        else DiagnosticStatus.WARN)
        status.name = "behavior"
        status.message = self.state
        status.values = [
            KeyValue(key="state", value=self.state),
            KeyValue(key="max_risk", value="{:.2f}".format(self.max_risk)),
            KeyValue(key="goal", value=("({:.2f}, {:.2f})".format(*self.goal)
                                        if self.goal else "none")),
            KeyValue(key="path_blocked", value=str(self._path_blocked())),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def PoseStampedType():
    from geometry_msgs.msg import PoseStamped
    return PoseStamped


def main():
    rospy.init_node("behavior_node", anonymous=False)
    try:
        BehaviorNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("behavior interrupted.")
    except Exception as exc:
        rospy.logfatal("behavior failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
