#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motion_prediction_node.py — dynamic obstacle prediction node.

Subscribes to /object_tracks (SORT output) and publishes predicted
trajectories as /dynamic_obstacles (adaptive_amr_msgs/DynamicObstacleArray),
with RViz markers and statistics.

Run:
    rosrun motion_prediction motion_prediction_node.py
"""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from adaptive_amr_msgs.msg import (DynamicObstacle, DynamicObstacleArray,
                                   ObjectTrackArray)
from motion_prediction.predictor import MotionPredictor


class MotionPredictionNode:
    """Predicts trajectories for tracked objects."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.input_topic = rospy.get_param("~input_topic", "/object_tracks")
        self.output_topic = rospy.get_param("~output_topic", "/dynamic_obstacles")
        self.horizon_s = rospy.get_param("~horizon_s", 3.0)
        self.steps = rospy.get_param("~steps", 10)
        self.process_noise = rospy.get_param("~process_noise", 0.5)
        self.measurement_noise = rospy.get_param("~measurement_noise", 0.1)
        self.safe_distance = rospy.get_param("~safe_distance", 1.5)
        self.frame_id = rospy.get_param("~frame_id", "laser")
        self.publish_markers = rospy.get_param("~publish_markers", True)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.predictor = MotionPredictor(horizon_s=self.horizon_s,
                                         steps=self.steps,
                                         process_noise=self.process_noise,
                                         measurement_noise=self.measurement_noise)

        self.frames_processed = 0
        self.last_ms = 0.0
        self.active_obstacles = 0

        # ---- ROS ------------------------------------------------------------------
        self.pub_obstacles = rospy.Publisher(self.output_topic,
                                             DynamicObstacleArray, queue_size=5)
        self.pub_markers = rospy.Publisher(self.output_topic + "/markers",
                                           MarkerArray, queue_size=2)
        self.pub_stats = rospy.Publisher(self.output_topic + "/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Subscriber(self.input_topic, ObjectTrackArray, self._on_tracks)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("motion_prediction: %s -> %s (horizon=%.1fs, %d steps)",
                      self.input_topic, self.output_topic, self.horizon_s,
                      self.steps)

    # ------------------------------------------------------------------ #
    def _on_tracks(self, msg: ObjectTrackArray) -> None:
        import time
        t0 = time.time()
        stamp_s = msg.header.stamp.to_sec()

        obstacles = DynamicObstacleArray()
        obstacles.header = msg.header

        active_ids = set()
        for track in msg.tracks:
            active_ids.add(track.track_id)
            position = [track.position.x, track.position.y, track.position.z]
            _, velocity, closest = self.predictor.update(
                track.track_id, position, stamp_s)

            traj = self.predictor.predict_trajectory(track.track_id)

            obs = DynamicObstacle()
            obs.track_id = track.track_id
            obs.label = track.label
            obs.score = track.score
            obs.position = Point(x=float(position[0]), y=float(position[1]),
                                 z=float(position[2]))
            obs.velocity = Vector3(x=float(velocity[0]), y=float(velocity[1]),
                                   z=float(velocity[2]))
            obs.predicted_trajectory = [Point(x=float(p[0]), y=float(p[1]),
                                              z=float(p[2])) for p in traj]
            obs.horizon = self.horizon_s
            obs.collision_risk = self.predictor.collision_risk(
                closest, self.safe_distance)
            obstacles.obstacles.append(obs)

        self.predictor.prune(active_ids)

        self.pub_obstacles.publish(obstacles)
        if self.publish_markers:
            self.pub_markers.publish(self._build_markers(obstacles))

        self.frames_processed += 1
        self.active_obstacles = len(obstacles.obstacles)
        self.last_ms = (time.time() - t0) * 1000.0

    # ------------------------------------------------------------------ #
    def _build_markers(self, obstacles) -> MarkerArray:
        """Trajectory line-strips + velocity arrows for RViz."""
        markers = MarkerArray()
        for i, obs in enumerate(obstacles.obstacles):
            traj_marker = Marker()
            traj_marker.header.frame_id = self.frame_id
            traj_marker.header.stamp = obstacles.header.stamp
            traj_marker.ns = "trajectories"
            traj_marker.id = obs.track_id
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.scale.x = 0.08
            traj_marker.color = ColorRGBA(
                r=1.0 - obs.collision_risk, g=obs.collision_risk, b=0.0, a=0.9)
            traj_marker.points = list(obs.predicted_trajectory)
            markers.markers.append(traj_marker)
        return markers

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.frames_processed > 0
                        else DiagnosticStatus.WARN)
        status.name = "motion_prediction"
        status.message = "{} obstacles".format(self.active_obstacles)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="active_obstacles", value=str(self.active_obstacles)),
            KeyValue(key="horizon_s", value="{:.1f}".format(self.horizon_s)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("motion_prediction_node", anonymous=False)
    try:
        MotionPredictionNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("motion_prediction interrupted.")
    except Exception as exc:
        rospy.logfatal("motion_prediction failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
