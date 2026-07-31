#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
object_tracking_node.py — SORT multi-object tracking node.

Subscribes to /object_detections (adaptive_amr_msgs), runs the SORT tracker,
and publishes stable tracks with IDs on /object_tracks, plus RViz markers
and statistics.

Publishers:
    /object_tracks            (adaptive_amr_msgs/ObjectTrackArray)
    /object_tracks/markers    (visualization_msgs/MarkerArray, 2D boxes)
    /object_tracks/statistics (DiagnosticArray)

Run:
    rosrun object_tracking object_tracking_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from adaptive_amr_msgs.msg import (ObjectDetectionArray, ObjectTrack,
                                   ObjectTrackArray)
from object_tracking.sort import SortTracker


class ObjectTrackingNode:
    """Tracks detected objects with SORT."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/object_detections")
        self.output_topic = rospy.get_param("~output_topic", "/object_tracks")
        self.max_age = rospy.get_param("~max_age", 4)
        self.min_hits = rospy.get_param("~min_hits", 1)
        self.iou_threshold = rospy.get_param("~iou_threshold", 0.3)
        self.publish_markers = rospy.get_param("~publish_markers", True)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.tracker = SortTracker(max_age=self.max_age,
                                   min_hits=self.min_hits,
                                   iou_threshold=self.iou_threshold)
        self.frames_processed = 0
        self.active_tracks = 0
        self.last_ms = 0.0

        self.pub_tracks = rospy.Publisher(self.output_topic, ObjectTrackArray,
                                          queue_size=5)
        self.pub_markers = rospy.Publisher(self.output_topic + "/markers",
                                           MarkerArray, queue_size=2)
        self.pub_stats = rospy.Publisher(self.output_topic + "/statistics",
                                         DiagnosticArray, queue_size=5)

        rospy.Subscriber(self.input_topic, ObjectDetectionArray, self._on_detections)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("object_tracking: %s -> %s (max_age=%d, iou=%.2f)",
                      self.input_topic, self.output_topic, self.max_age,
                      self.iou_threshold)

    def _on_detections(self, msg) -> None:
        start = rospy.Time.now()
        dets = np.array([[d.bbox[0], d.bbox[1], d.bbox[2], d.bbox[3]]
                         for d in msg.detections], dtype=float) \
            if msg.detections else np.empty((0, 4))
        track_boxes = self.tracker.update(dets)

        tracks = ObjectTrackArray()
        tracks.header = msg.header

        # map track id -> most recent detection info for label/score/position
        info = {i: d for i, d in enumerate(msg.detections)}
        for row in track_boxes:
            x1, y1, x2, y2, track_id = [float(v) for v in row]
            track = ObjectTrack()
            track.track_id = int(track_id)
            track.bbox = [x1, y1, x2, y2]
            # find the detection this track was matched to (this frame)
            match = None
            for i, det in enumerate(msg.detections):
                if abs(det.bbox[0] - x1) < 2.0 and abs(det.bbox[1] - y1) < 2.0:
                    match = det
                    break
            if match is not None:
                track.label = match.label
                track.score = match.score
                track.position = Point(x=match.position[0], y=match.position[1],
                                       z=match.position[2])
            tracks.tracks.append(track)

        # attach per-track lifecycle info from the tracker
        for track in tracks.tracks:
            trk = self._find_tracker(track.track_id)
            if trk is not None:
                track.age = trk.age
                track.hit_streak = trk.hit_streak
                track.velocity = Vector3(
                    x=float(trk.kf.x[4]), y=float(trk.kf.x[5]), z=0.0)

        self.pub_tracks.publish(tracks)
        if self.publish_markers:
            self.pub_markers.publish(self._build_markers(tracks, msg.header))

        self.frames_processed += 1
        self.active_tracks = len(tracks.tracks)
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    def _find_tracker(self, track_id):
        for trk in self.tracker.trackers:
            if trk.id == track_id:
                return trk
        return None

    def _build_markers(self, tracks, header) -> MarkerArray:
        markers = MarkerArray()
        for track in tracks.tracks:
            marker = Marker()
            marker.header = header
            marker.ns = "tracks"
            marker.id = track.track_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            x1, y1, x2, y2 = track.bbox
            marker.points = [Point(x=x1, y=y1, z=0.0),
                             Point(x=x2, y=y1, z=0.0),
                             Point(x=x2, y=y2, z=0.0),
                             Point(x=x1, y=y2, z=0.0),
                             Point(x=x1, y=y1, z=0.0)]
            marker.scale.x = 3.0
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
            marker.lifetime = rospy.Duration(0.3)
            markers.markers.append(marker)
        return markers

    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.frames_processed > 0 else DiagnosticStatus.WARN
        status.name = "object_tracking"
        status.message = "{} tracks active".format(self.active_tracks)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="active_tracks", value=str(self.active_tracks)),
            KeyValue(key="last_track_ms", value="{:.2f}".format(self.last_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("object_tracking_node", anonymous=False)
    try:
        ObjectTrackingNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_tracking interrupted.")
    except Exception as exc:
        rospy.logfatal("object_tracking failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
