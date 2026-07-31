#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
perf_collector.py — live ROS performance collector.

Subscribes to every */statistics DiagnosticArray produced by the stack's
nodes, extracts the per-node latency keys, and logs a CSV timeline. Run it
during a rosbag replay to profile the whole stack on the robot/VM:

    roslaunch evaluation perf_collector.launch csv:=/tmp/perf.csv

Keys recognized (each node's diagnostics):
    last_process_ms, last_inference_ms, last_track_ms, last_fusion_ms,
    last_inference_ms (segmentation), last_process_ms (depth) ...
"""

import csv
import os
import rospy
from diagnostic_msgs.msg import DiagnosticArray


class PerfCollector:
    def __init__(self):
        self.csv_path = rospy.get_param("~csv", "/tmp/perf.csv")
        self.ns_prefix = rospy.get_param("~ns", "")
        self.sample_rate = rospy.get_param("~sample_rate", 5.0)
        self.rows = []

        if self.ns_prefix:
            # single namespace: subscribe to <ns>/statistics
            topic = self.ns_prefix + "/statistics"
            rospy.Subscriber(topic, DiagnosticArray, self._on_stats)
            rospy.loginfo("perf_collector: subscribing %s", topic)
        else:
            # subscribe to all statistics topics by wildcard
            from rosgraph import masterapi
            master = masterapi.Master(rospy.get_name())
            topics = master.getPublishedTopics("")
            for topic, _type in topics:
                if topic.endswith("/statistics"):
                    rospy.Subscriber(topic, DiagnosticArray, self._on_stats)
                    rospy.loginfo("perf_collector: subscribing %s", topic)

        rospy.Timer(rospy.Duration(1.0 / max(self.sample_rate, 0.1)),
                    self._flush)
        rospy.on_shutdown(self._flush)

    def _on_stats(self, msg: DiagnosticArray) -> None:
        stamp = msg.header.stamp.to_sec()
        for status in msg.status:
            node = status.name
            values = {kv.key: kv.value for kv in status.values}
            row = {"t": "{:.6f}".format(stamp), "node": node}
            row.update(values)
            self.rows.append(row)

    def _flush(self, *_) -> None:
        if not self.rows:
            return
        keys = ["t", "node"]
        for row in self.rows:
            for k in row:
                if k not in keys:
                    keys.append(k)
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        with open(self.csv_path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            writer.writerows(self.rows)
        rospy.loginfo("perf_collector: %d samples -> %s", len(self.rows),
                      self.csv_path)
        self.rows = []


def main():
    rospy.init_node("perf_collector", anonymous=True)
    PerfCollector()
    rospy.spin()


if __name__ == "__main__":
    main()
