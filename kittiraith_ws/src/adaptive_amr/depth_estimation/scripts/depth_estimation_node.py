#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
depth_estimation_node.py — sparse-to-dense depth completion node.

Subscribes to the sparse LiDAR depth image from sensor_fusion and publishes
a dense depth map (+ colored visualization):

    /fusion/sparse_depth -> nearest_fill -> optional smooth
    /depth/dense          (sensor_msgs/Image, 32FC1)
    /depth/colored        (sensor_msgs/Image, bgr8)
    /depth/statistics     (DiagnosticArray)

Run:
    rosrun depth_estimation depth_estimation_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image

from depth_estimation.depth_completion import (colorize_depth, nearest_fill,
                                               smooth_depth)


class DepthEstimationNode:
    """Completes the sparse LiDAR depth into a dense depth map."""

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/fusion/sparse_depth")
        self.output_ns = rospy.get_param("~output_ns", "/depth")
        self.max_fill_distance = rospy.get_param("~max_fill_distance", 0)  # 0 = unlimited
        self.smooth_sigma = rospy.get_param("~smooth_sigma", 2.0)
        self.visualize_max_depth = rospy.get_param("~visualize_max_depth", 80.0)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.frames_processed = 0
        self.last_ms = 0.0
        self.last_coverage = 0.0

        self.pub_dense = rospy.Publisher(self.output_ns + "/dense", Image, queue_size=2)
        self.pub_colored = rospy.Publisher(self.output_ns + "/colored", Image, queue_size=2)
        self.pub_stats = rospy.Publisher(self.output_ns + "/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Subscriber(self.input_topic, Image, self._on_sparse)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("depth_estimation: %s -> %s/dense (fill=%s, sigma=%.1f)",
                      self.input_topic, self.output_ns,
                      self.max_fill_distance or "unlimited", self.smooth_sigma)

    # ------------------------------------------------------------------ #
    def _on_sparse(self, msg: Image) -> None:
        start = rospy.Time.now()
        if msg.encoding != "32FC1":
            rospy.logwarn_throttle(5.0, "depth_estimation: expected 32FC1, got %s",
                                   msg.encoding)
        sparse = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        try:
            dense = nearest_fill(sparse, self.max_fill_distance or None)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "depth_estimation: %s", exc)
            return
        dense = smooth_depth(dense, self.smooth_sigma)

        # ---- publish dense ----------------------------------------------------
        dense_msg = Image()
        dense_msg.header = msg.header
        dense_msg.height, dense_msg.width = dense.shape
        dense_msg.encoding = "32FC1"
        dense_msg.step = dense.shape[1] * 4
        dense_msg.data = dense.astype("<f4").tobytes()
        self.pub_dense.publish(dense_msg)

        # ---- publish colored -----------------------------------------------------
        colored = colorize_depth(dense, self.visualize_max_depth)
        colored_msg = Image()
        colored_msg.header = msg.header
        colored_msg.height, colored_msg.width = colored.shape[:2]
        colored_msg.encoding = "bgr8"
        colored_msg.step = colored.shape[1] * 3
        colored_msg.data = colored.tobytes()
        self.pub_colored.publish(colored_msg)

        self.frames_processed += 1
        total = dense.size
        self.last_coverage = float(np.count_nonzero(~np.isnan(dense))) / total * 100.0
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.frames_processed > 0 else DiagnosticStatus.WARN
        status.name = "depth_estimation"
        status.message = "{} frames completed".format(self.frames_processed)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="dense_coverage_percent",
                     value="{:.1f}".format(self.last_coverage)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("depth_estimation_node", anonymous=False)
    try:
        DepthEstimationNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("depth_estimation interrupted.")
    except Exception as exc:
        rospy.logfatal("depth_estimation failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
