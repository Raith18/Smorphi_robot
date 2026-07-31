#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
time_sync_node.py — multi-sensor time synchronization node.

Goal: emit *only* sets of messages (image, point cloud, IMU, GPS) that
describe the same physical instant, so downstream fusion never mixes data
from different times. This is the gatekeeper every sensor-fusion stack has.

Two strategies (configurable via `sync_type`):

  * exact   — message_filters.TimeSynchronizer: requires identical header
              stamps. Works perfectly on KITTI _sync drives (all sensors
              share one timestamp per frame). Zero latency jitter by
              construction; drops any message without its partners.
  * approx  — message_filters.ApproximateTimeSynchronizer: pairs messages
              within `approx_slop` seconds. This is what real robots need,
              where camera/LiDAR/IMU clocks are never perfectly aligned.

Publishers:
    /time_sync/image    (same type as input)   republished synchronized image
    /time_sync/points   (PointCloud2)          synchronized point cloud
    /time_sync/imu      (Imu)                  synchronized IMU sample
    /time_sync/gps      (NavSatFix)            synchronized GPS fix
    /time_sync/statistics (DiagnosticArray)    counts, drops, latency @1 Hz

Subscribers:
    /camera/image_raw   /velodyne_points   /imu/data   (/gps/fix)

Parameters (private namespace, see config/time_sync.yaml):
    sync_type, queue_size, approx_slop, diagnostics_rate, subscribe_gps

Run:
    rosrun time_sync time_sync_node.py
"""

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
from sensor_msgs.msg import Imu, Image, NavSatFix, PointCloud2


class TimeSyncNode:
    """Synchronizes camera/LiDAR/IMU/GPS streams and reports diagnostics."""

    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.subscribe_gps = rospy.get_param("~subscribe_gps", True)
        self.gps_topic = rospy.get_param("~gps_topic", "/gps/fix")
        self.sync_type = rospy.get_param("~sync_type", "exact")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.05)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)
        self.synced_ns = rospy.get_param("~synced_ns", "/time_sync")

        # ---- subscribers (message_filters) ---------------------------------
        self.sub_image = Subscriber(self.image_topic, Image)
        self.sub_points = Subscriber(self.points_topic, PointCloud2)
        self.sub_imu = Subscriber(self.imu_topic, Imu)
        self.sub_gps = Subscriber(self.gps_topic, NavSatFix)

        # ---- publishers ------------------------------------------------------
        self.pub_image = rospy.Publisher("{}/image".format(self.synced_ns), Image, queue_size=10)
        self.pub_points = rospy.Publisher("{}/points".format(self.synced_ns), PointCloud2, queue_size=10)
        self.pub_imu = rospy.Publisher("{}/imu".format(self.synced_ns), Imu, queue_size=10)
        self.pub_gps = rospy.Publisher("{}/gps".format(self.synced_ns), NavSatFix, queue_size=10)
        self.pub_stats = rospy.Publisher("{}/statistics".format(self.synced_ns),
                                         DiagnosticArray, queue_size=5)

        # ---- statistics ------------------------------------------------------
        self.counts = {"image": 0, "points": 0, "imu": 0, "gps": 0}
        self.synced_sets = 0
        self.max_latency_s = 0.0

        # ---- synchronizer ------------------------------------------------------
        subs = [self.sub_image, self.sub_points, self.sub_imu]
        if self.subscribe_gps:
            subs.append(self.sub_gps)
        self._subs = subs

        if self.sync_type == "exact":
            self.sync = TimeSynchronizer(subs, self.queue_size)
            rospy.loginfo("time_sync: EXACT synchronizer (identical stamps)")
        elif self.sync_type == "approx":
            self.sync = ApproximateTimeSynchronizer(subs, self.queue_size,
                                                    self.approx_slop)
            rospy.loginfo("time_sync: APPROXIMATE synchronizer (slop=%.3fs)",
                          self.approx_slop)
        else:
            raise ValueError("sync_type must be 'exact' or 'approx', got '{}'".format(
                self.sync_type))

        self.sync.registerCallback(self._on_synced)

        # Count received messages per stream (for diagnostics)
        self.sub_image.registerCallback(lambda msg: self._count("image"))
        self.sub_points.registerCallback(lambda msg: self._count("points"))
        self.sub_imu.registerCallback(lambda msg: self._count("imu"))
        if self.subscribe_gps:
            self.sub_gps.registerCallback(lambda msg: self._count("gps"))

        # diagnostics timer
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

    # ------------------------------------------------------------------ #
    def _count(self, stream: str) -> None:
        """Increment the received-message counter for a stream."""
        self.counts[stream] += 1

    # ------------------------------------------------------------------ #
    def _on_synced(self, *msgs) -> None:
        """Republish the synchronized set and update statistics."""
        self.synced_sets += 1

        # Order matches `subs`: image, points, imu, (gps)
        self.pub_image.publish(msgs[0])
        self.pub_points.publish(msgs[1])
        self.pub_imu.publish(msgs[2])
        if self.subscribe_gps:
            self.pub_gps.publish(msgs[3])

        stamps = [m.header.stamp.to_sec() for m in msgs if hasattr(m, "header")]
        if len(stamps) >= 2:
            latency = max(stamps) - min(stamps)
            self.max_latency_s = max(self.max_latency_s, latency)

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        """Publish a DiagnosticArray with sync health metrics (1 Hz)."""
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.synced_sets > 0 else DiagnosticStatus.WARN
        status.name = "time_sync/{}".format(self.sync_type)
        status.message = "{} synced sets".format(self.synced_sets)
        status.values = [
            KeyValue(key="received/image", value=str(self.counts["image"])),
            KeyValue(key="received/points", value=str(self.counts["points"])),
            KeyValue(key="received/imu", value=str(self.counts["imu"])),
            KeyValue(key="received/gps", value=str(self.counts["gps"])),
            KeyValue(key="synced_sets", value=str(self.synced_sets)),
            KeyValue(key="max_set_latency_s", value="{:.6f}".format(self.max_latency_s)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.loginfo("time_sync running: %s -> %s", self.sync_type, self.synced_ns)
        rospy.spin()


def main():
    rospy.init_node("time_sync_node", anonymous=False)
    try:
        TimeSyncNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("time_sync interrupted.")
    except Exception as exc:
        rospy.logfatal("time_sync failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
