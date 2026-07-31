#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_node.py — KITTI Velodyne LiDAR driver node.

Reads Velodyne HDL-64E scans (.bin: x, y, z, reflectance float32) and
publishes them as sensor_msgs/PointCloud2 with fields [x, y, z, intensity].

The .bin payload is already a PointCloud2 payload (four float32 per point), so
the node zero-copy references the file bytes — no repacking.

Publishers:
    /velodyne_points   (sensor_msgs/PointCloud2)   frame: laser

Parameters (private namespace):
    dataset_root, date, drive : KITTI dataset location
    points_topic              : output topic
    frame_id                  : "laser"
    rate_factor               : 1.0 real-time, >1 faster, 0 = as fast as possible
    loop                      : repeat when finished

Run:
    rosrun lidar_node lidar_node.py
"""

import os

import rospy
from sensor_msgs.msg import PointCloud2

from dataset_loader.kitti_parsers import KittiPaths, read_timestamps_ns
from dataset_loader.pacing import RatePacer
from dataset_loader.player_utils import build_point_cloud2_msg, make_header


class LidarNode:
    """Replays a KITTI Velodyne sequence as PointCloud2 topics."""

    def __init__(self):
        root = rospy.get_param("~dataset_root")
        date = rospy.get_param("~date")
        drive = rospy.get_param("~drive")
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.frame_id = rospy.get_param("~frame_id", "laser")
        self.rate_factor = rospy.get_param("~rate_factor", 1.0)
        self.loop = rospy.get_param("~loop", False)

        self.paths = KittiPaths(root, date, drive)
        self.paths.validate()
        self.stamps_ns = read_timestamps_ns(self.paths.timestamps_path)
        rospy.loginfo("lidar_node: %d scans from %s",
                      len(self.stamps_ns), self.paths.velodyne_dir)

        self.pub = rospy.Publisher(self.points_topic, PointCloud2, queue_size=2)
        self.pacer = RatePacer(self.stamps_ns, self.rate_factor)

    def _publish_frame(self, frame: int, ns: int) -> bool:
        path = self.paths.velodyne_path(frame)
        if not os.path.isfile(path):
            rospy.logwarn_throttle(5.0, "Missing scan: %s", path)
            return False
        with open(path, "rb") as handle:
            raw = handle.read()
        self.pub.publish(build_point_cloud2_msg(make_header(self.frame_id, ns), raw))
        return True

    def run(self) -> None:
        rospy.loginfo("lidar_node running (rate=%s)", self.rate_factor)
        frame = 0
        while not rospy.is_shutdown():
            self.pacer.sleep_until_frame(frame)
            self._publish_frame(frame, self.stamps_ns[frame])
            frame += 1
            if frame >= len(self.stamps_ns):
                if not self.loop:
                    rospy.loginfo("lidar_node finished all %d frames.", frame)
                    break
                frame = 0
                rospy.loginfo("lidar_node looping.")
        rospy.loginfo("lidar_node shutting down.")


def main():
    rospy.init_node("lidar_node", anonymous=False)
    try:
        LidarNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("lidar_node interrupted.")
    except Exception as exc:
        rospy.logfatal("lidar_node failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
