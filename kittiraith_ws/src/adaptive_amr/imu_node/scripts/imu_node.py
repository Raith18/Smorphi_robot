#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_node.py — KITTI IMU driver node.

Reads the OXTS (GPS/IMU) samples of a KITTI drive and publishes the inertial
part as sensor_msgs/Imu (/imu/data):

    * orientation        : fused roll/pitch/yaw from the OXTS navigation
                           solution -> quaternion (drift-free in KITTI because
                           the GPS-aided filter produces it)
    * angular_velocity   : (wx, wy, wz) rad/s in the vehicle/IMU frame
    * linear_acceleration: (ax, ay, az) m/s^2 in the vehicle/IMU frame

Publishers:
    /imu/data   (sensor_msgs/Imu)   frame: imu_link

Parameters (private namespace):
    dataset_root, date, drive : KITTI dataset location
    imu_topic                 : output topic
    frame_id                  : "imu_link"
    rate_factor               : pacing (1.0 real-time, 0 = as fast as possible)
    loop                      : repeat when finished

Run:
    rosrun imu_node imu_node.py
"""

import rospy
from sensor_msgs.msg import Imu

from dataset_loader.kitti_parsers import KittiPaths, OxtsParser, read_timestamps_ns
from dataset_loader.pacing import RatePacer
from dataset_loader.player_utils import build_imu_msg, make_header


class ImuNode:
    """Replays KITTI OXTS inertial data as Imu topics."""

    def __init__(self):
        root = rospy.get_param("~dataset_root")
        date = rospy.get_param("~date")
        drive = rospy.get_param("~drive")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        self.rate_factor = rospy.get_param("~rate_factor", 1.0)
        self.loop = rospy.get_param("~loop", False)

        self.paths = KittiPaths(root, date, drive)
        self.paths.validate()
        self.stamps_ns = read_timestamps_ns(self.paths.timestamps_path)
        self.oxts = OxtsParser(self.paths.oxts_dir)
        rospy.loginfo("imu_node: %d samples expected from %s",
                      len(self.stamps_ns), self.paths.oxts_dir)

        self.pub = rospy.Publisher(self.imu_topic, Imu, queue_size=2)
        self.pacer = RatePacer(self.stamps_ns, self.rate_factor)

    def _publish_frame(self, frame: int, ns: int) -> bool:
        sample = self.oxts.read_sample(frame)
        if sample is None:
            rospy.logwarn_throttle(5.0, "Missing OXTS sample for frame %d", frame)
            return False
        self.pub.publish(build_imu_msg(make_header(self.frame_id, ns), sample))
        return True

    def run(self) -> None:
        rospy.loginfo("imu_node running (rate=%s)", self.rate_factor)
        frame = 0
        while not rospy.is_shutdown():
            self.pacer.sleep_until_frame(frame)
            self._publish_frame(frame, self.stamps_ns[frame])
            frame += 1
            if frame >= len(self.stamps_ns):
                if not self.loop:
                    rospy.loginfo("imu_node finished all %d frames.", frame)
                    break
                frame = 0
                rospy.loginfo("imu_node looping.")
        rospy.loginfo("imu_node shutting down.")


def main():
    rospy.init_node("imu_node", anonymous=False)
    try:
        ImuNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("imu_node interrupted.")
    except Exception as exc:
        rospy.logfatal("imu_node failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
