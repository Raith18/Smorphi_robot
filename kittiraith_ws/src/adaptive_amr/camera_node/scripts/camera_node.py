#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_node.py — KITTI camera driver node.

Reads one KITTI camera image sequence (image_00..03, 10 Hz JPEG) and
publishes it as ROS topics. The node is deliberately structured like a real
camera driver: it owns the sensor (here: a directory of files) and publishes
standardized messages with accurate timestamps.

Publishers:
    /camera/image_raw    (sensor_msgs/Image)            or CompressedImage
    /camera/image_rect   (sensor_msgs/Image)            (KITTI images are
                                                         already rectified)

Parameters (private namespace, set by camera_node.launch / YAML):
    dataset_root, date, drive : KITTI dataset location
    camera_index              : 0..3 (2 = left color, default)
    image_topic, rect_topic   : output topic names
    frame_id                  : "camera_optical_frame"
    encoding                  : "image" (decode via OpenCV) | "compressed"
    publish_rect_copy         : also publish /camera/image_rect (KITTI rectified)
    rate_factor               : 1.0 real-time, >1 faster, 0 as fast as possible
    loop                      : repeat the sequence when finished

Run:
    rosrun camera_node camera_node.py
"""

import os

import rospy
from sensor_msgs.msg import CompressedImage, Image

from dataset_loader.kitti_parsers import KittiPaths, read_timestamps_ns
from dataset_loader.pacing import RatePacer
from dataset_loader.player_utils import (build_compressed_image_msg,
                                         build_image_msg, make_header)

try:
    import cv2  # noqa: F401  (only needed for the "image" encoding)
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


class CameraNode:
    """Replays a KITTI camera sequence as ROS image topics."""

    def __init__(self):
        # ---- configuration (all from ROS params / YAML, never hardcoded) ----
        root = rospy.get_param("~dataset_root")
        date = rospy.get_param("~date")
        drive = rospy.get_param("~drive")
        self.cam_index = rospy.get_param("~camera_index", 2)
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.rect_topic = rospy.get_param("~rect_topic", "/camera/image_rect")
        self.frame_id = rospy.get_param("~frame_id", "camera_optical_frame")
        self.encoding = rospy.get_param("~encoding", "image")
        self.publish_rect = rospy.get_param("~publish_rect_copy", True)
        self.rate_factor = rospy.get_param("~rate_factor", 1.0)
        self.loop = rospy.get_param("~loop", False)

        if self.encoding == "image" and not HAVE_CV2:
            rospy.logwarn("OpenCV not available — falling back to CompressedImage")
            self.encoding = "compressed"

        # ---- dataset access ---------------------------------------------------
        self.paths = KittiPaths(root, date, drive)
        self.paths.validate()
        self.stamps_ns = read_timestamps_ns(self.paths.timestamps_path)
        rospy.loginfo("camera_node: %d frames from %s (image_%02d)",
                      len(self.stamps_ns), self.paths.drive_dir, self.cam_index)

        # ---- publishers --------------------------------------------------------
        self.pub_raw = rospy.Publisher(self.image_topic, Image
                                       if self.encoding == "image"
                                       else CompressedImage, queue_size=2)
        if self.publish_rect:
            self.pub_rect = rospy.Publisher(self.rect_topic, Image
                                            if self.encoding == "image"
                                            else CompressedImage, queue_size=2)
        self.pacer = RatePacer(self.stamps_ns, self.rate_factor)

    # ------------------------------------------------------------------ #
    def _read_image_file(self, frame: int) -> bytes:
        """Read the JPEG bytes for `frame`, raising FileNotFoundError if absent."""
        path = self.paths.image_path(frame, self.cam_index)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        with open(path, "rb") as handle:
            return handle.read()

    # ------------------------------------------------------------------ #
    def _publish_frame(self, frame: int, ns: int) -> bool:
        """Publish one frame. Returns False when the file is missing."""
        try:
            jpg = self._read_image_file(frame)
        except FileNotFoundError as exc:
            rospy.logwarn_throttle(5.0, "Missing image: %s", exc)
            return False

        header = make_header(self.frame_id, ns)
        if self.encoding == "image":
            msg = build_image_msg(header, jpg)
        else:
            msg = build_compressed_image_msg(header, jpg)
        self.pub_raw.publish(msg)
        if self.publish_rect:
            self.pub_rect.publish(msg)      # KITTI _sync images are rectified
        return True

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        """Main loop: pace and publish every frame (optionally looping)."""
        rospy.loginfo("camera_node running (encoding=%s, rate=%s)",
                      self.encoding, self.rate_factor)
        frame = 0
        while not rospy.is_shutdown():
            self.pacer.sleep_until_frame(frame)
            self._publish_frame(frame, self.stamps_ns[frame])
            frame += 1
            if frame >= len(self.stamps_ns):
                if not self.loop:
                    rospy.loginfo("camera_node finished all %d frames.", frame)
                    break
                frame = 0
                rospy.loginfo("camera_node looping.")
        rospy.loginfo("camera_node shutting down.")


def main():
    rospy.init_node("camera_node", anonymous=False)
    try:
        node = CameraNode()
        node.run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("camera_node interrupted.")
    except Exception as exc:  # never die silently on the ROS bus
        rospy.logfatal("camera_node failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
