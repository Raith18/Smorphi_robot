#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kitti_to_bag.py — convert a KITTI raw drive into a rosbag (offline).

Produces a self-contained bag with the exact topics the online player emits,
so debugging/replays can run without the dataset on disk:

    /camera/image_raw      sensor_msgs/Image          (needs OpenCV for decode)
    /camera/image_rect     sensor_msgs/Image          (KITTI images are rectified)
    /camera/camera_info    sensor_msgs/CameraInfo     (every frame)
    /velodyne_points       sensor_msgs/PointCloud2
    /imu/data              sensor_msgs/Imu
    /gps/fix               sensor_msgs/NavSatFix
    /tf_static             tf2_msgs/TFMessage         (calibration tree)

Usage (run from a ROS-sourced shell):
    rosrun dataset_loader kitti_to_bag.py \
        --root /data/kitti --date 2011_09_26 --drive 2011_09_26_drive_0005 \
        --output /data/kitti/kitti_sample.bag
    # without OpenCV, store JPEGs as CompressedImage:
    rosrun dataset_loader kitti_to_bag.py ... --compressed
"""

import argparse
import os
import sys

import rospy
from rosbag import Bag

from dataset_loader.kitti_parsers import KittiCalib, KittiPaths, OxtsParser, read_timestamps_ns
from dataset_loader.player_utils import (build_camera_info_msg, build_compressed_image_msg,
                                         build_image_msg, build_imu_msg,
                                         build_navsatfix_msg, build_point_cloud2_msg,
                                         build_tf_message, build_transform_stamped,
                                         make_header, transform_translation_quat)

try:
    import cv2  # noqa: F401  (needed only for raw image encoding)
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

IDENTITY_T = [0.0, 0.0, 0.0]
IDENTITY_Q = [0.0, 0.0, 0.0, 1.0]


def ns_to_ros(ns):
    """Integer nanoseconds -> rospy.Time (lossless)."""
    return rospy.Time(secs=int(ns // 1_000_000_000), nsecs=int(ns % 1_000_000_000))


def build_static_tf_message(calib):
    """Assemble the /tf_static message from the KITTI calibration tree."""
    t_velo_cam = calib.velo_to_cam_transform()
    t_imu_cam = calib.imu_to_cam_transform()
    return build_tf_message([
        build_transform_stamped("map", "odom", IDENTITY_T, IDENTITY_Q),
        build_transform_stamped("odom", "base_link", IDENTITY_T, IDENTITY_Q),
        build_transform_stamped("base_link", "laser", IDENTITY_T, IDENTITY_Q),
        build_transform_stamped("laser", "camera_link",
                                *transform_translation_quat(t_velo_cam)),
        build_transform_stamped("camera_link", "camera_optical_frame",
                                IDENTITY_T, IDENTITY_Q),
        build_transform_stamped("camera_link", "imu_link",
                                *transform_translation_quat(t_imu_cam)),
    ])


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Convert a KITTI raw drive to a rosbag")
    parser.add_argument("--root", default=os.environ.get("KITTI_ROOT", "/data/kitti"),
                        help="dataset root (default: $KITTI_ROOT or /data/kitti)")
    parser.add_argument("--date", default="2011_09_26")
    parser.add_argument("--drive", default="2011_09_26_drive_0005")
    parser.add_argument("--output", required=True, help="output .bag path")
    parser.add_argument("--cameras", default="2", help="comma-separated camera indexes (default 2)")
    parser.add_argument("--compressed", action="store_true",
                        help="store images as CompressedImage (no OpenCV needed)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if not args.compressed and not HAVE_CV2:
        rospy.logerr("OpenCV (cv2) is required for raw image encoding. "
                     "Install python3-opencv or re-run with --compressed.")
        return 1

    paths = KittiPaths(args.root, args.date, args.drive)
    paths.validate()
    calib = KittiCalib(paths.calib_dir, args.date)
    stamps_ns = read_timestamps_ns(paths.timestamps_path)
    oxts = OxtsParser(paths.oxts_dir)
    cameras = [int(c) for c in args.cameras.split(",")]

    rospy.init_node("kitti_to_bag", anonymous=True, disable_signals=True)
    log = rospy.loginfo

    log("Writing bag: %s", args.output)
    with Bag(args.output, "w") as bag:
        bag.write("/tf_static", build_static_tf_message(calib), rospy.Time(0))

        for frame, ns in enumerate(stamps_ns):
            header = make_header("camera_optical_frame", ns)

            # ---- cameras ------------------------------------------------------
            for cam in cameras:
                img_path = paths.image_path(frame, cam)
                if not os.path.isfile(img_path):
                    continue
                with open(img_path, "rb") as handle:
                    jpg = handle.read()
                if args.compressed:
                    msg = build_compressed_image_msg(header, jpg)
                else:
                    msg = build_image_msg(header, jpg)
                bag.write("/camera/image_raw", msg, ns_to_ros(ns))
                if cam == cameras[0]:
                    bag.write("/camera/image_rect", msg, ns_to_ros(ns))
                    ci = build_camera_info_msg("camera_optical_frame", calib, cam,
                                               ns_to_ros(ns))
                    bag.write("/camera/camera_info", ci, ns_to_ros(ns))

            # ---- velodyne ------------------------------------------------------
            velo_path = paths.velodyne_path(frame)
            if os.path.isfile(velo_path):
                with open(velo_path, "rb") as handle:
                    bag.write("/velodyne_points",
                              build_point_cloud2_msg(make_header("laser", ns), handle.read()),
                              ns_to_ros(ns))

            # ---- imu / gps -------------------------------------------------------
            sample = oxts.read_sample(frame)
            if sample is not None:
                bag.write("/imu/data",
                          build_imu_msg(make_header("imu_link", ns), sample), ns_to_ros(ns))
                bag.write("/gps/fix",
                          build_navsatfix_msg(make_header("imu_link", ns), sample), ns_to_ros(ns))

            if frame % 100 == 0:
                log("frame %d/%d", frame, len(stamps_ns))

    log("Done. %d frames -> %s", len(stamps_ns), args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
