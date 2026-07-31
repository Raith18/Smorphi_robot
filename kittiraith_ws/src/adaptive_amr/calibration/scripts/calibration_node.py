#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibration_node.py — KITTI calibration server.

Loads the three KITTI calibration files and exposes them on the ROS bus:

  1. sensor_msgs/CameraInfo for every camera (latched), including the alias
     /camera/camera_info for camera 02 (the image published by camera_node).
     KITTI images are already rectified, so R = R_rect_00 and P = P_rect_0X.

  2. Static TF tree (latched /tf_static):

        map -> odom -> base_link -> laser -> camera_link -> imu_link
                                             |
                                             +-> camera_optical_frame

     * map->odom and odom->base_link are identity placeholders (Phase 5
       localization will make them dynamic).
     * base_link -> laser is identity: the KITTI velodyne defines our base.
     * laser -> camera_link  = T_velo_cam0  (from calib_velo_to_cam.txt)
     * camera_link -> imu_link = T_velo_cam0 * T_imu_velo (chained extrinsics)

Publishers:
    /camera_00/camera_info .. /camera_03/camera_info  (CameraInfo, latched)
    /camera/camera_info                                (alias for cam 02)
    /tf_static                                         (TFMessage)

Parameters (private namespace):
    dataset_root, date, drive : KITTI dataset location
    camera_info_ns            : namespace prefix for camera_info topics
    publish_placeholders      : emit identity map->odom / odom->base_link TFs

Run:
    rosrun calibration calibration_node.py
"""

import rospy
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from dataset_loader.kitti_parsers import KittiCalib, KittiPaths, matrix_to_quaternion
from dataset_loader.player_utils import (build_camera_info_msg,
                                         build_transform_stamped)


class CalibrationNode:
    """Publishes CameraInfo and the static TF tree from KITTI calibration."""

    def __init__(self):
        root = rospy.get_param("~dataset_root")
        date = rospy.get_param("~date")
        drive = rospy.get_param("~drive")
        self.camera_info_ns = rospy.get_param("~camera_info_ns", "/camera")
        self.publish_placeholders = rospy.get_param("~publish_placeholders", True)

        # Frame names (fixed by the project TF tree)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.laser_frame = rospy.get_param("~laser_frame", "laser")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.camera_optical_frame = rospy.get_param("~camera_optical_frame",
                                                    "camera_optical_frame")
        self.imu_frame = rospy.get_param("~imu_frame", "imu_link")

        # ---- load calibration -----------------------------------------------
        self.paths = KittiPaths(root, date, drive)
        self.paths.validate()
        self.calib = KittiCalib(self.paths.calib_dir, date)
        rospy.loginfo("calibration_node: loaded KITTI calibration from %s",
                      self.paths.calib_dir)

        # ---- publishers --------------------------------------------------------
        self.camera_info_pubs = {}
        for cam in range(4):
            topic = "{}/camera_{:02d}/camera_info".format(self.camera_info_ns, cam)
            self.camera_info_pubs[cam] = rospy.Publisher(topic, CameraInfo,
                                                         queue_size=1, latch=True)
        # alias for the camera used at /camera/image_raw (left color, cam 02)
        self.camera_info_pubs["main"] = rospy.Publisher(
            "{}/camera_info".format(self.camera_info_ns), CameraInfo,
            queue_size=1, latch=True)

        self.tf_broadcaster = StaticTransformBroadcaster()

    # ------------------------------------------------------------------ #
    def _publish_camera_info(self) -> None:
        """Publish latched CameraInfo for cameras 0..3 (+ alias for cam 2)."""
        for cam in range(4):
            msg = build_camera_info_msg(self.camera_optical_frame, self.calib, cam)
            self.camera_info_pubs[cam].publish(msg)
            rospy.loginfo("calibration_node: camera_info cam %d (%dx%d)",
                          cam, msg.width, msg.height)
        main_msg = build_camera_info_msg(self.camera_optical_frame, self.calib, 2)
        self.camera_info_pubs["main"].publish(main_msg)

    # ------------------------------------------------------------------ #
    def _publish_static_tf(self) -> None:
        """Broadcast the static TF tree derived from the KITTI extrinsics."""
        identity_t = [0.0, 0.0, 0.0]
        identity_q = [0.0, 0.0, 0.0, 1.0]

        t_velo_cam = self.calib.velo_to_cam_transform()   # laser -> camera_link
        t_imu_cam = self.calib.imu_to_cam_transform()     # imu -> camera_link

        transforms = []
        if self.publish_placeholders:
            transforms.append(build_transform_stamped(
                self.map_frame, self.odom_frame, identity_t, identity_q))
            transforms.append(build_transform_stamped(
                self.odom_frame, self.base_frame, identity_t, identity_q))
        transforms.append(build_transform_stamped(
            self.base_frame, self.laser_frame, identity_t, identity_q))
        transforms.append(build_transform_stamped(
            self.laser_frame, self.camera_frame,
            t_velo_cam[:3, 3], matrix_to_quaternion(t_velo_cam[:3, :3])))
        transforms.append(build_transform_stamped(
            self.camera_frame, self.camera_optical_frame, identity_t, identity_q))
        transforms.append(build_transform_stamped(
            self.camera_frame, self.imu_frame,
            t_imu_cam[:3, 3], matrix_to_quaternion(t_imu_cam[:3, :3])))

        for t in transforms:
            rospy.loginfo("calibration_node: static TF %s -> %s",
                          t.header.frame_id, t.child_frame_id)
        self.tf_broadcaster.sendTransform(transforms)

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        """Publish everything once (latched), then keep the node alive."""
        self._publish_camera_info()
        self._publish_static_tf()
        rospy.loginfo("calibration_node: published CameraInfo + static TF. "
                      "Node stays alive to keep latched topics available.")
        rospy.spin()


def main():
    rospy.init_node("calibration_node", anonymous=False)
    try:
        CalibrationNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("calibration_node interrupted.")
    except Exception as exc:
        rospy.logfatal("calibration_node failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
