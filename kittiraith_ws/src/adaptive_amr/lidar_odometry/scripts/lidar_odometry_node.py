#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_odometry_node.py — LiDAR odometry node (scan-to-scan ICP).

Pipeline:
    /velodyne_points
        -> voxel downsample (lidar_processing.filters.VoxelGrid)
        -> ICP vs the previous scan (point-to-plane by default,
           point-to-point optional)
        -> accumulate pose -> /lidar_odometry (nav_msgs/Odometry)
                               /lidar_odometry/path
                               /lidar_odometry/statistics
                               TF odom -> base_link (enable_tf:=true)

Run:
    rosrun lidar_odometry lidar_odometry_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster, TransformStamped

from dataset_loader.kitti_parsers import matrix_to_quaternion
from dataset_loader.player_utils import pointcloud2_to_arrays
from lidar_odometry.icp import (estimate_normals, icp_point_to_plane,
                                icp_point_to_point)
from lidar_processing.filters import VoxelGrid


class LidarOdometryNode:
    """Registers consecutive LiDAR scans with ICP and accumulates the pose."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.output_topic = rospy.get_param("~output_topic", "/lidar_odometry")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.enable_tf = rospy.get_param("~enable_tf", True)

        self.voxel_leaf = rospy.get_param("~voxel_leaf", 0.5)
        self.method = rospy.get_param("~icp_method", "point_to_plane")
        self.max_iterations = rospy.get_param("~max_iterations", 40)
        self.max_corr_dist = rospy.get_param("~max_correspondence_distance", 2.0)
        self.tolerance = rospy.get_param("~tolerance", 1e-7)
        self.min_points = rospy.get_param("~min_points", 1000)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        if self.method not in ("point_to_plane", "point_to_point"):
            raise ValueError("icp_method must be 'point_to_plane' or "
                             "'point_to_point'")

        self.voxel = VoxelGrid(self.voxel_leaf)

        # ---- state -----------------------------------------------------------------
        self.prev_scan = None
        self.prev_normals = None
        self.T_w_prev = np.eye(4)
        self.last_stamp = None
        self.frames_processed = 0
        self.skipped_frames = 0
        self.last_ms = 0.0
        self.last_rmse = 0.0
        self.path = Path()

        # ---- ROS ----------------------------------------------------------------------
        self.pub_odom = rospy.Publisher(self.output_topic, Odometry, queue_size=5)
        self.pub_path = rospy.Publisher(self.output_topic + "/path", Path, queue_size=1)
        self.pub_stats = rospy.Publisher(self.output_topic + "/statistics",
                                         DiagnosticArray, queue_size=5)
        self.tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber(self.points_topic, PointCloud2, self._on_scan,
                         queue_size=2, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)
        rospy.loginfo("lidar_odometry: %s -> %s (method=%s, leaf=%.2f)",
                      self.points_topic, self.output_topic, self.method,
                      self.voxel_leaf)

    # ------------------------------------------------------------------ #
    def _on_scan(self, msg: PointCloud2) -> None:
        start = rospy.Time.now()
        arrays = pointcloud2_to_arrays(msg)
        points = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
        points = points[np.all(np.isfinite(points), axis=1)]
        if points.shape[0] == 0:
            self.skipped_frames += 1
            return

        scan, _ = self.voxel.downsample(points)
        if scan.shape[0] < self.min_points:
            self.skipped_frames += 1
            rospy.logwarn_throttle(5.0, "lidar_odometry: too few points "
                                         "after downsample (%d)", scan.shape[0])
            return

        if self.prev_scan is None:
            # first scan: initialize and publish identity
            self.prev_scan = scan
            self.prev_normals = estimate_normals(scan, sensor_origin=np.zeros(3))
            self.T_w_prev = np.eye(4)
            self._publish_odom(msg.header.stamp, self.T_w_prev, np.zeros(3))
            self.frames_processed += 1
            return

        # ---- register current scan against previous ---------------------------------
        try:
            if self.method == "point_to_plane":
                T_rel, rmse = icp_point_to_plane(
                    scan, self.prev_scan, self.prev_normals,
                    init=np.eye(4), max_iterations=self.max_iterations,
                    tolerance=self.tolerance,
                    max_correspondence_distance=self.max_corr_dist)
            else:
                T_rel, rmse = icp_point_to_point(
                    scan, self.prev_scan, init=np.eye(4),
                    max_iterations=self.max_iterations,
                    tolerance=max(self.tolerance, 1e-6),
                    max_correspondence_distance=self.max_corr_dist)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "lidar_odometry: %s", exc)
            self.skipped_frames += 1
            return
        self.last_rmse = rmse

        # ---- accumulate: T_w_cur = T_w_prev @ T_rel --------------------------------
        T_w_cur = self.T_w_prev @ T_rel

        # velocity estimate: relative translation / inter-frame time
        velocity = np.zeros(3)
        if self.last_stamp is not None and msg.header.stamp > self.last_stamp:
            dt = (msg.header.stamp - self.last_stamp).to_sec()
            if dt > 0.0:
                velocity = T_rel[:3, 3] / dt
        self.last_stamp = msg.header.stamp
        self._publish_odom(msg.header.stamp, T_w_cur, velocity)

        # ---- roll over state -----------------------------------------------------------
        self.prev_scan = scan
        self.prev_normals = estimate_normals(scan, sensor_origin=np.zeros(3))
        self.T_w_prev = T_w_cur

        self.frames_processed += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_odom(self, stamp, T_w_base, velocity) -> None:
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        q = matrix_to_quaternion(T_w_base[:3, :3])
        odom.pose.pose = Pose(
            position=Point(x=float(T_w_base[0, 3]), y=float(T_w_base[1, 3]),
                           z=float(T_w_base[2, 3])),
            orientation=Quaternion(x=float(q[0]), y=float(q[1]),
                                   z=float(q[2]), w=float(q[3])))
        odom.twist.twist.linear.x = float(velocity[0])
        odom.twist.twist.linear.y = float(velocity[1])
        self.pub_odom.publish(odom)

        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path.header = odom.header
        self.path.poses.append(pose_stamped)
        self.pub_path.publish(self.path)

        if self.enable_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation = odom.pose.pose.position
            tf_msg.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf_msg)

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.frames_processed > 0
                        else DiagnosticStatus.WARN)
        status.name = "lidar_odometry"
        status.message = "{} frames registered".format(self.frames_processed)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="skipped_frames", value=str(self.skipped_frames)),
            KeyValue(key="last_rmse", value="{:.5f}".format(self.last_rmse)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
            KeyValue(key="icp_method", value=self.method),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("lidar_odometry_node", anonymous=False)
    try:
        LidarOdometryNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("lidar_odometry interrupted.")
    except Exception as exc:
        rospy.logfatal("lidar_odometry failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
