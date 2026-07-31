#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
localization_node.py — ICP scan-to-map localization node (no EKF).

Synchronizes /velodyne_points with /lidar_odometry:

  * Mapping phase:   accumulates scans (transformed by the odometry pose)
                     into a voxel map; T_map_odom = identity.
  * Localization:    registers each scan against the map with point-to-plane
                     ICP; publishes the correction + refined pose.

Publishers:
    /localization_pose          (geometry_msgs/PoseWithCovarianceStamped, map)
    /localization/map           (sensor_msgs/PointCloud2, downsampled map)
    /localization/statistics    (DiagnosticArray)
    TF map -> odom              (dynamic, the ICP correction)

Run:
    rosrun localization localization_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster, TransformStamped

from dataset_loader.kitti_parsers import matrix_to_quaternion
from dataset_loader.player_utils import (build_point_cloud2_from_arrays,
                                         pointcloud2_to_arrays)
from lidar_processing.filters import VoxelGrid
from localization.icp_localizer import IcpLocalizer, transform_points
from adaptive_amr_msgs.srv import Relocalize, RelocalizeResponse


def odom_to_matrix(msg: Odometry) -> np.ndarray:
    """nav_msgs/Odometry pose -> 4x4 matrix (odom -> base_link)."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    T = np.eye(4)
    T[:3, 3] = [p.x, p.y, p.z]
    T[:3, :3] = _quat_to_matrix(q)
    return T


class LocalizationNode:
    """ICP-based localization on top of LiDAR odometry."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.odom_topic = rospy.get_param("~odom_topic", "/lidar_odometry")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.output_topic = rospy.get_param("~output_topic", "/localization_pose")
        self.enable_tf = rospy.get_param("~enable_tf", True)
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.1)

        self.voxel_leaf = rospy.get_param("~voxel_leaf", 0.5)
        self.min_map_points = rospy.get_param("~min_map_points", 60_000)
        self.max_map_points = rospy.get_param("~max_map_points", 400_000)
        self.max_mapping_frames = rospy.get_param("~max_mapping_frames", 200)
        self.max_corr_dist = rospy.get_param("~max_correspondence_distance", 2.0)
        self.max_translation_jump = rospy.get_param("~max_translation_jump", 5.0)
        self.publish_map_interval = rospy.get_param("~publish_map_interval", 10)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.localizer = IcpLocalizer(
            voxel_size=self.voxel_leaf,
            min_map_points=self.min_map_points,
            max_map_points=self.max_map_points,
            max_mapping_frames=self.max_mapping_frames,
            max_corr_dist=self.max_corr_dist,
            max_translation_jump=self.max_translation_jump)
        self.scan_voxel = VoxelGrid(self.voxel_leaf)

        # ---- ROS ------------------------------------------------------------------
        self.pub_pose = rospy.Publisher(self.output_topic,
                                        PoseWithCovarianceStamped, queue_size=5)
        self.pub_map = rospy.Publisher("/localization/map", PointCloud2,
                                       queue_size=1, latch=True)
        self.pub_stats = rospy.Publisher("/localization/statistics",
                                         DiagnosticArray, queue_size=5)
        self.tf_broadcaster = TransformBroadcaster()

        self.sub_points = Subscriber(self.points_topic, PointCloud2)
        self.sub_odom = Subscriber(self.odom_topic, Odometry)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_points, self.sub_odom], self.queue_size, self.approx_slop)
        self.sync.registerCallback(self._on_synced)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        # ---- service: trigger re-localization ----------------------------------
        self.srv_relocalize = rospy.Service(
            "/localization/relocalize", Relocalize, self._on_relocalize)

        self.frame_count = 0
        self.last_process_ms = 0.0
        rospy.loginfo("localization: %s + %s (mapping until %d pts)",
                      self.points_topic, self.odom_topic, self.min_map_points)

    # ------------------------------------------------------------------ #
    def _on_relocalize(self, req) -> RelocalizeResponse:
        """
        Relocalize: reset the ICP correction (and optionally the whole map).

        Industrial use: after a kidnapping event (robot picked up and moved),
        a supervisor calls this service to force the localizer back into a
        known state instead of trusting a drifted pose.
        """
        if req.restart_mapping:
            self.localizer.reset()
            self.frame_count = 0
            message = ("map reset and relocalization started (mapping phase)")
        else:
            # keep the map, but re-anchor the pose at the odometry prior
            self.localizer.T_map_odom = np.eye(4)
            self.localizer.last_correction = np.eye(4)
            message = ("relocalized: map->odom reset to identity "
                       "(pose = odometry prior)")
        rospy.loginfo("localization: relocalize requested -> %s", message)
        return RelocalizeResponse(success=True, message=message)

    # ------------------------------------------------------------------ #
    def _on_synced(self, points: PointCloud2, odom: Odometry) -> None:
        import time
        t0 = time.time()
        self.frame_count += 1

        arrays = pointcloud2_to_arrays(points)
        scan = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
        scan = scan[np.all(np.isfinite(scan), axis=1)]
        if scan.shape[0] < 100:
            return
        scan, _ = self.scan_voxel.downsample(scan)

        T_odom_base = odom_to_matrix(odom)

        if self.localizer.mapping:
            self.localizer.add_scan_to_map(scan, T_odom_base)
            if not self.localizer.mapping:
                rospy.loginfo("localization: mapping finished (%d points)",
                              self.localizer.map_points.shape[0])
                self._publish_map()
            return

        # ---- localization phase ---------------------------------------------------
        T_map_odom, T_map_base, error = self.localizer.localize(scan, T_odom_base)

        # ---- publish refined pose (base in map frame) ------------------------------
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = points.header.stamp
        msg.header.frame_id = self.map_frame
        q = matrix_to_quaternion(T_map_base[:3, :3])
        msg.pose = PoseWithCovariance()
        msg.pose.pose.position.x = float(T_map_base[0, 3])
        msg.pose.pose.position.y = float(T_map_base[1, 3])
        msg.pose.pose.position.z = float(T_map_base[2, 3])
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])
        self.pub_pose.publish(msg)

        # ---- TF map -> odom ----------------------------------------------------------
        if self.enable_tf:
            qm = matrix_to_quaternion(T_map_odom[:3, :3])
            tf_msg = TransformStamped()
            tf_msg.header.stamp = points.header.stamp
            tf_msg.header.frame_id = self.map_frame
            tf_msg.child_frame_id = self.odom_frame
            tf_msg.transform.translation.x = float(T_map_odom[0, 3])
            tf_msg.transform.translation.y = float(T_map_odom[1, 3])
            tf_msg.transform.translation.z = float(T_map_odom[2, 3])
            tf_msg.transform.rotation.x = float(qm[0])
            tf_msg.transform.rotation.y = float(qm[1])
            tf_msg.transform.rotation.z = float(qm[2])
            tf_msg.transform.rotation.w = float(qm[3])
            self.tf_broadcaster.sendTransform(tf_msg)

        if self.frame_count % self.publish_map_interval == 0:
            self._publish_map()

        self.last_process_ms = (time.time() - t0) * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_map(self) -> None:
        """Publish the (downsampled) map cloud for visualization."""
        pts = self.localizer.map_points
        if pts.shape[0] == 0:
            return
        header = _make_header(self.map_frame)
        self.pub_map.publish(build_point_cloud2_from_arrays(
            header, {"x": pts[:, 0], "y": pts[:, 1], "z": pts[:, 2]}))

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.localizer.map_ready
                        else DiagnosticStatus.WARN)
        status.name = "localization"
        status.message = ("mapping" if self.localizer.mapping
                          else "localizing ({} pts map)".format(
                              self.localizer.map_points.shape[0]))
        status.values = [
            KeyValue(key="phase", value="mapping" if self.localizer.mapping
                     else "localizing"),
            KeyValue(key="map_points", value=str(self.localizer.map_points.shape[0])),
            KeyValue(key="frames_localized",
                     value=str(self.localizer.frames_localized)),
            KeyValue(key="last_icp_error",
                     value="{:.5f}".format(self.localizer.last_error)
                     if np.isfinite(self.localizer.last_error) else "n/a"),
            KeyValue(key="last_process_ms",
                     value="{:.2f}".format(self.last_process_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def _make_header(frame_id):
    from std_msgs.msg import Header
    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()
    return header


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("localization_node", anonymous=False)
    try:
        LocalizationNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("localization interrupted.")
    except Exception as exc:
        rospy.logfatal("localization failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
