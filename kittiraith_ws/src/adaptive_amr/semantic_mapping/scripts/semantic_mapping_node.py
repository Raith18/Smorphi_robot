#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
semantic_mapping_node.py — global semantic map node.

Subscribes to the semantic-colored LiDAR (/semantic_map/colored_points) and
the refined pose (/localization_pose, fallback /lidar_odometry) and
accumulates them into a global voxel map published on /semantic_map.

Publishers:
    /semantic_map           (sensor_msgs/PointCloud2, xyz + rgb)
    /semantic_map/statistics (DiagnosticArray)

Run:
    rosrun semantic_mapping semantic_mapping_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from adaptive_amr_msgs.srv import RelocalizeResponse
from dataset_loader.kitti_parsers import matrix_to_quaternion
from dataset_loader.player_utils import (build_point_cloud2_from_arrays,
                                         pointcloud2_to_arrays)
from semantic_mapping.map_builder import SemanticMapBuilder


def _pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """(N,3) uint8 RGB -> float32 view of packed uint32."""
    packed = (rgb[:, 0].astype(np.uint32) << 16 |
              rgb[:, 1].astype(np.uint32) << 8 |
              rgb[:, 2].astype(np.uint32))
    return packed.view(np.float32)


def _pose_msg_to_matrix(pose):
    """geometry_msgs/Pose -> 4x4 (map -> base_link)."""
    p = pose.position
    q = pose.orientation
    T = np.eye(4)
    T[:3, 3] = [p.x, p.y, p.z]
    T[:3, :3] = _quat_to_matrix(q)
    return T


class SemanticMappingNode:
    """Accumulates semantic-colored clouds into a global map."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.colored_topic = rospy.get_param("~colored_points_topic",
                                             "/semantic_map/colored_points")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization_pose")
        self.fallback_odom_topic = rospy.get_param("~fallback_odom_topic",
                                                   "/lidar_odometry")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.voxel_size = rospy.get_param("~voxel_size", 0.2)
        self.max_voxels = rospy.get_param("~max_voxels", 400_000)
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.1)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)
        self.use_localization = rospy.get_param("~use_localization", True)

        self.builder = SemanticMapBuilder(self.voxel_size, self.max_voxels)

        # ---- ROS ------------------------------------------------------------------
        self.pub_map = rospy.Publisher("/semantic_map", PointCloud2,
                                       queue_size=1, latch=True)
        self.pub_stats = rospy.Publisher("/semantic_map/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        self.sub_points = Subscriber(self.colored_topic, PointCloud2)
        if self.use_localization:
            self.sub_pose = Subscriber(self.pose_topic, PoseWithCovarianceStamped)
        else:
            self.sub_pose = Subscriber(self.fallback_odom_topic, Odometry)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_points, self.sub_pose], self.queue_size, self.approx_slop)
        self.sync.registerCallback(self._on_synced)

        # service: clear the map (mirrors localization relocalize)
        from adaptive_amr_msgs.srv import Relocalize
        self.srv_clear = rospy.Service("/semantic_map/clear", Relocalize,
                                       self._on_clear)

        self.frames_added = 0
        self.last_ms = 0.0
        rospy.loginfo("semantic_mapping: %s + %s -> /semantic_map (voxel=%.2f)",
                      self.colored_topic, self.pose_topic, self.voxel_size)

    # ------------------------------------------------------------------ #
    def _on_clear(self, req):
        """Clear the map (service) — restarts building from the next frame."""
        self.builder.clear()
        self.pub_map.publish(_empty_cloud(self.map_frame))
        return RelocalizeResponse(success=True, message="semantic map cleared")

    # ------------------------------------------------------------------ #
    def _on_synced(self, points: PointCloud2, pose_msg) -> None:
        start = rospy.Time.now()
        arrays = pointcloud2_to_arrays(points)
        xyz = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
        # rgb stored as float32 view of packed uint32 -> unpack
        if "rgb" in arrays:
            packed = arrays["rgb"].view(np.uint32)
            rgb = np.stack([(packed >> 16) & 0xFF, (packed >> 8) & 0xFF,
                            packed & 0xFF], axis=1).astype(np.uint8)
        else:
            rgb = np.zeros((xyz.shape[0], 3), dtype=np.uint8)

        T_map_base = _pose_msg_to_matrix(pose_msg.pose.pose)
        self.builder.add_frame(xyz, rgb, T_map_base)

        map_xyz, map_rgb = self.builder.get_map()
        if map_xyz.shape[0] > 0:
            cloud = build_point_cloud2_from_arrays(
                _make_header(self.map_frame),
                {"x": map_xyz[:, 0], "y": map_xyz[:, 1], "z": map_xyz[:, 2],
                 "rgb": _pack_rgb(map_rgb)})
            self.pub_map.publish(cloud)

        self.frames_added += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        map_xyz, _ = self.builder.get_map()
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if map_xyz.shape[0] > 0
                        else DiagnosticStatus.WARN)
        status.name = "semantic_mapping"
        status.message = "{} voxels".format(map_xyz.shape[0])
        status.values = [
            KeyValue(key="frames_added", value=str(self.frames_added)),
            KeyValue(key="map_voxels", value=str(map_xyz.shape[0])),
            KeyValue(key="voxel_size", value="{:.2f}".format(self.builder.voxel_size)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
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


def _empty_cloud(frame_id):
    return build_point_cloud2_from_arrays(
        _make_header(frame_id),
        {"x": np.zeros(0, np.float32), "y": np.zeros(0, np.float32),
         "z": np.zeros(0, np.float32), "rgb": np.zeros(0, np.float32)})


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("semantic_mapping_node", anonymous=False)
    try:
        SemanticMappingNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("semantic_mapping interrupted.")
    except Exception as exc:
        rospy.logfatal("semantic_mapping failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
