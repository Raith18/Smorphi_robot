#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_processing_node.py — LiDAR pipeline node.

Pipeline (each stage can be disabled via parameters):

    /velodyne_points (or /time_sync/points)
        -> PassthroughFilter (axis + range limits)
        -> VoxelGrid (downsample)
        -> GroundRemover (RANSAC plane)
        -> EuclideanClusterExtraction
        -> OrientedBoundingBox (PCA)
    publishes:
        /lidar_processing/obstacles   (PointCloud2, non-ground)
        /lidar_processing/ground      (PointCloud2, ground)
        /lidar_processing/clusters    (PointCloud2 + cluster_id field)
        /lidar_processing/boxes       (visualization_msgs/MarkerArray, 3D cubes)
        /lidar_processing/statistics  (DiagnosticArray @1 Hz)

Run:
    rosrun lidar_processing lidar_processing_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from dataset_loader.kitti_parsers import matrix_to_quaternion
from dataset_loader.player_utils import (build_point_cloud2_from_arrays,
                                         pointcloud2_to_arrays)
from lidar_processing.clustering import EuclideanClusterExtraction, OrientedBoundingBox
from lidar_processing.filters import PassthroughFilter, VoxelGrid
from lidar_processing.ground_segmentation import GroundRemover, RansacPlaneSegmenter


def _axis_limits(param_value) -> dict:
    """Parse {'x': [lo, hi], ...} rosparam into axis->(lo, hi)."""
    limits = {}
    for axis in ("x", "y", "z"):
        if axis in param_value:
            lo, hi = param_value[axis][:2]
            limits[axis] = (float(lo), float(hi))
    return limits


class LidarProcessingNode:
    """Runs the LiDAR processing pipeline on each incoming scan."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.ns = rospy.get_param("~output_ns", "/lidar_processing")

        self.enable_passthrough = rospy.get_param("~enable_passthrough", True)
        self.limits = _axis_limits(rospy.get_param("~limits", {"x": [-80, 80], "y": [-80, 80], "z": [-3, 8]}))
        self.min_range = rospy.get_param("~min_range", 0.5)
        self.max_range = rospy.get_param("~max_range", 120.0)

        self.enable_voxel = rospy.get_param("~enable_voxel", True)
        self.voxel_leaf = rospy.get_param("~voxel_leaf", 0.3)

        self.enable_ground_removal = rospy.get_param("~enable_ground_removal", True)
        self.ground_threshold = rospy.get_param("~ground_distance_threshold", 0.2)
        self.ground_iterations = rospy.get_param("~ground_max_iterations", 50)
        self.min_normal_z = rospy.get_param("~ground_min_normal_z", 0.6)

        self.enable_clustering = rospy.get_param("~enable_clustering", True)
        self.cluster_tolerance = rospy.get_param("~cluster_tolerance", 0.5)
        self.cluster_min = rospy.get_param("~cluster_min_size", 10)
        self.cluster_max = rospy.get_param("~cluster_max_size", 200_000)

        self.enable_boxes = rospy.get_param("~enable_boxes", True)
        self.box_min_dim = rospy.get_param("~box_min_dimension", 0.4)

        # ---- algorithm objects ------------------------------------------------
        self.passthrough = PassthroughFilter(self.limits, self.min_range, self.max_range)
        self.voxel = VoxelGrid(self.voxel_leaf)
        self.segmenter = RansacPlaneSegmenter(self.ground_threshold, self.ground_iterations)
        self.ground_remover = GroundRemover(self.segmenter, self.min_normal_z)
        self.clusterer = EuclideanClusterExtraction(self.cluster_tolerance,
                                                    self.cluster_min, self.cluster_max)

        # ---- publishers ----------------------------------------------------------
        self._publishers = {}
        self.pub_obstacles = rospy.Publisher("{}/obstacles".format(self.ns),
                                             PointCloud2, queue_size=2)
        self.pub_ground = rospy.Publisher("{}/ground".format(self.ns),
                                          PointCloud2, queue_size=2)
        self.pub_clusters = rospy.Publisher("{}/clusters".format(self.ns),
                                            PointCloud2, queue_size=2)
        self.pub_boxes = rospy.Publisher("{}/boxes".format(self.ns),
                                         MarkerArray, queue_size=2)
        self.pub_stats = rospy.Publisher("{}/statistics".format(self.ns),
                                         DiagnosticArray, queue_size=5)

        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        # ---- statistics ------------------------------------------------------------
        self.processed_scans = 0
        self.last_ms = 0.0
        self.last_counts = {}

        rospy.Subscriber(self.points_topic, PointCloud2, self._on_scan,
                         queue_size=2, buff_size=2 ** 24)
        rospy.loginfo("lidar_processing: %s -> %s", self.points_topic, self.ns)

    # ------------------------------------------------------------------ #
    def _publish_cloud(self, topic, header, arrays):
        self._publishers.setdefault(topic, rospy.Publisher(topic, PointCloud2, queue_size=2))
        self._publishers[topic].publish(build_point_cloud2_from_arrays(header, arrays))

    # ------------------------------------------------------------------ #
    def _on_scan(self, msg: PointCloud2) -> None:
        start = rospy.Time.now()
        arrays = pointcloud2_to_arrays(msg)
        points = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
        intensity = arrays.get("intensity", np.zeros(points.shape[0], dtype=np.float32))
        header = msg.header
        counts = {}

        # ---- 1. passthrough ------------------------------------------------------
        if self.enable_passthrough:
            mask = self._passthrough_mask(points)
            points = points[mask]
            intensity = intensity[mask]
        counts["after_passthrough"] = points.shape[0]
        if points.shape[0] == 0:
            self._emit_empty(header)
            return

        # ---- 2. voxel downsampling -------------------------------------------------
        if self.enable_voxel:
            points, intensity = self.voxel.downsample(points, intensity[:, None])
            intensity = intensity[:, 0]
        counts["after_voxel"] = points.shape[0]

        # ---- 3. ground removal -------------------------------------------------------
        ground = np.zeros((0, 3), dtype=np.float32)
        if self.enable_ground_removal:
            try:
                ground, obstacles, _ = self.ground_remover.separate(points)
            except ValueError as exc:
                rospy.logwarn_throttle(5.0, "lidar_processing: %s", exc)
                obstacles = points
        else:
            obstacles = points
        counts["ground"] = ground.shape[0]
        counts["obstacles"] = obstacles.shape[0]

        self._publish_cloud("{}/ground".format(self.ns), header,
                            {"x": ground[:, 0], "y": ground[:, 1], "z": ground[:, 2]})
        self._publish_cloud("{}/obstacles".format(self.ns), header,
                            {"x": obstacles[:, 0], "y": obstacles[:, 1],
                             "z": obstacles[:, 2], "intensity": intensity})

        # ---- 4. clustering + boxes ---------------------------------------------------
        markers = MarkerArray()
        if self.enable_clustering and obstacles.shape[0] >= self.cluster_min:
            clusters = self.clusterer.extract(obstacles)
            counts["clusters"] = len(clusters)
            cluster_ids = np.full(obstacles.shape[0], -1, dtype=np.int32)
            for cid, indices in enumerate(clusters):
                cluster_ids[indices] = cid
            self._publish_cloud("{}/clusters".format(self.ns), header,
                                {"x": obstacles[:, 0], "y": obstacles[:, 1],
                                 "z": obstacles[:, 2], "intensity": intensity,
                                 "cluster_id": cluster_ids})
            if self.enable_boxes:
                markers = self._build_box_markers(obstacles, clusters, header)
        else:
            counts["clusters"] = 0
        self.pub_boxes.publish(markers)

        self.processed_scans += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0
        self.last_counts = counts

    # ------------------------------------------------------------------ #
    def _passthrough_mask(self, points) -> np.ndarray:
        """Boolean mask of points passing the axis + range limits."""
        mask = np.ones(points.shape[0], dtype=bool)
        for axis, (lo, hi) in self.limits.items():
            idx = {"x": 0, "y": 1, "z": 2}[axis]
            mask &= (points[:, idx] >= lo) & (points[:, idx] <= hi)
        r = np.hypot(points[:, 0], points[:, 1])
        mask &= (r >= self.min_range) & (r <= self.max_range)
        return mask

    # ------------------------------------------------------------------ #
    def _build_box_markers(self, obstacles, clusters, header):
        """Build a MarkerArray of 3D cubes, one per cluster (PCA OBB)."""
        markers = MarkerArray()
        for cid, indices in enumerate(clusters):
            cluster_points = obstacles[indices]
            try:
                box = OrientedBoundingBox(cluster_points)
            except ValueError:
                continue
            if np.any(box.extents < self.box_min_dim):
                continue
            marker = Marker()
            marker.header = header
            marker.ns = "lidar_boxes"
            marker.id = cid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = Pose(
                position=Point(x=box.center[0], y=box.center[1], z=box.center[2]),
                orientation=Quaternion(*matrix_to_quaternion(box.axes)))
            marker.scale = Vector3(x=box.extents[0], y=box.extents[1], z=box.extents[2])
            marker.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.6)
            marker.lifetime = rospy.Duration(0.2)
            markers.markers.append(marker)
        return markers

    # ------------------------------------------------------------------ #
    def _emit_empty(self, header) -> None:
        empty = np.zeros((0,), dtype=np.float32)
        for topic in ("ground", "obstacles", "clusters"):
            self._publish_cloud("{}/{}".format(self.ns, topic), header,
                                {"x": empty, "y": empty, "z": empty})
        self.pub_boxes.publish(MarkerArray())

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.processed_scans > 0 else DiagnosticStatus.WARN
        status.name = "lidar_processing"
        status.message = "{} scans processed".format(self.processed_scans)
        status.values = [KeyValue(key="processed_scans", value=str(self.processed_scans)),
                         KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms))]
        for key, value in self.last_counts.items():
            status.values.append(KeyValue(key=key, value=str(value)))
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("lidar_processing_node", anonymous=False)
    try:
        LidarProcessingNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("lidar_processing interrupted.")
    except Exception as exc:
        rospy.logfatal("lidar_processing failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
