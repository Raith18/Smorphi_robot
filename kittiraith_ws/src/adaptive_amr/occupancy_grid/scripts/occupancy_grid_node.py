#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
occupancy_grid_node.py — 2D occupancy grid node.

Synchronizes the obstacle cloud (/lidar_processing/obstacles) with the pose
(/localization_pose, fallback /lidar_odometry) and ray-casts it into a
log-odds grid published on /occupancy_grid (nav_msgs/OccupancyGrid).

Run:
    rosrun occupancy_grid occupancy_grid_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2

from tf2_ros import Buffer, TransformListener

from dataset_loader.player_utils import pointcloud2_to_arrays
from occupancy_grid.grid_mapping import OccupancyGridMapper


def _pose_to_xy(pose) -> tuple:
    """geometry_msgs/Pose -> (x, y) in the grid frame."""
    return pose.position.x, pose.position.y


def _transform_matrix(t) -> np.ndarray:
    """geometry_msgs/Transform -> 4x4 matrix."""
    q = t.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
    return T


def _apply_transform(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 rigid transform to (N, >=3) points."""
    hom = np.hstack([points[:, :3], np.ones((points.shape[0], 1))])
    return (T @ hom.T).T[:, :3]


class OccupancyGridNode:
    """Builds and publishes the probabilistic 2D occupancy grid."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.obstacles_topic = rospy.get_param("~obstacles_topic",
                                               "/lidar_processing/obstacles")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization_pose")
        self.fallback_odom_topic = rospy.get_param("~fallback_odom_topic",
                                                   "/lidar_odometry")
        self.use_localization = rospy.get_param("~use_localization", True)
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.output_topic = rospy.get_param("~output_topic", "/occupancy_grid")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.1)

        self.mapper = OccupancyGridMapper(
            resolution=rospy.get_param("~resolution", 0.2),
            width_m=rospy.get_param("~width_m", 60.0),
            height_m=rospy.get_param("~height_m", 60.0),
            l_occ=rospy.get_param("~l_occ", 0.85),
            l_free=rospy.get_param("~l_free", 0.4),
            l_clamp=rospy.get_param("~l_clamp", 3.5))
        self.max_range = rospy.get_param("~max_range", 50.0)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        # TF: transform obstacle points (laser frame) into the map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # ---- ROS ------------------------------------------------------------------
        self.pub_grid = rospy.Publisher(self.output_topic, OccupancyGrid,
                                        queue_size=1, latch=True)
        self.pub_stats = rospy.Publisher(self.output_topic + "/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        self.sub_points = Subscriber(self.obstacles_topic, PointCloud2)
        if self.use_localization:
            self.sub_pose = Subscriber(self.pose_topic, PoseWithCovarianceStamped)
        else:
            self.sub_pose = Subscriber(self.fallback_odom_topic, Odometry)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_points, self.sub_pose], self.queue_size, self.approx_slop)
        self.sync.registerCallback(self._on_synced)

        self.frames_added = 0
        self.last_ms = 0.0
        rospy.loginfo("occupancy_grid: %s + %s -> %s (%dx%d, %.2f m/px)",
                      self.obstacles_topic, self.pose_topic, self.output_topic,
                      self.mapper.width, self.mapper.height, self.mapper.resolution)

    # ------------------------------------------------------------------ #
    def _on_synced(self, points: PointCloud2, pose_msg) -> None:
        start = rospy.Time.now()
        arrays = pointcloud2_to_arrays(points)
        xyz = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])

        # ---- frame discipline: obstacles are in the cloud's frame (laser),
        #      the grid lives in the map frame. Transform, falling back to
        #      identity (with a warning) when the TF is unavailable.
        try:
            stamped = self.tf_buffer.lookup_transform(
                self.frame_id, points.header.frame_id, points.header.stamp,
                timeout=rospy.Duration(0.5))
            xyz = _apply_transform(xyz, _transform_matrix(stamped.transform))
        except Exception:  # noqa: BLE001 - tf can raise several types
            rospy.logwarn_throttle(
                5.0, "occupancy_grid: TF %s -> %s unavailable, "
                     "assuming identity (grid may smear)",
                     points.header.frame_id, self.frame_id)

        robot_x, robot_y = _pose_to_xy(pose_msg.pose.pose)

        self.mapper.add_scan(xyz[:, :2], robot_x, robot_y,
                             max_range=self.max_range)
        self._publish_grid(points.header.stamp)

        self.frames_added += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_grid(self, stamp) -> None:
        occ = self.mapper.get_occupancy()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.info.resolution = self.mapper.resolution
        msg.info.width = self.mapper.width
        msg.info.height = self.mapper.height
        msg.info.origin.position.x = -self.mapper.half_w * self.mapper.resolution
        msg.info.origin.position.y = -self.mapper.half_h * self.mapper.resolution
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.ravel(order="C").tolist()
        self.pub_grid.publish(msg)

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        occ = self.mapper.get_occupancy()
        occupied = int((occ == 100).sum())
        free = int((occ == 0).sum())
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.frames_added > 0
                        else DiagnosticStatus.WARN)
        status.name = "occupancy_grid"
        status.message = "{} frames, {} occupied cells".format(
            self.frames_added, occupied)
        status.values = [
            KeyValue(key="frames_added", value=str(self.frames_added)),
            KeyValue(key="occupied_cells", value=str(occupied)),
            KeyValue(key="free_cells", value=str(free)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("occupancy_grid_node", anonymous=False)
    try:
        OccupancyGridNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("occupancy_grid interrupted.")
    except Exception as exc:
        rospy.logfatal("occupancy_grid failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
