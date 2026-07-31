#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sensor_fusion_node.py — camera-LiDAR fusion node.

Synchronizes the rectified camera image with the Velodyne scan (exact or
approximate), looks up (or loads) the velodyne->camera transform, then
produces three fused outputs:

    /fusion/colored_points   (sensor_msgs/PointCloud2, rgb field)  LiDAR painted
                                                                   with camera color
    /fusion/sparse_depth     (sensor_msgs/Image, 32FC1)            LiDAR depth per
                                                                   projected pixel
    /fusion/overlay          (sensor_msgs/Image, bgr8)             camera image with
                                                                   projected LiDAR

Inputs (configurable):
    /camera/image_rect  +  /camera/camera_info_rect (or /camera/camera_info)
    /velodyne_points    (or /lidar_processing/obstacles)
    TF: laser -> camera_optical_frame   (fallback: KITTI calib files)

Run:
    rosrun sensor_fusion sensor_fusion_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener

from dataset_loader.kitti_parsers import KittiCalib, KittiPaths
from dataset_loader.player_utils import (build_point_cloud2_from_arrays,
                                         pointcloud2_to_arrays)
from sensor_fusion.projection import LidarCameraProjection

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

NANOSECONDS_PER_SECOND = 1_000_000_000


def _pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """(N,3) uint8 RGB -> float32 view of packed uint32 (RViz RGB8)."""
    packed = (rgb[:, 0].astype(np.uint32) << 16 |
              rgb[:, 1].astype(np.uint32) << 8 |
              rgb[:, 2].astype(np.uint32))
    return packed.view(np.float32)


class SensorFusionNode:
    """Projects LiDAR into the camera and publishes fused outputs."""

    def __init__(self):
        # ---- configuration -----------------------------------------------------
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_rect")
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.camera_info_topic = rospy.get_param("~camera_info_topic",
                                                 "/camera/camera_info")
        self.sync_type = rospy.get_param("~sync_type", "exact")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.05)
        self.laser_frame = rospy.get_param("~laser_frame", "laser")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_optical_frame")
        self.ns = rospy.get_param("~output_ns", "/fusion")
        self.use_tf = rospy.get_param("~use_tf", True)
        self.load_calibration_from_dataset = rospy.get_param(
            "~load_calibration_from_dataset", True)
        self.dataset_root = rospy.get_param("~dataset_root", "")
        self.dataset_date = rospy.get_param("~date", "2011_09_26")
        self.dataset_drive = rospy.get_param("~drive", "2011_09_26_drive_0005")
        self.publish_overlay = rospy.get_param("~publish_overlay", True)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)
        self.max_projection_distance = rospy.get_param("~max_projection_distance", 120.0)

        # ---- state ---------------------------------------------------------------
        self.latest_camera_info = None
        self.projection = None
        self.tf_buffer = None
        self.fused_sets = 0
        self.projected_points = 0
        self.last_ms = 0.0

        # ---- tf (for laser -> camera_optical_frame) ------------------------------
        if self.use_tf:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer)

        # ---- publishers ------------------------------------------------------------
        self.pub_colored = rospy.Publisher("{}/colored_points".format(self.ns),
                                           PointCloud2, queue_size=2)
        self.pub_depth = rospy.Publisher("{}/sparse_depth".format(self.ns),
                                         Image, queue_size=2)
        self.pub_overlay = rospy.Publisher("{}/overlay".format(self.ns),
                                           Image, queue_size=2)
        self.pub_stats = rospy.Publisher("{}/statistics".format(self.ns),
                                         DiagnosticArray, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        # ---- synchronized inputs ---------------------------------------------------
        self.sub_image = Subscriber(self.image_topic, Image)
        self.sub_points = Subscriber(self.points_topic, PointCloud2)
        if self.sync_type == "exact":
            self.sync = TimeSynchronizer([self.sub_image, self.sub_points],
                                         self.queue_size)
        elif self.sync_type == "approx":
            self.sync = ApproximateTimeSynchronizer(
                [self.sub_image, self.sub_points], self.queue_size, self.approx_slop)
        else:
            raise ValueError("sync_type must be 'exact' or 'approx'")
        self.sync.registerCallback(self._on_synced)

        self.sub_ci = rospy.Subscriber(self.camera_info_topic, CameraInfo,
                                       self._on_camera_info)
        rospy.loginfo("sensor_fusion: %s + %s -> %s (sync=%s)",
                      self.image_topic, self.points_topic, self.ns, self.sync_type)

    # ------------------------------------------------------------------ #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache CameraInfo and (re)build the projection on first arrival."""
        self.latest_camera_info = msg
        if self.projection is None:
            self._rebuild_projection()

    # ------------------------------------------------------------------ #
    def _rebuild_projection(self) -> None:
        """Build LidarCameraProjection from CameraInfo + TF or KITTI calib."""
        if self.latest_camera_info is None:
            return
        ci = self.latest_camera_info
        P = np.asarray(ci.P, dtype=float).reshape(3, 4)
        R = np.asarray(ci.R, dtype=float).reshape(3, 3)

        t_velo_cam = None
        if self.use_tf and self.tf_buffer is not None:
            try:
                stamped = self.tf_buffer.lookup_transform(
                    self.camera_frame, self.laser_frame, rospy.Time(0),
                    timeout=rospy.Duration(2.0))
                t = stamped.transform
                t_velo_cam = np.eye(4)
                t_velo_cam[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
                t_velo_cam[:3, :3] = _quat_to_matrix(t.rotation)
                rospy.loginfo("sensor_fusion: using TF transform %s -> %s",
                              self.laser_frame, self.camera_frame)
            except Exception as exc:  # noqa: BLE001 - tf can raise several types
                rospy.logwarn_throttle(5.0, "sensor_fusion: TF lookup failed: %s", exc)

        if t_velo_cam is None and self.load_calibration_from_dataset:
            try:
                paths = KittiPaths(self.dataset_root, self.dataset_date,
                                   self.dataset_drive)
                calib = KittiCalib(paths.calib_dir, self.dataset_date)
                t_velo_cam = calib.velo_to_cam_transform()
                rospy.loginfo("sensor_fusion: using KITTI calib T_velo_cam0")
            except (FileNotFoundError, ValueError) as exc:
                rospy.logwarn_throttle(5.0,
                                       "sensor_fusion: no calibration available: %s", exc)

        if t_velo_cam is None:
            rospy.logerr("sensor_fusion: missing velodyne->camera transform "
                         "(enable use_tf or load_calibration_from_dataset)")
            return

        self.projection = LidarCameraProjection(
            P, R, t_velo_cam, width=ci.width, height=ci.height)
        rospy.loginfo("sensor_fusion: projection built (%dx%d)",
                      ci.width, ci.height)

    # ------------------------------------------------------------------ #
    def _on_synced(self, image: Image, points: PointCloud2) -> None:
        """Fuse one synchronized (image, scan) pair."""
        if self.projection is None:
            self._rebuild_projection()
        if self.projection is None:
            rospy.logwarn_throttle(5.0, "sensor_fusion: no projection yet")
            return

        start = rospy.Time.now()
        arrays = pointcloud2_to_arrays(points)
        xyz = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])

        u, v, depth, valid = self.projection.project(xyz)
        # only fuse points in front and within the projection range
        valid &= (depth > 0) & (depth < self.max_projection_distance)

        # ---- 1. colored point cloud -------------------------------------------
        image_np = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        rgb = self.projection.colorize(xyz, image_np, u, v, valid)
        colored = build_point_cloud2_from_arrays(
            points.header,
            {"x": arrays["x"], "y": arrays["y"], "z": arrays["z"],
             "intensity": arrays.get("intensity", np.zeros(xyz.shape[0], np.float32)),
             "rgb": _pack_rgb(rgb)})
        self.pub_colored.publish(colored)

        # ---- 2. sparse depth image ----------------------------------------------
        depth_image = self.projection.build_sparse_depth(u, v, depth, valid)
        depth_msg = Image()
        depth_msg.header = image.header
        depth_msg.height, depth_msg.width = depth_image.shape
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = depth_image.shape[1] * 4
        depth_msg.data = depth_image.astype("<f4").tobytes()
        self.pub_depth.publish(depth_msg)

        # ---- 3. overlay (camera + projected points, jet by depth) ---------------
        if self.publish_overlay:
            if HAVE_CV2:
                overlay = image_np.copy()
                inside = self.projection.in_image_mask(u, v) & valid
                uu = u[inside].astype(np.int32)
                vv = v[inside].astype(np.int32)
                dd = depth[inside]
                norm = cv2.normalize(dd, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                colors = cv2.applyColorMap(norm, cv2.COLORMAP_JET)[:, 0, :]  # BGR
                for ui, vi, color in zip(uu, vv, colors):
                    cv2.circle(overlay, (int(ui), int(vi)), 2, tuple(int(c) for c in color), -1)
                overlay_msg = Image()
                overlay_msg.header = image.header
                overlay_msg.height, overlay_msg.width = overlay.shape[:2]
                overlay_msg.encoding = "bgr8"
                overlay_msg.step = overlay.shape[1] * 3
                overlay_msg.data = overlay.tobytes()
                self.pub_overlay.publish(overlay_msg)
            else:
                rospy.logwarn_throttle(10.0, "sensor_fusion: cv2 missing, "
                                             "skipping overlay")

        self.fused_sets += 1
        self.projected_points += int(valid.sum())
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = (DiagnosticStatus.OK if self.fused_sets > 0
                        else DiagnosticStatus.WARN)
        status.name = "sensor_fusion"
        status.message = "{} fused sets".format(self.fused_sets)
        status.values = [
            KeyValue(key="fused_sets", value=str(self.fused_sets)),
            KeyValue(key="projected_points", value=str(self.projected_points)),
            KeyValue(key="last_fusion_ms", value="{:.2f}".format(self.last_ms)),
            KeyValue(key="projection_ready", value=str(self.projection is not None)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        rospy.spin()


def _quat_to_matrix(q) -> np.ndarray:
    """geometry_msgs/Quaternion -> 3x3 rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("sensor_fusion_node", anonymous=False)
    try:
        SensorFusionNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("sensor_fusion interrupted.")
    except Exception as exc:
        rospy.logfatal("sensor_fusion failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
