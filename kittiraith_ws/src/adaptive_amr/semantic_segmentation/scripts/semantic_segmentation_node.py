#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
semantic_segmentation_node.py — YOLOv8-seg segmentation node.

Pipeline:
    /camera/image_rect + /velodyne_points
        -> YOLOv8-seg instance masks -> class label image
        -> label image + projection onto LiDAR -> colored point cloud

Publishers:
    /semantic_map              (sensor_msgs/Image, uint8 class labels)
    /semantic_map/colored      (sensor_msgs/Image, bgr8)
    /semantic_map/colored_points (sensor_msgs/PointCloud2, rgb + label)
    /semantic_map/statistics   (DiagnosticArray)

Model: yolov8n-seg.pt (auto-download on first run), CPU by default.

Run:
    rosrun semantic_segmentation semantic_segmentation_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from dataset_loader.player_utils import (build_point_cloud2_from_arrays,
                                         pointcloud2_to_arrays)
from semantic_segmentation.segmentation_utils import (build_label_image,
                                                      class_color, colorize_labels)


class SemanticSegmentationNode:
    """Runs YOLOv8-seg and publishes semantic labels + colored LiDAR."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_rect")
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.camera_info_topic = rospy.get_param("~camera_info_topic",
                                                 "/camera/camera_info_rect")
        self.sync_type = rospy.get_param("~sync_type", "exact")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.05)
        self.model_name = rospy.get_param("~model", "yolov8n-seg.pt")
        self.conf = rospy.get_param("~conf_threshold", 0.35)
        self.imgsz = rospy.get_param("~imgsz", 640)
        self.device = rospy.get_param("~device", "cpu")
        self.max_det = rospy.get_param("~max_det", 20)
        self.ns = rospy.get_param("~output_ns", "/semantic_map")
        self.publish_colored_points = rospy.get_param("~publish_colored_points", True)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        self.model = self._load_model()

        self.latest_camera_info = None
        self.projection = None
        self.frames_processed = 0
        self.last_ms = 0.0

        # label image lives under /semantic_map/labels; /semantic_map itself is
        # reserved for the GLOBAL semantic point cloud (Phase 6 semantic_mapping)
        self.pub_labels = rospy.Publisher(self.ns + "/labels", Image, queue_size=2)
        self.pub_colored = rospy.Publisher(self.ns + "/colored", Image, queue_size=2)
        self.pub_points = rospy.Publisher(self.ns + "/colored_points",
                                          PointCloud2, queue_size=2)
        self.pub_stats = rospy.Publisher(self.ns + "/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        self.sub_image = Subscriber(self.image_topic, Image)
        self.sub_points = Subscriber(self.points_topic, PointCloud2)
        if self.sync_type == "exact":
            self.sync = TimeSynchronizer([self.sub_image, self.sub_points], self.queue_size)
        else:
            self.sync = ApproximateTimeSynchronizer(
                [self.sub_image, self.sub_points], self.queue_size, self.approx_slop)
        self.sync.registerCallback(self._on_synced)

        self.sub_ci = rospy.Subscriber(self.camera_info_topic, CameraInfo,
                                       self._on_camera_info)

        rospy.loginfo("semantic_segmentation: %s + %s -> %s (model=%s)",
                      self.image_topic, self.points_topic, self.ns, self.model_name)

    # ------------------------------------------------------------------ #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """Cache CameraInfo and (re)build the projection for colored points."""
        self.latest_camera_info = msg
        self.projection = self._build_projection(msg)

    # ------------------------------------------------------------------ #
    def _build_projection(self, camera_info):
        """LidarCameraProjection from CameraInfo + TF (fallback: dataset calib)."""
        from sensor_fusion.projection import LidarCameraProjection
        P = np.asarray(camera_info.P, dtype=float).reshape(3, 4)
        R = np.asarray(camera_info.R, dtype=float).reshape(3, 3)
        t_velo_cam = None
        try:
            from tf2_ros import Buffer, TransformListener
            buf = Buffer()
            TransformListener(buf)
            stamped = buf.lookup_transform(camera_info.header.frame_id, "laser",
                                           rospy.Time(0), timeout=rospy.Duration(1.0))
            t = stamped.transform
            t_velo_cam = np.eye(4)
            t_velo_cam[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
            t_velo_cam[:3, :3] = _quat_to_matrix(t.rotation)
        except Exception:  # noqa: BLE001
            root = rospy.get_param("~dataset_root", "/data/kitti")
            date = rospy.get_param("~date", "2011_09_26")
            drive = rospy.get_param("~drive", "2011_09_26_drive_0005")
            try:
                from dataset_loader.kitti_parsers import KittiCalib, KittiPaths
                paths = KittiPaths(root, date, drive)
                calib = KittiCalib(paths.calib_dir, date)
                t_velo_cam = calib.velo_to_cam_transform()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0,
                                       "semantic_segmentation: no calibration: %s", exc)
                return None
        return LidarCameraProjection(P, R, t_velo_cam, width=camera_info.width,
                                     height=camera_info.height)

    # ------------------------------------------------------------------ #
    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            rospy.logfatal("ultralytics is not installed. Install Phase 4 deps:\n"
                           "  pip install -r docker/pip_requirements_phase4.txt")
            raise RuntimeError("ultralytics missing") from exc
        if self.device == "auto":
            try:
                import torch
                self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
            except ImportError:
                self.device = "cpu"
        rospy.loginfo("semantic_segmentation: loading model %s on %s",
                      self.model_name, self.device)
        return YOLO(self.model_name)

    # ------------------------------------------------------------------ #
    def _on_synced(self, image: Image, points: PointCloud2) -> None:
        start = rospy.Time.now()
        image_np = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        h, w = image_np.shape[:2]

        results = self.model.predict(image_np, conf=self.conf, imgsz=self.imgsz,
                                     device=self.device, max_det=self.max_det,
                                     verbose=False)
        result = results[0]

        names = self.model.names
        masks = []
        class_ids = []
        if result.masks is not None and len(result.masks) > 0:
            for mask, cls in zip(result.masks.data, result.boxes.cls):
                # resize mask from (640,640)-ish to image size
                import cv2
                mask_np = mask.cpu().numpy().astype(np.float32)
                mask_resized = cv2.resize(mask_np, (w, h),
                                          interpolation=cv2.INTER_NEAREST)
                masks.append(mask_resized > 0.5)
                class_ids.append(int(cls.item()))

        label = build_label_image(masks, class_ids, h, w)
        self.pub_labels.publish(self._to_image(image.header, label, "8UC1",
                                               label.strides[0]))
        colored = colorize_labels(label, names)
        self.pub_colored.publish(self._to_image(image.header, colored, "bgr8",
                                                colored.strides[0]))

        # ---- project labels onto LiDAR (PointPainting-style) --------------------
        if self.publish_colored_points:
            self._publish_colored_points(points, label, names)

        self.frames_processed += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_colored_points(self, points, label, names) -> None:
        """Color each LiDAR point by the semantic label of its pixel."""
        proj = self.projection
        if proj is None:
            self.projection = self._build_projection(self.latest_camera_info) \
                if self.latest_camera_info else None
            proj = self.projection
        if proj is None:
            rospy.logwarn_throttle(5.0, "semantic_segmentation: no projection "
                                        "available, skipping colored points")
            return

        arrays = pointcloud2_to_arrays(points)
        xyz = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
        u, v, depth, valid = proj.project(xyz)
        inside = proj.in_image_mask(u, v) & valid
        uu = u[inside].astype(np.int32)
        vv = v[inside].astype(np.int32)

        rgb = np.zeros((xyz.shape[0], 3), dtype=np.uint8)
        if inside.any():
            # label value -> class id -> color
            labels_at = label[vv, uu].astype(np.int32)
            colors = np.zeros((labels_at.shape[0], 3), dtype=np.uint8)
            for value in np.unique(labels_at):
                if value == 0:
                    continue
                cls_id = int(value) - 1
                if 0 <= cls_id < len(names):
                    colors[labels_at == value] = class_color(names[cls_id])
            rgb[inside] = colors

        packed = (rgb[:, 0].astype(np.uint32) << 16 |
                  rgb[:, 1].astype(np.uint32) << 8 |
                  rgb[:, 2].astype(np.uint32)).view(np.float32)
        cloud = build_point_cloud2_from_arrays(
            points.header,
            {"x": arrays["x"], "y": arrays["y"], "z": arrays["z"],
             "intensity": arrays.get("intensity", np.zeros(xyz.shape[0], np.float32)),
             "rgb": packed})
        self.pub_points.publish(cloud)

    # ------------------------------------------------------------------ #
    @staticmethod
    def _to_image(header, np_image, encoding, row_step):
        msg = Image()
        msg.header = header
        msg.height, msg.width = np_image.shape[:2]
        msg.encoding = encoding
        msg.step = row_step
        msg.data = np_image.tobytes()
        return msg

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.frames_processed > 0 else DiagnosticStatus.WARN
        status.name = "semantic_segmentation"
        status.message = "{} frames segmented".format(self.frames_processed)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="last_inference_ms", value="{:.1f}".format(self.last_ms)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("semantic_segmentation_node", anonymous=False)
    try:
        SemanticSegmentationNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("semantic_segmentation interrupted.")
    except Exception as exc:
        rospy.logfatal("semantic_segmentation failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
