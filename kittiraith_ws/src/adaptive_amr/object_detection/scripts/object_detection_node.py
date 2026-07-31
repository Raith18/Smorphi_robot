#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
object_detection_node.py — YOLOv8 + LiDAR frustum fusion node.

Pipeline:
    /camera/image_rect  ──┐
    /camera/camera_info ──┤--> YOLOv8 (yolov8n, CPU) -> 2D boxes
    /velodyne_points    ──┘--> project into image (sensor_fusion.projection)
                                 -> median depth per bbox (frustum fusion)
                                 -> 3D center in the laser frame

Publishers:
    /object_detections        (adaptive_amr_msgs/ObjectDetectionArray)
    /object_detections/image  (sensor_msgs/Image, annotated)
    /object_detections/statistics (DiagnosticArray)

Model notes:
    * Model: yolov8n.pt (auto-downloaded by ultralytics on first run;
      ~6 MB). Override with `model` param (path or name).
    * CPU by default (`device: cpu`); auto-selects CUDA only if
      torch.cuda.is_available() and device=auto.

Run:
    rosrun object_detection object_detection_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import ApproximateTimeSynchronizer, Subscriber, TimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from adaptive_amr_msgs.msg import ObjectDetection, ObjectDetectionArray

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False


class ObjectDetectionNode:
    """Runs YOLOv8 detection with LiDAR-fused depth on each frame."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.image_topic = rospy.get_param("~image_topic", "/camera/image_rect")
        self.points_topic = rospy.get_param("~points_topic", "/velodyne_points")
        self.camera_info_topic = rospy.get_param("~camera_info_topic",
                                                 "/camera/camera_info_rect")
        self.sync_type = rospy.get_param("~sync_type", "exact")
        self.queue_size = rospy.get_param("~queue_size", 20)
        self.approx_slop = rospy.get_param("~approx_slop", 0.05)

        self.model_name = rospy.get_param("~model", "yolov8n.pt")
        self.conf = rospy.get_param("~conf_threshold", 0.35)
        self.imgsz = rospy.get_param("~imgsz", 640)
        self.device = rospy.get_param("~device", "cpu")      # cpu | auto | cuda:0
        self.max_det = rospy.get_param("~max_det", 30)
        self.target_classes = rospy.get_param("~target_classes",
                                              ["person", "bicycle", "car",
                                               "motorcycle", "bus", "truck",
                                               "traffic light", "stop sign"])
        self.publish_image = rospy.get_param("~publish_image", True)
        self.ns = rospy.get_param("~output_ns", "/object_detections")
        self.max_projection_distance = rospy.get_param("~max_projection_distance", 120.0)
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        # ---- model (lazy import: ultralytics may not be installed) ---------------
        self.model = self._load_model()

        # ---- state ------------------------------------------------------------------
        self.latest_camera_info = None
        self.projection = None          # from sensor_fusion (shared projection math)
        self.frames_processed = 0
        self.last_ms = 0.0
        self.last_detections = 0
        # ---- ROS wiring ---------------------------------------------------------------
        self.pub_detections = rospy.Publisher("/object_detections",
                                              ObjectDetectionArray, queue_size=5)
        self.pub_image = rospy.Publisher("/object_detections/image", Image, queue_size=2)
        self.pub_stats = rospy.Publisher("/object_detections/statistics",
                                         DiagnosticArray, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        self.sub_ci = rospy.Subscriber(self.camera_info_topic, CameraInfo,
                                       self._on_camera_info)
        self.sub_image = Subscriber(self.image_topic, Image)
        self.sub_points = Subscriber(self.points_topic, PointCloud2)
        if self.sync_type == "exact":
            self.sync = TimeSynchronizer([self.sub_image, self.sub_points], self.queue_size)
        elif self.sync_type == "approx":
            self.sync = ApproximateTimeSynchronizer(
                [self.sub_image, self.sub_points], self.queue_size, self.approx_slop)
        else:
            raise ValueError("sync_type must be 'exact' or 'approx'")
        self.sync.registerCallback(self._on_synced)

        rospy.loginfo("object_detection: %s + %s -> /object_detections (model=%s)",
                      self.image_topic, self.points_topic, self.model_name)

    # ------------------------------------------------------------------ #
    def _load_model(self):
        """Load the YOLOv8 model; gives a clear error when ultralytics is missing."""
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
        rospy.loginfo("object_detection: loading model %s on %s", self.model_name, self.device)
        model = YOLO(self.model_name)
        return model

    # ------------------------------------------------------------------ #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg

    # ------------------------------------------------------------------ #
    def _build_projection(self, camera_info, points_msg):
        """Build LidarCameraProjection from CameraInfo + TF/calib."""
        from sensor_fusion.projection import LidarCameraProjection
        P = np.asarray(camera_info.P, dtype=float).reshape(3, 4)
        R = np.asarray(camera_info.R, dtype=float).reshape(3, 3)
        t_velo_cam = None
        try:
            from dataset_loader.kitti_parsers import KittiCalib, KittiPaths
            from tf2_ros import Buffer, TransformListener
            buf = Buffer()
            TransformListener(buf)
            stamped = buf.lookup_transform(camera_info.header.frame_id, points_msg.header.frame_id,
                                           rospy.Time(0), timeout=rospy.Duration(1.0))
            t = stamped.transform
            t_velo_cam = np.eye(4)
            t_velo_cam[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
            t_velo_cam[:3, :3] = _quat_to_matrix(t.rotation)
        except Exception:  # noqa: BLE001 - fall back to dataset calib
            root = rospy.get_param("~dataset_root", "/data/kitti")
            date = rospy.get_param("~date", "2011_09_26")
            drive = rospy.get_param("~drive", "2011_09_26_drive_0005")
            try:
                paths = KittiPaths(root, date, drive)
                calib = KittiCalib(paths.calib_dir, date)
                t_velo_cam = calib.velo_to_cam_transform()
            except Exception as exc:  # noqa: BLE001
                rospy.logwarn_throttle(5.0, "object_detection: no calibration: %s", exc)
                return None
        return LidarCameraProjection(P, R, t_velo_cam, width=camera_info.width,
                                     height=camera_info.height)

    # ------------------------------------------------------------------ #
    def _on_synced(self, image: Image, points: PointCloud2) -> None:
        if self.latest_camera_info is None:
            rospy.logwarn_throttle(5.0, "object_detection: no CameraInfo yet")
            return
        start = rospy.Time.now()

        image_np = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        # ---- YOLOv8 inference ----------------------------------------------------
        results = self.model.predict(image_np, conf=self.conf, imgsz=self.imgsz,
                                     device=self.device, max_det=self.max_det,
                                     verbose=False)
        result = results[0]

        # ---- fuse LiDAR depth ------------------------------------------------------
        proj = self._build_projection(self.latest_camera_info, points)
        u = v = depth = valid = None
        if proj is not None:
            from dataset_loader.player_utils import pointcloud2_to_arrays
            arrays = pointcloud2_to_arrays(points)
            xyz = np.column_stack([arrays["x"], arrays["y"], arrays["z"]])
            u, v, depth, valid = proj.project(xyz)

        detections = ObjectDetectionArray()
        detections.header = image.header
        if result.boxes is not None and len(result.boxes) > 0:
            for box in result.boxes:
                x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                name = self.model.names[cls_id]
                if self.target_classes and name not in self.target_classes:
                    continue

                det = ObjectDetection()
                det.label = _normalize(name)
                det.score = score
                det.bbox = [x1, y1, x2, y2]
                det.depth = 0.0
                det.position = [0.0, 0.0, 0.0]
                det.dimensions = [0.0, 0.0, 0.0]

                if proj is not None:
                    from object_detection.detection_utils import (
                        backproject_to_laser, box_center, median_depth_in_box,
                        points_in_box)
                    in_box = points_in_box(u, v, (x1, y1, x2, y2))
                    depth_value = median_depth_in_box(depth, in_box & valid,
                                                      self.max_projection_distance)
                    if depth_value is not None:
                        det.depth = depth_value
                        cu, cv = box_center([x1, y1, x2, y2])
                        K = np.asarray(self.latest_camera_info.K, dtype=float).reshape(3, 3)
                        pos = backproject_to_laser(cu, cv, depth_value, K, proj.T_velo_cam)
                        det.position = [float(pos[0]), float(pos[1]), float(pos[2])]

                detections.detections.append(det)

        self.pub_detections.publish(detections)
        self.last_detections = len(detections.detections)

        # ---- annotated image ---------------------------------------------------------
        if self.publish_image and HAVE_CV2:
            annotated = image_np.copy()
            for det in detections.detections:
                x1, y1, x2, y2 = [int(v) for v in det.bbox]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated,
                            "{} {:.2f} {:.1f}m".format(det.label, det.score, det.depth),
                            (x1, max(y1 - 8, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1)
            self.pub_image.publish(_to_image_msg(image.header, annotated))

        self.frames_processed += 1
        self.last_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.frames_processed > 0 else DiagnosticStatus.WARN
        status.name = "object_detection"
        status.message = "{} frames processed".format(self.frames_processed)
        status.values = [
            KeyValue(key="frames_processed", value=str(self.frames_processed)),
            KeyValue(key="last_inference_ms", value="{:.1f}".format(self.last_ms)),
            KeyValue(key="last_detections", value=str(self.last_detections)),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    def run(self) -> None:
        rospy.spin()


def _normalize(name: str) -> str:
    """COCO name -> KITTI-style label."""
    try:
        from object_detection.detection_utils import normalize_label
        return normalize_label(name)
    except Exception:  # noqa: BLE001
        return name


def _to_image_msg(header, np_image):
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header = header
    msg.height, msg.width = np_image.shape[:2]
    msg.encoding = "bgr8"
    msg.step = np_image.shape[1] * 3
    msg.data = np_image.tobytes()
    return msg


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rospy.init_node("object_detection_node", anonymous=False)
    try:
        ObjectDetectionNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("object_detection interrupted.")
    except Exception as exc:
        rospy.logfatal("object_detection failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
