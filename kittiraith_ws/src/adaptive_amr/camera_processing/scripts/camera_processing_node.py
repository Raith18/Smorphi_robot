#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_processing_node.py — camera pipeline node.

Subscribes to the raw camera stream + its CameraInfo and publishes a
processed image on /camera/image_rect:

    /camera/image_raw  +  /camera/camera_info
        -> [undistort/rectify] -> [color conversion] -> [crop] -> [resize]
    /camera/image_rect          (sensor_msgs/Image)
    /camera/camera_info_rect    (adjusted CameraInfo, latched)

Rectification modes:
  * none               — KITTI _sync images are already rectified (default);
                         performs crop/resize only.
  * undistort_rectify  — full undistortion+rectification via
                         cv2.initUndistortRectifyMap (or the pure-NumPy
                         reference implementation when OpenCV is missing).

Run:
    rosrun camera_processing camera_processing_node.py
"""

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

from camera_processing.rectify import RectifyMapper, adjust_intrinsics, adjust_projection_matrix


def _np_from_camera_info(msg: CameraInfo) -> tuple:
    """Extract numpy K, D, R, P from a CameraInfo message."""
    K = np.asarray(msg.K, dtype=float).reshape(3, 3)
    D = np.asarray(msg.D, dtype=float)
    R = np.asarray(msg.R, dtype=float).reshape(3, 3)
    P = np.asarray(msg.P, dtype=float).reshape(3, 4)
    return K, D, R, P


class CameraProcessingNode:
    """Processes the camera stream and keeps CameraInfo consistent."""

    def __init__(self):
        # ---- configuration ----------------------------------------------------
        self.input_topic = rospy.get_param("~input_topic", "/camera/image_raw")
        self.output_topic = rospy.get_param("~output_topic", "/camera/image_rect")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")
        self.output_ci_topic = rospy.get_param("~output_ci_topic", "/camera/camera_info_rect")
        self.rectify_mode = rospy.get_param("~rectify_mode", "none")
        self.resize_scale = rospy.get_param("~resize_scale", 1.0)
        crop = rospy.get_param("~crop", [0, 0, 0, 0])
        self.crop = tuple(int(v) for v in crop[:4])
        self.output_encoding = rospy.get_param("~output_encoding", "bgr8")
        self.adjust_camera_info = rospy.get_param("~adjust_camera_info", True)
        self.frame_id_override = rospy.get_param("~frame_id_override", "")
        self.diagnostics_topic = rospy.get_param("~diagnostics_topic",
                                                 "/camera_processing/statistics")
        self.diagnostics_rate = rospy.get_param("~diagnostics_rate", 1.0)

        if self.rectify_mode not in ("none", "undistort_rectify"):
            raise ValueError("rectify_mode must be 'none' or 'undistort_rectify'")

        # ---- state --------------------------------------------------------------
        self.mapper = None            # RectifyMapper, rebuilt when CameraInfo changes
        self.latest_camera_info = None
        self.processed_frames = 0
        self.skipped_frames = 0
        self.last_process_ms = 0.0

        # ---- ROS wiring -----------------------------------------------------------
        self.sub_image = rospy.Subscriber(self.input_topic, Image, self._on_image)
        self.sub_ci = rospy.Subscriber(self.camera_info_topic, CameraInfo,
                                       self._on_camera_info)
        self.pub_image = rospy.Publisher(self.output_topic, Image, queue_size=5)
        self.pub_ci = rospy.Publisher(self.output_ci_topic, CameraInfo,
                                      queue_size=1, latch=True)
        self.pub_stats = rospy.Publisher(self.diagnostics_topic, DiagnosticArray,
                                         queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.diagnostics_rate, 0.1)),
                    self._publish_statistics)

        rospy.loginfo("camera_processing: %s -> %s (mode=%s, scale=%s, crop=%s)",
                      self.input_topic, self.output_topic, self.rectify_mode,
                      self.resize_scale, self.crop)

    # ------------------------------------------------------------------ #
    def _on_camera_info(self, msg: CameraInfo) -> None:
        """(Re)build the RectifyMapper whenever CameraInfo arrives/changes."""
        self.latest_camera_info = msg
        K, D, R, P = _np_from_camera_info(msg)
        size = (msg.width, msg.height)
        try:
            self.mapper = RectifyMapper(K, D, R, P, size,
                                        mode=self.rectify_mode,
                                        crop=self.crop,
                                        scale=self.resize_scale,
                                        use_cv2=HAVE_CV2)
        except (ValueError, ZeroDivisionError) as exc:
            rospy.logerr("camera_processing: could not build mapper: %s", exc)
            self.mapper = None
            return
        rospy.loginfo("camera_processing: mapper rebuilt (output %dx%d)",
                      self.mapper.output_size[0], self.mapper.output_size[1])
        if self.adjust_camera_info:
            self._publish_adjusted_camera_info()

    # ------------------------------------------------------------------ #
    def _publish_adjusted_camera_info(self) -> None:
        """Publish CameraInfo consistent with crop/resize (latched)."""
        if self.latest_camera_info is None or self.mapper is None:
            return
        src = self.latest_camera_info
        out = CameraInfo()
        out.header = src.header
        out.width, out.height = self.mapper.output_size
        out.distortion_model = src.distortion_model
        out.D = list(src.D)
        K, D, R, P = _np_from_camera_info(src)
        out.K = list(adjust_intrinsics(K, self.crop, self.resize_scale).flatten())
        out.R = list(R.flatten())
        out.P = list(self.mapper.output_P.flatten())
        self.pub_ci.publish(out)
        rospy.loginfo("camera_processing: camera_info adjusted -> %s",
                      self.output_ci_topic)

    # ------------------------------------------------------------------ #
    def _on_image(self, msg: Image) -> None:
        """Process one raw image and publish the rectified/processed one."""
        if self.mapper is None:
            self.skipped_frames += 1
            rospy.logwarn_throttle(5.0, "camera_processing: no CameraInfo yet "
                                        "(%d frames skipped)", self.skipped_frames)
            return
        if msg.encoding not in ("bgr8", "rgb8", "mono8", "8UC1"):
            rospy.logwarn_throttle(5.0, "camera_processing: unexpected encoding %s",
                                   msg.encoding)
        start = rospy.Time.now()
        try:
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1) if msg.encoding not in ("mono8", "8UC1") \
                else np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            processed = self.mapper.apply(image)
        except ValueError as exc:
            rospy.logerr_throttle(5.0, "camera_processing: %s", exc)
            self.skipped_frames += 1
            return

        # ---- color conversion ---------------------------------------------------
        if HAVE_CV2 and self.output_encoding in ("rgb8", "mono8"):
            if self.output_encoding == "rgb8":
                processed = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
            else:
                processed = cv2.cvtColor(processed, cv2.COLOR_BGR2GRAY)

        # ---- publish ---------------------------------------------------------------
        out = Image()
        out.header = msg.header
        if self.frame_id_override:
            out.header.frame_id = self.frame_id_override
        out.height, out.width = processed.shape[:2]
        out.encoding = self.output_encoding
        out.is_bigendian = False
        out.step = processed.shape[1] * (1 if processed.ndim == 2 else processed.shape[2])
        out.data = processed.tobytes()
        self.pub_image.publish(out)

        self.processed_frames += 1
        self.last_process_ms = (rospy.Time.now() - start).to_sec() * 1000.0

    # ------------------------------------------------------------------ #
    def _publish_statistics(self, _event=None) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK if self.mapper is not None else DiagnosticStatus.WARN
        status.name = "camera_processing"
        status.message = "{} processed frames".format(self.processed_frames)
        status.values = [
            KeyValue(key="processed_frames", value=str(self.processed_frames)),
            KeyValue(key="skipped_frames", value=str(self.skipped_frames)),
            KeyValue(key="last_process_ms", value="{:.2f}".format(self.last_process_ms)),
            KeyValue(key="output_size", value="{}x{}".format(
                self.mapper.output_size[0], self.mapper.output_size[1])
                if self.mapper else "n/a"),
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status.append(status)
        self.pub_stats.publish(array)

    # ------------------------------------------------------------------ #
    def run(self) -> None:
        rospy.spin()


def main():
    rospy.init_node("camera_processing_node", anonymous=False)
    try:
        CameraProcessingNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("camera_processing interrupted.")
    except Exception as exc:
        rospy.logfatal("camera_processing failed: %s", exc)
        raise


if __name__ == "__main__":
    main()
