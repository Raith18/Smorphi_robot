#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
player_utils.py — replay pacing and shared ROS message builders.

Kept separate from kitti_parsers.py so parsers stay pure-Python (testable
without ROS) while everything that touches rospy/sensor_msgs lives here and is
shared by the driver nodes and the offline bag converter (no duplicated code).
"""

from typing import Optional, Sequence

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import (CameraInfo, CompressedImage, Image, Imu,
                             NavSatFix, NavSatStatus, PointCloud2, PointField)
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from dataset_loader.kitti_parsers import KittiCalib, OxtsSample, matrix_to_quaternion

# --------------------------------------------------------------------------- #
# Time handling
# --------------------------------------------------------------------------- #
def ns_to_ros_time(ns: int) -> rospy.Time:
    """Integer nanoseconds -> rospy.Time (lossless; keeps exact sync working)."""
    return rospy.Time(secs=int(ns // 1_000_000_000), nsecs=int(ns % 1_000_000_000))


def make_header(frame_id: str, ns: int) -> Header:
    """Standard header with our integer-nanosecond stamp."""
    header = Header()
    header.frame_id = frame_id
    header.stamp = ns_to_ros_time(ns)
    return header


# --------------------------------------------------------------------------- #
# Message builders (shared by driver nodes AND kitti_to_bag.py)
# --------------------------------------------------------------------------- #
def build_image_msg(header: Header, jpg_bytes: bytes, encoding: str = "bgr8") -> Image:
    """
    Decode a JPEG file into a sensor_msgs/Image.

    Requires OpenCV (cv2). KITTI stores JPEG; most consumers need raw pixels.
    """
    import cv2  # imported lazily: only needed when decoding images

    image = cv2.imdecode(np.frombuffer(jpg_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Could not decode JPEG data ({} bytes)".format(len(jpg_bytes)))
    msg = Image()
    msg.header = header
    msg.height, msg.width = image.shape[:2]
    msg.encoding = encoding  # cv2 IMREAD_COLOR -> BGR8
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = image.tobytes()
    return msg


def build_compressed_image_msg(header: Header, jpg_bytes: bytes) -> CompressedImage:
    """Wrap raw JPEG bytes into a sensor_msgs/CompressedImage (no decoding)."""
    msg = CompressedImage()
    msg.header = header
    msg.format = "jpeg"
    msg.data = jpg_bytes
    return msg


def build_point_cloud2_msg(header: Header, bin_bytes: bytes,
                           point_step: int = 16) -> PointCloud2:
    """
    Build a PointCloud2 from a raw KITTI velodyne buffer.

    The .bin layout (x, y, z, reflectance — four float32) is *already* a
    PointCloud2 payload, so we reference the bytes directly without copying.
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(bin_bytes) // point_step
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * msg.width
    msg.data = bin_bytes
    msg.is_dense = True
    return msg


def build_point_cloud2_from_arrays(header: Header,
                                   arrays: "dict") -> PointCloud2:
    """
    Build a PointCloud2 from named float32/int32 arrays (same length).

    `arrays` preserves field order, e.g.:
        {"x": x, "y": y, "z": z, "intensity": i, "cluster_id": ids}
    Field datatypes are inferred from the array dtype.
    """
    names = list(arrays.keys())
    if not names:
        raise ValueError("arrays must contain at least one field")
    n = None
    chunks = []
    offset = 0
    fields = []
    for name in names:
        arr = np.ascontiguousarray(arrays[name])
        if n is None:
            n = arr.shape[0]
        elif arr.shape[0] != n:
            raise ValueError("field '{}' has {} points, expected {}".format(
                name, arr.shape[0], n))
        if arr.dtype == np.float32:
            datatype = PointField.FLOAT32
        elif arr.dtype == np.int32:
            datatype = PointField.INT32
        elif arr.dtype == np.uint32:
            datatype = PointField.UINT32
        elif arr.dtype == np.uint8:
            datatype = PointField.UINT8
        else:
            raise ValueError("unsupported dtype {} for field '{}'".format(arr.dtype, name))
        fields.append(PointField(name=name, offset=offset, datatype=datatype, count=1))
        chunks.append(arr.view(np.uint8).reshape(n, -1))
        offset += arr.dtype.itemsize

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = offset
    msg.row_step = offset * n
    msg.data = np.hstack(chunks).tobytes() if chunks else b""
    msg.is_dense = True
    return msg


def pointcloud2_to_arrays(msg: PointCloud2, names=None) -> "dict":
    """
    Extract named fields from a PointCloud2 into numpy float32 arrays.
    Uses a single structured-dtype view (fast, no per-point copies).
    """
    point_step = msg.point_step
    total = msg.height * msg.width
    fields = {f.name: f for f in msg.fields}
    if names is None:
        names = [f.name for f in msg.fields]
    formats = []
    offsets = []
    itemsize = point_step if point_step else sum(fields[n].offset + 4 for n in names)
    for name in names:
        field = fields[name]
        if field.datatype == PointField.FLOAT32:
            fmt = "<f4"
        elif field.datatype in (PointField.INT32, PointField.UINT32):
            fmt = "<i4"
        elif field.datatype in (PointField.UINT8, PointField.INT8):
            fmt = "<u1"
        elif field.datatype in (PointField.UINT16, PointField.INT16):
            fmt = "<u2"
        else:
            raise ValueError("unsupported PointField datatype {}".format(field.datatype))
        formats.append(fmt)
        offsets.append(field.offset)
    dtype = np.dtype({"names": list(names), "formats": formats,
                      "offsets": offsets, "itemsize": itemsize})
    structured = np.frombuffer(msg.data, dtype=dtype, count=total)
    return {name: structured[name].astype(np.float32) for name in names}


def build_imu_msg(header: Header, sample: OxtsSample) -> Imu:
    """
    OXTS sample -> sensor_msgs/Imu.

    Conventions (documented):
      * orientation: fused roll/pitch/yaw -> quaternion (KITTI quality is good).
      * angular_velocity: (wx, wy, wz) rad/s in the vehicle/IMU frame.
      * linear_acceleration: (ax, ay, az) m/s^2 in the vehicle/IMU frame
        (OXTS reports accelerometer readings excluding gravity compensation).
      * covariances: engineering estimates; fine-tune per sensor in YAML later.
    """
    msg = Imu()
    msg.header = header
    q = sample.quaternion_xyzw
    msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    msg.orientation_covariance = [1e-4] * 9            # ~0.01 rad std-dev
    msg.angular_velocity = Vector3(x=sample.wx, y=sample.wy, z=sample.wz)
    msg.angular_velocity_covariance = [1e-4] * 9
    msg.linear_acceleration = Vector3(x=sample.ax, y=sample.ay, z=sample.az)
    msg.linear_acceleration_covariance = [1e-4] * 9
    return msg


def build_navsatfix_msg(header: Header, sample: OxtsSample) -> NavSatFix:
    """OXTS sample -> sensor_msgs/NavSatFix (GPS position + fix status)."""
    msg = NavSatFix()
    msg.header = header
    msg.status.status = NavSatStatus.STATUS_FIX if sample.has_fix else NavSatStatus.STATUS_NO_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = sample.lat
    msg.longitude = sample.lon
    msg.altitude = sample.alt
    acc = max(sample.pos_accuracy, 1e-6)               # [m]
    msg.position_covariance = [acc ** 2, 0.0, 0.0,
                               0.0, acc ** 2, 0.0,
                               0.0, 0.0, (acc * 4.0) ** 2]   # z much less accurate
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
    return msg


def build_camera_info_msg(frame_id: str, calib: KittiCalib, cam_index: int,
                          stamp: Optional[rospy.Time] = None) -> CameraInfo:
    """
    KITTI calibration -> sensor_msgs/CameraInfo for one camera.

    KITTI images are already rectified, so R = R_rect_00 and P = P_rect_0X
    (the rectified projection matrices), matching how the images are produced.
    """
    width, height = calib.image_size(cam_index)
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp if stamp is not None else rospy.Time(0)
    msg.width = width
    msg.height = height
    msg.distortion_model = "plumb_bob"
    msg.D = list(calib.distortion(cam_index))                       # 5 coeffs
    msg.K = list(calib.camera_intrinsics(cam_index).flatten())      # 3x3
    msg.R = list(calib.rect_matrix().flatten())                     # 3x3
    msg.P = list(calib.projection_matrix(cam_index).flatten())      # 3x4
    return msg


def build_transform_stamped(parent: str, child: str, translation: Sequence[float],
                            rotation: Sequence[float],
                            stamp: Optional[rospy.Time] = None) -> TransformStamped:
    """Build a TransformStamped from translation + [x, y, z, w] quaternion."""
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = stamp if stamp is not None else rospy.Time(0)
    t.child_frame_id = child
    t.transform.translation = Vector3(x=translation[0], y=translation[1], z=translation[2])
    t.transform.rotation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
    return t


def build_tf_message(transforms: Sequence[TransformStamped]) -> TFMessage:
    """Wrap one or more TransformStamped into a TFMessage (for rosbag /tf_static)."""
    msg = TFMessage()
    msg.transforms = list(transforms)
    return msg


def transform_translation_quat(matrix4: np.ndarray):
    """4x4 homogeneous matrix -> (translation (3,), quaternion [x,y,z,w])."""
    translation = matrix4[:3, 3]
    quat = matrix4_rotation_to_quat(matrix4)
    return translation, quat


def matrix4_rotation_to_quat(matrix4: np.ndarray):
    """Extract the 3x3 rotation from a 4x4 matrix and convert to quaternion."""
    return matrix_to_quaternion(np.asarray(matrix4, dtype=float)[:3, :3])
