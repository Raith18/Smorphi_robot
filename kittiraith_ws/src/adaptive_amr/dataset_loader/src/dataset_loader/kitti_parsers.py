#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
kitti_parsers.py — pure-Python parsers for the KITTI dataset.

This module is deliberately free of any ROS import so it can be unit-tested
on any host (including this sandbox) and reused by:
  * the ROS sensor driver nodes (camera_node, lidar_node, gps_node, imu_node)
  * the offline kitti_to_bag converter
  * later phases (calibration, sensor_fusion, odometry, mapping, evaluation)

Supported inputs (official KITTI RAW layout):
  <root>/raw/<date>/<drive>_sync/
      image_00..03/  velodyne/  oxts/  timestamps.txt
  <root>/raw/<date>/<date>_calib/
      calib_cam_to_cam.txt  calib_velo_to_cam.txt  calib_imu_to_velo.txt

All time handling uses *integer nanoseconds* — float64 cannot represent
epoch-scale nanoseconds exactly (~0.5 us resolution), which would silently
break exact time synchronization. See parse_kitti_timestamp_ns().
"""

import calendar
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# --------------------------------------------------------------------------- #
# Constants (WGS-84 ellipsoid, used for GPS -> ECEF -> local ENU)
# --------------------------------------------------------------------------- #
WGS84_A = 6378137.0                 # semi-major axis [m]
WGS84_E2 = 6.69437999014e-3         # first eccentricity squared
NAVSTAT_NO_FIX = 0                  # KITTI oxts navstat codes
NAVSTAT_FIX = 4


# --------------------------------------------------------------------------- #
# OXTS (GPS/IMU combined) — column indexes, from the KITTI devkit readme
# --------------------------------------------------------------------------- #
class OxtsIndex:
    """Column indexes into the 30-value OXTS data line (0-based)."""

    LAT = 0
    LON = 1
    ALT = 2
    ROLL = 3
    PITCH = 4
    YAW = 5
    VN = 6
    VE = 7
    VF = 8
    VL = 9
    VU = 10
    AX = 11
    AY = 12
    AZ = 13
    AF = 14
    AL = 15
    AU = 16
    WX = 17
    WY = 18
    WZ = 19
    WF = 20
    WL = 21
    WU = 22
    POS_ACC = 23
    VEL_ACC = 24
    NAVSTAT = 25
    NUMSATS = 26
    POSMODE = 27
    VELMODE = 28
    ORIMODE = 29


class OxtsSample:
    """
    One OXTS (GPS+IMU) sample: the 30 values KITTI logs per timestep.

    Units (KITTI devkit):
        lat/lon [deg], alt [m], roll/pitch/yaw [rad],
        velocities [m/s] (vn/ve north/east, vf/vl/vu vehicle-frame),
        accelerations [m/s^2] (ax/ay/az sensor frame, af/al/au vehicle frame),
        angular rates [rad/s] (wx/wy/wz sensor frame, wf/wl/wu vehicle frame),
        accuracies [m] / [m/s], navstat, numsats, posmode/velmode/orimode.
    """

    __slots__ = (
        "lat", "lon", "alt", "roll", "pitch", "yaw",
        "vn", "ve", "vf", "vl", "vu",
        "ax", "ay", "az", "af", "al", "au",
        "wx", "wy", "wz", "wf", "wl", "wu",
        "pos_accuracy", "vel_accuracy",
        "navstat", "numsats", "posmode", "velmode", "orimode",
    )

    def __init__(self, values: Sequence[float]):
        """Build an OxtsSample from a 30-element numeric sequence."""
        if len(values) != 30:
            raise ValueError(
                "OXTS line must contain 30 values, got {}: {}".format(len(values), values)
            )
        (
            self.lat, self.lon, self.alt,
            self.roll, self.pitch, self.yaw,
            self.vn, self.ve, self.vf, self.vl, self.vu,
            self.ax, self.ay, self.az, self.af, self.al, self.au,
            self.wx, self.wy, self.wz, self.wf, self.wl, self.wu,
            self.pos_accuracy, self.vel_accuracy,
            self.navstat, self.numsats, self.posmode, self.velmode, self.orimode,
        ) = values

    @property
    def has_fix(self) -> bool:
        """True when the GPS fix is usable (KITTI navstat != NO_FIX)."""
        return int(self.navstat) != NAVSTAT_NO_FIX

    @property
    def quaternion_xyzw(self) -> np.ndarray:
        """Fused orientation as [x, y, z, w] (vehicle->world rotation)."""
        return quaternion_from_euler(self.roll, self.pitch, self.yaw)

    @classmethod
    def from_line(cls, line: str) -> "OxtsSample":
        """Parse a whitespace-separated 30-value OXTS line."""
        values = [float(token) for token in line.split()]
        return cls(values)

    @classmethod
    def from_file(cls, path: str) -> "OxtsSample":
        """Parse the single OXTS line stored in `path`."""
        with open(path, "r") as handle:
            line = handle.read().strip()
        if not line:
            raise ValueError("Empty OXTS file: {}".format(path))
        return cls.from_line(line)


class OxtsParser:
    """Reads OXTS sample files (one 30-value line per file, 10 Hz)."""

    def __init__(self, oxts_dir: str):
        self.oxts_dir = oxts_dir

    def read_sample(self, index: int) -> Optional[OxtsSample]:
        """
        Read the sample at frame `index`, or None if the file is missing.
        Frame indexes are zero-padded to 10 digits, like KITTI filenames.
        """
        path = os.path.join(self.oxts_dir, "{:010d}.txt".format(index))
        if not os.path.isfile(path):
            return None
        return OxtsSample.from_file(path)


# --------------------------------------------------------------------------- #
# Timestamps — integer nanoseconds everywhere
# --------------------------------------------------------------------------- #
def parse_kitti_timestamp_ns(line: str) -> int:
    """
    Parse one KITTI timestamp line ("2011-09-26 13:02:10.123456789") into
    integer nanoseconds since the Unix epoch.

    Rationale: float64 at epoch scale (~1.3e9 s) has ~0.5 us resolution and
    would round different frames' nanoseconds identically; integer ns keeps
    exact per-frame stamps, which exact time synchronization depends on.
    """
    line = line.strip()
    date_part, _, frac_part = line.partition(".")
    dt = calendar.timegm(time.strptime(date_part, "%Y-%m-%d %H:%M:%S"))  # UTC epoch
    if not frac_part:
        return dt * 1_000_000_000
    frac_ns = int(frac_part[:9].ljust(9, "0"))
    return dt * 1_000_000_000 + frac_ns


def read_timestamps_ns(path: str) -> List[int]:
    """Read a KITTI timestamps.txt file into a list of int nanoseconds."""
    stamps: List[int] = []
    with open(path, "r") as handle:
        for lineno, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                stamps.append(parse_kitti_timestamp_ns(line))
            except ValueError as exc:
                raise ValueError("Invalid timestamp at {}:{}: {}".format(path, lineno, line)) from exc
    if not stamps:
        raise ValueError("No timestamps found in {}".format(path))
    return stamps


# --------------------------------------------------------------------------- #
# Velodyne point clouds
# --------------------------------------------------------------------------- #
def read_velodyne_bin(path: str) -> np.ndarray:
    """
    Read a Velodyne HDL-64E scan file (.bin) as an (N, 4) float32 array:
    columns are [x, y, z, reflectance] in the velodyne frame
    (x forward, y left, z up — KITTI/velodyne convention).
    """
    data = np.fromfile(path, dtype=np.float32)
    if data.size == 0 or data.size % 4 != 0:
        raise ValueError("Invalid velodyne file (size not multiple of 4 floats): {}".format(path))
    return data.reshape(-1, 4)


# --------------------------------------------------------------------------- #
# Calibration
# --------------------------------------------------------------------------- #
def parse_matrix_line(line: str, rows: int, cols: int) -> np.ndarray:
    """Parse "KEY: v0 v1 ..." into a rows x cols float matrix."""
    _, _, values = line.partition(":")
    nums = [float(tok) for tok in values.split()]
    expected = rows * cols
    if len(nums) != expected:
        raise ValueError("Expected {} values, got {} in line: {}".format(expected, len(nums), line.strip()))
    return np.asarray(nums, dtype=float).reshape(rows, cols)


class KittiCalib:
    """
    Parsed KITTI calibration (calib_cam_to_cam / calib_velo_to_cam /
    calib_imu_to_velo).

    Conventions (KITTI devkit):
        P_rect_0X : 3x4 projection matrix of rectified camera X (cam 0 = origin)
        R_rect_00 : 3x3 rotation rectifying cam 0's frame (identity for cam 0)
        T_velo_cam: [R|t] mapping velodyne -> cam0:  p_cam = R*p_velo + t
        T_imu_velo: [R|t] mapping imu -> velodyne:    p_velo = R*p_imu + t
    """

    CAM_TO_CAM_KEYS = ("P_rect_00", "P_rect_01", "P_rect_02", "P_rect_03",
                       "R_rect_00", "K_00", "D_00", "S_rect_00")
    VELO_TO_CAM_KEYS = ("R", "T")
    IMU_TO_VELO_KEYS = ("R", "T")

    def __init__(self, calib_dir: str, date: str):
        """
        Load and parse all three calibration files for a KITTI date folder.

        Args:
            calib_dir: path of the "<date>_calib" directory.
            date:      KITTI date tag, e.g. "2011_09_26" (used for filenames).
        """
        self.cam_to_cam: Dict[str, np.ndarray] = self._load_key_value(
            os.path.join(calib_dir, "calib_cam_to_cam.txt"), self.CAM_TO_CAM_KEYS)
        self.velo_to_cam: Dict[str, np.ndarray] = self._load_key_value(
            os.path.join(calib_dir, "calib_velo_to_cam.txt"), self.VELO_TO_CAM_KEYS)
        self.imu_to_velo: Dict[str, np.ndarray] = self._load_key_value(
            os.path.join(calib_dir, "calib_imu_to_velo.txt"), self.IMU_TO_VELO_KEYS)

    # ---- parsing helpers -------------------------------------------------
    @staticmethod
    def _load_key_value(path: str, keys: Sequence[str]) -> Dict[str, np.ndarray]:
        if not os.path.isfile(path):
            raise FileNotFoundError("Calibration file not found: {}".format(path))
        result: Dict[str, np.ndarray] = {}
        with open(path, "r") as handle:
            lines = handle.readlines()
        for key in keys:
            match = None
            for line in lines:
                if line.strip().startswith(key + ":"):
                    match = line
                    break
            if match is None:
                raise ValueError("Missing key '{}' in {}".format(key, path))
            nvalues = len(match.partition(":")[2].split())
            if nvalues == 12:
                result[key] = parse_matrix_line(match, 3, 4)
            elif nvalues == 9:
                result[key] = parse_matrix_line(match, 3, 3)
            elif nvalues == 5:
                result[key] = parse_matrix_line(match, 1, 5).flatten()
            elif nvalues == 4:
                result[key] = parse_matrix_line(match, 2, 2)
            elif nvalues == 3:
                result[key] = parse_matrix_line(match, 3, 1).flatten()
            elif nvalues == 2:
                result[key] = parse_matrix_line(match, 1, 2).flatten()
            else:
                result[key] = np.asarray([float(t) for t in match.partition(":")[2].split()])
        return result

    # ---- accessors ----------------------------------------------------------
    def projection_matrix(self, cam_index: int) -> np.ndarray:
        """3x4 rectified projection matrix P_rect_0X of camera X."""
        return self.cam_to_cam["P_rect_{:02d}".format(cam_index)]

    def rect_matrix(self) -> np.ndarray:
        """3x3 rectification rotation R_rect_00 (cam0 frame)."""
        return self.cam_to_cam["R_rect_00"]

    def camera_intrinsics(self, cam_index: int) -> np.ndarray:
        """3x3 unrectified intrinsics K_0X (informational; images are rectified)."""
        return self.cam_to_cam["K_{:02d}".format(cam_index)]

    def distortion(self, cam_index: int) -> np.ndarray:
        """5 distortion coefficients D_0X (k1 k2 p1 p2 k3)."""
        return self.cam_to_cam["D_{:02d}".format(cam_index)]

    def image_size(self, cam_index: int) -> Tuple[int, int]:
        """(width, height) of the rectified image S_rect_0X."""
        size = self.cam_to_cam["S_rect_{:02d}".format(cam_index)]
        return int(size[0]), int(size[1])

    def velo_to_cam_transform(self) -> np.ndarray:
        """4x4 homogeneous transform T_velo_cam0 (velodyne -> cam0)."""
        r = self.velo_to_cam["R"]
        t = self.velo_to_cam["T"]
        return np.vstack([np.hstack([r, t.reshape(3, 1)]), [0.0, 0.0, 0.0, 1.0]])

    def cam_to_velo_transform(self) -> np.ndarray:
        """4x4 homogeneous transform T_cam0_velo (cam0 -> velodyne)."""
        return np.linalg.inv(self.velo_to_cam_transform())

    def imu_to_velo_transform(self) -> np.ndarray:
        """4x4 homogeneous transform T_imu_velo (imu -> velodyne)."""
        r = self.imu_to_velo["R"]
        t = self.imu_to_velo["T"]
        return np.vstack([np.hstack([r, t.reshape(3, 1)]), [0.0, 0.0, 0.0, 1.0]])

    def imu_to_cam_transform(self) -> np.ndarray:
        """4x4 homogeneous transform T_imu_cam0 (imu -> cam0)."""
        return self.velo_to_cam_transform().dot(self.imu_to_velo_transform())


# --------------------------------------------------------------------------- #
# Paths
# --------------------------------------------------------------------------- #
class KittiPaths:
    """Builds the on-disk layout of one KITTI raw drive (no hardcoded paths)."""

    def __init__(self, root: str, date: str, drive: str):
        self.root = root
        self.date = date
        self.drive = drive
        self.raw_root = os.path.join(root, "raw", date)
        self.drive_dir = os.path.join(self.raw_root, drive + "_sync")
        self.calib_dir = os.path.join(self.raw_root, date + "_calib")

    @property
    def timestamps_path(self) -> str:
        return os.path.join(self.drive_dir, "timestamps.txt")

    def image_dir(self, cam_index: int) -> str:
        return os.path.join(self.drive_dir, "image_{:02d}".format(cam_index))

    def image_path(self, frame_index: int, cam_index: int) -> str:
        return os.path.join(self.image_dir(cam_index), "{:010d}.jpg".format(frame_index))

    @property
    def velodyne_dir(self) -> str:
        return os.path.join(self.drive_dir, "velodyne")

    def velodyne_path(self, frame_index: int) -> str:
        return os.path.join(self.velodyne_dir, "{:010d}.bin".format(frame_index))

    @property
    def oxts_dir(self) -> str:
        return os.path.join(self.drive_dir, "oxts")

    def oxts_path(self, frame_index: int) -> str:
        return os.path.join(self.oxts_dir, "{:010d}.txt".format(frame_index))

    def validate(self) -> None:
        """Raise FileNotFoundError early when the expected layout is missing."""
        required = [self.drive_dir, self.calib_dir, self.timestamps_path,
                    self.image_dir(0), self.velodyne_dir, self.oxts_dir]
        for path in required:
            if not os.path.exists(path):
                raise FileNotFoundError("Missing KITTI path: {}".format(path))


class KittiOdometryPaths:
    """Builds the on-disk layout of the KITTI ODOMETRY dataset (sequences 00-10)."""

    def __init__(self, root: str, sequence: str = "00"):
        self.root = root
        self.sequence = str(sequence).zfill(2)
        self.odometry_root = os.path.join(root, "odometry")
        self.seq_dir = os.path.join(self.odometry_root, "sequences", self.sequence)

    @property
    def calib_path(self) -> str:
        return os.path.join(self.seq_dir, "calib.txt")

    @property
    def times_path(self) -> str:
        return os.path.join(self.seq_dir, "times.txt")

    def image_dir(self, cam_index: int) -> str:
        return os.path.join(self.seq_dir, "image_{:02d}".format(cam_index))

    def image_path(self, frame_index: int, cam_index: int) -> str:
        return os.path.join(self.image_dir(cam_index),
                            "{:06d}.png".format(frame_index))

    @property
    def velodyne_dir(self) -> str:
        return os.path.join(self.seq_dir, "velodyne")

    def velodyne_path(self, frame_index: int) -> str:
        return os.path.join(self.velodyne_dir, "{:06d}.bin".format(frame_index))

    @property
    def poses_path(self) -> str:
        return os.path.join(self.odometry_root, "poses",
                            "{}.txt".format(self.sequence))

    def validate(self) -> None:
        required = [self.seq_dir, self.calib_path, self.times_path,
                    self.image_dir(0), self.velodyne_dir, self.poses_path]
        for path in required:
            if not os.path.exists(path):
                raise FileNotFoundError("Missing KITTI odometry path: {}".format(path))


# --------------------------------------------------------------------------- #
# Odometry ground truth (KITTI odometry benchmark)
# --------------------------------------------------------------------------- #
def read_odometry_poses(path: str) -> List[np.ndarray]:
    """
    Read KITTI odometry ground-truth poses file (poses/<seq>.txt).

    Each line holds 12 floats = a 3x4 [R | t] matrix (camera 0 -> world).
    Returns a list of 4x4 homogeneous matrices.
    """
    poses: List[np.ndarray] = []
    with open(path, "r") as handle:
        for lineno, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            values = [float(tok) for tok in line.split()]
            if len(values) != 12:
                raise ValueError(
                    "Invalid pose at {}:{} (expected 12 floats, got {})".format(
                        path, lineno, len(values)))
            matrix = np.eye(4)
            matrix[:3, :] = np.asarray(values, dtype=float).reshape(3, 4)
            poses.append(matrix)
    if not poses:
        raise ValueError("No poses found in {}".format(path))
    return poses


def read_odometry_times(path: str) -> List[float]:
    """Read KITTI odometry times.txt (one relative timestamp per frame)."""
    times: List[float] = []
    with open(path, "r") as handle:
        for lineno, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                times.append(float(line))
            except ValueError as exc:
                raise ValueError("Invalid time at {}:{}".format(path, lineno)) from exc
    if not times:
        raise ValueError("No times found in {}".format(path))
    return times


# --------------------------------------------------------------------------- #
# Geometry helpers
# --------------------------------------------------------------------------- #
def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Euler angles (rad) -> quaternion [x, y, z, w].

    Convention (matches the KITTI devkit OXTS handling):
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    rot = rotation_matrix_from_euler(roll, pitch, yaw)
    return matrix_to_quaternion(rot)


def rotation_matrix_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build the 3x3 rotation matrix R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])
    return rz.dot(ry).dot(rx)


def matrix_to_quaternion(rot: np.ndarray) -> np.ndarray:
    """
    Rotation matrix (3x3) -> quaternion [x, y, z, w] (stable Shepperd's method).
    """
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=float)


# --------------------------------------------------------------------------- #
# Geodesy (GPS -> local ENU frame, used by gps_node / localization later)
# --------------------------------------------------------------------------- #
def ecef_from_geo(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    """WGS-84 geodetic (lat, lon, alt) -> ECEF [x, y, z] in meters."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * math.sin(lat) ** 2)
    x = (n + alt) * math.cos(lat) * math.cos(lon)
    y = (n + alt) * math.cos(lat) * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt) * math.sin(lat)
    return np.array([x, y, z])


def enu_from_ecef(origin_ecef: np.ndarray, point_ecef: np.ndarray,
                  origin_lat_deg: float, origin_lon_deg: float) -> np.ndarray:
    """ECEF delta from `origin_ecef` -> local East-North-Up coordinates."""
    delta = np.asarray(point_ecef, dtype=float) - np.asarray(origin_ecef, dtype=float)
    lat = math.radians(origin_lat_deg)
    lon = math.radians(origin_lon_deg)
    rotation = np.array([
        [-math.sin(lon), math.cos(lon), 0.0],
        [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)],
        [math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)],
    ])
    return rotation.dot(delta)


def geo_to_enu(lat_deg: float, lon_deg: float, alt: float,
               origin_lat_deg: float, origin_lon_deg: float) -> np.ndarray:
    """Geodetic point -> local ENU relative to an origin (meters)."""
    origin_ecef = ecef_from_geo(origin_lat_deg, origin_lon_deg, 0.0)
    point_ecef = ecef_from_geo(lat_deg, lon_deg, alt)
    return enu_from_ecef(origin_ecef, point_ecef, origin_lat_deg, origin_lon_deg)
