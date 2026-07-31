#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_kitti_parsers.py — unit tests for the pure-Python KITTI parsers.

Run without ROS:
    python3 test_kitti_parsers.py            # verbose
    python3 -m unittest test_kitti_parsers   # summary
    python3 -m unittest discover dataset_loader/test

Covers: timestamps (ns precision), OXTS parsing, calibration parsing,
velodyne scans, quaternion/euler conversion, ECEF/ENU geodesy, paths, pacing.
"""

import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dataset_loader.kitti_parsers import (  # noqa: E402
    WGS84_A, KittiCalib, KittiPaths, OxtsParser, OxtsSample,
    ecef_from_geo, geo_to_enu, matrix_to_quaternion,
    parse_kitti_timestamp_ns, quaternion_from_euler,
    read_timestamps_ns, read_velodyne_bin, rotation_matrix_from_euler,
)

CALIB_CAM = """calib_time: 09-Jan-2012 13:57:47
corner_dist: 9.950000e-02
S_00: 1.392000e+03 5.120000e+02
K_00: 9.842439e+02 0.000000e+00 6.900000e+02 0.000000e+00 9.808141e+02 2.331966e+02 0.000000e+00 0.000000e+00 1.000000e+00
D_00: 7.083413e-03 -1.907353e-01 -2.847186e-03 -1.372087e-03 1.144850e-01
R_00: 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00
T_00: 2.573699e-16 -1.059758e-16 1.614870e-16
S_rect_00: 1.242000e+03 3.760000e+02
R_rect_00: 9.999239e-01 9.837760e-03 -7.445048e-03 -9.869795e-03 9.999421e-01 -4.278459e-03 7.402527e-03 4.351614e-03 9.999631e-01
P_rect_00: 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
P_rect_01: 7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
P_rect_02: 7.215377e+02 0.000000e+00 6.095593e+02 -3.895575e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
P_rect_03: 7.215377e+02 0.000000e+00 6.095593e+02 -7.806575e+02 0.000000e+00 7.215377e+02 1.728540e+02 0.000000e+00 0.000000e+00 0.000000e+00 1.000000e+00 0.000000e+00
"""

CALIB_VELO = """calib_time: 09-Jan-2012 13:57:47
R: 7.533745e-03 -9.999714e-01 -6.166020e-04 1.480249e-02 7.280733e-04 -9.998902e-01 9.998621e-01 1.332968e-02 7.527479e-03
T: -4.069766e-03 -7.631618e-02 -2.717806e-01
delta_f: 0.000000e+00 0.000000e+00
delta_c: 0.000000e+00 0.000000e+00
"""

CALIB_IMU = """calib_time: 09-Jan-2012 13:57:47
R: 9.999976e-01 7.553071e-04 -2.035826e-03 -7.854027e-04 9.998898e-01 -1.482298e-02 2.024406e-03 1.482454e-02 9.998881e-01
T: -8.086759e-01 3.195559e-01 -7.997231e-01
"""

OXTS_LINE = ("4.900228e+02 8.138182e+02 1.043909e+02 1.307151e-04 -1.451651e-02 "
             "-4.448093e-03 1.982758e-01 1.885651e-01 2.702453e-01 -1.683047e-02 "
             "2.173039e-02 -1.215741e+00 2.173696e-02 -9.709203e-01 -1.215741e+00 "
             "2.173696e-02 -9.709203e-01 4.910699e-03 5.358119e-04 2.724368e-03 "
             "4.910699e-03 5.358119e-04 2.724368e-03 5.299938e-01 2.575907e-02 "
             "4 4 4 5 5")


class TestTimestamps(unittest.TestCase):
    def test_parse_nanosecond_precision(self):
        ns = parse_kitti_timestamp_ns("2011-09-26 13:02:10.123456789")
        self.assertEqual(1317042130123456789, ns)

    def test_parse_no_fraction(self):
        ns = parse_kitti_timestamp_ns("2011-09-26 13:02:10")
        self.assertEqual(1317042130000000000, ns)

    def test_read_file(self):
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as handle:
            handle.write("2011-09-26 13:02:10.000000000\n2011-09-26 13:02:10.100000000\n")
            path = handle.name
        try:
            stamps = read_timestamps_ns(path)
            self.assertEqual(2, len(stamps))
            self.assertEqual(100_000_000, stamps[1] - stamps[0])  # exactly 100 ms
        finally:
            os.unlink(path)

    def test_precision_kept(self):
        """Two frames 1 ns apart must stay distinct (float64 would collapse them)."""
        a = parse_kitti_timestamp_ns("2011-09-26 13:02:10.123456780")
        b = parse_kitti_timestamp_ns("2011-09-26 13:02:10.123456781")
        self.assertEqual(1, b - a)


class TestOxts(unittest.TestCase):
    def test_parse_line(self):
        sample = OxtsSample.from_line(OXTS_LINE)
        self.assertAlmostEqual(490.0228, sample.lat, places=3)
        self.assertAlmostEqual(813.8182, sample.lon, places=3)
        self.assertEqual(4, sample.navstat)
        self.assertTrue(sample.has_fix)
        self.assertEqual(4, sample.numsats)

    def test_wrong_length_raises(self):
        with self.assertRaises(ValueError):
            OxtsSample.from_line("1 2 3")

    def test_parser_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "0000000000.txt")
            with open(path, "w") as handle:
                handle.write(OXTS_LINE + "\n")
            sample = OxtsParser(tmp).read_sample(0)
            self.assertIsNotNone(sample)
            self.assertIsNone(OxtsParser(tmp).read_sample(1))

    def test_quaternion_fields(self):
        sample = OxtsSample.from_line(OXTS_LINE)
        q = sample.quaternion_xyzw
        self.assertAlmostEqual(1.0, np.sum(q ** 2), places=9)


class TestCalibration(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        with open(os.path.join(self.tmp, "calib_cam_to_cam.txt"), "w") as f:
            f.write(CALIB_CAM)
        with open(os.path.join(self.tmp, "calib_velo_to_cam.txt"), "w") as f:
            f.write(CALIB_VELO)
        with open(os.path.join(self.tmp, "calib_imu_to_velo.txt"), "w") as f:
            f.write(CALIB_IMU)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_parse(self):
        calib = KittiCalib(self.tmp, "2011_09_26")
        p0 = calib.projection_matrix(0)
        self.assertEqual((3, 4), p0.shape)
        self.assertAlmostEqual(721.5377, p0[0, 0], places=3)
        self.assertEqual((1242, 376), calib.image_size(0))
        self.assertEqual(5, len(calib.distortion(0)))
        self.assertEqual((3, 3), calib.camera_intrinsics(0).shape)

    def test_velo_to_cam_transform(self):
        calib = KittiCalib(self.tmp, "2011_09_26")
        t = calib.velo_to_cam_transform()
        self.assertEqual((4, 4), t.shape)
        # velodyne origin -> cam0 position must equal T (by definition)
        p_velo = np.array([0.0, 0.0, 0.0, 1.0])
        p_cam = t.dot(p_velo)
        self.assertAlmostEqual(-4.069766e-03, p_cam[0], places=5)
        self.assertAlmostEqual(-7.631618e-02, p_cam[1], places=5)
        self.assertAlmostEqual(-2.717806e-01, p_cam[2], places=5)

    def test_imu_to_cam_chain(self):
        calib = KittiCalib(self.tmp, "2011_09_26")
        t_imu_cam = calib.imu_to_cam_transform()
        expected = calib.velo_to_cam_transform().dot(calib.imu_to_velo_transform())
        np.testing.assert_allclose(expected, t_imu_cam, atol=1e-9)


class TestVelodyne(unittest.TestCase):
    def test_read_bin(self):
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as handle:
            points = np.random.rand(100, 4).astype(np.float32)
            handle.write(points.tobytes())
            path = handle.name
        try:
            parsed = read_velodyne_bin(path)
            self.assertEqual((100, 4), parsed.shape)
            np.testing.assert_array_equal(points, parsed)
        finally:
            os.unlink(path)

    def test_bad_size_raises(self):
        with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as handle:
            handle.write(b"\x00\x00\x00\x00\x00\x00\x00")  # 7 bytes -> invalid
            path = handle.name
        try:
            with self.assertRaises(ValueError):
                read_velodyne_bin(path)
        finally:
            os.unlink(path)


class TestGeometry(unittest.TestCase):
    def test_quaternion_euler_identity(self):
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        np.testing.assert_allclose([0.0, 0.0, 0.0, 1.0], q, atol=1e-12)

    def test_quaternion_yaw_90(self):
        """Rz(90deg): +x -> +y, so the quaternion must be a z-rotation."""
        q = quaternion_from_euler(0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(0.0, q[0], places=9)
        self.assertAlmostEqual(0.0, q[1], places=9)
        self.assertAlmostEqual(math.sqrt(0.5), q[2], places=9)
        self.assertAlmostEqual(math.sqrt(0.5), q[3], places=9)

    def test_matrix_quaternion_roundtrip(self):
        rot = rotation_matrix_from_euler(0.3, -0.2, 0.7)
        q = matrix_to_quaternion(rot)
        self.assertAlmostEqual(1.0, np.sum(q ** 2), places=9)
        # recompose and compare
        recomposed = rotation_matrix_from_euler(0.3, -0.2, 0.7)
        np.testing.assert_allclose(recomposed, rot, atol=1e-9)


class TestGeodesy(unittest.TestCase):
    def test_ecef_equator(self):
        x, y, z = ecef_from_geo(0.0, 0.0, 0.0)
        self.assertAlmostEqual(WGS84_A, x, places=3)
        self.assertAlmostEqual(0.0, y, places=3)
        self.assertAlmostEqual(0.0, z, places=3)

    def test_enu_roundtrip(self):
        lat0, lon0 = 49.0, 8.4  # Karlsruhe (KITTI location)
        enu = geo_to_enu(lat0 + 1e-4, lon0, 0.0, lat0, lon0)
        # 1e-4 deg latitude ~ 11.1 m north
        self.assertAlmostEqual(0.0, enu[0], places=2)      # east ~ 0
        self.assertAlmostEqual(11.1, enu[1], places=0)     # north ~ 11.1 m
        self.assertAlmostEqual(0.0, enu[2], places=2)      # up ~ 0


class TestPaths(unittest.TestCase):
    def test_layout(self):
        paths = KittiPaths("/data/kitti", "2011_09_26", "2011_09_26_drive_0005")
        self.assertTrue(paths.image_path(3, 2).endswith("image_02/0000000003.jpg"))
        self.assertTrue(paths.velodyne_path(3).endswith("velodyne/0000000003.bin"))
        self.assertTrue(paths.oxts_path(3).endswith("oxts/0000000003.txt"))
        self.assertTrue(paths.timestamps_path.endswith("timestamps.txt"))


class TestPacer(unittest.TestCase):
    def test_pacing(self):
        from dataset_loader.pacing import RatePacer

        sleeps = []
        pacer = RatePacer([0, 100_000_000, 200_000_000], rate_factor=1.0,
                          sleep_fn=sleeps.append)
        pacer.sleep_until_frame(0)
        self.assertEqual([], sleeps)          # first frame: no sleep
        pacer.sleep_until_frame(1)
        self.assertAlmostEqual(0.1, sleeps[0], places=6)
        pacer.sleep_until_frame(2)
        self.assertAlmostEqual(0.1, sleeps[1], places=6)

    def test_fast_mode_no_sleep(self):
        from dataset_loader.pacing import RatePacer

        sleeps = []
        pacer = RatePacer([0, 100_000_000], rate_factor=0.0, sleep_fn=sleeps.append)
        pacer.sleep_until_frame(1)
        self.assertEqual([], sleeps)


if __name__ == "__main__":
    unittest.main(verbosity=2)
