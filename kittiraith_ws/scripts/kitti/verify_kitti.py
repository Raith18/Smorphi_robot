#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_kitti.py — KITTI dataset structure validator.

Validates the directory layout of a KITTI dataset (raw or odometry) against the
official conventions, so that every later module (player, sync, calibration,
odometry, mapping) can rely on the data layout without surprises.

Pure Python 3 standard library — runs on any host, no ROS needed.

Usage:
    python3 verify_kitti.py --root /data/kitti
    python3 verify_kitti.py --root /data/kitti --kind odometry --max-frames 500

Exit code: 0 = valid, 1 = problems found.
"""

import argparse
import os
import sys

# --------------------------------------------------------------------------- #
# Expected calibration keys (subset; see KITTI devkit readme)
# --------------------------------------------------------------------------- #
RAW_CALIB_FILES = {
    "calib_cam_to_cam.txt": ("P0:", "P1:", "P2:", "P3:", "R_rect_00:", "S_00:"),
    "calib_velo_to_cam.txt": ("R:", "T:", "delta_f:", "delta_c:"),
    "calib_imu_to_velo.txt": ("R:", "T:"),
}
ODO_CALIB_KEYS = ("P0:", "P1:", "P2:", "P3:", "Tr:", "R0_rect:")

RAW_SENSOR_DIRS = ("image_00", "image_01", "image_02", "image_03", "velodyne", "oxts")
ODOMETRY_SENSOR_DIRS = ("image_00", "image_01", "velodyne")


class Report:
    """Collects check results and formats them."""

    def __init__(self, quiet=False):
        self.quiet = quiet
        self.ok = 0
        self.warnings = 0
        self.errors = 0

    def _emit(self, symbol, msg):
        if not self.quiet:
            print("  [{}] {}".format(symbol, msg))

    def pass_(self, msg):
        self.ok += 1
        self._emit("PASS", msg)

    def warn(self, msg):
        self.warnings += 1
        self._emit("WARN", msg)

    def error(self, msg):
        self.errors += 1
        self._emit("FAIL", msg)

    def summary(self):
        return self.ok, self.warnings, self.errors


def count_files(directory, suffix=None):
    """Count files in `directory`, optionally filtering by suffix."""
    if not os.path.isdir(directory):
        return -1
    total = 0
    for name in os.listdir(directory):
        path = os.path.join(directory, name)
        if os.path.isfile(path) and (suffix is None or name.endswith(suffix)):
            total += 1
    return total


def count_lines(path):
    """Count non-empty lines of a text file."""
    with open(path, "r") as handle:
        return sum(1 for line in handle if line.strip())


def check_calib_keys(path, required_keys, report, label):
    """Ensure a calibration file contains the expected keys."""
    if not os.path.isfile(path):
        report.error("{}: missing calibration file {}".format(label, path))
        return
    with open(path, "r") as handle:
        content = handle.read()
    missing = [k for k in required_keys if k not in content]
    if missing:
        report.error("{}: missing keys {} in {}".format(label, missing, path))
    else:
        report.pass_("{}: calibration keys present ({})".format(label, ", ".join(required_keys)))


def detect_kind(root):
    """Auto-detect raw vs odometry layout."""
    if os.path.isdir(os.path.join(root, "raw")):
        return "raw"
    if os.path.isdir(os.path.join(root, "odometry")):
        return "odometry"
    if os.path.isdir(os.path.join(root, "sequences")):
        return "odometry"
    return None


# --------------------------------------------------------------------------- #
# Raw dataset checks
# --------------------------------------------------------------------------- #
def validate_raw(root, report, max_frames):
    raw_root = os.path.join(root, "raw")
    if not os.path.isdir(raw_root):
        report.error("raw root missing: {}".format(raw_root))
        return False

    dates = sorted(d for d in os.listdir(raw_root) if os.path.isdir(os.path.join(raw_root, d)))
    if not dates:
        report.error("no date folders under {}".format(raw_root))
        return False
    report.pass_("raw date folders: {}".format(", ".join(dates)))

    all_ok = True
    for date in dates:
        date_dir = os.path.join(raw_root, date)
        drives = sorted(
            d for d in os.listdir(date_dir)
            if os.path.isdir(os.path.join(date_dir, d)) and d.endswith("_sync")
        )
        if not drives:
            report.error("{}: no '_sync' drive folders found".format(date))
            all_ok = False
            continue

        calib_dir = os.path.join(date_dir, "{}_calib".format(date))
        for fname, keys in RAW_CALIB_FILES.items():
            check_calib_keys(os.path.join(calib_dir, fname), keys, report, date)

        for drive in drives:
            drive_dir = os.path.join(date_dir, drive)
            report.pass_("drive found: {}/{}".format(date, drive))

            timestamps = os.path.join(drive_dir, "timestamps.txt")
            ts_lines = count_lines(timestamps) if os.path.isfile(timestamps) else -1

            counts = {}
            for sensor in RAW_SENSOR_DIRS:
                ext = ".jpg" if sensor.startswith("image") else (".bin" if sensor == "velodyne" else ".txt")
                n = count_files(os.path.join(drive_dir, sensor), ext)
                counts[sensor] = n
                if n <= 0:
                    report.error("{}/{}: no {} files in {}/".format(date, drive, ext, sensor))
                    all_ok = False
                elif max_frames and n < max_frames:
                    report.warn("{}/{}: {} has {} frames (< --max-frames {})".format(date, drive, sensor, n, max_frames))

            # Cross-sensor frame consistency (the essence of the *sync* drive)
            ref = None
            for sensor in ("image_02", "velodyne", "oxts"):
                n = counts[sensor]
                if n <= 0:
                    continue
                if ref is None:
                    ref = n
                elif n != ref:
                    report.error("{}/{}: frame mismatch image_02={} velodyne={} oxts={}".format(
                        date, drive, counts["image_02"], counts["velodyne"], counts["oxts"]))
                    all_ok = False

            if ts_lines > 0:
                if ref is not None and ts_lines != ref:
                    report.error("{}/{}: timestamps.txt has {} lines but {} frames".format(date, drive, ts_lines, ref))
                    all_ok = False
                else:
                    report.pass_("{}/{}: timestamps.txt OK ({} lines)".format(date, drive, ts_lines))
            else:
                report.error("{}/{}: timestamps.txt missing/empty".format(date, drive))
                all_ok = False

            if ref is not None and all_ok:
                report.pass_("{}/{}: synchronized frames consistent ({} frames)".format(date, drive, ref))
    return all_ok


# --------------------------------------------------------------------------- #
# Odometry dataset checks
# --------------------------------------------------------------------------- #
def validate_odometry(root, report, max_frames):
    odo_root = os.path.join(root, "odometry")
    if not os.path.isdir(odo_root):
        odo_root = root
    seq_root = os.path.join(odo_root, "sequences")
    if not os.path.isdir(seq_root):
        report.error("sequences root missing: {}".format(seq_root))
        return False

    seqs = sorted(
        d for d in os.listdir(seq_root)
        if os.path.isdir(os.path.join(seq_root, d)) and d.isdigit()
    )
    if not seqs:
        report.error("no sequence folders (00..NN) under {}".format(seq_root))
        return False
    report.pass_("odometry sequences found: {}".format(", ".join(seqs)))

    all_ok = True
    for seq in seqs:
        seq_dir = os.path.join(seq_root, seq)
        check_calib_keys(os.path.join(seq_dir, "calib.txt"), ODO_CALIB_KEYS, report, "seq {}".format(seq))

        times = os.path.join(seq_dir, "times.txt")
        n_times = count_lines(times) if os.path.isfile(times) else -1

        counts = {}
        for sensor in ODOMETRY_SENSOR_DIRS:
            ext = ".png" if sensor.startswith("image") else ".bin"
            n = count_files(os.path.join(seq_dir, sensor), ext)
            counts[sensor] = n
            if n <= 0:
                report.error("seq {}: no {} files in {}/".format(seq, ext, sensor))
                all_ok = False
            elif max_frames and n < max_frames:
                report.warn("seq {}: {} has {} frames (< --max-frames {})".format(seq, sensor, n, max_frames))

        ref = counts["image_00"]
        if ref > 0:
            for sensor in ("image_01", "velodyne"):
                if counts[sensor] not in (ref, -1):
                    report.error("seq {}: frame mismatch image_00={} {}={}".format(
                        seq, ref, sensor, counts[sensor]))
                    all_ok = False
            if n_times > 0 and n_times != ref:
                report.error("seq {}: times.txt has {} lines but {} frames".format(seq, n_times, ref))
                all_ok = False
            elif n_times > 0:
                report.pass_("seq {}: synchronized ({} frames, times.txt OK)".format(seq, ref))

        poses_file = os.path.join(odo_root, "poses", "{}.txt".format(seq))
        if os.path.isfile(poses_file):
            n_poses = count_lines(poses_file)
            if ref > 0 and n_poses != ref:
                report.error("seq {}: poses have {} lines but {} frames".format(seq, n_poses, ref))
                all_ok = False
            else:
                report.pass_("seq {}: ground-truth poses OK ({} poses)".format(seq, n_poses))
        else:
            report.warn("seq {}: ground-truth poses not found ({} missing)".format(seq, poses_file))
    return all_ok


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def parse_args(argv):
    parser = argparse.ArgumentParser(description="KITTI dataset structure validator")
    parser.add_argument("--root", default=os.environ.get("KITTI_ROOT", "./data/kitti"),
                        help="KITTI dataset root (default: $KITTI_ROOT or ./data/kitti)")
    parser.add_argument("--kind", choices=("auto", "raw", "odometry"), default="auto",
                        help="dataset kind (default: auto-detect)")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="warn when a sensor folder has fewer frames than this")
    parser.add_argument("--quiet", action="store_true", help="only print problems")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    report = Report(quiet=args.quiet)

    print("KITTI validator")
    print("  root : {}".format(args.root))

    if not os.path.isdir(args.root):
        report.error("root does not exist: {}".format(args.root))
        return 1

    kind = args.kind if args.kind != "auto" else detect_kind(args.root)
    if kind is None:
        report.error("could not detect dataset kind under {}".format(args.root))
        report.warn("expected: <root>/raw/<date>/<drive>_sync/...  or  <root>/odometry/sequences/00/...")
        return 1
    print("  kind : {}".format(kind))

    if kind == "raw":
        ok = validate_raw(args.root, report, args.max_frames)
    else:
        ok = validate_odometry(args.root, report, args.max_frames)

    passed, warns, errors = report.summary()
    print("-" * 60)
    print("RESULT: {} passed, {} warnings, {} errors".format(passed, warns, errors))
    return 0 if (ok and errors == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
