# dataset_loader

> **Phase 2 — ✅ implemented**

## 1. Objective
Provide the shared KITTI dataset infrastructure: pure parsers (calibration,
OXTS, timestamps, velodyne), replay pacing, ROS message builders, and an
offline `kitti_to_bag` converter. Everything the sensor driver nodes need, in
one reusable package.

## 2. Theory
- **Timestamps**: parsed to *integer nanoseconds* — float64 at epoch scale
  (~1.3e9 s) has only ~0.5 µs resolution and would silently break exact sync.
- **Calibration**: `calib_cam_to_cam.txt` (P_rect, R_rect, K, D),
  `calib_velo_to_cam.txt` (T_velo_cam0), `calib_imu_to_velo.txt` (T_imu_velo).
- **Point clouds**: `.bin` files are already `[x,y,z,intensity]` float32 —
  referenced directly as a `PointCloud2` payload (zero-copy).
- **Pacing**: `RatePacer` replays frame timestamps at configurable speed.

## 3. Industrial importance
Dataset infrastructure is a first-class engineering artifact at AMR companies
(record once, replay forever). Offline bag conversion + deterministic replay
are the backbone of debugging and CI for autonomy stacks.

## 4. Folder structure
```
dataset_loader/
├── package.xml  CMakeLists.txt  setup.py
├── src/dataset_loader/
│   ├── kitti_parsers.py   # pure Python, no ROS (unit-tested)
│   ├── pacing.py          # RatePacer (pure Python)
│   └── player_utils.py    # ROS message builders (shared)
├── scripts/kitti_to_bag.py
├── launch/kitti_player.launch
└── test/test_kitti_parsers.py   # 21 unit tests (run without ROS)
```

## 5. Required packages
`rospy std_msgs sensor_msgs geometry_msgs tf2_msgs message_filters rosbag
python3-numpy python3-opencv` (+ `dataset_loader` module importable by others)

## 6-8. ROS topics
None of its own — it *builds* the messages the driver nodes publish and the
bags they produce.

## 9. Parameters / 10. Configuration
Pacing defaults (`rate_factor`, `loop`) are defined in each driver node's own
`config/*.yaml` (single source of truth per node); dataset paths come from
launch args / `$KITTI_ROOT` / `config/kitti_dataset.yaml`.

## 11. Python classes
`KittiPaths`, `KittiCalib`, `OxtsSample`, `OxtsParser`, `RatePacer`,
`parse_kitti_timestamp_ns()`, `read_timestamps_ns()`, `read_velodyne_bin()`,
`quaternion_from_euler()`, `matrix_to_quaternion()`, `ecef_from_geo()`,
`geo_to_enu()`, message builders in `player_utils`.

## 12. Launch
```bash
roslaunch dataset_loader kitti_player.launch            # all 4 drivers
roslaunch dataset_loader kitti_player.launch rate:=0    # fast replay
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/dataset_loader/test/test_kitti_parsers.py
# on the robot/VM:
rosrun dataset_loader kitti_to_bag.py --root /data/kitti \
    --date 2011_09_26 --drive 2011_09_26_drive_0005 \
    --output /data/kitti/sample.bag
```

## 14. RViz configuration
Use `adaptive_amr/rviz/kitti_sensors.rviz` (this package provides data).

## 15. Expected outputs
`kitti_to_bag` → bag with `/camera/image_raw`, `/velodyne_points`, `/imu/data`,
`/gps/fix`, `/tf_static`. Unit tests: `Ran 21 tests ... OK`.

## 16. Performance metrics
Zero-copy PointCloud2 (no per-point repacking); bag conversion ~faster than
real-time; parsers < 1 ms/frame.

## 17. Debugging guide
`rostopic echo` a bag; `rosbag info`; unit tests isolate parser bugs.

## 18. Common errors
"Missing key P_rect_01" → wrong calib file for the date; validate with
`scripts/kitti/verify_kitti.py`.

## 19. Improvements
Add streaming (load-next-file-async), dataset versioning, odometry-format
support, `sensor_msgs/CompressedImage` recording option.

## 20. Git commit
`feat(dataset_loader): add KITTI parsers, message builders and bag converter`
