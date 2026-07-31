# Phase 2 — KITTI ROS Player · Time Synchronization · Calibration · Sensor Drivers

> **Module status: ✅ COMPLETE**
>
> This document is the full Phase 2 tutorial: theory (with mathematics),
> industrial context, every implemented file, testing procedure, debugging,
> performance, and interview questions.

---

## Table of contents

1. [Objective](#1-objective)
2. [Theory](#2-theory)
3. [Industrial importance](#3-industrial-importance)
4. [Folder structure](#4-folder-structure)
5. [Required packages](#5-required-packages)
6. [ROS topics](#6-ros-topics)
7. [TF frames](#7-tf-frames)
8. [Parameters & configuration](#8-parameters--configuration)
9. [Python classes](#9-python-classes)
10. [Implementation walkthrough](#10-implementation-walkthrough)
11. [Launch files](#11-launch-files)
12. [RViz configuration](#12-rviz-configuration)
13. [Testing procedure](#13-testing-procedure)
14. [Expected outputs](#14-expected-outputs)
15. [Performance metrics](#15-performance-metrics)
16. [Debugging guide](#16-debugging-guide)
17. [Common errors](#17-common-errors)
18. [Improvements](#18-improvements)
19. [Interview questions](#19-interview-questions)
20. [Git commits for this phase](#20-git-commits-for-this-phase)

---

## 1. Objective

Build the **sensor layer** of the stack: turn raw KITTI files into a live,
time-synchronized, calibrated ROS robot.

| Deliverable | Package(s) |
|---|---|
| Dataset infrastructure (parsers, pacing, bag tool) | `dataset_loader` |
| Camera driver (`/camera/image_raw`, `/camera/image_rect`) | `camera_node` |
| LiDAR driver (`/velodyne_points`) | `lidar_node` |
| GPS driver (`/gps/fix`) | `gps_node` |
| IMU driver (`/imu/data`) | `imu_node` |
| Time synchronization (`/time_sync/*`) | `time_sync` |
| Calibration → `CameraInfo` + TF tree | `calibration` |
| Shared message package (created; msgs in Phase 4) | `adaptive_amr_msgs` |

**Definition of done:** `roslaunch adaptive_amr phase2_kitti_sensors.launch`
produces all Phase-2 topics with correct content and stamps; exact time sync
reports zero-latency synced sets; RViz shows KITTI imagery + point clouds in
the right TF frames.

---

## 2. Theory

### 2.1 Coordinate frames, transforms and the TF tree

A rigid transform `T_a_b` maps points from frame `b` into frame `a`:

```
p_a = T_a_b · p_b = R_a_b · p_b + t_a_b        (4x4 homogeneous form)
```

Transforms **compose**: `T_a_c = T_a_b · T_b_c`. This is the entire machinery
behind ROS `tf`/`tf2` — a tree of frames where every node knows its parent
transform, and the `tf` library computes any pair by walking the tree.

The project TF tree (from the spec):

```
map ──> odom ──> base_link ──> laser ──> camera_link ──> camera_optical_frame
                                            └──────> imu_link
```

KITTI gives us two extrinsic transforms (as `[R | t]`):
`T_velo_cam0` (velodyne→cam0) and `T_imu_velo` (imu→velodyne). We assign:

| TF edge | Transform | Source |
|---|---|---|
| `base_link → laser` | identity | KITTI has no body frame; velodyne defines it |
| `laser → camera_link` | `T_velo_cam0` | `calib_velo_to_cam.txt` |
| `camera_link → camera_optical_frame` | identity | KITTI cam0 *is* an optical frame (x right, y down, z forward) |
| `camera_link → imu_link` | `T_imu_cam0 = T_velo_cam0 · T_imu_velo` | chained extrinsics |
| `map → odom`, `odom → base_link` | identity | placeholders until Phase 5 localization |

### 2.2 Camera model and projection

The pinhole model projects a 3D point into the image:

```
s · [u, v, 1]ᵀ = K · [R | t] · [X, Y, Z, 1]ᵀ
        with  K = [ fx  0  cx ]
                  [ 0  fy  cy ]
                  [ 0   0   1 ]
```

- `K`: intrinsics (focal lengths + principal point).
- `[R|t]`: extrinsics (world→camera).
- Rectification rotates both cameras to a common plane so that epipolar lines
  are horizontal: `p_rect = R_rect · p_cam`.
- KITTI's `P_rect_0X` are **3×4 rectified projection matrices** — the product
  `K_rect · R_rect · [I|0]` in camera-0 coordinates.

KITTI `_sync` images are **already rectified**, so `camera_info` uses
`R = R_rect_00` and `P = P_rect_0X` (and `image_rect == image_raw` initially;
Phase 3 `camera_processing` will perform real rectification for other inputs).

### 2.3 LiDAR → PointCloud2

KITTI velodyne scans are 10 Hz files of `[x, y, z, reflectance]` float32 —
already the layout of a `PointCloud2` message. The node therefore publishes
the **raw bytes** with a 4-field description (zero-copy):

| field | offset | type |
|---|---|---|
| x | 0 | float32 |
| y | 4 | float32 |
| z | 8 | float32 |
| intensity | 12 | float32 |

### 2.4 GPS and IMU (OXTS)

The KITTI `oxts` folder holds one 30-value line per frame (10 Hz):

```
lat lon alt | roll pitch yaw | vn ve vf vl vu | ax ay az af al au |
wx wy wz wf wl wu | pos_acc vel_acc | navstat numsats posmode velmode orimode
```

- **GPS** → `NavSatFix`: WGS-84 lat/lon/alt, fix status from `navstat` /
  `numsats`, diagonal covariance from `pos_accuracy`. GPS is global but noisy
  (meters); indoor AMRs replace it with landmarks.
- **IMU** → `Imu`: fused orientation (roll/pitch/yaw → quaternion), angular
  rates `(wx, wy, wz)`, accelerations `(ax, ay, az)` — all in the
  vehicle/IMU frame (x forward, y left, z up).
- Quaternion convention (matches the KITTI devkit):
  `R = Rz(yaw) · Ry(pitch) · Rx(roll)`.

### 2.5 Time synchronization (the algorithm card)

**Exact synchronizer** — `message_filters.TimeSynchronizer`:

```
Inputs:  streams S1..Sn with stamps t1..tn
Output:  tuple (m1..mn) where t1 == t2 == ... == tn (rospy.Time equality)
Complexity: O(1) amortized per message (queue of size Q)
Memory: O(Q · Σ payload size)
```

- ✅ Deterministic pairing, zero added latency.
- ❌ Drops everything when one stream misses a frame; requires identical stamps.

**Approximate synchronizer** — `ApproximateTimeSynchronizer`:

```
Output: tuple (m1..mn) where max(ti) - min(ti) <= slop
```

- ✅ Robust to jittery, independent sensor clocks (the real-world case).
- ❌ Approximate pairing (up to `slop` error) — must be < fusion tolerance.

**Why precision matters:** KITTI stamps carry nanoseconds. A `float64` at
epoch scale (~1.3e9 s) has ~0.5 µs resolution; two nodes converting the same
string could round to *different* values and exact sync would fail silently.
We therefore parse and keep timestamps as **integer nanoseconds** and convert
losslessly to `rospy.Time`. (This is a real bug class in industry — clock
precision — and why teams use integer/fixed-point time internally.)

### 2.6 Pacing (replay clock)

A dataset player must reproduce the sensor cadence (10 Hz) so downstream code
sees a realistic stream. `RatePacer` sleeps `(t_i − t_{i−1}) / rate` between
frames; `rate=0` plays as fast as possible (for quick tests), `rate=0.5` for
slow-motion debugging.

---

## 3. Industrial importance

- **Sensor drivers** are the first thing an autonomy team writes and the last
  thing they debug in the field; standardized message interfaces
  (`Image`, `PointCloud2`, `Imu`, `NavSatFix`) make drivers swappable.
- **Time sync** is what makes fusion legitimate — no sync, no fusion.
- **Calibration as a ROS service** (topics + TF) means every module reads the
  same numbers; warehouse fleets version per-robot calibration.
- **Deterministic replay** (player + bag converter) is how AV/AMR teams
  reproduce bugs offline — the same topics, the same stamps, forever.

---

## 4. Folder structure

```
src/adaptive_amr/
├── dataset_loader/
│   ├── src/dataset_loader/{kitti_parsers,pacing,player_utils}.py
│   ├── scripts/kitti_to_bag.py
│   ├── launch/kitti_player.launch
│   ├── config/dataset_loader.yaml
│   └── test/test_kitti_parsers.py
├── camera_node/   ├── lidar_node/   ├── gps_node/   ├── imu_node/
│   ├── scripts/<name>_node.py      # driver node
│   ├── launch/<name>_node.launch
│   └── config/<name>_node.yaml
├── calibration/   (calibration_node.py + launch + config)
├── time_sync/     (time_sync_node.py + launch + config)
├── adaptive_amr_msgs/              # custom msgs (Phase 4+)
├── launch/phase2_kitti_sensors.launch
└── rviz/kitti_sensors.rviz
```

## 5. Required packages

| Layer | Packages |
|---|---|
| ROS | `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `tf2_msgs`, `tf2_ros`, `message_filters`, `diagnostic_msgs`, `rosbag` |
| Python | `numpy`, `opencv-python` (decode images), `rospkg` |
| Tooling | `roslaunch`, `rostopic`, `rosbag`, `rviz`, `tf` |

## 6. ROS topics

| Topic | Type | Pub | Sub |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` (or CompressedImage) | camera_node | time_sync, Phase 3+ |
| `/camera/image_rect` | `sensor_msgs/Image` | camera_node | Phase 3 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | calibration | Phase 3+ |
| `/camera_00..03/camera_info` | `sensor_msgs/CameraInfo` | calibration | any |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | lidar_node | time_sync, Phase 3+ |
| `/imu/data` | `sensor_msgs/Imu` | imu_node | time_sync, Phase 5 |
| `/gps/fix` | `sensor_msgs/NavSatFix` | gps_node | time_sync, Phase 5 |
| `/time_sync/{image,points,imu,gps}` | same as inputs | time_sync | Phase 3+ |
| `/time_sync/statistics` | `diagnostic_msgs/DiagnosticArray` | time_sync | monitoring |
| `/tf_static` | `tf2_msgs/TFMessage` | calibration | all |

## 7. TF frames

```
map ──> odom ──> base_link ──> laser ──> camera_link ──> camera_optical_frame
                                      └──────> imu_link
```
All static in Phase 2 (identity `map→odom`, `odom→base_link` placeholders).
Dynamic `map→odom` arrives with Phase 5 localization.

## 8. Parameters & configuration

Every node reads **private parameters** loaded from its `config/*.yaml`
(`<rosparam command="load">`); dataset paths come from launch args
(default `$KITTI_ROOT` or `/data/kitti`). No hardcoded paths in code.

## 9. Python classes

| Class / function | Package | Purpose |
|---|---|---|
| `KittiPaths` | dataset_loader | on-disk layout of a drive |
| `KittiCalib` | dataset_loader | parse + query calibration |
| `OxtsSample` / `OxtsParser` | dataset_loader | GPS/IMU data |
| `RatePacer` | dataset_loader | replay pacing |
| `read_timestamps_ns`, `read_velodyne_bin` | dataset_loader | files → data |
| `build_*_msg` builders | dataset_loader | ROS message construction |
| `CameraNode`, `LidarNode`, `GpsNode`, `ImuNode` | driver packages | pacing loops + publish |
| `CalibrationNode` | calibration | CameraInfo + static TF |
| `TimeSyncNode` | time_sync | message_filters sync + diagnostics |

## 10. Implementation walkthrough

1. **`kitti_parsers.py`** — pure Python (no ROS imports): timestamp parsing to
   integer ns, calibration key-value parsing with shape inference (12→3×4,
   9→3×3, 5→D, 2→S), OXTS line parsing with 30-field validation, velodyne
   reader, quaternion/Euler and ECEF/ENU helpers.
2. **`pacing.py` / `player_utils.py`** — `RatePacer` (pure); ROS message
   builders shared by drivers *and* the bag tool (no duplicated code).
3. **Driver nodes** — each is `config → dataset → pace → publish`, with
   throttle-logged missing-file handling and clean shutdown.
4. **`calibration_node.py`** — publishes latched `CameraInfo` ×4 (+ alias),
   broadcasts 6 static transforms via `StaticTransformBroadcaster`.
5. **`time_sync_node.py`** — exact/approx synchronizer, republish, 1 Hz
   diagnostics (`DiagnosticArray`).
6. **`kitti_to_bag.py`** — offline converter reusing the same parsers +
   builders → self-contained bag for later replay/debug.

## 11. Launch files

```bash
roslaunch adaptive_amr phase2_kitti_sensors.launch            # everything
roslaunch adaptive_amr phase2_kitti_sensors.launch rate:=0    # fast replay
roslaunch adaptive_amr phase2_kitti_sensors.launch sync_type:=approx
roslaunch dataset_loader kitti_player.launch                  # just drivers
```

## 12. RViz configuration

```bash
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/kitti_sensors.rviz
```
Shows: KITTI camera (2D + 3D Camera display), velodyne point cloud
(Intensity coloring), TF frames/axes, grid. Fixed frame: `map`.

## 13. Testing procedure

```bash
# 1) Unit tests (no ROS needed)
python3 src/adaptive_amr/dataset_loader/test/test_kitti_parsers.py   # 21 OK

# 2) Full sensor stack
roslaunch adaptive_amr phase2_kitti_sensors.launch rate:=2 &

# 3) Verify topics, rates, content
rostopic list | grep -E "camera|velodyne|imu|gps|time_sync|tf"
rostopic hz /camera/image_raw /velodyne_points /imu/data /gps/fix
rostopic echo -n1 /camera/camera_info | grep -E "width|height|P:"
rostopic echo -n1 /time_sync/statistics
rosrun tf view_frames && evince frames.pdf

# 4) RViz
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/kitti_sensors.rviz
```

## 14. Expected outputs

- 10 Hz on all four sensor topics; stamps equal to `timestamps.txt`.
- `time_sync/statistics`: `level=OK`, `synced_sets` increasing,
  `max_set_latency_s=0.000000` (exact mode).
- `camera_info`: 1242×376, P with baseline term −3.9e2 (cam 02).
- TF tree matches §7 (check `tf view_frames`).

## 15. Performance metrics

| Metric | Target |
|---|---|
| Image stream | ~14 MB/s @10 Hz (1241×376 bgr8) |
| Point cloud | zero-copy; ~1.5 MB/scan @10 Hz |
| Parser latency | < 1 ms/frame |
| Time sync latency | 0 s (exact), ≤ slop (approx) |
| Node count | 6 nodes, one `roscore` |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| No topics at all | roscore / launch issue | `roscore`; `roslaunch --screen` |
| Missing camera images | bad path | `roslaunch ... dataset_root:=<abs path>`; check `logs/` |
| `No module named cv2` | OpenCV absent | install `python3-opencv` or use `encoding:=compressed` |
| `time_sync` no output | stamps differ | use `sync_type:=approx`; check `/time_sync/statistics` |
| TF frames missing in RViz | calibration not running | launch `calibration.launch` |
| Fast replay still slow | disk I/O | `rate:=0`; use SSD |
| Images show wrong color | BGR vs RGB | `image_raw` is bgr8 (ROS convention) |

## 17. Common errors

1. `ValueError: OXTS line must contain 30 values` → corrupted/partial oxts
   file; re-download drive.
2. `Missing key 'P_rect_01'` → calib zip missing/old; re-run step 04.
3. Exact sync outputs nothing → `header.stamp` mismatch (float rounding in a
   custom node) — always use `read_timestamps_ns` + `ns_to_ros_time`.
4. `rostopic hz` shows 5 Hz → two cameras publishing to the same topic or
   `rate_factor` misconfigured.
5. RViz point cloud misplaced → TF tree wrong; verify with `tf_echo laser
   camera_optical_frame`.

## 18. Improvements

- `/clock` (sim time) support so `rosbag play`-style time control works.
- Camera rectification/undistortion in `camera_processing` (Phase 3).
- 100 Hz IMU interpolation from unsynchronized OXTS data.
- `adaptive_amr_msgs` sync-set message bundling all modalities (reduces
  per-topic overhead).
- Per-frame diagnostics with `diagnostic_aggregator` integration.

## 19. Interview questions

1. Why must sensor timestamps be synchronized before fusion, and what happens
   if they are 100 ms off at 10 Hz? *(one full frame of error)*
2. Exact vs approximate synchronization — when is each correct?
3. Why integer nanoseconds instead of float seconds for stamps?
4. Derive `T_imu_cam0` from `T_velo_cam0` and `T_imu_velo`. *(composition)*
5. What is the difference between `tf` and `tf2`? What is a latched topic and
   why are `CameraInfo` / `/tf_static` latched?
6. Why is the camera frame `camera_optical_frame` different from a body frame?
7. What does `rostopic hz` measure, and what would 5 Hz (vs 10) indicate?
8. How would you replay a rosbag at 2× speed? How would you test a bug that
   only appears with real-time cadence?
9. Why does `NavSatFix` carry a covariance matrix?
10. What are the trade-offs of publishing images as `CompressedImage` vs raw?
11. How does `message_filters` know two messages match? (stamp equality / slop)
12. Describe the zero-copy path for velodyne `.bin` → `PointCloud2`.
13. What is the KITTI baseline term in `P_rect_02` (−3.9e2) and what is it for?
14. Why does the player need a pacer at all, if we could publish instantly?
15. How would you detect clock skew between a camera and a LiDAR in a real robot?

## 20. Git commits for this phase

```bash
feat(dataset_loader): add KITTI parsers, message builders and bag converter
feat(camera_node): add KITTI camera driver node
feat(lidar_node): add KITTI Velodyne driver node
feat(gps_node): add KITTI GPS driver node (NavSatFix)
feat(imu_node): add KITTI IMU driver node (sensor_msgs/Imu)
feat(calibration): publish KITTI CameraInfo and static TF tree
feat(time_sync): add exact/approximate multi-sensor synchronization node
docs(kittiraith_ws): add phase 2 documentation, smoke tests and RViz config
```

---

**Phase 2 complete. Next: Phase 3 — Camera Pipeline, LiDAR Pipeline & Fusion.**
