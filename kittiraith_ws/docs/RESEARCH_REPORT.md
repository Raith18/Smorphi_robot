# Adaptive Multi-Sensor Perception, Localization and Semantic Mapping Framework for Autonomous Mobile Robots

**A KITTI-Dataset Replay Study on ROS Noetic / Ubuntu 20.04**

> **Research-style technical report — kittiraith_ws · Raith18 · 2026**
> This report accompanies a production-style, modular autonomy stack
> (22 ROS packages, ~180 unit tests, an end-to-end integration test, CI/CD,
> Docker, and a measured CPU performance profile). It is written in the style
> of an industrial research report so it can be reused as a thesis appendix,
> portfolio piece, or engineering design document.

---

## Abstract

Autonomous Mobile Robots (AMRs) in warehouses and logistics must perceive
their environment through complementary sensors, localize against a map,
understand semantics, and plan safe motion — all on constrained, often
GPU-less hardware. This work presents a complete, modular autonomy framework
that replays the KITTI dataset through an industrial-style ROS Noetic
pipeline: synchronized multi-sensor drivers, camera and LiDAR processing,
camera–LiDAR fusion, YOLOv8-based detection and segmentation, SORT-based
multi-object tracking, sparse-to-dense depth completion, stereo visual
odometry, ICP LiDAR odometry, ICP scan-to-map localization (no EKF), semantic
and occupancy mapping, constant-velocity motion prediction, and an A*/costmap
navigation layer with a safety behavior state machine. Every algorithm is
implemented as a pure-Python, unit-tested core; the full stack is validated
by 180+ unit tests, a headless end-to-end integration test, CI/CD, and a
measured CPU latency profile that drove one 19× occupancy-grid optimization
and exposed two integration bugs. The framework is designed for education and
as a drop-in baseline for research on camera–LiDAR fusion, SLAM, and
navigation for warehouse robots.

**Keywords:** autonomous mobile robot, sensor fusion, LiDAR, camera, SLAM,
occupancy grid, object detection, tracking, ROS Noetic, KITTI.

---

## 1. Introduction

### 1.1 Motivation

Warehouse automation companies (Amazon Robotics, GreyOrange, Geek+, Locus,
Addverb, …) deploy fleets of AMRs that must operate reliably indoors, around
people, in changing lighting, and on limited onboard compute. The de-facto
industrial pattern is a **layered autonomy stack**:

```
sensors → drivers → time sync → calibration → perception → odometry/SLAM →
localization → mapping → prediction → navigation → behavior
```

Each layer is a reusable module communicating through standardized ROS topics,
so any layer can be replaced, replayed (rosbag), or unit-tested in isolation.

### 1.2 Why KITTI

The KITTI dataset [1] is the de-facto benchmark for autonomous driving and
mobile robotics research: 10 Hz stereo cameras, a 64-beam Velodyne LiDAR, and
a GPS/IMU unit, all time-synchronized and calibrated, with ground truth for
odometry, detection, tracking, and semantics. Replaying KITTI lets every
module be validated quantitatively against ground truth.

### 1.3 Contributions

1. A **22-package, modular ROS Noetic framework** whose every algorithm core
   is pure Python and unit-tested (no GPU required).
2. An **ICP-based localization** pipeline (point-to-plane scan-to-map, no EKF)
   with a dynamic `map→odom` transform.
3. A **perception stack** (YOLOv8 detection/segmentation, frustum fusion,
   SORT tracking, EDT depth completion) that runs on CPU.
4. A **navigation layer** (costmap inflation + A* + safety state machine)
   producing `move_base`-compatible outputs.
5. A **measurement-driven optimization** story: a 19× occupancy-grid speedup
   and two real bugs found by integration testing.
6. **CI/CD, Docker, and 180+ tests** making the stack reproducible and
   regression-safe.

---

## 2. Related Work

| Area | Representative systems | Relationship to this work |
|---|---|---|
| LiDAR odometry | LOAM [2], F-LOAM, FAST-LIO [3], KISS-ICP | This work implements the classic point-to-plane ICP primitive those systems build on, in pure Python; the topics/interface are FAST-LIO-compatible for a drop-in C++ upgrade |
| Visual odometry | viso2, ORB-SLAM2 [4], VINS-Fusion [5] | We implement the classic stereo front-end (KLT + DLT triangulation + PnP) |
| Localization | NDT localizer (autoware), AMCL, ICP scan-to-map | We implement scan-to-map ICP (no EKF), deterministic and tunable-free |
| Detection/Segmentation | YOLO family [6], Mask R-CNN, DeepLab | YOLOv8 / YOLOv8-seg (nano) chosen for CPU real-time |
| Tracking | SORT [7], DeepSORT, ByteTrack | SORT as the standard baseline; ByteTrack as future work |
| Mapping | OctoMap, gmapping, costmap_2d [8] | Log-odds occupancy grid (Bresenham/vectorized rays) + semantic voxel map |
| Motion prediction | Constant-velocity, Trajectron++ [9] | CV-Kalman baseline with collision-risk output |

---

## 3. System Architecture

### 3.1 Package map (22 packages)

```
dataset_loader   camera_node   lidar_node   gps_node   imu_node      (Phase 2)
time_sync        calibration   adaptive_amr_msgs                       (Phase 2)
camera_processing  lidar_processing  sensor_fusion                     (Phase 3)
object_detection  semantic_segmentation  object_tracking  depth_estimation (Phase 4)
visual_odometry  lidar_odometry  localization                          (Phase 5)
semantic_mapping  occupancy_grid  motion_prediction                    (Phase 6)
navigation_layer                                                       (Phase 7)
evaluation                                                             (Phase 8)
```

### 3.2 Data flow

```
KITTI files
  → camera_node /camera/image_raw · lidar_node /velodyne_points
  → imu_node /imu/data · gps_node /gps/fix · calibration /tf_static + CameraInfo
  → time_sync → camera_processing (/camera/image_rect) · lidar_processing (ground/obstacles/clusters)
  → sensor_fusion (/fusion/{colored_points,sparse_depth,overlay})
  → object_detection (/object_detections) · object_tracking (/object_tracks)
  → semantic_segmentation (/semantic_map/*) · depth_estimation (/depth/dense)
  → visual_odometry (/visual_odometry) · lidar_odometry (/lidar_odometry)
  → localization (/localization_pose, map→odom TF)
  → semantic_mapping (/semantic_map) · occupancy_grid (/occupancy_grid)
  → motion_prediction (/dynamic_obstacles)
  → navigation_layer (/navigation_costmap, /planned_path)
  → behavior (/behavior/state, /behavior/velocity_command)
```

### 3.3 TF tree

```
map ──(dynamic, ICP correction)──▶ odom ──(dynamic, odometry)──▶ base_link
  └──▶ laser ──▶ camera_link ──▶ camera_optical_frame
                    └──▶ imu_link
```

### 3.4 Design principles

1. One module = one ROS package; communication only through topics/services.
2. Configuration in YAML, never hardcoded paths.
3. Every algorithm core is pure Python (no ROS imports) → unit-testable and
   CI-friendly without roscore.
4. Deterministic replay: the KITTI player paces every sensor from one
   timestamps file, and all stamps are integer nanoseconds (lossless).
5. Measurement over assumption: every module ships diagnostics + a benchmark.

---

## 4. Methodology (per module, with mathematics)

### 4.1 Sensor layer & calibration

- **Timestamps** parsed to integer nanoseconds; `rospy.Time(secs, nsecs)`
  conversion is exact — float64 at epoch scale has ~0.5 µs resolution and
  would break exact synchronization.
- **Calibration** serves `CameraInfo` (rectified `P_rect`, `R_rect`, `K`, `D`)
  and the static TF tree; `T_imu_cam = T_velo_cam · T_imu_velo` chains the
  KITTI extrinsics.
- **Time sync**: `message_filters` exact (KITTI `_sync` drives) or
  approximate (real robots), with `DiagnosticArray` health telemetry.

### 4.2 Camera pipeline

Pinhole + plumb-bob distortion; rectification maps built per output pixel:
```
x = (u − c'x)/f'x ,  y = (v − c'y)/f'y
p_cam = R_rectᵀ·[x,y,1]
x_d = x(1 + k₁r² + k₂r⁴ + k₃r⁶) + 2p₁xy + p₂(r²+2x²)   (same for y)
u_src = fx·x_d + cx
```
Validated pixel-for-pixel against `cv2.initUndistortRectifyMap`.
Crop/resize adjusts `P`/`K`: `f′=s·f, c′=s·(c−crop), t′=s·t`.

### 4.3 LiDAR pipeline

- **Ground segmentation**: RANSAC plane `n·p + d = 0` (upward normal
  canonicalized), SVD refit.
- **Voxel downsampling**: `floor(p/leaf)` per-cell centroid, O(n).
- **Clustering**: KD-tree + radius BFS (PCL-style Euclidean extraction).
- **Bounding boxes**: PCA axes + min/max projections (tight boxes).

### 4.4 Camera–LiDAR fusion

```
p_img = P_rect · R_rect4x4 · T_velo_cam · p_velo
u = x/z, v = y/z, depth = z
```
Outputs: colored LiDAR (RGB8), sparse depth image (32FC1), overlay.

### 4.5 Detection, segmentation, tracking, depth

- **YOLOv8n / YOLOv8n-seg** (anchor-free single-stage CNN), CPU at 10 Hz.
- **Frustum fusion**: median depth of projected points inside each box →
  back-project the box center to the laser frame.
- **SORT**: per-track Kalman `x=[x,y,s,r,vx,vy,vs]` with predict
  `x'=Fx, P'=FPFᵀ+Q` and update `K=PHᵀ(HPHᵀ+R)⁻¹`; Hungarian association on
  IoU; lifecycle `max_age`/`min_hits`.
- **Depth completion**: nearest-valid-pixel fill via the Euclidean distance
  transform (O(n)) + Gaussian smoothing.

### 4.6 Odometry & localization

- **Stereo VO**: Shi-Tomasi → KLT (temporal + stereo) → DLT triangulation →
  PnP (RANSAC) → accumulate.
- **LiDAR odometry**: point-to-plane ICP (default) / point-to-point (Kabsch):
  `min Σ(nᵢ·(R sᵢ + t − tᵢ))²` solved by the linearized 6-DOF least squares.
- **Localization (no EKF)**: scan-to-map point-to-plane ICP; the map is built
  in the odom frame from odometry poses; `map→odom = T_corr` is the dynamic
  TF; a `max_translation_jump` guard rejects implausible corrections;
  `/localization/relocalize` service handles kidnapping.

### 4.7 Mapping & prediction

- **Semantic map**: `p_map = T_map_base·p_base`, voxel merge with dominant
  color; auto-coarsening bounds memory.
- **Occupancy grid**: log-odds `l ← l + l_occ/l_free` (clamped), vectorized
  ray sampling with per-ray endpoint guarantees; `p = 1 − 1/(1+e^l)` → 0..100.
- **Motion prediction**: CV-Kalman per track; `p(τ) = p0 + v·τ`;
  `collision_risk = clip(1 − d_min/safe, 0, 1)`.

### 4.8 Navigation & behavior

- **Costmap inflation**: `cost = 100·(1 − d/R_inflate)` via EDT.
- **A\***: 8-connectivity, Euclidean heuristic (admissible → optimal),
  min-heap; unknown cells penalized.
- **Behavior state machine**: `INIT → NAVIGATE ⇄ AVOID ⇄ NAVIGATE`,
  `→ STOP → RESUME → NAVIGATE → GOAL_REACHED`; decision layer emits
  `(v, ω)` with a heading-error P-controller.

### 4.9 Evaluation

ATE (Umeyama-aligned RMSE), RPE (KITTI relative pose error), MOTA/MOTP
(CLEAR MOT), mIoU, grid precision/recall — all pure NumPy and unit-tested.

---

## 5. Experimental Setup & Results

### 5.1 Setup

- Host (this development sandbox): Debian 12 VM, 2 vCPUs, no GPU — the
  **worst-case** CPU envelope. The project targets Ubuntu 20.04 / ROS Noetic /
  Python 3.8 (CI runs on 3.8).
- Dataset: KITTI raw drive `2011_09_26_drive_0005` (sample downloader) +
  KITTI odometry sequences 00–10 for ground truth (benchmark runner).

### 5.2 Unit & integration testing

- **180+ unit tests** across all 22 packages (pure algorithm cores).
- **Headless integration test**: a synthetic box world with a pillar; the
  robot drives through it while the real modules run in sequence
  (LiDAR → grid → costmap/A* → behavior → semantic map → prediction).
  Asserts: pillar region occupied, walls mapped, path free of the robot's
  own route, A* reaches the goal, behavior reaches GOAL_REACHED, map grows,
  predictions extrapolate.

### 5.3 Measured CPU latency (median, 3 repeats, this 2-core VM)

| module / algorithm | latency [ms] |
|---|---|
| motion_prediction_20_tracks | 1.2 |
| sort_update_10_tracks | 0.6 |
| astar_300x300 | 2.7 |
| costmap_inflation_300x300 | 2.4 |
| projection_100k | 7.5 |
| ransac_ground_30k | 24.9 |
| depth_completion_1242x376 | 35.5 |
| semantic_map_merge_20k | 48.3 |
| voxel_downsample_100k | 110.0 |
| euclidean_cluster_20k | 149.5 |
| icp_point_to_plane_20k | 226.3 |
| occupancy_raycast_5k (realistic) | 94.3 |

> Real desktop/server CPUs are typically 5–10× faster than this VM; the
> numbers are intended as a reproducible relative baseline.

### 5.4 The optimization study

| State | occupancy ray-cast, 20k pts | speedup |
|---|---|---|
| Original (per-point Bresenham loop) | 7629.9 ms | — |
| Vectorized ray sampling | 676.0 ms | 11× |
| `np.add.at` → `np.bincount` | 401.0 ms | 19× total |
| Realistic input (5k pts, 60 m grid) | 94.3 ms | ≈ 11 Hz feasible |

### 5.5 Bugs found by integration testing (post-mortems)

1. **Vectorized ray-cast endpoint regression** (Phase 8 rewrite): a global
   `linspace(0,1,max_n)` left short rays short of their endpoints → every
   obstacle closer than the farthest silently lost its occupied hit. Fix:
   per-ray sampling `t = j/(nᵢ−1)`. *Only an end-to-end test with a close
   obstacle could catch this — unit tests on single points could not.*
2. **Occupancy node frame mismatch**: obstacles (laser frame) were ray-cast
   against the robot pose (map frame), smearing the grid under motion. Fix:
   TF `laser→map` transform before ray-casting.
3. **Thin-obstacle washout** (insight, not a bug): free rays through a sparse
   obstacle can erase its occupied hits; mitigated by occlusion-aware sensor
   modeling and `l_occ`/`l_free` tuning.

---

## 6. Discussion

- **CPU-first design**: the choice of YOLOv8n, pure-Python ICP, EDT depth
  completion, and CV-Kalman prediction keeps the whole stack real-time on
  ~2 GB shared graphics hardware; the measured worst-case per-frame cost
  (≈ 700 ms worst module on a 2-core VM, ≈ 100 ms at realistic scales) fits
  the 10 Hz replay cadence on real hardware.
- **ICP vs EKF localization**: scan-to-map ICP needs no tuned noise models and
  is deterministic; the cost is sensitivity to local minima, mitigated by
  warm starts, correspondence thresholds, and a jump guard. An EKF or
  FAST-LIO could replace the layer without changing the interface.
- **Testability by construction**: separating pure algorithms from ROS nodes
  is what makes 180+ tests and CI feasible without roscore or a GPU.
- **Limitations**: SORT loses IDs on long occlusions (ByteTrack/DeepSORT as
  follow-up); CV prediction is naive for turning pedestrians (learned
  predictors as follow-up); ICP odometry drifts without loop closure.

---

## 7. Future Work

1. **FAST-LIO / LOAM** drop-in (C++) — same topics, real-time LiDAR-inertial
   odometry.
2. **Loop closure + pose-graph optimization** for drift-free SLAM.
3. **ByteTrack / DeepSORT** with appearance re-identification.
4. **Learned depth completion and motion prediction** (KBNet, Trajectron++)
   once a GPU is available.
5. **OctoMap** 3D occupancy and semantic costmap layers.
6. **DWA/TEB local planner** beneath A* for dynamic avoidance.
7. **Full KITTI odometry leaderboard harness** (all 11 sequences, RPE table).
8. **ROS 2 (Humble) port** of the message interfaces and nodes.

---

## 8. Conclusion

This work demonstrates a complete, modular, CPU-friendly autonomy stack for
AMRs on the KITTI dataset: from synchronized sensors through perception,
odometry, ICP localization, semantic and occupancy mapping, motion prediction,
and navigation, all validated by 180+ unit tests, an end-to-end integration
test, CI/CD, and a measured performance profile that drove one 19×
optimization and surfaced two real integration bugs. The framework is
deliberately educational in structure (pure-Python algorithm cores, full
documentation per module) while industrial in discipline (packages, topics,
YAML config, diagnostics, benchmarking, CI). It is a reproducible baseline
for research and teaching in multi-sensor perception, localization, and
semantic mapping for warehouse robots.

---

## 9. References

1. A. Geiger, P. Lenz, R. Urtasun, "Are we ready for Autonomous Driving? The
   KITTI Vision Benchmark Suite," CVPR 2012.
2. J. Zhang, S. Singh, "LOAM: Lidar Odometry and Mapping in Real-time," RSS
   2014.
3. W. Xu, F. Zhang, "FAST-LIO: A Fast, Robust LiDAR-inertial Odometry
   Package," ICRA 2022.
4. R. Mur-Artal, J. D. Tardós, "ORB-SLAM2," IEEE Trans. Robotics 2017.
5. T. Qin, P. Li, S. Shen, "VINS-Mono," IEEE Trans. Robotics 2018.
6. G. Jocher et al., "Ultralytics YOLOv8," 2023.
7. A. Bewley, Z. Ge, L. Ott, F. Ramos, B. Upcroft, "Simple Online and
   Realtime Tracking," ICIP 2016.
8. D. Lu, W. Smart, "Costmap2D," ROS navigation stack.
9. B. Ivanovic, M. Pavone, "The Trajectron: Probabilistic Multi-Agent
   Trajectory Modeling," NeurIPS 2019.

---

## Appendix A — Module map

| Package | Phase | Responsibility | Key outputs |
|---|---|---|---|
| dataset_loader | 2 | KITTI parsers, pacing, bag tool | parsers, `kitti_to_bag` |
| camera_node | 2 | camera driver | `/camera/image_raw` |
| lidar_node | 2 | LiDAR driver | `/velodyne_points` |
| gps_node | 2 | GPS driver | `/gps/fix` |
| imu_node | 2 | IMU driver | `/imu/data` |
| time_sync | 2 | exact/approx sync | `/time_sync/*` |
| calibration | 2 | CameraInfo + static TF | `/camera*/camera_info`, `/tf_static` |
| camera_processing | 3 | rectify/undistort | `/camera/image_rect` |
| lidar_processing | 3 | ground/cluster/boxes | `/lidar_processing/*` |
| sensor_fusion | 3 | projection fusion | `/fusion/*` |
| object_detection | 4 | YOLOv8 + frustum | `/object_detections` |
| object_tracking | 4 | SORT | `/object_tracks` |
| semantic_segmentation | 4 | YOLOv8-seg + painting | `/semantic_map/labels...` |
| depth_estimation | 4 | EDT completion | `/depth/dense` |
| visual_odometry | 5 | stereo VO | `/visual_odometry` |
| lidar_odometry | 5 | ICP odometry | `/lidar_odometry` |
| localization | 5 | ICP scan-to-map | `/localization_pose`, `map→odom` |
| semantic_mapping | 6 | global semantic map | `/semantic_map` |
| occupancy_grid | 6 | log-odds grid | `/occupancy_grid` |
| motion_prediction | 6 | trajectory prediction | `/dynamic_obstacles` |
| navigation_layer | 7 | costmap + A* + behavior | `/navigation_costmap`, `/planned_path`, `/behavior/*` |
| evaluation | 8 | metrics + profiling | report, CSV |

## Appendix B — Topic inventory

See `docs/architecture_overview.md` (ROS topic bus) and each package README.

## Appendix C — Test & tooling inventory

- 19 unit-test files, ~180 tests (pure algorithm cores).
- 1 headless integration test (end-to-end pipeline).
- 9 phase smoke tests (`tests/smoke_test_phase*.sh`).
- `scripts/verify_all.sh` — one-command full verification.
- `.github/workflows/ci.yml` + `docker.yml` — CI/CD.
- `docker/` — ROS Noetic development image + compose.
- `docs/api_reference.md` — machine-generated API reference.
