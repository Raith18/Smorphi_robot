# Glossary — kittiraith_ws

Terminology used throughout the project, mapped to where it appears in code.

## Sensors & data

| Term | Meaning | Where |
|---|---|---|
| KITTI `_sync` drive | Time-synchronized raw drive: stereo images + velodyne + oxts share one timestamp per frame | `dataset_loader` |
| OXTS | KITTI's combined GPS/IMU logger (30 values per line) | `kitti_parsers.OxtsSample` |
| `oxts` | The GPS/IMU folder of a KITTI drive | `KittiPaths` |
| Velodyne `.bin` | Raw HDL-64E scan: `[x, y, z, reflectance]` float32 | `read_velodyne_bin` |
| `timestamps.txt` | ISO-8601 UTC stamp per frame | `parse_kitti_timestamp_ns` |

## Geometry & frames

| Term | Meaning | Where |
|---|---|---|
| Transform `T_a_b` | 4×4 rigid map from frame b to frame a | `invert_pose`, `compose_poses` |
| TF tree | Parent→child frame graph (`map→odom→base_link→…`) | `calibration` |
| `map` frame | Global reference (localization + maps) | `localization`, `occupancy_grid` |
| `odom` frame | Odometry reference (drift-free-ish local) | `lidar_odometry`, `visual_odometry` |
| `camera_optical_frame` | Camera frame: x right, y down, z forward | `calibration` |
| Rectification | Rotating stereo pairs so epipolar lines are horizontal | `camera_processing.rectify` |
| DLT triangulation | Linear (SVD) stereo point reconstruction | `geometry_utils.triangulate_many` |
| PnP | Perspective-n-Point pose estimation (RANSAC-robust) | `StereoVisualOdometry` |

## Algorithms

| Term | Meaning | Where |
|---|---|---|
| RANSAC | Robust model fitting by random sampling + inlier counting | `ground_segmentation` |
| Voxel grid | Uniform downsampling by grid quantization + centroid | `filters.VoxelGrid` |
| Euclidean clustering | Radius-based grouping via KD-tree BFS | `clustering` |
| OBB (PCA) | Oriented bounding box from covariance eigenvectors | `clustering.OrientedBoundingBox` |
| ICP | Iterative Closest Point registration (pt-to-pt / pt-to-plane) | `lidar_odometry.icp` |
| Kabsch | SVD closed-form rotation+translation alignment | `icp._kabsch` |
| Scan-to-map ICP | Register each scan against a global map | `localization.icp_localizer` |
| Log-odds occupancy | `l ← l + l_occ/l_free`, clamped; p = σ(l) | `grid_mapping` |
| Bresenham | Integer-only line rasterization | `grid_mapping.bresenham` |
| EDT | Euclidean distance transform (nearest-valid fill / inflation) | `depth_completion`, `costmap` |
| A* | Optimal grid search with admissible heuristic | `planner.AStarPlanner` |
| Kalman filter | Recursive state estimator (predict/update) | `sort.KalmanBoxFilter`, `predictor` |
| SORT | Tracking = Kalman + Hungarian IoU association | `object_tracking.sort` |
| MOTA/MOTP | CLEAR multi-object tracking metrics | `evaluation.metrics` |
| ATE/RPE | Absolute/Relative Trajectory Error | `evaluation.metrics` |
| Umeyama | Least-squares similarity alignment (ATE pre-step) | `evaluation.metrics` |
| Frustum fusion | Median LiDAR depth inside a 2D box | `detection_utils` |

## ROS concepts

| Term | Meaning | Where |
|---|---|---|
| Topic | Typed pub/sub bus | everywhere |
| Service | Request/reply (e.g. `Relocalize`, `SetGoal`) | `adaptive_amr_msgs/srv` |
| Message | Typed data structure | `adaptive_amr_msgs/msg` |
| Latched topic | Replays last message to new subscribers (`/tf_static`, maps) | `calibration`, `occupancy_grid` |
| `DiagnosticArray` | Standard health/statistics messages | every node's `/statistics` |
| `rosparam` | Parameter server; YAML loaded at launch | each `config/*.yaml` |
| Catkin metapackage | Aggregates packages without code | `adaptive_amr` |

## Performance & testing

| Term | Meaning | Where |
|---|---|---|
| Pure algorithm core | Module with no ROS imports (unit-testable anywhere) | every `src/<pkg>/…` |
| Unit test | One algorithm, one behavior | `test/test_*.py` |
| Integration test | Modules chained like the ROS graph (headless) | `tests/integration/` |
| Smoke test | Phase deliverable validation | `tests/smoke_test_phase*.sh` |
| `verify_all.sh` | One-command full verification | `scripts/verify_all.sh` |
| CI | GitHub Actions (lint + tests + docker) | `.github/workflows/` |
