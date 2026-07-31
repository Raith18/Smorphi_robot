# Module Reference — the 25-point spec for all 22 packages

Every module follows the project's 25-point specification. This document
consolidates them; the authoritative per-module detail lives in each
package's `README.md`.

**The 25 points:** 1 Objective · 2 Theory · 3 Industrial Importance ·
4 Folder Structure · 5 Required Packages · 6 ROS Topics · 7 Publishers ·
8 Subscribers · 9 Services · 10 Messages · 11 TF Frames · 12 Parameters ·
13 Configuration Files · 14 Python Classes · 15 Complete Source Code ·
16 Launch Files · 17 Testing Procedure · 18 RViz Configuration ·
19 Expected Outputs · 20 Performance Metrics · 21 Debugging Guide ·
22 Common Errors · 23 Improvements · 24 Git Commit Message · 25 Documentation

---

## Phase 2 — sensor layer

### dataset_loader
| # | Point |
|---|---|
| 1–3 | KITTI parsing infrastructure (calib, oxts, timestamps, velodyne), pacing, bag conversion. Pure parsers = testable anywhere. |
| 4–5 | `src/dataset_loader/{kitti_parsers,pacing,player_utils}.py`, `scripts/kitti_to_bag.py`, `launch/`, `test/` · rospy, sensor_msgs, rosbag, numpy, opencv |
| 6–10 | No own topics (builds others' messages) · messages: `CameraInfo`/`Imu`/`NavSatFix`/`PointCloud2` builders · no services |
| 11–13 | No TF · pacing in driver YAMLs · launch: `kitti_player.launch` |
| 14 | `KittiPaths`, `KittiCalib`, `KittiOdometryPaths`, `OxtsSample/Parser`, `RatePacer`, message builders |
| 16–19 | `kitti_player.launch`, `kitti_to_bag.py` · tests: `test_kitti_parsers.py` (21) · outputs: parsers/bag |
| 20 | Zero-copy PointCloud2; parsers < 1 ms/frame |
| 21–23 | Debug: unit tests isolate parser bugs · errors: bad calib keys, 30-value OXTS · improvements: streaming, odometry support |
| 24–25 | `feat(dataset_loader): …` · README ✅ |

### camera_node / lidar_node / gps_node / imu_node
| # | Point |
|---|---|
| 1–3 | KITTI sensor drivers publishing standard messages (`/camera/image_raw`, `/velodyne_points`, `/gps/fix`, `/imu/data`). |
| 4–5 | `scripts/<name>_node.py`, `launch/`, `config/` · rospy + sensor_msgs + dataset_loader |
| 6–10 | One topic each (publisher) · no services · standard msgs |
| 11–13 | Frames: `camera_optical_frame`, `laser`, `imu_link` · params in `<name>_node.yaml` |
| 14 | `CameraNode`, `LidarNode`, `GpsNode`, `ImuNode` |
| 16–19 | `<name>_node.launch` · test: `rostopic hz` (~10 Hz) · outputs: 10 Hz streams, exact stamps |
| 20 | ~14 MB/s camera; zero-copy LiDAR |
| 21–23 | Debug: missing-file warnings · errors: cv2 missing (compressed fallback) · improvements: stereo, sim-time |
| 24–25 | `feat(camera_node): …` etc. · READMEs ✅ |

### time_sync
| # | Point |
|---|---|
| 1–3 | Exact/approximate multi-sensor synchronization + diagnostics. |
| 4–5 | `scripts/time_sync_node.py`, `launch/`, `config/`, `test/` · message_filters, diagnostic_msgs |
| 6–10 | `/time_sync/{image,points,imu,gps,statistics}` (pub) · 4 sensor topics (sub) |
| 11–13 | No new TF · `sync_type`, `queue_size`, `approx_slop` in YAML |
| 14 | `TimeSyncNode` |
| 16–19 | `time_sync.launch` · test: `rostopic echo /time_sync/statistics` (latency 0) |
| 20 | Zero added latency (exact); O(Q) memory |
| 21–23 | No output → stamps differ → approx mode · improvements: sim-time, clock-skew estimation |
| 24–25 | `feat(time_sync): …` · README ✅ |

### calibration
| # | Point |
|---|---|
| 1–3 | Publish CameraInfo for all cameras + the static TF tree from KITTI calib. |
| 4–5 | `scripts/calibration_node.py`, `launch/`, `config/` · tf2_ros, sensor_msgs |
| 6–10 | `/camera_00..03/camera_info`, `/camera/camera_info` (latched), `/tf_static` (pub) |
| 11–13 | TF: `map→odom→base_link→laser→camera_link→(camera_optical_frame, imu_link)` · frame names in YAML |
| 14 | `CalibrationNode` |
| 16–19 | `calibration.launch` · test: `tf_echo laser camera_optical_frame`, `view_frames` |
| 20 | Latched → zero steady-state bandwidth |
| 21–23 | TF missing → run calibration first · errors: wrong calib files for date |
| 24–25 | `feat(calibration): …` · README ✅ |

## Phase 3 — pipelines

### camera_processing
| # | Point |
|---|---|
| 1–3 | Rectify/undistort/color/crop/resize with CameraInfo consistency. |
| 4–5 | `src/camera_processing/rectify.py` (pure), node, launch, config, test (13) |
| 6–10 | `/camera/image_rect`, `/camera/camera_info_rect` (pub) · raw image + ci (sub) |
| 11–13 | Frames preserved · `rectify_mode`, `resize_scale`, `crop`, `output_encoding` in YAML |
| 14 | `RectifyMapper`, `build_rectify_maps_numpy`, `adjust_projection_matrix`, … |
| 16–19 | `camera_processing.launch` · tests · outputs: rectified image + adjusted ci |
| 20 | cv2 remap 2–5 ms/frame |
| 21–23 | Input size mismatch → stale camera_info · improvements: stereo rectify, CUDA |
| 24–25 | `feat(camera_processing): …` · README ✅ |

### lidar_processing
| # | Point |
|---|---|
| 1–3 | Ground removal (RANSAC), voxel grid, Euclidean clustering, PCA boxes. |
| 4–5 | `src/lidar_processing/{filters,ground_segmentation,clustering}.py`, node, config, test (13) |
| 6–10 | `/lidar_processing/{ground,obstacles,clusters,boxes,statistics}` (pub) · `/velodyne_points` (sub) |
| 11–13 | No new TF · thresholds in YAML |
| 14 | `PassthroughFilter`, `VoxelGrid`, `RansacPlaneSegmenter`, `GroundRemover`, `EuclideanClusterExtraction`, `OrientedBoundingBox` |
| 16–19 | `lidar_processing.launch` · tests · outputs: ground/obstacles/clusters/boxes |
| 20 | RANSAC 5–15 ms; clustering 20–60 ms @100k |
| 21–23 | All ground → normal flip/threshold · errors: scipy missing · improvements: LineFit ground, L-shape boxes |
| 24–25 | `feat(lidar_processing): …` · README ✅ |

### sensor_fusion
| # | Point |
|---|---|
| 1–3 | Project LiDAR into the camera: colored cloud, sparse depth, overlay. |
| 4–5 | `src/sensor_fusion/projection.py`, node, config, test (8) |
| 6–10 | `/fusion/{colored_points,sparse_depth,overlay,statistics}` (pub) · image+scan+ci (sub) |
| 11–13 | Reads `laser→camera_optical_frame` · `use_tf`, `load_calibration_from_dataset` in YAML |
| 14 | `LidarCameraProjection` |
| 16–19 | `sensor_fusion.launch` · tests · outputs: aligned colored cloud |
| 20 | Projection 100k < 1 ms |
| 21–23 | Misalignment → calib/TF · errors: TF lookup failed |
| 24–25 | `feat(sensor_fusion): …` · README ✅ |

## Phase 4 — perception

### object_detection / semantic_segmentation / object_tracking / depth_estimation
| # | Point |
|---|---|
| 1–3 | YOLOv8n detection + frustum fusion; YOLOv8n-seg semantics painted on LiDAR; SORT tracking; EDT depth completion. |
| 4–5 | Each: `src/<pkg>/…py` (pure), node, launch, config, test (13/7/13/10) |
| 6–10 | `/object_detections(+/image)`, `/object_tracks(+/markers)`, `/semantic_map/{labels,colored,colored_points}`, `/depth/{dense,colored}` · custom msgs `ObjectDetection(Array)`, `ObjectTrack(Array)` |
| 11–13 | Frames documented per node · model/conf/imgsz/device in YAML |
| 14 | `detection_utils`, `sort.SortTracker/KalmanBoxFilter`, `segmentation_utils`, `depth_completion` |
| 16–19 | `phase4_perception.launch` · tests · outputs: detections, tracks, semantics, dense depth |
| 20 | YOLOv8n 60–150 ms CPU; SORT < 1 ms; EDT 5–20 ms |
| 21–23 | No detections → conf/classes · ID swaps → SORT limit · errors: ultralytics missing, CPU memory |
| 24–25 | `feat(object_detection): …` etc. · READMEs ✅ |

## Phase 5 — odometry & localization

### visual_odometry / lidar_odometry / localization
| # | Point |
|---|---|
| 1–3 | Stereo VO (KLT+DLT+PnP); ICP odometry (pt-plane default); ICP scan-to-map localization (no EKF). |
| 4–5 | `src/<pkg>/…py`, node, launch, config, tests (8/6/4) |
| 6–10 | `/visual_odometry(+/path)`, `/lidar_odometry(+/path)`, `/localization_pose`, `/localization/map` · service `Relocalize` |
| 11–13 | Dynamic `odom→base_link` (odometry) and `map→odom` (localization) TF · params in YAML |
| 14 | `StereoVisualOdometry`, `geometry_utils`, `icp`, `IcpLocalizer` |
| 16–19 | `phase5_odometry.launch` · tests · outputs: two paths + refined pose on map |
| 20 | VO 30–80 ms; ICP 30–150 ms; scan-to-map 40–200 ms |
| 21–23 | TF double-publisher → one `enable_tf` · errors: too few points, mapping never ends |
| 24–25 | `feat(visual_odometry): …` etc. · READMEs ✅ |

## Phase 6 — mapping & prediction

### semantic_mapping / occupancy_grid / motion_prediction
| # | Point |
|---|---|
| 1–3 | Global semantic voxel map; log-odds ray-cast grid; CV-Kalman trajectory prediction. |
| 4–5 | `src/<pkg>/…py`, node, launch, config, tests (6/9/7) |
| 6–10 | `/semantic_map`, `/occupancy_grid`, `/dynamic_obstacles(+/markers)` · msgs `DynamicObstacle(Array)` · service `/semantic_map/clear` |
| 11–13 | Maps in `map` frame; prediction in `laser` · params in YAML |
| 14 | `SemanticMapBuilder`, `OccupancyGridMapper`, `MotionPredictor` |
| 16–19 | `phase6_mapping.launch` · tests · outputs: colored map, 0..100 grid, trajectories |
| 20 | Merge 10–30 ms; ray-cast 5k ≈ 94 ms; prediction < 1 ms |
| 21–23 | Grid smear → frame/pose bug (fixed) · thin-obstacle washout → occlusion + tuning |
| 24–25 | `feat(semantic_mapping): …` etc. · READMEs ✅ |

## Phase 7 — navigation

### navigation_layer
| # | Point |
|---|---|
| 1–3 | Costmap inflation (EDT), A* planning, safety behavior state machine. |
| 4–5 | `src/navigation_layer/{costmap,planner,behavior}.py`, two nodes, launch, config, tests (6/5/12) |
| 6–10 | `/navigation_costmap`, `/planned_path`, `/behavior/{state,velocity_command}` · service `SetGoal` |
| 11–13 | All in `map` frame · `inflation_radius`, `risk_stop/avoid`, speeds in YAML |
| 14 | `CostmapBuilder`, `AStarPlanner`, `BehaviorStateMachine` |
| 16–19 | `phase7_navigation.launch` · tests · outputs: inflated costmap, A* path, gated (v,ω) |
| 20 | Inflation 5–20 ms; A* 10–100 ms; behavior < 1 ms @10 Hz |
| 21–23 | No path → goal/TF · stuck STOP → risk source |
| 24–25 | `feat(navigation_layer): …` · README ✅ |

## Phase 8 — evaluation

### evaluation
| # | Point |
|---|---|
| 1–3 | KITTI metrics (ATE/RPE, MOTA/MOTP, mIoU, grid), CPU profiler, ROS perf collector. |
| 4–5 | `src/evaluation/metrics.py`, `scripts/{benchmark,profile,perf_collector}.py`, launch, config, tests (14+5) |
| 6–10 | `perf_collector` subscribes `*/statistics` · config presets (accuracy/balanced/fast) |
| 11–13 | n/a · benchmark.yaml, performance_presets.yaml |
| 14 | `umeyama_alignment`, `absolute_trajectory_error`, `relative_pose_error`, `mot_metrics`, `segmentation_iou`, `grid_precision_recall` |
| 16–19 | `perf_collector.launch` · tests · outputs: benchmark.md, profile table, CSV |
| 20 | CI ~3 min; verify_all ~35 s |
| 21–23 | ATE≈0 with scale → linear drift absorbed (report both) · ID swap = 2 IDSW |
| 24–25 | `feat(evaluation): …` · README ✅ |

---

## Coverage status

All 22 packages ship a README implementing the 25-point template (objective
through documentation), launch files, YAML config, tests, and (where
applicable) RViz configs. Machine-generated API docs: `docs/api_reference.md`.
