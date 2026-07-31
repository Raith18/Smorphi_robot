# Phase 5 — Visual Odometry · LiDAR Odometry · ICP Localization

> **Module status: ✅ COMPLETE**

Full tutorial: theory with mathematics, industrial context, every implemented
file, testing procedure, debugging, performance and interview questions.

---

## 1. Objective

| Module | Package | Algorithm (chosen per your constraints) |
|---|---|---|
| Visual odometry | `visual_odometry` | Classic **stereo VO**: Shi-Tomasi corners + KLT optical flow + DLT triangulation + PnP |
| LiDAR odometry | `lidar_odometry` | **ICP** scan-to-scan: point-to-point (Kabsch) and point-to-plane (default) |
| Localization | `localization` | **ICP scan-to-map** (point-to-plane) — **no EKF**, as requested |

**Definition of done:** `roslaunch adaptive_amr phase5_odometry.launch` produces
`/visual_odometry`, `/lidar_odometry`, `/localization_pose`, the dynamic
`map → odom` TF, and RViz shows the robot moving with a growing map.

## 2. Theory

### 2.1 Stereo visual odometry (the classic pipeline)

```
left_prev ──KLT──▶ left_cur       right_prev ──KLT──▶ right_cur
left_cur  ──KLT(stereo)──▶ right_cur
triangulate(left_cur, right_cur, P_left, P_right)  -> 3D points (metric!)
solvePnP(3D_prev, 2D_cur, K)                        -> T_cur_prev (R,t)
accumulate: T_w_cur = T_w_prev * inv(T_cur_prev)
```

- **DLT triangulation** — for projections P₁, P₂, each pixel contributes two
  rows `u·P[i] − P[0]`, `v·P[i] − P[1]` to `A x = 0`; `x = last row of Vᵀ`
  from `SVD(A)`. Because KITTI images are rectified, disparity is horizontal
  and the depth is **metric** (fixed baseline) — no scale ambiguity (unlike
  monocular).
- **PnP (RANSAC)** — given 3D points from the *previous* frame and their 2D
  projections in the *current* frame, solve for `T_cur_prev` robustly.
- Weaknesses: textureless scenes, abrupt motion, feature dropout — the node
  skips degenerate frames and re-initializes.

### 2.2 ICP (Iterative Closest Point)

**Point-to-point (Besl & McKay):**
```
for each source point: nearest neighbor in target (cKDTree, O(n log n))
solve R,t via Kabsch (SVD of the cross-covariance H = SᵀT)
apply, repeat until RMSE converges
```

**Point-to-plane (Chen & Medioni / Low) — the default:**
```
minimize  Σ_i ( n_i · (R s_i + t − t_i) )²
linearize R ≈ I + [ω]× :
    error_i ≈ n_i·(s_i − t_i) + (s_i × n_i)·ω + n_i·t
    a_i = [s_i × n_i, n_i]   (6-vector)
    solve (Σ a_i a_iᵀ) x = Σ a_i (n_i·(t_i − s_i))
```
Point-to-plane converges faster and more accurately for LiDAR (structured
surfaces) — our tests show mm-level translation and ~0.2° rotation recovery
on a 15 m noisy plane.

**Normals** — PCA smallest eigenvector of the k-NN covariance, **oriented
toward the sensor** (a bug I fixed during development: the flip condition was
inverted, which broke point-to-plane on walls; the test `test_oriented_
toward_origin` catches it).

### 2.3 ICP scan-to-map localization (no EKF)

```
Mapping phase:      map += transform(scan, T_odom_base)     (voxel-downsampled)
Localization:       scan_odom = transform(scan, T_odom_base)     (odometry prior)
                    T_corr = icp_point_to_plane(scan_odom, map)   (vs the map)
                    T_map_odom = T_corr
                    T_map_base = T_map_odom @ T_odom_base         (refined pose)
```

- The **map lives in the odom frame**; `map → odom` is the ICP correction —
  published as a dynamic TF so the whole tree shifts coherently.
- Robustness: warm-started ICP (previous correction as init), a
  `max_translation_jump` sanity guard (tested), and a mapping→localization
  transition driven by map size / frame count.
- **Why ICP and not EKF here (your call):** scan-to-map ICP is
  measurement-driven and doesn't need tuned noise models; it's the classic
  NDT-localizer alternative. FAST-LIO (LiDAR-inertial odometry, C++) is the
  documented future upgrade — this node already publishes the same
  `/localization_pose` + TF interface, so swapping is drop-in.

## 3. Industrial importance

- **Odometry is the robot's proprioceptive motion sense** — the layer every
  planner and mapper depends on. Warehouse AMRs fuse camera/LiDAR/IMU
  odometry before trusting any single source.
- **ICP localization** is the standard way to re-anchor drifted odometry
  against a map (Amazon/PAL-style localizers, autoware's ndt_localizer). No
  EKF = fewer tuned parameters, fully deterministic behavior.
- FAST-LIO / LOAM-class systems are the industrial LiDAR-odometry gold
  standard; our pure-Python ICP is the educational, bug-provable stepping
  stone with an identical interface.

## 4. Folder structure

```
src/adaptive_amr/
├── visual_odometry/   src/geometry_utils.py, src/stereo_vo.py + node/config/launch/test
├── lidar_odometry/    src/icp.py (normals, pp/pt-plane) + node/config/launch/test
├── localization/      src/icp_localizer.py (map + scan-to-map) + node/config/launch/test
├── launch/phase5_odometry.launch
└── rviz/phase5_odometry.rviz
```

## 5. Required packages

`rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
message_filters dataset_loader lidar_processing lidar_odometry
numpy scipy opencv-python` (no new heavy deps — CPU-friendly)

## 6. ROS topics (Phase 5 additions)

| Topic | Type | Pub |
|---|---|---|
| `/visual_odometry` · `/visual_odometry/path` | `nav_msgs/Odometry` · `Path` | visual_odometry |
| `/lidar_odometry` · `/lidar_odometry/path` | `nav_msgs/Odometry` · `Path` | lidar_odometry |
| `/localization_pose` | `PoseWithCovarianceStamped` | localization |
| `/localization/map` | `PointCloud2` | localization |
| `/visual_odometry/statistics`, `/lidar_odometry/statistics`, `/localization/statistics` | `DiagnosticArray` | each node |
| `/tf` (odom→base_link, map→odom) | `TFMessage` | odometry/localization nodes |

## 7-8. TF & parameters
```
map ──(dynamic, localization)──▶ odom ──(dynamic, odometry)──▶ base_link
```
All parameters in each `config/*.yaml` (voxel sizes, ICP thresholds, feature
counts, enable_tf…). One odometry node publishes `odom→base_link` at a time
(default: lidar; flip with `vo_enable_tf`/`lo_enable_tf` launch args).

## 9-11. Python classes
`geometry_utils` (DLT triangulation, pose conversions), `StereoVisualOdometry`
(feature tracking pipeline), `icp` (estimate_normals, icp_point_to_point,
icp_point_to_plane), `IcpLocalizer` (map + scan-to-map).

## 12. Launch

```bash
roslaunch adaptive_amr phase5_odometry.launch             # full stack
roslaunch adaptive_amr phase5_odometry.launch rate:=0     # fast replay
```

## 13. Testing procedure

```bash
# unit tests (no ROS)
python3 src/adaptive_amr/visual_odometry/test/test_geometry_utils.py   # 8
python3 src/adaptive_amr/lidar_odometry/test/test_icp.py               # 6
python3 src/adaptive_amr/localization/test/test_localizer.py           # 4

# live
roslaunch adaptive_amr phase5_odometry.launch rate:=1 &
rostopic hz /visual_odometry /lidar_odometry /localization_pose
rostopic echo -n1 /localization/statistics    # phase: mapping -> localizing
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase5_odometry.rviz
```

## 14. Expected outputs

- `/lidar_odometry`: smooth pose, `last_rmse` < 0.05 m.
- `/visual_odometry`: pose growing with the drive (stereo VO).
- `/localization/statistics`: `phase=mapping` for the first ~60-200 frames,
  then `phase=localizing`; `/localization/map` fills in RViz; the robot's
  pose in RViz stays on the map.
- TF: `map → odom → base_link` fully connected.

## 15. Performance metrics (CPU, indicative)

| Stage | Latency |
|---|---|
| Stereo VO frame (KLT+PnP) | 30-80 ms |
| LiDAR ICP (voxel 0.5, ~10-30k pts) | 30-150 ms |
| Scan-to-map ICP localization | 40-200 ms |
| Normal estimation (10-30k pts) | 20-80 ms |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| VO path flat/zero | no features / P not loaded | check CameraInfo topics; `min_inliers` too high |
| LiDAR odometry jumps | ICP got stuck in local minimum | lower `max_correspondence_distance`, raise `max_iterations` |
| Localization stuck in "mapping" | map never reaches min size | lower `min_map_points` or `max_mapping_frames` |
| TF: "Lookup would require extrapolation" | TF from odometry not publishing | check `enable_tf` on exactly one odometry node |
| RViz pose off the map | ICP correction rejected | check `/localization/statistics` `last_icp_error` |

## 17. Common errors

1. `Two publishers on /tf` → run only one odometry node with `enable_tf`.
2. VO `No module named cv2` → `python3-opencv` required.
3. ICP `need at least 3 points` → scan too small after voxel; lower `voxel_leaf`.
4. `/localization/map` empty → mapping phase hasn't ended yet.
5. Odd VO drift → KITTI cam 02/03 P matrices must come from the *same* date.

## 18. Improvements

- **FAST-LIO / LOAM** (C++) as drop-in LiDAR-odometry upgrade (same topics).
- **NDT** registration (faster on CPU than ICP for large maps).
- Loop closure + pose-graph optimization (the "SLAM" upgrade).
- IMU pre-integration for VO/LIO (motion blur, roll correction).
- KITTI odometry benchmark evaluation (ATE/RPE) — Phase 8 formalizes this.

## 19. Interview questions

1. Why is stereo VO metric but monocular VO not? (fixed baseline)
2. Derive the DLT triangulation rows from a projection matrix.
3. Point-to-point vs point-to-plane ICP: when is each better and why?
4. Why orient normals toward the sensor? What breaks if they flip randomly?
5. How does scan-to-map ICP localization correct odometry drift?
6. Why is the map built in the odom frame, and what does `map→odom` encode?
7. What is the Kabsch solution, and why use SVD?
8. How does RANSAC-PnP reject outlier feature tracks?
9. What happens if ICP converges to a local minimum? How do you detect it?
10. Why publish odometry as `nav_msgs/Odometry` and not a custom message?
11. What is the warm-start trick in ICP localization?
12. EKF vs ICP localization: trade-offs (tuning, observability, cost).
13. How would you add IMU to the LiDAR odometry (FAST-LIO style)?
14. What limits ICP speed, and how does voxel downsampling help?
15. How would you detect and handle a kidnapped-robot (relocalization) case?

## 20. Git commits for this phase

```bash
feat(visual_odometry): add stereo feature-based visual odometry (DLT + PnP)
feat(lidar_odometry): add point-to-point and point-to-plane ICP odometry
feat(localization): add ICP scan-to-map localization (no EKF)
docs(kittiraith_ws): add phase 5 documentation, tests, launch and RViz config
```

---

**Phase 5 complete. Next: Phase 6 — Semantic Mapping, Dynamic Occupancy Grid & Motion Prediction.**
