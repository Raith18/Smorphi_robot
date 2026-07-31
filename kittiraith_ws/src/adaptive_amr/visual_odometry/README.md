# visual_odometry

> **Phase 5 — ✅ implemented**

## 1. Objective
Classic **stereo visual odometry**: Shi-Tomasi corners + KLT optical flow +
DLT triangulation + PnP. Publishes `/visual_odometry` (nav_msgs/Odometry),
a path and optional `odom → base_link` TF.

## 2. Theory
- DLT triangulation: `A x = 0` with rows `u·P[i]−P[0]`, `v·P[i]−P[1]`; solved
  by SVD. Metric because the stereo baseline is fixed.
- PnP (RANSAC) with the *previous* frame's 3D points → current 2D points
  yields `T_cur_prev`; accumulate `T_w_cur = T_w_prev · inv(T_cur_prev)`.
- Degenerate frames (too few inliers) are skipped and re-initialized.

## 3. Industrial importance
VO is the camera-only motion backbone (viso2/ORB-SLAM front-ends). It
complements LiDAR odometry indoors/outdoors and gives motion even when the
LiDAR sees nothing (e.g., long corridors).

## 4. Folder structure
```
visual_odometry/
├── src/visual_odometry/{geometry_utils,stereo_vo}.py
├── scripts/visual_odometry_node.py
├── launch/visual_odometry.launch
├── config/visual_odometry.yaml
└── test/test_geometry_utils.py      # 8 tests
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
dataset_loader numpy opencv-python`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/visual_odometry` | `nav_msgs/Odometry` | pub |
| `/visual_odometry/path` | `nav_msgs/Path` | pub |
| `/visual_odometry/statistics` | `DiagnosticArray` | pub |
| `/camera/image_raw` + `/camera_right/image_raw` | `Image` | sub |
| `/camera/camera_02/camera_info` + `/camera/camera_03/camera_info` | `CameraInfo` | sub |

## 9. Parameters / 10. Configuration
`config/visual_odometry.yaml`: topics, frames, `enable_tf`, `max_features`,
`quality_level`, `min_distance`, `min_inliers`, `ransac_reproj`.

## 11. Python classes
`StereoVisualOdometry` (process/_klt/_detect_and_match),
`geometry_utils` (triangulate_dlt/many, pose conversions).

## 12. Launch
```bash
roslaunch adaptive_amr phase5_odometry.launch     # includes stereo cameras
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/visual_odometry/test/test_geometry_utils.py
# live:
rostopic hz /visual_odometry
```

## 14. RViz configuration
`adaptive_amr/rviz/phase5_odometry.rviz` (yellow path).

## 15. Expected outputs
10 Hz odometry; path advances with the drive; `last_pnp_inliers` ≥ 15.

## 16. Performance metrics
30-80 ms/frame CPU (KLT + PnP).

## 17. Debugging guide
Flat path → features lost (textureless scene); check P matrices loaded.

## 18. Common errors
`No module named cv2`; CameraInfo not arriving (start calibration first).

## 19. Improvements
Stereo matching by NCC (denser), IMU-aided tracking, bundle adjustment,
loop closure (Phase 6 SLAM upgrade).

## 20. Git commit
`feat(visual_odometry): add stereo feature-based visual odometry (DLT + PnP)`
