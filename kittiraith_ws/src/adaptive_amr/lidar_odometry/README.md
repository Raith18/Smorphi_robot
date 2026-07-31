# lidar_odometry

> **Phase 5 — ✅ implemented**

## 1. Objective
Frame-to-frame LiDAR odometry with **ICP**: point-to-point (Kabsch) and
point-to-plane (default). Publishes `/lidar_odometry` + `odom → base_link`.

## 2. Theory
- `estimate_normals`: PCA smallest eigenvector of k-NN covariance, oriented
  toward the sensor (flip rule `dot > 0` — unit-tested).
- `icp_point_to_point`: nearest neighbors via cKDTree + Kabsch SVD alignment.
- `icp_point_to_plane`: linearized least squares
  `(Σ aᵢaᵢᵀ)x = Σ aᵢ(nᵢ·(tᵢ−sᵢ))`, `aᵢ=[sᵢ×nᵢ, nᵢ]`; converges to mm-level
  on planes.
- Accumulate `T_w_cur = T_w_prev @ T_rel`; velocity from `T_rel`/dt.

## 3. Industrial importance
LiDAR odometry is the backbone of LOAM/FAST-LIO-class systems; ICP is the
classic registration primitive. Deterministic, no training data, works in
the dark.

## 4. Folder structure
```
lidar_odometry/
├── src/lidar_odometry/icp.py     # pure math (unit-tested)
├── scripts/lidar_odometry_node.py
├── launch/lidar_odometry.launch
├── config/lidar_odometry.yaml
└── test/test_icp.py              # 6 tests
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
dataset_loader lidar_processing numpy scipy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/lidar_odometry` | `nav_msgs/Odometry` | pub |
| `/lidar_odometry/path` | `nav_msgs/Path` | pub |
| `/lidar_odometry/statistics` | `DiagnosticArray` | pub |
| `/velodyne_points` | `PointCloud2` | sub |

## 9. Parameters / 10. Configuration
`config/lidar_odometry.yaml`: `voxel_leaf`, `icp_method`
(point_to_plane|point_to_point), `max_iterations`,
`max_correspondence_distance`, `min_points`, `enable_tf`.

## 11. Python classes
`estimate_normals`, `icp_point_to_point`, `icp_point_to_plane`,
`LidarOdometryNode`.

## 12. Launch
```bash
roslaunch lidar_odometry lidar_odometry.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/lidar_odometry/test/test_icp.py
# live:
rostopic hz /lidar_odometry
rostopic echo -n1 /lidar_odometry/statistics | grep rmse
```

## 14. RViz configuration
`adaptive_amr/rviz/phase5_odometry.rviz` (green path).

## 15. Expected outputs
10 Hz odometry; `last_rmse` typically < 0.05 m on KITTI city scenes.

## 16. Performance metrics
30-150 ms/frame (voxel 0.5, ~10-30k pts).

## 17. Debugging guide
Jumps → reduce `max_correspondence_distance`; slow → raise `voxel_leaf`.

## 18. Common errors
Too few points after voxel → lower `voxel_leaf` or raise `min_points` guard.

## 19. Improvements
NDT registration (faster), IMU pre-integration (FAST-LIO-style), loop
closure, feature-based (featureless-plane) selection.

## 20. Git commit
`feat(lidar_odometry): add point-to-point and point-to-plane ICP odometry`
