# localization

> **Phase 5 — ✅ implemented** (ICP scan-to-map, **no EKF** per project spec)

## 1. Objective
Re-anchor drifted LiDAR odometry against a point-cloud map using **ICP
scan-to-map** (point-to-plane). Publishes `/localization_pose`, the dynamic
`map → odom` TF and the map cloud.

## 2. Theory
- Mapping: accumulate downsampled scans (transformed by odometry poses) into
  a voxel map in the odom frame; `T_map_odom = I`.
- Localization: `scan_odom = T_odom_base · scan`; ICP vs the map with the
  previous correction as warm start; `T_map_odom = T_corr`;
  `T_map_base = T_corr · T_odom_base`.
- Sanity: reject corrections larger than `max_translation_jump`.

## 3. Industrial importance
This is the classic NDT/ICP localizer pattern used by autoware and warehouse
AMRs: measurement-driven, no tuned noise models, deterministic. The interface
(`/localization_pose` + TF) matches FAST-LIO, so a C++ upgrade is drop-in.

## 4. Folder structure
```
localization/
├── src/localization/icp_localizer.py   # pure math (unit-tested)
├── scripts/localization_node.py
├── launch/localization.launch
├── config/localization.yaml
└── test/test_localizer.py              # 4 tests
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
message_filters dataset_loader lidar_odometry lidar_processing numpy scipy`

## 6-8. ROS topics & services
| Topic | Type | Dir |
|---|---|---|
| `/localization_pose` | `PoseWithCovarianceStamped` | pub |
| `/localization/map` | `PointCloud2` | pub (latched) |
| `/localization/statistics` | `DiagnosticArray` | pub |
| `/velodyne_points` + `/lidar_odometry` | — | sub |

| Service | Type | Purpose |
|---|---|---|
| `/localization/relocalize` | `adaptive_amr_msgs/Relocalize` | Reset map->odom (or restart mapping) after a kidnapping event |

```bash
rosservice call /localization/relocalize "{restart_mapping: false}"
```

## 9. Parameters / 10. Configuration
`config/localization.yaml`: mapping phase (`voxel_leaf`, `min_map_points`,
`max_map_points`, `max_mapping_frames`), ICP (`max_correspondence_distance`,
`max_translation_jump`), TF frames, topics.

## 11. Python classes
`IcpLocalizer` (add_scan_to_map / localize / reset / get_map),
`LocalizationNode`.

## 12. Launch
```bash
roslaunch localization localization.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/localization/test/test_localizer.py
# live:
rostopic echo -n1 /localization/statistics   # phase transitions
```

## 14. RViz configuration
`adaptive_amr/rviz/phase5_odometry.rviz` (map + pose + TF).

## 15. Expected outputs
Mapping phase fills `/localization/map`; after switching, `/localization_pose`
stays on the map; `map → odom` TF shifts smoothly.

## 16. Performance metrics
40-200 ms/frame scan-to-map ICP (10-30k pts vs 60-400k map pts).

## 17. Debugging guide
Pose drifts off map → raise `max_correspondence_distance` or reduce
`voxel_leaf`; stuck in mapping → lower `min_map_points`.

## 18. Common errors
Map not published → mapping not finished; ICP jump guard rejecting →
check odometry quality first.

## 19. Improvements
NDT for CPU speed, KITTI ground-truth validation (Phase 8), relocalization
(global search), FAST-LIO drop-in (same interface).

## 20. Git commit
`feat(localization): add ICP scan-to-map localization (no EKF)`
