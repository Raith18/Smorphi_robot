# semantic_mapping

> **Phase 6 — ✅ implemented**

## 1. Objective
Build the global **semantic map**: accumulate the semantic-colored LiDAR cloud
into a voxel map in the `map` frame, published on `/semantic_map`.

## 2. Theory
`p_map = T_map_base · p_base`; voxel-quantize; per voxel average position +
**dominant color** (mode) so transient mislabels can't repaint persistent
voxels. Memory bound via auto-coarsening (leaf doubles past `max_voxels`).

## 3. Industrial importance
The operator-facing "world model" and the substrate for semantic planning —
the same output warehouse fleet dashboards and pick planners consume.

## 4. Folder structure
```
semantic_mapping/
├── src/semantic_mapping/map_builder.py   # pure math (6 tests)
├── scripts/semantic_mapping_node.py
├── launch/semantic_mapping.launch
├── config/semantic_mapping.yaml
└── test/test_map_builder.py
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
message_filters dataset_loader lidar_processing numpy`

## 6-8. ROS topics & services
| Topic | Type | Dir |
|---|---|---|
| `/semantic_map` | `PointCloud2` (xyz+rgb) | pub (latched) |
| `/semantic_map/statistics` | `DiagnosticArray` | pub |
| `/semantic_map/colored_points` + `/localization_pose` (or `/lidar_odometry`) | — | sub |
| `/semantic_map/clear` | `adaptive_amr_msgs/Relocalize` (service) | — |

## 9. Parameters / 10. Configuration
`config/semantic_mapping.yaml`: `voxel_size`, `max_voxels`, `use_localization`,
topics, `map_frame`.

## 11. Python classes
`SemanticMapBuilder` — add_frame / get_map / clear / _coarsen / _dominant_color.

## 12. Launch
```bash
roslaunch semantic_mapping semantic_mapping.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/semantic_mapping/test/test_map_builder.py
# live:
rostopic hz /semantic_map
```

## 14. RViz configuration
`adaptive_amr/rviz/phase6_mapping.rviz` (Semantic map display, RGB8).

## 15. Expected outputs
A colored global cloud growing with the drive; `map_voxels` in statistics.

## 16. Performance metrics
Merge 10-30k pts: 10-30 ms.

## 17. Debugging guide
Empty map → no colored_points (enable perception) or pose not arriving.

## 18. Common errors
Wrong frame → check `map_frame`; colors missing → rgb unpacking (float32 view).

## 19. Improvements
OctoMap 3D, semantic grid layers, per-class confidence, map persistence
(Phase 10 saves maps to disk).

## 20. Git commit
`feat(semantic_mapping): add global semantic voxel map builder and node`
