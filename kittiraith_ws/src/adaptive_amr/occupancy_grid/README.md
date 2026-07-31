# occupancy_grid

> **Phase 6 — ✅ implemented**

## 1. Objective
Probabilistic 2D occupancy grid from the obstacle cloud + pose using log-odds
ray casting (Bresenham) — published as `nav_msgs/OccupancyGrid`.

## 2. Theory
`l ← l + l_occ` (hit), `l ← l + l_free` (ray cells); clamp ±L; `p = 1−1/(1+e^l)`
→ 0..100. Bresenham = integer-only O(L) rays. Repeated evidence accumulates;
single spurious hits can't flip a cell.

## 3. Industrial importance
The exact format `move_base` costmaps, AMCL and warehouse planners consume —
the map the robot plans *on*.

## 4. Folder structure
```
occupancy_grid/
├── src/occupancy_grid/grid_mapping.py   # pure math (9 tests)
├── scripts/occupancy_grid_node.py
├── launch/occupancy_grid.launch
├── config/occupancy_grid.yaml
└── test/test_grid_mapping.py
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
message_filters dataset_loader lidar_processing numpy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/occupancy_grid` | `nav_msgs/OccupancyGrid` | pub (latched) |
| `/occupancy_grid/statistics` | `DiagnosticArray` | pub |
| `/lidar_processing/obstacles` + `/localization_pose` (or `/lidar_odometry`) | — | sub |

## 9. Parameters / 10. Configuration
`config/occupancy_grid.yaml`: `resolution`, `width_m`, `height_m`, `l_occ`,
`l_free`, `l_clamp`, `max_range`, `use_localization`.

## 11. Python classes
`OccupancyGridMapper` — add_scan / get_occupancy / get_log_odds / reset;
`bresenham`.

## 12. Launch
```bash
roslaunch occupancy_grid occupancy_grid.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/occupancy_grid/test/test_grid_mapping.py
# live:
rostopic hz /occupancy_grid
```

## 14. RViz configuration
`adaptive_amr/rviz/phase6_mapping.rviz` (Map display).

## 15. Expected outputs
Walls/vehicles 70-100, traversed area 0-40, unseen -1; grid fills as the
robot drives.

## 16. Performance metrics
10-40 ms per frame (10-30k pts, 300×300 grid).

## 17. Debugging guide
All -1 → pose/obstacles topics not arriving; smearing → bad localization.

## 18. Common errors
Grid origin off-center → check `width_m`/`height_m`; values all 0 or 100 →
`l_clamp` too low / observations too few.

## 19. Improvements
Inflation layer (Phase 7), 3D OctoMap, dynamic-object erasure (don't paint
moving cars into the static map), multi-robot map merge.

## 20. Git commit
`feat(occupancy_grid): add log-odds ray-cast occupancy grid mapping`
