# Phase 6 — Semantic Mapping · Dynamic Occupancy Grid · Motion Prediction

> **Module status: ✅ COMPLETE**

Full tutorial: theory with mathematics, industrial context, every implemented
file, testing procedure, debugging, performance and interview questions.

---

## 1. Objective

| Module | Package | Input → Output |
|---|---|---|
| Semantic mapping | `semantic_mapping` | colored points + pose → `/semantic_map` (global voxel cloud, rgb) |
| Occupancy grid | `occupancy_grid` | obstacle cloud + pose → `/occupancy_grid` (nav_msgs/OccupancyGrid) |
| Motion prediction | `motion_prediction` | `/object_tracks` → `/dynamic_obstacles` (predicted trajectories) |

**Definition of done:** `roslaunch adaptive_amr phase6_mapping.launch` produces
all three topics; RViz shows the occupancy grid, the growing semantic map, and
predicted obstacle trajectories.

## 2. Theory

### 2.1 Log-odds occupancy grid

```
belief:   l(x) = log( p_occ(x) / (1 - p_occ(x)) )
update:   l(x) <- l(x) + l_occ    (endpoint / hit)
          l(x) <- l(x) + l_free   (every cell along the ray, Bresenham)
clamp:    l(x) <- clip(l(x), -L, L)
output:   p_occ = 1 - 1/(1 + exp(l))   ->  0..100 OccupancyGrid value
```

- **Bresenham**: integer-only line rasterization, O(L) per ray, exact
  (no floating-point drift).
- **Why log-odds?** additive updates are commutative and associative →
  frame order doesn't matter; repeated evidence accumulates; a single spurious
  hit cannot flip a cell (see the `test_incremental_evidence` test: 10 free
  observations clamp p to ~0.03).
- One observation gives p≈0.70 for a hit and p≈0.40 for a free cell — honest
  probabilities, not binary flags.

### 2.2 Semantic map accumulation

```
p_map = T_map_base · p_base        (pose from /localization_pose)
voxel = floor(p_map / leaf)
per voxel: mean position + DOMINANT color (mode of the semantic rgb)
```

Dominant color keeps the map stable against transient mislabels — a voxel
painted "car" 90% of the time stays car even if one frame mislabels it.
Memory bounded: above `max_voxels` the leaf doubles (auto-coarsen).

### 2.3 Constant-velocity Kalman prediction

Per track: `x = [px, py, pz, vx, vy, vz]`,
```
F = [ I3  dt·I3 ]      H = [ I3  0 ]
    [ 0     I3 ]
predict:  x' = Fx ,  P' = FPFᵀ + Q
update:   K = P'Hᵀ(HP'Hᵀ+R)⁻¹ ,  x = x' + K(z − Hx')
trajectory: p(τ) = p0 + v·τ   (τ ∈ [0, horizon])
```
- `collision_risk = clip(1 − d_min/safe_distance, 0, 1)` where `d_min` is the
  closest approach of the predicted path to the ego origin.
- **Bug fixed during development:** `F` was built once with `dt=0.1` but the
  frame rate varies; the velocity extrapolation used the wrong timestep. The
  fix refreshes `F` from `dt` on every predict — caught by
  `test_track_velocity_estimate`.

## 3. Industrial importance

- The **occupancy grid is the lingua franca of robot navigation** — `move_base`
  costmaps, AMCL, and every warehouse AMR planner consume exactly this format.
- The **semantic map** is the operator-facing deliverable ("where are the
  pallets/cars/people") and the substrate for semantic navigation.
- **Motion prediction** turns "something is there" into "it will be *here* in
  2 s" — the difference between reactive stopping and safe proactive
  planning, mandated by safety standards in industrial robotics.

## 4. Folder structure

```
src/adaptive_amr/
├── semantic_mapping/   src/map_builder.py + node + config/launch/test (6)
├── occupancy_grid/     src/grid_mapping.py + node + config/launch/test (9)
├── motion_prediction/  src/predictor.py + node + config/launch/test (7)
├── adaptive_amr_msgs/msg/{DynamicObstacle,DynamicObstacleArray}.msg
├── launch/phase6_mapping.launch
└── rviz/phase6_mapping.rviz
```

## 5. Required packages

`rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
message_filters visualization_msgs dataset_loader lidar_processing
adaptive_amr_msgs numpy scipy`

## 6. ROS topics (Phase 6 additions)

| Topic | Type | Pub |
|---|---|---|
| `/semantic_map` | `PointCloud2` (xyz + rgb) | semantic_mapping |
| `/occupancy_grid` | `nav_msgs/OccupancyGrid` | occupancy_grid |
| `/dynamic_obstacles` | `adaptive_amr_msgs/DynamicObstacleArray` | motion_prediction |
| `/dynamic_obstacles/markers` | `MarkerArray` | motion_prediction |
| `/semantic_map/statistics`, `/occupancy_grid/statistics`, `/dynamic_obstacles/statistics` | `DiagnosticArray` | each node |
| `/semantic_map/clear` | `adaptive_amr_msgs/Relocalize` (service) | semantic_mapping |

## 7-8. TF & parameters
All maps live in the `map` frame (pose from `/localization_pose`, fallback
`/lidar_odometry`). Prediction runs in the `laser` frame (documented). All
parameters in `config/*.yaml`.

## 9-11. Python classes
`SemanticMapBuilder` (add_frame/get_map/clear), `OccupancyGridMapper`
(add_scan/get_occupancy), `MotionPredictor` + `ConstantVelocityFilter`
(update/predict_trajectory/collision_risk), `bresenham`.

## 12. Launch

```bash
roslaunch adaptive_amr phase6_mapping.launch                 # full stack
roslaunch adaptive_amr phase6_mapping.launch with_perception:=false
roslaunch adaptive_amr phase6_mapping.launch rate:=0.5       # CPU-friendly
```

## 13. Testing procedure

```bash
# unit tests (no ROS)
python3 src/adaptive_amr/semantic_mapping/test/test_map_builder.py      # 6
python3 src/adaptive_amr/occupancy_grid/test/test_grid_mapping.py       # 9
python3 src/adaptive_amr/motion_prediction/test/test_predictor.py       # 7

# live
roslaunch adaptive_amr phase6_mapping.launch rate:=1 &
rostopic hz /occupancy_grid /semantic_map /dynamic_obstacles
rostopic echo -n1 /occupancy_grid | head -30     # data[0..] 0/100/-1
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase6_mapping.rviz
```

## 14. Expected outputs

- `/occupancy_grid`: 10 Hz, map fills as the robot moves; walls/vehicles
  become 70-100, traversed areas ~0-40, unseen -1.
- `/semantic_map`: colored global cloud growing with the drive.
- `/dynamic_obstacles`: one obstacle per track with a predicted trajectory
  and `collision_risk` (0..1).
- Diagnostics: cell counts, map voxels, obstacle count.

## 15. Performance metrics (CPU, indicative)

| Stage | Latency |
|---|---|
| Grid ray-cast (10-30k pts, 300×300 grid) | 10-40 ms |
| Semantic map merge (10-30k pts) | 10-30 ms |
| Motion prediction (≤ 20 tracks) | < 1 ms |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| Grid all -1 | pose topic missing / obstacles topic wrong | check `/localization_pose` + `/lidar_processing/obstacles` |
| Grid "smears" | pose jumps (bad localization) | run localization first; check `/localization/statistics` |
| Semantic map empty | no `/semantic_map/colored_points` | enable `with_perception` (YOLOv8-seg) |
| No dynamic obstacles | no `/object_tracks` | enable detection+tracking; check `/object_tracks` |
| Trajectories wild | CV filter diverging (occluded track) | raise `measurement_noise`, lower `process_noise` |

## 17. Common errors

1. Map frames wrong → set `frame_id: map` (grid) / `map_frame: map`.
2. `/occupancy_grid` type mismatch → the topic is `nav_msgs/OccupancyGrid`,
   not PointCloud2.
3. Prediction in `laser` frame while map in `map` — document the frame in the
   message header (custom msg has no frame; consumers use the topic header).

## 18. Improvements

- **3D occupancy (OctoMap)** for full volumetric awareness.
- **EDT costmap inflation** for the navigation layer (Phase 7).
- Learned motion prediction (Trajectron++, social LSTM) once a GPU exists.
- Semantic grid layers (per-class costmaps) — directly consumable by
  semantic planners.
- Ray-traced GPU grid updates for large maps.

## 19. Interview questions

1. Why log-odds instead of raw probabilities for occupancy?
2. What does a value of 40 in OccupancyGrid mean? (p≈0.40, one free hit)
3. Complexity of Bresenham ray casting? (O(L) per ray, no float ops)
4. How does the semantic map stay stable against transient mislabels?
   (dominant color per voxel)
5. Why does the CV filter need F refreshed from dt?
6. What is `collision_risk` and how would you make it rigorous?
7. How would you fuse multiple robots' grids? (map merging)
8. How does motion prediction differ from tracking? (future vs present)
9. What happens to predictions during occlusions? (CV extrapolation, growth)
10. How would you evaluate occupancy-grid accuracy on KITTI?
11. Why keep the map in the `map` frame? (frame discipline)
12. What are the limits of constant-velocity prediction? (turning, stops)
13. How would you add an inflated obstacle layer for planning?
14. What does auto-coarsening of the voxel map buy you? (bounded memory)
15. How does this phase's output feed Phase 7 navigation directly?

## 20. Git commits for this phase

```bash
feat(adaptive_amr_msgs): add dynamic obstacle message definitions
feat(semantic_mapping): add global semantic voxel map builder and node
feat(occupancy_grid): add log-odds ray-cast occupancy grid mapping
feat(motion_prediction): add constant-velocity Kalman trajectory prediction
docs(kittiraith_ws): add phase 6 documentation, tests, launch and RViz config
```

---

**Phase 6 complete. Next: Phase 7 — Navigation Layer, Behavior Layer & Decision Layer.**
