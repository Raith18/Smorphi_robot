# Architecture Overview

> Adaptive Multi-Sensor Perception, Localization and Semantic Mapping Framework
> for Autonomous Mobile Robots (KITTI · ROS Noetic · Ubuntu 20.04)

---

## 1. System block diagram

```
                        ┌──────────────────────────────────────────────────┐
                        │                  KITTI DATASET                   │
                        │  raw/2011_09_26/<drive>_sync/   odometry/seqs/   │
                        │  image_00..03 · velodyne · oxts · calib          │
                        └───────────────────────┬──────────────────────────┘
                                                │ Phase 2 (player)
        ┌───────────┬─────────────┬─────────────┴───────────┬─────────────┐
        ▼           ▼             ▼                         ▼             ▼
   camera_node  lidar_node    gps_node                  imu_node     dataset_loader
   /camera/     /velodyne_    /gps/fix                  /imu/data    (bags & files)
   image_raw    points        (navsatfix)                           
        └───────────┴─────────────┴─────────────┬─────────────┘
                                                │ time_sync (Phase 2)
                                                ▼
                                       calibration (Phase 2)
                                                │
        ┌───────────────────────────────────────┼──────────────────────────┐
        ▼                                       ▼                          ▼
 camera_processing                        lidar_processing          sensor_fusion
 (rectify/undistort)                     (ground seg, clustering)   (projection, fusion)
        │                                       │                          │
        └───────────────────┬───────────────────┘                          │
                            ▼                                              │
   ┌─────────────────────────────────────────────────────┐                 │
   │ object_detection · semantic_segmentation · tracking │ (Phase 4)       │
   └─────────────────────────────────────────────────────┘                 │
                            │                                              │
        ┌───────────────────┼───────────────────┐                          │
        ▼                   ▼                   ▼                          │
 visual_odometry      lidar_odometry     sensor_fusion localization (Ph5) │
        └───────────────────┼───────────────────┘                          │
                            ▼                                              │
                    ┌──────────────────┐                                   │
                    │  localization    │  map ── odom ── base_link TF      │
                    └──────────────────┘                                   │
                            │                                              │
        ┌───────────────────┼───────────────────┐                          │
        ▼                   ▼                   ▼                          │
 semantic_mapping     occupancy_grid     motion_prediction                 │
        │                   │                   │                          │
        └───────────────────┼───────────────────┘                          │
                            ▼                                              │
                    navigation_layer  →  navigation_costmap (Phase 7)      │
                                                                           │
                            RViz visualization across all phases ◄─────────┘
```

## 2. TF tree (target)

```
map
 │  (localization: map -> odom correction)
 ▼
odom
 │  (odometry: visual/lidar/sensor-fusion)
 ▼
base_link
 │  (lidar mount)
 ▼
laser
 │  (camera mount)
 ▼
camera_link
 │  (optical rotation: +x right, +y down, +z forward)
 ▼
camera_optical_frame
 │  (IMU mount)
 ▼
imu_link
```

Published via `tf2`: static transforms (`tf_static`) for mounts/calibration,
dynamic transforms for `map -> odom` (localization) and `odom -> base_link`
(odometry).

## 3. ROS topic bus (planned)

| Topic | Type | Pub | Sub |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node | camera_processing, detection… |
| `/camera/image_rect` | `sensor_msgs/Image` | camera_processing | fusion, segmentation |
| `/velodyne_points` | `sensor_msgs/PointCloud2` | lidar_node | lidar_processing, odometry |
| `/imu/data` | `sensor_msgs/Imu` | imu_node | localization, odometry |
| `/gps/fix` | `sensor_msgs/NavSatFix` | gps_node | localization |
| `/object_detections` | `vision_msgs/Detection2DArray` | object_detection | tracking |
| `/object_tracks` | custom `adaptive_amr_msgs/ObjectTrackArray` | object_tracking | prediction, mapping |
| `/semantic_map` | `sensor_msgs/PointCloud2` (label channel) | semantic_mapping | navigation |
| `/visual_odometry` | `nav_msgs/Odometry` | visual_odometry | localization |
| `/lidar_odometry` | `nav_msgs/Odometry` | lidar_odometry | localization |
| `/localization_pose` | `geometry_msgs/PoseWithCovarianceStamped` | localization | navigation |
| `/occupancy_grid` | `nav_msgs/OccupancyGrid` | occupancy_grid | navigation |
| `/dynamic_obstacles` | `adaptive_amr_msgs/DynamicObstacleArray` | motion_prediction | navigation |
| `/navigation_costmap` | `nav_msgs/OccupancyGrid` | navigation_layer | planner/UI |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | all | all |

## 4. Module → phase map

| Phase | Packages |
|---|---|
| 2 | dataset_loader, time_sync, calibration, camera_node, lidar_node, gps_node, imu_node |
| 3 | camera_processing, lidar_processing, sensor_fusion |
| 4 | object_detection, object_tracking, semantic_segmentation |
| 5 | visual_odometry, lidar_odometry, localization |
| 6 | semantic_mapping, occupancy_grid, motion_prediction |
| 7 | navigation_layer |

## 5. Design principles

1. **One module = one ROS package** — replaceable in isolation (e.g., swap
   YOLO-based detection for CenterPoint without touching tracking).
2. **Topic-based decoupling** — modules communicate only through ROS topics,
   so any module can be replayed, recorded (rosbag), or unit-tested alone.
3. **Config in YAML, not code** — every node reads parameters from
   `config/*.yaml` (or rosparam), no hardcoded paths.
4. **Time-synchronized fusion** — Phase 2 guarantees aligned
   camera/LiDAR/IMU/GPS stamps before fusion starts.
5. **Evaluation from day one** — KITTI ground truth (poses, labels, semantics)
   enables quantitative benchmarking of every module (Phase 8).
