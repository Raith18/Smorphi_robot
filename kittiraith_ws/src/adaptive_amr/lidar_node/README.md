# lidar_node

> **Phase 2 — ✅ implemented**

## 1. Objective
Replay KITTI Velodyne HDL-64E scans as `sensor_msgs/PointCloud2` on
`/velodyne_points`, structured like a real LiDAR driver.

## 2. Theory
Velodyne `.bin` files are `[x, y, z, reflectance]` float32 — exactly a
PointCloud2 payload, so the node publishes the raw bytes (zero copy) with
fields `x, y, z, intensity`. Frame `laser` (KITTI velodyne convention:
x forward, y left, z up).

## 3. Industrial importance
LiDAR is the primary 3D geometry sensor in AMRs; its driver must deliver
~120k-point scans at 10 Hz without copies or drops.

## 4. Folder structure
```
lidar_node/
├── package.xml  CMakeLists.txt
├── scripts/lidar_node.py
├── launch/lidar_node.launch
└── config/lidar_node.yaml
```

## 5. Required packages
`rospy std_msgs sensor_msgs dataset_loader python3-numpy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | pub (frame `laser`) |

## 9. Parameters
`dataset_root, date, drive, points_topic, frame_id, rate_factor, loop`

## 10. Configuration
`config/lidar_node.yaml` — all of the above.

## 11. Python classes
`LidarNode` — pacing loop + zero-copy PointCloud2 publishing.

## 12. Launch
```bash
roslaunch lidar_node lidar_node.launch
roslaunch lidar_node lidar_node.launch rate:=0
```

## 13. Testing procedure
```bash
rostopic hz /velodyne_points            # ~10 Hz
rostopic echo -n1 /velodyne_points --noarr | grep -E "width|point_step"
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` — PointCloud2 display, Intensity coloring.

## 15. Expected outputs
10 Hz scans, width ≈ 100k-120k points, fields x/y/z/intensity (16-byte points).

## 16. Performance metrics
Zero-copy (file bytes = payload); ~0.5-1 ms/frame I/O at 10 Hz.

## 17. Debugging guide
Empty cloud → wrong `date/drive`; check `rostopic type`; `rosbag record` to verify.

## 18. Common errors
PointCloud2 shows nothing in RViz → Fixed Frame is not `map` with TF running
(run `calibration`); or `Size (Pixels)` too small.

## 19. Improvements
Downsampling param, ring/intensity normalization, scan-line (ring) fields,
PCL-based passthrough (Phase 3 lidar_processing).

## 20. Git commit
`feat(lidar_node): add KITTI Velodyne driver node`
