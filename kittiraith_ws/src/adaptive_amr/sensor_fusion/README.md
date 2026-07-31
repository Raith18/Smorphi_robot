# sensor_fusion

> **Phase 3 — ✅ implemented**

## 1. Objective
Fuse camera and LiDAR at the pixel level: project every Velodyne point into
the image, producing a camera-colored point cloud, a sparse LiDAR depth image
and a projected overlay.

## 2. Theory
`p_img = P_rect · R_rect4x4 · T_velo_cam · p_velo`; `u = x/z, v = y/z,
depth = z`. The combined 3×4 matrix maps velodyne points straight to pixels
(one multiply per scan, O(n)).

## 3. Industrial importance
This is the projection primitive behind every late-fusion perception stack:
colored clouds feed detection/segmentation, sparse depth seeds dense depth,
overlays validate calibration by eye in minutes.

## 4. Folder structure
```
sensor_fusion/
├── src/sensor_fusion/projection.py   # pure NumPy projection math
├── scripts/sensor_fusion_node.py
├── launch/sensor_fusion.launch
├── config/sensor_fusion.yaml
└── test/test_projection.py           # 8 tests
```

## 5. Required packages
`rospy std_msgs sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs
message_filters diagnostic_msgs dataset_loader python3-numpy python3-opencv`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/fusion/colored_points` | `PointCloud2` (rgb) | pub |
| `/fusion/sparse_depth` | `Image` 32FC1 | pub |
| `/fusion/overlay` | `Image` bgr8 | pub |
| `/fusion/statistics` | `DiagnosticArray` | pub |
| `/camera/image_rect`, `/velodyne_points`, `/camera/camera_info_rect` | — | sub |

## 9. Parameters / 10. Configuration
`config/sensor_fusion.yaml`: topics, `sync_type`, `use_tf`,
`load_calibration_from_dataset`, `max_projection_distance`, frames.

## 11. Python classes
`LidarCameraProjection` (project / build_sparse_depth / colorize),
`combined_projection_matrix`.

## 12. Launch
```bash
roslaunch sensor_fusion sensor_fusion.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/sensor_fusion/test/test_projection.py
# live: RViz colored cloud should align with the camera image
```

## 14. RViz configuration
`adaptive_amr/rviz/phase3_fusion.rviz`.

## 15. Expected outputs
Colored cloud (RGB8), sparse depth with NaN holes, overlay with depth-colored
projections; diagnostics `projection_ready=true`.

## 16. Performance metrics
Projection 100k pts < 1 ms; colorize < 2 ms; overlay (cv2) ~5-10 ms.

## 17. Debugging guide
Misaligned overlay → calibration/TF wrong (check `tf_echo`); depth all NaN →
wrong projection matrix (camera_info_rect vs camera_info).

## 18. Common errors
`TF lookup failed` → run calibration node first, or set
`load_calibration_from_dataset:=true` with valid dataset_root.

## 19. Improvements
Dense depth interpolation (Phase 4), ego-motion compensation for moving
objects, rolling-shutter compensation, point-view fusion (Phase 4).

## 20. Git commit
`feat(sensor_fusion): add camera-lidar projection, depth and colorization fusion`
