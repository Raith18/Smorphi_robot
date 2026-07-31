# camera_processing

> **Phase 3 — ✅ implemented**

## 1. Objective
Process the raw camera stream: undistortion/rectification (OpenCV or pure
NumPy), color conversion, ROI crop and resize — always keeping `CameraInfo`
consistent with the output image.

## 2. Theory
Pinhole + plumb-bob distortion: `p_img = K·[R|t]·P_world`; rectification maps
built as `(u,v) → ray → R_rectᵀ → distort(K,D) → source pixel`.
Crop/resize rule: `f' = s·f, c' = s·(c−crop), t' = s·t`.

## 3. Industrial importance
Every vision stack rectifies before fusion; publishing processed images with
stale intrinsics silently breaks all projections — this node prevents that by
design.

## 4. Folder structure
```
camera_processing/
├── src/camera_processing/rectify.py   # pure NumPy math (unit-tested)
├── scripts/camera_processing_node.py
├── launch/camera_processing.launch
├── config/camera_processing.yaml
└── test/test_rectify.py               # 13 tests
```

## 5. Required packages
`rospy std_msgs sensor_msgs diagnostic_msgs python3-numpy python3-opencv`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/camera/image_rect` | `sensor_msgs/Image` | pub |
| `/camera/camera_info_rect` | `sensor_msgs/CameraInfo` | pub (latched, adjusted) |
| `/camera/image_raw`, `/camera/camera_info` | sensor_msgs | sub |

## 9. Parameters / 10. Configuration
`config/camera_processing.yaml`: `rectify_mode (none|undistort_rectify)`,
`resize_scale`, `crop [x,y,w,h]`, `output_encoding (bgr8|rgb8|mono8)`,
`adjust_camera_info`, topics, diagnostics.

## 11. Python classes
`RectifyMapper` (build+apply maps), `build_rectify_maps_numpy`,
`build_crop_resize_maps_numpy`, `adjust_projection_matrix`,
`adjust_intrinsics`, `remap_numpy`, `distort_point_norm`.

## 12. Launch
```bash
roslaunch camera_processing camera_processing.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/camera_processing/test/test_rectify.py
# live:
rostopic hz /camera/image_rect
```

## 14. RViz configuration
`adaptive_amr/rviz/phase3_fusion.rviz`.

## 15. Expected outputs
Rectified image at configured size; `camera_info_rect` with adjusted P;
diagnostics with process time.

## 16. Performance metrics
`cv2.remap` ~2-5 ms/frame @1241×376; NumPy fallback slower (nearest).

## 17. Debugging guide
No output → check camera_info topic exists; `rectify_mode` wrong for already
rectified KITTI → use `none`.

## 18. Common errors
`Input image does not match camera_info size` → stale camera_info (latched
topic from another camera); restart calibration node.

## 19. Improvements
Stereo rectification via `cv2.stereoRectify`, `initUndistortRectifyMap`
alpha/ROI support, CUDA remap, sim-time clock.

## 20. Git commit
`feat(camera_processing): add camera rectification/processing pipeline`
