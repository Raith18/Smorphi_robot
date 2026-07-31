# calibration

> **Phase 2 — ✅ implemented**

## 1. Objective
Publish KITTI calibration as ROS artifacts: `CameraInfo` for every camera and
the static TF tree — so all later modules can look up any transform or
intrinsic on the ROS bus instead of parsing files themselves.

## 2. Theory
- Camera: `P_rect_0X` (3×4 rectified projection), `R_rect_00`, `K`, `D` →
  `sensor_msgs/CameraInfo` (`plumb_bob` distortion model).
- Extrinsics: `T_velo_cam0` (laser→camera_link), `T_imu_velo`; chained
  `T_imu_cam0 = T_velo_cam0 · T_imu_velo` for camera_link→imu_link.
- TF tree: `map → odom → base_link → laser → camera_link →
  camera_optical_frame` and `camera_link → imu_link`.

## 3. Industrial importance
Calibration is *the* difference between fused perception that works and
catastrophic misalignment. Industrial stacks version calibration per-robot
(serial-numbered) and serve it over the ROS bus exactly like this.

## 4. Folder structure
```
calibration/
├── package.xml  CMakeLists.txt
├── scripts/calibration_node.py
├── launch/calibration.launch
└── config/calibration.yaml
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs tf2_ros dataset_loader`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/camera_00..03/camera_info` | `sensor_msgs/CameraInfo` | pub (latched) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | pub (alias for cam 02) |
| `/tf_static` | `tf2_msgs/TFMessage` | pub |

## 9. Parameters
`dataset_root, date, drive, camera_info_ns, publish_placeholders, map_frame,
odom_frame, base_frame, laser_frame, camera_frame, camera_optical_frame,
imu_frame`

## 10. Configuration
`config/calibration.yaml` — frame names + namespaces (project TF tree).

## 11. Python classes
`CalibrationNode` — loads `KittiCalib`, publishes CameraInfo + static TF.

## 12. Launch
```bash
roslaunch calibration calibration.launch
```

## 13. Testing procedure
```bash
rostopic echo -n1 /camera/camera_info | grep -E "width|height|P"
rosrun tf tf_echo laser camera_optical_frame   # print the transform
rosrun tf view_frames                          # visualize the TF tree
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` — TF display shows all frames.

## 15. Expected outputs
Latched CameraInfo (1242×376, P with -3.9e2 baseline term for cam 02),
`/tf_static` with 6 transforms, TF tree: `map→odom→base_link→laser→
camera_link→(camera_optical_frame, imu_link)`.

## 16. Performance metrics
Latched topics = zero steady-state bandwidth; TF lookup < 1 ms.

## 17. Debugging guide
`tf_echo` fails → run calibration node; `TF_OLD_DATA` → run `/use_sim_time`
consistently or leave it off for replay.

## 18. Common errors
Wrong calib files for the date → `Missing key` error; validator catches it.

## 19. Improvements
Dynamic reconfiguration, odometry-format calib support, calibration
quality metrics (reprojection error), ROS services for querying calib.

## 20. Git commit
`feat(calibration): publish KITTI CameraInfo and static TF tree`
