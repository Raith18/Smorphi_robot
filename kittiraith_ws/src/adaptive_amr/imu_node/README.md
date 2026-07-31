# imu_node

> **Phase 2 — ✅ implemented**

## 1. Objective
Replay the inertial part of KITTI OXTS data as `sensor_msgs/Imu` on
`/imu/data`, like a real IMU driver.

## 2. Theory
OXTS provides fused roll/pitch/yaw (GPS-aided INS), angular rates
(`wx, wy, wz` rad/s) and accelerations (`ax, ay, az` m/s²) in the
vehicle/IMU frame (x forward, y left, z up). The node publishes all three in
`sensor_msgs/Imu` with engineering-estimated covariances (tunable in YAML).

## 3. Industrial importance
IMUs give the highest-rate motion data (100-1000 Hz on real robots) and
bridge GPS outages; they are the backbone of every localization filter.

## 4. Folder structure
```
imu_node/
├── package.xml  CMakeLists.txt
├── scripts/imu_node.py
├── launch/imu_node.launch
└── config/imu_node.yaml
```

## 5. Required packages
`rospy sensor_msgs dataset_loader`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | pub (frame `imu_link`) |

## 9. Parameters
`dataset_root, date, drive, imu_topic, frame_id, rate_factor, loop`

## 10. Configuration
`config/imu_node.yaml` — all of the above.

## 11. Python classes
`ImuNode` — OXTS reader + `Imu` builder (shared `build_imu_msg`).

## 12. Launch
```bash
roslaunch imu_node imu_node.launch
```

## 13. Testing procedure
```bash
rostopic echo -n1 /imu/data | grep -A2 orientation
# quaternion norm should be ~1.0; angular_velocity ~ 0.005 rad/s at standstill
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` — TF axes show IMU orientation.

## 15. Expected outputs
10 Hz `Imu` messages, normalized quaternion, gravity-free accelerations.

## 16. Performance metrics
Trivial (one line per frame). Real robots: 100-1000 Hz streams.

## 17. Debugging guide
Non-unit quaternion → parser bug (run unit tests); NaN → missing OXTS values.

## 18. Common errors
Orientation flips between +/− → Euler convention mismatch — the project uses
`R = Rz(yaw)·Ry(pitch)·Rx(roll)` (matches KITTI devkit).

## 19. Improvements
Bias/scale modeling, gravity compensation toggle, raw-vs-filtered topics,
100 Hz interpolation (oxts unsync data).

## 20. Git commit
`feat(imu_node): add KITTI IMU driver node (sensor_msgs/Imu)`
