# gps_node

> **Phase 2 — ✅ implemented**

## 1. Objective
Replay the GPS part of KITTI OXTS data as `sensor_msgs/NavSatFix` on
`/gps/fix`, like a real GNSS receiver driver.

## 2. Theory
OXTS logs WGS-84 latitude/longitude/altitude plus `pos_accuracy`, `navstat`
and `numsats`. The node maps those to `NavSatFix` (status, covariance
diagonal). GPS is *global but noisy* — later phases fuse it with IMU/odometry.

## 3. Industrial importance
Warehouse robots use GPS rarely (indoors), but outdoor AMRs and AGVs depend on
GNSS/RTK; the same `NavSatFix` interface is the industry standard.

## 4. Folder structure
```
gps_node/
├── package.xml  CMakeLists.txt
├── scripts/gps_node.py
├── launch/gps_node.launch
└── config/gps_node.yaml
```

## 5. Required packages
`rospy sensor_msgs dataset_loader`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/gps/fix` | `sensor_msgs/NavSatFix` | pub (frame `imu_link`) |

## 9. Parameters
`dataset_root, date, drive, fix_topic, frame_id, rate_factor, loop`

## 10. Configuration
`config/gps_node.yaml` — all of the above.

## 11. Python classes
`GpsNode` — OXTS reader + `NavSatFix` builder (shared `build_navsatfix_msg`).

## 12. Launch
```bash
roslaunch gps_node gps_node.launch
```

## 13. Testing procedure
```bash
rostopic echo -n1 /gps/fix | grep -E "latitude|longitude|status"
# Karlsruhe drive 0005: lat ≈ 49.00, lon ≈ 8.44 (degrees)
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` (or `rviz/NavSatFix` display in Phase 5).

## 15. Expected outputs
10 Hz `NavSatFix` with `STATUS_FIX`, covariance from OXTS `pos_accuracy`.

## 16. Performance metrics
Trivial (one 30-float line per frame).

## 17. Debugging guide
No fix → check `navstat`/`numsats` in the OXTS file (`kitti_parsers`),
`rostopic hz`.

## 18. Common errors
GPS at (0,0) → OXTS file unparsed (30 values required) — run unit tests.

## 19. Improvements
ENU conversion topic (`/gps/odom`), RTK-style covariance, geoid correction.

## 20. Git commit
`feat(gps_node): add KITTI GPS driver node (NavSatFix)`
