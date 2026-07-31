# time_sync

> **Phase 2 — ✅ implemented**

## 1. Objective
Gatekeeper of multi-sensor fusion: emit only sets of camera/LiDAR/IMU/GPS
messages that describe the *same instant*, plus sync-health diagnostics.

## 2. Theory
- **Exact sync** (`message_filters.TimeSynchronizer`): requires identical
  stamps — correct for KITTI `_sync` drives (one shared timestamp per frame).
- **Approx sync** (`ApproximateTimeSynchronizer`): pairs messages within
  `approx_slop` — what real robots need (independent sensor clocks).
- Latency metric = max−min stamp within a synced set (0 s for exact).

## 3. Industrial importance
Sensor fusion (camera-lidar projection, EKF localization) assumes inputs
describe the same time. At 10 Hz, a 100 ms misalignment is a full frame of
error — every autonomy stack has a synchronization layer.

## 4. Folder structure
```
time_sync/
├── package.xml  CMakeLists.txt
├── scripts/time_sync_node.py
├── launch/time_sync.launch
└── config/time_sync.yaml
```

## 5. Required packages
`rospy std_msgs sensor_msgs diagnostic_msgs message_filters`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/time_sync/image` | `sensor_msgs/Image` | pub (synced) |
| `/time_sync/points` | `sensor_msgs/PointCloud2` | pub (synced) |
| `/time_sync/imu` | `sensor_msgs/Imu` | pub (synced) |
| `/time_sync/gps` | `sensor_msgs/NavSatFix` | pub (synced) |
| `/time_sync/statistics` | `DiagnosticArray` | pub @1 Hz |
| `/camera/image_raw` etc. | sensor_msgs | sub (4 streams) |

## 9. Parameters
`image_topic, points_topic, imu_topic, subscribe_gps, gps_topic, sync_type
(exact|approx), queue_size, approx_slop, diagnostics_rate, synced_ns`

## 10. Configuration
`config/time_sync.yaml` — all of the above.

## 11. Python classes
`TimeSyncNode` — message_filters wiring, callback, diagnostics.

## 12. Launch
```bash
roslaunch time_sync time_sync.launch                 # exact
roslaunch time_sync time_sync.launch sync_type:=approx
```

## 13. Testing procedure
```bash
rostopic hz /time_sync/image            # ~10 Hz
rostopic echo -n1 /time_sync/statistics # synced_sets increasing, latency 0
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` — use `/time_sync/*` topics to verify.

## 15. Expected outputs
10 Hz synced sets; diagnostics `OK`; `max_set_latency_s == 0` (exact).

## 16. Performance metrics
Zero added latency (pure republish); memory bounded by queue size (20 msgs).

## 17. Debugging guide
No synced sets → stamps differ (check `header.stamp` equality); increase
`approx_slop`; check `rostopic info` that all 4 topics exist.

## 18. Common errors
`AttributeError: 'NoneType'` in exact mode → a stream stopped; GPS topic
missing → set `subscribe_gps:=false`.

## 19. Improvements
Custom synced message with all payloads, time-offset estimation
(clock-skew calibration), `/clock` (sim time) support, latency histograms.

## 20. Git commit
`feat(time_sync): add exact/approximate multi-sensor synchronization node`
