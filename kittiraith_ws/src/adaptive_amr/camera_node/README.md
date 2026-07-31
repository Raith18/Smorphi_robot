# camera_node

> **Phase 2 — ✅ implemented**

## 1. Objective
Replay a KITTI camera sequence as ROS image topics, structured like a real
camera driver (owns the "sensor", publishes standard messages + stamps).

## 2. Theory
Pinhole camera model; KITTI `_sync` images are already rectified; JPEG decoded
to `sensor_msgs/Image` (bgr8) via OpenCV, or wrapped as `CompressedImage`.

## 3. Industrial importance
Every perception stack starts with a camera driver producing standardized
`Image` + `CameraInfo` topics. This node is that driver for dataset replay.

## 4. Folder structure
```
camera_node/
├── package.xml  CMakeLists.txt
├── scripts/camera_node.py
├── launch/camera_node.launch
└── config/camera_node.yaml
```

## 5. Required packages
`rospy std_msgs sensor_msgs dataset_loader python3-opencv python3-numpy`

## 6-8. ROS topics
| Topic | Type | Dir | Note |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | pub | frame `camera_optical_frame` |
| `/camera/image_rect` | `sensor_msgs/Image` | pub | same content (KITTI rectified) |

## 9. Parameters
`dataset_root, date, drive, camera_index (2), image_topic, rect_topic,
frame_id, encoding (image|compressed), publish_rect_copy, rate_factor, loop`

## 10. Configuration
`config/camera_node.yaml` — all of the above, loaded by launch.

## 11. Python classes
`CameraNode` — config → dataset access → pacing → publish loop.

## 12. Launch
```bash
roslaunch camera_node camera_node.launch camera_index:=2
roslaunch camera_node camera_node.launch encoding:=compressed rate:=0
```

## 13. Testing procedure
```bash
rostopic hz /camera/image_raw    # expect ~10 Hz at rate:=1
rostopic echo -n1 /camera/image_raw | head
```

## 14. RViz configuration
`adaptive_amr/rviz/kitti_sensors.rviz` — Camera + Image displays.

## 15. Expected outputs
10 Hz image stream, 1241×376 bgr8, stamps identical to KITTI `timestamps.txt`.

## 16. Performance metrics
~14 MB/s per topic @10 Hz (1241×376×3×10); decode ~2-5 ms/frame with OpenCV.

## 17. Debugging guide
No images → check `dataset_root`; `rostopic info`; missing frames log `Missing
image:` warnings (throttled).

## 18. Common errors
`ImportError: No module named cv2` → falls back to `compressed` encoding
(with a warning); install `python3-opencv` for raw images.

## 19. Improvements
Stereo publishing (cam 0+1), compressed-image transport hints, sim-time clock
support, rosbag-while-playing.

## 20. Git commit
`feat(camera_node): add KITTI camera driver node`
