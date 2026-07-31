# object_detection

> **Phase 4 — ✅ implemented**

## 1. Objective
2D object detection with **YOLOv8** fused with LiDAR depth: pixel boxes,
classes, scores + fused 3D depth/position (frustum fusion).

## 2. Theory
- YOLOv8: single-stage anchor-free CNN (backbone CSPDarknet → neck PAN-FPN →
  head predicting boxes/classes per cell). `yolov8n` = nano (3.2M params),
  CPU-friendly at 10 Hz.
- Frustum fusion: project the Velodyne cloud into the image (Phase 3 math),
  take the **median** depth of points inside each box, back-project the box
  center: `p_laser = T_velo_cam⁻¹ · (depth · K⁻¹ · [u,v,1])`.

## 3. Industrial importance
YOLO-family detectors are the standard 2D baseline in warehouse AMR stacks —
cheap, real-time, easily retrained on custom classes. Fused depth turns a
2D box into a 3D cue for the tracker and the planner.

## 4. Folder structure
```
object_detection/
├── src/object_detection/detection_utils.py   # pure math (unit-tested)
├── scripts/object_detection_node.py
├── launch/object_detection.launch
├── config/object_detection.yaml
└── test/test_detection_utils.py              # 13 tests
```

## 5. Required packages
`rospy sensor_msgs geometry_msgs tf2_ros message_filters diagnostic_msgs
dataset_loader sensor_fusion adaptive_amr_msgs
numpy opencv-python torch torchvision ultralytics` (Phase 4 reqs)

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/object_detections` | `adaptive_amr_msgs/ObjectDetectionArray` | pub |
| `/object_detections/image` | `sensor_msgs/Image` | pub (annotated) |
| `/object_detections/statistics` | `DiagnosticArray` | pub |
| `/camera/image_rect`, `/velodyne_points`, `/camera/camera_info_rect` | — | sub |

## 9. Parameters / 10. Configuration
`config/object_detection.yaml`: `model (yolov8n.pt)`, `conf_threshold`,
`imgsz`, `device (cpu|auto)`, `max_det`, `target_classes`, topics.

## 11. Python classes
`detection_utils`: `clip_box`, `box_iou`, `box_center`, `points_in_box`,
`median_depth_in_box`, `backproject_to_laser`, COCO/KITTI class maps.

## 12. Launch
```bash
roslaunch object_detection object_detection.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/object_detection/test/test_detection_utils.py
# live:
rostopic hz /object_detections
rostopic echo -n1 /object_detections
```

## 14. RViz configuration
`adaptive_amr/rviz/phase4_perception.rviz`.

## 15. Expected outputs
10 Hz detections with `label/score/bbox/depth/position`; annotated image.

## 16. Performance metrics
YOLOv8n @640px CPU: 60-150 ms; frustum fusion < 2 ms.

## 17. Debugging guide
No detections → `conf_threshold` too high, or `target_classes` excludes
everything; check `/object_detections/image`.

## 18. Common errors
`ultralytics missing` → install `pip_requirements_phase4.txt` (CPU index);
model download needs internet on first run.

## 19. Improvements
ONNX/OpenVINO export (2× CPU speed), ByteTrack-compatible output, class-aware
NMS, multi-camera fusion.

## 20. Git commit
`feat(object_detection): add YOLOv8 detection with LiDAR frustum fusion`
