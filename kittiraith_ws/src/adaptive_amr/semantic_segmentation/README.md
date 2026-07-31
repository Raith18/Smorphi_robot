# semantic_segmentation

> **Phase 4 — ✅ implemented**

## 1. Objective
Per-pixel class labels with **YOLOv8-seg** (same library as detection):
label image, colored image, and labels painted onto the LiDAR cloud
(PointPainting-style).

## 2. Theory
YOLOv8-seg adds a mask head: per-cell mask coefficients + box-cropped mask
upsampling → instance masks. We merge instances into a class-id image
(overlap resolved by confidence order), colorize it, and project each LiDAR
point into the label image to color the cloud.

## 3. Industrial importance
Semantic point clouds are what planners use to distinguish obstacles from
drivable space and people from boxes (Phases 6-7). Painting semantics onto
LiDAR is the cheap version of PointPainting used by many AMR stacks.

## 4. Folder structure
```
semantic_segmentation/
├── src/semantic_segmentation/segmentation_utils.py  # pure helpers (tested)
├── scripts/semantic_segmentation_node.py
├── launch/semantic_segmentation.launch
├── config/semantic_segmentation.yaml
└── test/test_segmentation_utils.py                  # 7 tests
```

## 5. Required packages
`rospy sensor_msgs message_filters diagnostic_msgs dataset_loader
sensor_fusion numpy opencv-python torch torchvision ultralytics`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/semantic_map` | `sensor_msgs/Image` (uint8 labels) | pub |
| `/semantic_map/colored` | `sensor_msgs/Image` (bgr8) | pub |
| `/semantic_map/colored_points` | `sensor_msgs/PointCloud2` (rgb) | pub |
| `/semantic_map/statistics` | `DiagnosticArray` | pub |
| `/camera/image_rect`, `/velodyne_points`, `/camera/camera_info_rect` | — | sub |

## 9. Parameters / 10. Configuration
`config/semantic_segmentation.yaml`: `model (yolov8n-seg.pt)`,
`conf_threshold`, `imgsz`, `device`, `max_det`, `publish_colored_points`.

## 11. Python classes
`segmentation_utils`: `build_label_image`, `colorize_labels`, `class_color`,
fixed `CLASS_COLORS` palette.

## 12. Launch
```bash
roslaunch semantic_segmentation semantic_segmentation.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/semantic_segmentation/test/test_segmentation_utils.py
# live:
rostopic hz /semantic_map
```

## 14. RViz configuration
`adaptive_amr/rviz/phase4_perception.rviz`.

## 15. Expected outputs
Label/colored images; colored LiDAR where cars are blue, people red-ish, etc.

## 16. Performance metrics
YOLOv8n-seg @640px CPU: 80-200 ms.

## 17. Debugging guide
All-black map → model not producing masks (`yolov8n-seg.pt` — NOT the det
model); colored points missing → calibration/TF (see sensor_fusion).

## 18. Common errors
Wrong model file → masks empty; use the `-seg` variant.

## 19. Improvements
Share one inference server across detection+segmentation; true semantic
classes (road/sidewalk) via KITTI semantic devkit; learned PointPainting.

## 20. Git commit
`feat(semantic_segmentation): add YOLOv8-seg labels and semantic point painting`
