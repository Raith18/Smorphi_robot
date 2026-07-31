# Phase 4 — Detection · Tracking · Semantic Segmentation · Depth Estimation

> **Module status: ✅ COMPLETE**

Full tutorial: theory with mathematics, industrial context, every implemented
file, testing procedure, debugging, performance and interview questions.

---

## 1. Objective

| Module | Package | Input → Output |
|---|---|---|
| 2D detection | `object_detection` | image + LiDAR → `/object_detections` (boxes, classes, fused 3D depth) |
| Multi-object tracking | `object_tracking` | detections → `/object_tracks` (stable IDs) |
| Semantic segmentation | `semantic_segmentation` | image + LiDAR → `/semantic_map/labels` + colored points |
| Depth estimation | `depth_estimation` | sparse LiDAR depth → `/depth/dense` (dense depth) |

**Design constraint (your hardware):** no dedicated GPU — everything runs on
CPU with **YOLOv8n** / **YOLOv8n-seg** (nano, ~3.2M params) and classic
algorithms (SORT, nearest-fill depth). No learned depth networks.

**Definition of done:** `roslaunch adaptive_amr phase4_perception.launch`
produces all four outputs; RViz shows detections, tracks, semantics and dense
depth live on KITTI.

## 2. Theory

### 2.1 YOLOv8 (one-page summary)

YOLOv8 (Ultralytics) is a **single-stage anchor-free CNN detector**:

```
image (640x640)
  -> backbone:  CSPDarknet (multi-scale features)
  -> neck:      PAN-FPN (fuses features across scales)
  -> head:      per-cell predictions:
                   box:      (x, y, w, h) offsets from the cell
                   class:    softmax logits over C classes
                   (seg:     per-cell mask coefficients, YOLOv8-seg)
```

- **Anchor-free**: each grid cell directly predicts its object center —
  simpler and faster than anchor-based predecessors (YOLOv3/v5).
- **Loss**: classification (BCE), box (CIoU + DFL), segmentation (mask BCE).
- **NMS**: non-max suppression merges duplicate boxes.
- `yolov8n` = nano: 3.2M params, ~8.7 GFLOPs — **real-time on CPU** at the
  10 Hz replay rate (60-150 ms/frame depending on CPU).

### 2.2 SORT — Kalman + Hungarian (the math)

Each track is a constant-velocity Kalman filter over `x = [x, y, s, r, vx, vy, vs]`
(center, area, aspect ratio, velocities). Per frame:

```
Predict:   x' = F x ,          P' = F P Fᵀ + Q
Associate: cost(i,j) = 1 − IoU(box_i_det, box_j_trk)
           matched = Hungarian(cost)  with IoU ≥ τ
Update:    K = P' Hᵀ (H P' Hᵀ + R)⁻¹
           x = x' + K (z − H x') ,  P = (I − K H) P'
Lifecycle: unmatched detection → new track
           unmatched track      → time_since_update += 1; delete if ≥ max_age
           output only tracks with hit_streak ≥ min_hits (confirmed)
```

- ✅ ~O(1)/track, ~10 μs per frame at typical object counts; robust with
  high-quality detections.
- ❌ Constant-velocity model fails on sharp maneuvers; no appearance/re-ID —
  IDs swap on long occlusions (that's where DeepSORT/ByteTrack come in).

### 2.3 Frustum fusion (depth per detection)

For each 2D box, take the LiDAR points whose projections fall inside it
(Phase 3 projection), take the **median** depth (robust to outliers), then
back-project the box center:

```
p_cam   = depth · K⁻¹ · [u, v, 1]
p_laser = T_velo_cam⁻¹ · p_cam
```

### 2.4 Semantic segmentation (YOLOv8-seg)

The seg head predicts per-cell mask coefficients; masks are cropped to the
box, resized, thresholded → binary instance masks. We merge instances into a
class-label image (0 = background, class_id+1 otherwise; overlap resolved by
detection order = confidence order), colorize it, and **paint the labels onto
the LiDAR cloud** (PointPainting-style) by projecting each point into the
label image.

### 2.5 Depth completion (EDT nearest fill)

```
valid   = ¬isnan(sparse)
nearest = distance_transform_edt(valid, return_indices=True)
dense   = sparse[nearest]        (for pixels within max_fill_distance)
smooth  = Gaussian(dense)        (sigma=2 px)
```

- The Euclidean distance transform computes, for every pixel, the index of
  the nearest valid pixel in **O(n)** — this is the classic non-learned
  depth-completion baseline (S2D/Benchmark baseline).

## 3. Industrial importance

- YOLO-family detectors + SORT-style trackers are the de-facto baseline in
  warehouse AMR perception (cheap, real-time, well understood).
- Tracking gives objects **identity** — required for collision avoidance
  (predict where *that* person is going), handoff between cameras, and
  counting.
- Semantics on the LiDAR cloud is what lets planners reason about
  "drivable vs obstacle vs person" (Phases 6-7).
- Dense depth enables 3D reconstruction and costmap height layers.

## 4. Folder structure

```
src/adaptive_amr/
├── adaptive_amr_msgs/msg/          # ObjectDetection(.Array).msg, ObjectTrack(.Array).msg
├── object_detection/               # detection_utils.py (pure) + node + config/launch/test
├── object_tracking/                # sort.py (pure KF+Hungarian) + node + config/launch/test
├── semantic_segmentation/          # segmentation_utils.py (pure) + node + config/launch/test
├── depth_estimation/               # depth_completion.py (pure) + node + config/launch/test
├── launch/phase4_perception.launch
└── rviz/phase4_perception.rviz
```

## 5. Required packages

`rospy std_msgs sensor_msgs geometry_msgs visualization_msgs tf2_ros
message_filters diagnostic_msgs dataset_loader sensor_fusion adaptive_amr_msgs
numpy scipy opencv-python torch torchvision ultralytics`
(Phase 4 Python deps: `docker/pip_requirements_phase4.txt` — CPU wheels)

## 6. ROS topics (Phase 4 additions)

| Topic | Type | Pub |
|---|---|---|
| `/object_detections` | `adaptive_amr_msgs/ObjectDetectionArray` | object_detection |
| `/object_detections/image` | `Image` | object_detection |
| `/object_tracks` | `adaptive_amr_msgs/ObjectTrackArray` | object_tracking |
| `/object_tracks/markers` | `MarkerArray` | object_tracking |
| `/semantic_map/labels` (uint8) / `/semantic_map/colored` (bgr8) | `Image` | semantic_segmentation |
| `/semantic_map/colored_points` | `PointCloud2` (rgb) | semantic_segmentation |
| `/depth/dense` (32FC1) / `/depth/colored` (bgr8) | `Image` | depth_estimation |
| `…/statistics` | `DiagnosticArray` | all four |

## 7-8. TF & parameters
No new TF frames (fusion reuses `laser → camera_optical_frame`). All
parameters in each `config/*.yaml` (model, conf, imgsz, device, thresholds).

## 9-11. Python classes
`detection_utils` (boxes/IoU/frustum), `sort.KalmanBoxFilter/KalmanBoxTracker/
SortTracker`, `segmentation_utils` (labels/colors), `depth_completion`
(nearest_fill/smooth/colorize).

## 12. Launch

```bash
roslaunch adaptive_amr phase4_perception.launch               # full stack
roslaunch adaptive_amr phase4_perception.launch rate:=0       # fast
# first run downloads yolov8n.pt + yolov8n-seg.pt (internet required)
```

## 13. Testing procedure

```bash
# unit tests (no ROS, no torch)
python3 src/adaptive_amr/object_detection/test/test_detection_utils.py       # 13
python3 src/adaptive_amr/object_tracking/test/test_sort.py                    # 13
python3 src/adaptive_amr/semantic_segmentation/test/test_segmentation_utils.py # 7
python3 src/adaptive_amr/depth_estimation/test/test_depth_completion.py       # 10

# live (Ubuntu 20.04 + deps)
roslaunch adaptive_amr phase4_perception.launch rate:=1 &
rostopic hz /object_detections /object_tracks /semantic_map/labels /depth/dense
rostopic echo -n1 /object_tracks
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase4_perception.rviz
```

## 14. Expected outputs

- `/object_detections` at ~10 Hz (CPU-dependent): cars/pedestrians with fused
  depth; `/object_detections/image` annotated.
- `/object_tracks`: stable IDs across frames (same car keeps its ID).
- `/semantic_map/colored`: person/car/bike masks; colored_points painted.- `/depth/dense`: 32FC1 with coverage > 90% (unlimited fill).

## 15. Performance metrics (CPU, indicative)

| Stage | Latency |
|---|---|
| YOLOv8n detection (640px) | 60-150 ms |
| YOLOv8n-seg | 80-200 ms |
| SORT update | < 1 ms |
| Depth completion (1241×376) | 5-20 ms |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| `ultralytics is not installed` | Phase 4 deps missing | `pip install -r docker/pip_requirements_phase4.txt` (CPU index) |
| Model download fails | no internet first run | pre-download `yolov8n.pt`/`yolov8n-seg.pt`; pass `model:=/path/...` |
| No detections | conf too high / wrong topics | `conf_threshold:=0.25`; verify `/camera/image_rect` exists |
| Track IDs swap | SORT limitation (no re-ID) | expected; use `max_age` tuning; ByteTrack later |
| Depth all NaN | `/fusion/sparse_depth` absent | run sensor_fusion (Phase 3) first |
| CPU too slow at 10 Hz | 640px inference | `imgsz:=416`, `rate:=0.5` for replay |

## 17. Common errors

1. `torch: module compiled with ...` mismatch → reinstall CPU wheels with the
   documented index.
2. `adaptive_amr_msgs not found` → build order: `catkin build adaptive_amr_msgs`
   first (message package).
3. Memory spikes with 2 GB shared graphics → keep `imgsz:=416`, `max_det:=20`.
4. `cv2.resize` on masks shape mismatch → masks are resized to image size
   before thresholding (already handled).

## 18. Improvements

- **ByteTrack** (association with low-confidence boxes) — better occlusions.
- **DeepSORT / OSNet** appearance re-ID.
- **PointPainting full**: paint semantics into features, not just RGB.
- Learned depth completion (KBNet) once a GPU is available.
- Quantized/ONNX YOLOv8 (OpenVINO) for 2× CPU speed.

## 19. Interview questions

1. Why is YOLOv8 anchor-free, and what does that simplify?
2. Explain the Kalman predict/update equations in SORT. What do Q and R mean?
3. Why Hungarian assignment instead of greedy matching?
4. Why median depth rather than mean for frustum fusion?
5. Why does SORT lose IDs on long occlusions, and how would you fix it?
6. How does the distance transform fill depth in O(n)?
7. What does a "confirmed" track mean (min_hits)?
8. Why paint semantics onto the LiDAR cloud? (cross-modal features)
9. How would you measure MOTA/MOTP for the tracker?
10. What is NMS and why is it needed after YOLO inference?
11. CPU vs GPU latency for yolov8n at 640px — how would you profile?
12. Why keep aspect ratio fixed in the SORT state?
13. What happens to tracks when detections stop (occlusion)? (max_age)
14. How would you make depth completion robust to moving objects?
15. What custom messages did we add and why not reuse vision_msgs?

## 20. Git commits for this phase

```bash
feat(adaptive_amr_msgs): add detection and tracking message definitions
feat(object_detection): add YOLOv8 detection with LiDAR frustum fusion
feat(object_tracking): add SORT multi-object tracker (Kalman + Hungarian)
feat(semantic_segmentation): add YOLOv8-seg labels and semantic point painting
feat(depth_estimation): add sparse-to-dense depth completion (EDT fill)
chore(docker): add CPU-only Phase 4 Python requirements (torch + ultralytics)
docs(kittiraith_ws): add phase 4 documentation, tests, launch and RViz config
```

---

**Phase 4 complete. Next: Phase 5 — Visual Odometry, LiDAR Odometry & Sensor-Fusion Localization.**
