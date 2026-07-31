# object_tracking

> **Phase 4 — ✅ implemented**

## 1. Objective
Multi-object tracking with **SORT**: per-track Kalman filters + Hungarian
assignment on IoU → stable object IDs on `/object_tracks`.

## 2. Theory
- KalmanBoxFilter (implemented in NumPy): constant-velocity 7-state filter
  over `[x, y, s, r, vx, vy, vs]`:
  predict `x'=Fx, P'=FPFᵀ+Q`; update `K=PHᵀ(HPHᵀ+R)⁻¹`.
- Association: Hungarian algorithm minimizes `1 − IoU`; matches below
  `iou_threshold` are rejected.
- Lifecycle: unmatched det → new track; unmatched track → delete after
  `max_age`; output only confirmed tracks (`hit_streak ≥ min_hits`).

## 3. Industrial importance
Identity is what separates "a person is there" from "that specific person is
walking toward aisle 3". SORT is the standard cheap baseline; industrial
trackers add appearance re-ID (DeepSORT/ByteTrack) on top.

## 4. Folder structure
```
object_tracking/
├── src/object_tracking/sort.py      # pure NumPy/SciPy (unit-tested)
├── scripts/object_tracking_node.py
├── launch/object_tracking.launch
├── config/object_tracking.yaml
└── test/test_sort.py                # 13 tests
```

## 5. Required packages
`rospy std_msgs geometry_msgs diagnostic_msgs visualization_msgs
adaptive_amr_msgs numpy scipy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/object_tracks` | `adaptive_amr_msgs/ObjectTrackArray` | pub |
| `/object_tracks/markers` | `visualization_msgs/MarkerArray` | pub |
| `/object_tracks/statistics` | `DiagnosticArray` | pub |
| `/object_detections` | `adaptive_amr_msgs/ObjectDetectionArray` | sub |

## 9. Parameters / 10. Configuration
`config/object_tracking.yaml`: `max_age`, `min_hits`, `iou_threshold`,
`publish_markers`, topics.

## 11. Python classes
`KalmanBoxFilter`, `KalmanBoxTracker`, `SortTracker`, `box_to_xyah`,
`xyah_to_box`, `iou_batch`.

## 12. Launch
```bash
roslaunch object_tracking object_tracking.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/object_tracking/test/test_sort.py
# live: a car keeps the same track_id across frames
rostopic echo -n1 /object_tracks
```

## 14. RViz configuration
`adaptive_amr/rviz/phase4_perception.rviz` (Tracks markers).

## 15. Expected outputs
Stable IDs; `age`/`hit_streak` per track; velocity estimate in `velocity`.

## 16. Performance metrics
< 1 ms per frame at ~10 objects.

## 17. Debugging guide
IDs flicker → `iou_threshold` too high or `max_age` too low; empty tracks →
detections not arriving (`rostopic hz /object_detections`).

## 18. Common errors
`scipy missing` → install `python3-scipy`; messages not generated → build
`adaptive_amr_msgs` first.

## 19. Improvements
ByteTrack low-confidence association, appearance re-ID, 3D (bird's-eye)
tracking, MOTA/MOTP evaluation harness (Phase 8).

## 20. Git commit
`feat(object_tracking): add SORT multi-object tracker (Kalman + Hungarian)`
