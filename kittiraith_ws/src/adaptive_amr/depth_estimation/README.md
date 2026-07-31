# depth_estimation

> **Phase 4 — ✅ implemented**

## 1. Objective
Convert the sparse LiDAR depth image (`/fusion/sparse_depth`) into a dense
depth map (`/depth/dense`) using the classic nearest-valid-pixel fill +
smoothing — the standard non-learned depth-completion baseline.

## 2. Theory
- `nearest_fill`: `scipy.ndimage.distance_transform_edt(valid,
  return_indices=True)` gives every pixel the index of its nearest valid
  pixel in O(n); copy that depth (optionally capped by `max_fill_distance`).
- `smooth_depth`: Gaussian blur (cv2 if available, scipy fallback).
- `colorize_depth`: jet colormap in pure NumPy.

## 3. Industrial importance
Dense depth is required by 3D reconstruction, costmap height layers and
safety zones. The EDT baseline is exactly what benchmark papers (KITTI depth
completion) compare against — knowing it makes learned models meaningful.

## 4. Folder structure
```
depth_estimation/
├── src/depth_estimation/depth_completion.py   # pure math (unit-tested)
├── scripts/depth_estimation_node.py
├── launch/depth_estimation.launch
├── config/depth_estimation.yaml
└── test/test_depth_completion.py              # 10 tests
```

## 5. Required packages
`rospy sensor_msgs diagnostic_msgs numpy scipy opencv-python`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/depth/dense` | `sensor_msgs/Image` (32FC1) | pub |
| `/depth/colored` | `sensor_msgs/Image` (bgr8) | pub |
| `/depth/statistics` | `DiagnosticArray` | pub |
| `/fusion/sparse_depth` | `sensor_msgs/Image` (32FC1) | sub |

## 9. Parameters / 10. Configuration
`config/depth_estimation.yaml`: `max_fill_distance`, `smooth_sigma`,
`visualize_max_depth`, topics.

## 11. Python classes
`nearest_fill`, `smooth_depth`, `colorize_depth`.

## 12. Launch
```bash
roslaunch depth_estimation depth_estimation.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/depth_estimation/test/test_depth_completion.py
# live:
rostopic hz /depth/dense
rostopic echo -n1 /depth/statistics | grep coverage
```

## 14. RViz configuration
`adaptive_amr/rviz/phase4_perception.rviz`.

## 15. Expected outputs
Dense 32FC1 depth (coverage > 90% with unlimited fill), jet-colored view.

## 16. Performance metrics
EDT fill 1241×376: 5-20 ms (O(n)).

## 17. Debugging guide
Coverage 0% → sparse input all-NaN (run sensor_fusion first).

## 18. Common errors
NaNs remain → `max_fill_distance` too small; input encoding not 32FC1.

## 19. Improvements
Guided/learned completion (KBNet, S2D), depth confidence channel, temporal
fusion across frames.

## 20. Git commit
`feat(depth_estimation): add sparse-to-dense depth completion (EDT fill)`
