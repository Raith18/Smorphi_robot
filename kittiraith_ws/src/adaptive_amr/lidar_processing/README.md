# lidar_processing

> **Phase 3 — ✅ implemented**

## 1. Objective
Turn raw scans into usable 3D structure: filter, downsample, remove ground,
cluster obstacles and fit bounding boxes — pure NumPy/SciPy implementations of
the classic PCL pipeline.

## 2. Theory
- **RANSAC plane**: sample 3 pts → plane `n·p+d=0` → inliers `|n·p+d|<τ` →
  repeat → SVD refit. Normal canonicalized up (z>0).
- **Voxel grid**: `floor(p/leaf)` → per-cell centroid. O(n).
- **Euclidean clustering**: KD-tree (O(n log n)) + BFS within `tolerance`.
- **PCA OBB**: covariance eigenvectors = axes; extents = min/max of axis
  projections (tight box for any distribution).

## 3. Industrial importance
Ground removal + clustering is the classic obstacle-detection backbone for
AMR navigation (before learned detectors); OBBs feed trackers and planners.

## 4. Folder structure
```
lidar_processing/
├── src/lidar_processing/
│   ├── filters.py                # PassthroughFilter, VoxelGrid
│   ├── ground_segmentation.py    # RANSAC + SVD + GroundRemover
│   └── clustering.py             # EuclideanClusterExtraction, OBB, AABB
├── scripts/lidar_processing_node.py
├── launch/lidar_processing.launch
├── config/lidar_processing.yaml
└── test/test_lidar_processing.py # 13 tests
```

## 5. Required packages
`rospy std_msgs sensor_msgs visualization_msgs diagnostic_msgs
python3-numpy python3-scipy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/lidar_processing/obstacles` | `PointCloud2` | pub |
| `/lidar_processing/ground` | `PointCloud2` | pub |
| `/lidar_processing/clusters` | `PointCloud2` (+cluster_id) | pub |
| `/lidar_processing/boxes` | `MarkerArray` (cubes) | pub |
| `/lidar_processing/statistics` | `DiagnosticArray` | pub |
| `/velodyne_points` | `PointCloud2` | sub |

## 9. Parameters / 10. Configuration
`config/lidar_processing.yaml`: per-stage enable flags + thresholds
(`limits`, `voxel_leaf`, `ground_distance_threshold`, `cluster_tolerance`,
`box_min_dimension`, …).

## 11. Python classes
`PassthroughFilter`, `VoxelGrid`, `RansacPlaneSegmenter`, `GroundRemover`,
`fit_plane_svd`, `EuclideanClusterExtraction`, `OrientedBoundingBox`,
`AxisAlignedBoundingBox`.

## 12. Launch
```bash
roslaunch lidar_processing lidar_processing.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/lidar_processing/test/test_lidar_processing.py
# live:
rostopic hz /lidar_processing/obstacles
```

## 14. RViz configuration
`adaptive_amr/rviz/phase3_fusion.rviz` (obstacles red, ground green, boxes).

## 15. Expected outputs
Obstacle cloud (no ground), cluster_id-tagged clusters, 3D box markers,
diagnostics with point counts per stage.

## 16. Performance metrics
Voxel ~O(n) ≈ 2-5 ms; clustering 100k pts ≈ 20-60 ms (KD-tree);
RANSAC 50 iters ≈ 5-15 ms. Full pipeline comfortably < 100 Hz budget.

## 17. Debugging guide
Everything labeled ground → plane normal flip (fixed by canonicalization) or
threshold too high; no clusters → `cluster_tolerance` too small.

## 18. Common errors
`scipy missing` → `pip install scipy` (declared dependency).

## 19. Improvements
Ring-based ground segmentation (LineFit/ray grid), DBSCAN option, L-shape box
fitting, GPU voxelization (Open3D).

## 20. Git commit
`feat(lidar_processing): add ground segmentation, clustering and bounding boxes`
