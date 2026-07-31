# Phase 3 — Camera Pipeline · LiDAR Pipeline · Fusion Pipeline

> **Module status: ✅ COMPLETE**

Full tutorial: theory with mathematics, industrial context, every implemented
file, testing procedure, debugging, performance and interview questions.

---

## 1. Objective

| Pipeline | Package | Input → Output |
|---|---|---|
| Camera | `camera_processing` | `/camera/image_raw` → rectified/processed `/camera/image_rect` + consistent `CameraInfo` |
| LiDAR | `lidar_processing` | `/velodyne_points` → ground/obstacles/clusters/boxes |
| Fusion | `sensor_fusion` | image + scan + calib → colored cloud, sparse depth, overlay |

**Definition of done:** `roslaunch adaptive_amr phase3_pipelines.launch`
produces `/camera/image_rect`, `/lidar_processing/{ground,obstacles,clusters,
boxes}` and `/fusion/{colored_points,sparse_depth,overlay}`; RViz shows the
colored cloud aligned with the camera image.

## 2. Theory

### 2.1 Camera processing math

Pinhole model: `s·[u,v,1]ᵀ = K·[R|t]·[X,Y,Z,1]ᵀ`, `K = [[fx,0,cx],[0,fy,cy],[0,0,1]]`.
Rectification map (per output pixel):
```
x = (u − c'x)/f'x ,  y = (v − c'y)/f'y          (rectified ray)
p_cam = R_rectᵀ · [x, y, 1]ᵀ                     (back to original camera)
x_d = x·(1 + k₁r² + k₂r⁴ + k₃r⁶) + 2p₁xy + p₂(r² + 2x²)
y_d = y·(1 + k₁r² + k₂r⁴ + k₃r⁶) + p₁(r² + 2y²) + 2p₂xy
u_src = fx·x_d + cx ,  v_src = fy·y_d + cy        (source pixel)
```
Implemented in pure NumPy (`build_rectify_maps_numpy`) **and** validated
against `cv2.initUndistortRectifyMap` in the unit tests. KITTI `_sync` images
are already rectified → `rectify_mode: none` (crop/resize only).

**CameraInfo consistency rule:** crop/resize changes the projection:
`f' = s·f`, `c' = s·(c−crop)`, `t' = s·t`. The node publishes
`/camera/camera_info_rect` with the adjusted P/K so fusion never uses stale
intrinsics.

### 2.2 LiDAR processing algorithms (cards)

**RANSAC plane (ground):**
```
repeat N:  sample p0,p1,p2 ; n = (p1-p0)×(p2-p0)/|..| ; d = −n·p0
           inliers = |n·p + d| < τ
keep model with max inliers ; refit via SVD on inliers
```
- ✅ Robust to >50% outliers · ❌ random, needs tuning
- Complexity O(N·n), memory O(n)
- Applications: ground removal on AMRs, plane detection in mapping
- Alternatives: ground-constrained SVD, ray-cast min-height grids, Patchwork++

**Voxel grid:**
```
voxel = floor(p / leaf) ; centroid of each cell
```
- ✅ O(n), uniform density · ❌ loses thin structures
- Alternatives: random sampling, farthest-point, learning-based (KPConv)

**Euclidean clustering:**
```
KD-tree (O(n log n)); BFS: neighbors of each point within `tolerance`
```
- ✅ exact radius neighborhoods · ❌ sensitive to `tolerance`; O(n) memory
- Alternatives: DBSCAN, region growing on normals, HDBSCAN

**PCA OBB:**
```
C = (1/n)Σ(p−μ)(p−μ)ᵀ ; axes = eigenvectors(C) ; extents = min/max of p·axes
```
- ✅ O(n), tight for boxes · ❌ poor for L-shaped objects
- Alternatives: L-shape fitting (Apollo/Waymo), minimum-volume box search

### 2.3 Fusion math

```
p_img = P_rect · R_rect4x4 · T_velo_cam · p_velo      (combined 3×4)
u = p_img[0]/p_img[2] , v = p_img[1]/p_img[2] , depth = p_img[2]
```
- **Sparse depth**: LiDAR points inside the image write `depth` into a 32FC1
  image (NaN elsewhere) — the seed for Phase 4 dense depth.
- **Colorization**: each point reads the pixel it lands on → RGB fields.
- **Overlay**: circles at projected pixels, jet-colored by depth.

## 3. Industrial importance

- The rectified camera + calibrated LiDAR + projection pipeline is the
  foundation of every modern detection/segmentation stack (Frustum PointNets,
  PointPainting, BEV fusion) — same math, deeper nets later.
- Ground-removed, clustered obstacle lists are what navigation costmaps eat
  (Phase 6/7).
- Sparse LiDAR depth is the training signal for monocular depth networks.

## 4. Folder structure

```
src/adaptive_amr/
├── camera_processing/  src/camera_processing/rectify.py · scripts/camera_processing_node.py
│                       launch/ config/ test/test_rectify.py
├── lidar_processing/   src/lidar_processing/{filters,ground_segmentation,clustering}.py
│                       scripts/lidar_processing_node.py · launch/ config/ test/
├── sensor_fusion/      src/sensor_fusion/projection.py · scripts/sensor_fusion_node.py
│                       launch/ config/ test/test_projection.py
├── launch/phase3_pipelines.launch
└── rviz/phase3_fusion.rviz
```

## 5. Required packages

`rospy std_msgs sensor_msgs geometry_msgs visualization_msgs diagnostic_msgs
tf2_ros tf2_geometry_msgs message_filters dataset_loader
python3-numpy python3-scipy python3-opencv`

## 6. ROS topics (Phase 3 additions)

| Topic | Type | Pub |
|---|---|---|
| `/camera/image_rect` | `Image` | camera_processing (owns it from Phase 3) |
| `/camera/camera_info_rect` | `CameraInfo` | camera_processing (adjusted) |
| `/lidar_processing/ground` · `/obstacles` | `PointCloud2` | lidar_processing |
| `/lidar_processing/clusters` | `PointCloud2` (+cluster_id) | lidar_processing |
| `/lidar_processing/boxes` | `MarkerArray` | lidar_processing |
| `/fusion/colored_points` | `PointCloud2` (rgb) | sensor_fusion |
| `/fusion/sparse_depth` | `Image` 32FC1 | sensor_fusion |
| `/fusion/overlay` | `Image` bgr8 | sensor_fusion |
| `/…/statistics` ×3 | `DiagnosticArray` | all three nodes |

## 7-8. TF & parameters
No new TF frames (fusion reads `laser → camera_optical_frame` from Phase 2
TF/calibration). All parameters in each package's `config/*.yaml` — see the
per-package READMEs.

## 9-11. Python classes
See §4 tree: `RectifyMapper`, `PassthroughFilter`, `VoxelGrid`,
`RansacPlaneSegmenter`, `GroundRemover`, `EuclideanClusterExtraction`,
`OrientedBoundingBox`, `LidarCameraProjection`.

## 12. Launch

```bash
roslaunch adaptive_amr phase3_pipelines.launch                 # full stack
roslaunch adaptive_amr phase3_pipelines.launch rate:=0         # fast
roslaunch adaptive_amr phase3_pipelines.launch sync_type:=approx
```

## 13. Testing procedure

```bash
# unit tests (no ROS)
python3 src/adaptive_amr/camera_processing/test/test_rectify.py        # 13
python3 src/adaptive_amr/lidar_processing/test/test_lidar_processing.py # 13
python3 src/adaptive_amr/sensor_fusion/test/test_projection.py          # 8

# live
roslaunch adaptive_amr phase3_pipelines.launch rate:=1 &
rostopic hz /camera/image_rect /lidar_processing/obstacles /fusion/sparse_depth
rostopic echo -n1 /lidar_processing/statistics
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase3_fusion.rviz
```

## 14. Expected outputs

- `image_rect` at 10 Hz (1241×376, or resized); `camera_info_rect` P consistent.
- Obstacles cloud without ground; cluster boxes around vehicles.
- `colored_points` aligned with the camera; `sparse_depth` with NaN holes;
  `overlay` showing depth-colored projections.
- Diagnostics: `lidar_processing` counts per stage; `sensor_fusion`
  `projection_ready=true`.

## 15. Performance metrics

| Stage | Latency (per frame, CPU) |
|---|---|
| Camera rectify (cv2 remap) | 2-5 ms |
| LiDAR passthrough+voxel | 2-5 ms |
| RANSAC ground | 5-15 ms |
| Clustering (KD-tree) | 20-60 ms @ 100k pts |
| Fusion projection + colorize | < 3 ms |
| Overlay (cv2) | 5-10 ms |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| Colored cloud misaligned with image | bad calib/TF | `tf_echo laser camera_optical_frame`; check `calibration` node |
| Everything is "ground" | normal flip or τ too high | canonicalization built-in; lower `ground_distance_threshold` |
| No clusters | tolerance too small | raise `cluster_tolerance`; check `min_cluster_size` |
| Sparse depth all NaN | wrong camera_info | fusion must use `/camera/camera_info_rect` after crop/resize |
| Two publishers on `/camera/image_rect` | camera_node still enabled | `publish_rect:=false` (phase3 launch does this) |

## 17. Common errors

1. `Input image does not match camera_info size` → stale latched CameraInfo.
2. `scipy is required for clustering` → install `python3-scipy`.
3. `TF lookup failed` → start calibration first or enable dataset-calib fallback.
4. RViz RGB8 shows grayscale → the rgb field must be the float32 view of
   packed uint32 (handled by `_pack_rgb`).

## 18. Improvements

- Dense depth: bilinear interpolation + dilation (Phase 4 depth estimation).
- PointPainting-style semantic augmentation (Phase 4 fusion).
- L-shape box fitting for vehicles; tracking-friendly box IDs (Phase 4).
- GPU acceleration (CUDA remap, cuML clustering).

## 19. Interview questions

1. Write the projection equation mapping a velodyne point to a pixel. (P·R·T)
2. Why must crop/resize change `P`, and how? (f′=s·f, c′=s·(c−crop), t′=s·t)
3. How does RANSAC find the ground plane, and why refit with SVD?
4. Why canonicalize the plane normal (z>0)?
5. Complexity of KD-tree clustering vs brute force? (O(n log n) vs O(n²))
6. Why is `2√λ` wrong for OBB extents of a filled cloud? (min/max projections)
7. What does a NaN in the sparse depth image mean?
8. How does RViz render an RGB field in PointCloud2? (packed uint32 as float32)
9. When is exact vs approximate time sync appropriate for fusion?
10. How would you validate camera-LiDAR alignment quantitatively?
11. What are the failure modes of PCA OBB on L-shaped vehicles?
12. Why does KITTI use `rectify_mode: none` by default?
13. What is the difference between early (point-level) and late (box-level)
    fusion? Which does this phase implement?
14. How would you downsample a scan without losing thin structures?
15. What does the `cluster_id` field enable downstream? (per-object features)

## 20. Git commits for this phase

```bash
feat(camera_processing): add camera rectification/processing pipeline
feat(lidar_processing): add ground segmentation, clustering and bounding boxes
feat(sensor_fusion): add camera-lidar projection, depth and colorization fusion
docs(kittiraith_ws): add phase 3 documentation, tests, launch and RViz config
```

---

**Phase 3 complete. Next: Phase 4 — Detection, Tracking, Semantic Segmentation & Depth Estimation.**
