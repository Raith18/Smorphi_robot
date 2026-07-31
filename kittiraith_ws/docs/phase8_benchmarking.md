# Phase 8 — Performance Benchmarking · Profiling · Optimization

> **Module status: ✅ COMPLETE**

Full tutorial: theory, the benchmark framework, the measured profiling table,
the optimization performed (with before/after numbers), testing, debugging,
performance and interview questions.

---

## 1. Objective

| Deliverable | Tool |
|---|---|
| Quantitative KITTI benchmarks | `evaluation` package: ATE/RPE, MOTA/MOTP, mIoU, grid precision/recall |
| CPU latency profiling | `evaluation/scripts/profile.py` (pure algorithms, any host) |
| Live ROS profiling | `evaluation/scripts/perf_collector.py` (diagnostics → CSV) |
| Optimization | measured, evidence-driven parameter/preset changes |

**Definition of done:** `profile.py` produces a per-algorithm latency table;
`benchmark.py` computes the stack's KITTI metrics against ground truth; the
optimization section documents at least one measured improvement.

## 2. Theory

### 2.1 ATE / RPE (odometry & localization)

```
ATE (TUM):  align est to GT with Umeyama (sim3) -> RMSE/mean/median of
            per-pose translation error
RPE (KITTI): for each i:  P_rel = inv(est_i)·est_{i+L}   Q_rel = inv(gt_i)·gt_{i+L}
            t_err = ||P_rel[:3,3] − Q_rel[:3,3]|| ,  r_err = angle(P_relᵀQ_rel)
```

- **Umeyama absorbs linear drift when `with_scale=True`** (proven by the unit
  test `test_scale_alignment_absorbs_linear_drift`) — a classic trap: scale
  alignment can hide odometry scale drift. Report both settings.
- RPE is the metric the **KITTI odometry benchmark** ranks on (sub-sequence
  lengths 100/200/... frames).

### 2.2 CLEAR MOT (tracking)

```
per frame: greedy match GT<->pred boxes by IoU (>= 0.5)
MOTA = 1 − (FP + FN + IDSW) / GT_count
MOTP = mean IoU over matches
```
A track-ID swap counts as **2 switches** (both identities switched).

### 2.3 mIoU (segmentation) & grid precision/recall

Per-class IoU = intersection/union, mean over present classes; grid
precision/recall ignores unknown (−1) cells.

## 3. Industrial importance

- **You cannot optimize what you don't measure.** Warehouse AMR teams keep
  per-module latency budgets (e.g., perception < 100 ms, planning < 50 ms)
  and run CI benchmarks on every merge — exactly what this phase adds.
- KITTI metrics (ATE/RPE/MOTA) are the *accepted currency* for comparing
  SLAM/tracking systems in papers and products.

## 4. Folder structure

```
src/adaptive_amr/evaluation/
├── src/evaluation/metrics.py        # pure metrics (ATE/RPE/MOT/mIoU/grid)
├── scripts/benchmark.py             # offline KITTI benchmark runner -> report.md
├── scripts/profile.py               # CPU latency profiler (pure algorithms)
├── scripts/perf_collector.py        # ROS diagnostics -> CSV collector
├── config/{benchmark,performance_presets}.yaml
├── launch/perf_collector.launch
└── test/{test_metrics,test_kitti_gt}.py   # 14 + 5 tests
```

## 5. Required packages
`rospy std_msgs diagnostic_msgs dataset_loader numpy scipy` (no new heavy deps)

## 6-8. ROS topics & parameters
`perf_collector` subscribes to `*/statistics` DiagnosticArrays; parameters in
`config/benchmark.yaml` (GT root, sequence, RPE lengths) and
`config/performance_presets.yaml` (accuracy/balanced/fast presets).

## 9-11. Python classes
`umeyama_alignment`, `absolute_trajectory_error`, `relative_pose_error`,
`mot_metrics`, `segmentation_iou`, `grid_precision_recall`,
`KittiOdometryPaths`/`read_odometry_poses` (dataset_loader).

## 12. Launch / usage

```bash
# CPU profile (any machine, no ROS):
python3 src/adaptive_amr/evaluation/scripts/profile.py --csv /tmp/perf.csv

# Offline KITTI benchmark:
python3 src/adaptive_amr/evaluation/scripts/benchmark.py \
    --odom est_poses.npz --gt-odom-root /data/kitti/odometry --seq 05

# Live ROS profiling:
roslaunch evaluation perf_collector.launch csv:=/tmp/perf.csv
```

## 13. Testing procedure

```bash
python3 src/adaptive_amr/evaluation/test/test_metrics.py     # 14
python3 src/adaptive_amr/evaluation/test/test_kitti_gt.py    # 5
```

## 14. Expected outputs

`profile.py` prints the latency table; `benchmark.py` writes
`evaluation/report/benchmark.md`; `perf_collector` writes a CSV timeline.

## 15. Performance metrics — MEASURED (this sandbox, 2-core VM, 3 repeats, median)

| module / algorithm | latency [ms] |
|---|---|
| voxel_downsample_100k | 110.0 |
| ransac_ground_30k | 24.9 |
| euclidean_cluster_20k | 149.5 |
| icp_point_to_plane_20k | 226.3 |
| projection_100k | 7.5 |
| depth_completion_1242x376 | 35.5 |
| sort_update_10_tracks | 0.6 |
| **occupancy_raycast_20k (BEFORE)** | **7629.9** |
| **occupancy_raycast_20k (AFTER)** | **401.0** |
| occupancy_raycast_5k (realistic) | 94.3 |
| costmap_inflation_300x300 | 2.4 |
| astar_300x300 | 2.7 |
| semantic_map_merge_20k | 48.3 |
| motion_prediction_20_tracks | 1.2 |

> Real hardware (modern laptop/desktop CPU) is typically 5-10x faster than
> this VM. All timings are median wall-clock, single-threaded.

## 16. Optimization — the measured improvement

**Finding:** the occupancy-grid ray casting used a per-point Bresenham Python
loop → **7.6 s** for 20k points (unusable).

**Fix (2 steps, each measured):**
1. **Vectorized ray sampling** — every ray is sampled at ≤ 0.5-cell steps and
   the free/occupied updates applied to a flattened grid in chunks of 2048
   points: 7629 → 676 ms (**11×**).
2. **`np.add.at` → `np.bincount`** for the scatter updates: 676 → 401 ms
   (**another 1.7×**; **19× total**).
3. Realistic input (5k points after voxel downsampling): **94 ms → ~11 Hz**
   feasible, comfortably inside the 10 Hz camera/LiDAR cadence.

**Result:** the occupancy grid node is now real-time-capable, validated by
re-running the same profiler + the unit tests (`test_grid_mapping.py` green).

## 17. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| `profile.py` imports fail | missing package on path | run from the workspace root |
| `benchmark.py` pose mismatch | est/GT lengths differ | script truncates to the shorter |
| perf_collector empty CSV | diagnostics topics not running | check nodes publish `*/statistics` |
| ATE ~0 with scale=True | linear drift absorbed | also report with_scale=False |

## 18. Common errors

1. Umeyama on < 3 poses → ValueError (need N ≥ 3).
2. RPE length > trajectory → ValueError; use lengths 100/200.
3. npz keys must be `poses`, `gt_boxes`, `pred_boxes`, etc. (see docstrings).

## 19. Improvements

- CI benchmark job (Phase 9) enforcing latency budgets.
- GPU profiling (torch.cuda) once a GPU exists.
- NDT vs ICP benchmark; OpenVINO YOLO latency.
- Full KITTI odometry leaderboard harness (all 11 sequences).

## 20. Interview questions

1. Why does Umeyama scale alignment hide linear odometry drift?
2. ATE vs RPE: when is each the right metric?
3. Why does a track-ID swap count as 2 ID switches in CLEAR MOT?
4. How do you profile a ROS node without slowing it down?
5. What does a 94 ms occupancy update mean for a 10 Hz pipeline?
6. Why is `np.bincount` faster than `np.add.at` for scatter updates?
7. How would you enforce a 100 ms perception latency budget in CI?
8. What is the KITTI odometry ranking metric? (RPE at lengths 100/200/...)
9. How do unknown cells affect grid precision/recall? (ignored)
10. How would you benchmark MOTA against the KITTI tracking devkit?

## 21. Git commits for this phase

```bash
feat(evaluation): add KITTI benchmark metrics (ATE/RPE, MOTA/MOTP, mIoU, grid)
feat(evaluation): add CPU profiler, benchmark runner and ROS perf collector
perf(occupancy_grid): vectorize ray casting + bincount scatter (19x faster)
docs(kittiraith_ws): add phase 8 documentation, tests and optimization report
```

---

**Phase 8 complete. Next: Phase 9 — Docker, CI/CD, Unit & Integration Testing.**
