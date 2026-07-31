# evaluation

> **Phase 8 — ✅ implemented**

## 1. Objective
Quantitative benchmarking, CPU profiling and performance reporting for every
module: KITTI ATE/RPE (odometry & localization), MOTA/MOTP (tracking), mIoU
(segmentation), grid precision/recall, plus a live ROS diagnostics collector.

## 2. Theory
- ATE: Umeyama alignment → per-pose translation RMSE (scale absorbs linear
  drift — report both settings).
- RPE (KITTI): relative motion error over sub-sequences of length L.
- CLEAR MOT: greedy IoU matching; `MOTA = 1 − (FP+FN+IDSW)/GT`; a swap = 2 IDSW.
- Profiling: median wall-clock over repeats; optimization = measured change.

## 3. Industrial importance
You can't optimize what you don't measure — CI latency budgets and KITTI
metrics are the acceptance currency of industrial autonomy.

## 4. Folder structure
```
evaluation/
├── src/evaluation/metrics.py      # pure metrics (14 tests)
├── scripts/benchmark.py           # offline KITTI benchmark runner
├── scripts/profile.py             # CPU latency profiler (pure algorithms)
├── scripts/perf_collector.py      # ROS diagnostics -> CSV
├── config/{benchmark,performance_presets}.yaml
├── launch/perf_collector.launch
└── test/{test_metrics,test_kitti_gt}.py
```

## 5. Required packages
`rospy std_msgs diagnostic_msgs dataset_loader numpy scipy`

## 6-8. ROS topics & parameters
`perf_collector` subscribes to all `*/statistics`; parameters in
`config/benchmark.yaml` + `config/performance_presets.yaml` (accuracy /
balanced / fast presets).

## 9-11. Python classes
`umeyama_alignment`, `absolute_trajectory_error`, `relative_pose_error`,
`mot_metrics`, `segmentation_iou`, `grid_precision_recall`; GT parsers in
`dataset_loader` (`KittiOdometryPaths`, `read_odometry_poses`).

## 12. Launch / usage
```bash
python3 src/adaptive_amr/evaluation/scripts/profile.py --csv /tmp/perf.csv
python3 src/adaptive_amr/evaluation/scripts/benchmark.py \
    --odom est.npz --gt-odom-root /data/kitti/odometry --seq 05
roslaunch evaluation perf_collector.launch csv:=/tmp/perf.csv
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/evaluation/test/test_metrics.py     # 14
python3 src/adaptive_amr/evaluation/test/test_kitti_gt.py    # 5
```

## 14. RViz configuration
Not needed (report/markdown output).

## 15. Expected outputs
Latency table (see `docs/phase8_benchmarking.md` §15), `benchmark.md` report,
CSV timelines.

## 16. Performance metrics
See `docs/phase8_benchmarking.md` — including the 19× occupancy-raycast
optimization (7629 → 401 ms; realistic 5k points = 94 ms ≈ 11 Hz).

## 17. Debugging guide
Import errors → run from the workspace root; empty CSV → diagnostics topics
not publishing.

## 18. Common errors
Umeyama needs N ≥ 3 poses; RPE length must be < trajectory length.

## 19. Improvements
CI benchmark job (Phase 9), GPU profiling, full 11-sequence KITTI harness.

## 20. Git commit
`feat(evaluation): add KITTI benchmark metrics, CPU profiler and perf collector`
