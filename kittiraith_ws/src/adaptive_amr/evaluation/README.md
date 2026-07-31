# evaluation/ — benchmarking (adaptive_amr)

Quantitative evaluation against KITTI ground truth (Phase 8 formalizes this):

| Metric | Used for |
|---|---|
| ATE / RPE (KITTI odometry benchmark) | visual & LiDAR odometry, localization |
| 3D IoU / mAP | detection |
| mIoU | semantic segmentation |
| MOTA / MOTP | tracking |
| Precision / recall of occupied cells | occupancy grid |

Every module ships with an evaluation script in this folder, so Phase 8
benchmarking is just running them.
