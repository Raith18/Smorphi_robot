# rviz/ — RViz configurations (adaptive_amr)

RViz display configs for visualizing the whole stack:

| File (planned) | Shows |
|---|---|
| `rviz/perception.rviz` | images, point clouds, TF (Phase 2) |
| `rviz/detection.rviz` | detections, tracks, segmentation (Phase 4) |
| `rviz/localization.rviz` | odometry, localization pose, maps (Phase 5/6) |
| `rviz/navigation.rviz` | costmaps, dynamic obstacles (Phase 7) |

Open with:

```bash
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/perception.rviz
```
