# adaptive_amr (metapackage)

Aggregates all 20 pipeline modules of the **Adaptive Multi-Sensor Perception,
Localization and Semantic Mapping Framework**.

```
camera_node  lidar_node  gps_node  imu_node        # Phase 2: sensor drivers
dataset_loader  time_sync  calibration             # Phase 2: data + sync
camera_processing  lidar_processing  sensor_fusion # Phase 3: pipelines
object_detection  semantic_segmentation            # Phase 4: perception
object_tracking  depth_estimation                 # Phase 4
visual_odometry  lidar_odometry  localization      # Phase 5: SLAM + fusion
semantic_mapping  occupancy_grid  motion_prediction # Phase 6: mapping + prediction
navigation_layer                                  # Phase 7: navigation
```

Custom messages/services used by several modules will live in a dedicated
`adaptive_amr_msgs` package (added in Phase 2) — a metapackage cannot define msgs.
