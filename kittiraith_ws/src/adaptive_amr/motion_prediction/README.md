# motion_prediction

> **Phase 6 — ✅ implemented**

## 1. Objective
Predict where tracked objects will be: per-track constant-velocity Kalman
filters produce trajectories on `/dynamic_obstacles` with a collision-risk
heuristic.

## 2. Theory
CV Kalman `x=[p, v]`: predict `x'=Fx, P'=FPFᵀ+Q` (F refreshed from the
measured dt — a bug the tests caught); update with the track position;
trajectory `p(τ)=p0+v·τ`; `collision_risk = clip(1 − d_min/safe, 0, 1)`.

## 3. Industrial importance
"Where will that person be in 2 s?" is the difference between reactive
stopping and safe proactive planning — required for industrial safety
certification and smooth AMR traffic.

## 4. Folder structure
```
motion_prediction/
├── src/motion_prediction/predictor.py   # pure math (7 tests)
├── scripts/motion_prediction_node.py
├── launch/motion_prediction.launch
├── config/motion_prediction.yaml
└── test/test_predictor.py
```

## 5. Required packages
`rospy std_msgs geometry_msgs diagnostic_msgs visualization_msgs
adaptive_amr_msgs numpy`

## 6-8. ROS topics
| Topic | Type | Dir |
|---|---|---|
| `/dynamic_obstacles` | `adaptive_amr_msgs/DynamicObstacleArray` | pub |
| `/dynamic_obstacles/markers` | `MarkerArray` | pub |
| `/dynamic_obstacles/statistics` | `DiagnosticArray` | pub |
| `/object_tracks` | `adaptive_amr_msgs/ObjectTrackArray` | sub |

## 9. Parameters / 10. Configuration
`config/motion_prediction.yaml`: `horizon_s`, `steps`, `process_noise`,
`measurement_noise`, `safe_distance`, `frame_id`.

## 11. Python classes
`MotionPredictor`, `ConstantVelocityFilter` — update / predict_trajectory /
collision_risk / prune / clear.

## 12. Launch
```bash
roslaunch motion_prediction motion_prediction.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/motion_prediction/test/test_predictor.py
# live:
rostopic echo -n1 /dynamic_obstacles | grep -E "track_id|collision_risk"
```

## 14. RViz configuration
`adaptive_amr/rviz/phase6_mapping.rviz` (trajectory markers, red = risky).

## 15. Expected outputs
One obstacle per track; `predicted_trajectory` sampled over the horizon;
`collision_risk` 0..1 (marker color green→red).

## 16. Performance metrics
< 1 ms for ≤ 20 tracks.

## 17. Debugging guide
No obstacles → no `/object_tracks` (enable perception); wild trajectories →
raise `measurement_noise` / lower `process_noise`.

## 18. Common errors
Frame mismatch (laser vs map) — the message carries the topic header frame;
consumers must transform.

## 19. Improvements
Interaction-aware prediction (social LSTM), lane/path constraints, uncertainty
ellipses in RViz, collision-time (TTC) output for the behavior layer.

## 20. Git commit
`feat(motion_prediction): add constant-velocity Kalman trajectory prediction`
