# adaptive_amr_msgs

Custom messages & services shared by the adaptive_amr stack.

| Message | Phase | Purpose |
|---|---|---|
| `ObjectDetection.msg` / `ObjectDetectionArray.msg` | 4 | 2D detection + fused 3D depth/position |
| `ObjectTrack.msg` / `ObjectTrackArray.msg` | 4 | Multi-object tracking output |
| `DynamicObstacle.msg` / `DynamicObstacleArray.msg` | 6 | Predicted obstacle states (motion_prediction) |
| `Relocalize.srv` | 5 | Trigger re-localization (ICP reset / remap) |
| `SetGoal.srv` | 7 | Set the navigation goal (planning + behavior layers) |

**Phase 4:** perception messages defined. **Phase 5:** `Relocalize.srv` added.
**Phase 6:** `DynamicObstacle(Array).msg` added. **Phase 7:** `SetGoal.srv` added.
