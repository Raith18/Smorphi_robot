# adaptive_amr_msgs

Custom messages & services shared by the adaptive_amr stack.

| Message | Phase | Purpose |
|---|---|---|
| `ObjectDetection.msg` / `ObjectDetectionArray.msg` | 4 | 2D detection + fused 3D depth/position |
| `ObjectTrack.msg` / `ObjectTrackArray.msg` | 4 | Multi-object tracking output |
| `DynamicObstacle.msg` / `DynamicObstacleArray.msg` | 6 | Predicted obstacle states |
| `Relocalize.srv` | 5 | Trigger re-localization |

**Phase 4:** perception messages defined. Later phases extend this package
(DynamicObstacleArray in Phase 6, services in Phase 5/7).
