# adaptive_amr_msgs

Custom messages & services shared by the adaptive_amr stack.

| Planned message | Phase | Purpose |
|---|---|---|
| `ObjectTrack.msg` / `ObjectTrackArray.msg` | 4 | Multi-object tracking output |
| `DynamicObstacle.msg` / `DynamicObstacleArray.msg` | 6 | Predicted obstacle states |
| `SemanticLabel.msg` | 6 | Per-point semantic labels |
| `GetCalibration.srv` | 2 | (superseded by topics in Phase 2) |
| `Relocalize.srv` | 5 | Trigger re-localization |

**Note:** Phase 2 ships without custom messages — the sensor layer uses only
standard types (`sensor_msgs`, `geometry_msgs`, `tf2_msgs`). Definitions are
added here as soon as a module needs them.
