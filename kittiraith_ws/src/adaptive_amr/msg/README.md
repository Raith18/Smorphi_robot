# msg/ — custom messages (adaptive_amr)

A **metapackage cannot define messages** (it contains no build code), so custom
messages shared by several modules will live in a dedicated package
`adaptive_amr_msgs`, created in **Phase 2**.

Planned messages (updated as phases land):

| Message | Phase | Purpose |
|---|---|---|
| `ObjectTrack.msg` / `ObjectTrackArray.msg` | 4 | Multi-object tracking output |
| `DynamicObstacle.msg` / `DynamicObstacleArray.msg` | 6 | Predicted obstacle states |
| `SemanticLabel.msg` | 6 | Per-point semantic labels |

Until then, modules use standard types (`sensor_msgs`, `nav_msgs`,
`geometry_msgs`, `vision_msgs`).
