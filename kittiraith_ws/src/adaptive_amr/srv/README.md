# srv/ — custom services (adaptive_amr)

Custom request/reply services will be defined in `adaptive_amr_msgs`
(Phase 2), following the same convention as `msg/`.

Planned services:

| Service | Phase | Purpose |
|---|---|---|
| `GetCalibration.srv` | 2 | Query camera/LiDAR/IMU transforms |
| `Relocalize.srv` | 5 | Trigger re-localization |

Until then, modules use standard services where needed.
