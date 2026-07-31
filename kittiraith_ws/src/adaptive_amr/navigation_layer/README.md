# navigation_layer

> **Phase 7 — ✅ implemented** (Navigation · Behavior · Decision)

## 1. Objective
The final autonomy layer: build the navigation costmap (inflation), plan A*
paths, and run the safety behavior state machine that gates the velocity
command.

## 2. Theory
- Costmap: `cost = 100·(1 − d/R)` with d = EDT distance to nearest lethal
  cell; dynamic obstacles marked lethal before inflation.
- A*: 8-connectivity, Euclidean heuristic (admissible → optimal),
  min-heap open list; lethal blocked, unknown penalized.
- Behavior: `INIT → NAVIGATE ⇄ AVOID ⇄ NAVIGATE`, `→ STOP → RESUME`,
  `→ GOAL_REACHED`, driven by `max_risk` + goal distance; decision layer
  emits `(v, ω)` with a heading-error P-controller.

## 3. Industrial importance
Planner-proposes/behavior-vetoes is the safety-critical architecture of real
AMRs; the costmap+path+cmd output is exactly what move_base-style stacks
produce and what warehouse planners consume.

## 4. Folder structure
```
navigation_layer/
├── src/navigation_layer/{costmap,planner,behavior}.py
├── scripts/{navigation_layer_node,behavior_node}.py
├── launch/{navigation_layer,behavior}.launch
├── config/{navigation_layer,behavior}.yaml
└── test/{test_costmap,test_planner,test_behavior}.py    # 23 tests
```

## 5. Required packages
`rospy geometry_msgs nav_msgs tf2_ros diagnostic_msgs adaptive_amr_msgs
numpy scipy`

## 6-8. ROS topics & services
| Topic | Type | Dir |
|---|---|---|
| `/navigation_costmap` | `OccupancyGrid` | nav node pub |
| `/planned_path` | `Path` | nav node pub |
| `/behavior/state` | `String` | behavior pub |
| `/behavior/velocity_command` | `Twist` | behavior pub |
| `/goal` | `PoseStamped` | sub (both) |
| `/navigation_layer/set_goal` | `adaptive_amr_msgs/SetGoal` | service |

## 9. Parameters / 10. Configuration
`config/navigation_layer.yaml`: `inflation_radius`, `lethal_threshold`,
`unknown_penalty`, `plan_interval`. `config/behavior.yaml`: `risk_stop`,
`risk_avoid`, `goal_tolerance`, `cruise_speed`, `kp_angular`, `control_rate`.

## 11. Python classes
`CostmapBuilder`, `AStarPlanner`, `BehaviorStateMachine`,
`NavigationLayerNode`, `BehaviorNode`.

## 12. Launch
```bash
roslaunch navigation_layer navigation_layer.launch
roslaunch navigation_layer behavior.launch
```

## 13. Testing procedure
```bash
python3 src/adaptive_amr/navigation_layer/test/test_costmap.py    # 6
python3 src/adaptive_amr/navigation_layer/test/test_planner.py    # 5
python3 src/adaptive_amr/navigation_layer/test/test_behavior.py   # 12
```

## 14. RViz configuration
`adaptive_amr/rviz/phase7_navigation.rviz`.

## 15. Expected outputs
Inflated costmap; A* path around obstacles; behavior state transitions;
gated velocity command.

## 16. Performance metrics
Inflation 5-20 ms; A* 10-100 ms; behavior < 1 ms @10 Hz.

## 17. Debugging guide
No path → goal/TF; stuck STOP → risk never clears (check dynamic obstacles).

## 18. Common errors
Goal in wrong frame; costmap all -1 (occupancy not publishing); START not
localized.

## 19. Improvements
DWA local planner, TTC in decision, semantic per-class costs, typed behavior
enum.

## 20. Git commit
`feat(navigation_layer): add costmap inflation, A* planner and behavior state machine`
