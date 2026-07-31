# Phase 7 — Navigation Layer · Behavior Layer · Decision Layer

> **Module status: ✅ COMPLETE**

Full tutorial: theory with mathematics, industrial context, every implemented
file, testing procedure, debugging, performance and interview questions.

---

## 1. Objective

| Layer | Package | Output |
|---|---|---|
| Navigation | `navigation_layer` | `/navigation_costmap` (inflated) + `/planned_path` (A*) |
| Behavior | `navigation_layer` | `/behavior/state` (NAVIGATE/AVOID/STOP/RESUME/GOAL_REACHED) |
| Decision | `navigation_layer` | `/behavior/velocity_command` (v, ω) |

**Definition of done:** `roslaunch adaptive_amr phase7_navigation.launch`
runs the *entire* stack (Phases 2–7); after publishing a goal, RViz shows the
costmap, an A* path avoiding inflated obstacles, and the behavior node
commands a velocity that stops when a dynamic obstacle is risky.

## 2. Theory

### 2.1 Costmap inflation

```
d(cell)   = Euclidean distance to the nearest lethal cell   (EDT, O(n))
cost(cell) = 100·(1 − d/R_inflate)   for d < R_inflate
              0                       otherwise
              -1                      unknown (never observed)
```

This is the `costmap_2d` inflation model: it turns the binary occupancy grid
into a smooth cost field so the planner keeps the robot's **body** (radius
`R_inflate`) away from walls, not just its center. Dynamic obstacles are
marked lethal before inflation, so the costmap already "knows" about moving
objects' current positions + predicted trajectories.

### 2.2 A* path planning

```
f(n) = g(n) + h(n)
g(n) = path cost so far      (step length × cell-cost factor)
h(n) = Euclidean distance to the goal   (admissible -> optimal)
open list: min-heap          (O(E log V))
```

- 8-connectivity with step costs `1` (cardinal) / `√2` (diagonal).
- Lethal cells (cost ≥ 100) blocked; unknown cells get a small penalty
  (prefer observed free space, still explore).
- The path is smoothed (collinear points removed).

### 2.3 Behavior state machine (the safety layer)

```
INIT → NAVIGATE ⇄ AVOID ⇄ NAVIGATE
NAVIGATE → STOP → RESUME → NAVIGATE
NAVIGATE → GOAL_REACHED → (new goal) → NAVIGATE
```

Transitions:
- `max_risk ≥ risk_stop` (0.8) → **STOP** (hard safety)
- `max_risk ≥ risk_avoid` (0.4) or path blocked → **AVOID** (slow + turn)
- risk clears → **RESUME** → **NAVIGATE**
- `distance_to_goal ≤ tolerance` → **GOAL_REACHED**

The decision layer converts the state into `(v, ω)`:
- NAVIGATE: `v = cruise`, `ω = kp·heading_error` (P-controller to the next waypoint)
- AVOID: `v = 0.3·cruise`, `ω = 0.6` (turn away)
- STOP/GOAL_REACHED: `(0, 0)`

**Why industrial:** the planner proposes, the behavior layer *vetoes*. A
warehouse AMR must be able to STOP regardless of what the planner thinks —
this three-layer separation (plan → behave → decide) is the safety-critical
architecture required in real deployments.

## 3. Industrial importance

- `navigation_costmap` + `planned_path` are exactly what `move_base` /
  navigation stacks produce and consume — the format every AMR planner eats.
- The behavior state machine is the direct ancestor of industrial "safety
  supervisor" layers (STOP is certified; planning is not).
- Dynamic-obstacle-aware costmaps are what make warehouse robots coexist
  with humans — they plan *around* predicted motion, not just static walls.

## 4. Folder structure

```
src/adaptive_amr/navigation_layer/
├── src/navigation_layer/{costmap,planner,behavior}.py   # pure, unit-tested
├── scripts/navigation_layer_node.py                     # costmap + A*
├── scripts/behavior_node.py                             # state machine + cmd
├── launch/{navigation_layer,behavior}.launch
├── config/{navigation_layer,behavior}.yaml
└── test/{test_costmap,test_planner,test_behavior}.py    # 23 tests
adaptive_amr_msgs/srv/SetGoal.srv
launch/phase7_navigation.launch
rviz/phase7_navigation.rviz
```

## 5. Required packages

`rospy std_msgs geometry_msgs nav_msgs tf2_ros diagnostic_msgs
adaptive_amr_msgs numpy scipy` (no new heavy deps)

## 6. ROS topics & services

| Topic | Type | Pub |
|---|---|---|
| `/navigation_costmap` | `nav_msgs/OccupancyGrid` | navigation_layer |
| `/planned_path` | `nav_msgs/Path` | navigation_layer |
| `/behavior/state` | `std_msgs/String` | behavior |
| `/behavior/velocity_command` | `geometry_msgs/Twist` | behavior |
| `/navigation_layer/statistics`, `/behavior/statistics` | `DiagnosticArray` | each |
| `/goal` | `geometry_msgs/PoseStamped` | user → both |
| `/navigation_layer/set_goal` | `adaptive_amr_msgs/SetGoal` (service) | user |

## 7-8. TF & parameters
Costmap/path live in `map`; robot start from TF `map → base_link`; dynamic
obstacles transformed `laser → map` via TF. All parameters in
`config/*.yaml` (inflation radius, A* penalty, risk thresholds, speeds).

## 9-11. Python classes
`CostmapBuilder` (build + world/cell), `AStarPlanner` (plan + simplify),
`BehaviorStateMachine` (update + desired_velocity), plus the two nodes.

## 12. Launch

```bash
roslaunch adaptive_amr phase7_navigation.launch rate:=0.5     # full stack
# terminal 2:
rostopic pub /goal geometry_msgs/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 15, y: 0, z: 0}, \
   orientation: {w: 1}}}" --once
rostopic echo /behavior/state
rostopic echo /behavior/velocity_command
```

## 13. Testing procedure

```bash
# unit tests (no ROS)
python3 src/adaptive_amr/navigation_layer/test/test_costmap.py    # 6
python3 src/adaptive_amr/navigation_layer/test/test_planner.py    # 5
python3 src/adaptive_amr/navigation_layer/test/test_behavior.py   # 12

# live
roslaunch adaptive_amr phase7_navigation.launch rate:=0.5 &
rostopic hz /navigation_costmap /planned_path /behavior/state
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase7_navigation.rviz
```

## 14. Expected outputs

- `/navigation_costmap`: inflated costmap; walls 100 fading to 0 over 1 m.
- `/planned_path`: A* path from the robot to the goal, around inflated
  obstacles.
- `/behavior/state`: INIT → NAVIGATE; STOP when a risky obstacle appears;
  GOAL_REACHED when close to the goal.
- `/behavior/velocity_command`: (v, ω) gated by the state.

## 15. Performance metrics (CPU, indicative)

| Stage | Latency |
|---|---|
| Costmap inflation (300×300) | 5-20 ms |
| A* (300×300 grid) | 10-100 ms |
| Behavior decision loop | < 1 ms @ 10 Hz |

## 16. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| No costmap | `/occupancy_grid` not publishing | run occupancy_grid (Phase 6) |
| No path | goal not set / start TF missing | publish `/goal`; check `map→base_link` TF |
| Path through walls | inflation radius too small | raise `inflation_radius` |
| Stuck in STOP | risk never clears | check `/dynamic_obstacles` source |
| No velocity | behavior in INIT | set a goal |

## 17. Common errors

1. `goal` in the wrong frame → always `frame_id: map`.
2. Costmap all -1 → occupancy grid has no data yet (localization first).
3. Behavior STOP with `path_blocked=true` → planner has no path (goal
   unreachable or start not localized).

## 18. Improvements

- Local planner (DWA/teb) for dynamic avoidance; global A* is Phase 7.
- Costmap inflation from the semantic map (per-class costs: people cost >
  boxes).
- Time-to-collision (TTC) in the decision layer.
- Receding-horizon replanning (already interval-based).
- Behavior state via `std_msgs/String` → upgrade to a typed enum message.

## 19. Interview questions

1. Why inflate the costmap instead of planning on the raw grid?
2. Why is Euclidean distance an admissible A* heuristic? (optimality)
3. Complexity of A* on a 300×300 grid? (O(E log V))
4. Why does the planner prefer observed free space over unknown?
5. What is the difference between a costmap and an occupancy grid?
6. Why a state machine for safety rather than a single planner?
7. What does `risk_stop` vs `risk_avoid` mean in production?
8. How would you certify the STOP behavior in a real AMR?
9. How do dynamic obstacles enter the costmap? (mark + reinflate)
10. Why a P-controller on heading error, and its limitation?
11. How would you add a local planner (DWA) below A*?
12. What happens if the goal is inside an obstacle?
13. How do you make A* prefer the semantic map's "drivable" classes?
14. Why transform everything into the `map` frame?
15. How would you handle a deadlock (robot surrounded)?

## 20. Git commits for this phase

```bash
feat(adaptive_amr_msgs): add SetGoal service definition
feat(navigation_layer): add costmap inflation, A* planner and behavior state machine
docs(kittiraith_ws): add phase 7 documentation, tests, launch and RViz config
```

---

**Phase 7 complete. Next: Phase 8 — Performance Benchmarking, Profiling & Optimization.**
