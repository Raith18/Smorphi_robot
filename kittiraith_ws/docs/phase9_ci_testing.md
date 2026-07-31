# Phase 9 — Docker · CI/CD · Unit Testing · Integration Testing

> **Module status: ✅ COMPLETE**

Full tutorial: the CI/CD design, the testing pyramid, the stability hardening,
and — the honest payoff — the **two real bugs the integration test found and
fixed**.

---

## 1. Objective

| Deliverable | Tool |
|---|---|
| CI/CD | `.github/workflows/ci.yml` (lint + tests on every push/PR) + `docker.yml` (weekly image build) |
| Unit tests | `tests/run_unit_tests.sh` — discovers all 19 test files (~180 tests), aggregated |
| Integration test | `tests/run_integration_tests.sh` — headless end-to-end pipeline on synthetic data |
| Stability scripts | `scripts/verify_all.sh` (one command: everything), `scripts/ci/check_xml_yaml.py` |
| Docker | existing `docker/` image + CI build job + compose validation |

## 2. Theory — the testing pyramid

```
        / integration \        (few: end-to-end module chains)
       /    unit tests   \     (many: one algorithm, one behavior)
      /   compile/lint     \
     /   XML/YAML validity   \
    /   shell syntax           \
```

- **Unit tests** prove each algorithm in isolation (180+ tests, pure Python —
  no ROS/GPU needed, so CI is fast).
- **Integration tests** prove the modules *work together the way the ROS graph
  does*: the same classes the nodes use, fed in sequence, end-to-end.
- **CI** runs the whole pyramid on every push — a regression anywhere fails
  the pipeline immediately.

## 3. What the integration test does

Builds a synthetic KITTI-like world (ground + 4 walls + a pillar obstacle),
drives a virtual robot along a path, and at every pose runs the REAL modules:

```
robot_scan (LiDAR-like, occlusion-aware)
  -> lidar_processing   (voxel, RANSAC ground removal, clustering)
  -> occupancy_grid      (log-odds ray-cast grid)
  -> navigation_layer    (costmap inflation + A*)
  -> behavior            (state machine + velocity)
  -> semantic_mapping    (voxel map with colors)
  -> motion_prediction   (trajectory + collision risk)
```

Asserts: the pillar region is occupied in the grid, walls map, the robot's
path stays free, A* finds a path that avoids obstacles, the behavior machine
reaches GOAL_REACHED, the semantic map grows, and predictions extrapolate.

## 4. The two REAL bugs found (and fixed) by integration testing

### Bug 1 — Phase 8 vectorized ray-cast dropped short-ray endpoints
The Phase 8 "19× faster" rewrite sampled every ray with a **global**
`linspace(0, 1, max_n)`. For any ray shorter than the longest one, its
endpoint (t = 1) was never sampled — the last sample sat ~⅔ along the ray.
Consequence: **every obstacle closer than the farthest one silently lost its
occupied hit** (the pillar in our world never appeared in the grid). The
single-point unit test passed because there `n_samples == max_n`.
**Fix:** sample each ray on its own spacing (`t = j/(n_i − 1)`) so every
ray's last sample is exactly its endpoint. Unit tests + integration green
again, speedup preserved (the per-ray division is still vectorized).

### Bug 2 — occupancy_grid node frame mismatch
The node ray-cast obstacle points (in the `laser` frame) against the robot
pose (in the `map` frame) — the grid smears the moment the robot moves.
**Fix:** transform the obstacle cloud `laser → map` via TF (with a warned
identity fallback) before ray-casting. The integration test mirrors the fixed
behavior and passes.

## 5. The "thin obstacle" insight

During debugging we confirmed a classic occupancy-grid behavior: **free-ray
updates from distant obstacles can wash out thin obstacles** if the sensor
model leaks rays through them. Two mitigations that matter in production:
model **occlusion** (a range sensor only reports the nearest hit per bearing —
the synthetic scanner now does exactly this) and tune `l_occ`/`l_free`/`max_range`.

## 6. Stability hardening (script changes made this phase)

- `00_install_ros_noetic.sh`: `DEBIAN_FRONTEND=noninteractive` + `apt-get
  update --fix-missing` (never blocks on debconf, survives flaky mirrors).
- `02_install_python_deps.sh`: `pip` self-upgrade failure no longer aborts
  the install (`|| warn`).
- New `scripts/verify_all.sh`: one command runs shell-check, compile-check,
  XML/YAML validation, all unit tests, integration test, all smoke tests,
  and (optionally) the CPU profiler.
- New `tests/run_unit_tests.sh` / `run_integration_tests.sh` /
  `run_all_smoke.sh`: deterministic, aggregated, exit-code-driven.
- New `scripts/ci/check_xml_yaml.py`: CI-grade validity check for every XML
  and YAML file (catches launch-file typos like the `--once` comment bug
  from Phase 7 before they reach roslaunch).

## 7. CI/CD design

```yaml
# .github/workflows/ci.yml  (every push/PR, ~3 min)
  verify job:
    ubuntu-22.04 + Python 3.8 (ROS Noetic target)
    pip install numpy scipy opencv-python-headless pyyaml   # CPU only, no torch
    bash -n every script · py_compile every module
    python scripts/ci/check_xml_yaml.py
    bash tests/run_unit_tests.sh
    bash tests/run_integration_tests.sh
    bash tests/run_all_smoke.sh
    python .../profile.py --repeats 2        # latency sanity
  docker job: (manual or "[docker]" commit)
    docker build -f docker/Dockerfile .
    docker compose -f docker/docker-compose.yml config

# .github/workflows/docker.yml  (weekly Monday 03:00 UTC + manual)
  builds the ROS Noetic image to catch base-image drift
```

**Why Python 3.8 on ubuntu-22.04:** our target is ROS Noetic (Python 3.8);
`actions/setup-python` provides 3.8 on any runner, and the pure-algorithm
cores are 3.8-compatible (verified — the whole suite runs here on 3.11 too).

## 8. Folder structure

```
.github/workflows/{ci,docker}.yml
scripts/verify_all.sh
scripts/ci/check_xml_yaml.py
tests/
├── run_unit_tests.sh
├── run_integration_tests.sh
├── run_all_smoke.sh
└── integration/test_pipeline_integration.py
```

## 9. Required packages (CI)
`numpy scipy opencv-python-headless pyyaml` — no torch, no ROS (all cores are
pure Python).

## 10-11. Usage / testing procedure

```bash
# everything, one command (this is what CI runs):
bash scripts/verify_all.sh

# individual pieces:
bash tests/run_unit_tests.sh
bash tests/run_integration_tests.sh
bash tests/run_all_smoke.sh
python scripts/ci/check_xml_yaml.py
```

## 12. Expected outputs
`verify_all.sh` → `RESULT: ALL CHECKS PASSED`; CI green check on the GitHub
branch; `docker build` succeeds on demand/weekly.

## 13. Performance metrics
- CI: ~3 min (unit + integration + smoke, no ROS/torch).
- Integration test: ~2 s locally.
- Full `verify_all.sh`: ~35 s (fast mode) on this VM.

## 14. Debugging guide

| Symptom | Cause | Fix |
|---|---|---|
| CI fails on `py_compile` | syntax error on a runner-only path | fix the file; check locally with `verify_all.sh` |
| Integration fails on grid | ray-cast or frame bug | run the integration test with the diagnostic loop |
| `docker compose config` fails | compose schema | docker compose >= 2; check the file |
| pytest not found | we use `unittest` | `run_unit_tests.sh` calls `python test_x.py` directly |

## 15. Common errors
1. Forgetting `bash -n` on new scripts → caught by CI job 1.
2. A unit test passing but integration failing → frame/ordering issue between
   modules (exactly what integration testing is for).
3. Python 3.9+ syntax in a module → breaks the 3.8 CI job; keep it 3.8-safe.

## 16. Improvements
- `pytest` with coverage thresholds (Phase 10 polish).
- ROS-level integration (roslaunch a headless bag through the real nodes).
- Pre-commit hooks (ruff/black) mirroring CI checks locally.
- Matrix CI: Python 3.8 + 3.10; optional GPU job for the perception tests.

## 17. Interview questions
1. Why test the pure algorithms without ROS? (speed, determinism, CI-ability)
2. What does the integration test catch that unit tests can't? (cross-module
   frame/order bugs — we found 2!)
3. Why is CI on Python 3.8 despite ubuntu-22.04? (target match via setup-python)
4. How does occlusion modeling change an occupancy grid?
5. Why does the vectorized ray-cast need per-ray spacing?
6. What is a regression and how does CI prevent it?
7. How would you add a ROS-level integration test? (roslaunch + rosbag + rostopic)
8. Why keep torch OUT of the CI unit tests? (detection_utils is pure math)
9. What does `verify_all.sh` give a new developer? (10-minute onboarding)
10. How do you measure test coverage and decide it's enough?

## 18. Git commits for this phase

```bash
ci: add GitHub Actions workflows (lint + unit + integration + smoke; docker)
test: add unit/integration/smoke runners and the headless pipeline integration test
fix(occupancy_grid): per-ray sampling in vectorized ray-cast (short-ray endpoints)
fix(occupancy_grid_node): transform obstacles laser->map before ray-casting
chore(scripts): harden install scripts and add verify_all.sh
docs(kittiraith_ws): add phase 9 documentation, smoke tests and CI guide
```

---

**Phase 9 complete. Next: Phase 10 — Technical Documentation, Demo Videos,
GitHub Portfolio & Research-style Report.**
