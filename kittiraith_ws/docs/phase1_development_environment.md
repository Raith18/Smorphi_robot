# Phase 1 — Development Environment, ROS Workspace, Git, Docker & Dataset Management

> **Module status: ✅ COMPLETE**
>
> This document is the full tutorial for Phase 1: the *theory* (why each choice
> is made the way industrial robotics teams make it), the *implementation*
> (every file in the repository), the *testing procedure*, *debugging*, and
> *interview questions*.

---

## Table of contents

1. [Objective](#1-objective)
2. [Theory](#2-theory)
3. [Industrial importance](#3-industrial-importance)
4. [Folder structure](#4-folder-structure)
5. [Required packages](#5-required-packages)
6. [Implementation (step by step)](#6-implementation-step-by-step)
7. [Testing procedure](#7-testing-procedure)
8. [Expected outputs](#8-expected-outputs)
9. [Performance metrics](#9-performance-metrics)
10. [Debugging guide](#10-debugging-guide)
11. [Common errors](#11-common-errors)
12. [Future improvements](#12-future-improvements)
13. [Interview questions](#13-interview-questions)
14. [Git commits for this phase](#14-git-commits-for-this-phase)

---

## 1. Objective

Phase 1 builds the **foundation** on which all 20 modules will stand:

1. A reproducible **Ubuntu 20.04 + ROS Noetic** development environment.
2. A **catkin workspace** (`kittiraith_ws`) containing the `adaptive_amr`
   metapackage and the skeleton of every module package.
3. **Git** workflow: conventional commits, clean history, one commit per phase.
4. **Docker** dev image so the environment is identical on any machine.
5. **KITTI dataset management**: download scripts, layout conventions, and a
   structure validator — so every later phase can assume a known data layout.

**Definition of done for Phase 1:** from a clean Ubuntu 20.04 machine (or the
Docker image), running `./scripts/setup_all.sh` yields a *built* workspace and
a *validated* KITTI sample dataset, with the smoke test green.

---

## 2. Theory

### 2.1 Why Ubuntu 20.04 + ROS Noetic?

| Component | Choice | Why |
|---|---|---|
| OS | Ubuntu 20.04 LTS | 5-year support (to 2025+); the *only* officially supported OS for ROS Noetic; matches KITTI-era toolchains |
| ROS | Noetic (ROS 1) | Final ROS 1 distro, still dominant in industrial AMR deployments; richest ecosystem (PCL, move_base, RViz) |
| Python | 3.8 | Ships with 20.04; `cv_bridge`, `tf2`, `rospy` are built against it |
| Build | Catkin | The ROS 1 native build system; `catkin build` (catkin_tools) preferred over `catkin_make` |

> **Why not ROS 2?** ROS 2 (Foxy/Humble) is the future, but a huge fraction of
> deployed warehouse robots (and almost all of the classic SLAM/vision stack:
> LOAM, ORB-SLAM2, VINS-Fusion, autoware.ai) is ROS 1. Learning ROS 1 first is
> still the fastest path into industrial robotics, and the ROS concepts you
> learn transfer directly to ROS 2. This project deliberately chooses the
> stable, well-documented ROS 1 ecosystem.

### 2.2 ROS 1 mental model (30-second primer)

```
Node (process)  ──publishes──▶  Topic (typed bus, pub/sub, decoupled)
Node (process)  ──calls──────▶  Service (request/reply)
Node (process)  ──reads──────▶  Parameter server (static config)
roscore (master) ──matches──── publishers ↔ subscribers
rosbag ────────── records/replays everything on the bus
rviz ──────────── visualizes topics + TF
```

Key design consequence: **modules never call each other directly** — they only
exchange typed messages on topics. That is what makes a ROS stack *modular,
testable, and replayable* (record once, replay forever).

### 2.3 The catkin workspace

```
kittiraith_ws/
├── src/     ← source space (your packages)
├── build/   ← build space (generated)
├── devel/   ← development space (generated: setup.bash, libraries)
└── install/ ← install space (optional, generated)
```

- `catkin_init_workspace src` creates `src/CMakeLists.txt`.
- `catkin build` (catkin_tools) is **per-package parallel**, has better
  dependency handling and failure isolation than `catkin_make` — which is why
  we prefer it (both are supported by our scripts).
- After a build: `source devel/setup.bash` overlays the workspace on ROS.

### 2.4 Why a metapackage + 20 packages (not one big node)?

A metapackage (`adaptive_amr`) is a package that only *aggregates* others. Each
pipeline stage lives in its own package with its own `package.xml`,
`CMakeLists.txt`, tests, and launch files. This mirrors how industrial stacks
are organized (e.g., `navigation` metapackage wrapping `move_base`,
`amcl`, `map_server`):

- **Reusability**: `object_detection` can be dropped into another robot.
- **Replaceability**: swap `lidar_odometry` (e.g., LOAM → FAST-LIO) without
  touching anything else.
- **Build times**: only changed packages rebuild.
- **Testing**: each package can be tested in isolation.

### 2.5 Git workflow

- **Trunk-based with conventional commits** (used by most modern robotics
  teams): small, atomic commits — `feat()`, `fix()`, `docs()`, `chore()`,
  `refactor()`, `test()`.
- One commit per phase deliverable; messages describe *what & why*.
- `main` is always in a buildable state; phases are developed on feature
  branches and merged (Phase 9 adds CI to enforce this).
- **Never commit**: datasets, rosbags, build artifacts, logs → enforced by
  `.gitignore`.

### 2.6 Docker for robotics

Why industrial teams containerize:

1. **Reproducibility** — "works on my machine" disappears; the image pins OS,
   ROS, Python and library versions.
2. **CI/CD** — the same image builds in the pipeline and runs on the robot.
3. **Onboarding** — a new engineer is productive in 10 minutes, not 2 days.
4. **Isolation** — multiple ROS versions on one host without conflicts.

The trade-offs we design around:

- **ROS 1 networking** → we use `network_mode: host` (ROS1's master/multicast
  model dislikes bridge networking).
- **GUI (rviz/rqt)** → X11 socket + `.Xauthority` forwarding on Linux.
- **GPU (Phases 4+)** → `nvidia-container-toolkit` + `--gpus all`.
- **Data** → a named volume `/data/kitti` keeps 30+ GB datasets out of the
  image and the git repo.

### 2.7 KITTI dataset management

KITTI raw drive `2011_09_26_drive_0005_sync` contains, **time-synchronized**:

```
image_00/ image_01/   ← grayscale stereo (left/right), 10 Hz, 1241×376
image_02/ image_03/   ← color stereo, 10 Hz
velodyne/             ← Velodyne HDL-64E point clouds, 10 Hz, ~120k pts
oxts/                 ← GPS/IMU combined (lat, lon, alt, vel, yaw/pitch/roll, acc, gyro), 100 Hz
timestamps.txt        ← ISO-8601 UTC timestamp per frame
```

The `_calib` folder holds the intrinsic/extrinsic calibration
(`calib_cam_to_cam.txt`, `calib_velo_to_cam.txt`, `calib_imu_to_velo.txt`)
that Phase 2 will parse into ROS TF transforms.

The **odometry** dataset reorganizes the same sensors into 11 sequences
(`00`..`10`) with `calib.txt`, `times.txt`, and ground-truth `poses/<seq>.txt`
— the benchmark we will use for SLAM evaluation (Phase 5/8).

**Why manage the dataset as code?** A validator (`verify_kitti.py`) turns
"the dataset looks wrong" into a precise, scripted error message. In industry
this is the difference between a 2-day debugging session and a 2-minute
diagnosis.

---

## 3. Industrial importance

- **Amazon Robotics / Geek+ / GreyOrange** run thousands of identical robot
  instances; a reproducible environment is what makes fleet-wide software
  updates safe.
- **Autonomous vehicle / AMR teams** spend >30% of engineering time on data
  pipelines — dataset tooling is a first-class engineering artifact, not a
  chore.
- **Docker + Git + CI** are *required* skills in every robotics job posting at
  these companies; this phase gives you the exact workflow they use.
- KITTI remains the de-facto benchmark for perception/SLAM papers, so the
  skills here transfer to Waymo nuScenes, Argoverse, etc.

---

## 4. Folder structure

```
kittiraith_ws/
├── README.md
├── LICENSE · .gitignore · .dockerignore
├── docs/
│   ├── architecture_overview.md
│   └── phase1_development_environment.md      ← this file
├── scripts/
│   ├── 00_install_ros_noetic.sh   # OS-level ROS install (Ubuntu 20.04 only)
│   ├── 01_create_workspace.sh     # catkin layout + scaffold
│   ├── 02_install_python_deps.sh  # venv + pip requirements
│   ├── 03_build_workspace.sh      # catkin build / catkin_make
│   ├── 04_download_kitti_sample.sh# KITTI sample + validation
│   ├── setup_all.sh               # orchestrator (steps 00-04)
│   ├── scaffold_adaptive_amr.sh   # generates package skeletons
│   ├── docker/                    # build_image.sh, run_dev_container.sh
│   └── kitti/verify_kitti.py      # dataset structure validator
├── docker/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── pip_requirements_phase1.txt
├── config/kitti_dataset.yaml      # dataset manifest (single source of truth)
├── tests/smoke_test_phase1.sh
├── data/  logs/                   # git-ignored
└── src/adaptive_amr/              # metapackage + 20 module packages
```

---

## 5. Required packages

| Layer | Packages |
|---|---|
| OS | ubuntu 20.04, build-essential, curl, wget, unzip, git, git-lfs |
| ROS | ros-noetic-desktop-full, python3-rosdep, python3-catkin-tools, python3-rosinstall |
| Python | numpy, scipy, matplotlib, pandas, tqdm, pyyaml, rospkg, defusedxml (see `docker/pip_requirements_phase1.txt`) |
| Docker (optional) | Docker Engine ≥ 20.10, docker-compose, nvidia-container-toolkit (GPU, Phase 4+) |

---

## 6. Implementation (step by step)

### Step 00 — Install ROS Noetic

```bash
./scripts/00_install_ros_noetic.sh
```

What it does (see source): verifies Ubuntu 20.04 → adds the ROS apt repo
(packages.ros.org) → installs `ros-noetic-desktop-full` + `catkin_tools` →
`rosdep init && rosdep update` → appends the ROS source line to `~/.bashrc`.
Idempotent and fully logged to `logs/`.

### Step 01 — Create the workspace

```bash
./scripts/01_create_workspace.sh
```

Runs `catkin_init_workspace src` (when ROS is present) and invokes
`scaffold_adaptive_amr.sh`, which generates:

- the `adaptive_amr` **metapackage** (`package.xml` with `<metapackage/>` and
  20 `<run_depend>` entries; `CMakeLists.txt` with `catkin_metapackage()`);
- **20 module packages**, each with a minimal but valid `package.xml` +
  `CMakeLists.txt`, and `src/ launch/ config/ rviz/ test/` folders;
- shared directories: `msg/ srv/ launch/ config/ rviz/ scripts/ utilities/
  evaluation/ documentation/`.

The scaffold is **idempotent** — re-running it never overwrites your code.

### Step 02 — Python dependencies

```bash
./scripts/02_install_python_deps.sh
```

Creates a `.venv` (fallback: `pip --user`) and installs the exact same
requirements file the Docker image uses — host and container stay in sync.

### Step 03 — Build

```bash
./scripts/03_build_workspace.sh
```

Sources ROS, then `catkin build` (or `catkin_make`), then sources
`devel/setup.bash`. You should see 21 packages build cleanly (metapackage + 20
skeletons — skeletons compile instantly; they are where future phases add code).

### Step 04 — KITTI sample

```bash
./scripts/04_download_kitti_sample.sh
```

Downloads `2011_09_26_calib.zip` + `2011_09_26_drive_0005_sync.zip`
(~1.1 GB) into `data/kitti/raw/`, extracts, and validates with
`verify_kitti.py`.

### Everything at once

```bash
./scripts/setup_all.sh
# or, inside the Docker image (no OS-level install needed):
./scripts/setup_all.sh --skip-ros
```

### Docker route (any OS)

```bash
./scripts/docker/build_image.sh          # ~3 GB image, first build is slow
./scripts/docker/run_dev_container.sh    # X11-forwarded shell, workspace mounted
```

---

## 7. Testing procedure

```bash
# 1. Structure + environment checks (no ROS required)
./tests/smoke_test_phase1.sh

# 2. Validate a downloaded dataset (any machine)
python3 scripts/kitti/verify_kitti.py --root data/kitti

# 3. Inside the ROS environment
source /opt/ros/noetic/setup.bash
cd ~/kittiraith_ws
catkin build                      # or: ./scripts/03_build_workspace.sh
source devel/setup.bash
rospack find adaptive_amr         # → .../src/adaptive_amr
rospack find object_detection     # → .../src/adaptive_amr/object_detection
catkin list                       # → all 21 packages
```

---

## 8. Expected outputs

| Check | Expected |
|---|---|
| `smoke_test_phase1.sh` | All `[PASS]`, exit code 0 |
| `catkin build` | 21 packages, no errors |
| `rospack find <any module>` | Valid path under `src/adaptive_amr/` |
| `verify_kitti.py --root data/kitti` | `RESULT: N passed, 0 errors`, exit 0 |
| `docker build` | Image `kittiraith_ws:noetic` |
| `roscore` + `rosnode list` | `/rosout` only (no nodes yet — Phase 2 adds them) |

---

## 9. Performance metrics

| Metric | Target / Note |
|---|---|
| ROS install time | ~15–25 min (bandwidth dependent) |
| `catkin build` (skeleton) | < 1 min |
| Docker image size | ~3 GB (`ros:noetic-desktop-full-focal` base) |
| KITTI sample download | ~1.1 GB |
| `verify_kitti.py` runtime | < 1 s |
| Workspace reproducibility | 100% (same scripts + Docker ⇒ same environment) |

---

## 10. Debugging guide

| Symptom | Likely cause | Fix |
|---|---|---|
| `apt-key add` fails | Missing `curl`/`gnupg2` | Script installs them first; re-run step 00 |
| `rosdep update` errors | Network/proxy | Re-run; check `logs/`; `sudo rosdep init` if `/etc/ros` missing |
| `catkin build: Package ... not found` | ROS not sourced | `source /opt/ros/noetic/setup.bash` (or restart shell) |
| `roscore: command not found` | ROS not installed | Verify `/opt/ros/noetic`; re-run step 00 |
| `verify_kitti.py: no files in image_02` | Partial/unzipped download | Re-run step 04 (wget `-c` resumes) |
| Docker: no DISPLAY / rviz won't open | X11 not forwarded | Use `run_dev_container.sh`; ensure `xauth` installed on host |
| Docker on macOS/Windows: no ROS comms | `network_mode: host` unsupported | Use compose `ports: ["11311:11311"]` variant (see compose comments) |
| Disk full during build/download | Large packages/data | `docker system prune`, move `KITTI_ROOT` to another disk |

---

## 11. Common errors

1. **"ROS Noetic requires Ubuntu 20.04"** — you are on 21.10/22.04/24.04.
   → Use the Docker image (works on any OS).
2. **`E: Unable to locate package ros-noetic-desktop-full`** — ROS apt repo not
   added or `apt update` not run. → Re-run step 00 in full.
3. **`git push` rejected** — stale remote. → `git pull --rebase`, then push.
4. **Dataset zips re-download every run** — expected; the script skips files
   already present (resumable with `wget -c`).
5. **`.venv` python vs system python mismatch** — always `source .venv/bin/activate`
   before running Python tooling, or use the Docker image where the venv is
   unnecessary.

---

## 12. Future improvements

- **Devcontainer** (`.devcontainer/`) for VS Code: open repo → ready-to-code ROS.
- **`pre-commit` hooks** (ruff/black/clang-format, shellcheck).
- **DVC / dataset versioning** for the full KITTI set.
- **Automated environment test matrix** (host vs Docker) in CI (Phase 9).
- **GPU-enabled image variant** (`Dockerfile.gpu`) once Phase 4 needs PyTorch.

---

## 13. Interview questions

**ROS / general:**
1. What is the difference between a topic, a service, and an action?
2. Why are ROS 1 nodes decoupled via topics, and how does that help testing?
3. What does `roscore` do? What happens to topics if it dies?
4. What is a metapackage, and why would you use one?
5. `catkin_make` vs `catkin build` — differences and why catkin_tools is preferred.
6. How do `tf` and `tf2` differ? What is a static transform?

**Docker:**
7. Why does ROS 1 typically need `--network host` in Docker?
8. How do you pass a GUI (rviz) into a container?
9. How would you add GPU access to a container?

**Datasets:**
10. What does the KITTI `_sync` suffix mean, and why does it matter for sensor fusion?
11. Why must timestamps from different sensors be synchronized before fusion?
12. How would you validate a dataset programmatically (and why)?

**Git / engineering:**
13. What belongs in `.gitignore` for a robotics repo?
14. What is a conventional commit, and why do teams enforce message formats?
15. How do you keep "main always buildable" in a team of 5 engineers?

---

## 14. Git commits for this phase

```bash
chore(kittiraith_ws): add project scaffolding, license, gitignore and config
feat(kittiraith_ws): add ROS Noetic + workspace setup scripts
feat(kittiraith_ws): add Docker development environment
feat(kittiraith_ws): add KITTI dataset management tooling
docs(kittiraith_ws): add phase 1 documentation and smoke tests
```

---

**Phase 1 complete. Next: Phase 2 — KITTI ROS Player, Time Synchronization,
Calibration & Sensor Drivers.**
