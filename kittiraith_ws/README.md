# kittiraith_ws

## Adaptive Multi-Sensor Perception, Localization and Semantic Mapping Framework for Autonomous Mobile Robots

> A production-quality, industrial-style autonomy stack that replays the **KITTI dataset**
> on **ROS Noetic / Ubuntu 20.04** and performs multi-sensor perception, sensor-fusion
> localization, semantic mapping, object detection & tracking, motion prediction and
> navigation-ready output — built module by module, the way a real warehouse-automation
> company would build it.

| | |
|---|---|
| **OS** | Ubuntu 20.04 (Focal Fossa) |
| **ROS** | Noetic Ninjemys (ROS 1) |
| **Languages** | Python 3.8 · C++17 |
| **Build** | Catkin (`catkin build` / `catkin_make`) |
| **Frameworks** | OpenCV · PCL · Eigen · PyTorch · NumPy · SciPy |
| **Visualization** | RViz |
| **Dev environment** | Docker (optional) + Git |
| **License** | MIT |

---

## 1. Why this project exists (the industrial story)

Autonomous Mobile Robots (AMRs) in warehouses (Amazon Robotics, GreyOrange, Geek+,
Locus Robotics, Addverb, …) must perceive the world through **multiple complementary
sensors**: cameras (texture, color, semantics), LiDAR (precise 3D geometry, range,
robust to lighting), IMU (high-rate motion), and GPS/GNSS (global context). No single
sensor is reliable enough on its own — the industry converges on the same pattern:

```
Sensors → Drivers → Time-Synchronized Fusion → Perception (Detection/Tracking/
Segmentation/Depth) → Odometry & SLAM → Localization → Semantic Mapping →
Motion Prediction → Navigation-ready costmaps
```

This repository implements that entire pattern **on the KITTI dataset**, the de-facto
standard benchmark for autonomous driving / mobile robotics research, so every module
can be validated against ground truth and compared with published results.

---

## 2. Roadmap (10 phases)

| Phase | Title | Status |
|---|---|---|
| **1** | Development Environment · ROS Workspace · Git · Docker · Dataset Management | ✅ **Done** |
| **2** | KITTI ROS Player · Time Synchronization · Calibration · Sensor Drivers | ✅ **Done** |
| **3** | Camera Pipeline · LiDAR Pipeline · Fusion Pipeline | ✅ **Done** |
| **4** | Detection · Tracking · Semantic Segmentation · Depth Estimation | ✅ **Done** |
| **5** | Visual Odometry · LiDAR Odometry · ICP Localization | ✅ **Done** |
| **6** | Semantic Mapping · Dynamic Occupancy Grid · Motion Prediction | ⏳ Next |
| **7** | Navigation Layer · Behavior Layer · Decision Layer | ⏳ Pending |
| **8** | Performance Benchmarking · Profiling · Optimization | ⏳ Pending |
| **9** | Docker · CI/CD · Unit Testing · Integration Testing | ⏳ Pending |
| **10** | Technical Documentation · Demo Videos · GitHub Portfolio · Research-style Report | ⏳ Pending |

Each phase is delivered as: **theory → industrial context → mathematics → implementation
→ testing → debugging → documentation → interview questions**, with a Git commit at the end.

### Phase 2 quick demo

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch adaptive_amr phase2_kitti_sensors.launch rate:=1   # real-time KITTI replay
# new terminal:
rostopic hz /camera/image_raw /velodyne_points /imu/data /gps/fix
rostopic echo -n1 /time_sync/statistics
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/kitti_sensors.rviz
# offline bag (no roscore needed):
rosrun dataset_loader kitti_to_bag.py --root /data/kitti --output /data/kitti/sample.bag
```

### Phase 3 quick demo

```bash
source /opt/ros/noetic/setup.bash && source devel/setup.bash
roslaunch adaptive_amr phase3_pipelines.launch rate:=1
# new terminal:
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase3_fusion.rviz
#   - colored LiDAR aligned with the camera (fusion!)
#   - obstacles (red) vs ground (green), 3D cluster boxes
#   - fusion overlay + sparse depth image
rostopic hz /camera/image_rect /lidar_processing/obstacles /fusion/sparse_depth
```

### Phase 4 quick demo

```bash
# first run downloads yolov8n.pt + yolov8n-seg.pt (internet required, ~12 MB)
roslaunch adaptive_amr phase4_perception.launch rate:=1
# new terminal:
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase4_perception.rviz
#   - YOLOv8 detections with fused depth on the annotated image
#   - SORT tracks with stable IDs (markers)
#   - semantic map colored + semantic-colored LiDAR cloud
#   - dense depth image (LiDAR depth completed everywhere)
rostopic hz /object_detections /object_tracks /semantic_map /depth/dense
# CPU-only stack: if 10 Hz is too much for your CPU, use rate:=0.5 or imgsz:=416
```

### Phase 5 quick demo

```bash
roslaunch adaptive_amr phase5_odometry.launch rate:=1
# new terminal:
rostopic hz /visual_odometry /lidar_odometry /localization_pose
rostopic echo -n1 /localization/statistics      # phase: mapping -> localizing
rosrun rviz rviz -d $(rospack find adaptive_amr)/rviz/phase5_odometry.rviz
#   - green path = LiDAR odometry (ICP scan-to-scan)
#   - yellow path = stereo visual odometry
#   - grey map = /localization/map growing during mapping
#   - red pose arrow = /localization_pose (ICP scan-to-map, no EKF)
#   - TF tree: map -> odom -> base_link fully dynamic
```

---

## 3. Repository layout

```
kittiraith_ws/
├── README.md                     # this file
├── docs/                         # phase documentation + architecture diagrams
├── scripts/                      # environment, build, dataset & tooling scripts
│   ├── 00_install_ros_noetic.sh
│   ├── 01_create_workspace.sh
│   ├── 02_install_python_deps.sh
│   ├── 03_build_workspace.sh
│   ├── 04_download_kitti_sample.sh
│   ├── setup_all.sh              # one-shot orchestrator
│   ├── scaffold_adaptive_amr.sh  # generates the ROS package skeleton
│   ├── docker/                   # container build/run helpers
│   └── kitti/verify_kitti.py     # dataset structure validator
├── docker/                       # Dockerfile + compose + entrypoint
├── config/                       # YAML configuration (no hardcoded paths in code)
├── tests/                        # smoke tests
├── data/                         # datasets & bags (git-ignored)
├── logs/                         # setup/build logs (git-ignored)
└── src/
    └── adaptive_amr/             # metapackage
        ├── camera_node/          # … 20 ROS packages, one per module
        ├── lidar_node/
        ├── gps_node/
        ├── imu_node/
        ├── dataset_loader/
        ├── time_sync/
        ├── camera_processing/
        ├── lidar_processing/
        ├── calibration/
        ├── sensor_fusion/
        ├── object_detection/
        ├── semantic_segmentation/
        ├── object_tracking/
        ├── visual_odometry/
        ├── lidar_odometry/
        ├── localization/
        ├── semantic_mapping/
        ├── occupancy_grid/
        ├── motion_prediction/
        ├── navigation_layer/
        ├── msg/  srv/  launch/  config/  rviz/
        ├── scripts/  utilities/  evaluation/  documentation/
        └── package.xml            # metapackage
```

---

## 4. Quickstart (Ubuntu 20.04)

```bash
# 0) (Optional but recommended) create the GitHub repo once:
#    https://github.com/new  →  name: kittiraith_ws

# 1) Clone
git clone <your-fork-url> && cd kittiraith_ws

# 2) Everything in one shot (ROS install → workspace → deps → build → KITTI sample)
./scripts/setup_all.sh

# 3) Or step by step
./scripts/00_install_ros_noetic.sh
./scripts/01_create_workspace.sh
./scripts/02_install_python_deps.sh
./scripts/03_build_workspace.sh
./scripts/04_download_kitti_sample.sh

# 4) Validate everything
./tests/smoke_test_phase1.sh
```

### Docker alternative (any OS)

```bash
./scripts/docker/build_image.sh          # builds kittiraith_ws:noetic
./scripts/docker/run_dev_container.sh    # X11-forwarded dev shell
```

---

## 5. Documentation

| Document | Contents |
|---|---|
| `docs/phase1_development_environment.md` | Full Phase 1 tutorial: theory, steps, testing, debugging, interview questions |
| `docs/architecture_overview.md` | System architecture, TF tree, ROS graph, data flow |
| Per-module `README.md` (in `src/adaptive_amr/<module>/`) | 25-point module spec (objective → docs) |

---

## 6. Development rules (this project)

1. One module = one ROS package = one responsibility (no monoliths).
2. All communication through ROS topics/services — modules stay decoupled & reusable.
3. Configuration in YAML, never hardcoded paths.
4. OOP, documented classes/functions, logging, exception handling.
5. Conventional Git commits (`feat/`, `fix/`, `docs/`, `chore/`, `refactor/`).
6. Every phase ends with tests + documentation + a commit.

---

## 7. License

MIT — see [LICENSE](LICENSE).
