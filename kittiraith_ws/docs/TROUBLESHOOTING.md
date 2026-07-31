# Troubleshooting & FAQ — kittiraith_ws

Consolidated debugging guide across all phases. Per-module tables live in each
package README; this is the fast index.

## Setup / environment

| Symptom | Cause | Fix |
|---|---|---|
| `roscore: command not found` | ROS not installed/sourced | `source /opt/ros/noetic/setup.bash`; re-run `scripts/00_install_ros_noetic.sh` |
| `catkin build: package not found` | ROS not sourced | source `/opt/ros/noetic/setup.bash` + `devel/setup.bash` |
| `apt-key add` fails / apt broken | missing tools / flaky mirror | re-run step 00 (now non-interactive + `--fix-missing`) |
| Docker: no DISPLAY (rviz) | X11 not forwarded | use `scripts/docker/run_dev_container.sh` (Linux) |
| Docker on macOS/Windows: no comms | `network_mode: host` unsupported | use the compose `ports: ["11311:11311"]` variant |
| `pip install` fails in venv | PEP 668 / broken pip | `source .venv/bin/activate` first; pip upgrade is now non-fatal |

## Dataset

| Symptom | Cause | Fix |
|---|---|---|
| `verify_kitti.py` reports missing files | partial download | re-run `scripts/04_download_kitti_sample.sh` (resumable) |
| `Missing key 'P_rect_01'` | wrong/old calib for the date | re-download the date's `_calib.zip` |
| `OXTS line must contain 30 values` | corrupt oxts file | re-download the drive; run unit tests |
| Model download fails (YOLO) | no internet on first run | pre-download `yolov8n.pt`/`yolov8n-seg.pt`; pass `model:=/path` |

## Runtime (ROS)

| Symptom | Cause | Fix |
|---|---|---|
| Exact sync outputs nothing | stamps differ (float rounding) | use `read_timestamps_ns` + `ns_to_ros_time`; or `sync_type:=approx` |
| Two publishers on `/camera/image_rect` | camera_node still publishing rect | launch with `publish_rect:=false` (phase3+ does this) |
| Two publishers on `/tf` | two odometry nodes with `enable_tf` | keep only one (`vo_enable_tf`/`lo_enable_tf`) |
| TF "would require extrapolation" | TF not publishing / stale | check `enable_tf`; run calibration first |
| `No module named cv2` | opencv missing | `python3-opencv` (or `encoding:=compressed` / `--compressed`) |
| `ultralytics is not installed` | Phase 4 deps missing | `pip install --extra-index-url https://download.pytorch.org/whl/cpu -r docker/pip_requirements_phase4.txt` |
| Occupancy grid all `-1` | pose/obstacles topics not arriving | check `/localization_pose` + `/lidar_processing/obstacles` |
| Grid smears under motion | frame mismatch (laser vs map) | fixed in Phase 9 — use the TF transform; verify topics |
| Thin obstacles vanish from grid | free-ray washout | occlusion-aware sensor model; raise `l_occ`; lower `max_range` |
| Localization stuck in "mapping" | map never reaches min size | lower `min_map_points`/`max_mapping_frames` |
| A* finds no path | goal unreachable / start not localized | check goal frame (`map`), `map→base_link` TF |
| Behavior stuck in STOP | risk never clears | inspect `/dynamic_obstacles` source / `risk_stop` |

## Testing / CI

| Symptom | Cause | Fix |
|---|---|---|
| `verify_all.sh` fails on XML | malformed launch file | fix the XML; run `python scripts/ci/check_xml_yaml.py` |
| CI fails on py_compile | syntax error | fix; re-run `bash scripts/verify_all.sh --fast` locally |
| Integration test fails on grid | ray-cast/frame bug | run the integration test with the diagnostic loop (see test file) |
| `python test_x.py` not found | wrong cwd | run from the workspace root or via `tests/run_unit_tests.sh` |
| ATE ≈ 0 despite drift | Umeyama scale absorbs linear drift | report `with_scale=False` too |
| `docker compose config` fails | old compose | docker compose v2 |

## FAQ

**Q: Why is everything pure Python without ROS in `src/<pkg>/`?**
A: So every algorithm is unit-testable on any machine, CI-friendly without
roscore, and readable — the ROS nodes are thin wrappers around these cores.

**Q: Do I need a GPU?**
A: No. YOLOv8n (CPU), pure-NumPy ICP/EDT/A*, and SORT run at 10 Hz on modest
hardware. Set `device:=cpu` (default) and `imgsz:=416` if needed.

**Q: Can I use this on real hardware?**
A: The interfaces are standard ROS topics/services. Swap the KITTI drivers for
real camera/LiDAR/IMU drivers and the stack consumes them unchanged (use
`sync_type:=approx` for real clocks).

**Q: Why ICP localization instead of EKF?**
A: Your requirement: measurement-driven, deterministic, no tuned noise models.
FAST-LIO (EKF-based) is a documented drop-in upgrade with the same interface.

**Q: Where are the numbers?**
A: `docs/phase8_benchmarking.md` (measured profile + optimization) and
`docs/RESEARCH_REPORT.md` (full results section). Re-run
`python src/adaptive_amr/evaluation/scripts/profile.py` on your machine.

**Q: How do I run everything?**
A: `bash scripts/verify_all.sh` validates the code; on Ubuntu 20.04 with ROS:
`roslaunch adaptive_amr phase7_navigation.launch` runs the full stack.
