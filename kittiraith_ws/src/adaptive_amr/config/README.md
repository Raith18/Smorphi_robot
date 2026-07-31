# config/ — shared YAML configuration (adaptive_amr)

All node parameters are loaded from YAML at launch time via rosparam.
Conventions:

- `config/<module>.yaml` — parameters for that module's nodes.
- `config/kitti_dataset.yaml` (workspace root) — dataset manifest: paths,
  frame rates, calibration file names. Loaded by `dataset_loader`.
- No hardcoded paths in code. Paths come from YAML or ROS parameters with
  sane defaults + environment overrides (`KITTI_ROOT`).
