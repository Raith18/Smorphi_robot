# scripts/ — shared tooling (adaptive_amr)

Workspace-level Python/Bash utilities shared across modules (importable from
any package via `rospack find adaptive_amr`). Conventions:

- Every script is executable, has a `#!/usr/bin/env python3` shebang and
  `if __name__ == "__main__":` entry point.
- Logging goes to the module's logger or `logs/` — never `print()` only.
- Reusable helpers live here instead of being copy-pasted into modules
  (no duplicated code).

| Planned utility | Phase | Purpose |
|---|---|---|
| `kitti_utils.py` | 2 | KITTI file/calib parsing helpers |
| `transform_utils.py` | 2 | TF/eigen helpers |
| `metrics.py` | 8 | ATE/RPE, IoU, MOTA... |
