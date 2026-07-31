# launch/ — shared launch files (adaptive_amr)

Conventions for this workspace:

- Each module owns a `launch/<module>.launch` inside its own package.
- Top-level compositions live here in the metapackage:
  - `perception.launch`  — sensors + sync + processing (Phase 3)
  - `perception_full.launch` — everything up to navigation (Phase 7)
- Launch files use `$(find adaptive_amr)/config/*.yaml` via
  `<rosparam command="load">` — never hardcode paths.
- Every launch file must be runnable with:
  ```bash
  roslaunch adaptive_amr <file>.launch
  ```
