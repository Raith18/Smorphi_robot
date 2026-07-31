#!/usr/bin/env bash
# =============================================================================
# scaffold_adaptive_amr.sh
# -----------------------------------------------------------------------------
# Generates the `adaptive_amr` metapackage and the skeleton of every module
# package (package.xml + CMakeLists.txt + README + empty subfolders), plus the
# shared directories (msg/, srv/, launch/, config/, rviz/, scripts/,
# utilities/, evaluation/, documentation/).
#
# Idempotent: never overwrites existing files, so it can be re-run safely.
#
# Usage:
#   ./scripts/scaffold_adaptive_amr.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${PROJECT_ROOT}/src"
METAPKG_DIR="${SRC_DIR}/adaptive_amr"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }

# --- Maintainer identity from git (fall back to defaults) ---------------------
GIT_NAME="$(git config user.name 2>/dev/null || true)"
GIT_EMAIL="$(git config user.email 2>/dev/null || true)"
MAINTAINER_NAME="${GIT_NAME:-Raith18}"
MAINTAINER_EMAIL="${GIT_EMAIL:-186810016+Raith18@users.noreply.github.com}"

# --- Module packages (one per pipeline module) ---------------------------------
# depth_estimation was added in Phase 4 (spec: Depth Estimation).
MODULES=(
  camera_node lidar_node gps_node imu_node dataset_loader time_sync
  camera_processing lidar_processing calibration sensor_fusion
  object_detection semantic_segmentation object_tracking depth_estimation
  visual_odometry lidar_odometry localization semantic_mapping
  occupancy_grid motion_prediction navigation_layer evaluation
)

# --- Shared directories inside the metapackage ---------------------------------
SHARED_DIRS=(msg srv launch config rviz scripts utilities evaluation documentation)

write_file_if_absent() {
  local path="$1"; shift
  if [[ -f "${path}" ]]; then
    warn "exists (kept): ${path}"
    return 0
  fi
  cat > "${path}"
  info "created: ${path}"
}

# =============================================================================
# 1. Metapackage: adaptive_amr
# =============================================================================
create_metapackage() {
  mkdir -p "${METAPKG_DIR}"

  # package.xml — metapackages cannot contain code or msgs; they only aggregate.
  # NOTE: generated atomically in one block so re-runs can never append duplicates.
  if [[ -f "${METAPKG_DIR}/package.xml" ]]; then
    warn "exists (kept): ${METAPKG_DIR}/package.xml"
  else
    {
      cat <<'EOF'
<?xml version="1.0"?>
<package format="2">
  <name>adaptive_amr</name>
  <version>0.1.0</version>
  <description>
    Adaptive Multi-Sensor Perception, Localization and Semantic Mapping Framework
    for Autonomous Mobile Robots (KITTI + ROS Noetic + Ubuntu 20.04).
    Metapackage aggregating all pipeline modules.
  </description>
EOF
      echo "  <maintainer email=\"${MAINTAINER_EMAIL}\">${MAINTAINER_NAME}</maintainer>"
      cat <<'EOF'
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>catkin</exec_depend>
  <metapackage/>
EOF
      for m in "${MODULES[@]}"; do
        echo "  <run_depend>${m}</run_depend>"
      done
      echo "</package>"
    } > "${METAPKG_DIR}/package.xml"
    info "created: ${METAPKG_DIR}/package.xml"
  fi

  write_file_if_absent "${METAPKG_DIR}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(adaptive_amr)

# A metapackage only aggregates other packages: no build code, no messages.
find_package(catkin REQUIRED)
catkin_metapackage()
EOF

  write_file_if_absent "${METAPKG_DIR}/README.md" <<'EOF'
# adaptive_amr (metapackage)

Aggregates all 20 pipeline modules of the **Adaptive Multi-Sensor Perception,
Localization and Semantic Mapping Framework**.

```
camera_node  lidar_node  gps_node  imu_node        # Phase 2: sensor drivers
dataset_loader  time_sync  calibration             # Phase 2: data + sync
camera_processing  lidar_processing  sensor_fusion # Phase 3: pipelines
object_detection  semantic_segmentation            # Phase 4: perception
object_tracking                                   # Phase 4
visual_odometry  lidar_odometry  localization      # Phase 5: SLAM + fusion
semantic_mapping  occupancy_grid  motion_prediction # Phase 6: mapping + prediction
navigation_layer                                  # Phase 7: navigation
```

Custom messages/services used by several modules will live in a dedicated
`adaptive_amr_msgs` package (added in Phase 2) — a metapackage cannot define msgs.
EOF

  info "Metapackage 'adaptive_amr' ready."
}

# =============================================================================
# 2. Module packages
# =============================================================================
create_module_package() {
  local m="$1"
  local pkg_dir="${METAPKG_DIR}/${m}"
  mkdir -p "${pkg_dir}"/{src,launch,config,rviz,test}

  write_file_if_absent "${pkg_dir}/package.xml" <<EOF
<?xml version="1.0"?>
<package format="2">
  <name>${m}</name>
  <version>0.1.0</version>
  <description>${m}: module of the adaptive_amr perception &amp; autonomy stack (KITTI / ROS Noetic). Skeleton — implementation added in its phase.</description>
  <maintainer email="${MAINTAINER_EMAIL}">${MAINTAINER_NAME}</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <depend>roscpp</depend>
  <depend>rospy</depend>
  <depend>std_msgs</depend>
</package>
EOF

  write_file_if_absent "${pkg_dir}/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.0.2)
project(PKG_NAME_PLACEHOLDER)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
EOF
  # Replace the placeholder with the real project name.
  if [[ -f "${pkg_dir}/CMakeLists.txt" ]]; then
    sed -i "s/PKG_NAME_PLACEHOLDER/${m}/" "${pkg_dir}/CMakeLists.txt"
  fi

  write_file_if_absent "${pkg_dir}/README.md" <<EOF
# ${m}

| Phase | Status |
|---|---|
| Implementation | Pending (see phase roadmap in the root README) |

## Objective
<!-- Filled in during the implementation phase. -->

## Theory
<!-- Mathematical formulation, algorithms, alternatives. -->

## Industrial importance
<!-- Why warehouse/AMR companies need this module. -->

## Folder structure
\`\`\`
${m}/
├── src/          # C++ / Python sources
├── launch/       # launch files
├── config/       # YAML parameters
├── rviz/         # RViz display configs
└── test/         # unit/integration tests
\`\`\`

## ROS topics (planned)
<!-- /topic_name  (MsgType)  publisher|subscriber  — description -->

## TF frames (planned)
<!-- parent -> child  — description -->

## Parameters (planned)
<!-- name (type, default) — description -->

## Launch (planned)
\`\`\`bash
roslaunch ${m} ${m}.launch
\`\`\`

## Testing procedure
<!-- How to verify the module on KITTI data. -->

## Expected outputs
<!-- Topics/plots/metrics to expect. -->

## Performance metrics
<!-- Latency, throughput, accuracy, resource usage. -->

## Debugging guide
<!-- rqt_graph, rostopic, rosbag, gdb... -->

## Common errors
<!-- Symptom -> cause -> fix. -->

## Improvements
<!-- Future work. -->

## Git commit message (suggested)
\`\`\`
feat(${m}): <what was implemented>
\`\`\`
EOF

  # Keep empty dirs visible in git.
  for d in src launch config rviz test; do
    touch "${pkg_dir}/${d}/.gitkeep"
  done
  info "scaffolded: ${m}"
}

# =============================================================================
# 3. Shared directories
# =============================================================================
create_shared_dirs() {
  for d in "${SHARED_DIRS[@]}"; do
    mkdir -p "${METAPKG_DIR}/${d}"
    touch "${METAPKG_DIR}/${d}/.gitkeep"
  done
  info "shared directories ready: ${SHARED_DIRS[*]}"
}

# =============================================================================
# Main
# =============================================================================
main() {
  info "=== adaptive_amr scaffold ==="
  mkdir -p "${SRC_DIR}"
  create_metapackage
  for m in "${MODULES[@]}"; do
    create_module_package "${m}"
  done
  create_shared_dirs
  info "Scaffold complete. Package count: $(( ${#MODULES[@]} + 1 ))"
}

main "$@"
