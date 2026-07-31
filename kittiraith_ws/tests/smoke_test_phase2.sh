#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase2.sh — validates the Phase 2 deliverable without ROS:
#   * all Phase 2 Python sources compile
#   * all package.xml and launch files are well-formed XML
#   * all YAML config files parse
#   * the dataset_loader unit tests pass (pure Python)
#   * the expected Phase 2 files exist
#
# Usage:
#   ./tests/smoke_test_phase2.sh
# Exit code: 0 = all critical checks passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AMR_DIR="${PROJECT_ROOT}/src/adaptive_amr"
VENV="${PROJECT_ROOT}/.venv/bin/activate"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; WARN=0
ok()   { PASS=$((PASS + 1)); echo -e "${GREEN}[PASS]${NC} $1"; }
bad()  { FAIL=$((FAIL + 1)); echo -e "${RED}[FAIL]${NC} $1"; }
warn() { WARN=$((WARN + 1)); echo -e "${YELLOW}[WARN]${NC} $1"; }

echo "=== kittiraith_ws Phase 2 smoke test ==="

PYTHON_BIN="python3"
if [[ -f "${VENV}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV}"
  ok "using project venv: $(which python3)"
fi

# --- 1. All Phase 2 Python sources compile -----------------------------------
echo "--- compiling Phase 2 Python sources ---"
p2_py_files=(
  "src/adaptive_amr/dataset_loader/src/dataset_loader/kitti_parsers.py"
  "src/adaptive_amr/dataset_loader/src/dataset_loader/pacing.py"
  "src/adaptive_amr/dataset_loader/src/dataset_loader/player_utils.py"
  "src/adaptive_amr/dataset_loader/scripts/kitti_to_bag.py"
  "src/adaptive_amr/dataset_loader/test/test_kitti_parsers.py"
  "src/adaptive_amr/camera_node/scripts/camera_node.py"
  "src/adaptive_amr/lidar_node/scripts/lidar_node.py"
  "src/adaptive_amr/gps_node/scripts/gps_node.py"
  "src/adaptive_amr/imu_node/scripts/imu_node.py"
  "src/adaptive_amr/calibration/scripts/calibration_node.py"
  "src/adaptive_amr/time_sync/scripts/time_sync_node.py"
)
for f in "${p2_py_files[@]}"; do
  if "${PYTHON_BIN}" -m py_compile "${PROJECT_ROOT}/${f}" 2>/dev/null; then
    ok "compiles: ${f}"
  else
    bad "syntax error: ${f}"
  fi
done

# --- 2. XML validity of package.xml + launch files -----------------------------
echo "--- XML validity ---"
xml_files=()
while IFS= read -r -d '' f; do xml_files+=("$f"); done \
  < <(find "${AMR_DIR}" \( -name "package.xml" -o -name "*.launch" \) -print0 | sort -z)
xml_bad=0
for f in "${xml_files[@]}"; do
  if ! "${PYTHON_BIN}" -c "import sys,xml.dom.minidom; xml.dom.minidom.parse('${f}')" 2>/dev/null; then
    bad "invalid XML: ${f}"
    xml_bad=1
  fi
done
[[ "${xml_bad}" -eq 0 ]] && ok "all ${#xml_files[@]} XML files well-formed"

# --- 3. YAML validity (skip if pyyaml unavailable) ------------------------------
echo "--- YAML validity ---"
if "${PYTHON_BIN}" -c "import yaml" 2>/dev/null; then
  yaml_bad=0; yaml_count=0
  while IFS= read -r -d '' f; do
    yaml_count=$((yaml_count + 1))
    if ! "${PYTHON_BIN}" -c "import sys,yaml; yaml.safe_load(open('${f}'))" 2>/dev/null; then
      bad "invalid YAML: ${f}"; yaml_bad=1
    fi
  done < <(find "${PROJECT_ROOT}/src" "${PROJECT_ROOT}/config" -name "*.yaml" -print0 | sort -z)
  [[ "${yaml_bad}" -eq 0 ]] && ok "all ${yaml_count} YAML files parse"
else
  warn "pyyaml not installed — YAML validation skipped"
fi

# --- 4. dataset_loader unit tests ------------------------------------------------
echo "--- dataset_loader unit tests ---"
test_runner=("${PYTHON_BIN}" "${AMR_DIR}/dataset_loader/test/test_kitti_parsers.py")
if ! "${test_runner[@]}" > /tmp/p2_unittest.log 2>&1; then
  bad "unit tests failed — see /tmp/p2_unittest.log"
  tail -20 /tmp/p2_unittest.log
else
  ok "dataset_loader unit tests: $(grep -E '^Ran ' /tmp/p2_unittest.log)"
fi

# --- 5. Expected Phase 2 files ------------------------------------------------------
echo "--- Phase 2 structure ---"
expected_files=(
  "src/adaptive_amr/dataset_loader/setup.py"
  "src/adaptive_amr/dataset_loader/launch/kitti_player.launch"
  "src/adaptive_amr/dataset_loader/scripts/kitti_to_bag.py"
  "src/adaptive_amr/camera_node/scripts/camera_node.py"
  "src/adaptive_amr/lidar_node/scripts/lidar_node.py"
  "src/adaptive_amr/gps_node/scripts/gps_node.py"
  "src/adaptive_amr/imu_node/scripts/imu_node.py"
  "src/adaptive_amr/calibration/scripts/calibration_node.py"
  "src/adaptive_amr/time_sync/scripts/time_sync_node.py"
  "src/adaptive_amr/adaptive_amr_msgs/package.xml"
  "src/adaptive_amr/launch/phase2_kitti_sensors.launch"
  "src/adaptive_amr/rviz/kitti_sensors.rviz"
  "docs/phase2_kitti_ros_player.md"
)
missing=0
for f in "${expected_files[@]}"; do
  [[ -f "${PROJECT_ROOT}/${f}" ]] || { bad "missing: ${f}"; missing=1; }
done
[[ "${missing}" -eq 0 ]] && ok "all ${#expected_files[@]} expected Phase 2 files present"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
