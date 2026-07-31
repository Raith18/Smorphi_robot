#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase5.sh — validates the Phase 5 deliverable without ROS:
#   * all Phase 5 Python sources compile
#   * all package.xml / launch XML files well-formed
#   * all YAML config files parse
#   * the pure-algorithm unit tests pass (VO geometry / ICP / localizer)
#
# Usage:
#   ./tests/smoke_test_phase5.sh
# Exit code: 0 = all critical checks passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AMR_DIR="${PROJECT_ROOT}/src/adaptive_amr"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; WARN=0
ok()   { PASS=$((PASS + 1)); echo -e "${GREEN}[PASS]${NC} $1"; }
bad()  { FAIL=$((FAIL + 1)); echo -e "${RED}[FAIL]${NC} $1"; }
warn() { WARN=$((WARN + 1)); echo -e "${YELLOW}[WARN]${NC} $1"; }

echo "=== kittiraith_ws Phase 5 smoke test ==="

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

# --- 1. Compile all Phase 5 Python sources ------------------------------------
echo "--- compiling Phase 5 Python sources ---"
p5_files=(
  "src/adaptive_amr/visual_odometry/src/visual_odometry/geometry_utils.py"
  "src/adaptive_amr/visual_odometry/src/visual_odometry/stereo_vo.py"
  "src/adaptive_amr/visual_odometry/scripts/visual_odometry_node.py"
  "src/adaptive_amr/visual_odometry/test/test_geometry_utils.py"
  "src/adaptive_amr/lidar_odometry/src/lidar_odometry/icp.py"
  "src/adaptive_amr/lidar_odometry/scripts/lidar_odometry_node.py"
  "src/adaptive_amr/lidar_odometry/test/test_icp.py"
  "src/adaptive_amr/localization/src/localization/icp_localizer.py"
  "src/adaptive_amr/localization/scripts/localization_node.py"
  "src/adaptive_amr/localization/test/test_localizer.py"
)
for f in "${p5_files[@]}"; do
  if "${PYTHON_BIN}" -m py_compile "${PROJECT_ROOT}/${f}" 2>/dev/null; then
    ok "compiles: ${f}"
  else
    bad "syntax error: ${f}"
  fi
done

# --- 2. XML validity -------------------------------------------------------------
echo "--- XML validity ---"
xml_files=()
while IFS= read -r -d '' f; do xml_files+=("$f"); done \
  < <(find "${AMR_DIR}" \( -name "package.xml" -o -name "*.launch" \) -print0 | sort -z)
xml_bad=0
for f in "${xml_files[@]}"; do
  "${PYTHON_BIN}" -c "import sys,xml.dom.minidom; xml.dom.minidom.parse('${f}')" 2>/dev/null \
    || { bad "invalid XML: ${f}"; xml_bad=1; }
done
[[ "${xml_bad}" -eq 0 ]] && ok "all ${#xml_files[@]} XML files well-formed"

# --- 3. YAML validity ---------------------------------------------------------------
echo "--- YAML validity ---"
if "${PYTHON_BIN}" -c "import yaml" 2>/dev/null; then
  yaml_bad=0; yaml_count=0
  while IFS= read -r -d '' f; do
    yaml_count=$((yaml_count + 1))
    "${PYTHON_BIN}" -c "import sys,yaml; yaml.safe_load(open('${f}'))" 2>/dev/null \
      || { bad "invalid YAML: ${f}"; yaml_bad=1; }
  done < <(find "${PROJECT_ROOT}/src" "${PROJECT_ROOT}/config" -name "*.yaml" -print0 | sort -z)
  [[ "${yaml_bad}" -eq 0 ]] && ok "all ${yaml_count} YAML files parse"
else
  warn "pyyaml not installed — YAML validation skipped"
fi

# --- 4. Unit tests -------------------------------------------------------------------
echo "--- unit tests ---"
run_tests() {
  local label="$1"; shift
  if ! "$@" > /tmp/p5_${label}.log 2>&1; then
    bad "unit tests failed: ${label} (see /tmp/p5_${label}.log)"
    tail -15 "/tmp/p5_${label}.log"
    return 1
  fi
  ok "unit tests ${label}: $(grep -E '^Ran ' "/tmp/p5_${label}.log")"
  return 0
}
run_tests "vo_geometry" "${PYTHON_BIN}" "${AMR_DIR}/visual_odometry/test/test_geometry_utils.py"
run_tests "icp"         "${PYTHON_BIN}" "${AMR_DIR}/lidar_odometry/test/test_icp.py"
run_tests "localizer"   "${PYTHON_BIN}" "${AMR_DIR}/localization/test/test_localizer.py"

# --- 5. Expected Phase 5 files ---------------------------------------------------------
echo "--- Phase 5 structure ---"
expected=(
  "src/adaptive_amr/visual_odometry/scripts/visual_odometry_node.py"
  "src/adaptive_amr/lidar_odometry/scripts/lidar_odometry_node.py"
  "src/adaptive_amr/localization/scripts/localization_node.py"
  "src/adaptive_amr/lidar_odometry/src/lidar_odometry/icp.py"
  "src/adaptive_amr/localization/src/localization/icp_localizer.py"
  "src/adaptive_amr/launch/phase5_odometry.launch"
  "src/adaptive_amr/rviz/phase5_odometry.rviz"
  "docs/phase5_odometry_localization.md"
)
missing=0
for f in "${expected[@]}"; do
  [[ -f "${PROJECT_ROOT}/${f}" ]] || { bad "missing: ${f}"; missing=1; }
done
[[ "${missing}" -eq 0 ]] && ok "all ${#expected[@]} expected Phase 5 files present"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
