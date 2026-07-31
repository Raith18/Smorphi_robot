#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase4.sh — validates the Phase 4 deliverable without ROS/torch:
#   * all Phase 4 Python sources compile
#   * all package.xml / launch XML files well-formed
#   * all YAML config files parse
#   * the pure-algorithm unit tests pass (detection utils / SORT / seg / depth)
#
# Usage:
#   ./tests/smoke_test_phase4.sh
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

echo "=== kittiraith_ws Phase 4 smoke test ==="

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

# --- 1. Compile all Phase 4 Python sources ------------------------------------
echo "--- compiling Phase 4 Python sources ---"
p4_files=(
  "src/adaptive_amr/object_detection/src/object_detection/detection_utils.py"
  "src/adaptive_amr/object_detection/scripts/object_detection_node.py"
  "src/adaptive_amr/object_detection/test/test_detection_utils.py"
  "src/adaptive_amr/object_tracking/src/object_tracking/sort.py"
  "src/adaptive_amr/object_tracking/scripts/object_tracking_node.py"
  "src/adaptive_amr/object_tracking/test/test_sort.py"
  "src/adaptive_amr/semantic_segmentation/src/semantic_segmentation/segmentation_utils.py"
  "src/adaptive_amr/semantic_segmentation/scripts/semantic_segmentation_node.py"
  "src/adaptive_amr/semantic_segmentation/test/test_segmentation_utils.py"
  "src/adaptive_amr/depth_estimation/src/depth_estimation/depth_completion.py"
  "src/adaptive_amr/depth_estimation/scripts/depth_estimation_node.py"
  "src/adaptive_amr/depth_estimation/test/test_depth_completion.py"
)
for f in "${p4_files[@]}"; do
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
  if ! "$@" > /tmp/p4_${label}.log 2>&1; then
    bad "unit tests failed: ${label} (see /tmp/p4_${label}.log)"
    tail -15 "/tmp/p4_${label}.log"
    return 1
  fi
  ok "unit tests ${label}: $(grep -E '^Ran ' "/tmp/p4_${label}.log")"
  return 0
}
run_tests "detection" "${PYTHON_BIN}" "${AMR_DIR}/object_detection/test/test_detection_utils.py"
run_tests "sort"      "${PYTHON_BIN}" "${AMR_DIR}/object_tracking/test/test_sort.py"
run_tests "seg"       "${PYTHON_BIN}" "${AMR_DIR}/semantic_segmentation/test/test_segmentation_utils.py"
run_tests "depth"     "${PYTHON_BIN}" "${AMR_DIR}/depth_estimation/test/test_depth_completion.py"

# --- 5. Message definitions present ----------------------------------------------------
echo "--- adaptive_amr_msgs ---"
msg_count=$(find "${AMR_DIR}/adaptive_amr_msgs/msg" -name "*.msg" 2>/dev/null | wc -l)
[[ "${msg_count}" -ge 4 ]] && ok "message definitions present (${msg_count} .msg files)" \
  || bad "expected >= 4 message files, found ${msg_count}"

# --- 6. Expected Phase 4 files -----------------------------------------------------------
echo "--- Phase 4 structure ---"
expected=(
  "src/adaptive_amr/object_detection/scripts/object_detection_node.py"
  "src/adaptive_amr/object_tracking/scripts/object_tracking_node.py"
  "src/adaptive_amr/semantic_segmentation/scripts/semantic_segmentation_node.py"
  "src/adaptive_amr/depth_estimation/scripts/depth_estimation_node.py"
  "src/adaptive_amr/launch/phase4_perception.launch"
  "src/adaptive_amr/rviz/phase4_perception.rviz"
  "docker/pip_requirements_phase4.txt"
  "docs/phase4_perception.md"
)
missing=0
for f in "${expected[@]}"; do
  [[ -f "${PROJECT_ROOT}/${f}" ]] || { bad "missing: ${f}"; missing=1; }
done
[[ "${missing}" -eq 0 ]] && ok "all ${#expected[@]} expected Phase 4 files present"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
