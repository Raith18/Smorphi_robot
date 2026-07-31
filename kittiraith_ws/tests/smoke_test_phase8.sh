#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase8.sh — validates the Phase 8 deliverable without ROS:
#   * all Phase 8 Python sources compile
#   * the pure-metric + GT-parser unit tests pass
#   * the CPU profiler runs and produces a latency table
#   * all package.xml / launch XML files well-formed, YAML parses
#
# Usage:
#   ./tests/smoke_test_phase8.sh
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

echo "=== kittiraith_ws Phase 8 smoke test ==="

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

# --- 1. Compile all Phase 8 Python sources ------------------------------------
echo "--- compiling Phase 8 Python sources ---"
p8_files=(
  "src/adaptive_amr/evaluation/src/evaluation/metrics.py"
  "src/adaptive_amr/evaluation/scripts/benchmark.py"
  "src/adaptive_amr/evaluation/scripts/profile.py"
  "src/adaptive_amr/evaluation/scripts/perf_collector.py"
  "src/adaptive_amr/evaluation/test/test_metrics.py"
  "src/adaptive_amr/evaluation/test/test_kitti_gt.py"
)
for f in "${p8_files[@]}"; do
  if "${PYTHON_BIN}" -m py_compile "${PROJECT_ROOT}/${f}" 2>/dev/null; then
    ok "compiles: ${f}"
  else
    bad "syntax error: ${f}"
  fi
done

# --- 2. Unit tests ------------------------------------------------------------------
echo "--- unit tests ---"
run_tests() {
  local label="$1"; shift
  if ! "$@" > /tmp/p8_${label}.log 2>&1; then
    bad "unit tests failed: ${label} (see /tmp/p8_${label}.log)"
    tail -15 "/tmp/p8_${label}.log"
    return 1
  fi
  ok "unit tests ${label}: $(grep -E '^Ran ' "/tmp/p8_${label}.log")"
  return 0
}
run_tests "metrics" "${PYTHON_BIN}" "${AMR_DIR}/evaluation/test/test_metrics.py"
run_tests "kitti_gt" "${PYTHON_BIN}" "${AMR_DIR}/evaluation/test/test_kitti_gt.py"

# --- 3. CPU profiler runs -------------------------------------------------------------
echo "--- CPU profiler ---"
if timeout 300 "${PYTHON_BIN}" "${AMR_DIR}/evaluation/scripts/profile.py" \
    --repeats 2 > /tmp/p8_profile.log 2>&1; then
  ok "profile.py ran: $(grep -c '|' /tmp/p8_profile.log) table rows"
else
  bad "profile.py failed — see /tmp/p8_profile.log"
  tail -20 /tmp/p8_profile.log
fi

# --- 4. XML / YAML ----------------------------------------------------------------------
echo "--- XML / YAML ---"
xml_bad=0; xml_count=0
while IFS= read -r -d '' f; do
  xml_count=$((xml_count + 1))
  "${PYTHON_BIN}" -c "import sys,xml.dom.minidom; xml.dom.minidom.parse('${f}')" 2>/dev/null \
    || { bad "invalid XML: ${f}"; xml_bad=1; }
done < <(find "${AMR_DIR}" \( -name "package.xml" -o -name "*.launch" \) -print0 | sort -z)
[[ "${xml_bad}" -eq 0 ]] && ok "all ${xml_count} XML files well-formed"

yaml_bad=0; yaml_count=0
while IFS= read -r -d '' f; do
  yaml_count=$((yaml_count + 1))
  "${PYTHON_BIN}" -c "import sys,yaml; yaml.safe_load(open('${f}'))" 2>/dev/null \
    || { bad "invalid YAML: ${f}"; yaml_bad=1; }
done < <(find "${PROJECT_ROOT}/src" "${PROJECT_ROOT}/config" -name "*.yaml" -print0 | sort -z)
[[ "${yaml_bad}" -eq 0 ]] && ok "all ${yaml_count} YAML files parse"

# --- 5. Structure --------------------------------------------------------------------------
echo "--- structure ---"
expected=(
  "src/adaptive_amr/evaluation/src/evaluation/metrics.py"
  "src/adaptive_amr/evaluation/scripts/benchmark.py"
  "src/adaptive_amr/evaluation/scripts/profile.py"
  "src/adaptive_amr/evaluation/scripts/perf_collector.py"
  "src/adaptive_amr/evaluation/config/performance_presets.yaml"
  "docs/phase8_benchmarking.md"
)
missing=0
for f in "${expected[@]}"; do
  [[ -f "${PROJECT_ROOT}/${f}" ]] || { bad "missing: ${f}"; missing=1; }
done
[[ "${missing}" -eq 0 ]] && ok "all ${#expected[@]} expected Phase 8 files present"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
