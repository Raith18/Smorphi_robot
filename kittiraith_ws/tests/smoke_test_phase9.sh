#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase9.sh — validates the Phase 9 deliverable:
#   * CI workflows exist and are valid YAML
#   * the runners exist (unit / integration / smoke / verify_all)
#   * the integration test passes
#   * the stability script fixes are in place
#
# Usage:
#   ./tests/smoke_test_phase9.sh
# Exit code: 0 = all critical checks passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; WARN=0
ok()   { PASS=$((PASS + 1)); echo -e "${GREEN}[PASS]${NC} $1"; }
bad()  { FAIL=$((FAIL + 1)); echo -e "${RED}[FAIL]${NC} $1"; }
warn() { WARN=$((WARN + 1)); echo -e "${YELLOW}[WARN]${NC} $1"; }

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

echo "=== kittiraith_ws Phase 9 smoke test ==="

# --- 1. CI workflows exist + valid YAML ----------------------------------------
for f in ci docker; do
  path="${PROJECT_ROOT}/.github/workflows/${f}.yml"
  if [[ -f "${path}" ]] && "${PYTHON_BIN}" -c "import yaml,sys; yaml.safe_load(open('${path}'))"; then
    ok "workflow ${f}.yml present and valid"
  else
    bad "workflow ${f}.yml missing or invalid"
  fi
done

# --- 2. Runners exist + executable ------------------------------------------------
for f in tests/run_unit_tests.sh tests/run_integration_tests.sh \
         tests/run_all_smoke.sh scripts/verify_all.sh \
         scripts/ci/check_xml_yaml.py tests/integration/test_pipeline_integration.py; do
  if [[ -f "${PROJECT_ROOT}/${f}" ]]; then
    ok "runner present: ${f}"
  else
    bad "missing: ${f}"
  fi
done

# --- 3. Integration test passes ------------------------------------------------------
if bash "${PROJECT_ROOT}/tests/run_integration_tests.sh" > /tmp/p9_integration.log 2>&1; then
  ok "integration test passes"
else
  bad "integration test failed — see /tmp/p9_integration.log"
  tail -15 /tmp/p9_integration.log
fi

# --- 4. Unit runner reports -----------------------------------------------------------
if bash "${PROJECT_ROOT}/tests/run_unit_tests.sh" > /tmp/p9_unit.log 2>&1; then
  ok "unit runner: $(grep -E '^RESULT' /tmp/p9_unit.log)"
else
  bad "unit runner failed — see /tmp/p9_unit.log"
  tail -10 /tmp/p9_unit.log
fi

# --- 5. Stability fixes in place ---------------------------------------------------------
grep -q "DEBIAN_FRONTEND=noninteractive" "${PROJECT_ROOT}/scripts/00_install_ros_noetic.sh" \
  && ok "00_install: noninteractive apt" || bad "00_install: missing DEBIAN_FRONTEND"
grep -q -- "--fix-missing" "${PROJECT_ROOT}/scripts/00_install_ros_noetic.sh" \
  && ok "00_install: apt --fix-missing" || bad "00_install: missing --fix-missing"
grep -Fq "pip self-upgrade failed (continuing" "${PROJECT_ROOT}/scripts/02_install_python_deps.sh" \
  && ok "02_install: pip upgrade non-fatal" || bad "02_install: pip upgrade still fatal"
grep -Fq "ITS OWN spacing" "${PROJECT_ROOT}/src/adaptive_amr/occupancy_grid/src/occupancy_grid/grid_mapping.py" \
  && ok "ray-cast per-ray sampling fix present" \
  || bad "ray-cast fix missing (grid_mapping.py)"

# --- 6. Expected Phase 9 files -----------------------------------------------------------------
expected=(
  "docs/phase9_ci_testing.md"
  "tests/smoke_test_phase9.sh"
)
missing=0
for f in "${expected[@]}"; do
  [[ -f "${PROJECT_ROOT}/${f}" ]] || { bad "missing: ${f}"; missing=1; }
done
[[ "${missing}" -eq 0 ]] && ok "all ${#expected[@]} expected Phase 9 files present"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
