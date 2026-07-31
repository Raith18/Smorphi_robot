#!/usr/bin/env bash
# =============================================================================
# verify_all.sh — ONE command to validate the whole workspace (no ROS needed):
#   1. shell syntax of every script
#   2. python compile of every module
#   3. XML + YAML validity
#   4. all unit tests
#   5. integration test
#   6. all phase smoke tests
#
# Usage:
#   bash scripts/verify_all.sh [--fast]     (--fast skips the CPU profiler)
# Exit code: 0 = everything passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FAST=0
[[ "${1:-}" == "--fast" ]] && FAST=1

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[PASS]${NC} $1"; }
bad()  { echo -e "${RED}[FAIL]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }

FAILURES=0

echo "=== kittiraith_ws — full workspace verification ==="

# --- 1. shell syntax -----------------------------------------------------------
echo "--- shell syntax ---"
for f in $(find "${PROJECT_ROOT}" -name "*.sh" \
           -not -path "*/.venv/*" -not -path "*/.git/*"); do
  bash -n "${f}" 2>/dev/null || { bad "shell syntax: ${f}"; FAILURES=$((FAILURES+1)); }
done
ok "all shell scripts pass 'bash -n'"

# --- 2. python compile -----------------------------------------------------------
echo "--- python compile ---"
PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi
py_bad=0
while IFS= read -r -d '' f; do
  "${PYTHON_BIN}" -m py_compile "${f}" 2>/dev/null || { bad "compile: ${f}"; py_bad=$((py_bad+1)); }
done < <(find "${PROJECT_ROOT}/src" "${PROJECT_ROOT}/scripts" "${PROJECT_ROOT}/tests" \
         -name "*.py" -not -path "*/.venv/*" -print0)
if [[ "${py_bad}" -eq 0 ]]; then ok "all python sources compile"; else FAILURES=$((FAILURES+py_bad)); fi

# --- 3. XML + YAML -----------------------------------------------------------------
echo "--- XML / YAML ---"
"${PYTHON_BIN}" "${PROJECT_ROOT}/scripts/ci/check_xml_yaml.py" >/dev/null 2>&1 \
  && ok "all XML + YAML files valid" \
  || { bad "XML/YAML validation failed"; FAILURES=$((FAILURES+1)); }

# --- 4. unit tests -------------------------------------------------------------------
echo "--- unit tests ---"
bash "${PROJECT_ROOT}/tests/run_unit_tests.sh" >/tmp/verify_unit.log 2>&1 \
  && ok "all unit tests passed ($(grep -E '^RESULT' /tmp/verify_unit.log))" \
  || { bad "unit tests failed"; tail -20 /tmp/verify_unit.log; FAILURES=$((FAILURES+1)); }

# --- 5. integration -----------------------------------------------------------------------
echo "--- integration ---"
bash "${PROJECT_ROOT}/tests/run_integration_tests.sh" >/tmp/verify_integration.log 2>&1 \
  && ok "integration test passed" \
  || { bad "integration test failed"; tail -20 /tmp/verify_integration.log; FAILURES=$((FAILURES+1)); }

# --- 6. smoke tests ---------------------------------------------------------------------------
echo "--- smoke tests ---"
bash "${PROJECT_ROOT}/tests/run_all_smoke.sh" >/tmp/verify_smoke.log 2>&1 \
  && ok "all phase smoke tests passed" \
  || { bad "smoke tests failed"; tail -20 /tmp/verify_smoke.log; FAILURES=$((FAILURES+1)); }

# --- 7. CPU profiler (optional) -----------------------------------------------------------------
if [[ "${FAST}" -eq 0 ]]; then
  echo "--- CPU profiler ---"
  timeout 400 "${PYTHON_BIN}" "${PROJECT_ROOT}/src/adaptive_amr/evaluation/scripts/profile.py" \
      --repeats 2 >/tmp/verify_profile.log 2>&1 \
    && ok "CPU profiler ran" \
    || { bad "CPU profiler failed"; tail -10 /tmp/verify_profile.log; FAILURES=$((FAILURES+1)); }
fi

echo "--------------------------------------------"
if [[ "${FAILURES}" -eq 0 ]]; then
  echo -e "${GREEN}RESULT: ALL CHECKS PASSED${NC}"
  exit 0
else
  echo -e "${RED}RESULT: ${FAILURES} check(s) failed${NC}"
  exit 1
fi
