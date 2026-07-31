#!/usr/bin/env bash
# =============================================================================
# run_unit_tests.sh — discover and run EVERY pure-algorithm unit test in the
# workspace, aggregating results into a single pass/fail summary.
#
# Usage:
#   bash tests/run_unit_tests.sh [--verbose]
# Exit code: 0 = all test files passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERBOSE=0
[[ "${1:-}" == "--verbose" ]] && VERBOSE=1

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

mapfile -t TEST_FILES < <(find "${PROJECT_ROOT}/src" -name "test_*.py" \
  -not -path "*/.venv/*" | sort)

total_files=0; passed_files=0; failed_files=0
failed_list=()

echo "=== kittiraith_ws unit test runner ==="
echo "found ${#TEST_FILES[@]} test files"

for f in "${TEST_FILES[@]}"; do
  total_files=$((total_files + 1))
  rel="${f#"${PROJECT_ROOT}/"}"
  if "$PYTHON_BIN" "$f" > "/tmp/unit_${total_files}.log" 2>&1; then
    passed_files=$((passed_files + 1))
    if [[ "${VERBOSE}" -eq 1 ]]; then
      echo -e "${GREEN}[PASS]${NC} ${rel}: $(grep -E '^Ran ' "/tmp/unit_${total_files}.log")"
    fi
  else
    failed_files=$((failed_files + 1))
    failed_list+=("${rel}")
    echo -e "${RED}[FAIL]${NC} ${rel}"
    tail -15 "/tmp/unit_${total_files}.log"
  fi
done

echo "--------------------------------------------"
echo "RESULT: ${passed_files}/${total_files} test files passed, ${failed_files} failed"
if [[ "${failed_files}" -gt 0 ]]; then
  echo "Failed:"
  for f in "${failed_list[@]}"; do echo "  - ${f}"; done
  exit 1
fi
exit 0
