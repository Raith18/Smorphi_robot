#!/usr/bin/env bash
# =============================================================================
# run_all_smoke.sh — run every phase smoke test (1..N), aggregated.
#
# Usage:
#   bash tests/run_all_smoke.sh
# Exit code: 0 = all smoke tests passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'
passed=0; failed=0
failed_list=()

for f in "${SCRIPT_DIR}"/smoke_test_phase*.sh; do
  name="$(basename "${f}")"
  if bash "${f}" > "/tmp/smoke_${name}.log" 2>&1; then
    passed=$((passed + 1))
    echo -e "${GREEN}[PASS]${NC} ${name}"
  else
    failed=$((failed + 1))
    failed_list+=("${name}")
    echo -e "${RED}[FAIL]${NC} ${name}"
    tail -15 "/tmp/smoke_${name}.log"
  fi
done

echo "--------------------------------------------"
echo "RESULT: ${passed} smoke tests passed, ${failed} failed"
if [[ "${failed}" -gt 0 ]]; then
  printf 'Failed: %s\n' "${failed_list[*]}"
  exit 1
fi
exit 0
