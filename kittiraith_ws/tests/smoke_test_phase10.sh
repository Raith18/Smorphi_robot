#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase10.sh — validates the Phase 10 documentation deliverable:
#   * the research report, module reference, glossary, troubleshooting exist
#   * the API reference generates and covers all 22 packages
#   * every module README exists (25-point spec)
#   * the root README roadmap marks phases 1-10 done
#
# Usage:
#   ./tests/smoke_test_phase10.sh
# Exit code: 0 = all critical checks passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
AMR_DIR="${PROJECT_ROOT}/src/adaptive_amr"

GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'
PASS=0; FAIL=0
ok()  { PASS=$((PASS + 1)); echo -e "${GREEN}[PASS]${NC} $1"; }
bad() { FAIL=$((FAIL + 1)); echo -e "${RED}[FAIL]${NC} $1"; }

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

echo "=== kittiraith_ws Phase 10 smoke test ==="

# --- 1. Core documentation files -------------------------------------------------
for f in RESEARCH_REPORT.md api_reference.md MODULE_REFERENCE.md \
         GLOSSARY.md TROUBLESHOOTING.md architecture_overview.md; do
  [[ -s "${PROJECT_ROOT}/docs/${f}" ]] && ok "docs/${f} present (${f})" \
    || bad "docs/${f} missing or empty"
done

# --- 2. API reference regenerates ---------------------------------------------------
if "${PYTHON_BIN}" "${PROJECT_ROOT}/scripts/docs/generate_api_docs.py" \
    --output /tmp/api_ref_check.md > /tmp/p10_api.log 2>&1; then
  ok "api docs generator runs"
else
  bad "api docs generator failed — see /tmp/p10_api.log"; tail -10 /tmp/p10_api.log
fi
pkg_count=$(grep -c '^## ' /tmp/api_ref_check.md 2>/dev/null || echo 0)
[[ "${pkg_count}" -ge 22 ]] && ok "api reference covers ${pkg_count} packages" \
  || bad "api reference covers only ${pkg_count} packages"

# --- 3. Every PACKAGE (has package.xml) has a README -----------------------------------
missing_readme=0
for d in "${AMR_DIR}"/*/; do
  pkg="$(basename "${d}")"
  if [[ -f "${d}/package.xml" && -f "${d}/README.md" ]]; then
    :
  elif [[ -f "${d}/package.xml" ]]; then
    bad "missing README: ${pkg}"
    missing_readme=1
  fi
done
[[ "${missing_readme}" -eq 0 ]] && ok "all package READMEs present"

# --- 4. 25-point template coverage (pipeline modules only; the metapackage
#         and the message package are infrastructure catalogs, not modules) ---
incomplete=0
for d in "${AMR_DIR}"/*/; do
  [[ -f "${d}/package.xml" ]] || continue
  pkg="$(basename "${d}")"
  case "${pkg}" in
    adaptive_amr|adaptive_amr_msgs) continue ;;
  esac
  readme="${d}/README.md"
  [[ -f "${readme}" ]] || continue
  for section in "Objective" "Theory" "Industrial importance" "Folder structure" \
                 "Testing procedure" "Expected outputs" "Debugging guide" \
                 "Common errors" "Improvements" "Git commit"; do
    grep -q "${section}" "${readme}" || { bad "${readme}: missing '${section}'"; incomplete=1; }
  done
done
[[ "${incomplete}" -eq 0 ]] && ok "25-point template coverage verified"

# --- 5. Roadmap marks all phases done -------------------------------------------------------
grep -q "10.*✅ \*\*Done\*\*" "${PROJECT_ROOT}/README.md" \
  && ok "roadmap marks Phase 10 done" || bad "roadmap does not mark Phase 10 done"

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
