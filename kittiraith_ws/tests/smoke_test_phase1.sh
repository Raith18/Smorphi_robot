#!/usr/bin/env bash
# =============================================================================
# smoke_test_phase1.sh — validates the Phase 1 deliverable:
#   environment, workspace skeleton, scripts, config, docker files, tooling.
#
# Usage:
#   ./tests/smoke_test_phase1.sh
#
# Exit code: 0 = all critical checks passed, 1 = at least one failed.
# (Warnings do not fail the run.)
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'; NC='\033[0m'
PASS=0; FAIL=0; WARN=0

ok()   { PASS=$((PASS + 1)); echo -e "${GREEN}[PASS]${NC} $1"; }
bad()  { FAIL=$((FAIL + 1)); echo -e "${RED}[FAIL]${NC} $1"; }
warn() { WARN=$((WARN + 1)); echo -e "${YELLOW}[WARN]${NC} $1"; }

echo "=== kittiraith_ws Phase 1 smoke test ==="
echo "project root: ${PROJECT_ROOT}"

# --- 1. Python ----------------------------------------------------------------
if command -v python3 >/dev/null 2>&1; then
  v="$(python3 -c 'import sys; print("{}.{}".format(sys.version_info[0], sys.version_info[1]))' 2>/dev/null || echo unknown)"
  case "${v}" in
    3.8|3.9|3.10|3.11) ok "python3 found (${v})" ;;
    *) warn "python3 version ${v} (project target is 3.8)" ;;
  esac
else
  bad "python3 missing"
fi

# --- 2. ROS (optional on this host) ---------------------------------------------
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  ok "ROS Noetic detected (/opt/ros/noetic)"
else
  warn "ROS Noetic not installed on this host (expected when using Docker)"
fi

# --- 3. Workspace skeleton --------------------------------------------------------
[[ -d "${PROJECT_ROOT}/src/adaptive_amr" ]] \
  && ok "adaptive_amr metapackage directory exists" \
  || bad "src/adaptive_amr missing — run ./scripts/01_create_workspace.sh"

if grep -q "<metapackage/>" "${PROJECT_ROOT}/src/adaptive_amr/package.xml" 2>/dev/null; then
  ok "adaptive_amr/package.xml declares <metapackage/>"
else
  bad "adaptive_amr/package.xml missing or not a metapackage"
fi

module_count="$(find "${PROJECT_ROOT}/src/adaptive_amr" -maxdepth 2 -name package.xml \
  -not -path "*/adaptive_amr/package.xml" 2>/dev/null | wc -l)"
if [[ "${module_count}" -ge 20 ]]; then
  ok "module packages found: ${module_count}"
else
  bad "expected >= 20 module packages, found ${module_count}"
fi

# --- 4. Configuration ---------------------------------------------------------------
[[ -f "${PROJECT_ROOT}/config/kitti_dataset.yaml" ]] \
  && ok "config/kitti_dataset.yaml exists" \
  || bad "missing config/kitti_dataset.yaml"

# --- 5. Setup scripts executable ------------------------------------------------------
missing_scripts=0
for s in 00_install_ros_noetic.sh 01_create_workspace.sh 02_install_python_deps.sh \
         03_build_workspace.sh 04_download_kitti_sample.sh setup_all.sh \
         scaffold_adaptive_amr.sh; do
  if [[ ! -x "${PROJECT_ROOT}/scripts/${s}" ]]; then
    bad "not executable: scripts/${s}"
    missing_scripts=1
  fi
done
[[ "${missing_scripts}" -eq 0 ]] && ok "all setup scripts are executable"

# --- 6. Docker files -------------------------------------------------------------------
[[ -f "${PROJECT_ROOT}/docker/Dockerfile" ]] && ok "Dockerfile exists" || bad "missing Dockerfile"
[[ -f "${PROJECT_ROOT}/docker/docker-compose.yml" ]] && ok "docker-compose.yml exists" || bad "missing docker-compose.yml"
[[ -f "${PROJECT_ROOT}/docker/entrypoint.sh" ]] && ok "entrypoint.sh exists" || bad "missing entrypoint.sh"

# --- 7. KITTI validator -------------------------------------------------------------------
if [[ -f "${PROJECT_ROOT}/scripts/kitti/verify_kitti.py" ]]; then
  if python3 -m py_compile "${PROJECT_ROOT}/scripts/kitti/verify_kitti.py" >/dev/null 2>&1; then
    ok "verify_kitti.py compiles"
  else
    bad "verify_kitti.py has a syntax error"
  fi
else
  bad "missing scripts/kitti/verify_kitti.py"
fi

# --- 8. Git -----------------------------------------------------------------------------------
if git -C "${PROJECT_ROOT}" rev-parse --git-dir >/dev/null 2>&1; then
  ok "project is inside a git repository"
else
  warn "not inside a git repository"
fi

echo "------------------------------------------------------------"
echo "RESULT: ${PASS} passed, ${WARN} warnings, ${FAIL} failed"
[[ "${FAIL}" -eq 0 ]]
