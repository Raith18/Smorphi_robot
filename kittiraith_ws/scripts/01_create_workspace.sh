#!/usr/bin/env bash
# =============================================================================
# 01_create_workspace.sh
# -----------------------------------------------------------------------------
# Creates the catkin workspace layout:
#   kittiraith_ws/src/            (catkin source space)
#   kittiraith_ws/src/adaptive_amr/   (metapackage + 20 module package skeletons)
#   kittiraith_ws/data/           (git-ignored: datasets, bags)
#   kittiraith_ws/logs/           (git-ignored: setup/build logs)
#
# Usage:
#   ./scripts/01_create_workspace.sh
#
# Idempotent: existing files are preserved.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/workspace_$(date +%Y%m%d_%H%M%S).log"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
log()  { echo "[$(date '+%F %T')] $*" >> "${LOG_FILE}"; }

create_workspace_layout() {
  # Source space
  mkdir -p "${PROJECT_ROOT}/src"
  log "Ensured src/ exists"

  # catkin_init_workspace creates src/CMakeLists.txt (needed by catkin_make;
  # catkin build does not require it, but harmless).
  if command -v catkin_init_workspace >/dev/null 2>&1; then
    if [[ ! -f "${PROJECT_ROOT}/src/CMakeLists.txt" ]]; then
      (cd "${PROJECT_ROOT}" && catkin_init_workspace src >> "${LOG_FILE}" 2>&1) \
        || warn "catkin_init_workspace failed (catkin build will still work)."
      log "Ran catkin_init_workspace"
    fi
  else
    warn "catkin_init_workspace not found (ROS not installed on this host?)."
    warn "If you use catkin_make later, run: cd ${PROJECT_ROOT} && catkin_init_workspace src"
  fi

  # Git-ignored data & logs directories (keep the .gitkeep files tracked)
  mkdir -p "${PROJECT_ROOT}/data" "${PROJECT_ROOT}/logs"
  touch "${PROJECT_ROOT}/data/.gitkeep" "${PROJECT_ROOT}/logs/.gitkeep"

  # Shared directories at workspace level (mirrors the intended final repo layout)
  mkdir -p "${PROJECT_ROOT}/config" "${PROJECT_ROOT}/docs" "${PROJECT_ROOT}/tests" \
           "${PROJECT_ROOT}/docker" "${PROJECT_ROOT}/scripts/kitti" \
           "${PROJECT_ROOT}/scripts/docker"
  log "Ensured top-level directories exist"
}

main() {
  info "=== kittiraith_ws | Step 01: workspace creation ==="
  log "Creating workspace layout under ${PROJECT_ROOT}"
  create_workspace_layout
  if [[ -f "${PROJECT_ROOT}/scripts/scaffold_adaptive_amr.sh" ]]; then
    info "Scaffolding the adaptive_amr metapackage + module packages..."
    bash "${PROJECT_ROOT}/scripts/scaffold_adaptive_amr.sh"
  else
    warn "scaffold_adaptive_amr.sh not found — skipping package skeleton."
  fi
  info "Workspace created. Log: ${LOG_FILE}"
  info "Next: ./scripts/02_install_python_deps.sh  (or build: ./scripts/03_build_workspace.sh)"
}

main "$@"
