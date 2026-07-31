#!/usr/bin/env bash
# =============================================================================
# 03_build_workspace.sh
# -----------------------------------------------------------------------------
# Builds the catkin workspace.
#
#   - Uses `catkin build` (catkin_tools) when available (recommended),
#   - falls back to `catkin_make`.
#
# Usage:
#   ./scripts/03_build_workspace.sh [--force]
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/build_$(date +%Y%m%d_%H%M%S).log"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
log()  { echo "[$(date '+%F %T')] $*" >> "${LOG_FILE}"; }

FORCE=0
[[ "${1:-}" == "--force" ]] && FORCE=1

main() {
  info "=== kittiraith_ws | Step 03: workspace build ==="

  if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
    fail "ROS Noetic not found (/opt/ros/noetic). Run ./scripts/00_install_ros_noetic.sh or use Docker."
  fi
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
  info "Sourced /opt/ros/noetic/setup.bash (ROS ${ROS_DISTRO})"

  local use_catkin_tools=0
  if command -v catkin >/dev/null 2>&1; then use_catkin_tools=1; fi

  if [[ "${use_catkin_tools}" -eq 1 ]]; then
    info "Build system: catkin_tools (catkin build)"
    catkin build -j "$(nproc)" 2>&1 | tee -a "${LOG_FILE}"
    # shellcheck disable=SC1091
    source "${PROJECT_ROOT}/devel/setup.bash"
  else
    warn "catkin_tools not found — falling back to catkin_make."
    if [[ ! -f "${PROJECT_ROOT}/src/CMakeLists.txt" ]]; then
      (cd "${PROJECT_ROOT}" && catkin_init_workspace src) >> "${LOG_FILE}" 2>&1
    fi
    (cd "${PROJECT_ROOT}" && catkin_make -j"$(nproc)") 2>&1 | tee -a "${LOG_FILE}"
    # shellcheck disable=SC1091
    source "${PROJECT_ROOT}/devel/setup.bash"
  fi

  info "Build finished. Log: ${LOG_FILE}"
  info "Workspace packages:"
  (cd "${PROJECT_ROOT}" && catkin list 2>/dev/null || ls -1 "${PROJECT_ROOT}/src/adaptive_amr")

  info "Source the workspace in every shell: source ${PROJECT_ROOT}/devel/setup.bash"
  info "Next: ./scripts/04_download_kitti_sample.sh"
}

main "$@"
