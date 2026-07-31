#!/usr/bin/env bash
# =============================================================================
# setup_all.sh
# -----------------------------------------------------------------------------
# One-shot Phase 1 setup: ROS -> workspace -> Python deps -> build -> KITTI.
#
# Usage:
#   ./scripts/setup_all.sh                  # everything
#   ./scripts/setup_all.sh --skip-ros       # skip OS-level ROS install (e.g. in Docker)
#   ./scripts/setup_all.sh --only 03        # run only step 03
#   ./scripts/setup_all.sh --list           # list steps
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

STEP_00="scripts/00_install_ros_noetic.sh"
STEP_01="scripts/01_create_workspace.sh"
STEP_02="scripts/02_install_python_deps.sh"
STEP_03="scripts/03_build_workspace.sh"
STEP_04="scripts/04_download_kitti_sample.sh"
ALL_STEPS=(00 01 02 03 04)

usage() {
  sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

main() {
  local skip_ros=0 only="" steps=("${ALL_STEPS[@]}")

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --skip-ros) skip_ros=1 ;;
      --only)     only="$2"; shift ;;
      --list)     usage; exit 0 ;;
      -h|--help)  usage; exit 0 ;;
      *)          fail "Unknown option: $1 (see --help)" ;;
    esac
    shift
  done

  [[ -n "${only}" ]] && steps=("${only}")

  info "Phase 1 setup — steps: ${steps[*]}"

  for s in "${steps[@]}"; do
    case "$s" in
      00)
        if [[ "${skip_ros}" -eq 1 ]]; then
          info "Skipping step 00 (ROS install) as requested."
        elif [[ -f /opt/ros/noetic/setup.bash ]]; then
          info "ROS Noetic already present — skipping step 00."
        else
          bash "${PROJECT_ROOT}/${STEP_00}"
        fi
        ;;
      01) bash "${PROJECT_ROOT}/${STEP_01}" ;;
      02) bash "${PROJECT_ROOT}/${STEP_02}" ;;
      03) bash "${PROJECT_ROOT}/${STEP_03}" ;;
      04) bash "${PROJECT_ROOT}/${STEP_04}" ;;
      *)  fail "Unknown step: $s (valid: 00 01 02 03 04)" ;;
    esac
  done

  info "Phase 1 setup finished. Run ./tests/smoke_test_phase1.sh to validate."
}

main "$@"
