#!/usr/bin/env bash
# =============================================================================
# 02_install_python_deps.sh
# -----------------------------------------------------------------------------
# Installs the Python packages required by the perception stack.
#
# The authoritative list lives in docker/pip_requirements_phase1.txt so the
# Docker image and the host machine stay in sync (single source of truth).
#
# Usage:
#   ./scripts/02_install_python_deps.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/python_deps_$(date +%Y%m%d_%H%M%S).log"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
log()  { echo "[$(date '+%F %T')] $*" >> "${LOG_FILE}"; }

REQ_FILE="${PROJECT_ROOT}/docker/pip_requirements_phase1.txt"

main() {
  info "=== kittiraith_ws | Step 02: Python dependencies ==="

  WITH_PHASE4=0
  [[ "${1:-}" == "--with-phase4" ]] && WITH_PHASE4=1
  if [[ "${WITH_PHASE4}" -eq 1 ]]; then
    info "Including Phase 4 dependencies (torch CPU + ultralytics)."
  fi

  command -v python3 >/dev/null 2>&1 || fail "python3 not found. Install ROS Noetic first (step 00)."
  PY_VERSION="$(python3 --version 2>&1 | awk '{print $2}')"
  info "Using $(python3 --version)"

  if [[ ! -f "${REQ_FILE}" ]]; then
    fail "Requirements file not found: ${REQ_FILE}"
  fi

  # Use a virtualenv when possible (clean, no sudo); fall back to --user.
  if python3 -m venv --help >/dev/null 2>&1; then
    VENV_DIR="${PROJECT_ROOT}/.venv"
    if [[ ! -d "${VENV_DIR}" ]]; then
      info "Creating virtual environment at ${VENV_DIR}"
      python3 -m venv "${VENV_DIR}"
    fi
    # shellcheck disable=SC1091
    source "${VENV_DIR}/bin/activate"
    info "Virtual environment active: $(which python3)"
    python3 -m pip install --upgrade pip >> "${LOG_FILE}" 2>&1
    python3 -m pip install -r "${REQ_FILE}" 2>&1 | tee -a "${LOG_FILE}"
    if [[ "${WITH_PHASE4}" -eq 1 ]]; then
      python3 -m pip install --extra-index-url https://download.pytorch.org/whl/cpu \
        -r "${PROJECT_ROOT}/docker/pip_requirements_phase4.txt" 2>&1 | tee -a "${LOG_FILE}"
    fi
    info "Activate with: source ${VENV_DIR}/bin/activate"
  else
    warn "python3-venv unavailable — installing with 'pip3 install --user'."
    python3 -m pip install --user --upgrade pip >> "${LOG_FILE}" 2>&1 || true
    python3 -m pip install --user -r "${REQ_FILE}" 2>&1 | tee -a "${LOG_FILE}"
    if [[ "${WITH_PHASE4}" -eq 1 ]]; then
      python3 -m pip install --user \
        --extra-index-url https://download.pytorch.org/whl/cpu \
        -r "${PROJECT_ROOT}/docker/pip_requirements_phase4.txt" 2>&1 | tee -a "${LOG_FILE}"
    fi
  fi

  info "Python dependencies installed. Log: ${LOG_FILE}"
  info "Next: ./scripts/03_build_workspace.sh"
}

main "$@"
