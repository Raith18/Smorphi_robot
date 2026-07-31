#!/usr/bin/env bash
# =============================================================================
# 00_install_ros_noetic.sh
# -----------------------------------------------------------------------------
# Installs ROS Noetic (desktop-full) + catkin_tools + rosdep on Ubuntu 20.04.
#
# Usage:
#   ./scripts/00_install_ros_noetic.sh          # interactive (asks before apt)
#   NONINTERACTIVE=1 ./scripts/00_install_ros_noetic.sh   # no prompt
#
# Idempotent: safe to re-run (apt skips already-installed packages).
# Logs: <project_root>/logs/install_ros_<timestamp>.log
# =============================================================================
set -euo pipefail

# --- Project root discovery (no hardcoded paths) ----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/install_ros_$(date +%Y%m%d_%H%M%S).log"

# --- Colored output helpers --------------------------------------------------
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()  { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
log()   { echo "[$(date '+%F %T')] $*" >> "${LOG_FILE}"; }

# Use sudo only when not root.
if [[ "${EUID}" -eq 0 ]]; then SUDO=(); else SUDO=(sudo); fi

# Non-interactive apt (stability: never block on debconf prompts).
export DEBIAN_FRONTEND=noninteractive

# --- 1. Verify OS ------------------------------------------------------------
check_os() {
  if [[ ! -f /etc/os-release ]]; then
    fail "Cannot detect the OS. This script requires Ubuntu 20.04 (Focal Fossa)."
  fi
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID}" != "ubuntu" || "${VERSION_ID}" != "20.04" ]]; then
    warn "Detected: ${PRETTY_NAME:-unknown}. ROS Noetic is only supported on Ubuntu 20.04."
    warn "If you are on another OS, use the Docker environment instead:"
    warn "  ./scripts/docker/build_image.sh && ./scripts/docker/run_dev_container.sh"
    fail "Aborting ROS installation."
  fi
  info "OS check passed: ${PRETTY_NAME}"
}

# --- 2. Base packages --------------------------------------------------------
install_base_tools() {
  info "Installing base tools (curl, gnupg2, lsb-release, software-properties-common)..."
  "${SUDO[@]}" apt-get update --fix-missing >> "${LOG_FILE}" 2>&1 || { tail -20 "${LOG_FILE}"; fail "apt-get update failed."; }
  "${SUDO[@]}" apt-get install -y --no-install-recommends \
    curl gnupg2 lsb-release software-properties-common \
    >> "${LOG_FILE}" 2>&1 || { tail -20 "${LOG_FILE}"; fail "Base tool installation failed."; }
}

# --- 3. Add ROS apt repository ----------------------------------------------
add_ros_repo() {
  if [[ -f /etc/apt/sources.list.d/ros-latest.list ]]; then
    info "ROS apt repository already configured — skipping."
    return 0
  fi
  info "Adding ROS Noetic apt repository (packages.ros.org)..."
  curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | "${SUDO[@]}" apt-key add - >> "${LOG_FILE}" 2>&1
  "${SUDO[@]}" add-apt-repository -y \
    "deb http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" >> "${LOG_FILE}" 2>&1
  "${SUDO[@]}" apt-get update >> "${LOG_FILE}" 2>&1
  info "ROS apt repository added."
}

# --- 4. Install ROS Noetic desktop-full + build tooling ----------------------
install_ros() {
  if [[ -d /opt/ros/noetic ]]; then
    info "/opt/ros/noetic already exists — skipping ROS installation."
    return 0
  fi
  info "Installing ros-noetic-desktop-full + catkin tooling (~2 GB, please be patient)..."
  "${SUDO[@]}" apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator \
    python3-wstool python3-catkin-tools build-essential \
    >> "${LOG_FILE}" 2>&1 || { tail -30 "${LOG_FILE}"; fail "ROS installation failed."; }
}

# --- 5. rosdep ------------------------------------------------------------------
setup_rosdep() {
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    info "Running 'rosdep init'..."
    "${SUDO[@]}" rosdep init >> "${LOG_FILE}" 2>&1 || true
  fi
  info "Running 'rosdep update'..."
  rosdep update >> "${LOG_FILE}" 2>&1 || warn "rosdep update had warnings (see ${LOG_FILE})."
}

# --- 6. Shell environment -------------------------------------------------------
configure_shell() {
  local RC="${HOME}/.bashrc"
  if ! grep -q "/opt/ros/noetic/setup.bash" "${RC}" 2>/dev/null; then
    echo "source /opt/ros/noetic/setup.bash" >> "${RC}"
    info "Added 'source /opt/ros/noetic/setup.bash' to ${RC}"
  fi
}

# --- Main ------------------------------------------------------------------------
main() {
  info "=== kittiraith_ws | Step 00: ROS Noetic installation ==="
  log "Starting ROS Noetic installation on $(hostname)"
  check_os
  install_base_tools
  add_ros_repo
  install_ros
  setup_rosdep
  configure_shell
  info "ROS Noetic installation complete. Log: ${LOG_FILE}"
  info "Next: ./scripts/01_create_workspace.sh"
}

main "$@"
