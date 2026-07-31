#!/usr/bin/env bash
# =============================================================================
# run_dev_container.sh — launch an interactive dev shell with:
#   * the workspace bind-mounted at /home/ros/kittiraith_ws (live edits)
#   * a named volume for KITTI data at /data/kitti
#   * X11 forwarding (Linux) so rviz/rqt can open
#
# Usage:
#   ./scripts/docker/run_dev_container.sh
#
# GPU (Phases 4+): add `--gpus all` after installing nvidia-container-toolkit.
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-kittiraith_ws:noetic}"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

main() {
  command -v docker >/dev/null 2>&1 || fail "docker not found."

  # --- Build the image on first use ------------------------------------------
  if [[ -z "$(docker images -q "${IMAGE_NAME}" 2>/dev/null)" ]]; then
    info "Image ${IMAGE_NAME} not found — building it first..."
    bash "${SCRIPT_DIR}/build_image.sh"
  fi

  # --- X11 forwarding (Linux) --------------------------------------------------
  local x11_flags=()
  if [[ -n "${DISPLAY:-}" ]] && command -v xauth >/dev/null 2>&1; then
    local xauth="/tmp/.docker.xauth"
    touch "${xauth}"
    xauth nlist "${DISPLAY}" 2>/dev/null | sed -e 's/^/add /' \
      | xauth -f "${xauth}" nmerge - 2>/dev/null || true
    chmod 600 "${xauth}"
    x11_flags=(
      -e "DISPLAY=${DISPLAY}"
      -e QT_X11_NO_MITSHM=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
      -v "${xauth}":/tmp/.docker.xauth:rw
      -e XAUTHORITY=/tmp/.docker.xauth
    )
  else
    warn "DISPLAY not set — GUI tools (rviz, rqt) will not open inside the container."
  fi

  info "Starting dev shell (${IMAGE_NAME})..."
  docker run -it --rm \
    --name kittiraith_ws_dev \
    --network host \
    "${x11_flags[@]}" \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -v "${PROJECT_ROOT}:/home/ros/kittiraith_ws" \
    -v kitti_data:/data/kitti \
    -w /home/ros/kittiraith_ws \
    "${IMAGE_NAME}" \
    bash
}

main "$@"
