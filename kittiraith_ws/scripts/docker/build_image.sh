#!/usr/bin/env bash
# =============================================================================
# build_image.sh — build the kittiraith_ws development image.
#
# Usage:
#   ./scripts/docker/build_image.sh [--no-cache]
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-kittiraith_ws:noetic}"

GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

main() {
  command -v docker >/dev/null 2>&1 || fail "docker not found. Install Docker Engine 20.10+ first."
  local extra=()
  [[ "${1:-}" == "--no-cache" ]] && extra=(--no-cache)

  info "Building ${IMAGE_NAME} (this pulls ~3 GB the first time, be patient)..."
  docker build "${extra[@]}" \
    -t "${IMAGE_NAME}" \
    -f "${PROJECT_ROOT}/docker/Dockerfile" \
    "${PROJECT_ROOT}"

  info "Image built: ${IMAGE_NAME}"
  info "Run the dev shell: ./scripts/docker/run_dev_container.sh"
}

main "$@"
