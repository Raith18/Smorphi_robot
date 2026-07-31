#!/usr/bin/env bash
# =============================================================================
# run_integration_tests.sh — run the headless end-to-end pipeline integration
# test (no ROS, no GPU): synthetic KITTI-like drive through the real modules
# (lidar_processing -> occupancy_grid -> navigation_layer -> behavior ->
# semantic_mapping -> motion_prediction).
#
# Usage:
#   bash tests/run_integration_tests.sh
# Exit code: 0 = integration test passed.
# =============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GREEN='\033[0;32m'; RED='\033[0;31m'; NC='\033[0m'

PYTHON_BIN="python3"
if [[ -f "${PROJECT_ROOT}/.venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT_ROOT}/.venv/bin/activate"
fi

echo "=== kittiraith_ws integration tests ==="
if "$PYTHON_BIN" "${SCRIPT_DIR}/integration/test_pipeline_integration.py"; then
  echo -e "${GREEN}[PASS]${NC} pipeline integration test"
  exit 0
else
  echo -e "${RED}[FAIL]${NC} pipeline integration test"
  exit 1
fi
