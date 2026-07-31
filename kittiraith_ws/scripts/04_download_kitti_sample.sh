#!/usr/bin/env bash
# =============================================================================
# 04_download_kitti_sample.sh
# -----------------------------------------------------------------------------
# Downloads a small KITTI RAW sample (drive 2011_09_26_drive_0005: stereo
# images, Velodyne point clouds, GPS/IMU oxts, calibration) and validates it.
#
# This is the "getting started" dataset — enough to develop every module in
# Phases 2-7. For the full dataset (all 11 odometry sequences / all raw
# drives) see the reference URLs at the bottom of this file.
#
# Usage:
#   ./scripts/04_download_kitti_sample.sh
#   KITTI_ROOT=/data/kitti ./scripts/04_download_kitti_sample.sh
#
# Env overrides:
#   KITTI_ROOT   target dataset root (default: <project>/data/kitti)
#   KITTI_DRIVE  sample drive id (default: 2011_09_26_drive_0005)
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
mkdir -p "${LOG_DIR}"
LOG_FILE="${LOG_DIR}/kitti_download_$(date +%Y%m%d_%H%M%S).log"

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
log()  { echo "[$(date '+%F %T')] $*" >> "${LOG_FILE}"; }

# --- Configuration (overridable via environment) ------------------------------
KITTI_BASE_URL="${KITTI_BASE_URL:-https://s3.eu-central-1.amazonaws.com/avg-kitti}"
KITTI_ROOT="${KITTI_ROOT:-${PROJECT_ROOT}/data/kitti}"
KITTI_DATE="2011_09_26"
KITTI_DRIVE="${KITTI_DRIVE:-2011_09_26_drive_0005}"

# Files to fetch (official KITTI layout on the avg-kitti S3 bucket).
#   calib      -> camera/LiDAR/IMU calibration (small)
#   sync       -> time-synchronized stereo + velodyne + oxts (the ROS-replay set)
declare -A FILES=(
  ["${KITTI_DATE}_calib.zip"]="${KITTI_BASE_URL}/raw_data/${KITTI_DATE}/${KITTI_DATE}_calib.zip"
  ["${KITTI_DRIVE}_sync.zip"]="${KITTI_BASE_URL}/raw_data/${KITTI_DATE}/${KITTI_DRIVE}_sync.zip"
)

main() {
  info "=== kittiraith_ws | Step 04: KITTI sample download ==="
  log "KITTI_ROOT=${KITTI_ROOT}  DRIVE=${KITTI_DRIVE}"

  for tool in wget unzip; do
    command -v "${tool}" >/dev/null 2>&1 || fail "${tool} not found. Install it with: sudo apt-get install -y ${tool}"
  done

  local raw_dir="${KITTI_ROOT}/raw"
  mkdir -p "${raw_dir}/_downloads"
  info "Dataset root: ${KITTI_ROOT}"

  # --- Download (resumable) ---------------------------------------------------
  local name url
  for name in "${!FILES[@]}"; do
    url="${FILES[$name]}"
    if [[ -f "${raw_dir}/${name}" ]]; then
      info "Already present, skipping: ${name}"
      continue
    fi
    info "Downloading ${name} ..."
    wget -c --show-progress -q -O "${raw_dir}/_downloads/${name}" "${url}" \
      >> "${LOG_FILE}" 2>&1 || fail "Download failed for ${url} (see ${LOG_FILE})."
  done

  # --- Extract ------------------------------------------------------------------
  info "Extracting archives into ${raw_dir} ..."
  ( cd "${raw_dir}" && unzip -q -o _downloads/*.zip ) >> "${LOG_FILE}" 2>&1 \
    || fail "Unzip failed (see ${LOG_FILE})."
  info "Extraction done."

  # --- Validate ------------------------------------------------------------------
  if command -v python3 >/dev/null 2>&1; then
    info "Validating dataset structure ..."
    python3 "${PROJECT_ROOT}/scripts/kitti/verify_kitti.py" --root "${KITTI_ROOT}" \
      || warn "Validation reported problems (see output above)."
  else
    warn "python3 not found — skipped structure validation."
  fi

  # --- Report ---------------------------------------------------------------------
  info "KITTI sample ready at: ${KITTI_ROOT}"
  echo
  ( cd "${raw_dir}" && find . -maxdepth 2 -type d | sort | head -25 )
  echo
  info "Sizes:"
  du -sh "${raw_dir}" 2>/dev/null || true
  echo
  info "Next steps:"
  info "  1) Phase 2 will replay this data with kitti2bag -> rosbag -> ROS topics."
  info "     Install the converter now?  pip install kitti2bag"
  info "  2) Full dataset reference (download only what you need):"
  info "     - Odometry color : ${KITTI_BASE_URL}/odometry/data_odometry_color.zip   (7.4 GB)"
  info "     - Odometry gray  : ${KITTI_BASE_URL}/odometry/data_odometry_gray.zip    (4.2 GB)"
  info "     - Odometry velodyne: ${KITTI_BASE_URL}/odometry/data_odometry_velodyne.zip (29 GB)"
  info "     - Odometry calib : ${KITTI_BASE_URL}/odometry/data_odometry_calib.zip"
  info "     - Odometry poses : ${KITTI_BASE_URL}/odometry/data_odometry_poses.zip"
  info "     Official pages : https://www.cvlibs.net/datasets/kitti/raw_data.php ,"
  info "                      https://www.cvlibs.net/datasets/kitti/eval_odometry.php"
  info "Log: ${LOG_FILE}"
}

main "$@"
