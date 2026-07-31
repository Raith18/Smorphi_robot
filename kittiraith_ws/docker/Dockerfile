# =============================================================================
# kittiraith_ws — development image
# ROS Noetic (desktop-full) on Ubuntu 20.04, ready for the adaptive_amr stack.
#
# Build:   ./scripts/docker/build_image.sh
#   (or)   docker build -t kittiraith_ws:noetic -f docker/Dockerfile .
#
# Notes:
#   * Base image: official ROS image `ros:noetic-desktop-full-focal`.
#     For a leaner image, swap to `ros:noetic-ros-base-focal` and add the
#     perception packages you actually need (pcl-ros, cv-bridge, rviz...).
#   * PyTorch (CPU/GPU) is intentionally NOT installed in Phase 1 — it is
#     added in Phase 4 (Detection/Segmentation) so this image stays lean.
#   * GPU support (--gpus all / nvidia-container-toolkit) is documented in
#     docker-compose.yml and scripts/docker/run_dev_container.sh.
# =============================================================================
FROM ros:noetic-desktop-full-focal

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_WS=/home/ros/kittiraith_ws

# --- System packages ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git git-lfs \
    curl wget \
    vim tmux htop tree bash-completion \
    python3-pip python3-tk python3-venv \
    python3-catkin-tools python3-rosdep \
    python3-rosinstall python3-rosinstall-generator python3-wstool \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-image-transport \
    ros-noetic-image-transport-plugins \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-camera-calibration \
    && rm -rf /var/lib/apt/lists/*

# --- Python dependencies (kept in sync with scripts/02_install_python_deps.sh) ---
COPY docker/pip_requirements_phase1.txt /tmp/pip_requirements_phase1.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/pip_requirements_phase1.txt \
    && rm -f /tmp/pip_requirements_phase1.txt

# --- Non-root developer user ---------------------------------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && mkdir -p "${ROS_WS}/src" \
    && chown -R "${USERNAME}:${USERNAME}" "/home/${USERNAME}"

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

USER "${USERNAME}"
WORKDIR "${ROS_WS}"

# entrypoint sources ROS + (if present) the workspace overlay
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
