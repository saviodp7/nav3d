#!/usr/bin/env bash
set -eo pipefail

repo_root="${NAV3D_ROOT:-/workspaces/nav3d}"
cd "${repo_root}"

source /opt/ros/jazzy/setup.bash
bash scripts/get_px4.sh
export PX4_AUTOPILOT_DIR="${repo_root}/PX4-Autopilot"

# Keep the build products on the bind-mounted workspace for the next CI steps.
make -C "${PX4_AUTOPILOT_DIR}" -j2 px4_sitl_default
colcon --log-base log build --base-paths nav3d_* external --build-base build \
  --install-base install --symlink-install --parallel-workers 2
