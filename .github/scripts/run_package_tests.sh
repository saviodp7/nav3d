#!/usr/bin/env bash
set -eo pipefail

repo_root="${NAV3D_ROOT:-/workspaces/nav3d}"
cd "${repo_root}"

source /opt/ros/jazzy/setup.bash
if [[ ! -f install/setup.bash ]]; then
  echo "Nav3D build is missing; run build_ci.sh first." >&2
  exit 1
fi
source install/setup.bash

# System tests run in their dedicated CI step. Run every test registered by the
# remaining Nav3D packages, including new tests added with ament_add_gtest.
colcon --log-base log test --base-paths nav3d_* --build-base build \
  --test-result-base log/nav3d_package_tests --packages-skip nav3d_system_tests \
  --return-code-on-test-failure
colcon test-result --test-result-base log/nav3d_package_tests --verbose
