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

# Only CTest cases explicitly labeled "unit" belong to this phase.
# The current tree has none; this excludes lint and generated px4_msgs tests.
colcon --log-base log test --base-paths nav3d_* --build-base build \
  --test-result-base build --return-code-on-test-failure \
  --ctest-args -L '^unit$'
colcon test-result --test-result-base build --verbose
