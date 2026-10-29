#!/usr/bin/env bash
set -eo pipefail

repo_root="${NAV3D_ROOT:-/workspaces/nav3d}"
cd "${repo_root}"

source /opt/ros/jazzy/setup.bash
bash scripts/get_px4.sh
export PX4_AUTOPILOT_DIR="${repo_root}/PX4-Autopilot"

# PX4 SITL is prepared once here; the simulation launch only runs its binary.
make -C "${PX4_AUTOPILOT_DIR}" -j2 px4_sitl_default

colcon --log-base log build --base-paths nav3d_* external --build-base build \
  --install-base install --symlink-install --parallel-workers 2
source install/setup.bash

export ROS_LOG_DIR="${repo_root}/log/system_tests/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

if gz topic -l | grep -Eq '^/world/.*/clock$'; then
  echo "Another Gazebo world is already running; the takeoff test needs its own world." >&2
  exit 1
fi

report_file="${repo_root}/log/system_tests/takeoff_test_ci.json"
rm -f -- "${report_file}"

launch_exit_code=0
timeout --signal=INT --kill-after=30s 900s \
  ros2 run nav3d_system_tests run_system_test takeoff_test \
  headless:=true report_file:="${report_file}" || launch_exit_code=$?

if [[ ${launch_exit_code} -eq 124 || ${launch_exit_code} -eq 137 ]]; then
  echo "Takeoff test timed out (exit ${launch_exit_code})." >&2
  exit "${launch_exit_code}"
fi

# The test result is authoritative even if move_group faults during shutdown.
python3 - "${report_file}" <<'PY'
import json
import sys
from pathlib import Path

report_path = Path(sys.argv[1])
if not report_path.is_file():
    raise SystemExit(f"Takeoff test did not write a report: {report_path}")

with report_path.open(encoding="utf-8") as stream:
    report = json.load(stream)
if report.get("passed") is not True:
    raise SystemExit(f"Takeoff test failed: {report.get('failure')}")
PY

if [[ ${launch_exit_code} -ne 0 ]]; then
  echo "The flight test passed, but launch exited ${launch_exit_code} during shutdown." >&2
fi
