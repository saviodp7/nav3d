#!/usr/bin/env bash
set -eo pipefail

sim_root="${NAV3D_SIM_ROOT:-/workspaces/nav3d_sim}"
cd "${sim_root}"

source /opt/ros/jazzy/setup.bash

bash scripts/get_px4.sh
bash scripts/install_gz_assets.sh

colcon --log-base ros2_ws/log build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install --parallel-workers 2
source ros2_ws/install/setup.bash
export ROS_LOG_DIR="${sim_root}/ros2_ws/log/system_tests/ros_logs"
mkdir -p "${ROS_LOG_DIR}"

if gz topic -l | grep -Eq '^/world/.*/clock$'; then
  echo "Another Gazebo world is already running; baseline requires the default world." >&2
  exit 1
fi

report_file="${sim_root}/ros2_ws/log/system_tests/takeoff_test_ci.json"
rm -f -- "${report_file}"

timeout --signal=INT --kill-after=30s 900s \
  ros2 run nav3d_system_tests run_system_test takeoff_test \
  headless:=1 \
  report_file:="${report_file}"

# A move_group segfault during shutdown is currently tolerated, but the
# takeoff test itself must have produced a fresh, successful report.
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
