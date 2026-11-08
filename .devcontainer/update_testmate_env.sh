#!/usr/bin/env bash
set -eo pipefail

workspace_root="${NAV3D_WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_root}/install/setup.bash" ]]; then
  source "${workspace_root}/install/setup.bash"
fi

python3 - "${workspace_root}/.testmate-env.json" <<'PY'
import json
import os
import sys
import tempfile
from pathlib import Path

destination = Path(sys.argv[1])
names = (
    "AMENT_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PYTHONPATH",
    "PATH",
    "ROS_DISTRO",
    "ROS_VERSION",
    "ROS_PYTHON_VERSION",
    "RMW_IMPLEMENTATION",
    "ROS_DOMAIN_ID",
    "ROS_LOCALHOST_ONLY",
    "GZ_SIM_RESOURCE_PATH",
)
environment = {name: os.environ[name] for name in names if name in os.environ}
with tempfile.NamedTemporaryFile(
    mode="w", encoding="utf-8", dir=destination.parent, delete=False
) as stream:
    json.dump(environment, stream, indent=2)
    stream.write("\n")
    temporary_path = Path(stream.name)
temporary_path.replace(destination)
PY
