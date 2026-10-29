#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${NAV3D_WORKSPACE_ROOT:-}" ]]; then
  PX4_DIR="${NAV3D_WORKSPACE_ROOT}/PX4-Autopilot"
  PX4_MSGS_DIR="${NAV3D_WORKSPACE_ROOT}/src/px4_msgs"
else
  # Preserve the standalone/CI layout outside the Dev Container.
  PX4_DIR="${ROOT_DIR}/PX4-Autopilot"
  PX4_MSGS_DIR="${ROOT_DIR}/external/px4_msgs"
fi

PX4_URL="${PX4_URL:-https://github.com/PX4/PX4-Autopilot.git}"
PX4_MSGS_URL="${PX4_MSGS_URL:-https://github.com/PX4/px4_msgs.git}"

mkdir -p "$(dirname "${PX4_DIR}")" "$(dirname "${PX4_MSGS_DIR}")"

if [ -d "${PX4_DIR}/.git" ]; then
  echo "PX4 already present: ${PX4_DIR}"
else
  echo "Cloning PX4 into: ${PX4_DIR}"
  git clone --recursive --single-branch -b v1.16.2 "${PX4_URL}" "${PX4_DIR}"
fi

if [ -d "${PX4_MSGS_DIR}/.git" ]; then
  echo "PX4 messages already present: ${PX4_MSGS_DIR}"
else
  echo "Cloning PX4 messages into: ${PX4_MSGS_DIR}"
  git clone --single-branch -b v1.16.2 "${PX4_MSGS_URL}" "${PX4_MSGS_DIR}"
fi
