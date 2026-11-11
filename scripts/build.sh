#!/usr/bin/env bash
# Rebuild sensors_arm_ros_ws with system Python 3.8.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"
cd "${ROOT}"
echo "[build] workspace=${ROOT} python=$(command -v python3)"
colcon build --symlink-install --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 "$@"
# shellcheck disable=SC1091
source "${ROOT}/install/setup.bash"
echo "[build] done — source ${ROOT}/scripts/env.sh  (or re-open shell)"
