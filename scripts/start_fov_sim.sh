#!/usr/bin/env bash
# Gazebo Cell C + D405 camera + FOV validate node.
# GUI: GUI=true ./scripts/start_fov_sim.sh   or   ./scripts/start_fov_sim.sh gui:=true
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"
cd "${ROOT}"

GUI="${GUI:-false}"

echo "[start] cell_c_gazebo_fov  gui=${GUI}  (FOV service: /fov/validate)"
exec ros2 launch sensors_arm_bringup cell_c_gazebo_fov.launch.py \
  gui:="${GUI}" \
  "$@"
