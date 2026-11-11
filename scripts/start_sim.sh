#!/usr/bin/env bash
# Full sim: Gazebo FOV + arm_control + hw dry-run + rosbridge + Web UI :6008
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"
cd "${ROOT}"

WEB_PORT="${WEB_PORT:-6008}"
START_WEB="${START_WEB:-true}"
GUI="${GUI:-false}"

echo "[start] multi_cell_sim  web=${START_WEB} port=${WEB_PORT} gui=${GUI}"
echo "[start] UI → http://127.0.0.1:${WEB_PORT}/"
exec ros2 launch sensors_arm_bringup multi_cell_sim.launch.py \
  start_web:="${START_WEB}" \
  web_port:="${WEB_PORT}" \
  gui:="${GUI}" \
  "$@"
