#!/usr/bin/env bash
# Hardware / dry-run bringup (no Gazebo): control + hw + FOV node + rosbridge + web.
# Usage:
#   ./scripts/start_hw.sh              # dry_run:=true (default)
#   DRY_RUN=false ./scripts/start_hw.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"
cd "${ROOT}"

DRY_RUN="${DRY_RUN:-true}"
ARM_ID="${ARM_ID:-arm-c}"
WEB_PORT="${WEB_PORT:-6008}"
START_WEB="${START_WEB:-true}"

echo "[start] hardware  dry_run=${DRY_RUN} arm=${ARM_ID} web=${START_WEB}:${WEB_PORT}"
exec ros2 launch sensors_arm_bringup hardware.launch.py \
  dry_run:="${DRY_RUN}" \
  arm_id:="${ARM_ID}" \
  start_web:="${START_WEB}" \
  web_port:="${WEB_PORT}" \
  "$@"
