#!/usr/bin/env bash
# FOV CLI helper.
#   ./scripts/fov_cli.sh              # offline projection HD87
#   ./scripts/fov_cli.sh --ros        # call /fov/validate
#   ./scripts/fov_cli.sh --mode VGA80 --zip /tmp/fov.zip
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"

CFG="$(ros2 pkg prefix sensors_arm_fov)/share/sensors_arm_fov/config/cell_c_fov.yaml"
MODE="HD87"
EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done

echo "[fov] config=${CFG} mode=${MODE} args=${EXTRA[*]:-}"
exec ros2 run sensors_arm_fov fov_cli -c "${CFG}" --mode "${MODE}" "${EXTRA[@]}"
