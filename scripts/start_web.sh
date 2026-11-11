#!/usr/bin/env bash
# Web UI only (expects ROS / rosbridge already running, or uses offline FOV fallback).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${ROOT}/scripts/env.sh"
cd "${ROOT}"

PORT="${WEB_PORT:-6008}"
MODE="${MODE:-sim}"
ROSBRIDGE="${ROSBRIDGE:-ws://127.0.0.1:9090}"
PY="${SENSORS_ARM_WEB_PYTHON:-${HOME}/autodl-tmp/.venv/bin/python}"

if [[ ! -x "${PY}" ]]; then
  echo "[web] missing ${PY} (need ~/autodl-tmp/.venv with fastapi)" >&2
  exit 1
fi

echo "[web] ${PY} -m sensors_arm_web.web_server --port ${PORT} --mode ${MODE}"
echo "[web] UI → http://127.0.0.1:${PORT}/"
exec "${PY}" -m sensors_arm_web.web_server --port "${PORT}" --mode "${MODE}" --rosbridge "${ROSBRIDGE}" "$@"
