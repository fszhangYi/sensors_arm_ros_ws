#!/usr/bin/env bash
# Wire sibling product/core repos the same way sensors-view / sensors-arm do:
#   <ws>/sensors      → ../sensors
#   <ws>/sensors-arm  → ../sensors-arm
# Override with SENSORS_REPO / SENSORS_ARM_REPO (absolute or relative paths).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PARENT="$(cd "${ROOT}/.." && pwd)"
SENSORS_REPO="${SENSORS_REPO:-${PARENT}/sensors}"
SENSORS_ARM_REPO="${SENSORS_ARM_REPO:-${PARENT}/sensors-arm}"

link_one() {
  local target="$1" link="$2" label="$3"
  if [[ ! -e "${target}" && ! -L "${target}" ]]; then
    echo "[link_deps] WARN: missing ${label} at ${target}" >&2
    return 0
  fi
  # Prefer a relative sibling link when target lives next to this workspace.
  local link_val="${target}"
  if [[ "$(cd "$(dirname "${target}")" && pwd)" == "${PARENT}" ]]; then
    link_val="../$(basename "${target}")"
  fi
  ln -sfn "${link_val}" "${link}"
  echo "[link_deps] ${link} → $(readlink "${link}")"
}

link_one "${SENSORS_REPO}" "${ROOT}/sensors" "sensors"
link_one "${SENSORS_ARM_REPO}" "${ROOT}/sensors-arm" "sensors-arm"
