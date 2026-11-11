#!/usr/bin/env bash
# Source ROS 2 Foxy + this workspace. Safe to `source` from other scripts.
# Usage: source "$(dirname "$0")/env.sh"

# Prefer system Python 3.8 for Foxy (avoid conda hijacking).
_ros_path="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:${HOME}/.local/bin"
case ":${PATH}:" in
  *:/usr/bin:*) ;;
  *) export PATH="${_ros_path}:${PATH}" ;;
esac
# Drop conda shims from PATH if present (keep rest).
if command -v conda >/dev/null 2>&1 || [[ "${PATH}" == *miniconda* || "${PATH}" == *anaconda* ]]; then
  export PATH="$(printf '%s' "${PATH}" | tr ':' '\n' | grep -v -E 'miniconda|anaconda|conda/bin' | paste -sd: -)"
  export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:${HOME}/.local/bin:${PATH}"
fi

_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SENSORS_ARM_ROS_WS="${SENSORS_ARM_ROS_WS:-${_WS_ROOT}}"
export SENSORS_SRC="${SENSORS_SRC:-${HOME}/autodl-tmp/sensors/src}"

# Isolate this stack from other ROS 2 processes via domain id (e.g. snap
# foxglove_bridge on Jazzy / domain 0). Keep LOCALHOST_ONLY off by default:
# Gazebo Classic's ROS plugins often fail service RPC under LOCALHOST_ONLY=1
# (set_model_configuration hangs → arm pose never updates in gz).
# Override with SENSORS_ARM_ROS_DOMAIN_ID / SENSORS_ARM_ROS_LOCALHOST_ONLY if needed.
export ROS_DOMAIN_ID="${SENSORS_ARM_ROS_DOMAIN_ID:-31}"
export ROS_LOCALHOST_ONLY="${SENSORS_ARM_ROS_LOCALHOST_ONLY:-0}"

# Avoid nounset failures inside ROS setup scripts.
set +u
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
if [[ -f "${SENSORS_ARM_ROS_WS}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${SENSORS_ARM_ROS_WS}/install/setup.bash"
else
  echo "[env] WARN: ${SENSORS_ARM_ROS_WS}/install/setup.bash missing — run scripts/build.sh first" >&2
fi
set -u

# Gazebo Classic: must source setup.sh first so media/rtshaderlib is on
# GAZEBO_RESOURCE_PATH (otherwise CameraSensor fails: "Unable to find shader lib").
set +u
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo/setup.sh
elif [[ -f /usr/share/gazebo-11/setup.sh ]]; then
  # shellcheck disable=SC1091
  source /usr/share/gazebo-11/setup.sh
fi
set -u

# Prepend workspace package shares so package:// → model:// resolves locally.
_gazebo_extra=()
for _pkg in sensors_arm_description sensors_arm_gazebo; do
  if _prefix="$(ros2 pkg prefix "${_pkg}" 2>/dev/null)"; then
    _gazebo_extra+=("${_prefix}/share")
  fi
done
if ((${#_gazebo_extra[@]})); then
  _joined="$(IFS=:; echo "${_gazebo_extra[*]}")"
  # Package-local Gazebo models (e.g. fov_legend_board) live under share/<pkg>/models.
  _model_dirs=()
  for _share in "${_gazebo_extra[@]}"; do
    [[ -d "${_share}/sensors_arm_gazebo/models" ]] && _model_dirs+=("${_share}/sensors_arm_gazebo/models")
    [[ -d "${_share}/models" ]] && _model_dirs+=("${_share}/models")
  done
  _models_joined="$(IFS=:; echo "${_model_dirs[*]}")"
  export GAZEBO_MODEL_PATH="${_models_joined:+${_models_joined}:}${_joined}${GAZEBO_MODEL_PATH:+:}${GAZEBO_MODEL_PATH:-}"
  # Keep system /usr/share/gazebo-11 (media + shaders) ahead of package shares.
  export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-/usr/share/gazebo-11}:${_joined}"
fi
# Disable online model DB (failed downloads only slow startup).
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI-}"
# Force Gazebo master/advertising onto loopback. Otherwise gz picks Docker's
# 172.17.0.1 and ROS_LOCALHOST_ONLY=1 peers never reach /gazebo/* services
# (arm_control cannot SetModelConfiguration → GUI arm frozen).
export GAZEBO_IP="${GAZEBO_IP:-127.0.0.1}"
export GAZEBO_MASTER_URI="${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}"
# Workspace Gazebo plugins (camera_params frustum etc.)
if _gz_prefix="$(ros2 pkg prefix sensors_arm_gazebo 2>/dev/null)"; then
  export GAZEBO_PLUGIN_PATH="${_gz_prefix}/lib${GAZEBO_PLUGIN_PATH:+:}${GAZEBO_PLUGIN_PATH:-}"
fi
unset _gz_prefix
# Headless / no NVIDIA: software GL so Gazebo CameraSensor can render.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

# Web UI (FastAPI) lives in the autodl-tmp venv; prepend for launch scripts that need it.
_VENV="${HOME}/autodl-tmp/.venv"
if [[ -x "${_VENV}/bin/python" ]]; then
  export SENSORS_ARM_WEB_PYTHON="${SENSORS_ARM_WEB_PYTHON:-${_VENV}/bin/python}"
fi
