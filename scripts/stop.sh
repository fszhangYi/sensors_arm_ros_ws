#!/usr/bin/env bash
# Stop common sensors_arm / Gazebo / rosbridge / web processes started by these scripts.
set -euo pipefail

PATTERNS=(
  'ros2 launch sensors_arm_bringup'
  'cell_c_gazebo_fov'
  'multi_cell_sim'
  'hardware.launch'
  'sensors_arm_fov'
  'sensors_arm_control'
  'sensors_arm_hw'
  'sensors_arm_web'
  'web_server'
  'rosbridge_websocket'
  'gzserver'
  'gzclient'
  'spawn_entity.py'
  'sim_joint_driver'
)

echo "[stop] killing sensors_arm / gazebo related processes..."
for pat in "${PATTERNS[@]}"; do
  pkill -f "${pat}" 2>/dev/null || true
done
sleep 1
if pgrep -af 'gzserver|sensors_arm_bringup|rosbridge_websocket' >/dev/null 2>&1; then
  echo "[stop] some processes still alive, sending SIGKILL..."
  pkill -9 -f 'gzserver|gzclient|sensors_arm_bringup|rosbridge_websocket|sensors_arm_' 2>/dev/null || true
fi
echo "[stop] done"
