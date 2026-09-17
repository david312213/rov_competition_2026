#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

for setup in /opt/ros/humble/setup.bash "${PROJECT_DIR}/.venv/bin/activate" "${PROJECT_DIR}/ros2_ws/install/setup.bash"; do
  if [[ -r "${setup}" ]]; then
    set +u
    source "${setup}"
    set -u
  fi
done

export PYTHONNOUSERSITE=1
export PYTHONPATH="${PROJECT_DIR}/ros2_ws/src/rov_competition${PYTHONPATH:+:${PYTHONPATH}}"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"
if [[ ! -r "${CONFIG}" ]]; then
  CONFIG="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
fi

cd "${PROJECT_DIR}"
exec "${BLIND_GRAB_PYTHON:-python3}" -m rov_competition.blind_grab_pwm_console \
  --config "${CONFIG}" "$@"
