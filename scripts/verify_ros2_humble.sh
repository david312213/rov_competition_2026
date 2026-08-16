#!/usr/bin/env bash
set -euo pipefail

# 只做构建、接口生成和纯软件测试，不连接 MAVLink，不发送实机命令。
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
COLCON_BIN="${PROJECT_DIR}/.venv/bin/colcon"

if [[ ! -x "${PYTHON_BIN}" || ! -x "${COLCON_BIN}" ]]; then
  echo "请先运行 ./scripts/install_ubuntu_22_04.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${PROJECT_DIR}"
"${PYTHON_BIN}" -m pytest -q
find ros2_ws/src/rov_competition -name '*.py' -print0 \
  | xargs -0 "${PYTHON_BIN}" -m py_compile

cd "${PROJECT_DIR}/ros2_ws"
"${COLCON_BIN}" build --symlink-install

set +u
source "${PROJECT_DIR}/ros2_ws/install/setup.bash"
set -u

"${COLCON_BIN}" test --event-handlers console_direct+
"${COLCON_BIN}" test-result --verbose

ros2 interface show rov_interfaces/msg/MissionStatus >/dev/null
ros2 interface show rov_interfaces/srv/SetGripper >/dev/null
ros2 interface show rov_interfaces/msg/NormalizedMotionCommand >/dev/null

EXECUTABLES="$(ros2 pkg executables rov_competition)"
for REQUIRED in rov_vehicle rov_autonomy rov_axis_test rov_turn_test rov_replay; do
  if ! grep -q " ${REQUIRED}$" <<<"${EXECUTABLES}"; then
    echo "缺少 ROS 命令入口: ${REQUIRED}" >&2
    exit 1
  fi
done

echo "ROS 2 Humble 构建、测试、接口和命令入口检查全部通过。"
