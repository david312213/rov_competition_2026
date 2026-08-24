#!/usr/bin/env bash
set -euo pipefail

# 验证的必须是本工程 .venv + 系统 ROS，不是 ~/.local 里的旧包。
export PYTHONNOUSERSITE=1

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
  echo "请先运行 ./scripts/install.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

cd "${PROJECT_DIR}"
bash "${PROJECT_DIR}/scripts/check.sh"

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
for REQUIRED in rov_vehicle rov_autonomy rov_axis_test rov_dataset_drive rov_dataset_record rov_field_setup rov_frame_extractor rov_gripper_test rov_turn_test rov_replay rov_stream_bridge rov_search_approach_test; do
  if ! grep -q " ${REQUIRED}$" <<<"${EXECUTABLES}"; then
    echo "缺少 ROS 命令入口: ${REQUIRED}" >&2
    exit 1
  fi
done

if [[ ! -x "${PROJECT_DIR}/scripts/start_dataset_collection.sh" ]]; then
  echo "缺少一键数据集采集脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_dataset_recording.sh" ]]; then
  echo "缺少手柄只录像脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_frame_extractor.sh" ]]; then
  echo "缺少桌面抽帧启动脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_search_approach_test.sh" ]]; then
  echo "缺少搜索—接近一键水池测试脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_grasp_position_test.sh" ]]; then
  echo "缺少抓取位置标定一键脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_gripper_test.sh" ]]; then
  echo "缺少机械爪候选测试一键脚本" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/scripts/start_tomorrow_test.sh" ]]; then
  echo "缺少明日联调总向导脚本" >&2
  exit 1
fi

VIDEO_LAUNCH="$(ros2 pkg prefix --share rov_competition)/launch/video_test.launch.py"
if [[ ! -r "${VIDEO_LAUNCH}" ]]; then
  echo "缺少一键视频 launch 文件: ${VIDEO_LAUNCH}" >&2
  exit 1
fi
PACKAGE_SHARE="$(ros2 pkg prefix --share rov_competition)"
for REQUIRED_FILE in \
  "${PACKAGE_SHARE}/launch/perception_only.launch.py" \
  "${PACKAGE_SHARE}/config/search_test.yaml"; do
  if [[ ! -r "${REQUIRED_FILE}" ]]; then
    echo "缺少已安装的搜索测试文件: ${REQUIRED_FILE}" >&2
    exit 1
  fi
done

echo "ROS 2 Humble 构建、测试、接口和命令入口检查全部通过。"
