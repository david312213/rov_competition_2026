#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
LOCAL_CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"

cd "${PROJECT_DIR}"
chmod +x scripts/install.sh scripts/check.sh scripts/check_ros.sh \
  scripts/check_official_ros.sh scripts/start_blind_grab.sh \
  scripts/test_arm_gripper.sh scripts/tune_arm_gripper_pwm.sh \
  scripts/apply_corrected_pwm_direction.sh scripts/prepare_blind_grab_old_pc.sh
mkdir -p config
if [[ ! -r "${LOCAL_CONFIG}" ]]; then
  cp "${TEMPLATE}" "${LOCAL_CONFIG}"
  echo "已生成本机配置：${LOCAL_CONFIG}"
else
  echo "保留现有本机配置：${LOCAL_CONFIG}"
  echo "旧配置没有 official_ros 段时，程序会使用官方包默认地址 api.bjetone.com:40184。"
fi

./scripts/install.sh
./scripts/check_official_ros.sh --offline

cat <<EOF
旧电脑准备完成。
正式启动只运行：
  cd "${PROJECT_DIR}" && ./scripts/start_blind_grab.sh

启动后另开终端检查官方计分数据：
  cd "${PROJECT_DIR}" && ./scripts/check_official_ros.sh --live
EOF
