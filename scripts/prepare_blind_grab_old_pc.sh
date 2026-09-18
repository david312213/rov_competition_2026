#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
LOCAL_CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"

cd "${PROJECT_DIR}"
chmod +x scripts/install.sh scripts/check.sh scripts/check_ros.sh \
  scripts/check_official_ros.sh scripts/start_blind_grab.sh \
  scripts/set_official_team.sh scripts/start_blind_grab_team1.sh \
  scripts/start_blind_grab_team2.sh scripts/start_official_data_only.sh \
  scripts/start_official_data_only_team1.sh scripts/start_official_data_only_team2.sh \
  scripts/test_arm_gripper.sh scripts/tune_arm_gripper_pwm.sh \
  scripts/apply_corrected_pwm_direction.sh scripts/apply_blind_grab_power_10.sh \
  scripts/prepare_blind_grab_old_pc.sh
mkdir -p config
if [[ ! -r "${LOCAL_CONFIG}" ]]; then
  cp "${TEMPLATE}" "${LOCAL_CONFIG}"
  echo "已生成本机配置：${LOCAL_CONFIG}"
else
  echo "保留现有本机配置：${LOCAL_CONFIG}"
  if grep -Eq '^[[:space:]]*server_port:[[:space:]]*40184([[:space:]]*(#.*)?)?$' "${LOCAL_CONFIG}"; then
    sed -i -E 's/^([[:space:]]*server_port:[[:space:]]*)40184([[:space:]]*(#.*)?)$/\140197\2/' "${LOCAL_CONFIG}"
    echo "已将旧官方端口40184更新为队伍ddhyzx2端口40197。"
  fi
  echo "旧配置缺少 vertical/route/official_ros 段时，程序会自动采用当前永久盲抓默认值。"
fi

./scripts/install.sh
./scripts/apply_blind_grab_power_10.sh
./scripts/check_official_ros.sh --offline

cat <<EOF
旧电脑准备完成。
按队伍选择正式启动命令：
  一队（40198）：cd "${PROJECT_DIR}" && ./scripts/start_blind_grab_team1.sh
  二队（ddhyzx2，40197）：cd "${PROJECT_DIR}" && ./scripts/start_blind_grab_team2.sh

QGC/手柄人工驾驶、只上传主办方数据：
  一队（40198）：cd "${PROJECT_DIR}" && ./scripts/start_official_data_only_team1.sh
  二队（ddhyzx2，40197）：cd "${PROJECT_DIR}" && ./scripts/start_official_data_only_team2.sh

启动后另开终端检查官方计分数据：
  盲抓：cd "${PROJECT_DIR}" && ./scripts/check_official_ros.sh --live
  手柄：cd "${PROJECT_DIR}" && ./scripts/check_official_ros.sh --live-manual
EOF
