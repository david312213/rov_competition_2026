#!/usr/bin/env bash
set -euo pipefail

# 在 Ubuntu 应用菜单中安装一个无需终端的本地快捷方式。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
LAUNCHER="${PROJECT_DIR}/scripts/start_frame_extractor.sh"
APPLICATION_DIR="${HOME}/.local/share/applications"
DESKTOP_FILE="${APPLICATION_DIR}/rov-frame-extractor.desktop"

if [[ ! -x "${LAUNCHER}" ]]; then
  echo "抽帧启动脚本不存在或没有执行权限：${LAUNCHER}" >&2
  exit 1
fi
if [[ ! -x "${PROJECT_DIR}/.venv/bin/python" ]]; then
  echo "请先在工程目录运行 ./scripts/install.sh。" >&2
  exit 1
fi

mkdir -p "${APPLICATION_DIR}"
TEMP_FILE="$(mktemp)"
trap 'rm -f "${TEMP_FILE}"' EXIT

printf '%s\n' \
  '[Desktop Entry]' \
  'Type=Application' \
  'Version=1.0' \
  'Name=ROV 视频抽帧' \
  'Comment=拖入视频并导出全部帧图片' \
  "Exec=\"${LAUNCHER}\"" \
  "TryExec=${LAUNCHER}" \
  'Icon=video-x-generic' \
  'Terminal=false' \
  'Categories=Graphics;Utility;' \
  'StartupNotify=true' \
  >"${TEMP_FILE}"

install -m 0644 "${TEMP_FILE}" "${DESKTOP_FILE}"
if command -v update-desktop-database >/dev/null 2>&1; then
  update-desktop-database "${APPLICATION_DIR}" >/dev/null 2>&1 || true
fi

echo "应用菜单快捷方式已安装：${DESKTOP_FILE}"
echo "现在可在 Ubuntu 应用列表中搜索“ROV 视频抽帧”。"
