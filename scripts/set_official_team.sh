#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${BLIND_GRAB_TEMPLATE_PATH:-${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml}"
LOCAL_CONFIG="${BLIND_GRAB_CONFIG_PATH:-${PROJECT_DIR}/config/blind_grab.local.yaml}"

case "${1:-}" in
  1|team1|TEAM1|一队)
    TEAM_LABEL="一队"
    SERVER_PORT=40198
    ;;
  2|team2|TEAM2|二队|ddhyzx2)
    TEAM_LABEL="二队（ddhyzx2）"
    SERVER_PORT=40197
    ;;
  *)
    echo "用法：./scripts/set_official_team.sh 1|2" >&2
    echo "  一队 -> api.bjetone.com:40198" >&2
    echo "  二队（ddhyzx2） -> api.bjetone.com:40197" >&2
    exit 2
    ;;
esac

mkdir -p "$(dirname -- "${LOCAL_CONFIG}")"
if [[ ! -r "${LOCAL_CONFIG}" ]]; then
  cp "${TEMPLATE}" "${LOCAL_CONFIG}"
fi

python3 - "${LOCAL_CONFIG}" "${SERVER_PORT}" <<'PYCODE'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
port = int(sys.argv[2])
lines = path.read_text(encoding="utf-8").splitlines()
try:
    start = next(i for i, line in enumerate(lines) if re.match(r"^official_ros\s*:\s*(?:#.*)?$", line))
except StopIteration:
    if lines and lines[-1].strip():
        lines.append("")
    lines.extend([
        "official_ros:",
        "  enabled: true",
        '  server_ip: "api.bjetone.com"',
        f"  server_port: {port}",
    ])
else:
    end = len(lines)
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.strip() and not line.lstrip().startswith("#") and not line[0].isspace():
            end = i
            break

    def replace_field(name: str, value: str) -> bool:
        pattern = re.compile(rf"^(\s*{re.escape(name)}\s*:\s*)([^#]*?)(\s*(?:#.*)?)$")
        for i in range(start + 1, end):
            match = pattern.match(lines[i])
            if match:
                lines[i] = f"{match.group(1)}{value}{match.group(3)}"
                return True
        return False

    if not replace_field("server_ip", '"api.bjetone.com"'):
        lines.insert(start + 1, '  server_ip: "api.bjetone.com"')
        end += 1
    if not replace_field("server_port", str(port)):
        lines.insert(start + 2, f"  server_port: {port}")

path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PYCODE

echo "已选择${TEAM_LABEL}："
echo "  ROS数据：api.bjetone.com:${SERVER_PORT}"
echo "  ROS视频：rtmp://api.bjetone.com/ros/${SERVER_PORT}"
echo "  本机配置：${LOCAL_CONFIG}"
