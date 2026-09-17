#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"

if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != "--config" ]]; then
    echo "用法：./scripts/apply_corrected_pwm_direction.sh [--config YAML路径]" >&2
    exit 2
  fi
  CONFIG="$2"
fi

mkdir -p "$(dirname -- "${CONFIG}")"
if [[ ! -r "${CONFIG}" ]]; then
  if [[ $# -eq 0 ]]; then
    cp "${TEMPLATE}" "${CONFIG}"
    echo "已从修正后的模板生成：${CONFIG}"
    exit 0
  fi
  echo "配置不存在：${CONFIG}" >&2
  exit 1
fi

PYTHON_BIN="${BLIND_GRAB_PYTHON:-${PROJECT_DIR}/.venv/bin/python}"
if [[ ! -x "${PYTHON_BIN}" ]] || ! "${PYTHON_BIN}" -c 'import yaml' >/dev/null 2>&1; then
  PYTHON_BIN=python3
fi
if ! "${PYTHON_BIN}" -c 'import yaml' >/dev/null 2>&1; then
  echo "当前Python缺少PyYAML，请先运行 ./scripts/prepare_blind_grab_old_pc.sh" >&2
  exit 1
fi

"${PYTHON_BIN}" - "${CONFIG}" <<'PY'
from datetime import datetime
import os
from pathlib import Path
import shutil
import sys
import tempfile

import yaml

path = Path(sys.argv[1]).expanduser().resolve()
document = yaml.safe_load(path.read_text(encoding="utf-8"))
if not isinstance(document, dict) or not isinstance(document.get("actions"), dict):
    raise SystemExit("配置缺少 actions 段，未修改")

corrected = {
    "open_gripper": 1800,
    "close_gripper": 900,
    "arm_to_basket": 1800,
    "arm_to_grasp": 900,
}
for action_name, pwm in corrected.items():
    action = document["actions"].get(action_name)
    outputs = action.get("outputs") if isinstance(action, dict) else None
    if not isinstance(outputs, list) or not outputs:
        raise SystemExit(f"配置缺少 actions.{action_name}.outputs，未修改")
    for output in outputs:
        if not isinstance(output, dict) or "output_channel" not in output:
            raise SystemExit(f"actions.{action_name}.outputs 格式错误，未修改")
        output["pwm"] = pwm

stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
backup = path.with_name(f"{path.name}.before-pwm-direction-fix-{stamp}")
shutil.copy2(path, backup)
text = yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_name, path)
finally:
    if os.path.exists(temporary_name):
        os.unlink(temporary_name)

print(f"已备份：{backup}")
print(f"已修正：{path}")
print("  张爪 S11=1800，闭爪 S11=900")
print("  机械臂后仰 S10=1800，机械臂恢复 S10=900")
PY
