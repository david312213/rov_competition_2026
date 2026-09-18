#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
TEMPLATE="${PROJECT_DIR}/ros2_ws/src/rov_competition/config/blind_grab.yaml"
CONFIG="${PROJECT_DIR}/config/blind_grab.local.yaml"

if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != "--config" ]]; then
    echo "用法：./scripts/apply_blind_grab_power_08.sh [--config YAML路径]" >&2
    exit 2
  fi
  CONFIG="$2"
fi

mkdir -p "$(dirname -- "${CONFIG}")"
if [[ ! -r "${CONFIG}" ]]; then
  if [[ $# -eq 0 ]]; then
    cp "${TEMPLATE}" "${CONFIG}"
  else
    echo "配置不存在：${CONFIG}" >&2
    exit 1
  fi
fi

python3 - "${CONFIG}" <<'PY'
from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

path = Path(sys.argv[1]).expanduser().resolve()
original = path.read_text(encoding="utf-8")
lines = original.splitlines()
sections = {
    "vertical": [
        ("initial_descent_command", "-0.80"),
        ("ascent_command", "0.80"),
        ("repeat_descent_command", "-0.80"),
    ],
    "route": [
        ("forward_command", "0.80"),
        ("shift_command", "0.80"),
        ("turn_command", "0.80"),
    ],
    "grab": [("forward_command", "0.80")],
}

for section_name, fields in sections.items():
    try:
        start = next(
            index for index, line in enumerate(lines)
            if re.match(rf"^{re.escape(section_name)}\s*:\s*(?:#.*)?$", line)
        )
    except StopIteration:
        if lines and lines[-1].strip():
            lines.append("")
        lines.append(f"{section_name}:")
        lines.extend(f"  {name}: {value}" for name, value in fields)
        continue

    end = len(lines)
    for index in range(start + 1, len(lines)):
        line = lines[index]
        if line.strip() and not line.lstrip().startswith("#") and not line[0].isspace():
            end = index
            break

    insert_at = start + 1
    for name, value in fields:
        pattern = re.compile(
            rf"^(\s+{re.escape(name)}\s*:\s*)([^#]*?)(\s*(?:#.*)?)$"
        )
        for index in range(start + 1, end):
            match = pattern.match(lines[index])
            if match:
                lines[index] = f"{match.group(1)}{value}{match.group(3)}"
                break
        else:
            lines.insert(insert_at, f"  {name}: {value}")
            insert_at += 1
            end += 1

updated = "\n".join(lines) + "\n"
if updated == original:
    print(f"盲抓所有运动功率已是0.8：{path}")
    raise SystemExit(0)

stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
backup = path.with_name(f"{path.name}.before-power-08-{stamp}")
shutil.copy2(path, backup)
fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
try:
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        stream.write(updated)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary_name, path)
finally:
    if os.path.exists(temporary_name):
        os.unlink(temporary_name)

print(f"已备份：{backup}")
print(
    "已设置盲抓全部运动功率：下潜=-0.8，上潜=+0.8，"
    "前进/抓取前进/横移/转向=0.8"
)
PY
