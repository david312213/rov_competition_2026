"""让测试直接导入尚未安装的 ROS 2 Python 包。"""

from __future__ import annotations

import sys
from pathlib import Path

PACKAGE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "rov_competition"
)
sys.path.insert(0, str(PACKAGE_ROOT))

