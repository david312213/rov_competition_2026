#!/usr/bin/env bash
set -euo pipefail

# 不允许 ~/.local/lib/python* 里的旧 Torch/OpenCV 污染工程环境。
# 团队已经实际遇到过“激活 .venv 却加载用户级 CPU Torch”。
export PYTHONNOUSERSITE=1

# 仅支持 Ubuntu 22.04 + ROS 2 Humble。脚本不会配置飞控通道，也不会开启推进器。
if [[ ! -r /etc/os-release ]]; then
  echo "无法读取 /etc/os-release；请在 Ubuntu 22.04 上运行。" >&2
  exit 1
fi

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "当前系统不是 Ubuntu 22.04，停止安装。" >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "未检测到 ROS 2 Humble；请先按 ROS 官方文档安装。" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"
PYTHON_BIN="${VENV_DIR}/bin/python"
COLCON_BIN="${VENV_DIR}/bin/colcon"

sudo apt update
sudo apt install -y \
  python3-rosdep \
  python3-venv \
  python3-tk \
  python3-opencv \
  python3-gi \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  ffmpeg \
  gstreamer1.0-tools \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  nlohmann-json3-dev

if [[ ! -d "${VENV_DIR}" ]]; then
  python3 -m venv --system-site-packages "${VENV_DIR}"
fi
if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "${VENV_DIR} 不是完整的 Linux Python 虚拟环境。" >&2
  echo "请将该目录改名备份后重新运行安装脚本。" >&2
  exit 1
fi

"${PYTHON_BIN}" -m pip install --upgrade pip
"${PYTHON_BIN}" -m pip install -r "${PROJECT_DIR}/requirements.txt"

# 开启 --system-site-packages 后，pip 可能认为系统的 colcon 已满足依赖，
# 却不会在 .venv/bin 中生成 colcon 入口。为确保 ROS Python 节点
# 使用当前虚拟环境构建，缺失时只强制补装 colcon-core 入口；
# 其他扩展和 ROS 兼容依赖继续使用 requirements 和 Ubuntu 系统包的版本。
if [[ ! -x "${COLCON_BIN}" ]]; then
  echo "虚拟环境中缺少 colcon，正在补充安装……"
  "${PYTHON_BIN}" -m pip install --ignore-installed --no-deps \
    "colcon-core>=0.15,<1"
fi
if [[ ! -x "${COLCON_BIN}" ]]; then
  echo "colcon 安装后仍不存在：${COLCON_BIN}" >&2
  exit 1
fi

# ROS 2 Humble 的环境脚本不保证兼容 Bash 的 nounset（set -u）模式。
# 只在加载 ROS 环境时临时关闭 nounset，其余安装步骤仍保持严格检查。
set +u
source /opt/ros/humble/setup.bash
set -u

# rosdep 首次使用前必须初始化并下载依赖索引。重复执行时不重复 init。
if [[ ! -r /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

cd "${PROJECT_DIR}/ros2_ws"
rosdep install --from-paths src --ignore-src -r -y
"${COLCON_BIN}" build --symlink-install

echo "安装完成。真实执行仍为关闭状态，请阅读 README.md。"
