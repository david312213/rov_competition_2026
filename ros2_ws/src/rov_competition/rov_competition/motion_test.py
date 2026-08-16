"""单轴、低功率、限时的 ROS 2 实艇点动发布器。"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rov_interfaces.msg import NormalizedMotionCommand

from .config import ConfigurationError, ControlProfile, load_robot_config

PUBLISH_RATE_HZ = 20.0
NEUTRAL_REPEAT_COUNT = 10
MAXIMUM_TEST_DURATION_S = 5.0
DISCOVERY_TIMEOUT_S = 3.0
AXES = ("forward", "lateral", "vertical", "yaw")


def _default_config_path() -> str:
    """优先使用 ROS 安装后的 share 目录，源码直接运行时回退到包目录。"""

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return str(
            Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
        )
    try:
        share_directory = Path(get_package_share_directory("rov_competition"))
    except PackageNotFoundError:
        share_directory = Path(__file__).resolve().parents[1]
    return str(share_directory / "config" / "robot.example.yaml")


def _build_parser() -> argparse.ArgumentParser:
    """创建点动工具参数解析器。"""

    parser = argparse.ArgumentParser(
        description="以 20 Hz 发布单轴运动意图；默认只预览，--execute 才发布。"
    )
    parser.add_argument("--axis", required=True, choices=AXES)
    parser.add_argument("--value", required=True, type=float)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument(
        "--source",
        default=None,
        help="命令来源；默认使用 YAML control.profile",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="真正向 /rov/control/command 发布；不带时仅打印",
    )
    return parser


def _make_message(
    node: Node,
    *,
    source: str,
    axis: str | None = None,
    value: float = 0.0,
) -> NormalizedMotionCommand:
    """构造带当前 ROS 时间戳的四轴命令，未选中轴保持中位。"""

    message = NormalizedMotionCommand()
    message.stamp = node.get_clock().now().to_msg()
    message.source = source
    if axis is not None:
        setattr(message, axis, float(value))
    return message


def _publish_neutral(node: Node, publisher, source: str) -> None:
    """在正常结束或 ``Ctrl+C`` 后连续发布中位，覆盖队列中的点动命令。"""

    period_s = 1.0 / PUBLISH_RATE_HZ
    for _ in range(NEUTRAL_REPEAT_COUNT):
        publisher.publish(_make_message(node, source=source))
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(period_s)


def _wait_for_gateway_subscription(node: Node, publisher) -> bool:
    """在计时开始前等待 DDS 发现网关，避免 0.3s 点动全部丢在发现阶段。"""

    deadline = time.monotonic() + DISCOVERY_TIMEOUT_S
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() > 0:
            return True
    return False


def main(argv: list[str] | None = None) -> int:
    """验证参数，预览或执行一次限时单轴点动。"""

    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    arguments = _build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        config = load_robot_config(arguments.config)
    except ConfigurationError as exc:
        print(f"配置错误: {exc}")
        return 2

    if not math.isfinite(arguments.value):
        print("--value 必须是有限数")
        return 2
    if abs(arguments.value) > config.command_limit:
        print(f"--value={arguments.value} 超过 YAML 软件上限 {config.command_limit}")
        return 2
    if not math.isfinite(arguments.duration) or not (
        0.0 < arguments.duration <= MAXIMUM_TEST_DURATION_S
    ):
        print(f"--duration 必须在 (0, {MAXIMUM_TEST_DURATION_S}] 秒")
        return 2
    source = arguments.source or config.allowed_command_source
    if arguments.execute and config.control_profile != ControlProfile.COMMISSIONING:
        print("兼容点动只允许 control.profile: commissioning")
        return 2

    summary = (
        f"轴={arguments.axis}, 值={arguments.value:+.3f}, "
        f"时长={arguments.duration:.3f}s, 频率={PUBLISH_RATE_HZ:.0f}Hz, "
        f"来源={source!r}"
    )
    if not arguments.execute:
        print("预览：" + summary)
        print("未带 --execute，未创建发布器，不会产生动作。")
        return 0

    # 保留 Python 默认的 SIGINT 行为，让 Ctrl+C 先进入下面的 finally，
    # 在 ROS context 仍有效时连续发布中位，再主动 shutdown。若交给
    # rclpy 默认信号处理器，context 可能先失效，收尾中位便无法发出。
    rclpy.init(
        args=raw_args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Node("rov_motion_test")
    publisher = node.create_publisher(
        NormalizedMotionCommand, "/rov/control/command", 10
    )
    if not _wait_for_gateway_subscription(node, publisher):
        print(
            f"{DISCOVERY_TIMEOUT_S:.1f}s 内未发现 /rov/control/command 订阅者；"
            "未发布点动，请检查 rov_vehicle 和 ROS_DOMAIN_ID。"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    period_s = 1.0 / PUBLISH_RATE_HZ
    deadline = time.monotonic() + arguments.duration
    print("正在执行：" + summary)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            publisher.publish(
                _make_message(
                    node,
                    source=source,
                    axis=arguments.axis,
                    value=arguments.value,
                )
            )
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在连续发布中位。")
    finally:
        if rclpy.ok():
            _publish_neutral(node, publisher, source)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print("点动结束，已连续发布中位。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
