"""每次只允许一个方向的低功率实机点动工具。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .commissioning import AXIS_ACTIONS, CommissioningError, build_axis_test_plan
from .commissioning_runtime import CommissioningNode, PUBLISH_RATE_HZ
from .config import ConfigurationError, ControlProfile, load_robot_config

EXECUTION_CONFIRMATION = "MOVE ROV"


def _default_config_path() -> str:
    """优先使用 ROS 安装后的模板路径。"""

    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return str(Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml")
    try:
        root = Path(get_package_share_directory("rov_competition"))
    except PackageNotFoundError:
        root = Path(__file__).resolve().parents[1]
    return str(root / "config" / "robot.example.yaml")


def build_parser() -> argparse.ArgumentParser:
    """创建无自动动作序列的单轴参数解析器。"""

    parser = argparse.ArgumentParser(
        description="单次前/后/左/右/上/下点动；默认 0.05、0.3s 且只预览。"
    )
    parser.add_argument("--motion", required=True, choices=AXIS_ACTIONS)
    parser.add_argument("--value", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=0.3)
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def _telemetry_text(node: CommissioningNode) -> str:
    """把可用的深度和航向转成不夸大精度的日志。"""

    telemetry = node.telemetry
    if telemetry is None:
        return "depth=UNKNOWN, yaw=UNKNOWN"
    depth = f"{telemetry.depth_m:.3f}m" if telemetry.valid_depth else "INVALID"
    yaw = f"{telemetry.yaw_deg:.2f}deg" if telemetry.valid_attitude else "INVALID"
    return f"depth={depth}, yaw={yaw}"


def main(argv: list[str] | None = None) -> int:
    """预览或执行一次单轴点动；不自动解锁。"""

    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        config = load_robot_config(args.config)
        plan = build_axis_test_plan(
            args.motion,
            args.value,
            args.duration,
            command_limit=config.command_limit,
            execute=args.execute,
        )
    except (ConfigurationError, CommissioningError) as exc:
        print(f"参数/配置错误: {exc}")
        return 2
    summary = (
        f"动作={plan.action}, 归一化指令={plan.value:.3f}, "
        f"时长={plan.duration_s:.3f}s, 频率={PUBLISH_RATE_HZ:.0f}Hz"
    )
    if not plan.execute:
        print("预览：" + summary)
        print("未带 --execute：未初始化 ROS、未创建发布器、不会产生动作。")
        return 0
    if args.confirm != EXECUTION_CONFIRMATION:
        print(f"真实执行必须加 --confirm {EXECUTION_CONFIRMATION!r}")
        return 2
    if config.control_profile != ControlProfile.COMMISSIONING:
        print("robot.yaml 的 control.profile 必须是 commissioning")
        return 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    node = CommissioningNode("rov_axis_test", config.allowed_command_source)
    result = 0
    try:
        error = node.wait_until_ready()
        if error is not None:
            print(f"拒绝执行: {error}")
            return 2
        assert node.status is not None
        if node.status.flight_mode.upper() not in config.allowed_flight_modes:
            print(f"拒绝执行: 飞控模式必须是 {config.allowed_flight_modes}")
            return 2
        print("开始记录: " + _telemetry_text(node))
        print("正在执行：" + summary)
        deadline = time.monotonic() + plan.duration_s
        period_s = 1.0 / PUBLISH_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            error = node.runtime_error()
            if error is not None:
                raise CommissioningError(error)
            node.publish(plan.motion)
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，立即回中。")
        result = 130
    except CommissioningError as exc:
        print(f"测试中止: {exc}")
        result = 2
    finally:
        if rclpy.ok():
            node.publish_neutral()
            print("结束记录: " + _telemetry_text(node))
            disarm_error = node.request_normal_disarm()
            if disarm_error is not None:
                print(f"上锁未确认: {disarm_error}；立即用 QGC 上锁/物理断电。")
                result = 2
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result == 0:
        print("单轴点动完成；只记录遥测，没有把位移冒充为传感器测量。")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
