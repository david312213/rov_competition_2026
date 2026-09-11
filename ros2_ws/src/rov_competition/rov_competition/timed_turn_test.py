"""固定时长偏航点动：用于标定人工纠偏和换带翻转，不按罗盘闭环。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .axis_test import _default_config_path, _telemetry_text
from .commissioning import CommissioningError
from .commissioning_runtime import CommissioningNode, PUBLISH_RATE_HZ
from .config import ConfigurationError, ControlProfile, load_robot_config
from .timed_turn import build_timed_turn_plan


EXECUTION_CONFIRMATION = "PULSE TURN"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="固定时长左/右偏航点动；不按罗盘闭环、不自动解锁。"
    )
    parser.add_argument("--direction", required=True, choices=("left", "right"))
    parser.add_argument("--value", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=0.3)
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        config = load_robot_config(args.config)
        plan = build_timed_turn_plan(
            args.direction, args.value, args.duration,
            command_limit=config.command_limit, execute=args.execute,
        )
    except (ConfigurationError, CommissioningError) as exc:
        print(f"参数/配置错误: {exc}")
        return 2
    summary = f"方向={plan.direction}, 偏航={plan.value:.3f}, 时长={plan.duration_s:.3f}s"
    if not plan.execute:
        print("预览：" + summary)
        print("未带 --execute：不会初始化 ROS 或产生动作。")
        return 0
    if args.confirm != EXECUTION_CONFIRMATION:
        print(f"真实执行必须加 --confirm {EXECUTION_CONFIRMATION!r}")
        return 2
    if config.control_profile != ControlProfile.COMMISSIONING:
        print("robot.yaml 的 control.profile 必须是 commissioning")
        return 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    node = CommissioningNode("rov_timed_turn_test", config.allowed_command_source)
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
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            error = node.runtime_error()
            if error is not None:
                raise CommissioningError(error)
            node.publish(plan.motion)
            time.sleep(1.0 / PUBLISH_RATE_HZ)
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
    return result


if __name__ == "__main__":
    raise SystemExit(main())
