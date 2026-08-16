"""使用真实航向反馈的单次左/右转角实机测试。"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .axis_test import _default_config_path, _telemetry_text
from .commissioning import (
    ACCEPTANCE_TURN_SEQUENCE_DEG,
    TURN_DIRECTIONS,
    CommissioningError,
    TurnProgressTracker,
    build_turn_test_plan,
)
from .commissioning_runtime import CommissioningNode, PUBLISH_RATE_HZ
from .config import ConfigurationError, ControlProfile, load_robot_config

EXECUTION_CONFIRMATION = "TURN ROV"


def build_parser() -> argparse.ArgumentParser:
    """创建 1°..360° 单次转向参数解析器。"""

    parser = argparse.ArgumentParser(
        description="真实航向闭环转向；默认只预览，不自动解锁。"
    )
    parser.add_argument("--direction", required=True, choices=TURN_DIRECTIONS)
    parser.add_argument("--angle", required=True, type=float)
    parser.add_argument("--value", type=float, default=0.05)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def main(argv: list[str] | None = None) -> int:
    """预览或执行一次真实航向闭环转向。"""

    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        config = load_robot_config(args.config)
        plan = build_turn_test_plan(
            args.direction,
            args.angle,
            args.value,
            command_limit=config.command_limit,
            execute=args.execute,
        )
        if not math.isfinite(args.timeout) or not 1.0 <= args.timeout <= 300.0:
            raise CommissioningError("--timeout 必须在 1..300 s")
    except (ConfigurationError, CommissioningError) as exc:
        print(f"参数/配置错误: {exc}")
        return 2
    summary = (
        f"方向={plan.direction}, 目标={plan.angle_deg:.1f}°, "
        f"归一化偏航={plan.command:.3f}, 超时={args.timeout:.1f}s"
    )
    print("固定验收顺序：" + " -> ".join(f"{item}°" for item in ACCEPTANCE_TURN_SEQUENCE_DEG))
    if int(plan.angle_deg) not in ACCEPTANCE_TURN_SEQUENCE_DEG:
        print("提醒：当前角度可用于诊断，但不替代 30° -> 90° -> 180° -> 360° 验收顺序。")
    if not plan.execute:
        print("预览：" + summary)
        print("未带 --execute：未初始化 ROS、不会产生动作。")
        return 0
    if args.confirm != EXECUTION_CONFIRMATION:
        print(f"真实执行必须加 --confirm {EXECUTION_CONFIRMATION!r}")
        return 2
    if config.control_profile != ControlProfile.COMMISSIONING:
        print("robot.yaml 的 control.profile 必须是 commissioning")
        return 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    node = CommissioningNode("rov_turn_test", config.allowed_command_source)
    tracker = TurnProgressTracker(plan)
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
        deadline = time.monotonic() + args.timeout
        period_s = 1.0 / PUBLISH_RATE_HZ
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.0)
            error = node.runtime_error()
            if error is not None:
                raise CommissioningError(error)
            assert node.telemetry is not None
            progress = tracker.update(float(node.telemetry.yaw_deg), time.monotonic())
            if tracker.complete():
                print(f"航向闭环进度 {progress:.1f}°，已进入目标容差。")
                break
            node.publish(plan.motion)
            time.sleep(period_s)
        else:
            raise CommissioningError("转向测试总超时")
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
        print("转向测试完成。")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
