"""直接输入方向和推进力的实艇单轴控制台。

每次输入后持续发布单轴指令；操作员按回车立即回中。没有固定持续时间，
但会把实际保持时间、开始/结束遥测和操作员备注写入 CSV。
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import sys
import threading
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args

from .axis_test import _default_config_path, _telemetry_text
from .commissioning import CommissioningError
from .commissioning_runtime import CommissioningNode, PUBLISH_RATE_HZ
from .config import ConfigurationError, ControlProfile, load_robot_config
from .direct_motion import (
    DIRECT_ACTIONS,
    build_direct_motion_plan,
    format_measurement_result,
)


EXECUTION_CONFIRMATION = "DIRECT MOTION"
QUIT_WORDS = {"q", "quit", "exit"}
DIAGNOSTIC_FIELDS = (
    "stop_kind", "stop_reason", "stopped_monotonic_s",
    "telemetry_callback_age_s", "status_callback_age_s", "telemetry_ros_stamp",
    "heartbeat_age_s", "message_age_s", "valid_heartbeat", "valid_attitude",
    "valid_depth", "gateway_state", "gateway_runtime_enabled",
)


@dataclass(frozen=True)
class StopDetails:
    stopped_at: float
    kind: str
    reason: str
    diagnostics: dict[str, str]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="交互式单轴控制：输入方向和推进力，回车停止并记录。"
    )
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument(
        "--record-dir", default="output/direct_motion_records",
        help="CSV 记录目录（默认：output/direct_motion_records）",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser


def _record_path(record_dir: str) -> Path:
    directory = Path(record_dir)
    directory.mkdir(parents=True, exist_ok=True)
    return directory / f"direct_motion_{datetime.now():%Y%m%d_%H%M%S}.csv"


def _append_record(
    path: Path, *, action: str, value: float, held_s: float,
    start_telemetry: str, end_telemetry: str, note: str, stop: StopDetails,
) -> None:
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "recorded_at", "action", "value", "held_seconds", "start_telemetry",
                "end_telemetry", "observed_effect", *DIAGNOSTIC_FIELDS,
            ),
        )
        if new_file:
            writer.writeheader()
        writer.writerow({
            "recorded_at": datetime.now().isoformat(timespec="seconds"),
            "action": action,
            "value": f"{value:.4f}",
            "held_seconds": f"{held_s:.3f}",
            "start_telemetry": start_telemetry,
            "end_telemetry": end_telemetry,
            "observed_effect": note,
            **stop.diagnostics,
        })


def _diagnostics(node: CommissioningNode, now: float) -> dict[str, str]:
    """冻结本地回调时间与最后一条网关遥测，供故障归因使用。"""

    telemetry = node.telemetry
    status = node.status
    def age(received_at: float | None) -> str:
        return "UNKNOWN" if received_at is None else f"{max(0.0, now - received_at):.3f}"

    stamp = "UNKNOWN"
    if telemetry is not None:
        stamp = f"{telemetry.stamp.sec}.{telemetry.stamp.nanosec:09d}"
    return {
        "stop_kind": "operator" ,
        "stop_reason": "operator_enter",
        "stopped_monotonic_s": f"{now:.6f}",
        "telemetry_callback_age_s": age(node.telemetry_received_at),
        "status_callback_age_s": age(node.status_received_at),
        "telemetry_ros_stamp": stamp,
        "heartbeat_age_s": "UNKNOWN" if telemetry is None else f"{telemetry.heartbeat_age_s:.3f}",
        "message_age_s": "UNKNOWN" if telemetry is None else f"{telemetry.message_age_s:.3f}",
        "valid_heartbeat": "UNKNOWN" if telemetry is None else str(bool(telemetry.valid_heartbeat)),
        "valid_attitude": "UNKNOWN" if telemetry is None else str(bool(telemetry.valid_attitude)),
        "valid_depth": "UNKNOWN" if telemetry is None else str(bool(telemetry.valid_depth)),
        "gateway_state": "UNKNOWN" if status is None else status.state,
        "gateway_runtime_enabled": "UNKNOWN" if status is None else str(bool(status.runtime_enabled)),
    }


def _stop_details(node: CommissioningNode, now: float, *, kind: str, reason: str) -> StopDetails:
    diagnostics = _diagnostics(node, now)
    diagnostics["stop_kind"] = kind
    diagnostics["stop_reason"] = reason
    return StopDetails(now, kind, reason, diagnostics)


def _drive_until_stopped(
    node: CommissioningNode, motion, stop_event: threading.Event, stop_box: list[StopDetails],
) -> None:
    period_s = 1.0 / PUBLISH_RATE_HZ
    while rclpy.ok() and not stop_event.is_set():
        node.spin(0.0)
        error = node.runtime_error()
        if error is not None:
            stop_box.append(_stop_details(node, time.monotonic(), kind="runtime_protection", reason=error))
            stop_event.set()
            break
        node.publish(motion)
        time.sleep(period_s)


def _read_action() -> str | None:
    prompt = f"\n动作 [{'/'.join(DIRECT_ACTIONS)}，q 退出]: "
    action = input(prompt).strip().lower()
    return None if action in QUIT_WORDS else action


def _read_value(limit: float) -> float | None:
    raw = input(f"推进力 (0 到 {limit:.3f}]：").strip()
    try:
        return float(raw)
    except ValueError:
        print("推进力必须是数字。")
        return None


def main(argv: list[str] | None = None) -> int:
    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    if not args.execute or args.confirm != EXECUTION_CONFIRMATION:
        print("此工具只在显式实艇模式运行：加 --execute --confirm 'DIRECT MOTION'。")
        return 2
    try:
        config = load_robot_config(args.config)
    except ConfigurationError as exc:
        print(f"配置错误: {exc}")
        return 2
    if config.control_profile != ControlProfile.COMMISSIONING:
        print("robot.yaml 的 control.profile 必须是 commissioning")
        return 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    node = CommissioningNode("rov_direct_motion", config.allowed_command_source)
    record = _record_path(args.record_dir)
    session_results: list[str] = []
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
        print("已连接。输入方向和推进力；每次运动后按回车立即停车。")
        print(f"记录文件：{record}")
        while rclpy.ok():
            action = _read_action()
            if action is None:
                break
            value = _read_value(config.command_limit)
            if value is None:
                continue
            try:
                plan = build_direct_motion_plan(
                    action, value, command_limit=config.command_limit,
                )
            except CommissioningError as exc:
                print(f"输入无效: {exc}")
                continue
            start_telemetry = _telemetry_text(node)
            stop_event = threading.Event()
            stop_box: list[StopDetails] = []
            started_at = time.monotonic()
            worker = threading.Thread(
                target=_drive_until_stopped,
                args=(node, plan.motion, stop_event, stop_box), daemon=True,
            )
            worker.start()
            input(f"正在持续 {plan.action}，推进力 {plan.value:.3f}；按回车立即停止。")
            operator_stop_at = time.monotonic()
            stop_event.set()
            worker.join(timeout=1.0)
            node.publish_neutral()
            stop = stop_box[0] if stop_box else _stop_details(
                node, operator_stop_at, kind="operator", reason="operator_enter",
            )
            held_s = max(0.0, stop.stopped_at - started_at)
            end_telemetry = _telemetry_text(node)
            if stop.kind == "runtime_protection":
                print(f"已因运行时保护停止: {stop.reason}")
                print("停止诊断：" + ", ".join(
                    f"{key}={value}" for key, value in stop.diagnostics.items()
                ))
            note = input("观察到的效果（可留空）：").strip()
            _append_record(
                record, action=plan.action, value=plan.value, held_s=held_s,
                start_telemetry=start_telemetry, end_telemetry=end_telemetry, note=note, stop=stop,
            )
            print(f"已回中并记录：保持 {held_s:.2f}s。")
            copyable = format_measurement_result(
                action=plan.action,
                value=plan.value,
                held_s=held_s,
                observed_effect=note,
                stop_reason=(stop.reason if stop.kind == "runtime_protection" else None),
            )
            session_results.append(copyable)
            print("请复制以下内容发给我：")
            print(copyable)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，立即回中。")
        result = 130
    finally:
        if rclpy.ok():
            node.publish_neutral()
            disarm_error = node.request_normal_disarm()
            if disarm_error is not None:
                print(f"上锁未确认: {disarm_error}；立即用 QGC 上锁/物理断电。")
                result = 2
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if session_results:
        print("\n=== 本次全部结果（可整体复制） ===")
        print("\n\n".join(session_results))
    return result


if __name__ == "__main__":
    raise SystemExit(main())
