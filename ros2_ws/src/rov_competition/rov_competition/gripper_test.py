"""机械爪候选档案的上锁台架测试。

该工具直接使用独立 MAVLink 端点，不启动 ROS 飞控网关。它只在
飞控明确上锁、只读预检通过、三道真实输出配置门全部关闭时，
向机械爪候选输出发送 ``MAV_CMD_DO_SET_SERVO``。推进器必须在现场
物理断开；软件无法代替这个条件。
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import yaml

from .config import ConfigurationError, RobotConfig, load_robot_config
from .domain import GripperAction
from .vehicle import (
    EXTENDED_PWM_CONFIRMATION,
    GRIPPER_TEST_CONFIRMATION,
    MavlinkVehicle,
    VehicleError,
)


class GripperTestError(RuntimeError):
    """候选机械爪档案不能安全测试。"""


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GripperTestError(f"{name} 必须是 YAML 映射")
    return value


def write_candidate_config(
    source_path: str | Path,
    profile: str,
    destination: str | Path,
) -> RobotConfig:
    """生成一份仅供台架测试的配置副本。

    副本强制关闭真实运动、ROS 解锁和正常机械爪权限；不会
    改写操作员的 ``robot.yaml``。
    """

    source = Path(source_path).expanduser().resolve()
    destination_path = Path(destination).expanduser().resolve()
    if not source.is_file():
        raise GripperTestError(f"机器人配置不存在: {source}")
    content = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    root = dict(_mapping(content, "robot.yaml"))
    gripper = dict(_mapping(root.get("gripper"), "gripper"))
    profiles = _mapping(gripper.get("profiles"), "gripper.profiles")
    if profile not in profiles:
        raise GripperTestError(
            f"机械爪档案 {profile!r} 不存在；可用: {', '.join(profiles)}"
        )
    gripper["active_profile"] = profile
    root["gripper"] = gripper
    safety = dict(_mapping(root.get("safety"), "safety"))
    safety.update(
        {
            "allow_live_actuation": False,
            "allow_ros_arming": False,
            "allow_gripper_actuation": False,
        }
    )
    root["safety"] = safety
    destination_path.parent.mkdir(parents=True, exist_ok=True)
    destination_path.write_text(
        yaml.safe_dump(root, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return load_robot_config(destination_path)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="上锁状态测试 dalian 或 rst 机械爪候选档案"
    )
    parser.add_argument("profile", choices=("dalian", "rst"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--output-dir")
    parser.add_argument("--execute", action="store_true")
    return parser


def _yes_no(prompt: str) -> bool:
    while True:
        answer = input(f"{prompt} [y/n]: ").strip().lower()
        if answer in {"y", "yes"}:
            return True
        if answer in {"n", "no"}:
            return False
        print("请输入 y 或 n。")


def _write_results(
    output_directory: Path,
    *,
    profile: str,
    records: list[dict[str, object]],
    actual_opened: bool | None,
    actual_closed: bool | None,
    notes: str,
    outcome: str,
    detail: str,
) -> None:
    commands_path = output_directory / "commands.csv"
    with commands_path.open("w", encoding="utf-8", newline="") as stream:
        fields = ("action", "step", "output_channel", "pwm", "ack_result")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(records)
    result = {
        "schema_version": 1,
        "profile": profile,
        "finished_at_utc": datetime.now(timezone.utc).isoformat(),
        "outcome": outcome,
        "detail": detail,
        "actual_opened": actual_opened,
        "actual_closed": actual_closed,
        "operator_notes": notes,
        "active_profile_was_not_modified": True,
        "calibrated_was_not_modified": True,
        "files": {
            "commands": commands_path.name,
            "resolved_config": "resolved_gripper_test.yaml",
        },
    }
    temporary = output_directory / "result.json.tmp"
    temporary.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output_directory / "result.json")


def main(argv: list[str] | None = None) -> int:
    """预览候选档案，或在交互确认后执行上锁台架测试。"""

    args = _parser().parse_args(argv)
    temporary_directory: tempfile.TemporaryDirectory[str] | None = None
    if args.execute:
        if not args.output_dir:
            print("真实测试必须提供 --output-dir")
            return 2
        output_directory = Path(args.output_dir).expanduser().resolve()
        if output_directory.exists() and any(output_directory.iterdir()):
            print(f"输出目录已包含文件，拒绝覆盖: {output_directory}")
            return 2
        output_directory.mkdir(parents=True, exist_ok=True)
    else:
        temporary_directory = tempfile.TemporaryDirectory(prefix="rov_gripper_preview_")
        output_directory = Path(temporary_directory.name)

    resolved_config = output_directory / "resolved_gripper_test.yaml"
    try:
        config = write_candidate_config(args.config, args.profile, resolved_config)
    except (OSError, ConfigurationError, GripperTestError, TypeError, ValueError) as exc:
        print(f"配置错误: {exc}")
        if temporary_directory is not None:
            temporary_directory.cleanup()
        return 2

    values = config.gripper.all_pwm_values()
    print(f"候选档案: {config.gripper.profile}")
    print(f"绝对输出: {', '.join(f'S{item}' for item in config.gripper.output_channels)}")
    print(f"PWM 范围: {min(values)}..{max(values)} us")
    print(f"包含扩展 PWM: {'是' if config.gripper.uses_extended_pwm else '否'}")
    print("配置副本的真实运动/ROS解锁/正常爪权限均为关闭。")
    if not args.execute:
        print("预览完成；未连接 MAVLink，未发送任何执行器命令。")
        if temporary_directory is not None:
            temporary_directory.cleanup()
        return 0
    if not sys.stdin.isatty():
        print("真实测试必须在交互终端运行。")
        return 2

    vehicle = MavlinkVehicle(config)
    records: list[dict[str, object]] = []
    actual_opened: bool | None = None
    actual_closed: bool | None = None
    notes = ""
    outcome = "failed"
    detail = "测试未完成"
    exit_code = 2
    try:
        print("\n必须同时满足：")
        print("  1. 推进器动力或信号已物理断开；")
        print("  2. 机械爪和人员不在夹持危险区；")
        print("  3. QGC 显示飞控已上锁，可立即断电。")
        typed = input(f"完整输入 {GRIPPER_TEST_CONFIRMATION!r}: ").strip()
        if typed != GRIPPER_TEST_CONFIRMATION:
            raise GripperTestError("第一确认词不匹配")
        extended_confirmation = ""
        if config.gripper.uses_extended_pwm:
            print(
                "RST 历史档案含 500us，只有已核对接线、驱动器规格"
                "和机械行程时才能继续。"
            )
            extended_confirmation = input(
                f"完整输入 {EXTENDED_PWM_CONFIRMATION!r}: "
            ).strip()
            if extended_confirmation != EXTENDED_PWM_CONFIRMATION:
                raise GripperTestError("扩展 PWM 确认词不匹配")

        vehicle.connect()
        report = vehicle.run_preflight()
        json_path, markdown_path = report.write(output_directory / "preflight")
        print(f"只读预检报告: {json_path}")
        print(f"只读预检报告: {markdown_path}")
        if not report.passed:
            names = ", ".join(check.name for check in report.critical_failures)
            raise GripperTestError(f"飞控只读预检未通过: {names}")

        input("\n安全员再次确认推进器已断开，按 Enter 发送开爪序列……")
        opened = vehicle.run_disarmed_gripper_test(
            GripperAction.OPEN,
            confirmation=typed,
            extended_pwm_confirmation=extended_confirmation,
        )
        records.extend({"action": "open", **item} for item in opened)
        actual_opened = _yes_no("实物是否确实开爪")

        input("\n再次确认夹持区无人，按 Enter 发送闭爪序列……")
        closed = vehicle.run_disarmed_gripper_test(
            GripperAction.CLOSE,
            confirmation=typed,
            extended_pwm_confirmation=extended_confirmation,
        )
        records.extend({"action": "close", **item} for item in closed)
        actual_closed = _yes_no("实物是否确实闭爪")
        notes = input("操作员备注（可留空）: ").strip()
        outcome = "passed" if actual_opened and actual_closed else "not_calibrated"
        detail = (
            "开爪和闭爪均已由操作员确认"
            if outcome == "passed"
            else "至少一个实物动作未通过，不得启用该档案"
        )
        exit_code = 0 if outcome == "passed" else 3
    except KeyboardInterrupt:
        outcome = "interrupted"
        detail = "Ctrl+C 中断；请断电后检查机械爪"
        exit_code = 130
    except (OSError, VehicleError, GripperTestError) as exc:
        outcome = "failed"
        detail = str(exc)
        print(f"\n机械爪候选测试中止: {detail}")
        exit_code = 2
    finally:
        try:
            vehicle.close()
        except VehicleError as exc:
            detail += f"；关闭 MAVLink 失败: {exc}"
            exit_code = 2
        _write_results(
            output_directory,
            profile=args.profile,
            records=records,
            actual_opened=actual_opened,
            actual_closed=actual_closed,
            notes=notes,
            outcome=outcome,
            detail=detail,
        )

    print(f"\n测试记录: {output_directory}")
    print("本工具没有修改 robot.yaml，也没有自动标记 calibrated。")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
