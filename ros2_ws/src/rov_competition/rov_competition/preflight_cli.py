"""只读飞控预检命令行工具。"""

from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path

from .config import ConfigurationError, load_robot_config
from .vehicle import MavlinkVehicle, VehicleError


def _build_parser() -> argparse.ArgumentParser:
    """创建只读预检参数解析器。"""

    parser = argparse.ArgumentParser(
        description="读取 ArduSub 身份和参数，生成报告；绝不写参数或执行器。"
    )
    parser.add_argument("--config", required=True, help="已填写的 robot YAML 路径")
    parser.add_argument(
        "--output-dir",
        default="output/preflight",
        help="JSON 和 Markdown 报告目录（默认: output/preflight）",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """执行只读预检；通过返回 0，未通过返回 2。"""

    arguments = _build_parser().parse_args(argv)
    try:
        loaded = load_robot_config(arguments.config)
        # 即使实艇 YAML 已开启动作，这个独立工具也会强制用只读副本。
        config = replace(
            loaded,
            allow_live_actuation=False,
            allow_ros_arming=False,
            allow_gripper_actuation=False,
        )
    except ConfigurationError as exc:
        print(f"配置错误: {exc}")
        return 2

    vehicle = MavlinkVehicle(config)
    try:
        vehicle.connect()
        report = vehicle.run_preflight()
        json_path, markdown_path = report.write(Path(arguments.output_dir))
        print(report.to_markdown())
        print(f"JSON 报告: {json_path}")
        print(f"Markdown 报告: {markdown_path}")
        return 0 if report.passed else 2
    except (OSError, VehicleError) as exc:
        print(f"预检失败: {exc}")
        return 2
    finally:
        vehicle.close()


if __name__ == "__main__":
    raise SystemExit(main())
