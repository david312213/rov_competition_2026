"""QGC/手柄人工驾驶时，只上传比赛官方 ROS 数据。"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any
from uuid import uuid4

from .blind_grab_config import (
    BlindGrabConfigurationError,
    load_blind_grab_config,
    package_config_path,
)
from .blind_grab_official import OfficialDataService
from .official_data_only_mavlink import PassiveMavlinkTelemetry

POLL_PERIOD_S = 0.05


def say(message: str) -> None:
    try:
        print(message, flush=True)
    except (OSError, ValueError):
        pass


def run_data_only_loop(
    telemetry: PassiveMavlinkTelemetry,
    stop_event: Any,
    *,
    clock=time.monotonic,
) -> None:
    """以20 Hz读取遥测；通信失败时保持进程运行并继续重试。"""

    while not stop_event.is_set():
        started = clock()
        telemetry.poll(started)
        stop_event.wait(max(0.0, POLL_PERIOD_S - (clock() - started)))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "QGC/手柄人工驾驶时，仅被动读取MAVLink遥测并上传官方ROS数据；"
            "不发送任何飞控、运动、舵机、解锁或模式指令。"
        )
    )
    parser.add_argument(
        "--config",
        default=str(package_config_path("blind_grab.yaml")),
        help="包含MAVLink和官方服务器参数的配置文件",
    )
    parser.add_argument(
        "--output-dir",
        default="output/official_data_only_sessions",
        help="官方TCP转发日志目录",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只显示端点与只读策略，不连接MAVLink、不初始化ROS",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_blind_grab_config(args.config)
    except (BlindGrabConfigurationError, ImportError) as exc:
        say(str(exc))
        return 2
    if not config.official_ros.enabled:
        say(f"官方ROS在配置 {config.source_path} 中被关闭，请先运行队伍启动脚本。")
        return 2
    if args.dry_run:
        say(json.dumps({
            "mode": "official_data_only",
            "config": str(config.source_path),
            "mavlink": asdict(config.mavlink),
            "official_ros": asdict(config.official_ros),
            "flight_controller_writes": False,
            "motion_publishers_created": False,
            "starts_blind_grab": False,
            "starts_video_or_yolo": False,
        }, ensure_ascii=False, indent=2))
        return 0

    stop_event = threading.Event()
    previous_handlers: dict[int, Any] = {}

    def request_stop(signum: int, frame: Any) -> None:
        del signum, frame
        stop_event.set()

    handled_signals = (signal.SIGINT, signal.SIGTERM)
    if hasattr(signal, "SIGHUP"):
        handled_signals += (signal.SIGHUP,)
    for signum in handled_signals:
        previous_handlers[signum] = signal.signal(signum, request_stop)

    directory = Path(args.output_dir).expanduser().resolve() / (
        f"{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}"
    )
    telemetry = PassiveMavlinkTelemetry(
        config.mavlink,
        config.official_ros,
        report=say,
    )
    official = OfficialDataService(
        config.official_ros,
        config.mavlink,
        telemetry.telemetry_snapshot,
        directory,
        publish_motion=False,
        node_name="rov_official_data_only",
        report=say,
    )

    say("开始主办方数据上传：QGC/手柄继续人工驾驶。")
    say(
        "本进程只读MAVLink，不发送MANUAL_CONTROL、RC覆盖、舵机、解锁、"
        "模式或心跳报文，也不发布自造/cmd_vel或/cmd_accel。"
    )
    say(
        f"数据服务器: {config.official_ros.server_ip}:"
        f"{config.official_ros.server_port}；配置: {config.source_path}"
    )
    say(f"转发日志: {directory / 'official_ros_forwarder.log'}；Ctrl+C关闭上传。")
    try:
        official.start()
        run_data_only_loop(telemetry, stop_event)
    finally:
        stop_event.set()
        for error in telemetry.shutdown():
            say(f"[关闭只读遥测] {error}")
        official.shutdown()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    say("主办方数据上传已关闭；未向飞控发送关闭或归中指令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
