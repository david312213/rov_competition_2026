"""用持续盲抓配置直接检查夹爪和机械臂的交互式按键入口。"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import select
import sys
import termios
import time
import tty
from collections.abc import Callable
from typing import TextIO

from .blind_grab import BlindGrabConfig, ServoSetpoint
from .blind_grab_config import (
    BlindGrabConfigurationError,
    load_blind_grab_config,
    package_config_path,
)
from .blind_grab_mavlink import BlindMavlinkOutput
from .domain import MotionCommand


CONTROL_PERIOD_S = 0.05
KEY_BINDINGS = {
    "a": ("夹爪张开", "gripper", "open_gripper"),
    "b": ("夹爪闭合", "gripper", "close_gripper"),
    "c": ("机械臂后仰到筐位", "arm", "arm_to_basket"),
    "d": ("机械臂恢复抓取位", "arm", "arm_to_grasp"),
}


@dataclass
class ActuatorPose:
    """分别记住夹爪和机械臂的最后选择，后续按键不会覆盖另一组。"""

    gripper: tuple[ServoSetpoint, ...] = ()
    arm: tuple[ServoSetpoint, ...] = ()

    @property
    def outputs(self) -> tuple[ServoSetpoint, ...]:
        return self.gripper + self.arm

    def apply(
        self, key: str, config: BlindGrabConfig,
    ) -> tuple[str, tuple[ServoSetpoint, ...]] | None:
        binding = KEY_BINDINGS.get(key.lower())
        if binding is None:
            return None
        label, group, action_name = binding
        action = getattr(config, action_name)
        setattr(self, group, action.outputs)
        return label, action.outputs


def format_outputs(outputs: tuple[ServoSetpoint, ...]) -> str:
    return ", ".join(f"S{item.output_channel}={item.pwm}us" for item in outputs)


class TerminalKeyReader:
    """在交互终端读取单个按键，不要求按 Enter。"""

    def __init__(self, stream: TextIO = sys.stdin) -> None:
        self.stream = stream
        self.fd = stream.fileno()
        self._previous: list | None = None

    def __enter__(self) -> "TerminalKeyReader":
        if not self.stream.isatty():
            raise OSError("必须在交互终端运行，不能从管道读取按键")
        self._previous = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        del exc_type, exc, traceback
        if self._previous is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._previous)
            self._previous = None

    def read(self, timeout_s: float) -> str | None:
        ready, _, _ = select.select([self.fd], [], [], max(0.0, timeout_s))
        if not ready:
            return None
        value = os.read(self.fd, 1)
        if not value:
            return "q"
        return value.decode("utf-8", errors="ignore")


def run_console(
    config: BlindGrabConfig,
    output: BlindMavlinkOutput,
    read_key: Callable[[float], str | None],
    *,
    clock: Callable[[], float] = time.monotonic,
    report: Callable[[str], None] = print,
    period_s: float = CONTROL_PERIOD_S,
) -> ActuatorPose:
    """持续发送运动归中和当前臂爪姿态，直到 q、Esc 或 Ctrl+C。"""

    pose = ActuatorPose()
    neutral = MotionCommand.neutral()
    while True:
        started = clock()
        try:
            output.publish(neutral, pose.outputs, started)
        except Exception as exc:
            report(f"[臂爪测试通信] {type(exc).__name__}: {exc}；继续重试")

        key = read_key(max(0.0, period_s - (clock() - started)))
        if key is None:
            continue
        key = key.lower()
        if key in {"q", "\x1b"}:
            return pose
        if key in {"\r", "\n", " ", "\t"}:
            continue

        selected = pose.apply(key, config)
        if selected is None:
            report("未知按键；使用 a / b / c / d，按 q 退出。")
            continue
        label, changed = selected
        now = clock()
        try:
            sent = output.publish(neutral, pose.outputs, now)
        except Exception as exc:
            sent = False
            report(f"[臂爪测试通信] {type(exc).__name__}: {exc}；继续重试")
        state = "正在持续发送" if sent else "已选择，连接成功后持续发送"
        report(f"{label}：{format_outputs(changed)}（{state}）")


def _default_config_path() -> Path:
    local = Path.cwd() / "config" / "blind_grab.local.yaml"
    return local if local.is_file() else package_config_path("blind_grab.yaml")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="按 a/b/c/d 检查持续盲抓的夹爪与机械臂")
    parser.add_argument("--config", type=Path, default=_default_config_path())
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        app_config = load_blind_grab_config(args.config)
    except (BlindGrabConfigurationError, OSError, ValueError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    mission = app_config.mission
    print(f"配置：{app_config.source_path}")
    print(f"MAVLink：{app_config.mavlink.connection_uri}")
    print("按键：")
    for key in ("a", "b", "c", "d"):
        label, _, action_name = KEY_BINDINGS[key]
        print(f"  {key}  {label:<10} {format_outputs(getattr(mission, action_name).outputs)}")
    print("  q  退出")
    print("请关闭其他盲抓或运动控制程序。本工具不解锁飞控，所有运动轴持续归中。")

    output = BlindMavlinkOutput(app_config.mavlink)
    exit_code = 0
    try:
        with TerminalKeyReader() as reader:
            run_console(mission, output, reader.read)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，结束臂爪测试。")
    except OSError as exc:
        print(f"无法读取按键：{exc}", file=sys.stderr)
        exit_code = 2
    finally:
        for error in output.shutdown():
            print(f"结束通信：{error}", file=sys.stderr)
    print("臂爪测试已退出；最后一次舵机姿态保持不变。")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
