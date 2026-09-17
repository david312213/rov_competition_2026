"""交互输入动作和PWM，直接试调持续盲抓的夹爪与机械臂。"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import sys
import threading
import time

from .blind_grab import BlindGrabConfig, ServoSetpoint
from .blind_grab_actuator_console import ActuatorPose, KEY_BINDINGS, format_outputs
from .blind_grab_config import (
    BlindGrabConfigurationError,
    load_blind_grab_config,
    package_config_path,
)
from .blind_grab_mavlink import BlindMavlinkOutput
from .domain import MotionCommand


CONTROL_PERIOD_S = 0.05
MIN_PWM = 0
MAX_PWM = 65535


@dataclass(frozen=True)
class TestedPwm:
    key: str
    label: str
    outputs: tuple[ServoSetpoint, ...]


def apply_manual_pwm(
    pose: ActuatorPose,
    key: str,
    pwm: int,
    config: BlindGrabConfig,
) -> TestedPwm | None:
    """将输入PWM应用到所选动作的配置通道，并保留另一组当前姿态。"""

    binding = KEY_BINDINGS.get(key.lower())
    if binding is None:
        return None
    label, group, action_name = binding
    configured = getattr(config, action_name).outputs
    outputs = tuple(
        ServoSetpoint(item.output_channel, pwm)
        for item in configured
    )
    setattr(pose, group, outputs)
    return TestedPwm(key.lower(), label, outputs)


def parse_pwm(text: str) -> int | None:
    try:
        value = int(text.strip(), 10)
    except ValueError:
        return None
    return value if MIN_PWM <= value <= MAX_PWM else None


def run_pwm_prompt(
    config: BlindGrabConfig,
    set_servos: Callable[[tuple[ServoSetpoint, ...]], None],
    *,
    input_line: Callable[[str], str] = input,
    report: Callable[[str], None] = print,
) -> tuple[ActuatorPose, dict[str, TestedPwm]]:
    """按“动作字母、PWM数字”循环试调；EOF和q均正常退出。"""

    pose = ActuatorPose()
    tested: dict[str, TestedPwm] = {}
    while True:
        try:
            key = input_line("动作 [a张爪/b闭爪/c后仰/d恢复/q退出]：").strip().lower()
        except EOFError:
            return pose, tested
        if key == "q":
            return pose, tested
        if key not in KEY_BINDINGS:
            report("请输入 a、b、c、d 或 q。")
            continue

        label, _, action_name = KEY_BINDINGS[key]
        configured = getattr(config, action_name).outputs
        try:
            raw_pwm = input_line(
                f"{label}，通道 {', '.join(f'S{x.output_channel}' for x in configured)}，"
                f"当前配置 {format_outputs(configured)}；输入PWM（空行取消）："
            )
        except EOFError:
            return pose, tested
        if not raw_pwm.strip():
            report("已取消本次输入。")
            continue
        pwm = parse_pwm(raw_pwm)
        if pwm is None:
            report(f"PWM必须是 {MIN_PWM} 到 {MAX_PWM} 的整数。")
            continue

        result = apply_manual_pwm(pose, key, pwm, config)
        assert result is not None
        tested[key] = result
        set_servos(pose.outputs)
        report(f"{result.label}：{format_outputs(result.outputs)}（正在持续发送）")


class ContinuousServoSender:
    """输入等待期间仍以20Hz发送运动归中和当前舵机姿态。"""

    def __init__(
        self,
        output: BlindMavlinkOutput,
        *,
        report: Callable[[str], None] = print,
    ) -> None:
        self.output = output
        self.report = report
        self._servos: tuple[ServoSetpoint, ...] = ()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="blind-grab-pwm-test",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def set_servos(self, servos: tuple[ServoSetpoint, ...]) -> None:
        with self._lock:
            self._servos = servos

    def _run(self) -> None:
        neutral = MotionCommand.neutral()
        next_report_at = 0.0
        while not self._stop.is_set():
            started = time.monotonic()
            with self._lock:
                servos = self._servos
            try:
                self.output.publish(neutral, servos, started)
            except Exception as exc:
                if started >= next_report_at:
                    next_report_at = started + 2.0
                    self.report(f"[PWM测试通信] {type(exc).__name__}: {exc}；继续重试")
            self._stop.wait(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))

    def close(self) -> list[str]:
        self._stop.set()
        self._thread.join(timeout=2.0)
        return self.output.shutdown()


def _default_config_path() -> Path:
    local = Path.cwd() / "config" / "blind_grab.local.yaml"
    return local if local.is_file() else package_config_path("blind_grab.yaml")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="手动输入PWM试调夹爪与机械臂")
    parser.add_argument("--config", type=Path, default=_default_config_path())
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        app_config = load_blind_grab_config(args.config)
    except (BlindGrabConfigurationError, OSError, ValueError) as exc:
        print(f"配置错误：{exc}", file=sys.stderr)
        return 2

    print(f"配置：{app_config.source_path}")
    print(f"MAVLink：{app_config.mavlink.connection_uri}")
    print("输入动作字母并回车，再输入PWM数字并回车；输入q退出。")
    print("本工具只试调输出，不修改配置文件；所有运动轴持续归中。")

    sender = ContinuousServoSender(BlindMavlinkOutput(app_config.mavlink))
    sender.start()
    tested: dict[str, TestedPwm] = {}
    try:
        _, tested = run_pwm_prompt(app_config.mission, sender.set_servos)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，结束PWM测试。")
    finally:
        for error in sender.close():
            print(f"结束通信：{error}", file=sys.stderr)

    if tested:
        print("本次最后测试值：")
        for key in ("a", "b", "c", "d"):
            if key in tested:
                item = tested[key]
                print(f"  {key} {item.label}：{format_outputs(item.outputs)}")
    else:
        print("本次没有发送手动PWM。")
    print("PWM测试已退出；配置文件没有改变，最后一次舵机姿态保持不变。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
