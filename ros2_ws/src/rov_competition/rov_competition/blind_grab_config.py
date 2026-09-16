"""盲抓专用配置，只解析动作及通信参数，不读取旧网关的执行许可。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

from .blind_grab import BlindGrabConfig, ServoAction, ServoSetpoint


class BlindGrabConfigurationError(ValueError):
    pass


@dataclass(frozen=True)
class MavlinkSettings:
    connection_uri: str = "udpin:0.0.0.0:14551"
    baud: int = 115200
    source_system: int = 255
    source_component: int = 191
    target_system: int = 1
    target_component: int = 1
    directions: tuple[int, int, int, int] = (1, 1, 1, 1)
    reconnect_interval_s: float = 1.0
    servo_refresh_s: float = 0.25


@dataclass(frozen=True)
class VisionSettings:
    target_labels: tuple[str, ...] = ("scallop",)
    confidence: float = 0.18
    start_helpers: bool = True
    show_viewer: bool = True
    record_video: bool = False
    source_port: int = 5700
    inference_port: int = 5702
    record_port: int = 5704
    robot_config: Path | None = None
    autonomy_config: Path | None = None


@dataclass(frozen=True)
class BlindGrabAppConfig:
    mission: BlindGrabConfig
    mavlink: MavlinkSettings
    vision: VisionSettings
    source_path: Path


def package_config_path(name: str) -> Path:
    source = Path(__file__).resolve().parent.parent / "config" / name
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("rov_competition")) / "config" / name
    except (ImportError, LookupError):
        return source


def load_blind_grab_config(path: str | Path) -> BlindGrabAppConfig:
    import yaml

    path = Path(path).expanduser().resolve()
    try:
        root = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise BlindGrabConfigurationError(f"无法读取盲抓配置 {path}: {exc}") from exc
    if not isinstance(root, dict):
        raise BlindGrabConfigurationError("盲抓配置根节点必须是映射")
    errors: list[str] = []

    def section(parent: dict[str, Any], name: str) -> dict[str, Any]:
        value = parent.get(name, {})
        if not isinstance(value, dict):
            errors.append(f"{name}: 必须是映射")
            return {}
        return value

    def number(value: Any, name: str, *, positive: bool = True) -> float:
        if value is None:
            errors.append(f"{name}: 尚未填写")
            return 1.0
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors.append(f"{name}: 必须是数值")
            return 1.0
        result = float(value)
        if not math.isfinite(result) or (positive and result <= 0):
            errors.append(f"{name}: 必须是{'正的' if positive else ''}有限数")
        return result

    def integer(value: Any, name: str, maximum: int | None = None) -> int:
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            errors.append(f"{name}: {'尚未填写' if value is None else '必须是正整数'}")
            return 1
        if maximum is not None and value > maximum:
            errors.append(f"{name}: 协议取值不能大于 {maximum}")
        return value

    def boolean(value: Any, name: str) -> bool:
        if not isinstance(value, bool):
            errors.append(f"{name}: 必须是 true 或 false")
            return False
        return value

    def power(value: Any, name: str) -> float:
        result = number(value, name, positive=False)
        if not -1.0 <= result <= 1.0:
            errors.append(f"{name}: 归一化运动指令必须在 [-1, 1]")
        return result

    actions = section(root, "actions")

    def action(name: str) -> ServoAction:
        spec = section(actions, name)
        outputs = spec.get("outputs")
        values: list[ServoSetpoint] = []
        if not isinstance(outputs, list) or not outputs:
            errors.append(f"actions.{name}.outputs: 请填写舵机输出列表")
        else:
            for index, item in enumerate(outputs):
                label = f"actions.{name}.outputs[{index}]"
                if not isinstance(item, dict):
                    errors.append(f"{label}: 必须包含 output_channel 和 pwm")
                    continue
                values.append(ServoSetpoint(
                    integer(item.get("output_channel"), f"{label}.output_channel"),
                    integer(item.get("pwm"), f"{label}.pwm"),
                ))
        return ServoAction(tuple(values), number(spec.get("duration_s"), f"actions.{name}.duration_s"))

    search, grasp, trigger = (section(root, key) for key in ("search", "grab", "trigger"))
    mission = BlindGrabConfig(
        lane_forward_duration_s=number(search.get("lane_forward_duration_s"), "search.lane_forward_duration_s"),
        advance_duration_s=number(grasp.get("advance_duration_s"), "grab.advance_duration_s"),
        open_gripper=action("open_gripper"), close_gripper=action("close_gripper"),
        arm_to_basket=action("arm_to_basket"), arm_to_grasp=action("arm_to_grasp"),
        release_duration_s=number(grasp.get("release_duration_s"), "grab.release_duration_s"),
        forward_command=power(search.get("forward_command", 0.23), "search.forward_command"),
        shift_command=power(search.get("shift_command", 0.20), "search.shift_command"),
        shift_duration_s=number(search.get("shift_duration_s", 4.5), "search.shift_duration_s"),
        turn_command=power(search.get("turn_command", 0.20), "search.turn_command"),
        turn_duration_s=number(search.get("turn_duration_s", 11.5), "search.turn_duration_s"),
        grab_forward_command=power(grasp.get("forward_command", 0.23), "grab.forward_command"),
        required_boxes=integer(trigger.get("required_boxes", 4), "trigger.required_boxes"),
        confirmation_frames=integer(trigger.get("confirmation_frames", 5), "trigger.confirmation_frames"),
        confirmation_duration_s=number(trigger.get("confirmation_duration_s", 1.0), "trigger.confirmation_duration_s"),
        missing_frame_timeout_s=number(trigger.get("missing_frame_timeout_s", 1.0), "trigger.missing_frame_timeout_s"),
        fallback_after_s=number(trigger.get("fallback_after_s", 30.0), "trigger.fallback_after_s"),
        grabs_per_batch=integer(grasp.get("grabs_per_batch", 10), "grab.grabs_per_batch"),
    )
    # 同时保持夹爪和机械臂姿态时不能给同一输出发送互相覆盖的两种 PWM。
    for claw in (mission.open_gripper, mission.close_gripper):
        for arm in (mission.arm_to_basket, mission.arm_to_grasp):
            channels = [item.output_channel for item in claw.outputs + arm.outputs]
            if len(set(channels)) != len(channels) and not errors:
                errors.append("actions: 夹爪和机械臂同时使用了重复的输出通道，请填写各自通道")

    link = section(root, "mavlink")
    uri = link.get("connection_uri", "udpin:0.0.0.0:14551")
    if not isinstance(uri, str) or not uri.strip():
        errors.append("mavlink.connection_uri: 请填写连接地址")
    direction_values = section(link, "directions")
    directions = []
    for axis in ("forward", "lateral", "vertical", "yaw"):
        value = direction_values.get(axis, 1)
        if isinstance(value, bool) or value not in (-1, 1):
            errors.append(f"mavlink.directions.{axis}: 必须是 1 或 -1")
        directions.append(value)
    mavlink = MavlinkSettings(
        connection_uri=uri, baud=integer(link.get("baud", 115200), "mavlink.baud"),
        source_system=integer(link.get("source_system", 255), "mavlink.source_system", 255),
        source_component=integer(link.get("source_component", 191), "mavlink.source_component", 255),
        target_system=integer(link.get("target_system", 1), "mavlink.target_system", 255),
        target_component=integer(link.get("target_component", 1), "mavlink.target_component", 255),
        directions=tuple(directions),
    )
    vision_root = section(root, "vision")
    labels = vision_root.get("target_labels", ["scallop"])
    if not isinstance(labels, list) or not labels or any(not isinstance(x, str) or not x.strip() for x in labels):
        errors.append("vision.target_labels: 请填写类别名称列表")
        labels = ["scallop"]
    confidence = number(vision_root.get("confidence", 0.18), "vision.confidence", positive=False)
    if not 0 <= confidence <= 1:
        errors.append("vision.confidence: 必须在 [0, 1]")

    def config_file(key: str, default: str) -> Path:
        value = vision_root.get(key)
        if value is None:
            return package_config_path(default)
        if not isinstance(value, str):
            errors.append(f"vision.{key}: 必须是路径或 null")
            return package_config_path(default)
        p = Path(value).expanduser()
        return p.resolve() if p.is_absolute() else (path.parent / p).resolve()

    vision = VisionSettings(
        target_labels=tuple(labels), confidence=confidence,
        start_helpers=boolean(vision_root.get("start_helpers", True), "vision.start_helpers"),
        show_viewer=boolean(vision_root.get("show_viewer", True), "vision.show_viewer"),
        record_video=boolean(vision_root.get("record_video", False), "vision.record_video"),
        source_port=integer(vision_root.get("source_port", 5700), "vision.source_port", 65535),
        inference_port=integer(vision_root.get("inference_port", 5702), "vision.inference_port", 65535),
        record_port=integer(vision_root.get("record_port", 5704), "vision.record_port", 65535),
        robot_config=config_file("robot_config", "robot.example.yaml"),
        autonomy_config=config_file("autonomy_config", "autonomy.yaml"),
    )
    if errors:
        raise BlindGrabConfigurationError("请填写或修正以下盲抓参数：\n- " + "\n- ".join(dict.fromkeys(errors)))
    return BlindGrabAppConfig(mission, mavlink, vision, path)
