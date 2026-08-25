"""YAML 配置读取与严格校验。

旧代码把 UDP 端口、RC 通道、PWM 和识别阈值分散写死在多个脚本中。本模块把
这些会随机器人变化的值集中管理，并在连接真实硬件前尽早拒绝危险或缺失配置。
"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import yaml


class ConfigurationError(ValueError):
    """配置内容缺失、类型错误或超出安全范围。"""


def _read_yaml(path: str | Path) -> Mapping[str, Any]:
    """安全读取 YAML，并保证根节点是字典。"""

    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ConfigurationError(f"配置文件不存在: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        content = yaml.safe_load(stream) or {}
    if not isinstance(content, Mapping):
        raise ConfigurationError(f"配置文件根节点必须是映射: {config_path}")
    return content


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    """校验并返回一个映射配置段。"""

    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{name} 必须是映射")
    return value


def _boolean(value: Any, name: str) -> bool:
    """严格读取布尔值，拒绝会被 Python 误判为真的 ``"false"`` 字符串。"""

    if not isinstance(value, bool):
        raise ConfigurationError(f"{name} 必须是 YAML 布尔值 true 或 false")
    return value


def _string_tuple(value: Any, name: str) -> tuple[str, ...]:
    """读取非空、无重复的字符串列表，拒绝把单个字符串拆成字符。"""

    if not isinstance(value, (list, tuple)):
        raise ConfigurationError(f"{name} 必须是字符串列表")
    items = tuple(str(item).strip() for item in value)
    if not items or any(not item for item in items):
        raise ConfigurationError(f"{name} 不能为空或包含空字符串")
    if len(set(items)) != len(items):
        raise ConfigurationError(f"{name} 不能包含重复项")
    return items


def _finite(value: Any, name: str) -> float:
    """读取有限浮点数，拒绝 NaN 和正负无穷。"""

    number = float(value)
    if not math.isfinite(number):
        raise ConfigurationError(f"{name} 必须是有限数值")
    return number


def _optional_sign(value: Any, name: str) -> int | None:
    """读取待实机标定的图像控制方向。

    ``null`` 表示尚未标定，并不是默认的正方向。只接受 ``-1``
    或 ``1``，防止把布尔值或其他倍数当成方向。
    """

    if value is None:
        return None
    if isinstance(value, bool):
        raise ConfigurationError(f"{name} 只能是 -1、1 或 null")
    sign = int(value)
    if sign not in (-1, 1) or float(value) != sign:
        raise ConfigurationError(f"{name} 只能是 -1、1 或 null")
    return sign


class ControlProtocol(str, Enum):
    """ROS 运动意图发送给 ArduSub 时使用的固定协议。"""

    MANUAL_CONTROL = "manual_control"
    RC_OVERRIDE = "rc_override"


class ControlProfile(str, Enum):
    """互斥的控制所有者；网关启动后不得在两者之间切换。"""

    COMMISSIONING = "commissioning"
    AUTONOMY = "autonomy"


@dataclass(frozen=True)
class RcOverrideConfig:
    """旧固件兼容所需的 RC 输入通道。

    MIN/TRIM/MAX 不保存在 YAML，而是每次从飞控只读参数报告获取，
    避免四个通道被错误套用同一组旧艇标定。
    """

    channels: Mapping[str, int]

    def __post_init__(self) -> None:
        """拒绝缺轴、重复通道和越界通道号。"""

        required_axes = {"forward", "lateral", "vertical", "yaw"}
        if set(self.channels) != required_axes:
            raise ConfigurationError(
                f"rc_override.channels 必须且只能包含 {sorted(required_axes)}"
            )
        numbers = list(self.channels.values())
        if any(not 1 <= channel <= 18 for channel in numbers):
            raise ConfigurationError("RC 通道必须在 1..18")
        if len(set(numbers)) != len(numbers):
            raise ConfigurationError("四个运动轴不能复用同一个 RC 通道")


@dataclass(frozen=True)
class GripperSweepConfig:
    """一次开爪或闭爪的 PWM 渐变。

    旧大连工程不是只发送一个终点值，而是按固定步长连续发送一组
    ``MAV_CMD_DO_SET_SERVO``。把起点、终点和步长写进配置，既能复现
    实艇旧逻辑，也能在换舵机后只改 YAML，不再复制一份脚本。
    """

    start_pwm: int
    end_pwm: int
    step_pwm: int

    def __post_init__(self) -> None:
        """拒绝越界、零步长和朝错误方向变化的序列。"""

        # RST 原型的右侧执行器闭爪值是 500us。这里只允许配置文件忠实描述
        # 500..2500 的历史候选范围；是否真的允许超出常见 800..2200，仍由
        # GripperConfig.allow_extended_pwm 和实艇安全门共同决定。
        if not 500 <= self.start_pwm <= 2500 or not 500 <= self.end_pwm <= 2500:
            raise ConfigurationError("机械爪 PWM 必须在 500..2500")
        if self.step_pwm == 0 or abs(self.step_pwm) > 200:
            raise ConfigurationError("机械爪 PWM 步长必须非零且绝对值不超过 200")
        delta = self.end_pwm - self.start_pwm
        if delta != 0 and (delta > 0) != (self.step_pwm > 0):
            raise ConfigurationError("机械爪 PWM 步长方向必须指向终点")

    def values(self) -> tuple[int, ...]:
        """返回包含首尾值的有限 PWM 序列。"""

        if self.start_pwm == self.end_pwm:
            return (self.start_pwm,)
        values = [self.start_pwm]
        current = self.start_pwm
        if self.step_pwm > 0:
            while current < self.end_pwm:
                current = min(self.end_pwm, current + self.step_pwm)
                values.append(current)
        else:
            while current > self.end_pwm:
                current = max(self.end_pwm, current + self.step_pwm)
                values.append(current)
        return tuple(values)


@dataclass(frozen=True)
class GripperOutputConfig:
    """一个机械爪执行器对应的绝对 SERVO 输出和开闭曲线。"""

    output_channel: int
    open_sweep: GripperSweepConfig
    close_sweep: GripperSweepConfig

    def __post_init__(self) -> None:
        """输出号必须是 MAVLink SERVO 实例号，不接受 AUX 相对编号。"""

        if not 1 <= self.output_channel <= 32:
            raise ConfigurationError("机械爪绝对 SERVO 输出号必须在 1..32")

    def sweep_for(self, action: str) -> GripperSweepConfig:
        """按领域动作名返回该执行器的曲线。"""

        if action == "open":
            return self.open_sweep
        if action == "close":
            return self.close_sweep
        raise ConfigurationError(f"未知机械爪动作: {action}")


@dataclass(frozen=True)
class GripperConfig:
    """当前选中的机械爪档案；可包含一个或多个同步执行器。"""

    profile: str
    step_interval_s: float
    calibrated: bool
    allow_extended_pwm: bool
    outputs: tuple[GripperOutputConfig, ...]

    def __post_init__(self) -> None:
        """在连接飞控前校验输出号和节拍。"""

        if not re.fullmatch(r"[a-z][a-z0-9_]*", self.profile):
            raise ConfigurationError("机械爪档案名必须使用小写字母、数字或下划线")
        if not self.outputs:
            raise ConfigurationError("机械爪档案至少需要一个输出")
        output_channels = [output.output_channel for output in self.outputs]
        if len(set(output_channels)) != len(output_channels):
            raise ConfigurationError("机械爪档案不能重复使用同一个 SERVO 输出")
        if not math.isfinite(self.step_interval_s) or not (
            0.02 <= self.step_interval_s <= 1.0
        ):
            raise ConfigurationError("机械爪每步间隔必须在 0.02..1.0 秒")

    @property
    def output_channels(self) -> tuple[int, ...]:
        """返回档案使用的全部绝对 SERVO 输出号。"""

        return tuple(output.output_channel for output in self.outputs)

    @property
    def uses_extended_pwm(self) -> bool:
        """档案中出现常见 800..2200 范围外的历史 PWM 时返回真。"""

        return any(
            not 800 <= pwm <= 2200
            for output in self.outputs
            for sweep in (output.open_sweep, output.close_sweep)
            for pwm in sweep.values()
        )

    def steps_for(self, action: str) -> tuple[tuple[tuple[int, int], ...], ...]:
        """把一个或多个输出的曲线合成为按节拍执行的命令帧。

        同一帧中的多路命令会背靠背发送；较短曲线完成后不会反复重发终点。
        大连档案每帧只有 AUX4，RST 档案第一帧同时包含 AUX3 和 AUX2。
        """

        sweeps = tuple(
            (output.output_channel, output.sweep_for(action).values())
            for output in self.outputs
        )
        frame_count = max(len(values) for _, values in sweeps)
        frames: list[tuple[tuple[int, int], ...]] = []
        for index in range(frame_count):
            frame = tuple(
                (output_channel, values[index])
                for output_channel, values in sweeps
                if index < len(values)
            )
            frames.append(frame)
        return tuple(frames)

    def all_pwm_values(self) -> tuple[int, ...]:
        """返回当前档案全部开闭候选值，供预检和报告使用。"""

        return tuple(
            pwm
            for output in self.outputs
            for sweep in (output.open_sweep, output.close_sweep)
            for pwm in sweep.values()
        )


@dataclass(frozen=True)
class RobotConfig:
    """飞控连接、运动协议、实艇预检和执行安全门配置。"""

    connection_uri: str
    heartbeat_timeout_s: float
    heartbeat_stale_timeout_s: float
    telemetry_stale_timeout_s: float
    preflight_timeout_s: float
    baud: int
    source_system: int
    source_component: int
    control_protocol: ControlProtocol
    control_profile: ControlProfile
    allowed_flight_modes: tuple[str, ...]
    command_limit: float
    maximum_command_age_s: float
    slew_rate_per_s: float
    command_timeout_s: float
    allow_live_actuation: bool
    allow_ros_arming: bool
    allow_gripper_actuation: bool
    expected_frame_config: int | None
    expected_motor_count: int
    maximum_pilot_input_timeout_s: float
    expected_gcs_failsafe_action: int
    arm_ack_timeout_s: float
    axis_directions: Mapping[str, int]
    rc_override: RcOverrideConfig
    gripper: GripperConfig
    depth_message: str
    depth_field: str
    depth_multiplier: float
    depth_offset_m: float

    def __post_init__(self) -> None:
        """在任何硬件连接发生前检查完整配置和安全约束。"""

        required_axes = {"forward", "lateral", "vertical", "yaw"}
        if set(self.axis_directions) != required_axes:
            raise ConfigurationError(
                f"control.directions 必须且只能包含 {sorted(required_axes)}"
            )
        if any(direction not in (-1, 1) for direction in self.axis_directions.values()):
            raise ConfigurationError("运动轴方向只能是 -1 或 1")
        if not self.connection_uri.strip():
            raise ConfigurationError("MAVLink connection_uri 不能为空")
        positive_values = (
            self.command_timeout_s,
            self.heartbeat_timeout_s,
            self.heartbeat_stale_timeout_s,
            self.telemetry_stale_timeout_s,
            self.preflight_timeout_s,
            self.maximum_command_age_s,
            self.slew_rate_per_s,
            self.arm_ack_timeout_s,
            self.maximum_pilot_input_timeout_s,
        )
        if any(value <= 0 or not math.isfinite(value) for value in positive_values):
            raise ConfigurationError("超时时间和变化率必须是大于 0 的有限数值")
        if self.baud <= 0:
            raise ConfigurationError("MAVLink 串口波特率必须大于 0")
        if not 1 <= self.source_system <= 255:
            raise ConfigurationError("MAVLink source_system 必须在 1..255")
        if not 1 <= self.source_component <= 255:
            raise ConfigurationError("MAVLink source_component 必须在 1..255")
        if not self.allowed_flight_modes:
            raise ConfigurationError("allowed_flight_modes 不能为空")
        if any(mode != mode.upper() for mode in self.allowed_flight_modes):
            raise ConfigurationError("allowed_flight_modes 必须使用大写飞控模式名")
        if not 0.0 < self.command_limit <= 1.0:
            raise ConfigurationError("command_limit 必须在 (0, 1] 内")
        if self.expected_frame_config not in (None, 2, 3):
            raise ConfigurationError(
                "八推进器 expected_frame_config 只能是 2、3 或 null"
            )
        if self.expected_motor_count != 8:
            raise ConfigurationError("当前实艇配置必须明确 expected_motor_count: 8")
        if (
            self.allow_live_actuation or self.allow_ros_arming
        ) and self.expected_frame_config is None:
            raise ConfigurationError(
                "允许真实执行前必须明确 expected_frame_config 为 2 或 3"
            )
        if self.allow_ros_arming and not self.allow_live_actuation:
            raise ConfigurationError("allow_ros_arming=true 时必须同时允许真实执行")
        if self.allow_gripper_actuation and not self.allow_live_actuation:
            raise ConfigurationError("允许机械爪动作时必须同时允许真实执行")
        if self.allow_gripper_actuation and not self.gripper.calibrated:
            raise ConfigurationError(
                "允许机械爪动作前必须完成实机标定并设置 gripper.calibrated: true"
            )
        if (
            self.allow_gripper_actuation
            and self.gripper.uses_extended_pwm
            and not self.gripper.allow_extended_pwm
        ):
            raise ConfigurationError(
                "当前机械爪档案包含 800..2200 之外的历史 PWM；"
                "实测确认后还必须显式设置 allow_extended_pwm: true"
            )
        if self.expected_gcs_failsafe_action not in (1, 2, 3, 4):
            raise ConfigurationError("expected_gcs_failsafe_action 必须是 1..4")
        if self.gripper.steps_for("open") == self.gripper.steps_for("close"):
            raise ConfigurationError("机械爪开合 PWM 序列不能完全相同")
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", self.depth_message):
            raise ConfigurationError("depth.message 必须是 MAVLink 消息名")
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", self.depth_field):
            raise ConfigurationError("depth.field 必须是消息字段名")
        if not math.isfinite(self.depth_multiplier) or self.depth_multiplier == 0:
            raise ConfigurationError("depth.multiplier 必须是非零有限数值")
        if not math.isfinite(self.depth_offset_m):
            raise ConfigurationError("depth.offset_m 必须是有限数值")

    @property
    def allowed_command_source(self) -> str:
        """返回当前启动档案唯一允许的 ROS 命令来源。"""

        return self.control_profile.value


@dataclass(frozen=True)
class DatasetCollectionConfig:
    """水池数据采集的键盘、回收与链路安全参数。

    这些参数不包含电机通道或 PWM。键盘工具仍只发布四轴
    归一化运动意图，八推进器混控由 ArduSub 完成。
    """

    initial_command: float
    minimum_command: float
    maximum_command: float
    command_step: float
    publish_rate_hz: float

    recovery_gain: float
    recovery_max_command: float
    recovery_tolerance_m: float
    recovery_settle_s: float
    recovery_timeout_s: float

    maximum_telemetry_age_s: float
    maximum_status_age_s: float
    maximum_attitude_age_s: float
    allowed_flight_mode: str

    def __post_init__(self) -> None:
        """在连接实艇前拒绝不合法或会绕过限幅的参数。"""

        commands = (
            self.minimum_command,
            self.initial_command,
            self.maximum_command,
        )
        if any(not math.isfinite(value) or not 0.0 < value <= 1.0 for value in commands):
            raise ConfigurationError("键盘指令幅值必须在 (0, 1] 内")
        if not self.minimum_command <= self.initial_command <= self.maximum_command:
            raise ConfigurationError(
                "键盘初始指令必须位于最小与最大指令之间"
            )
        if not math.isfinite(self.command_step) or self.command_step <= 0.0:
            raise ConfigurationError("键盘指令调节步长必须大于 0")
        if not 5.0 <= self.publish_rate_hz <= 50.0:
            raise ConfigurationError("键盘控制发布频率必须在 5..50 Hz")

        recovery_values = (
            self.recovery_gain,
            self.recovery_max_command,
            self.recovery_tolerance_m,
            self.recovery_settle_s,
            self.recovery_timeout_s,
            self.maximum_telemetry_age_s,
            self.maximum_status_age_s,
            self.maximum_attitude_age_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in recovery_values):
            raise ConfigurationError("回收控制和数据新鲜度参数必须大于 0")
        if self.recovery_max_command > self.maximum_command:
            raise ConfigurationError("回收上升指令不能超过键盘最大指令")
        if self.allowed_flight_mode != "ALT_HOLD":
            raise ConfigurationError("数据采集工具只允许 ALT_HOLD 模式")

    def readiness_errors(self, robot: RobotConfig) -> tuple[str, ...]:
        """列出一键采集禁止解锁的全部配置原因。"""

        errors: list[str] = []
        if robot.control_profile != ControlProfile.COMMISSIONING:
            errors.append("robot.yaml 的 control.profile 必须是 commissioning")
        if self.allowed_flight_mode not in robot.allowed_flight_modes:
            errors.append("robot.yaml 的 allowed_flight_modes 必须包含 ALT_HOLD")
        if not robot.allow_live_actuation:
            errors.append("robot.yaml 尚未允许真实输出")
        if not robot.allow_ros_arming:
            errors.append("robot.yaml 尚未允许 ROS 解锁")
        if robot.allow_gripper_actuation:
            errors.append("数据采集时必须关闭机械爪权限")
        if self.maximum_command > robot.command_limit:
            errors.append(
                "dataset 最大指令超过 robot.yaml 的 control.command_limit"
            )
        return tuple(errors)


@dataclass(frozen=True)
class DetectorConfig:
    """YOLO 推理参数。"""

    model_path: Path
    expected_sha256: str
    confidence_threshold: float
    iou_threshold: float
    image_size: int
    device: str
    class_names: tuple[str, ...]


@dataclass(frozen=True)
class MissionConfig:
    """赛前自主抓取状态机参数。

    经常在水池调整的量位于 YAML 的 ``field_tuning``，其余高级安全限制位于
    ``mission``。这里使用平铺后的只读数据类，让状态机不必知道 YAML 结构。
    """

    descent_delta_m: float
    descent_tolerance_m: float
    advance_target_distance_m: float
    advance_estimated_speed_mps: float
    advance_forward_command: float
    grasp_aim_x_ratio: float
    grasp_aim_y_ratio: float
    image_yaw_sign: int | None
    image_vertical_sign: int | None
    approach_forward_command: float
    desired_approach_speed_mps: float
    grasp_area_ratios: Mapping[str, float]
    grasp_area_confirmation_frames: int
    recovery_depth_m: float
    horizontal_motion_confirmed: bool
    image_control_confirmed: bool
    grasp_thresholds_confirmed: bool

    allow_autonomous_mission: bool
    allow_open_loop_horizontal_motion: bool
    detection_confirmation_frames: int
    alignment_confirmation_frames: int
    target_lost_timeout_s: float
    horizontal_tolerance: float
    vertical_tolerance: float
    yaw_gain: float
    vertical_gain: float
    maximum_yaw_command: float
    maximum_vertical_command: float
    descent_gain: float
    descent_max_command: float
    descent_settle_s: float
    descent_timeout_s: float
    scan_angle_deg: float
    scan_direction: int
    scan_yaw_command: float
    scan_tolerance_deg: float
    scan_timeout_s: float
    maximum_yaw_step_deg: float
    yaw_progress_timeout_s: float
    maximum_search_cycles: int
    approach_alignment_multiplier: float
    approach_timeout_s: float
    maximum_grasp_area_ratio: float
    reacquire_grace_s: float
    reacquire_first_turn_deg: float
    reacquire_second_turn_deg: float
    reacquire_yaw_command: float
    reacquire_timeout_s: float
    gripper_hold_s: float
    gripper_command_timeout_s: float
    ascent_gain: float
    ascent_max_command: float
    ascent_tolerance_m: float
    ascent_settle_s: float
    ascent_timeout_s: float
    soft_mission_deadline_s: float
    hard_mission_timeout_s: float
    minimum_start_depth_m: float
    maximum_operation_depth_m: float
    maximum_heartbeat_age_s: float
    maximum_message_age_s: float
    perception_hold_timeout_s: float
    maximum_perception_age_s: float
    maximum_control_status_age_s: float
    allowed_flight_modes: tuple[str, ...]
    maximum_tracking_jump_ratio: float
    minimum_tracking_iou: float

    @property
    def advance_duration_s(self) -> float:
        """按目标距离和估计速度计算开环持续时间。"""

        return self.advance_target_distance_m / self.advance_estimated_speed_mps

    def readiness_errors(self, target_labels: tuple[str, ...]) -> tuple[str, ...]:
        """列出禁止真实自主启动的全部未授权或未标定项目。"""

        errors: list[str] = []
        if not self.allow_autonomous_mission:
            errors.append("mission.allow_autonomous_mission 尚未开启")
        if not self.allow_open_loop_horizontal_motion:
            errors.append("mission.allow_open_loop_horizontal_motion 尚未开启")
        if not self.horizontal_motion_confirmed:
            errors.append("field_tuning.calibration.horizontal_motion_confirmed 未确认")
        if not self.image_control_confirmed:
            errors.append("field_tuning.calibration.image_control_confirmed 未确认")
        if not self.grasp_thresholds_confirmed:
            errors.append("field_tuning.calibration.grasp_thresholds_confirmed 未确认")
        if self.image_yaw_sign is None:
            errors.append("field_tuning.image_yaw_sign 尚未标定")
        if self.image_vertical_sign is None:
            errors.append("field_tuning.image_vertical_sign 尚未标定")
        missing = sorted(set(target_labels) - set(self.grasp_area_ratios))
        if missing:
            errors.append(f"缺少类别抓取阈值: {', '.join(missing)}")
        return tuple(errors)


@dataclass(frozen=True)
class AutonomyConfig:
    """感知和任务状态机的组合配置。"""

    detector: DetectorConfig
    mission: MissionConfig


def _load_gripper_config(data: Mapping[str, Any]) -> GripperConfig:
    """读取可切换的机械爪档案，并兼容旧本地 YAML。

    新格式只加载 ``active_profile`` 指定的一份档案，网关运行中不能切换。
    旧格式仍可读取，以免队员的本地 ``robot.yaml`` 阻止遥测启动，但会被
    强制标记为未标定，不能绕过机械爪安全门。
    """

    def sweep(section: Mapping[str, Any]) -> GripperSweepConfig:
        """读取一条开爪或闭爪曲线。"""

        return GripperSweepConfig(
            start_pwm=int(section["start_pwm"]),
            end_pwm=int(section["end_pwm"]),
            step_pwm=int(section["step_pwm"]),
        )

    def output(section: Mapping[str, Any], name: str) -> GripperOutputConfig:
        """读取一个绝对 SERVO 输出及其开闭曲线。"""

        open_data = _mapping(section.get("open", {}), f"{name}.open")
        close_data = _mapping(section.get("close", {}), f"{name}.close")
        return GripperOutputConfig(
            output_channel=int(section["output_channel"]),
            open_sweep=sweep(open_data),
            close_sweep=sweep(close_data),
        )

    def profile(
        profile_name: str, section: Mapping[str, Any], name: str
    ) -> GripperConfig:
        """读取一个单路或多路机械爪档案。"""

        raw_outputs = section.get("outputs")
        if not isinstance(raw_outputs, list) or not raw_outputs:
            raise ConfigurationError(f"{name}.outputs 必须是非空列表")
        outputs = tuple(
            output(
                _mapping(item, f"{name}.outputs[{index}]"),
                f"{name}.outputs[{index}]",
            )
            for index, item in enumerate(raw_outputs)
        )
        return GripperConfig(
            profile=profile_name,
            step_interval_s=_finite(
                section.get("step_interval_s", 0.125), f"{name}.step_interval_s"
            ),
            calibrated=_boolean(
                section.get("calibrated", False), f"{name}.calibrated"
            ),
            allow_extended_pwm=_boolean(
                section.get("allow_extended_pwm", False),
                f"{name}.allow_extended_pwm",
            ),
            outputs=outputs,
        )

    if "profiles" in data:
        active_profile = str(data.get("active_profile", "")).strip()
        if not active_profile:
            raise ConfigurationError("gripper.active_profile 不能为空")
        profiles = _mapping(data["profiles"], "gripper.profiles")
        if active_profile not in profiles:
            raise ConfigurationError(
                f"gripper.active_profile={active_profile!r} 不存在于 profiles"
            )
        section = _mapping(
            profiles[active_profile], f"gripper.profiles.{active_profile}"
        )
        return profile(
            active_profile, section, f"gripper.profiles.{active_profile}"
        )

    # 兼容已经部署过的单输出渐变格式。
    output_channel = int(data["output_channel"])
    if "open" in data or "close" in data:
        single_output = output(data, "gripper")
        return GripperConfig(
            profile=str(data.get("profile", "dalian")).strip(),
            step_interval_s=_finite(
                data.get("step_interval_s", 0.125), "gripper.step_interval_s"
            ),
            calibrated=_boolean(
                data.get("calibrated", False), "gripper.calibrated"
            ),
            allow_extended_pwm=_boolean(
                data.get("allow_extended_pwm", False),
                "gripper.allow_extended_pwm",
            ),
            outputs=(single_output,),
        )

    # 更老的单次 PWM 格式只允许读取，不承认已经完成标定。
    open_pwm = int(data["open_pwm"])
    close_pwm = int(data["close_pwm"])
    return GripperConfig(
        profile="legacy",
        step_interval_s=0.125,
        calibrated=False,
        allow_extended_pwm=False,
        outputs=(
            GripperOutputConfig(
                output_channel=output_channel,
                open_sweep=GripperSweepConfig(open_pwm, open_pwm, 25),
                close_sweep=GripperSweepConfig(close_pwm, close_pwm, -25),
            ),
        ),
    )


def load_robot_config(path: str | Path) -> RobotConfig:
    """读取机器人配置。

    该函数不会连接飞控。只有配置全部通过检查后，调用方才可以创建 MAVLink
    连接，从而避免用半成品参数驱动真实推进器。
    """

    data = _read_yaml(path)
    mavlink = _mapping(data.get("mavlink", {}), "mavlink")
    control = _mapping(data.get("control", {}), "control")
    directions_data = _mapping(control.get("directions", {}), "control.directions")
    rc_override_data = _mapping(data.get("rc_override", {}), "rc_override")
    rc_channels_data = _mapping(
        rc_override_data.get("channels", {}), "rc_override.channels"
    )
    safety = _mapping(data.get("safety", {}), "safety")
    gripper = _mapping(data.get("gripper", {}), "gripper")
    depth = _mapping(data.get("depth", {}), "depth")

    try:
        try:
            protocol = ControlProtocol(str(control.get("protocol", "manual_control")))
        except ValueError as exc:
            raise ConfigurationError(
                "control.protocol 只能是 manual_control 或 rc_override"
            ) from exc
        try:
            profile = ControlProfile(
                str(
                    control.get(
                        "profile",
                        control.get("allowed_command_source", "commissioning"),
                    )
                ).strip()
            )
        except ValueError as exc:
            raise ConfigurationError(
                "control.profile 只能是 commissioning 或 autonomy"
            ) from exc
        frame_value = control.get("expected_frame_config")
        expected_frame_config = None if frame_value is None else int(frame_value)
        allowed_modes = tuple(
            mode.upper()
            for mode in _string_tuple(
                control.get("allowed_flight_modes", ["MANUAL"]),
                "control.allowed_flight_modes",
            )
        )
        rc_override = RcOverrideConfig(
            channels={name: int(value) for name, value in rc_channels_data.items()},
        )
        return RobotConfig(
            connection_uri=str(mavlink["connection_uri"]),
            heartbeat_timeout_s=float(mavlink.get("heartbeat_timeout_s", 10.0)),
            heartbeat_stale_timeout_s=float(
                mavlink.get("heartbeat_stale_timeout_s", 2.5)
            ),
            telemetry_stale_timeout_s=float(
                mavlink.get("telemetry_stale_timeout_s", 2.0)
            ),
            preflight_timeout_s=float(mavlink.get("preflight_timeout_s", 20.0)),
            baud=int(mavlink.get("baud", 115200)),
            source_system=int(mavlink.get("source_system", 255)),
            source_component=int(mavlink.get("source_component", 191)),
            control_protocol=protocol,
            control_profile=profile,
            allowed_flight_modes=allowed_modes,
            command_limit=_finite(control.get("command_limit", 0.10), "command_limit"),
            maximum_command_age_s=_finite(
                control.get("maximum_command_age_s", 0.50),
                "maximum_command_age_s",
            ),
            slew_rate_per_s=_finite(
                control.get("slew_rate_per_s", 0.5), "slew_rate_per_s"
            ),
            command_timeout_s=_finite(
                control.get("command_timeout_s", 0.5), "command_timeout_s"
            ),
            allow_live_actuation=_boolean(
                safety.get("allow_live_actuation", False),
                "safety.allow_live_actuation",
            ),
            allow_ros_arming=_boolean(
                safety.get("allow_ros_arming", False),
                "safety.allow_ros_arming",
            ),
            allow_gripper_actuation=_boolean(
                safety.get("allow_gripper_actuation", False),
                "safety.allow_gripper_actuation",
            ),
            expected_frame_config=expected_frame_config,
            expected_motor_count=int(control.get("expected_motor_count", 8)),
            maximum_pilot_input_timeout_s=_finite(
                safety.get("maximum_pilot_input_timeout_s", 3.0),
                "maximum_pilot_input_timeout_s",
            ),
            expected_gcs_failsafe_action=int(
                safety.get("expected_gcs_failsafe_action", 2)
            ),
            arm_ack_timeout_s=_finite(
                safety.get("arm_ack_timeout_s", 3.0), "arm_ack_timeout_s"
            ),
            axis_directions={
                name: int(value) for name, value in directions_data.items()
            },
            rc_override=rc_override,
            gripper=_load_gripper_config(gripper),
            depth_message=str(depth.get("message", "AHRS2")).upper(),
            depth_field=str(depth.get("field", "altitude")),
            depth_multiplier=_finite(depth.get("multiplier", -1.0), "depth.multiplier"),
            depth_offset_m=_finite(depth.get("offset_m", 0.0), "depth.offset_m"),
        )
    except KeyError as exc:
        raise ConfigurationError(f"机器人配置缺少字段: {exc.args[0]}") from exc


def load_dataset_config(path: str | Path) -> DatasetCollectionConfig:
    """读取键盘遥控和原始视频采集参数，不连接实艇。"""

    data = _read_yaml(path)
    manual = _mapping(data.get("manual_control", {}), "manual_control")
    recovery = _mapping(data.get("recovery", {}), "recovery")
    safety = _mapping(data.get("safety", {}), "safety")
    # 老版本模板把该值设得较小。图形线程短暂停顿后，ROS 的
    # telemetry/status 回调可能同时排队，过小阈值会在处理完第一个回调后
    # 误判第二个仍然过期。采集工具固定保证至少 3s 的桌面调度余量；
    # 网关自己的命令超时、飞控心跳和急停不由这个值控制。
    status_age_limit = max(
        3.0,
        _finite(
            safety.get("maximum_status_age_s", 3.0),
            "safety.maximum_status_age_s",
        ),
    )
    return DatasetCollectionConfig(
        initial_command=_finite(
            manual.get("initial_command", 0.05), "manual_control.initial_command"
        ),
        minimum_command=_finite(
            manual.get("minimum_command", 0.01), "manual_control.minimum_command"
        ),
        maximum_command=_finite(
            manual.get("maximum_command", 0.10), "manual_control.maximum_command"
        ),
        command_step=_finite(
            manual.get("command_step", 0.01), "manual_control.command_step"
        ),
        publish_rate_hz=_finite(
            manual.get("publish_rate_hz", 20.0), "manual_control.publish_rate_hz"
        ),
        recovery_gain=_finite(recovery.get("gain", 0.5), "recovery.gain"),
        recovery_max_command=_finite(
            recovery.get("maximum_command", 0.05), "recovery.maximum_command"
        ),
        recovery_tolerance_m=_finite(
            recovery.get("tolerance_m", 0.05), "recovery.tolerance_m"
        ),
        recovery_settle_s=_finite(
            recovery.get("settle_s", 1.0), "recovery.settle_s"
        ),
        recovery_timeout_s=_finite(
            recovery.get("timeout_s", 60.0), "recovery.timeout_s"
        ),
        maximum_telemetry_age_s=_finite(
            safety.get("maximum_telemetry_age_s", 1.50),
            "safety.maximum_telemetry_age_s",
        ),
        maximum_status_age_s=status_age_limit,
        maximum_attitude_age_s=_finite(
            safety.get("maximum_attitude_age_s", 5.0),
            "safety.maximum_attitude_age_s",
        ),
        allowed_flight_mode=str(
            safety.get("allowed_flight_mode", "ALT_HOLD")
        ).strip().upper(),
    )


def _unit_interval(value: Any, name: str, *, include_zero: bool = False) -> float:
    """读取 0..1 范围的小数配置。"""

    number = _finite(value, name)
    lower_ok = number >= 0.0 if include_zero else number > 0.0
    if not lower_ok or number > 1.0:
        raise ConfigurationError(
            f"{name} 必须在 {'[0, 1]' if include_zero else '(0, 1]'}"
        )
    return number


def load_autonomy_config(path: str | Path) -> AutonomyConfig:
    """读取模型及自主抓取配置，并解析相对于 YAML 的模型路径。"""

    config_path = Path(path).expanduser().resolve()
    data = _read_yaml(config_path)
    detector_data = _mapping(data.get("detector", {}), "detector")
    field_data = _mapping(data.get("field_tuning", {}), "field_tuning")
    calibration_data = _mapping(
        field_data.get("calibration", {}), "field_tuning.calibration"
    )
    grasp_area_data = _mapping(
        field_data.get("grasp_area_ratio", {}),
        "field_tuning.grasp_area_ratio",
    )
    mission_data = _mapping(data.get("mission", {}), "mission")
    try:
        model_path = Path(str(detector_data["model_path"])).expanduser()
        if not model_path.is_absolute():
            model_path = (config_path.parent / model_path).resolve()
        class_names = _string_tuple(
            detector_data["class_names"], "detector.class_names"
        )

        detector = DetectorConfig(
            model_path=model_path,
            expected_sha256=str(detector_data.get("expected_sha256", ""))
            .lower()
            .strip(),
            confidence_threshold=_unit_interval(
                detector_data.get("confidence_threshold", 0.45),
                "confidence_threshold",
            ),
            iou_threshold=_unit_interval(
                detector_data.get("iou_threshold", 0.50),
                "iou_threshold",
            ),
            image_size=int(detector_data.get("image_size", 640)),
            device=str(detector_data.get("device", "auto")),
            class_names=class_names,
        )
        scan_direction_name = str(
            mission_data.get("scan_direction", "right")
        ).strip().lower()
        if scan_direction_name not in {"left", "right"}:
            raise ConfigurationError("mission.scan_direction 只能是 left 或 right")
        grasp_area_ratios = {
            str(label).strip(): _unit_interval(
                value, f"field_tuning.grasp_area_ratio.{label}"
            )
            for label, value in grasp_area_data.items()
        }
        if not grasp_area_ratios or any(not label for label in grasp_area_ratios):
            raise ConfigurationError("field_tuning.grasp_area_ratio 不能为空")

        mission = MissionConfig(
            descent_delta_m=_finite(
                field_data.get("descent_delta_m", 0.30),
                "field_tuning.descent_delta_m",
            ),
            descent_tolerance_m=_finite(
                field_data.get("descent_tolerance_m", 0.05),
                "field_tuning.descent_tolerance_m",
            ),
            advance_target_distance_m=_finite(
                field_data.get("advance_target_distance_m", 0.40),
                "field_tuning.advance_target_distance_m",
            ),
            advance_estimated_speed_mps=_finite(
                field_data.get("advance_estimated_speed_mps", 0.30),
                "field_tuning.advance_estimated_speed_mps",
            ),
            advance_forward_command=_unit_interval(
                field_data.get("advance_forward_command", 0.05),
                "field_tuning.advance_forward_command",
            ),
            grasp_aim_x_ratio=_unit_interval(
                field_data.get("grasp_aim_x_ratio", 0.50),
                "field_tuning.grasp_aim_x_ratio",
                include_zero=True,
            ),
            grasp_aim_y_ratio=_unit_interval(
                field_data.get("grasp_aim_y_ratio", 0.70),
                "field_tuning.grasp_aim_y_ratio",
                include_zero=True,
            ),
            image_yaw_sign=_optional_sign(
                field_data.get("image_yaw_sign"), "field_tuning.image_yaw_sign"
            ),
            image_vertical_sign=_optional_sign(
                field_data.get("image_vertical_sign"),
                "field_tuning.image_vertical_sign",
            ),
            approach_forward_command=_unit_interval(
                field_data.get("approach_forward_command", 0.05),
                "field_tuning.approach_forward_command",
            ),
            desired_approach_speed_mps=_finite(
                field_data.get("desired_approach_speed_mps", 0.30),
                "field_tuning.desired_approach_speed_mps",
            ),
            grasp_area_ratios=grasp_area_ratios,
            grasp_area_confirmation_frames=int(
                field_data.get("grasp_area_confirmation_frames", 3)
            ),
            recovery_depth_m=_finite(
                field_data.get("recovery_depth_m", 0.30),
                "field_tuning.recovery_depth_m",
            ),
            horizontal_motion_confirmed=_boolean(
                calibration_data.get("horizontal_motion_confirmed", False),
                "field_tuning.calibration.horizontal_motion_confirmed",
            ),
            image_control_confirmed=_boolean(
                calibration_data.get("image_control_confirmed", False),
                "field_tuning.calibration.image_control_confirmed",
            ),
            grasp_thresholds_confirmed=_boolean(
                calibration_data.get("grasp_thresholds_confirmed", False),
                "field_tuning.calibration.grasp_thresholds_confirmed",
            ),
            allow_autonomous_mission=_boolean(
                mission_data.get("allow_autonomous_mission", False),
                "mission.allow_autonomous_mission",
            ),
            allow_open_loop_horizontal_motion=_boolean(
                mission_data.get("allow_open_loop_horizontal_motion", False),
                "mission.allow_open_loop_horizontal_motion",
            ),
            detection_confirmation_frames=int(
                mission_data.get("detection_confirmation_frames", 5)
            ),
            alignment_confirmation_frames=int(
                mission_data.get("alignment_confirmation_frames", 3)
            ),
            target_lost_timeout_s=_finite(
                mission_data.get("target_lost_timeout_s", 1.5),
                "mission.target_lost_timeout_s",
            ),
            horizontal_tolerance=_unit_interval(
                mission_data.get("horizontal_tolerance", 0.08),
                "mission.horizontal_tolerance",
            ),
            vertical_tolerance=_unit_interval(
                mission_data.get("vertical_tolerance", 0.10),
                "mission.vertical_tolerance",
            ),
            yaw_gain=_finite(mission_data.get("yaw_gain", 0.5), "mission.yaw_gain"),
            vertical_gain=_finite(
                mission_data.get("vertical_gain", 0.5), "mission.vertical_gain"
            ),
            maximum_yaw_command=_unit_interval(
                mission_data.get("maximum_yaw_command", 0.10),
                "mission.maximum_yaw_command",
            ),
            maximum_vertical_command=_unit_interval(
                mission_data.get("maximum_vertical_command", 0.10),
                "mission.maximum_vertical_command",
            ),
            descent_gain=_finite(
                mission_data.get("descent_gain", 0.5), "mission.descent_gain"
            ),
            descent_max_command=_unit_interval(
                mission_data.get("descent_max_command", 0.05),
                "mission.descent_max_command",
            ),
            descent_settle_s=_finite(
                mission_data.get("descent_settle_s", 1.0),
                "mission.descent_settle_s",
            ),
            descent_timeout_s=_finite(
                mission_data.get("descent_timeout_s", 45.0),
                "mission.descent_timeout_s",
            ),
            scan_angle_deg=_finite(
                mission_data.get("scan_angle_deg", 360.0), "mission.scan_angle_deg"
            ),
            scan_direction=1 if scan_direction_name == "right" else -1,
            scan_yaw_command=_unit_interval(
                mission_data.get("scan_yaw_command", 0.05),
                "mission.scan_yaw_command",
            ),
            scan_tolerance_deg=_finite(
                mission_data.get("scan_tolerance_deg", 3.0),
                "mission.scan_tolerance_deg",
            ),
            scan_timeout_s=_finite(
                mission_data.get("scan_timeout_s", 120.0),
                "mission.scan_timeout_s",
            ),
            maximum_yaw_step_deg=_finite(
                mission_data.get("maximum_yaw_step_deg", 45.0),
                "mission.maximum_yaw_step_deg",
            ),
            yaw_progress_timeout_s=_finite(
                mission_data.get("yaw_progress_timeout_s", 5.0),
                "mission.yaw_progress_timeout_s",
            ),
            maximum_search_cycles=int(mission_data.get("maximum_search_cycles", 3)),
            approach_alignment_multiplier=_finite(
                mission_data.get("approach_alignment_multiplier", 1.5),
                "mission.approach_alignment_multiplier",
            ),
            approach_timeout_s=_finite(
                mission_data.get("approach_timeout_s", 30.0),
                "mission.approach_timeout_s",
            ),
            maximum_grasp_area_ratio=_unit_interval(
                mission_data.get("maximum_grasp_area_ratio", 0.80),
                "mission.maximum_grasp_area_ratio",
            ),
            reacquire_grace_s=_finite(
                mission_data.get("reacquire_grace_s", 1.0),
                "mission.reacquire_grace_s",
            ),
            reacquire_first_turn_deg=_finite(
                mission_data.get("reacquire_first_turn_deg", 60.0),
                "mission.reacquire_first_turn_deg",
            ),
            reacquire_second_turn_deg=_finite(
                mission_data.get("reacquire_second_turn_deg", 120.0),
                "mission.reacquire_second_turn_deg",
            ),
            reacquire_yaw_command=_unit_interval(
                mission_data.get("reacquire_yaw_command", 0.05),
                "mission.reacquire_yaw_command",
            ),
            reacquire_timeout_s=_finite(
                mission_data.get("reacquire_timeout_s", 90.0),
                "mission.reacquire_timeout_s",
            ),
            gripper_hold_s=_finite(
                mission_data.get("gripper_hold_s", 1.5),
                "mission.gripper_hold_s",
            ),
            gripper_command_timeout_s=_finite(
                mission_data.get("gripper_command_timeout_s", 3.0),
                "mission.gripper_command_timeout_s",
            ),
            ascent_gain=_finite(
                mission_data.get("ascent_gain", 0.5), "mission.ascent_gain"
            ),
            ascent_max_command=_unit_interval(
                mission_data.get("ascent_max_command", 0.05),
                "mission.ascent_max_command",
            ),
            ascent_tolerance_m=_finite(
                mission_data.get("ascent_tolerance_m", 0.05),
                "mission.ascent_tolerance_m",
            ),
            ascent_settle_s=_finite(
                mission_data.get("ascent_settle_s", 1.0),
                "mission.ascent_settle_s",
            ),
            ascent_timeout_s=_finite(
                mission_data.get("ascent_timeout_s", 60.0),
                "mission.ascent_timeout_s",
            ),
            soft_mission_deadline_s=_finite(
                mission_data.get("soft_mission_deadline_s", 1680.0),
                "mission.soft_mission_deadline_s",
            ),
            hard_mission_timeout_s=_finite(
                mission_data.get("hard_mission_timeout_s", 1740.0),
                "mission.hard_mission_timeout_s",
            ),
            minimum_start_depth_m=_finite(
                mission_data.get("minimum_start_depth_m", 0.05),
                "mission.minimum_start_depth_m",
            ),
            maximum_operation_depth_m=_finite(
                mission_data.get("maximum_operation_depth_m", 20.0),
                "mission.maximum_operation_depth_m",
            ),
            maximum_heartbeat_age_s=_finite(
                mission_data.get("maximum_heartbeat_age_s", 2.5),
                "mission.maximum_heartbeat_age_s",
            ),
            maximum_message_age_s=_finite(
                mission_data.get("maximum_message_age_s", 1.5),
                "mission.maximum_message_age_s",
            ),
            perception_hold_timeout_s=_finite(
                mission_data.get("perception_hold_timeout_s", 1.0),
                "mission.perception_hold_timeout_s",
            ),
            maximum_perception_age_s=_finite(
                mission_data.get("maximum_perception_age_s", 5.0),
                "mission.maximum_perception_age_s",
            ),
            maximum_control_status_age_s=_finite(
                mission_data.get("maximum_control_status_age_s", 3.0),
                "mission.maximum_control_status_age_s",
            ),
            allowed_flight_modes=tuple(
                item.upper()
                for item in _string_tuple(
                    mission_data.get("allowed_flight_modes", ["ALT_HOLD"]),
                    "mission.allowed_flight_modes",
                )
            ),
            maximum_tracking_jump_ratio=_unit_interval(
                mission_data.get("maximum_tracking_jump_ratio", 0.25),
                "mission.maximum_tracking_jump_ratio",
            ),
            minimum_tracking_iou=_unit_interval(
                mission_data.get("minimum_tracking_iou", 0.05),
                "mission.minimum_tracking_iou",
                include_zero=True,
            ),
        )
    except KeyError as exc:
        raise ConfigurationError(f"自主配置缺少字段: {exc.args[0]}") from exc

    if detector.image_size <= 0:
        raise ConfigurationError("detector.image_size 必须大于 0")
    if detector.expected_sha256 and not re.fullmatch(
        r"[0-9a-f]{64}", detector.expected_sha256
    ):
        raise ConfigurationError("expected_sha256 必须是 64 位十六进制字符串")
    if min(
        mission.detection_confirmation_frames,
        mission.alignment_confirmation_frames,
        mission.grasp_area_confirmation_frames,
    ) < 1:
        raise ConfigurationError("连续确认帧数必须至少为 1")
    positive_distances = (
        mission.descent_delta_m,
        mission.descent_tolerance_m,
        mission.advance_target_distance_m,
        mission.advance_estimated_speed_mps,
        mission.desired_approach_speed_mps,
        mission.recovery_depth_m,
        mission.scan_angle_deg,
        mission.scan_tolerance_deg,
        mission.maximum_yaw_step_deg,
        mission.reacquire_first_turn_deg,
        mission.reacquire_second_turn_deg,
        mission.ascent_tolerance_m,
        mission.maximum_operation_depth_m,
    )
    positive_times = (
        mission.target_lost_timeout_s,
        mission.descent_settle_s,
        mission.descent_timeout_s,
        mission.scan_timeout_s,
        mission.yaw_progress_timeout_s,
        mission.approach_timeout_s,
        mission.reacquire_grace_s,
        mission.reacquire_timeout_s,
        mission.gripper_hold_s,
        mission.gripper_command_timeout_s,
        mission.ascent_settle_s,
        mission.ascent_timeout_s,
        mission.soft_mission_deadline_s,
        mission.hard_mission_timeout_s,
        mission.maximum_heartbeat_age_s,
        mission.maximum_message_age_s,
        mission.perception_hold_timeout_s,
        mission.maximum_perception_age_s,
        mission.maximum_control_status_age_s,
    )
    if min(*positive_distances, *positive_times) <= 0:
        raise ConfigurationError("任务时间参数必须大于 0")
    if mission.perception_hold_timeout_s >= mission.maximum_perception_age_s:
        raise ConfigurationError("感知回中等待时间必须小于感知硬中止时间")
    if min(
        mission.yaw_gain,
        mission.vertical_gain,
        mission.descent_gain,
        mission.ascent_gain,
        mission.approach_alignment_multiplier,
    ) <= 0:
        raise ConfigurationError("任务控制增益和倍数必须大于 0")
    if mission.maximum_search_cycles < 1:
        raise ConfigurationError("mission.maximum_search_cycles 必须至少为 1")
    if not 0.0 <= mission.minimum_start_depth_m < mission.maximum_operation_depth_m:
        raise ConfigurationError("自主运行深度范围不合法")
    if mission.recovery_depth_m >= mission.maximum_operation_depth_m:
        raise ConfigurationError("回收深度必须小于最大作业深度")
    if mission.scan_angle_deg > 360.0 or mission.scan_tolerance_deg >= mission.scan_angle_deg:
        raise ConfigurationError("扫描角度必须在容差之上且不超过 360°")
    if mission.reacquire_second_turn_deg <= mission.reacquire_first_turn_deg:
        raise ConfigurationError("重新捕获的第二段转角必须大于第一段")
    if mission.soft_mission_deadline_s >= mission.hard_mission_timeout_s:
        raise ConfigurationError("软截止时间必须早于硬超时")
    if mission.advance_duration_s > 60.0:
        raise ConfigurationError("单次开环前进时间不得超过 60 s")
    command_values = (
        mission.advance_forward_command,
        mission.approach_forward_command,
        mission.maximum_yaw_command,
        mission.maximum_vertical_command,
        mission.descent_max_command,
        mission.scan_yaw_command,
        mission.reacquire_yaw_command,
        mission.ascent_max_command,
    )
    if max(command_values) > 0.10:
        raise ConfigurationError("候选版任务指令不得超过 0.10 总限幅")
    if any(
        threshold >= mission.maximum_grasp_area_ratio
        for threshold in mission.grasp_area_ratios.values()
    ):
        raise ConfigurationError("各类抓取阈值必须小于异常大框上限")
    if not mission.allowed_flight_modes:
        raise ConfigurationError("allowed_flight_modes 不能为空")
    return AutonomyConfig(detector=detector, mission=mission)
