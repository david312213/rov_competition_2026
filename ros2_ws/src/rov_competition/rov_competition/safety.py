"""控制网关和自主任务共用的纯软件安全检查。"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

from .config import MissionConfig


@dataclass(frozen=True)
class AutonomyTelemetryStatus:
    """自主任务所需的最小实时遥测状态。"""

    heartbeat_valid: bool
    heartbeat_age_s: float
    message_age_s: float
    armed: bool
    depth_valid: bool
    depth_m: float
    attitude_valid: bool
    yaw_deg: float
    flight_mode: str
    received_monotonic: float = 0.0


@dataclass(frozen=True)
class AutonomyControlStatus:
    """自主节点所需的网关门控快照，不依赖 ROS 消息类。"""

    state: str
    command_source: str
    configuration_allows_actuation: bool
    configuration_allows_ros_arming: bool
    configuration_allows_gripper: bool
    runtime_enabled: bool
    preflight_passed: bool
    emergency_stop_latched: bool
    armed_by_ros: bool
    armed: bool
    flight_mode: str
    received_monotonic: float


def is_fresh_telemetry(timestamp: float | None, now: float, timeout_s: float) -> bool:
    """判断一组真实遥测是否仍在允许的数据年龄内。"""

    return timestamp is not None and 0.0 <= now - timestamp <= timeout_s


def validate_command_envelope(
    *,
    source: str,
    expected_source: str,
    values: Iterable[float],
    stamp_s: float,
    now_s: float,
    maximum_age_s: float,
    future_tolerance_s: float = 0.10,
) -> str | None:
    """验证运动命令的来源、数值范围和端到端时间戳。

    返回 ``None`` 表示命令可以继续执行；否则返回可直接写入状态话题
    和日志的拒绝原因。这里拒绝异常命令，不对数据做猜测性修复。
    """

    if source.strip() != expected_source:
        return f"非法命令来源 {source!r}；只允许 {expected_source!r}"
    numeric_values = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in numeric_values):
        return "运动命令包含 NaN/Inf"
    if any(abs(value) > 1.0 for value in numeric_values):
        return "归一化运动命令必须位于 [-1, 1]"
    if not math.isfinite(stamp_s) or stamp_s <= 0.0:
        return "运动命令缺少有效时间戳"
    if not math.isfinite(now_s):
        return "控制网关当前时间无效"
    age_s = now_s - stamp_s
    if age_s < -future_tolerance_s:
        return f"运动命令时间戳超前 {-age_s:.3f}s"
    if age_s > maximum_age_s:
        return f"运动命令已过期 {age_s:.3f}s，上限 {maximum_age_s:.3f}s"
    return None


def autonomy_safety_error(
    status: AutonomyTelemetryStatus | None,
    config: MissionConfig,
    *,
    now: float | None = None,
) -> str | None:
    """返回阻止自主运动的原因；全部条件满足时返回 ``None``。

    检查顺序刻意从通信链路到飞控状态、再到入水深度。这样现场日志首先显示最
    根本的故障，而不是在数据已经过期时继续判断旧深度或旧姿态。
    """

    if status is None:
        return "尚未收到真实飞控遥测"
    if not config.allow_autonomous_mission:
        return "真实自主任务尚未显式授权"
    if not config.allow_open_loop_horizontal_motion:
        return "无 DVL 开环水平运动尚未实测并授权"
    local_age = 0.0 if now is None else max(0.0, now - status.received_monotonic)
    heartbeat_age = status.heartbeat_age_s + local_age
    message_age = status.message_age_s + local_age
    if not all(math.isfinite(value) for value in (heartbeat_age, message_age, status.depth_m)):
        return "飞控遥测包含 NaN/Inf"
    if not status.heartbeat_valid:
        return "飞控心跳无效"
    if heartbeat_age > config.maximum_heartbeat_age_s:
        return f"飞控心跳过期: {heartbeat_age:.2f}s"
    if message_age > config.maximum_message_age_s:
        return f"飞控遥测过期: {message_age:.2f}s"
    if not status.armed:
        return "飞控尚未解锁"
    if status.flight_mode.upper() not in config.allowed_flight_modes:
        return (
            f"飞控模式 {status.flight_mode or 'UNKNOWN'} 不允许自主运行；"
            f"允许模式: {', '.join(config.allowed_flight_modes)}"
        )
    if not status.attitude_valid:
        return "姿态遥测无效"
    if not status.depth_valid:
        return "深度遥测无效"
    if not config.minimum_start_depth_m <= status.depth_m <= config.maximum_operation_depth_m:
        return (
            f"深度 {status.depth_m:.2f}m 超出自主运行范围 "
            f"[{config.minimum_start_depth_m:.2f}, "
            f"{config.maximum_operation_depth_m:.2f}]m"
        )
    return None


def autonomy_control_error(
    status: AutonomyControlStatus | None,
    config: MissionConfig,
    *,
    command_source: str,
    now: float,
) -> str | None:
    """返回网关门控不允许自主运动的第一个原因。"""

    if status is None:
        return "尚未收到 /rov/control/status"
    age = now - status.received_monotonic
    if not math.isfinite(age) or age < 0.0 or age > config.maximum_control_status_age_s:
        return f"控制状态过期: {age:.2f}s"
    if not status.configuration_allows_actuation:
        return "网关真实运动输出未授权"
    if not status.configuration_allows_ros_arming:
        return "网关 ROS 解锁未授权"
    if not status.configuration_allows_gripper:
        return "网关机械爪输出未独立授权"
    if not status.preflight_passed:
        return "网关只读预检未通过"
    if not status.runtime_enabled:
        return "网关运行时控制许可未开启"
    if status.emergency_stop_latched or status.state == "ESTOPPED":
        return "网关急停已锁定"
    if not status.armed or not status.armed_by_ros:
        return "飞控尚未经 ROS 确认词正常解锁"
    if status.state not in {"READY", "ACTIVE"}:
        return f"网关状态 {status.state or 'UNKNOWN'} 不允许任务"
    if status.flight_mode.upper() not in config.allowed_flight_modes:
        return (
            f"飞控模式 {status.flight_mode or 'UNKNOWN'} 不允许自主运行"
        )
    if status.command_source and status.command_source != command_source:
        return f"网关当前命令来源异常: {status.command_source!r}"
    return None
