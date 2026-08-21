"""数据集键盘遥控的纯 Python 逻辑。

本模块不导入 ROS、Pygame 或 MAVLink，因此可以在不连实艇的
电脑上测试按键映射、组合限幅、链路状态和可选回收上升。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

from .config import DatasetCollectionConfig
from .domain import MotionCommand


class DatasetControlError(RuntimeError):
    """键盘控制或回收无法继续时的明确异常。"""


MOVEMENT_KEYS = frozenset({"w", "s", "a", "d", "1", "2", "up", "down"})


@dataclass(frozen=True)
class DatasetRuntimeSnapshot:
    """一个控制周期所需的网关与遥测事实。

    使用普通 Python 值而不是 ROS 消息，便于用假网关精确
    测试过期、错误模式、来源冲突和解锁拒绝。
    """

    telemetry_age_s: float
    status_age_s: float
    heartbeat_valid: bool
    attitude_valid: bool
    attitude_age_s: float
    telemetry_mode: str
    status_mode: str
    preflight_passed: bool
    estop_latched: bool
    state: str
    allows_actuation: bool
    allows_arming: bool
    allows_gripper: bool
    runtime_enabled: bool
    telemetry_armed: bool
    status_armed: bool
    armed_by_ros: bool
    command_source: str


def _runtime_common_error(
    snapshot: DatasetRuntimeSnapshot,
    config: DatasetCollectionConfig,
    *,
    source: str,
) -> str | None:
    """返回预解锁和活动控制共用的安全拒绝原因。"""

    if (
        not math.isfinite(snapshot.telemetry_age_s)
        or snapshot.telemetry_age_s > config.maximum_telemetry_age_s
    ):
        return "遥测数据过期"
    if (
        not math.isfinite(snapshot.status_age_s)
        or snapshot.status_age_s > config.maximum_status_age_s
    ):
        return "控制状态过期"
    if not snapshot.heartbeat_valid:
        return "飞控心跳无效"
    if not snapshot.attitude_valid:
        if not math.isfinite(snapshot.attitude_age_s):
            return "尚未收到过有效姿态遥测"
        if snapshot.attitude_age_s > config.maximum_attitude_age_s:
            return (
                "姿态遥测连续无效 "
                f"{snapshot.attitude_age_s:.2f}s，超过允许的 "
                f"{config.maximum_attitude_age_s:.2f}s"
            )
    expected = config.allowed_flight_mode
    if snapshot.telemetry_mode.upper() != expected:
        return f"飞控必须保持 {expected}"
    if snapshot.status_mode.upper() != expected:
        return "遥测与控制状态中的飞控模式不一致"
    if not snapshot.preflight_passed:
        return "飞控只读预检未通过"
    if snapshot.estop_latched or snapshot.state == "ESTOPPED":
        return "急停已锁存"
    if not snapshot.allows_actuation:
        return "本次网关未允许真实输出"
    if not snapshot.allows_arming:
        return "本次网关未允许 ROS 解锁"
    if snapshot.allows_gripper:
        return "数据采集期间机械爪权限必须关闭"
    if snapshot.command_source and snapshot.command_source != source:
        return f"网关当前命令来源不是 {source!r}"
    if snapshot.telemetry_armed != snapshot.status_armed:
        return "遥测与控制状态中的解锁信息不一致"
    return None


def prearm_safety_error(
    snapshot: DatasetRuntimeSnapshot,
    config: DatasetCollectionConfig,
    *,
    source: str = "commissioning",
) -> str | None:
    """预解锁假网关检查；完全安全时返回空。"""

    error = _runtime_common_error(snapshot, config, source=source)
    if error is not None:
        return error
    if snapshot.runtime_enabled:
        return "检测到上次遗留的运行时许可"
    if snapshot.state != "LOCKED":
        return f"解锁前网关必须为 LOCKED，当前为 {snapshot.state}"
    if snapshot.telemetry_armed or snapshot.status_armed:
        return "开启许可前飞控必须明确上锁"
    if snapshot.armed_by_ros:
        return "解锁来源状态未清空"
    return None


def active_safety_error(
    snapshot: DatasetRuntimeSnapshot,
    config: DatasetCollectionConfig,
    *,
    source: str = "commissioning",
) -> str | None:
    """活动键盘控制检查；任一安全事实改变就返回原因。"""

    error = _runtime_common_error(snapshot, config, source=source)
    if error is not None:
        return error
    if not snapshot.runtime_enabled:
        return "运行时控制许可已关闭"
    if not snapshot.telemetry_armed or not snapshot.status_armed:
        return "飞控已上锁或解锁未确认"
    if not snapshot.armed_by_ros:
        return "飞控未经 ROS 确认词解锁"
    if snapshot.state not in {"READY", "ACTIVE"}:
        return f"网关状态不允许键盘控制: {snapshot.state}"
    return None


def _clamp(value: float, limit: float) -> float:
    """把有限数限制到对称区间。"""

    if not math.isfinite(value) or not math.isfinite(limit) or limit <= 0.0:
        raise DatasetControlError("控制量和限幅必须是有限正数")
    return max(-limit, min(limit, value))


def motion_from_keys(
    keys: Iterable[str], strength: float, combined_limit: float
) -> MotionCommand:
    """将当前按住的键转换为四轴指令。

    相反键会自然抵消。每个轴独立限幅，因此“下潜+转向”
    不会再被平分成两个过小指令；八推进器的组合和饱和
    仍由 ArduSub 控制分配。
    """

    if not math.isfinite(strength) or strength <= 0.0:
        raise DatasetControlError("键盘指令幅值必须大于 0")
    if not math.isfinite(combined_limit) or combined_limit <= 0.0:
        raise DatasetControlError("组合指令限幅必须大于 0")

    held = {str(key).lower() for key in keys}
    values = [
        strength * (("w" in held) - ("s" in held)),
        strength * (("d" in held) - ("a" in held)),
        strength * (("up" in held) - ("down" in held)),
        strength * (("2" in held) - ("1" in held)),
    ]
    values = [_clamp(value, combined_limit) for value in values]
    return MotionCommand(
        forward=values[0],
        lateral=values[1],
        vertical=values[2],
        yaw=values[3],
    )


@dataclass
class KeyCommandState:
    """保存当前按键和可调节的低功率指令。"""

    config: DatasetCollectionConfig
    held_keys: set[str] = field(default_factory=set)
    strength: float = field(init=False)

    def __post_init__(self) -> None:
        self.strength = self.config.initial_command

    def press(self, key: str) -> bool:
        """记录一个运动键；新增按键时返回真。"""

        value = key.lower()
        if value not in MOVEMENT_KEYS:
            return False
        before = len(self.held_keys)
        self.held_keys.add(value)
        return len(self.held_keys) != before

    def release(self, key: str) -> bool:
        """释放一个运动键；该键原先被按住时返回真。"""

        value = key.lower()
        if value not in self.held_keys:
            return False
        self.held_keys.remove(value)
        return True

    def clear(self) -> None:
        """立即清空所有按键，用于松键、失去焦点和停车。"""

        self.held_keys.clear()

    def adjust(self, direction: int) -> float:
        """按配置步长增大或减小指令幅值。"""

        if direction not in (-1, 1):
            raise DatasetControlError("指令调节方向只能是 -1 或 1")
        candidate = self.strength + direction * self.config.command_step
        candidate = max(
            self.config.minimum_command,
            min(self.config.maximum_command, candidate),
        )
        # 限制浮点累积误差，使界面和 CSV 中的值稳定可读。
        self.strength = round(candidate, 6)
        return self.strength

    def motion(self) -> MotionCommand:
        """返回当前按键对应的组合运动意图。"""

        return motion_from_keys(
            self.held_keys,
            self.strength,
            self.config.maximum_command,
        )


@dataclass(frozen=True)
class RecoveryStep:
    """按 0 后一个回收周期的输出。"""

    motion: MotionCommand
    complete: bool
    message: str


class RecoveryController:
    """只允许上升回启动深度的闭环回收器。"""

    def __init__(
        self,
        config: DatasetCollectionConfig,
        target_depth_m: float,
        started_at: float,
    ) -> None:
        if not math.isfinite(target_depth_m) or not math.isfinite(started_at):
            raise DatasetControlError("回收目标深度和时间必须是有限数")
        self.config = config
        self.target_depth_m = target_depth_m
        self.started_at = started_at
        self.settled_since: float | None = None

    def step(self, *, depth_valid: bool, depth_m: float, now: float) -> RecoveryStep:
        """使用真实深度生成一次上升或回中指令。"""

        if not depth_valid or not math.isfinite(depth_m) or not math.isfinite(now):
            raise DatasetControlError("回收时深度或时间无效")
        if now - self.started_at >= self.config.recovery_timeout_s:
            raise DatasetControlError("上升回收超时")

        error_m = depth_m - self.target_depth_m
        # 当前已比启动深度浅时也只回中，不为追深度重新下潜。
        if error_m <= self.config.recovery_tolerance_m:
            if self.settled_since is None:
                self.settled_since = now
            complete = now - self.settled_since >= self.config.recovery_settle_s
            return RecoveryStep(
                MotionCommand.neutral(),
                complete,
                "已回到启动深度，等待稳定" if not complete else "回收深度已稳定",
            )

        self.settled_since = None
        vertical = _clamp(
            error_m * self.config.recovery_gain,
            self.config.recovery_max_command,
        )
        return RecoveryStep(
            MotionCommand(vertical=max(0.0, vertical)),
            False,
            f"上升回收：{depth_m:.2f} -> {self.target_depth_m:.2f} m",
        )
