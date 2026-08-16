"""实机单轴和转向测试共用的纯 Python 逻辑。

本模块不导入 ROS，因此“默认预览不产生动作”、六种单轴映射、
360° 跨零累计、反向与无进展判断都可在普通电脑上单元测试。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .domain import MotionCommand
from .mission import signed_yaw_delta_deg


class CommissioningError(ValueError):
    """测试参数或实时航向进展不安全。"""


AXIS_ACTIONS = ("forward", "backward", "left", "right", "up", "down")
TURN_DIRECTIONS = ("left", "right")
ACCEPTANCE_TURN_SEQUENCE_DEG = (30, 90, 180, 360)


@dataclass(frozen=True)
class AxisTestPlan:
    """一次单轴点动的已校验计划。"""

    action: str
    value: float
    duration_s: float
    execute: bool
    motion: MotionCommand

    @property
    def produces_motion(self) -> bool:
        """只有显式 ``--execute`` 的计划才会产生真实动作。"""

        return self.execute and not self.motion.is_neutral()


def build_axis_test_plan(
    action: str,
    value: float,
    duration_s: float,
    *,
    command_limit: float,
    execute: bool,
) -> AxisTestPlan:
    """校验并生成一次前/后/左/右/上/下点动计划。"""

    action = action.strip().lower()
    if action not in AXIS_ACTIONS:
        raise CommissioningError(f"动作必须是 {', '.join(AXIS_ACTIONS)} 之一")
    if not math.isfinite(value) or not 0.0 < value <= command_limit:
        raise CommissioningError(f"指令幅值必须在 (0, {command_limit:.3f}] 内")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= 5.0:
        raise CommissioningError("单次点动时间必须在 (0, 5.0] s 内")
    mapping = {
        "forward": MotionCommand(forward=value),
        "backward": MotionCommand(forward=-value),
        "left": MotionCommand(lateral=-value),
        "right": MotionCommand(lateral=value),
        "up": MotionCommand(vertical=value),
        "down": MotionCommand(vertical=-value),
    }
    return AxisTestPlan(
        action=action,
        value=value,
        duration_s=duration_s,
        execute=execute,
        motion=mapping[action],
    )


@dataclass(frozen=True)
class TurnTestPlan:
    """一次航向闭环转角测试的已校验计划。"""

    direction: str
    angle_deg: float
    command: float
    execute: bool

    @property
    def direction_sign(self) -> int:
        """右转为 +1，左转为 -1。"""

        return 1 if self.direction == "right" else -1

    @property
    def motion(self) -> MotionCommand:
        """返回该转向计划的归一化偏航意图。"""

        return MotionCommand(yaw=self.direction_sign * self.command)

    @property
    def produces_motion(self) -> bool:
        """默认预览计划始终不产生动作。"""

        return self.execute


def build_turn_test_plan(
    direction: str,
    angle_deg: float,
    command: float,
    *,
    command_limit: float,
    execute: bool,
) -> TurnTestPlan:
    """校验 1°..360° 单次转向测试。"""

    direction = direction.strip().lower()
    if direction not in TURN_DIRECTIONS:
        raise CommissioningError("转向只能是 left 或 right")
    if not math.isfinite(angle_deg) or not 1.0 <= angle_deg <= 360.0:
        raise CommissioningError("转角必须在 1°..360°")
    if not math.isfinite(command) or not 0.0 < command <= command_limit:
        raise CommissioningError(f"偏航指令必须在 (0, {command_limit:.3f}] 内")
    return TurnTestPlan(direction, angle_deg, command, execute)


class TurnProgressTracker:
    """使用真实航向跟踪一次转角。"""

    def __init__(
        self,
        plan: TurnTestPlan,
        *,
        maximum_yaw_step_deg: float = 45.0,
        progress_timeout_s: float = 5.0,
    ) -> None:
        """保存计划，并设置航向跳变与无进展上限。"""

        if maximum_yaw_step_deg <= 0.0 or progress_timeout_s <= 0.0:
            raise CommissioningError("转向跟踪安全上限必须大于 0")
        self.plan = plan
        self.maximum_yaw_step_deg = maximum_yaw_step_deg
        self.progress_timeout_s = progress_timeout_s
        self.progress_deg = 0.0
        self._last_yaw_deg: float | None = None
        self._last_progress_at: float | None = None

    def update(self, yaw_deg: float, now: float) -> float:
        """加入一个新航向样本并返回累计转角。

        航向跳变、显著反向或长时间无进展会抛出异常，调用方必须
        立即回中并上锁。
        """

        if not math.isfinite(yaw_deg) or not math.isfinite(now):
            raise CommissioningError("航向和时间必须是有限数")
        if self._last_yaw_deg is None:
            self._last_yaw_deg = yaw_deg
            self._last_progress_at = now
            return self.progress_deg
        raw_delta = signed_yaw_delta_deg(self._last_yaw_deg, yaw_deg)
        self._last_yaw_deg = yaw_deg
        if abs(raw_delta) > self.maximum_yaw_step_deg:
            raise CommissioningError(f"航向跳变 {raw_delta:+.1f}°")
        signed_progress = raw_delta * self.plan.direction_sign
        if signed_progress < -2.0:
            raise CommissioningError("实际航向与命令方向相反")
        if signed_progress > 0.05:
            self.progress_deg += signed_progress
            self._last_progress_at = now
        if self._last_progress_at is not None:
            if now - self._last_progress_at >= self.progress_timeout_s:
                raise CommissioningError("偏航命令发出后航向长时间无进展")
        return self.progress_deg

    def complete(self, tolerance_deg: float = 2.0) -> bool:
        """进度进入目标角容差时返回真。"""

        if not 0.0 <= tolerance_deg < self.plan.angle_deg:
            raise CommissioningError("转角容差不合法")
        return self.progress_deg >= self.plan.angle_deg - tolerance_deg
