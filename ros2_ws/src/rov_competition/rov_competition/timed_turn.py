"""与 ROS 无关的固定时长偏航点动计划。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .commissioning import CommissioningError
from .domain import MotionCommand


@dataclass(frozen=True)
class TimedTurnPlan:
    direction: str
    value: float
    duration_s: float
    execute: bool

    @property
    def motion(self) -> MotionCommand:
        return MotionCommand(yaw=-self.value if self.direction == "left" else self.value)


def build_timed_turn_plan(
    direction: str, value: float, duration_s: float, *, command_limit: float, execute: bool
) -> TimedTurnPlan:
    direction = direction.strip().lower()
    if direction not in {"left", "right"}:
        raise CommissioningError("方向必须是 left 或 right")
    if not math.isfinite(value) or not 0.0 < value <= command_limit:
        raise CommissioningError(f"推力必须在 (0, {command_limit:.3f}] 内")
    if not math.isfinite(duration_s) or not 0.0 < duration_s <= 5.0:
        raise CommissioningError("单次偏航时长必须在 (0, 5.0] s 内")
    return TimedTurnPlan(direction, value, duration_s, execute)
