"""手动保持式单轴运动的纯 Python 参数校验。"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .commissioning import CommissioningError
from .domain import MotionCommand


DIRECT_ACTIONS = (
    "forward", "backward", "left", "right", "up", "down", "yaw_left", "yaw_right",
)


@dataclass(frozen=True)
class DirectMotionPlan:
    """一项由操作员手动开始和停止的单轴运动。"""

    action: str
    value: float
    motion: MotionCommand


def build_direct_motion_plan(
    action: str, value: float, *, command_limit: float,
) -> DirectMotionPlan:
    """校验输入，且保证一次只会驱动一个轴。"""

    action = action.strip().lower()
    if action not in DIRECT_ACTIONS:
        raise CommissioningError(f"动作必须是 {', '.join(DIRECT_ACTIONS)} 之一")
    if not math.isfinite(value) or not 0.0 < value <= command_limit:
        raise CommissioningError(f"推进力必须在 (0, {command_limit:.3f}] 内")
    mapping = {
        "forward": MotionCommand(forward=value),
        "backward": MotionCommand(forward=-value),
        "left": MotionCommand(lateral=-value),
        "right": MotionCommand(lateral=value),
        "up": MotionCommand(vertical=value),
        "down": MotionCommand(vertical=-value),
        "yaw_left": MotionCommand(yaw=-value),
        "yaw_right": MotionCommand(yaw=value),
    }
    return DirectMotionPlan(action=action, value=value, motion=mapping[action])


def format_measurement_result(
    *, action: str, value: float, held_s: float, observed_effect: str,
    stop_reason: str | None = None,
) -> str:
    """生成便于复制到沟通窗口的单条实艇结果。"""

    effect = observed_effect or "未填写"
    rows = [
        "--- ROV 推进力记录 ---",
        f"动作：{action}",
        f"推进力：{value:.4f}",
        f"实际保持：{held_s:.2f} s",
    ]
    if stop_reason:
        rows.append(f"停止原因：{stop_reason}")
    rows.extend((
        f"观察结果：{effect}",
        "--- 记录结束 ---",
    ))
    return "\n".join(rows)
