"""与 ROS 运行时无关的控制命令边界检查。"""

from __future__ import annotations

import math
from collections.abc import Iterable


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
    """验证命令来源、数值范围和端到端时间戳。

    返回 ``None`` 表示可以继续执行；否则返回可直接写入 ROS 状态
    和日志的拒绝原因。函数不会对超范围数据“猜测修复”。
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
