"""触底平台判定和离底稳定的共用纯 Python 控制器。

压力深度是相对水面深度，不是离底高度。这里只把“持续下潜后
深度在容差内稳定”当作疑似触底，再上浮一个相对深度差。
"""

from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class BottomProbeResult:
    """一次触底分析结果。"""

    descended_m: float
    enough_descent: bool
    stable_duration_s: float
    depth_span_m: float
    hold_neutral: bool
    confirmed: bool
    bottom_depth_m: float | None


def update_bottom_probe(
    samples: deque[tuple[float, float]],
    *,
    now: float,
    current_depth_m: float,
    probe_start_depth_m: float,
    stable_duration_required_s: float,
    stable_depth_tolerance_m: float,
    minimum_descent_m: float,
    neutral_confirmation_s: float,
) -> BottomProbeResult:
    """追加深度样本，并分析最长稳定后缀。

    比较整个时间窗的最大/最小深度，不是只比较相邻样本；
    因而慢速下潜不会因“每次变化小”而被误判成触底。
    """

    samples.append((float(now), float(current_depth_m)))
    cutoff = now - stable_duration_required_s
    while len(samples) >= 2 and samples[1][0] <= cutoff:
        samples.popleft()

    stable_samples: list[tuple[float, float]] = []
    stable_min = stable_max = float(current_depth_m)
    for sample_time, depth_m in reversed(samples):
        candidate_min = min(stable_min, depth_m)
        candidate_max = max(stable_max, depth_m)
        if candidate_max - candidate_min > stable_depth_tolerance_m + 1e-6:
            break
        stable_samples.append((sample_time, depth_m))
        stable_min, stable_max = candidate_min, candidate_max
    stable_samples.reverse()

    stable_duration_s = (
        max(0.0, now - stable_samples[0][0]) if stable_samples else 0.0
    )
    descended_m = current_depth_m - probe_start_depth_m
    enough_descent = descended_m + 1e-6 >= minimum_descent_m
    effective_stable_s = stable_duration_s if enough_descent else 0.0
    confirmed = enough_descent and stable_duration_s >= stable_duration_required_s
    bottom_depth_m = (
        float(statistics.median(depth for _, depth in stable_samples))
        if confirmed and stable_samples
        else None
    )
    return BottomProbeResult(
        descended_m=descended_m,
        enough_descent=enough_descent,
        stable_duration_s=effective_stable_s,
        depth_span_m=stable_max - stable_min,
        hold_neutral=(
            enough_descent and stable_duration_s >= neutral_confirmation_s
        ),
        confirmed=confirmed,
        bottom_depth_m=bottom_depth_m,
    )


@dataclass(frozen=True)
class BottomClearanceResult:
    """离底阶段的指令和更新后稳定状态。"""

    vertical_command: float
    reached_target: bool
    settled: bool
    unsafe_depth_increase: bool
    settled_since: float | None
    reference_depth_m: float | None


def update_bottom_clearance(
    *,
    now: float,
    current_depth_m: float,
    bottom_depth_m: float,
    target_depth_m: float,
    bottom_depth_tolerance_m: float,
    clearance_tolerance_m: float,
    settle_duration_s: float,
    up_command: float,
    settled_since: float | None,
    reference_depth_m: float | None,
) -> BottomClearanceResult:
    """只允许上浮或回中的离底控制。

    即使上浮超调到目标之上，也不会为追准压力深度而再次下潜。
    """

    unsafe = (
        current_depth_m - bottom_depth_m > bottom_depth_tolerance_m + 1e-6
    )
    if unsafe:
        return BottomClearanceResult(
            vertical_command=0.0,
            reached_target=False,
            settled=False,
            unsafe_depth_increase=True,
            settled_since=None,
            reference_depth_m=None,
        )

    reached = current_depth_m - target_depth_m <= clearance_tolerance_m
    if not reached:
        return BottomClearanceResult(
            vertical_command=up_command,
            reached_target=False,
            settled=False,
            unsafe_depth_increase=False,
            settled_since=None,
            reference_depth_m=None,
        )

    if (
        settled_since is None
        or reference_depth_m is None
        or abs(current_depth_m - reference_depth_m) > clearance_tolerance_m
    ):
        settled_since = float(now)
        reference_depth_m = float(current_depth_m)
    return BottomClearanceResult(
        vertical_command=0.0,
        reached_target=True,
        settled=now - settled_since >= settle_duration_s,
        unsafe_depth_increase=False,
        settled_since=settled_since,
        reference_depth_m=reference_depth_m,
    )
