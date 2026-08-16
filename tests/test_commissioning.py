"""单轴与转向实机工具的纯软件安全测试。"""

import pytest

from rov_competition.commissioning import (
    CommissioningError,
    TurnProgressTracker,
    build_axis_test_plan,
    build_turn_test_plan,
)


def test_axis_preview_never_produces_motion() -> None:
    """不带 --execute 时，即使计划中有 0.05 也必须标记为不执行。"""

    plan = build_axis_test_plan(
        "forward", 0.05, 0.3, command_limit=0.10, execute=False
    )
    assert plan.produces_motion is False
    assert plan.motion.forward == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("action", "axis", "sign"),
    [
        ("forward", "forward", 1),
        ("backward", "forward", -1),
        ("left", "lateral", -1),
        ("right", "lateral", 1),
        ("up", "vertical", 1),
        ("down", "vertical", -1),
    ],
)
def test_six_axis_actions_map_to_exactly_one_axis(action: str, axis: str, sign: int) -> None:
    """每次测试只能产生一个非中位轴。"""

    plan = build_axis_test_plan(action, 0.05, 0.3, command_limit=0.10, execute=True)
    values = {
        "forward": plan.motion.forward,
        "lateral": plan.motion.lateral,
        "vertical": plan.motion.vertical,
        "yaw": plan.motion.yaw,
    }
    assert values[axis] == pytest.approx(sign * 0.05)
    assert sum(value != 0.0 for value in values.values()) == 1


def test_axis_value_and_duration_cannot_bypass_limits() -> None:
    """幅值和时长超界不会被静默截断成“看似可用”。"""

    with pytest.raises(CommissioningError):
        build_axis_test_plan("forward", 0.11, 0.3, command_limit=0.10, execute=True)
    with pytest.raises(CommissioningError):
        build_axis_test_plan("forward", 0.05, 5.1, command_limit=0.10, execute=True)


def test_turn_preview_and_angle_range() -> None:
    """转向默认只预览，且支持的边界为 1°..360°。"""

    preview = build_turn_test_plan(
        "right", 360.0, 0.05, command_limit=0.10, execute=False
    )
    assert preview.produces_motion is False
    assert preview.motion.yaw == pytest.approx(0.05)
    with pytest.raises(CommissioningError):
        build_turn_test_plan("right", 361.0, 0.05, command_limit=0.10, execute=True)


def test_turn_tracker_accumulates_right_turn_across_zero() -> None:
    """350° -> 10° -> 30° 应累计右转 40°。"""

    plan = build_turn_test_plan(
        "right", 40.0, 0.05, command_limit=0.10, execute=True
    )
    tracker = TurnProgressTracker(plan)
    tracker.update(350.0, 0.0)
    tracker.update(10.0, 0.1)
    assert tracker.update(30.0, 0.2) == pytest.approx(40.0)
    assert tracker.complete(tolerance_deg=1.0)


def test_turn_tracker_rejects_opposite_direction_jump_and_no_progress() -> None:
    """方向相反、航向跳变或长时间没有进展都必须中止。"""

    plan = build_turn_test_plan(
        "right", 90.0, 0.05, command_limit=0.10, execute=True
    )
    opposite = TurnProgressTracker(plan)
    opposite.update(100.0, 0.0)
    with pytest.raises(CommissioningError, match="相反"):
        opposite.update(95.0, 0.1)

    jump = TurnProgressTracker(plan, maximum_yaw_step_deg=45.0)
    jump.update(0.0, 0.0)
    with pytest.raises(CommissioningError, match="跳变"):
        jump.update(90.0, 0.1)

    stalled = TurnProgressTracker(plan, progress_timeout_s=1.0)
    stalled.update(0.0, 0.0)
    with pytest.raises(CommissioningError, match="无进展"):
        stalled.update(0.0, 1.1)
