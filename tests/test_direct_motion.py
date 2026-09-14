import pytest

from rov_competition.commissioning import CommissioningError
from rov_competition.direct_motion import build_direct_motion_plan, format_measurement_result


@pytest.mark.parametrize(
    ("action", "axis", "expected"),
    [
        ("forward", "forward", .1),
        ("backward", "forward", -.1),
        ("left", "lateral", -.1),
        ("right", "lateral", .1),
        ("up", "vertical", .1),
        ("down", "vertical", -.1),
        ("yaw_left", "yaw", -.1),
        ("yaw_right", "yaw", .1),
    ],
)
def test_direct_motion_maps_one_requested_axis(action, axis, expected):
    plan = build_direct_motion_plan(action, .1, command_limit=.8)
    values = {
        "forward": plan.motion.forward,
        "lateral": plan.motion.lateral,
        "vertical": plan.motion.vertical,
        "yaw": plan.motion.yaw,
    }
    assert values[axis] == pytest.approx(expected)
    assert sum(value != 0 for value in values.values()) == 1


@pytest.mark.parametrize("action,value", [("spin", .1), ("forward", 0), ("forward", .81)])
def test_direct_motion_rejects_invalid_input(action, value):
    with pytest.raises(CommissioningError):
        build_direct_motion_plan(action, value, command_limit=.8)


def test_measurement_result_is_copyable_and_keeps_the_entered_values():
    result = format_measurement_result(
        action="forward", value=.08, held_s=1.234, observed_effect="平稳",
    )
    assert "动作：forward" in result
    assert "推进力：0.0800" in result
    assert "实际保持：1.23 s" in result
    assert "观察结果：平稳" in result
