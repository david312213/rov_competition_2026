import pytest

from rov_competition.commissioning import CommissioningError
from rov_competition.timed_turn import build_timed_turn_plan


def test_timed_turn_uses_fixed_direction_and_duration():
    left = build_timed_turn_plan("left", .1, .4, command_limit=.8, execute=True)
    right = build_timed_turn_plan("right", .1, .4, command_limit=.8, execute=True)
    assert left.motion.yaw == -.1
    assert right.motion.yaw == .1
    assert left.duration_s == .4


@pytest.mark.parametrize("direction,value,duration", [
    ("up", .1, .3), ("left", 0, .3), ("right", .9, .3), ("left", .1, 0),
])
def test_timed_turn_rejects_invalid_inputs(direction, value, duration):
    with pytest.raises(CommissioningError):
        build_timed_turn_plan(direction, value, duration, command_limit=.8, execute=True)
