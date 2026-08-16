"""控制命令来源、时间戳和数值边界测试。"""

import math

from rov_competition.control_safety import validate_command_envelope


def validate(**overrides):
    """使用一组合法默认值执行命令封套检查。"""

    arguments = {
        "source": "commissioning",
        "expected_source": "commissioning",
        "values": (0.05, 0.0, 0.0, 0.0),
        "stamp_s": 100.0,
        "now_s": 100.1,
        "maximum_age_s": 0.25,
    }
    arguments.update(overrides)
    return validate_command_envelope(**arguments)


def test_fresh_expected_source_is_accepted() -> None:
    assert validate() is None


def test_wrong_source_is_rejected() -> None:
    assert "非法命令来源" in validate(source="autonomy")


def test_missing_stale_and_future_timestamps_are_rejected() -> None:
    assert "缺少" in validate(stamp_s=0.0)
    assert "过期" in validate(stamp_s=99.0)
    assert "超前" in validate(stamp_s=101.0)


def test_non_finite_and_out_of_normalized_range_are_rejected() -> None:
    assert "NaN/Inf" in validate(values=(math.nan, 0.0, 0.0, 0.0))
    assert "[-1, 1]" in validate(values=(1.01, 0.0, 0.0, 0.0))
