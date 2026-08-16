"""ROS 遥测数据新鲜度判断测试。"""

from rov_competition.safety import is_fresh_telemetry


def test_telemetry_timestamp_expires() -> None:
    """旧深度/姿态不得被持续当作实时数据发布。"""

    assert is_fresh_telemetry(10.0, now=10.5, timeout_s=1.0) is True
    assert is_fresh_telemetry(10.0, now=11.1, timeout_s=1.0) is False
    assert is_fresh_telemetry(None, now=10.0, timeout_s=1.0) is False
