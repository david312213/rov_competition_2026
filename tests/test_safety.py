"""控制网关与自主任务共用安全门测试。"""

import math
from dataclasses import replace
from pathlib import Path

from rov_competition.config import load_autonomy_config
from rov_competition.safety import (
    AutonomyControlStatus,
    AutonomyTelemetryStatus,
    autonomy_control_error,
    autonomy_safety_error,
    is_fresh_telemetry,
    validate_command_envelope,
)

PACKAGE = (
    Path(__file__).resolve().parents[1]
    / "ros2_ws"
    / "src"
    / "rov_competition"
)
DEFAULT_MISSION = load_autonomy_config(PACKAGE / "config" / "autonomy.yaml").mission
MISSION = replace(
    DEFAULT_MISSION,
    allow_autonomous_mission=True,
    allow_open_loop_horizontal_motion=True,
)


def healthy_status() -> AutonomyTelemetryStatus:
    """返回满足全部自主启动条件的测试遥测。"""

    return AutonomyTelemetryStatus(
        heartbeat_valid=True,
        heartbeat_age_s=0.1,
        message_age_s=0.1,
        armed=True,
        depth_valid=True,
        depth_m=2.0,
        attitude_valid=True,
        yaw_deg=10.0,
        flight_mode="ALT_HOLD",
        received_monotonic=100.0,
    )


def healthy_control_status() -> AutonomyControlStatus:
    """返回已预检、已授权、已由 ROS 解锁的伪网关状态。"""

    return AutonomyControlStatus(
        state="READY",
        command_source="autonomy",
        configuration_allows_actuation=True,
        configuration_allows_ros_arming=True,
        configuration_allows_gripper=True,
        runtime_enabled=True,
        preflight_passed=True,
        emergency_stop_latched=False,
        armed_by_ros=True,
        armed=True,
        flight_mode="ALT_HOLD",
        received_monotonic=100.0,
    )


def command_envelope_error(**overrides):
    """使用一组合法默认值执行运动命令封套检查。"""

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


def test_healthy_telemetry_allows_autonomy() -> None:
    """实时、已解锁、已入水且姿态有效时允许启动。"""

    assert autonomy_safety_error(healthy_status(), MISSION) is None


def test_default_configuration_blocks_unvalidated_autonomy() -> None:
    """仓库默认值不能在未做水池标定时自动前进。"""

    assert DEFAULT_MISSION.allow_autonomous_mission is False
    assert "自主任务" in autonomy_safety_error(healthy_status(), DEFAULT_MISSION)


def test_missing_or_stale_telemetry_blocks_autonomy() -> None:
    """未收到消息或心跳过期均必须阻止运动。"""

    assert "尚未收到" in autonomy_safety_error(None, MISSION)
    stale = replace(
        healthy_status(), heartbeat_age_s=MISSION.maximum_heartbeat_age_s + 0.1
    )
    assert "心跳过期" in autonomy_safety_error(stale, MISSION)


def test_unarmed_or_out_of_water_blocks_autonomy() -> None:
    """未解锁或未达到最小入水深度时不得开始遍历。"""

    assert "尚未解锁" in autonomy_safety_error(
        replace(healthy_status(), armed=False), MISSION
    )
    assert "深度" in autonomy_safety_error(
        replace(healthy_status(), depth_m=0.0), MISSION
    )


def test_cached_telemetry_becomes_stale_with_local_time() -> None:
    """ROS 回调停止后，即使缓存中的年龄不变，也必须随本地时间判定过期。"""

    status = healthy_status()
    error = autonomy_safety_error(
        status,
        MISSION,
        now=status.received_monotonic + MISSION.maximum_message_age_s + 0.1,
    )
    assert "遥测过期" in error


def test_non_depth_hold_mode_blocks_autonomy() -> None:
    """MANUAL/STABILIZE 下中位油门不保证深度，必须阻止定时遍历。"""

    error = autonomy_safety_error(
        replace(healthy_status(), flight_mode="MANUAL"),
        MISSION,
    )
    assert "模式" in error


def test_non_finite_telemetry_blocks_autonomy() -> None:
    """NaN 深度或消息年龄不得通过入水检查。"""

    error = autonomy_safety_error(
        replace(healthy_status(), depth_m=float("nan")),
        MISSION,
    )
    assert "NaN/Inf" in error


def test_healthy_fake_gateway_allows_autonomy() -> None:
    """纯 Python 伪网关在全部门控通过时允许自主。"""

    assert (
        autonomy_control_error(
            healthy_control_status(), MISSION, command_source="autonomy", now=100.1
        )
        is None
    )


def test_fake_gateway_rejects_wrong_source_mode_and_stale_status() -> None:
    """错误来源、错误模式和过期状态都不能进入自主。"""

    wrong_source = replace(healthy_control_status(), command_source="commissioning")
    assert "来源" in autonomy_control_error(
        wrong_source, MISSION, command_source="autonomy", now=100.1
    )
    wrong_mode = replace(healthy_control_status(), flight_mode="MANUAL")
    assert "模式" in autonomy_control_error(
        wrong_mode, MISSION, command_source="autonomy", now=100.1
    )
    assert "过期" in autonomy_control_error(
        healthy_control_status(),
        MISSION,
        command_source="autonomy",
        now=100.0 + MISSION.maximum_control_status_age_s + 0.1,
    )


def test_fake_gateway_requires_preflight_runtime_and_ros_arming() -> None:
    """预检、运行时许可和 ROS 解锁任一缺失均必须拒绝。"""

    for changed, expected in (
        ({"preflight_passed": False}, "预检"),
        ({"runtime_enabled": False}, "运行时"),
        ({"armed_by_ros": False}, "解锁"),
        ({"configuration_allows_gripper": False}, "机械爪"),
    ):
        error = autonomy_control_error(
            replace(healthy_control_status(), **changed),
            MISSION,
            command_source="autonomy",
            now=100.1,
        )
        assert expected in error


def test_telemetry_timestamp_expires() -> None:
    """旧深度或姿态不得被持续当作实时数据发布。"""

    assert is_fresh_telemetry(10.0, now=10.5, timeout_s=1.0) is True
    assert is_fresh_telemetry(10.0, now=11.1, timeout_s=1.0) is False
    assert is_fresh_telemetry(None, now=10.0, timeout_s=1.0) is False


def test_fresh_expected_command_source_is_accepted() -> None:
    assert command_envelope_error() is None


def test_wrong_command_source_is_rejected() -> None:
    assert "非法命令来源" in command_envelope_error(source="autonomy")


def test_missing_stale_and_future_command_timestamps_are_rejected() -> None:
    assert "缺少" in command_envelope_error(stamp_s=0.0)
    assert "过期" in command_envelope_error(stamp_s=99.0)
    assert "超前" in command_envelope_error(stamp_s=101.0)


def test_invalid_command_numbers_are_rejected() -> None:
    assert "NaN/Inf" in command_envelope_error(
        values=(math.nan, 0.0, 0.0, 0.0)
    )
    assert "[-1, 1]" in command_envelope_error(values=(1.01, 0.0, 0.0, 0.0))
