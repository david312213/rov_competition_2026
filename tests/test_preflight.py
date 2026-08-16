"""八推进器 ArduSub 只读参数预检测试。"""

from dataclasses import replace
from pathlib import Path

from rov_competition.config import ControlProtocol, load_robot_config
from rov_competition.preflight import (
    build_preflight_report,
    load_ardupilot_parameter_log,
)

PROJECT = Path(__file__).resolve().parents[1]
PACKAGE = PROJECT / "ros2_ws" / "src" / "rov_competition"


def eight_thruster_parameters() -> dict[str, float]:
    """创建满足当前严格预检的最小飞控参数集。"""

    parameters: dict[str, float] = {
        "FRAME_CONFIG": 2,
        "FS_PILOT_INPUT": 2,
        "FS_PILOT_TIMEOUT": 3,
        "FS_GCS_ENABLE": 2,
        "ARMING_CHECK": 1,
        "SYSID_MYGCS": 255,
    }
    for motor in range(1, 9):
        parameters[f"SERVO{motor}_FUNCTION"] = 32 + motor
        parameters[f"SERVO{motor}_MIN"] = 1100
        parameters[f"SERVO{motor}_TRIM"] = 1500
        parameters[f"SERVO{motor}_MAX"] = 1900
        parameters[f"MOT_{motor}_DIRECTION"] = 1
    return parameters


def selected_config():
    """返回已明确选择 Vectored_6DOF 的只读配置。"""

    loaded = load_robot_config(PACKAGE / "config" / "robot.example.yaml")
    return replace(loaded, expected_frame_config=2)


def test_valid_eight_thruster_parameter_set_passes() -> None:
    """Motor1..8 各出现一次、机架和 failsafe 正确时应通过。"""

    report = build_preflight_report(
        selected_config(),
        target_system=1,
        target_component=1,
        autopilot_type=3,
        vehicle_type=12,
        firmware_version="4.5.7",
        parameters=eight_thruster_parameters(),
    )
    assert report.passed
    assert report.critical_failures == ()


def test_legacy_six_thruster_log_is_blocked_for_current_vehicle() -> None:
    """2022 旧日志的 FRAME_CONFIG=1 且只有 Motor1..6，必须阻止八推输出。"""

    parameters = load_ardupilot_parameter_log(
        PROJECT / "tests" / "fixtures" / "legacy_six_thruster_params.log"
    )
    report = build_preflight_report(
        selected_config(),
        target_system=1,
        target_component=1,
        autopilot_type=3,
        vehicle_type=12,
        firmware_version="legacy",
        parameters=parameters,
    )
    assert not report.passed
    failed_names = {check.name for check in report.critical_failures}
    assert "八推进器机架" in failed_names
    assert "Motor1–Motor8 功能" in failed_names


def test_duplicate_or_missing_motor_function_is_blocked() -> None:
    """即使有八个输出口，Motor 功能重复也不能通过。"""

    parameters = eight_thruster_parameters()
    parameters["SERVO8_FUNCTION"] = 39  # 重复 Motor7，同时缺 Motor8。
    report = build_preflight_report(
        selected_config(),
        target_system=1,
        target_component=1,
        autopilot_type=3,
        vehicle_type=12,
        firmware_version="4.5.7",
        parameters=parameters,
    )
    assert not report.passed
    assert any(
        check.name == "Motor1–Motor8 功能" and not check.passed
        for check in report.checks
    )


def test_rc_backend_accepts_valid_independent_channel_calibrations() -> None:
    """RC 兼容后端应使用每个通道自己的 MIN/TRIM/MAX，不强求四轴相同。"""

    config = replace(selected_config(), control_protocol=ControlProtocol.RC_OVERRIDE)
    parameters = eight_thruster_parameters()
    parameters.update({"RC_OPTIONS": 0, "RC_OVERRIDE_TIME": 0.5})
    for index, channel in enumerate((3, 4, 5, 6)):
        parameters[f"RC{channel}_MIN"] = 1090 + index
        parameters[f"RC{channel}_TRIM"] = 1495 + index
        parameters[f"RC{channel}_MAX"] = 1905 + index
    report = build_preflight_report(
        config,
        target_system=1,
        target_component=1,
        autopilot_type=3,
        vehicle_type=12,
        firmware_version="4.5.7",
        parameters=parameters,
    )
    assert report.passed
    assert any(
        check.name == "RC 通道实际标定可用" and check.passed for check in report.checks
    )


def test_wrong_vehicle_identity_is_blocked() -> None:
    """通用飞机固件或非 ArduPilot 心跳不得驱动 ROV。"""

    report = build_preflight_report(
        selected_config(),
        target_system=1,
        target_component=1,
        autopilot_type=8,
        vehicle_type=2,
        firmware_version="unknown",
        parameters=eight_thruster_parameters(),
    )
    assert not report.passed
    failed_names = {check.name for check in report.critical_failures}
    assert {"飞控类型", "载具类型"}.issubset(failed_names)
