"""MAVLink 双后端、限幅/斜坡、释放值和正常解锁的纯软件测试。"""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

import pytest
from rov_competition.config import (
    ControlProtocol,
    GripperConfig,
    GripperOutputConfig,
    GripperSweepConfig,
    RcOverrideConfig,
    load_robot_config,
)
from rov_competition.domain import GripperAction, MotionCommand
from rov_competition.preflight import PreflightCheck, PreflightReport
from rov_competition.vehicle import (
    EXTENDED_PWM_CONFIRMATION,
    GRIPPER_TEST_CONFIRMATION,
    MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_SET_SERVO,
    MAV_MODE_FLAG_SAFETY_ARMED,
    MavlinkVehicle,
    VehicleError,
)

PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "rov_competition"


class FakeMessage:
    """可按需要携带属性的 MAVLink 消息替身。"""

    def __init__(
        self,
        message_type: str,
        *,
        source_system: int = 1,
        source_component: int = 1,
        **fields: object,
    ) -> None:
        self._message_type = message_type
        self._source_system = source_system
        self._source_component = source_component
        for name, value in fields.items():
            setattr(self, name, value)

    def get_type(self) -> str:
        return self._message_type

    def get_srcSystem(self) -> int:
        return self._source_system

    def get_srcComponent(self) -> int:
        return self._source_component


def heartbeat(*, armed: bool) -> FakeMessage:
    """创建 ArduPilot submarine MANUAL 心跳。"""

    return FakeMessage(
        "HEARTBEAT",
        base_mode=MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
        system_status=4,
        autopilot=3,
        type=12,
        custom_mode=19,
    )


class FakeMav:
    """记录所有可能影响实艇的 MAVLink 调用。"""

    def __init__(self, owner: "FakeMaster") -> None:
        self.owner = owner
        self.manual_controls: list[tuple[int, ...]] = []
        self.overrides: list[tuple[int, ...]] = []
        self.commands: list[tuple[float, ...]] = []
        self.heartbeats: list[tuple[int, ...]] = []

    def manual_control_send(self, *values: int) -> None:
        self.manual_controls.append(values)

    def rc_channels_override_send(self, *values: int) -> None:
        self.overrides.append(values)

    def command_long_send(self, *values: float) -> None:
        self.commands.append(values)
        if self.owner.acknowledge_servo and int(values[2]) == MAV_CMD_DO_SET_SERVO:
            self.owner.messages.append(
                FakeMessage(
                    "COMMAND_ACK",
                    command=MAV_CMD_DO_SET_SERVO,
                    result=0,
                )
            )

    def heartbeat_send(self, *values: int) -> None:
        self.heartbeats.append(values)


class FakeMaster:
    """无需网络、串口或飞控的 MAVLink 连接替身。"""

    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.messages: list[FakeMessage] = []
        self.acknowledge_servo = False
        self.mav = FakeMav(self)
        self.wait_heartbeats: list[FakeMessage] | None = None
        self.closed = False

    def wait_heartbeat(self, timeout: float) -> FakeMessage | None:
        if self.wait_heartbeats is not None:
            return self.wait_heartbeats.pop(0) if self.wait_heartbeats else None
        return heartbeat(armed=False) if timeout > 0 else None

    def recv_match(self, **_options: object) -> FakeMessage | None:
        return self.messages.pop(0) if self.messages else None

    def mode_mapping(self) -> dict[str, int]:
        return {"MANUAL": 19, "STABILIZE": 0}

    def close(self) -> None:
        self.closed = True


def passed_report(*, include_gripper_checks: bool = False) -> PreflightReport:
    """创建不含关键失败的假预检报告。"""

    parameters = {}
    for channel in (3, 4, 5, 6):
        parameters[f"RC{channel}_MIN"] = 1100
        parameters[f"RC{channel}_TRIM"] = 1500
        parameters[f"RC{channel}_MAX"] = 1900
    checks = ()
    if include_gripper_checks:
        checks = (
            PreflightCheck(
                name="机械爪输出可由 MAVLink 控制",
                level="warning",
                passed=True,
                observed="free",
                expected="free",
            ),
            PreflightCheck(
                name="机械爪 AUX 输出已启用 PWM",
                level="warning",
                passed=True,
                observed="BRD_PWM_COUNT=4",
                expected=">=4",
            ),
        )
    return PreflightReport(
        generated_at_utc="2026-08-15T00:00:00+00:00",
        target_system=1,
        target_component=1,
        firmware_version="test",
        checks=checks,
        parameters=parameters,
    )


def make_vehicle(
    *,
    live: bool,
    protocol: ControlProtocol = ControlProtocol.MANUAL_CONTROL,
    ros_arming: bool = False,
    arm_timeout_s: float = 0.05,
) -> tuple[MavlinkVehicle, FakeMaster]:
    """创建指定协议和授权状态的测试飞控适配器。"""

    loaded = load_robot_config(PACKAGE / "config" / "robot.example.yaml")
    config = replace(
        loaded,
        expected_frame_config=2,
        allow_live_actuation=live,
        allow_ros_arming=ros_arming,
        control_protocol=protocol,
        arm_ack_timeout_s=arm_timeout_s,
    )
    master = FakeMaster()
    vehicle = MavlinkVehicle(config, connection_factory=lambda _uri, **_options: master)
    vehicle.connect()
    vehicle._preflight_report = passed_report()
    return vehicle, master


def test_default_safety_gate_rejects_motion_without_output() -> None:
    """未授权时不得产生 MANUAL_CONTROL 或 RC override。"""

    vehicle, master = make_vehicle(live=False)
    vehicle._telemetry.armed = True
    with pytest.raises(VehicleError, match="已禁用"):
        vehicle.send_motion(MotionCommand(forward=0.05))
    assert master.mav.manual_controls == []
    assert master.mav.overrides == []


def test_connection_skips_foreign_gcs_heartbeat_and_selects_ardusub() -> None:
    """连接阶段不能把先到的 QGC 心跳误当成目标飞控。"""

    config = load_robot_config(PACKAGE / "config" / "robot.example.yaml")
    master = FakeMaster()
    master.wait_heartbeats = [
        FakeMessage(
            "HEARTBEAT",
            source_system=255,
            source_component=190,
            autopilot=8,
            type=6,
            base_mode=0,
            system_status=4,
            custom_mode=0,
        ),
        heartbeat(armed=False),
    ]
    vehicle = MavlinkVehicle(config, connection_factory=lambda _uri, **_options: master)
    vehicle.connect()
    snapshot = vehicle.current_telemetry()
    assert master.target_system == 1
    assert master.target_component == 1
    assert snapshot.autopilot_type == 3
    assert snapshot.vehicle_type == 12


def test_manual_control_mapping_uses_ardusub_vertical_midpoint() -> None:
    """MANUAL_CONTROL 的升沉中位必须为 500，其他轴中位为 0。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    vehicle._last_output_at = 0.0
    vehicle.send_motion(
        MotionCommand(forward=0.10, lateral=-0.10, vertical=0.10, yaw=-0.10),
        now=1.0,
    )
    assert master.mav.manual_controls[-1] == (1, 100, -100, 550, -100, 0)
    vehicle.stop_motion(force=True)
    assert master.mav.manual_controls[-1] == (1, 0, 0, 500, 0, 0)


def test_rc_override_maps_actual_calibration_and_release_values() -> None:
    """RC 兼容后端使用真实 MIN/TRIM/MAX，释放 1..8 通道时发 0。"""

    vehicle, master = make_vehicle(live=True, protocol=ControlProtocol.RC_OVERRIDE)
    vehicle._telemetry.armed = True
    # 证明映射使用飞控报告中 RC5 自己的标定，而不是全局旧值。
    vehicle._preflight_report.parameters["RC5_TRIM"] = 1510
    vehicle._preflight_report.parameters["RC5_MAX"] = 1910
    vehicle._last_output_at = 0.0
    vehicle.send_motion(
        MotionCommand(forward=0.10, lateral=-0.10, vertical=0.10, yaw=-0.10),
        now=1.0,
    )
    values = master.mav.overrides[-1][2:]  # 前两项是 target system/component。
    assert values[4] == 1550  # forward -> RC5: 1510 + 0.1 * (1910-1510)
    assert values[5] == 1460  # lateral -> RC6
    assert values[2] == 1540  # vertical -> RC3
    assert values[3] == 1460  # yaw -> RC4
    assert values[0] == 65535

    vehicle.release_control()
    released = master.mav.overrides[-1][2:]
    for channel in vehicle.config.rc_override.channels.values():
        assert released[channel - 1] == 0
    assert released[0] == 65535


def test_rc_release_uses_65534_for_channels_nine_to_eighteen() -> None:
    """MAVLink 规定 RC9..18 释放值为 UINT16_MAX-1，不能套用 RC1..8 的 0。"""

    vehicle, master = make_vehicle(live=True, protocol=ControlProtocol.RC_OVERRIDE)
    channels = dict(vehicle.config.rc_override.channels)
    channels["forward"] = 9
    vehicle.config = replace(
        vehicle.config,
        rc_override=RcOverrideConfig(
            channels=channels,
        ),
    )
    vehicle.release_control()
    released = master.mav.overrides[-1][2:]
    assert released[8] == 65534


def test_first_command_is_slew_limited_then_reaches_configured_limit() -> None:
    """首次输出按 0.05s 起步，再缓升到配置的 0.80 上限。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    first = vehicle.send_motion(MotionCommand(forward=1.0), now=10.0)
    assert first.forward == pytest.approx(0.025)
    assert master.mav.manual_controls[-1][1] == 25
    second = vehicle.send_motion(MotionCommand(forward=1.0), now=10.05)
    assert second.forward == pytest.approx(0.05)
    third = vehicle.send_motion(MotionCommand(forward=1.0), now=11.05)
    assert third.forward == pytest.approx(0.55)
    final = vehicle.send_motion(MotionCommand(forward=1.0), now=12.05)
    assert final.forward == pytest.approx(0.80)
    assert master.mav.manual_controls[-1][1] == 800


def test_command_timeout_and_non_finite_input_are_detected() -> None:
    """看门狗超时必须可观测，NaN 必须回中并报错。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    vehicle.send_motion(MotionCommand(forward=0.05), now=5.0)
    assert not vehicle.command_timed_out(5.0 + vehicle.config.command_timeout_s)
    assert vehicle.command_timed_out(5.0 + vehicle.config.command_timeout_s + 0.01)
    with pytest.raises(VehicleError, match="非法"):
        vehicle.send_motion(MotionCommand(forward=float("nan")), now=6.0)
    assert master.mav.manual_controls[-1] == (1, 0, 0, 500, 0, 0)


def test_stale_heartbeat_unarmed_and_wrong_mode_reject_motion() -> None:
    """心跳、解锁状态和 MANUAL 模式任一不合格都不得运动。"""

    vehicle, _master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    vehicle._telemetry.last_heartbeat_monotonic = (
        time.monotonic() - vehicle.config.heartbeat_stale_timeout_s - 0.1
    )
    with pytest.raises(VehicleError, match="心跳超时"):
        vehicle.send_motion(MotionCommand(forward=0.05))

    vehicle._telemetry.last_heartbeat_monotonic = time.monotonic()
    vehicle._telemetry.armed = False
    with pytest.raises(VehicleError, match="尚未解锁"):
        vehicle.send_motion(MotionCommand(forward=0.05))

    vehicle._telemetry.armed = True
    vehicle._telemetry.flight_mode = "STABILIZE"
    with pytest.raises(VehicleError, match="不允许控制"):
        vehicle.send_motion(MotionCommand(forward=0.05))


def test_foreign_gcs_heartbeat_cannot_overwrite_vehicle_state() -> None:
    """同一路由上 QGC 或其他系统的心跳不能改写飞控解锁状态。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    master.messages.append(
        FakeMessage(
            "HEARTBEAT",
            source_system=42,
            source_component=190,
            base_mode=0,
            system_status=4,
            autopilot=8,
            type=6,
            custom_mode=0,
        )
    )
    snapshot = vehicle.poll_telemetry()
    assert snapshot.armed is True
    assert snapshot.autopilot_type == 3
    assert snapshot.vehicle_type == 12


def test_arm_waits_for_ack_and_heartbeat_without_force_magic_value() -> None:
    """解锁需 ACK 和心跳证据，param2 恒为 0，不出现 21196。"""

    vehicle, master = make_vehicle(live=True, ros_arming=True)
    master.messages.extend(
        [
            FakeMessage("COMMAND_ACK", command=MAV_CMD_COMPONENT_ARM_DISARM, result=0),
            heartbeat(armed=True),
        ]
    )
    vehicle.set_armed(True)
    arm_call = master.mav.commands[-1]
    assert arm_call[2] == MAV_CMD_COMPONENT_ARM_DISARM
    assert arm_call[4] == 1
    assert arm_call[5] == 0
    assert 21196 not in arm_call
    assert vehicle.current_telemetry().armed is True


def test_arm_rejection_and_heartbeat_timeout_are_reported() -> None:
    """飞控拒绝或 ACK 后心跳未变化都不能被当成解锁成功。"""

    rejected, rejected_master = make_vehicle(live=True, ros_arming=True)
    rejected_master.messages.append(
        FakeMessage("COMMAND_ACK", command=MAV_CMD_COMPONENT_ARM_DISARM, result=2)
    )
    with pytest.raises(VehicleError, match="飞控拒绝解锁"):
        rejected.set_armed(True)

    timed_out, timed_out_master = make_vehicle(
        live=True, ros_arming=True, arm_timeout_s=0.01
    )
    timed_out_master.messages.append(
        FakeMessage("COMMAND_ACK", command=MAV_CMD_COMPONENT_ARM_DISARM, result=0)
    )
    with pytest.raises(VehicleError, match="确认超时"):
        timed_out.set_armed(True)


def test_arm_without_command_ack_times_out() -> None:
    """完全收不到匹配 ACK 时必须超时，不能仅凭发送成功宣称已解锁。"""

    vehicle, _master = make_vehicle(
        live=True,
        ros_arming=True,
        arm_timeout_s=0.01,
    )
    with pytest.raises(VehicleError, match="确认超时"):
        vehicle.set_armed(True)


def test_arm_ack_with_unchanged_heartbeat_still_times_out() -> None:
    """即使 ACK 成功，后续心跳仍为上锁也不能把软件状态改成已解锁。"""

    vehicle, master = make_vehicle(
        live=True,
        ros_arming=True,
        arm_timeout_s=0.01,
    )
    master.messages.extend(
        [
            FakeMessage("COMMAND_ACK", command=MAV_CMD_COMPONENT_ARM_DISARM, result=0),
            heartbeat(armed=False),
        ]
    )
    with pytest.raises(VehicleError, match="确认超时"):
        vehicle.set_armed(True)
    assert vehicle.current_telemetry().armed is False


def test_emergency_sequence_neutralises_normally_disarms_and_releases() -> None:
    """假 MAVLink 必须证明急停序列发了中位、普通上锁和控制释放。"""

    vehicle, master = make_vehicle(live=True, ros_arming=True)
    vehicle._telemetry.armed = True
    master.messages.extend(
        [
            FakeMessage("COMMAND_ACK", command=MAV_CMD_COMPONENT_ARM_DISARM, result=0),
            heartbeat(armed=False),
        ]
    )
    errors = vehicle.stop_disarm_and_release()
    assert errors == []
    assert master.mav.manual_controls[0] == (1, 0, 0, 500, 0, 0)
    disarm_call = master.mav.commands[-1]
    assert disarm_call[2] == MAV_CMD_COMPONENT_ARM_DISARM
    assert disarm_call[4] == 0
    assert disarm_call[5] == 0
    assert 21196 not in disarm_call
    assert master.mav.manual_controls[-1] == (1, 0, 0, 500, 0, 0)


def test_gripper_uses_independent_gate_and_nonblocking_dalian_sweep() -> None:
    """授权后按旧大连曲线渐变，且 YAML 使用绝对输出号、不再做“+8”。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    with pytest.raises(VehicleError, match="未单独授权"):
        vehicle.set_gripper(GripperAction.OPEN)
    assert master.mav.commands == []

    vehicle.config = replace(
        vehicle.config,
        allow_gripper_actuation=True,
        gripper=replace(vehicle.config.gripper, calibrated=True),
    )
    output = vehicle.config.gripper.outputs[0]
    values = output.open_sweep.values()
    step_count, duration_s = vehicle.set_gripper(GripperAction.OPEN, now=10.0)
    assert step_count == len(values)
    assert duration_s == pytest.approx((len(values) - 1) * 0.125)
    assert len(master.mav.commands) == 1
    assert master.mav.commands[-1][4] == output.output_channel
    assert master.mav.commands[-1][5] == values[0]

    assert not vehicle.update_gripper(now=10.124)
    for index, expected_pwm in enumerate(values[1:], 1):
        assert vehicle.update_gripper(now=10.0 + index * 0.125)
        assert master.mav.commands[-1][4] == 12
        assert master.mav.commands[-1][5] == expected_pwm
    assert not vehicle.gripper_active


def test_rst_gripper_sends_two_exact_outputs_in_one_step() -> None:
    """RST 档案一次服务请求必须同时发送左右执行器的历史值。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    rst = GripperConfig(
        profile="rst",
        calibrated=True,
        allow_extended_pwm=True,
        step_interval_s=0.125,
        outputs=(
            GripperOutputConfig(
                output_channel=11,
                open_sweep=GripperSweepConfig(950, 950, 25),
                close_sweep=GripperSweepConfig(1450, 1450, 25),
            ),
            GripperOutputConfig(
                output_channel=10,
                open_sweep=GripperSweepConfig(1050, 1050, 25),
                close_sweep=GripperSweepConfig(500, 500, -25),
            ),
        ),
    )
    vehicle.config = replace(
        vehicle.config,
        allow_gripper_actuation=True,
        gripper=rst,
    )

    step_count, duration_s = vehicle.set_gripper(GripperAction.OPEN, now=30.0)
    assert step_count == 1
    assert duration_s == 0.0
    assert [(call[4], call[5]) for call in master.mav.commands[-2:]] == [
        (11, 950),
        (10, 1050),
    ]
    assert not vehicle.gripper_active

    vehicle.set_gripper(GripperAction.CLOSE, now=31.0)
    assert [(call[4], call[5]) for call in master.mav.commands[-2:]] == [
        (11, 1450),
        (10, 500),
    ]


def test_gripper_rejects_overlapping_sequence_and_cancel_stops_future_steps() -> None:
    """同一输出不能叠加开闭动作，取消后也不得继续发送旧序列。"""

    vehicle, master = make_vehicle(live=True)
    vehicle._telemetry.armed = True
    vehicle.config = replace(
        vehicle.config,
        allow_gripper_actuation=True,
        gripper=replace(vehicle.config.gripper, calibrated=True),
    )
    vehicle.set_gripper(GripperAction.CLOSE, now=20.0)
    with pytest.raises(VehicleError, match="拒绝重叠动作"):
        vehicle.set_gripper(GripperAction.OPEN, now=20.0)
    sent_before_cancel = len(master.mav.commands)
    vehicle.cancel_gripper()
    assert not vehicle.gripper_active
    assert not vehicle.update_gripper(now=99.0)
    assert len(master.mav.commands) == sent_before_cancel


def test_disarmed_dalian_candidate_test_waits_for_every_ack() -> None:
    """候选档案只能在上锁、预检通过时逐步发送，并保存每个 ACK。"""

    vehicle, master = make_vehicle(live=False)
    vehicle._preflight_report = passed_report(include_gripper_checks=True)
    master.acknowledge_servo = True
    slept: list[float] = []

    records = vehicle.run_disarmed_gripper_test(
        GripperAction.OPEN,
        confirmation=GRIPPER_TEST_CONFIRMATION,
        sleep_fn=slept.append,
    )

    expected_values = vehicle.config.gripper.outputs[0].open_sweep.values()
    assert [item["pwm"] for item in records] == list(expected_values)
    assert all(item["ack_result"] == 0 for item in records)
    assert [int(call[2]) for call in master.mav.commands] == [
        MAV_CMD_DO_SET_SERVO
    ] * len(expected_values)
    assert MAV_CMD_COMPONENT_ARM_DISARM not in {
        int(call[2]) for call in master.mav.commands
    }
    assert slept == [vehicle.config.gripper.step_interval_s] * (
        len(expected_values) - 1
    )


def test_disarmed_gripper_refreshes_heartbeat_after_operator_pause() -> None:
    """人工确认用时较长时，发送前必须处理 UDP 队列中的新心跳。"""

    vehicle, master = make_vehicle(live=False)
    vehicle._preflight_report = passed_report(include_gripper_checks=True)
    vehicle._telemetry.last_heartbeat_monotonic = (
        time.monotonic() - vehicle.config.heartbeat_stale_timeout_s - 1.0
    )
    # 模拟操作员看完提示后，新心跳已到达内核 UDP 队列，
    # 但候选测试命令尚未调用 poll_telemetry 处理它。
    master.messages.append(heartbeat(armed=False))
    master.acknowledge_servo = True

    records = vehicle.run_disarmed_gripper_test(
        GripperAction.OPEN,
        confirmation=GRIPPER_TEST_CONFIRMATION,
        sleep_fn=lambda _seconds: None,
    )

    assert records
    assert vehicle.current_telemetry().armed is False
    assert vehicle.heartbeat_is_fresh()


def test_disarmed_gripper_candidate_rejects_armed_or_missing_preflight() -> None:
    """软件配置门关闭并不够；飞控上锁和输出检查缺一不可。"""

    vehicle, master = make_vehicle(live=False)
    vehicle._preflight_report = passed_report(include_gripper_checks=True)
    vehicle._telemetry.armed = True
    with pytest.raises(VehicleError, match="必须明确上锁"):
        vehicle.run_disarmed_gripper_test(
            GripperAction.OPEN,
            confirmation=GRIPPER_TEST_CONFIRMATION,
            sleep_fn=lambda _seconds: None,
        )
    assert master.mav.commands == []

    missing, missing_master = make_vehicle(live=False)
    with pytest.raises(VehicleError, match="报告中缺失"):
        missing.run_disarmed_gripper_test(
            GripperAction.OPEN,
            confirmation=GRIPPER_TEST_CONFIRMATION,
            sleep_fn=lambda _seconds: None,
        )
    assert missing_master.mav.commands == []


def test_rst_candidate_requires_second_extended_pwm_confirmation() -> None:
    """RST 任一动作使用前都必须明示知晓整份档案包含 500us。"""

    vehicle, master = make_vehicle(live=False)
    vehicle.config = replace(
        vehicle.config,
        gripper=GripperConfig(
            profile="rst",
            calibrated=False,
            allow_extended_pwm=False,
            step_interval_s=0.125,
            outputs=(
                GripperOutputConfig(
                    output_channel=11,
                    open_sweep=GripperSweepConfig(950, 950, 25),
                    close_sweep=GripperSweepConfig(1450, 1450, 25),
                ),
                GripperOutputConfig(
                    output_channel=10,
                    open_sweep=GripperSweepConfig(1050, 1050, 25),
                    close_sweep=GripperSweepConfig(500, 500, -25),
                ),
            ),
        ),
    )
    vehicle._preflight_report = passed_report(include_gripper_checks=True)
    master.acknowledge_servo = True

    with pytest.raises(VehicleError, match="第二确认词"):
        vehicle.run_disarmed_gripper_test(
            GripperAction.OPEN,
            confirmation=GRIPPER_TEST_CONFIRMATION,
            sleep_fn=lambda _seconds: None,
        )
    assert master.mav.commands == []

    records = vehicle.run_disarmed_gripper_test(
        GripperAction.OPEN,
        confirmation=GRIPPER_TEST_CONFIRMATION,
        extended_pwm_confirmation=EXTENDED_PWM_CONFIRMATION,
        sleep_fn=lambda _seconds: None,
    )
    assert [(item["output_channel"], item["pwm"]) for item in records] == [
        (11, 950),
        (10, 1050),
    ]
