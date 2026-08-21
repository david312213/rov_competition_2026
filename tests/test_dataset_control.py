"""数据集键盘控制、可选回收和假网关回归测试。"""

from dataclasses import replace
from pathlib import Path

import pytest
from rov_competition.config import load_dataset_config
from rov_competition.dataset_control import (
    DatasetControlError,
    DatasetRuntimeSnapshot,
    KeyCommandState,
    RecoveryController,
    active_safety_error,
    motion_from_keys,
    prearm_safety_error,
)
from rov_competition.domain import MotionCommand


PROJECT = Path(__file__).resolve().parents[1]
DATASET_EXAMPLE = (
    PROJECT
    / "ros2_ws"
    / "src"
    / "rov_competition"
    / "config"
    / "dataset.example.yaml"
)


def _config():
    """返回数据集键盘控制测试配置。"""

    return load_dataset_config(DATASET_EXAMPLE)


@pytest.mark.parametrize(
    ("key", "expected"),
    [
        ("w", MotionCommand(forward=0.20)),
        ("s", MotionCommand(forward=-0.20)),
        ("a", MotionCommand(lateral=-0.20)),
        ("d", MotionCommand(lateral=0.20)),
        ("1", MotionCommand(yaw=-0.20)),
        ("2", MotionCommand(yaw=0.20)),
        ("up", MotionCommand(vertical=0.20)),
        ("down", MotionCommand(vertical=-0.20)),
    ],
)
def test_every_movement_key_has_one_clear_axis(key: str, expected: MotionCommand) -> None:
    """每个键只表达一个艇体运动意图。"""

    assert motion_from_keys({key}, 0.20, 0.80) == expected


def test_opposite_keys_cancel_and_release_returns_to_neutral() -> None:
    """相反键抵消，松开最后一键必须回中。"""

    state = KeyCommandState(_config())
    state.press("w")
    state.press("s")
    assert state.motion().is_neutral()
    state.release("s")
    assert state.motion().forward == pytest.approx(0.20)
    state.release("w")
    assert state.motion().is_neutral()


def test_focus_loss_clear_and_each_combined_axis_keeps_its_power() -> None:
    """窗口失焦回中，三轴组合不会再把每轴功率平分。"""

    state = KeyCommandState(_config())
    for key in ("w", "d", "2"):
        state.press(key)
    motion = state.motion()
    assert motion.forward == pytest.approx(0.20)
    assert motion.lateral == pytest.approx(0.20)
    assert motion.yaw == pytest.approx(0.20)
    assert all(
        abs(value) <= 0.80
        for value in (motion.forward, motion.lateral, motion.vertical, motion.yaw)
    )
    state.clear()
    assert state.motion().is_neutral()


def test_power_adjustment_is_clamped_to_configured_bounds() -> None:
    """+/- 每次 0.05，无论按多少次都不越界。"""

    state = KeyCommandState(_config())
    assert state.adjust(1) == pytest.approx(0.25)
    for _ in range(30):
        state.adjust(1)
    assert state.strength == pytest.approx(0.80)
    for _ in range(30):
        state.adjust(-1)
    assert state.strength == pytest.approx(0.05)


def test_recovery_never_descends_when_already_shallower() -> None:
    """按 0 后只能上升或回中，绝不为追目标深度下潜。"""

    config = _config()
    controller = RecoveryController(config, target_depth_m=0.20, started_at=10.0)
    rising = controller.step(depth_valid=True, depth_m=0.50, now=10.1)
    assert 0.0 < rising.motion.vertical <= config.recovery_max_command

    shallower = controller.step(depth_valid=True, depth_m=0.10, now=10.2)
    assert shallower.motion.is_neutral()
    assert not shallower.complete
    complete = controller.step(
        depth_valid=True,
        depth_m=0.10,
        now=10.2 + config.recovery_settle_s,
    )
    assert complete.complete
    assert complete.motion.is_neutral()


def _snapshot(**changes) -> DatasetRuntimeSnapshot:
    """构造一个预解锁安全的假网关快照。"""

    base = DatasetRuntimeSnapshot(
        telemetry_age_s=0.10,
        status_age_s=0.10,
        heartbeat_valid=True,
        attitude_valid=True,
        attitude_age_s=0.10,
        telemetry_mode="ALT_HOLD",
        status_mode="ALT_HOLD",
        preflight_passed=True,
        estop_latched=False,
        state="LOCKED",
        allows_actuation=True,
        allows_arming=True,
        allows_gripper=False,
        runtime_enabled=False,
        telemetry_armed=False,
        status_armed=False,
        armed_by_ros=False,
        command_source="",
    )
    return replace(base, **changes)


def test_fake_gateway_accepts_clean_prearm_and_active_states() -> None:
    """许可前必须 LOCKED，解锁后必须是 ROS 完整控制状态。"""

    config = _config()
    assert prearm_safety_error(_snapshot(), config) is None
    active = _snapshot(
        state="READY",
        runtime_enabled=True,
        telemetry_armed=True,
        status_armed=True,
        armed_by_ros=True,
        command_source="commissioning",
    )
    assert active_safety_error(active, config) is None


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"telemetry_age_s": 2.0}, "遥测数据过期"),
        ({"telemetry_mode": "MANUAL"}, "ALT_HOLD"),
        ({"estop_latched": True}, "急停"),
        ({"command_source": "autonomy"}, "命令来源"),
        ({"allows_arming": False}, "ROS 解锁"),
    ],
)
def test_fake_gateway_rejects_every_prearm_fault(changes: dict, message: str) -> None:
    """异常模式、过期数据、急停和来源冲突都要在解锁前拒绝。"""

    error = prearm_safety_error(_snapshot(**changes), _config())
    assert error is not None
    assert message in error


def test_fake_gateway_detects_runtime_permission_and_arm_rejection() -> None:
    """服务拒绝后不能把 LOCKED/未解锁状态当成可驾驶。"""

    error = active_safety_error(_snapshot(), _config())
    assert error is not None
    assert "运行时" in error


def test_short_attitude_dropout_is_tolerated_but_long_dropout_stops() -> None:
    """偶发姿态丢包不打断采集，连续超时仍返回明确原因。"""

    config = _config()
    brief = _snapshot(attitude_valid=False, attitude_age_s=2.9)
    assert prearm_safety_error(brief, config) is None

    stale = _snapshot(attitude_valid=False, attitude_age_s=3.1)
    error = prearm_safety_error(stale, config)
    assert error is not None
    assert "姿态遥测连续无效" in error

    never_received = _snapshot(attitude_valid=False, attitude_age_s=float("inf"))
    assert "尚未收到过" in prearm_safety_error(never_received, config)


def test_recovery_timeout_and_invalid_depth_are_hard_failures() -> None:
    """回收超时或深度失效时不能盲目上升。"""

    config = replace(_config(), recovery_timeout_s=2.0)
    controller = RecoveryController(config, target_depth_m=0.20, started_at=10.0)
    with pytest.raises(DatasetControlError, match="超时"):
        controller.step(depth_valid=True, depth_m=0.50, now=12.0)
    with pytest.raises(DatasetControlError, match="无效"):
        controller.step(depth_valid=False, depth_m=0.50, now=10.1)
