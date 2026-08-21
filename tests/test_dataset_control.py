"""数据集键盘控制、深度保护和假网关回归测试。"""

from dataclasses import replace
from pathlib import Path

import pytest
from rov_competition.config import load_dataset_config
from rov_competition.dataset_control import (
    DatasetControlError,
    DatasetRuntimeSnapshot,
    DepthSafetyController,
    KeyCommandState,
    RecoveryController,
    active_safety_error,
    motion_from_keys,
    prearm_safety_error,
    stable_start_depth,
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
    """返回已填写水池绝对深度的测试配置。"""

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

    assert motion_from_keys({key}, 0.20, 0.30) == expected


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
        abs(value) <= 0.30
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
    assert state.strength == pytest.approx(0.30)
    for _ in range(30):
        state.adjust(-1)
    assert state.strength == pytest.approx(0.05)


def test_start_depth_gate_is_disabled_for_dataset_collection() -> None:
    """采集模式记录深度，但不因浅水或短时波动拒绝启动。"""

    config = _config()
    assert stable_start_depth([0.20, 0.21, 0.19], config) == pytest.approx(0.20)
    assert stable_start_depth([0.01, 0.02], config) == pytest.approx(0.015)
    assert stable_start_depth([0.20, 0.30], config) == pytest.approx(0.25)

    gated = replace(config, check_start_depth_at_start=True)
    with pytest.raises(DatasetControlError, match="完全浸没"):
        stable_start_depth([0.01, 0.02], gated)
    with pytest.raises(DatasetControlError, match="尚未稳定"):
        stable_start_depth([0.20, 0.30], gated)


def test_absolute_and_relative_depth_limits_choose_the_shallower_one() -> None:
    """绝对上限与启动后 1.40 m 上限中取较浅者。"""

    relative_first = DepthSafetyController(_config(), start_depth_m=0.20)
    assert relative_first.depth_limit_m == pytest.approx(1.40)

    absolute_config = replace(_config(), maximum_depth_m=0.55)
    absolute_first = DepthSafetyController(absolute_config, start_depth_m=0.20)
    assert absolute_first.depth_limit_m == pytest.approx(0.55)


def test_depth_guard_blocks_descent_and_recovers_if_already_over_limit() -> None:
    """进入余量时只禁止下潜；已越界则停水平轴并小幅上升。"""

    guard = DepthSafetyController(_config(), start_depth_m=0.20)
    limited = guard.apply(
        MotionCommand(forward=0.05, vertical=-0.05),
        depth_valid=True,
        depth_m=1.36,
    )
    assert limited.limited
    assert limited.motion.forward == pytest.approx(0.05)
    assert limited.motion.vertical == 0.0

    over = guard.apply(
        MotionCommand(forward=0.05, yaw=0.05),
        depth_valid=True,
        depth_m=1.42,
    )
    assert over.over_limit
    assert over.motion.forward == 0.0
    assert over.motion.yaw == 0.0
    assert 0.0 < over.motion.vertical <= 0.05

    with pytest.raises(DatasetControlError, match="深度数据无效"):
        guard.apply(MotionCommand(), depth_valid=False, depth_m=0.0)


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
        depth_valid=True,
        depth_m=0.20,
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
        ({"depth_valid": False}, "深度反馈无效"),
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


def test_recovery_timeout_and_invalid_depth_are_hard_failures() -> None:
    """回收超时或深度失效时不能盲目上升。"""

    config = replace(_config(), recovery_timeout_s=2.0)
    controller = RecoveryController(config, target_depth_m=0.20, started_at=10.0)
    with pytest.raises(DatasetControlError, match="超时"):
        controller.step(depth_valid=True, depth_m=0.50, now=12.0)
    with pytest.raises(DatasetControlError, match="无效"):
        controller.step(depth_valid=False, depth_m=0.50, now=10.1)
