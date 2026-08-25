"""搜索—接近水池测试的纯软件回归测试。"""

from dataclasses import replace
from pathlib import Path

import pytest

from rov_competition.domain import BoundingBox, Detection, MissionObservation
from rov_competition.search_approach import (
    ManualCalibrationControl,
    SearchApproachConfig,
    SearchApproachMission,
    SearchStateReporter,
    SearchTestError,
    SearchTestState,
    SearchWorkflow,
    STATE_DISPLAY_NAMES,
    load_search_approach_config,
)


PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws/src/rov_competition"
BASE = load_search_approach_config(PACKAGE / "config/search_test.yaml")
FAST = replace(
    BASE,
    bottom_detection_stable_s=0.10,
    bottom_neutral_confirmation_s=0.05,
    descent_settle_s=0.10,
    return_settle_s=0.10,
    alignment_frames=2,
)


def target(
    *,
    label: str = "echinus",
    left: float = 280.0,
    right: float = 360.0,
    top: float = 180.0,
    bottom: float = 300.0,
    confidence: float = 0.9,
) -> Detection:
    return Detection(
        class_id=0,
        label=label,
        confidence=confidence,
        box=BoundingBox(left, top, right, bottom),
    )


def observation(
    frame_id: int,
    *,
    detections=(),
    depth: float = 1.0,
    yaw: float = 350.0,
    perception_valid: bool = True,
    depth_valid: bool = True,
) -> MissionObservation:
    return MissionObservation(
        frame_id=frame_id,
        detections=tuple(detections),
        frame_width=640,
        frame_height=480,
        perception_valid=perception_valid,
        depth_valid=depth_valid,
        depth_m=depth,
        attitude_valid=True,
        yaw_deg=yaw,
    )


def started(
    config: SearchApproachConfig = FAST,
    workflow: SearchWorkflow = SearchWorkflow.AUTO_APPROACH,
) -> SearchApproachMission:
    mission = SearchApproachMission(
        config,
        ("echinus", "scallop"),
        workflow=workflow,
    )
    mission.start(
        observation(0, depth=1.0),
        descent_command=0.20,
        now=0.0,
    )
    return mission


def reach_scan(mission: SearchApproachMission) -> tuple[int, float]:
    mission.step(observation(1, depth=1.30), 0.01)
    decision = mission.step(observation(2, depth=1.30), 0.12)
    assert decision.state == SearchTestState.SCANNING
    return 2, 0.12


def lock_and_approach(mission: SearchApproachMission) -> tuple[int, float, Detection]:
    frame, now = reach_scan(mission)
    item = target()
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(observation(frame, detections=(item,), depth=1.30), now)
    assert decision.state == SearchTestState.ALIGNING
    for _ in range(2):
        frame += 1
        now += 0.05
        decision = mission.step(observation(frame, detections=(item,), depth=1.30), now)
    assert decision.state == SearchTestState.APPROACHING
    return frame, now, item


def test_config_matches_rc2_pool_parameters() -> None:
    assert BASE.bottom_detection_stable_s == pytest.approx(3.0)
    assert BASE.bottom_detection_depth_tolerance_m == pytest.approx(0.05)
    assert BASE.bottom_detection_minimum_descent_m == pytest.approx(0.10)
    assert BASE.bottom_neutral_confirmation_s == pytest.approx(1.0)
    assert BASE.perception_hold_timeout_s == pytest.approx(1.0)
    assert BASE.perception_abort_timeout_s == pytest.approx(5.0)
    assert BASE.tracking_command_hold_s == pytest.approx(0.20)
    assert BASE.scan_yaw_command == pytest.approx(0.40)
    assert BASE.scan_report_step_deg == pytest.approx(15.0)
    assert BASE.advance_forward_command == pytest.approx(0.40)
    assert BASE.advance_duration_s == pytest.approx(2.0)
    assert BASE.stop_area_ratio == pytest.approx(0.10)
    assert BASE.acquisition_required_hits == 3
    assert BASE.loss_required_frames == 5
    assert BASE.loss_confirmation_s == pytest.approx(0.8)
    assert BASE.alignment_loss_grace_frames == 2
    assert BASE.alignment_loss_grace_s == pytest.approx(0.20)
    assert BASE.finish_required_hits == 4
    assert BASE.return_gain == pytest.approx(1.0)
    assert BASE.return_minimum_command == pytest.approx(0.40)
    assert BASE.return_maximum_command == pytest.approx(0.60)


def test_perception_abort_timeout_must_follow_hold_timeout() -> None:
    with pytest.raises(SearchTestError, match="感知中止"):
        replace(
            BASE,
            perception_hold_timeout_s=3.0,
            perception_abort_timeout_s=3.0,
        )


def test_alignment_hold_and_loss_grace_must_remain_bounded() -> None:
    with pytest.raises(SearchTestError, match="对准命令短时保持"):
        replace(
            BASE,
            tracking_command_hold_s=BASE.perception_hold_timeout_s,
        )
    with pytest.raises(SearchTestError, match="对准漏检宽限时间"):
        replace(
            BASE,
            alignment_loss_grace_s=BASE.loss_confirmation_s,
        )
    with pytest.raises(SearchTestError, match="对准漏检宽限帧数"):
        replace(
            BASE,
            alignment_loss_grace_frames=BASE.loss_required_frames,
        )


def test_return_minimum_cannot_exceed_return_maximum() -> None:
    with pytest.raises(SearchTestError, match="回收最小指令"):
        replace(
            BASE,
            return_minimum_command=0.70,
            return_maximum_command=0.60,
        )


def test_bottom_detection_parameters_must_not_make_confirmation_impossible() -> None:
    with pytest.raises(SearchTestError, match="变化容差"):
        replace(
            BASE,
            bottom_detection_depth_tolerance_m=0.10,
            bottom_detection_minimum_descent_m=0.10,
        )
    with pytest.raises(SearchTestError, match="稳定时间"):
        replace(
            BASE,
            bottom_detection_stable_s=45.0,
            descent_timeout_s=45.0,
        )
    with pytest.raises(SearchTestError, match="回中确认时间"):
        replace(
            BASE,
            bottom_neutral_confirmation_s=3.0,
            bottom_detection_stable_s=3.0,
        )


def test_start_uses_operator_power_without_requesting_target_depth() -> None:
    mission = SearchApproachMission(FAST, ("echinus",))
    decision = mission.start(
        observation(7, depth=0.20),
        descent_command=0.60,
        now=1.0,
    )
    assert mission.start_depth_m == pytest.approx(0.20)
    assert decision.target_depth_m is None
    assert mission.descent_command == pytest.approx(0.60)
    descent = mission.step(observation(8, depth=0.20), 1.05)
    assert descent.motion.vertical == pytest.approx(-0.60)


@pytest.mark.parametrize("value", (0.09, 0.81, float("nan")))
def test_invalid_descent_power_is_rejected(value: float) -> None:
    mission = SearchApproachMission(FAST, ("echinus",))
    with pytest.raises(SearchTestError, match="power"):
        mission.start(
            observation(0),
            descent_command=value,
            now=0.0,
        )


def test_descent_matches_fixed_keyboard_down_command() -> None:
    """触底判定完成前始终使用操作员输入的固定下潜值。"""

    mission = SearchApproachMission(FAST, ("echinus",))
    mission.start(
        observation(0, depth=0.20),
        descent_command=0.60,
        now=0.0,
    )
    far = mission.step(observation(1, depth=0.20), 0.05)
    assert far.motion.vertical == pytest.approx(-0.60)
    near = mission.step(observation(2, depth=0.40), 0.10)
    assert near.motion.vertical == pytest.approx(-0.60)


def test_depth_plateau_records_bottom_and_enters_scan_neutral() -> None:
    mission = SearchApproachMission(FAST, ("echinus",))
    mission.start(
        observation(0, depth=0.20),
        descent_command=0.60,
        now=0.0,
    )
    moving = mission.step(observation(1, depth=0.50), 0.01)
    assert moving.state == SearchTestState.DESCENDING
    assert moving.motion.vertical == pytest.approx(-0.60)
    decision = mission.step(observation(2, depth=0.50), 0.12)
    assert decision.state == SearchTestState.SCANNING
    assert decision.motion.is_neutral()
    assert decision.target_depth_m == pytest.approx(0.50)
    assert "触底" in decision.message


def test_suspected_bottom_neutralizes_before_full_confirmation() -> None:
    """疑似触底后不再持续压池底，由 ALT_HOLD 完成剩余确认。"""

    config = replace(
        BASE,
        bottom_detection_stable_s=3.0,
        bottom_neutral_confirmation_s=1.0,
    )
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    moving = mission.step(observation(1, depth=0.60), 0.10)
    assert moving.motion.vertical == pytest.approx(-0.60)
    holding = mission.step(observation(2, depth=0.60), 1.11)
    assert holding.state == SearchTestState.DESCENDING
    assert holding.motion.is_neutral()
    assert "ALT_HOLD 定深确认" in holding.message
    confirmed = mission.step(observation(3, depth=0.60), 3.11)
    assert confirmed.state == SearchTestState.SCANNING
    assert confirmed.motion.is_neutral()


def test_state_reporter_prints_entry_one_hz_countdown_and_transition() -> None:
    """现场终端能看到进入状态、1/3、2/3、3/3 和状态切换。"""

    config = replace(
        FAST,
        bottom_detection_stable_s=3.0,
        bottom_neutral_confirmation_s=1.0,
    )
    mission = SearchApproachMission(config, ("echinus",))
    first_observation = observation(0, depth=0.20)
    first = mission.start(first_observation, descent_command=0.60, now=0.0)
    messages: list[str] = []
    reporter = SearchStateReporter(mission, emit=messages.append)
    reporter.report(first, first_observation, 0.0)

    for frame_id, now in ((1, 0.10), (2, 1.10), (3, 2.10), (4, 3.11)):
        current = observation(frame_id, depth=0.60)
        decision = mission.step(current, now)
        reporter.report(decision, current, now)

    assert messages[0].startswith("[进入状态] DESCENDING")
    assert any("疑似触底稳定=1/3s" in item for item in messages)
    assert any("疑似触底稳定=2/3s" in item for item in messages)
    three_index = next(
        index for index, item in enumerate(messages) if "疑似触底稳定=3/3s" in item
    )
    transition_index = next(
        index
        for index, item in enumerate(messages)
        if "[状态切换] DESCENDING -> SCANNING" in item
    )
    assert three_index < transition_index
    assert any(item.startswith("[进入状态] SCANNING") for item in messages)
    assert any("由 ALT_HOLD 定深完成确认" in item for item in messages)


def test_every_search_state_has_a_terminal_display_name() -> None:
    assert set(STATE_DISPLAY_NAMES) == set(SearchTestState)


def test_depth_change_inside_full_window_does_not_false_trigger_bottom() -> None:
    """不能因为相邻读数很小，就把持续慢速下潜当成池底。"""

    config = replace(
        FAST,
        bottom_detection_stable_s=3.0,
        bottom_neutral_confirmation_s=1.0,
        bottom_detection_depth_tolerance_m=0.02,
    )
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision = None
    for index in range(1, 8):
        now = index * 0.5
        decision = mission.step(
            observation(index, depth=0.20 + index * 0.01), now
        )
    assert decision is not None
    # 慢速下潜不能满足完整 3 秒平台，因此不会误切到扫描。
    assert decision.state == SearchTestState.DESCENDING


def test_bottom_detection_requires_minimum_real_descent() -> None:
    mission = SearchApproachMission(FAST, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision = mission.step(observation(1, depth=0.20), 0.20)
    assert decision.state == SearchTestState.DESCENDING
    assert decision.motion.vertical == pytest.approx(-0.60)


def test_bottom_detection_accepts_small_pressure_noise() -> None:
    config = replace(
        FAST,
        bottom_detection_stable_s=0.20,
        bottom_detection_depth_tolerance_m=0.02,
    )
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    mission.step(observation(1, depth=0.60), 0.01)
    mission.step(observation(2, depth=0.61), 0.11)
    decision = mission.step(observation(3, depth=0.60), 0.22)
    assert decision.state == SearchTestState.SCANNING
    assert decision.motion.is_neutral()


def test_default_bottom_tolerance_accepts_float32_five_centimeter_span() -> None:
    config = replace(FAST, bottom_detection_stable_s=0.20)
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    mission.step(observation(1, depth=0.600000024), 0.01)
    mission.step(observation(2, depth=0.649999976), 0.11)
    decision = mission.step(observation(3, depth=0.610000014), 0.22)
    assert decision.state == SearchTestState.SCANNING
    assert decision.motion.is_neutral()


def test_pause_time_does_not_count_towards_three_second_bottom_window() -> None:
    config = replace(
        FAST,
        bottom_detection_stable_s=3.0,
        bottom_neutral_confirmation_s=1.0,
    )
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    mission.step(observation(1, depth=0.60), 0.10)

    # 窗口失焦或感知暂停时只发中位，这 10 秒不能偷偷算作
    # “持续向下施力后深度稳定”的证据。
    mission.delay_timers(10.0)
    resumed = mission.step(observation(2, depth=0.60), 10.20)
    assert resumed.state == SearchTestState.DESCENDING
    assert resumed.motion.vertical == pytest.approx(-0.60)

    confirmed = mission.step(observation(3, depth=0.60), 13.11)
    assert confirmed.state == SearchTestState.SCANNING
    assert confirmed.motion.is_neutral()


def test_descent_aborts_at_hard_maximum_depth() -> None:
    config = replace(FAST, maximum_operation_depth_m=0.70)
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision = mission.step(observation(1, depth=0.70), 0.05)
    assert decision.state == SearchTestState.ABORTED
    assert decision.motion.is_neutral()
    assert "作业上限" in decision.message


def test_descent_timeout_aborts_if_no_bottom_plateau_is_confirmed() -> None:
    config = replace(FAST, descent_timeout_s=0.20)
    mission = SearchApproachMission(config, ("echinus",))
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision = mission.step(observation(1, depth=0.40), 0.20)
    assert decision.state == SearchTestState.ABORTED
    assert decision.motion.is_neutral()
    assert "超时" in decision.message


def test_scan_accumulates_right_turn_across_north() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    for index in range(1, 19):
        frame += 1
        now += 0.05
        yaw = (350.0 + index * 20.0) % 360.0
        decision = mission.step(observation(frame, depth=1.30, yaw=yaw), now)
    assert decision.state == SearchTestState.ADVANCING
    assert decision.motion.is_neutral()


def test_scan_continues_between_fresh_detection_frames() -> None:
    """20Hz 控制不应因为 YOLO 本周期没有新帧而一转一停。"""

    mission = started()
    frame, now = reach_scan(mission)
    first = mission.step(observation(frame + 1, depth=1.30, yaw=355.0), now + 0.05)
    assert first.motion.yaw == pytest.approx(0.40)
    repeated = mission.step(
        observation(frame + 1, depth=1.30, yaw=10.0), now + 0.10
    )
    assert repeated.state == SearchTestState.SCANNING
    assert repeated.motion.yaw == pytest.approx(0.40)
    assert mission.scan_progress_deg == pytest.approx(20.0)


def test_scan_reporter_emits_fifteen_degree_milestone() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    messages: list[str] = []
    reporter = SearchStateReporter(mission, emit=messages.append)
    initial = mission.step(observation(frame + 1, depth=1.30, yaw=355.0), now + 0.05)
    reporter.report(initial, observation(frame + 1, depth=1.30, yaw=355.0), now + 0.05)
    current = mission.step(observation(frame + 2, depth=1.30, yaw=11.0), now + 0.10)
    reporter.report(current, observation(frame + 2, depth=1.30, yaw=11.0), now + 0.10)
    assert any("已完成 15/360°" in item for item in messages)


def test_no_target_advances_at_point_four_for_two_seconds() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    mission._enter(SearchTestState.ADVANCING, observation(frame, depth=1.30), now)
    decision = mission.step(observation(frame + 1, depth=1.30), now + 0.10)
    assert decision.motion.forward == pytest.approx(0.40)
    decision = mission.step(observation(frame + 2, depth=1.30), now + 2.01)
    assert decision.state == SearchTestState.SCANNING
    assert decision.search_cycle == 2


def test_three_empty_search_cycles_return_instead_of_starting_a_fourth() -> None:
    """无目标时最多执行三轮“扫描+前进”，然后回收。"""

    mission = started()
    frame, now = reach_scan(mission)
    for expected_cycle in (1, 2, 3):
        mission._enter(SearchTestState.ADVANCING, observation(frame, depth=1.30), now)
        frame += 1
        now += BASE.advance_duration_s + 0.01
        decision = mission.step(observation(frame, depth=1.30), now)
        if expected_cycle < 3:
            assert decision.state == SearchTestState.SCANNING
            assert decision.search_cycle == expected_cycle + 1
        else:
            assert decision.state == SearchTestState.RETURNING
            assert decision.motion.is_neutral()


def test_target_requires_three_hits_in_recent_five_new_frames() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    item = target()
    patterns = (True, False, True, False, True)
    for hit in patterns:
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(item,) if hit else (), depth=1.30), now
        )
    assert decision.state == SearchTestState.ALIGNING
    assert decision.selected_target == item


def test_duplicate_frame_does_not_count_towards_acquisition() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    item = target()
    for _ in range(5):
        now += 0.05
        decision = mission.step(observation(frame + 1, detections=(item,), depth=1.30), now)
    assert decision.state == SearchTestState.SCANNING
    assert decision.motion.yaw == pytest.approx(0.40)
    assert tuple(mission.candidate_history) == (True,)


def test_invalid_or_stale_perception_stops_motion_without_counting_a_miss() -> None:
    """断流由 ROS 层分级处理；纯状态机始终只允许回中等待。"""

    mission = started()
    frame, now = reach_scan(mission)
    decision = mission.step(
        observation(frame + 1, depth=1.30, perception_valid=False), now + 0.05
    )
    assert decision.state == SearchTestState.SCANNING
    assert decision.motion.is_neutral()
    assert mission.consecutive_misses == 0


def test_largest_box_wins_before_confidence() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    small = target(confidence=0.99)
    large = target(label="scallop", left=180, right=460, top=100, bottom=380, confidence=0.55)
    mission.step(observation(frame + 1, detections=(small, large), depth=1.30), now + 0.05)
    assert mission.candidate == large


def test_abnormally_full_screen_box_is_not_a_valid_target() -> None:
    """接近整屏的异常框不能触发锁定或面积完成条件。"""

    mission = started()
    frame, now = reach_scan(mission)
    oversized = target(left=0, right=640, top=0, bottom=480)
    for _ in range(5):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(oversized,), depth=1.30), now
        )
    assert decision.state == SearchTestState.SCANNING
    assert mission.tracked is None


def test_left_box_turns_right_and_right_box_turns_left() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    left = target(left=20, right=120)
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(observation(frame, detections=(left,), depth=1.30), now)
    assert decision.state == SearchTestState.ALIGNING
    # 第 3 次命中已经完成目标确认，这张帧必须立即
    # 产生偏航命令，不应额外插入一帧停车。
    assert decision.motion.yaw > 0.0

    mission.tracked = target(left=520, right=620)
    frame += 1
    decision = mission.step(
        observation(frame, detections=(mission.tracked,), depth=1.30), now + 0.05
    )
    assert decision.motion.yaw < 0.0


def test_alignment_short_misses_hold_yaw_without_state_flapping() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    left = target(left=20, right=120)
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(left,), depth=1.30), now
        )
    expected_yaw = decision.motion.yaw
    assert expected_yaw > 0.0

    frame += 1
    first = mission.step(observation(frame, depth=1.30), now + 0.05)
    assert first.state == SearchTestState.ALIGNING
    assert first.motion.yaw == pytest.approx(expected_yaw)
    assert mission.consecutive_misses == 1

    frame += 1
    second = mission.step(observation(frame, depth=1.30), now + 0.10)
    assert second.state == SearchTestState.ALIGNING
    assert second.motion.yaw == pytest.approx(expected_yaw)
    assert mission.consecutive_misses == 2

    frame += 1
    third = mission.step(observation(frame, depth=1.30), now + 0.25)
    assert third.state == SearchTestState.VERIFYING_LOSS
    assert third.motion.is_neutral()


def test_alignment_recovery_frame_immediately_restores_yaw() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    left = target(left=20, right=120)
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(left,), depth=1.30), now
        )
    expected_yaw = decision.motion.yaw

    for elapsed in (0.05, 0.10, 0.25):
        frame += 1
        decision = mission.step(observation(frame, depth=1.30), now + elapsed)
    assert decision.state == SearchTestState.VERIFYING_LOSS
    assert decision.motion.is_neutral()

    frame += 1
    recovered = mission.step(
        observation(frame, detections=(left,), depth=1.30), now + 0.30
    )
    assert recovered.state == SearchTestState.ALIGNING
    assert recovered.motion.yaw == pytest.approx(expected_yaw)


def test_alignment_duplicate_control_ticks_hold_only_recent_yaw() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    left = target(left=20, right=120)
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(left,), depth=1.30), now
        )
    expected_yaw = decision.motion.yaw

    repeated = mission.step(
        observation(frame, detections=(left,), depth=1.30), now + 0.05
    )
    assert repeated.state == SearchTestState.ALIGNING
    assert repeated.motion.yaw == pytest.approx(expected_yaw)

    expired = mission.step(
        observation(frame, detections=(left,), depth=1.30), now + 0.21
    )
    assert expired.state == SearchTestState.ALIGNING
    assert expired.motion.is_neutral()


def test_alternating_hit_and_single_miss_can_complete_alignment() -> None:
    """命中一帧、漏一帧时，一两帧漏检不得清空居中计数。"""

    mission = started()
    frame, now = reach_scan(mission)
    centered = target()
    for hit in (True, False, True, False, True):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(
                frame,
                detections=(centered,) if hit else (),
                depth=1.30,
            ),
            now,
        )
    assert decision.state == SearchTestState.ALIGNING
    assert mission.aligned_frames == 1

    frame += 1
    miss = mission.step(observation(frame, depth=1.30), now + 0.05)
    assert miss.state == SearchTestState.ALIGNING
    assert mission.aligned_frames == 1

    frame += 1
    recovered = mission.step(
        observation(frame, detections=(centered,), depth=1.30), now + 0.10
    )
    assert recovered.state == SearchTestState.APPROACHING
    assert recovered.motion.is_neutral()


def test_first_miss_stops_and_two_misses_do_not_unlock() -> None:
    mission = started()
    frame, now, item = lock_and_approach(mission)
    frame += 1
    first = mission.step(observation(frame, detections=(), depth=1.30), now + 0.05)
    assert first.state == SearchTestState.VERIFYING_LOSS
    assert first.motion.is_neutral()
    frame += 1
    second = mission.step(observation(frame, detections=(), depth=1.30), now + 0.15)
    assert second.state == SearchTestState.VERIFYING_LOSS
    assert mission.tracked == item


def test_target_recovery_after_two_misses_realigns() -> None:
    mission = started()
    frame, now, item = lock_and_approach(mission)
    for elapsed in (0.05, 0.15):
        frame += 1
        mission.step(observation(frame, detections=(), depth=1.30), now + elapsed)
    frame += 1
    decision = mission.step(observation(frame, detections=(item,), depth=1.30), now + 0.20)
    assert decision.state == SearchTestState.ALIGNING
    assert decision.motion.is_neutral()


def test_five_misses_and_point_eight_seconds_confirm_loss() -> None:
    mission = started()
    frame, now, _item = lock_and_approach(mission)
    for elapsed in (0.05, 0.20, 0.40, 0.60, 0.81):
        frame += 1
        decision = mission.step(observation(frame, detections=(), depth=1.30), now + elapsed)
    assert decision.state == SearchTestState.SCANNING
    assert mission.tracked is None


def test_loss_requires_both_five_frames_and_point_eight_seconds() -> None:
    mission = started()
    frame, now, item = lock_and_approach(mission)
    # 五帧很快到达，但时间还未达到 0.8 秒，仍保持锁定并停车。
    for elapsed in (0.05, 0.10, 0.15, 0.20, 0.25):
        frame += 1
        decision = mission.step(
            observation(frame, detections=(), depth=1.30), now + elapsed
        )
    assert decision.state == SearchTestState.VERIFYING_LOSS
    assert mission.tracked == item

    # 时间达到后还必须由一张新帧触发确认。
    frame += 1
    decision = mission.step(
        observation(frame, detections=(), depth=1.30), now + 0.81
    )
    assert decision.state == SearchTestState.SCANNING


def test_repeated_empty_frame_cannot_confirm_loss() -> None:
    mission = started()
    frame, now, item = lock_and_approach(mission)
    frame += 1
    for elapsed in (0.05, 0.20, 0.40, 0.60):
        decision = mission.step(observation(frame, detections=(), depth=1.30), now + elapsed)
    assert decision.state == SearchTestState.VERIFYING_LOSS
    assert mission.tracked == item


def test_approach_speed_changes_at_area_point_zero_seven() -> None:
    mission = started()
    frame, now, _item = lock_and_approach(mission)
    far = target(left=250, right=390, top=160, bottom=280)  # 0.055
    frame += 1
    decision = mission.step(observation(frame, detections=(far,), depth=1.30), now + 0.05)
    assert decision.motion.forward == pytest.approx(0.20)
    near = target(left=220, right=420, top=170, bottom=300)  # 0.085
    # 面积位于 0.07..0.10，命令切换到近距离速度。
    frame += 1
    decision = mission.step(observation(frame, detections=(near,), depth=1.30), now + 0.10)
    assert decision.motion.forward == pytest.approx(0.10)


def test_finish_requires_four_of_five_new_centered_near_frames() -> None:
    mission = started()
    frame, now, _item = lock_and_approach(mission)
    near = target(left=200, right=440, top=140, bottom=300)  # area 0.125
    results = (True, True, False, True, True)
    for index, good in enumerate(results, start=1):
        frame += 1
        now += 0.05
        # 失败帧仍是同一个且已居中的目标，只是面积尚未达标。
        item = near if good else target(left=230, right=410, top=170, bottom=290)
        decision = mission.step(observation(frame, detections=(item,), depth=1.30), now)
        if index < 5:
            assert decision.state != SearchTestState.RETURNING
        if good:
            assert decision.motion.forward == pytest.approx(0.0)
    assert decision.state == SearchTestState.RETURNING
    assert decision.motion.is_neutral()


def test_return_only_ascends_and_never_redescends() -> None:
    mission = started()
    mission.request_normal_finish(observation(1, depth=1.50), 1.0)
    ascending = mission.step(observation(2, depth=1.50), 1.05)
    assert ascending.motion.vertical == pytest.approx(0.50)
    far = mission.step(observation(3, depth=1.80), 1.10)
    assert far.motion.vertical == pytest.approx(0.60)
    near = mission.step(observation(4, depth=1.10), 1.15)
    assert near.motion.vertical == pytest.approx(0.40)
    shallower = mission.step(observation(5, depth=0.90), 1.20)
    assert shallower.motion.is_neutral()


def test_invalid_depth_aborts_instead_of_blind_return() -> None:
    mission = started()
    mission.request_normal_finish(observation(1, depth=1.50), 1.0)
    decision = mission.step(observation(2, depth=1.50, depth_valid=False), 1.05)
    assert decision.state == SearchTestState.ABORTED
    assert decision.motion.is_neutral()


def test_robot_limit_must_cover_point_six_return_command() -> None:
    assert BASE.readiness_errors(robot_command_limit=0.60) == ()
    assert "0.60" in BASE.readiness_errors(robot_command_limit=0.40)[0]


def test_robot_limit_must_cover_operator_descent_power() -> None:
    assert BASE.readiness_errors(
        robot_command_limit=0.80,
        descent_command=0.80,
    ) == ()
    error = BASE.readiness_errors(
        robot_command_limit=0.40,
        descent_command=0.60,
    )
    assert error and "0.60" in error[0]


def test_focus_pause_does_not_consume_advance_duration() -> None:
    mission = started()
    frame, now = reach_scan(mission)
    mission._enter(SearchTestState.ADVANCING, observation(frame, depth=1.30), now)
    mission.delay_timers(10.0)
    decision = mission.step(observation(frame + 1, depth=1.30), now + 10.5)
    assert decision.state == SearchTestState.ADVANCING
    assert decision.motion.forward == pytest.approx(0.40)


def test_manual_workflow_stops_after_alignment_instead_of_approaching() -> None:
    """两种流程共用搜索和对准，但人工模式不得自动向目标前进。"""

    mission = started(workflow=SearchWorkflow.MANUAL_GRASP_CALIBRATION)
    frame, now = reach_scan(mission)
    item = target()
    for _ in range(3):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(item,), depth=1.30), now
        )
    assert decision.state == SearchTestState.ALIGNING
    for _ in range(2):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(item,), depth=1.30), now
        )
    assert decision.state == SearchTestState.MANUAL_CALIBRATION
    assert decision.motion.is_neutral()
    assert "人工" in decision.message


def test_manual_calibration_keys_release_opposites_and_power_bounds() -> None:
    """人工精调沿用已验收 WASD 映射，并严格限制 power。"""

    controls = ManualCalibrationControl(FAST)
    assert controls.strength == pytest.approx(0.10)
    controls.press("w")
    controls.press("d")
    controls.press("2")
    controls.press("down")
    motion = controls.motion()
    assert motion.forward == pytest.approx(0.10)
    assert motion.lateral == pytest.approx(0.10)
    assert motion.yaw == pytest.approx(0.10)
    assert motion.vertical == pytest.approx(-0.10)

    controls.press("s")
    assert controls.motion().forward == 0.0
    controls.release("s")
    assert controls.motion().forward == pytest.approx(0.10)
    controls.clear()
    assert controls.motion().is_neutral()

    for _ in range(100):
        controls.adjust(1)
    assert controls.strength == pytest.approx(FAST.manual_maximum_command)
    for _ in range(100):
        controls.adjust(-1)
    assert controls.strength == pytest.approx(FAST.manual_minimum_command)


def test_manual_rescan_returns_to_search_depth_before_new_scan() -> None:
    """R 不直接转圈；先闭环回到本轮下潜深度，再清空目标重搜。"""

    mission = started(workflow=SearchWorkflow.MANUAL_GRASP_CALIBRATION)
    frame, now = reach_scan(mission)
    item = target()
    for _ in range(5):
        frame += 1
        now += 0.05
        decision = mission.step(
            observation(frame, detections=(item,), depth=1.30), now
        )
    assert decision.state == SearchTestState.MANUAL_CALIBRATION

    decision = mission.request_rescan(
        observation(frame + 1, detections=(item,), depth=1.45), now + 0.05
    )
    assert decision.state == SearchTestState.RESETTING_FOR_RESCAN
    assert decision.motion.is_neutral()
    returning = mission.step(observation(frame + 2, depth=1.45), now + 0.10)
    assert returning.motion.vertical > 0.0
    mission.step(observation(frame + 3, depth=1.30), now + 0.15)
    restarted = mission.step(observation(frame + 4, depth=1.30), now + 0.30)
    assert restarted.state == SearchTestState.SCANNING
    assert restarted.motion.is_neutral()


def test_manual_workflow_readiness_includes_manual_power_ceiling() -> None:
    assert BASE.readiness_errors(
        robot_command_limit=0.39,
        workflow=SearchWorkflow.MANUAL_GRASP_CALIBRATION,
    )
    assert BASE.readiness_errors(
        robot_command_limit=0.60,
        workflow=SearchWorkflow.MANUAL_GRASP_CALIBRATION,
    ) == ()
