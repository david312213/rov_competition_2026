from dataclasses import replace

import pytest

from blind_grab_test_helpers import config, confirmed_mission, observe
from rov_competition.blind_grab import BlindGrabMission, BlindState, GrabPhase, SnakePhase


def test_start_moves_immediately_and_no_frames_fall_back_at_exactly_30_seconds():
    mission = BlindGrabMission(config())
    first = mission.step(0.0)
    assert first.motion.forward == .23
    assert first.threshold == 4
    assert first.servos == config().open_gripper.outputs + config().arm_to_grasp.outputs
    assert not mission.step(29.999).permanent
    decision = mission.step(30.0)
    assert decision.state is BlindState.GRABBING
    assert decision.phase == "open"
    assert decision.threshold == 0
    assert decision.motion.is_neutral()


def test_trigger_frame_stops_but_five_frames_must_be_after_stopping():
    mission = BlindGrabMission(config())
    first = mission.step(0.0, observe(1, 0.0))
    assert first.state is BlindState.CONFIRMING
    assert first.motion.is_neutral()
    for i in range(1, 5):
        assert mission.step(i * .25, observe(i + 1, i * .25)).state is BlindState.CONFIRMING
    assert mission.step(1.25, observe(6, 1.25)).state is BlindState.GRABBING


def test_many_frames_in_a_short_burst_do_not_satisfy_one_second():
    mission = BlindGrabMission(config())
    mission.step(0, observe(1, 0))
    for i in range(1, 6):
        decision = mission.step(i / 10, observe(i + 1, i / 10))
        assert decision.state is BlindState.CONFIRMING
    # 等待时间本身不是新帧，不把只覆盖0.5秒的图像延长为1秒。
    assert mission.step(1.1).state is BlindState.CONFIRMING
    assert mission.step(1.2, observe(7, 1.2)).state is BlindState.GRABBING


def test_repeated_frame_id_never_refreshes_its_age_or_confirmation_count():
    mission = BlindGrabMission(config())
    mission.step(0, observe(1, 0))
    for i in range(1, 20):
        assert mission.step(i / 20, observe(1, i / 20)).state is BlindState.CONFIRMING
    assert mission.step(1.0, observe(1, 1.0)).state is BlindState.SEARCHING
    assert mission.step(30, observe(1, 30)).permanent


def test_a_missing_frame_gap_cannot_join_two_confirmation_sequences():
    mission = BlindGrabMission(config())
    mission.step(0, observe(1, 0))
    for i in range(1, 5):
        mission.step(i / 10, observe(i + 1, i / 10))
    assert mission.step(1.5, observe(6, 1.5)).state is BlindState.CONFIRMING
    assert mission.step(1.7, observe(7, 1.7)).state is BlindState.CONFIRMING
    assert mission.step(2.0, observe(8, 2.0)).state is BlindState.CONFIRMING


def test_less_than_four_resumes_and_next_candidate_requires_a_new_confirmation():
    mission = BlindGrabMission(config())
    mission.step(0)
    mission.step(2, observe(1, 2))
    mission.step(2.25, observe(2, 2.25))
    resume = mission.step(2.5, observe(3, 2.5, 3))
    assert resume.state is BlindState.SEARCHING
    assert resume.motion.forward == .23
    again = mission.step(2.75, observe(4, 2.75, 8))
    assert again.state is BlindState.CONFIRMING
    assert again.search_elapsed_s == pytest.approx(2.75)


def test_intermittent_false_positives_do_not_restart_fallback_countdown():
    mission = BlindGrabMission(config())
    for i in range(121):
        now = i / 4
        decision = mission.step(now, observe(i + 1, now, 4 if i % 2 == 0 else 3))
    assert decision.permanent
    assert decision.threshold == 0
    assert decision.phase == "open"


def test_deadline_wins_over_a_confirmation_at_the_same_tick():
    mission = BlindGrabMission(config(fallback_after_s=1.0))
    mission.step(0, observe(1, 0))
    for i in range(1, 6):
        decision = mission.step(i / 5, observe(i + 1, i / 5))
    assert decision.permanent


def test_forced_permanent_mode_starts_immediately_and_never_returns_to_search():
    mission = BlindGrabMission(config(fallback_after_s=1000))
    mission.step(0)
    mission.force_permanent(0.5)
    decision = mission.step(0.5, observe(1, 0.5, 100))
    assert decision.permanent
    assert decision.threshold == 0
    assert decision.state is BlindState.GRABBING
    assert decision.phase == "open"


def test_forced_permanent_during_a_grab_does_not_restart_the_current_batch():
    mission = confirmed_mission()
    mission.force_permanent(1.2)
    current = mission.step(1.2, observe(10, 1.2, 0))
    assert current.permanent
    assert current.batch_index == 1
    assert current.grab_in_batch == 1
    assert current.phase == "open"
    for now in range(2, 20):
        current = mission.step(float(now), observe(now + 10, float(now), 0))
    assert current.completed_batches == 1
    assert current.batch_index == 2
    assert current.phase == "open"


def test_original_snake_route_alternates_shift_and_yaw_directions():
    mission = BlindGrabMission(config(fallback_after_s=1000))
    assert mission.step(0).motion.forward == .23
    assert mission.step(20).motion.lateral == -.20
    assert mission.step(24.5).motion.yaw == -.20
    assert mission.step(36).motion.forward == .23
    assert mission.step(56).motion.lateral == .20
    assert mission.step(60.5).motion.yaw == .20
    done = mission.step(72)
    assert done.lane_index == 2
    assert done.motion.forward == .23


@pytest.mark.parametrize("phase,prepare_times,trigger_at", [
    (SnakePhase.FORWARD, [0.0], 2.0),
    (SnakePhase.SHIFT, [0.0, 5.0], 6.0),
    (SnakePhase.TURN, [0.0, 5.0, 9.5], 10.0),
])
def test_confirm_and_all_three_grabs_freeze_each_snake_phase(phase, prepare_times, trigger_at):
    mission = BlindGrabMission(config(lane_forward_duration_s=5.0))
    for now in prepare_times:
        mission.step(now)
    held = mission.step(trigger_at, observe(1, trigger_at))
    assert held.snake_phase is phase
    assert held.motion.is_neutral()
    saved_elapsed = held.snake_elapsed_s
    for i in range(1, 6):
        mission.step(trigger_at + i / 5, observe(i + 1, trigger_at + i / 5))
    assert mission.state is BlindState.GRABBING
    for offset in range(1, 19):
        now = trigger_at + 1 + offset
        d = mission.step(now, observe(6 + offset, now, 0))
        assert d.snake_phase is phase
        assert d.snake_elapsed_s == pytest.approx(saved_elapsed)
    assert d.state is BlindState.SEARCHING
    assert d.completed_cycles == 3
    assert d.search_elapsed_s == 0
    next_step = mission.step(trigger_at + 19.25)
    assert next_step.snake_elapsed_s == pytest.approx(saved_elapsed + .25)


def test_each_batch_has_three_complete_sequences_and_correct_joint_poses():
    mission = confirmed_mission()
    assert mission.grab_phase is GrabPhase.OPEN
    expected = ["open", "advance", "close", "transfer", "release", "return"] * 3
    states = [mission.step(1.0)]
    for now in range(2, 19):
        states.append(mission.step(float(now), observe(now + 100, float(now), 0)))
    assert [d.phase for d in states] == expected
    for d in states:
        assert d.motion.forward == (.23 if d.phase == "advance" else 0)
        claw, arm = d.servos
        assert claw.pwm == (1900 if d.phase in ("close", "transfer") else 1100)
        assert arm.pwm == (1800 if d.phase in ("transfer", "release") else 1200)
        assert not d.permanent
    finished = mission.step(19.0)
    assert finished.state is BlindState.SEARCHING
    assert finished.grasp_command_count == 3
    assert finished.completed_cycles == 3
    assert finished.completed_batches == 1


def test_configured_ten_grab_batch_completes_exactly_ten_cycles():
    mission = confirmed_mission(grabs_per_batch=10)
    for now in range(2, 62):
        decision = mission.step(float(now), observe(now + 100, float(now), 0))
    assert decision.state is BlindState.SEARCHING
    assert decision.grasp_command_count == 10
    assert decision.completed_cycles == 10
    assert decision.completed_batches == 1


def test_a_batch_longer_than_30_seconds_does_not_consume_search_time():
    c = config()
    changes = {name: replace(getattr(c, name), duration_s=10.0)
               for name in ("open_gripper", "close_gripper", "arm_to_basket", "arm_to_grasp")}
    mission = confirmed_mission(**changes, advance_duration_s=10, release_duration_s=10)
    for index in range(1, 19):
        d = mission.step(1.0 + 10 * index)
        assert not d.permanent
        assert d.search_elapsed_s == 0
    assert d.completed_cycles == 3
    assert d.state is BlindState.SEARCHING
    assert not mission.step(210.99).permanent
    assert mission.step(211.0).permanent


def test_post_batch_four_boxes_require_five_new_stationary_frames_again():
    mission = confirmed_mission()
    for now in range(2, 20):
        d = mission.step(float(now), observe(now + 100, float(now), 4))
    assert d.state is BlindState.CONFIRMING
    assert d.motion.is_neutral()
    assert d.batch_index == 1
    for i in range(1, 6):
        now = 19 + i / 5
        d = mission.step(now, observe(200 + i, now, 4))
    assert d.batch_index == 2
    assert d.state is BlindState.GRABBING


def test_permanent_mode_survives_visual_recovery_and_a_thousand_cycles():
    mission = BlindGrabMission(config())
    mission.step(0)
    mission.step(30)
    for i in range(1, 6001):
        now = 30.0 + i
        d = mission.step(now, observe(i, now, 0 if i % 2 else 100))
        assert d.permanent
        assert d.threshold == 0
        assert d.state is BlindState.GRABBING
    assert d.completed_cycles == 1000
    assert d.grasp_command_count == 1000
    assert d.completed_batches == 333


def test_a_late_tick_advances_one_grab_phase_without_skipping_servo_actions():
    mission = confirmed_mission()
    assert mission.step(1000).phase == "advance"
    assert mission.step(1000).phase == "advance"
    assert mission.step(1001).phase == "close"
    assert mission.grasp_command_count == 1
