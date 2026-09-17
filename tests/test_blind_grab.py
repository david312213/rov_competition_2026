from dataclasses import replace

import pytest
from blind_grab_test_helpers import config, depth, fallback_to_first_grab
from rov_competition.blind_grab import (
    BlindGrabMission,
    BlindState,
    GrabPhase,
    ServoAction,
    ServoSetpoint,
)


def phase_duration(mission):
    c = mission.config
    if mission.state is BlindState.GRABBING:
        return {
            GrabPhase.OPEN: c.open_gripper.duration_s,
            GrabPhase.ADVANCE: c.advance_duration_s,
            GrabPhase.CLOSE: c.close_gripper.duration_s,
            GrabPhase.TRANSFER: c.arm_to_basket.duration_s,
            GrabPhase.RELEASE: c.release_duration_s,
            GrabPhase.RETURN: c.arm_to_grasp.duration_s,
        }[mission.grab_phase]
    return {
        BlindState.INITIAL_DESCENT: c.initial_fallback_s,
        BlindState.ASCENDING: c.ascent_duration_s,
        BlindState.FORWARD: c.route_step_duration_s,
        BlindState.SHIFT: c.shift_duration_s,
        BlindState.TURN: c.turn_duration_s,
        BlindState.REPEAT_DESCENT: c.repeat_descent_duration_s,
    }[mission.state]


def advance_one_phase(mission, now):
    now += phase_duration(mission) + 1e-6
    return now, mission.step(now)


def test_start_immediately_descends_with_open_claw_and_arm_at_grasp_pose():
    c = config()
    decision = BlindGrabMission(c).step(0.0)
    assert decision.state is BlindState.INITIAL_DESCENT
    assert decision.motion.vertical == pytest.approx(-0.415)
    assert decision.servos == c.open_gripper.outputs + c.arm_to_grasp.outputs
    assert decision.bottom_source == "pending"


def test_three_second_depth_plateau_after_real_descent_starts_first_grab():
    mission = BlindGrabMission(config())
    mission.step(0.0, depth(0.0, 1.00))
    mission.step(0.1, depth(0.1, 1.20))
    mission.step(1.1, depth(1.1, 1.21))
    mission.step(2.1, depth(2.1, 1.19))
    decision = mission.step(3.1, depth(3.1, 1.20))
    assert decision.state is BlindState.GRABBING
    assert decision.phase == "open"
    assert decision.motion.is_neutral()
    assert decision.bottom_source == "pressure"
    assert decision.bottom_depth_m == pytest.approx(1.20)
    assert decision.bottom_stable_s == pytest.approx(3.0)
    assert decision.initial_descent_elapsed_s == pytest.approx(3.1)


def test_slowly_changing_depth_does_not_false_trigger_before_timer_fallback():
    mission = BlindGrabMission(config())
    for second in range(10):
        decision = mission.step(float(second), depth(float(second), 1.0 + .02 * second))
        assert decision.state is BlindState.INITIAL_DESCENT
    decision = mission.step(10.0, depth(10.0, 1.20))
    assert decision.state is BlindState.GRABBING
    assert decision.bottom_source == "timer"
    assert decision.initial_descent_elapsed_s == 10.0


def test_missing_or_duplicate_depth_uses_exact_ten_second_fallback():
    mission = BlindGrabMission(config())
    mission.step(0.0, depth(0.0, 1.0))
    for now in (1.0, 3.0, 9.999):
        decision = mission.step(now, depth(0.0, 9.0))
        assert decision.state is BlindState.INITIAL_DESCENT
    decision = mission.step(10.0)
    assert decision.state is BlindState.GRABBING
    assert decision.bottom_source == "timer"
    assert decision.bottom_depth_m == 1.0


def test_one_grab_runs_all_six_phases_then_ascends_for_two_seconds():
    c = config()
    mission = BlindGrabMission(c)
    now = fallback_to_first_grab(mission)
    states = [mission.step(now)]
    for _ in range(6):
        now, decision = advance_one_phase(mission, now)
        states.append(decision)
    assert [d.phase for d in states[:-1]] == [
        "open", "advance", "close", "transfer", "release", "return",
    ]
    assert states[1].motion.forward == .23
    assert states[2].servos == c.close_gripper.outputs + c.arm_to_grasp.outputs
    assert states[3].servos == c.close_gripper.outputs + c.arm_to_basket.outputs
    assert states[4].servos == c.open_gripper.outputs + c.arm_to_basket.outputs
    assert states[5].servos == c.open_gripper.outputs + c.arm_to_grasp.outputs
    ascending = states[-1]
    assert ascending.state is BlindState.ASCENDING
    assert ascending.motion.vertical == pytest.approx(.415)
    assert ascending.completed_cycles == 1
    assert ascending.grasp_command_count == 1


def test_late_tick_advances_only_one_servo_phase():
    mission = BlindGrabMission(config())
    fallback_to_first_grab(mission)
    assert mission.step(1000.0).phase == "advance"
    assert mission.step(1000.0).phase == "advance"
    assert mission.step(1001.0).phase == "close"
    assert mission.grasp_command_count == 1


def test_each_normal_cycle_ascends_moves_five_seconds_and_descends_five_seconds():
    mission = BlindGrabMission(config())
    now = fallback_to_first_grab(mission)
    while mission.state is BlindState.GRABBING:
        now, decision = advance_one_phase(mission, now)
    assert decision.state is BlindState.ASCENDING
    now, decision = advance_one_phase(mission, now)
    assert decision.state is BlindState.FORWARD
    assert decision.motion.forward == .23
    now, decision = advance_one_phase(mission, now)
    assert decision.state is BlindState.REPEAT_DESCENT
    assert decision.segment_in_lane == 1
    assert decision.motion.vertical == pytest.approx(-.415)
    now, decision = advance_one_phase(mission, now)
    assert decision.state is BlindState.GRABBING
    assert decision.phase == "open"


def test_four_forward_steps_then_shift_and_turn_with_alternating_directions():
    fast = config(
        open_gripper=replace(config().open_gripper, duration_s=.1),
        close_gripper=replace(config().close_gripper, duration_s=.1),
        arm_to_basket=replace(config().arm_to_basket, duration_s=.1),
        arm_to_grasp=replace(config().arm_to_grasp, duration_s=.1),
        advance_duration_s=.1,
        release_duration_s=.1,
        initial_fallback_s=.1,
        ascent_duration_s=.1,
        route_step_duration_s=.1,
        repeat_descent_duration_s=.1,
        shift_duration_s=.1,
        turn_duration_s=.1,
    )
    mission = BlindGrabMission(fast)
    now = fallback_to_first_grab(mission)
    shifts = []
    turns = []
    while len(turns) < 2:
        now, decision = advance_one_phase(mission, now)
        if decision.state is BlindState.SHIFT:
            shifts.append((decision.lane_index, decision.motion.lateral))
        elif decision.state is BlindState.TURN:
            turns.append((decision.lane_index, decision.motion.yaw))
    assert shifts == [(0, -.20), (1, .20)]
    assert turns == [(0, -.20), (1, .20)]
    assert mission.completed_cycles > 4


def test_depth_changes_after_first_grab_never_change_timed_route():
    first = BlindGrabMission(config())
    second = BlindGrabMission(config())
    now = 0.0
    first.step(now)
    second.step(now, depth(now, 1.0))
    for index in range(1, 300):
        now += .25
        a = first.step(now)
        b = second.step(now, depth(now, (-1) ** index * 1000))
        assert (a.state, a.phase, a.motion) == (b.state, b.phase, b.motion)


def test_permanent_route_completes_one_thousand_grabs_without_end_state():
    def action(channel, pwm):
        return ServoAction((ServoSetpoint(channel, pwm),), .001)

    fast = config(
        open_gripper=action(20, 1100),
        close_gripper=action(20, 1900),
        arm_to_basket=action(21, 1800),
        arm_to_grasp=action(21, 1200),
        advance_duration_s=.001,
        release_duration_s=.001,
        initial_fallback_s=.001,
        ascent_duration_s=.001,
        route_step_duration_s=.001,
        repeat_descent_duration_s=.001,
        shift_duration_s=.001,
        turn_duration_s=.001,
    )
    mission = BlindGrabMission(fast)
    now = 0.0
    mission.step(now)
    while mission.completed_cycles < 1000:
        now, decision = advance_one_phase(mission, now)
    assert decision.state is BlindState.ASCENDING
    assert mission.completed_cycles == mission.grasp_command_count == 1000
    assert not hasattr(BlindState, "FINISHED")
