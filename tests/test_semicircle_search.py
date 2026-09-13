from __future__ import annotations

import pytest

from rov_competition.domain import BoundingBox, Detection, MissionObservation
from rov_competition.semicircle_search import SearchAction, SearchConfig, SearchState, SemicircleSearchMission


def _config() -> SearchConfig:
    return SearchConfig(
        target_label="scallop", target_confidence=.18, target_required_hits=2, target_minimum_iou=.05,
        stop_on_stable_target=True,
        forward_command=.13, shift_command=.20, shift_duration_s=4.5,
        small_yaw_left_command=.15, small_yaw_left_duration_s=4.06,
        small_yaw_right_command=.15, small_yaw_right_duration_s=4.31,
        turn_command=.20, turn_duration_s=11.5, descent_command=.315, ascent_command=.315,
        probe_timeout_s=45., minimum_descent_m=.10, stable_depth_tolerance_m=.05,
        stable_duration_s=3., neutral_confirmation_s=1., clearance_m=.10,
        clearance_tolerance_m=.02, clearance_settle_s=1., clearance_timeout_s=15.,
        lane_forward_duration_s=20., search_duration_s=600.,
    )


def _observation(*, frame_id: int = 0, depth: float = 1., detections=()):
    return MissionObservation(frame_id, detections, 1920, 1080, True, True, depth, True, 0.)


def _mission_searching() -> SemicircleSearchMission:
    mission = SemicircleSearchMission(_config())
    mission.state = SearchState.SEARCHING
    mission.height_established = True
    mission.search_started_at = 0.
    mission.lane_started_at = 0.
    return mission


def test_measured_motion_values_are_used_for_cruise_and_touch_probe():
    mission = SemicircleSearchMission(_config())
    assert mission.step(_observation(), 0., SearchAction.TOUCH_AND_START).motion.vertical == pytest.approx(-.315)
    mission.state = SearchState.SEARCHING
    mission.height_established = True
    mission.search_started_at = 0.
    mission.lane_started_at = 0.
    assert mission.step(_observation(), 1.).motion.forward == pytest.approx(.13)


def test_pause_cannot_start_search_before_touch_and_clearance():
    mission = SemicircleSearchMission(_config())
    decision = mission.step(_observation(), 0., SearchAction.PAUSE_TOGGLE)
    assert decision.state is SearchState.PAUSED
    assert decision.motion.is_neutral()


def test_small_correction_is_timed_then_returns_to_continuous_search():
    mission = _mission_searching()
    assert mission.step(_observation(), 0., SearchAction.CORRECT_LEFT).motion.yaw == pytest.approx(-.15)
    assert mission.step(_observation(), 4.05).state is SearchState.CORRECTING
    done = mission.step(_observation(), 4.06)
    assert done.state is SearchState.SEARCHING
    assert done.motion.forward == pytest.approx(.13)


def test_next_lane_shifts_then_turns_then_reverses_direction():
    mission = _mission_searching()
    assert mission.step(_observation(), 0., SearchAction.NEXT_LANE).motion.lateral == pytest.approx(-.20)
    assert mission.step(_observation(), 4.5).motion.yaw == pytest.approx(-.20)
    done = mission.step(_observation(), 16.)
    assert done.state is SearchState.SEARCHING
    assert done.lane_index == 1
    assert done.lane_direction == "reverse"
    assert done.motion.forward == pytest.approx(.13)


def test_lane_and_total_duration_automatically_progress_or_surface():
    mission = _mission_searching()
    mission.height_established = True
    mission.search_started_at = 0.
    mission.lane_started_at = 0.
    lane_end = mission.step(_observation(), 20.)
    assert lane_end.state is SearchState.LANE_SHIFTING
    assert lane_end.motion.lateral == pytest.approx(-.20)

    mission = _mission_searching()
    mission.height_established = True
    mission.search_started_at = 0.
    mission.lane_started_at = 0.
    surfaced = mission.step(_observation(), 600.)
    assert surfaced.state is SearchState.SURFACING
    assert surfaced.motion.vertical == pytest.approx(.315)


def test_stable_scallop_stops_and_does_not_call_a_grasp_path():
    mission = _mission_searching()
    detection = Detection(2, "scallop", .90, BoundingBox(100, 100, 200, 200))
    assert mission.step(_observation(frame_id=1, detections=(detection,)), 1.).state is SearchState.SEARCHING
    held = mission.step(_observation(frame_id=2, detections=(detection,)), 2.)
    assert held.state is SearchState.TARGET_HELD
    assert held.motion.is_neutral()


def test_stable_scallop_does_not_stop_when_target_stop_is_disabled():
    config = SearchConfig(**{**_config().__dict__, "stop_on_stable_target": False})
    mission = SemicircleSearchMission(config)
    mission.state = SearchState.SEARCHING
    mission.height_established = True
    mission.search_started_at = mission.lane_started_at = 0.
    detection = Detection(2, "scallop", .90, BoundingBox(100, 100, 200, 200))
    decision = mission.step(_observation(frame_id=1, detections=(detection,)), 1.)
    assert decision.state is SearchState.SEARCHING
    assert decision.motion.forward == pytest.approx(.13)


def test_surface_has_priority_and_only_outputs_ascent():
    mission = _mission_searching()
    decision = mission.step(_observation(), 0., SearchAction.SURFACE)
    assert decision.state is SearchState.SURFACING
    assert decision.motion.vertical == pytest.approx(.315)
    assert decision.motion.forward == 0


def test_invalid_motion_configuration_is_rejected():
    with pytest.raises(ValueError, match="不能大于 1"):
        SearchConfig(**{**_config().__dict__, "ascent_command": 1.1})
