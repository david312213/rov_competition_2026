"""群体盲抓状态机的纯软件测试。"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from rov_competition.cluster_collection import (
    ClusterCollectionConfig,
    ClusterCollectionMission,
    ClusterCollectionState,
    DescentTrigger,
    cluster_scallops,
    exact_union_area_ratio,
    load_cluster_collection_config,
    select_cluster,
)
from rov_competition.domain import BoundingBox, Detection, MissionObservation


PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws/src/rov_competition"
FIELD = load_cluster_collection_config(PACKAGE / "config/cluster_collection.yaml")
# This suite retains regression coverage for the legacy group-center workflow.
BASE = replace(FIELD, descent_trigger=DescentTrigger.IMAGE_LINE)


def observation(
    frame_id: int,
    *,
    detections: tuple[Detection, ...] = (),
    depth: float = 0.50,
    yaw: float = 0.0,
) -> MissionObservation:
    return MissionObservation(
        frame_id=frame_id,
        detections=detections,
        frame_width=1000,
        frame_height=1000,
        perception_valid=True,
        depth_valid=True,
        depth_m=depth,
        attitude_valid=True,
        yaw_deg=yaw,
    )


def scallop_group(
    count: int,
    *,
    center_x: float = 0.50,
    center_y: float = 0.50,
    spacing: float = 0.035,
) -> tuple[Detection, ...]:
    result = []
    start = center_x - spacing * (count - 1) / 2.0
    for index in range(count):
        x = (start + index * spacing) * 1000.0
        y = (center_y + (index % 2 - 0.5) * 0.02) * 1000.0
        result.append(
            Detection(
                class_id=2,
                label="scallop",
                confidence=0.90 - index * 0.01,
                box=BoundingBox(x - 15, y - 15, x + 15, y + 15),
            )
        )
    return tuple(result)


def scanning_mission(config: ClusterCollectionConfig = BASE) -> ClusterCollectionMission:
    mission = ClusterCollectionMission(config)
    first = observation(0, depth=0.20)
    mission.start(first, descent_command=0.60, now=0.0)
    mission._enter(ClusterCollectionState.SCANNING, first, 0.0)
    return mission


def acquire(
    mission: ClusterCollectionMission,
    *,
    center_x: float = 0.50,
    center_y: float = 0.50,
) -> float:
    now = 0.0
    for frame_id in range(1, 4):
        now += 0.1
        mission.step(
            observation(
                frame_id,
                detections=scallop_group(6, center_x=center_x, center_y=center_y),
            ),
            now,
        )
    assert mission.tracked_cluster is not None
    return now


def test_default_config_matches_field_plan() -> None:
    assert BASE.minimum_cluster_count == 2
    assert ClusterCollectionConfig().minimum_cluster_count == 2
    assert BASE.acquisition_window_frames == 5
    assert BASE.acquisition_required_hits == 3
    assert BASE.bottom_clearance_m == pytest.approx(0.15)
    assert BASE.search_advance_command == pytest.approx(0.40)
    assert BASE.search_advance_duration_s == pytest.approx(2.0)
    assert BASE.grabs_per_cluster == 3
    assert BASE.mission_timeout_s == pytest.approx(900.0)
    assert BASE.descent_trigger == DescentTrigger.IMAGE_LINE
    assert FIELD.descent_trigger == DescentTrigger.TARGET_BOTTOM_LINE


def test_exact_union_area_does_not_double_count_overlap() -> None:
    first = BoundingBox(0, 0, 100, 100)
    second = BoundingBox(50, 0, 150, 100)
    assert exact_union_area_ratio((first, second), 1000, 1000) == pytest.approx(
        0.015
    )


def test_single_clearance_command_crosses_deadzone_and_keeps_protections() -> None:
    for config in (BASE, ClusterCollectionConfig()):
        assert config.bottom_clearance_up_command == pytest.approx(0.35)
        assert config.bottom_clearance_m == pytest.approx(0.15)
        assert config.bottom_clearance_tolerance_m == pytest.approx(0.02)
        assert config.bottom_clearance_settle_s == pytest.approx(1.0)
        assert config.bottom_clearance_timeout_s == pytest.approx(15.0)


def test_clearance_uses_one_command_until_target_then_goes_neutral() -> None:
    mission = ClusterCollectionMission(BASE)
    first = observation(0, depth=0.20)
    mission.start(first, descent_command=0.40, now=0.0)
    mission.bottom_depth_m = 0.50
    mission.target_depth_m = 0.35
    mission._enter(ClusterCollectionState.CLEARING_BOTTOM, first, 1.0)
    for frame_id, (depth, expected) in enumerate(
        ((0.50, 0.35), (0.41, 0.35), (0.40, 0.35), (0.38, 0.35), (0.36, 0.0)),
        start=1,
    ):
        decision = mission.step(observation(frame_id, depth=depth), 1.0 + frame_id * 0.1)
        assert decision.state == ClusterCollectionState.CLEARING_BOTTOM
        assert decision.motion.vertical == pytest.approx(expected)
        assert decision.motion.forward == 0.0
        assert decision.motion.lateral == 0.0
        assert decision.motion.yaw == 0.0
    decision = mission.step(observation(6, depth=0.36), 2.6)
    assert decision.state == ClusterCollectionState.SCANNING
    assert decision.motion.is_neutral()


def test_stable_clearance_overshoot_still_accepts_without_descending_or_aborting() -> None:
    """User rejected a new upper-clearance limit; preserve the old acceptance."""
    mission = ClusterCollectionMission(BASE)
    first = observation(0, depth=0.20)
    mission.start(first, descent_command=0.40, now=0.0)
    mission.bottom_depth_m = 0.50
    mission.target_depth_m = 0.35
    mission._enter(ClusterCollectionState.CLEARING_BOTTOM, first, 1.0)
    decision = mission.step(observation(1, depth=0.22), 1.1)
    assert decision.state == ClusterCollectionState.CLEARING_BOTTOM
    assert decision.motion.is_neutral()
    decision = mission.step(observation(2, depth=0.22), 2.2)
    assert decision.state == ClusterCollectionState.SCANNING
    assert decision.motion.is_neutral()


def test_spatial_components_and_candidate_priority() -> None:
    left = scallop_group(6, center_x=0.25)
    right = scallop_group(7, center_x=0.75)
    clusters = cluster_scallops(left + right, 1000, 1000)
    assert sorted(item.count for item in clusters) == [6, 7]
    selected = select_cluster(clusters, minimum_count=6, aim_x_ratio=0.5)
    assert selected is not None
    assert selected.count == 7


@pytest.mark.parametrize("minimum_count", (2, 6))
def test_cluster_requires_configured_count_and_three_fresh_hits(minimum_count: int) -> None:
    mission = scanning_mission(replace(BASE, minimum_cluster_count=minimum_count))
    for frame_id in range(1, 7):
        decision = mission.step(
            observation(frame_id, detections=scallop_group(minimum_count - 1)), frame_id * 0.1
        )
        assert decision.state == ClusterCollectionState.SCANNING
    for frame_id in range(7, 10):
        decision = mission.step(
            observation(frame_id, detections=scallop_group(minimum_count)), frame_id * 0.1
        )
        if frame_id < 9:
            assert decision.state == ClusterCollectionState.SCANNING
    assert decision.state == ClusterCollectionState.ALIGNING
    assert decision.cluster is not None and decision.cluster.count == minimum_count


def test_two_distant_scallops_do_not_form_one_eligible_group() -> None:
    mission = scanning_mission()
    detections = scallop_group(1, center_x=0.20) + scallop_group(1, center_x=0.80)
    for frame_id in range(1, 7):
        decision = mission.step(observation(frame_id, detections=detections), frame_id * 0.1)
        assert decision.state == ClusterCollectionState.SCANNING
        assert mission.tracked_cluster is None


def test_scan_message_uses_configured_cluster_threshold() -> None:
    mission = scanning_mission(replace(BASE, scan_angle_deg=30.0))
    mission.previous_yaw_deg = 0.0
    decision = mission.step(observation(1, yaw=30.0), 0.1)
    assert decision.state == ClusterCollectionState.SEARCH_ADVANCING
    assert "至少 2 个扇贝" in decision.message


def test_locked_group_may_fall_below_six_without_switching_target() -> None:
    mission = scanning_mission(replace(BASE, minimum_cluster_count=6))
    now = acquire(mission)
    decision = mission.step(
        observation(4, detections=scallop_group(3, center_x=0.51)), now + 0.1
    )
    assert decision.state in {
        ClusterCollectionState.ALIGNING,
        ClusterCollectionState.APPROACHING,
    }
    assert decision.cluster is not None
    assert decision.cluster.count == 3


def test_first_complete_miss_stops_then_five_frames_and_point_eight_seconds_lose() -> None:
    mission = scanning_mission()
    now = acquire(mission)
    first = mission.step(observation(4), now + 0.1)
    assert first.state == ClusterCollectionState.VERIFYING_LOSS
    assert first.motion.is_neutral()
    # 重复帧不增加丢失计数。
    repeated = mission.step(observation(4), now + 0.5)
    assert mission.consecutive_misses == 1
    assert repeated.state == ClusterCollectionState.VERIFYING_LOSS
    for index, elapsed in enumerate((0.3, 0.5, 0.7), start=5):
        decision = mission.step(observation(index), now + elapsed)
        assert decision.state == ClusterCollectionState.VERIFYING_LOSS
    decision = mission.step(observation(8), now + 0.9)
    assert decision.state == ClusterCollectionState.SCANNING
    assert mission.tracked_cluster is None


@pytest.mark.parametrize(
    ("center_x", "expected_sign"), ((0.25, -1), (0.75, 1))
)
def test_group_on_each_side_turns_toward_that_side(
    center_x: float, expected_sign: int
) -> None:
    mission = scanning_mission()
    now = acquire(mission, center_x=center_x)
    decision = mission.step(
        observation(4, detections=scallop_group(6, center_x=center_x)), now + 0.1
    )
    assert decision.motion.forward == 0.0
    assert decision.motion.yaw * expected_sign > 0.0


def test_approach_never_advances_outside_alignment_tolerance_and_has_two_speeds() -> None:
    mission = scanning_mission()
    now = acquire(mission, center_x=0.50)
    mission.state = ClusterCollectionState.APPROACHING

    mission.smoothed_center = (0.60, 0.40)
    mission.tracked_cluster = replace(
        mission.tracked_cluster, center_x_ratio=0.60, center_y_ratio=0.40
    )
    decision = mission._step_approaching(observation(4), now + 0.1)
    assert decision.motion.forward == 0.0

    mission.smoothed_center = (0.50, 0.40)
    mission.tracked_cluster = replace(
        mission.tracked_cluster, center_x_ratio=0.50, center_y_ratio=0.40
    )
    far = mission._step_approaching(observation(5), now + 0.2)
    assert far.motion.forward == pytest.approx(BASE.far_forward_command)

    mission.smoothed_center = (0.50, 0.60)
    mission.tracked_cluster = replace(
        mission.tracked_cluster, center_x_ratio=0.50, center_y_ratio=0.60
    )
    near = mission._step_approaching(observation(6), now + 0.3)
    assert near.motion.forward == pytest.approx(BASE.near_forward_command)


def test_no_gripper_mode_stops_at_approach_without_gripper_or_blind_descent() -> None:
    mission = ClusterCollectionMission(BASE, stop_after_approach=True)
    first = observation(0, depth=0.20)
    mission.start(first, descent_command=0.60, now=0.0)
    mission._enter(ClusterCollectionState.SCANNING, first, 0.0)
    now = acquire(mission, center_y=0.75)
    mission.state = ClusterCollectionState.APPROACHING
    mission.tracked_cluster = replace(
        mission.tracked_cluster,
        center_x_ratio=0.50,
        center_y_ratio=0.75,
    )

    for frame_id in range(4, 9):
        decision = mission._step_approaching(
            observation(frame_id, depth=0.35), now + 0.1
        )
        now += 0.1

    assert decision.state == ClusterCollectionState.COMPLETE
    assert decision.motion.is_neutral()
    assert decision.gripper_action is None
    assert mission.awaiting_gripper_action is None
    assert mission.total_grasp_attempt_count == 0
    assert mission.current_grasp_attempt_index == 0
    assert mission.completed_cluster_count == 0


def test_all_descent_trigger_modes() -> None:
    image = replace(BASE, descent_trigger=DescentTrigger.IMAGE_LINE)
    area = replace(
        BASE,
        descent_trigger=DescentTrigger.UNION_AREA,
        union_area_threshold=0.02,
    )
    both = replace(
        BASE,
        descent_trigger=DescentTrigger.BOTH,
        union_area_threshold=0.02,
    )
    cluster = scanning_mission().tracked_cluster
    # 用一个临时群体仅验证判据布尔组合。
    detections = scallop_group(6, center_y=0.75)
    source = cluster_scallops(detections, 1000, 1000)[0]
    line_ready_area_small = replace(
        source, center_x_ratio=0.5, center_y_ratio=0.75, union_area_ratio=0.01
    )
    line_not_ready_area_large = replace(
        source, center_x_ratio=0.5, center_y_ratio=0.60, union_area_ratio=0.03
    )
    assert ClusterCollectionMission(image)._descent_ready(
        line_ready_area_small, 0.0
    )
    assert ClusterCollectionMission(area)._descent_ready(
        line_not_ready_area_large, 0.0
    )
    assert not ClusterCollectionMission(both)._descent_ready(
        line_ready_area_small, 0.0
    )
    assert not ClusterCollectionMission(both)._descent_ready(
        line_not_ready_area_large, 0.0
    )
    assert ClusterCollectionMission(both)._descent_ready(
        replace(line_ready_area_small, union_area_ratio=0.03), 0.0
    )


def _finish_probe(
    mission: ClusterCollectionMission,
    *,
    frame_id: int,
    now: float,
    start_depth: float,
    bottom_depth: float,
):
    samples = (
        (now + 0.5, start_depth + 0.10),
        (now + 1.0, bottom_depth),
        (now + 1.5, bottom_depth),
        (now + 2.0, bottom_depth),
        (now + 3.0, bottom_depth),
        (now + 4.1, bottom_depth),
    )
    decision = None
    for offset, depth in samples:
        decision = mission.step(observation(frame_id, depth=depth), offset)
    assert decision is not None
    return decision, samples[-1][0]


def _finish_clearance(
    mission: ClusterCollectionMission,
    *,
    frame_id: int,
    now: float,
    depth: float,
):
    first = mission.step(observation(frame_id, depth=depth), now + 0.2)
    assert first.motion.is_neutral()
    return mission.step(observation(frame_id, depth=depth), now + 1.3), now + 1.3


def test_search_move_reprobes_bottom_instead_of_scanning_at_old_depth() -> None:
    mission = ClusterCollectionMission(BASE)
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision, now = _finish_probe(
        mission, frame_id=0, now=0.0, start_depth=0.20, bottom_depth=0.50
    )
    assert decision.state == ClusterCollectionState.CLEARING_BOTTOM
    decision, now = _finish_clearance(
        mission, frame_id=0, now=now, depth=0.35
    )
    assert decision.state == ClusterCollectionState.SCANNING
    mission._enter(ClusterCollectionState.SEARCH_ADVANCING, observation(0, depth=0.35), now)
    decision = mission.step(observation(0, depth=0.35), now + 2.1)
    assert decision.state == ClusterCollectionState.PROBING_BOTTOM


def test_one_group_runs_exactly_three_close_attempts_then_leaves_and_reprobes() -> None:
    mission = scanning_mission()
    now = acquire(mission, center_y=0.75)
    mission.state = ClusterCollectionState.APPROACHING
    mission.tracked_cluster = replace(
        mission.tracked_cluster,
        center_x_ratio=0.50,
        center_y_ratio=0.75,
    )
    for frame_id in range(4, 9):
        decision = mission._step_approaching(observation(frame_id, depth=0.35), now + 0.1)
        now += 0.1
    assert decision.gripper_action == "open"
    assert decision.grasp_attempt_index == 1

    for attempt in range(1, 4):
        decision = mission.acknowledge_gripper(
            "open", True, "accepted", observation(9, depth=0.35), now
        )
        assert decision.state == ClusterCollectionState.GRASP_DESCENDING
        decision, now = _finish_probe(
            mission,
            frame_id=9,
            now=now,
            start_depth=0.35,
            bottom_depth=0.55,
        )
        assert decision.gripper_action == "close"
        decision = mission.acknowledge_gripper(
            "close", True, "accepted", observation(9, depth=0.55), now
        )
        assert mission.total_grasp_attempt_count == attempt
        assert decision.grasp_attempt_index == attempt
        decision = mission.step(
            observation(9, depth=0.55), now + BASE.gripper_motion_wait_s + 0.1
        )
        now += BASE.gripper_motion_wait_s + 0.1
        assert decision.state == ClusterCollectionState.LIFTING_AFTER_GRASP
        decision, now = _finish_clearance(
            mission, frame_id=9, now=now, depth=0.40
        )
        assert decision.state == ClusterCollectionState.TRANSFER_TO_NET
        decision = mission.step(
            observation(9, depth=0.40), now + BASE.transfer_to_net_wait_s + 0.1
        )
        now += BASE.transfer_to_net_wait_s + 0.1
        assert decision.gripper_action == "reopen"
        decision = mission.acknowledge_gripper(
            "reopen", True, "accepted", observation(9, depth=0.40), now
        )
        if attempt < 3:
            assert decision.state == ClusterCollectionState.INTER_GRAB_ADVANCING
            decision = mission.step(
                observation(9, depth=0.40), now + BASE.inter_grab_duration_s + 0.1
            )
            now += BASE.inter_grab_duration_s + 0.1
            assert decision.gripper_action == "open"
            assert decision.grasp_attempt_index == attempt + 1
        else:
            assert decision.state == ClusterCollectionState.LEAVING_CLUSTER

    assert mission.total_grasp_attempt_count == 3
    assert mission.completed_cluster_count == 1
    decision = mission.step(
        observation(9, depth=0.40), now + BASE.leave_cluster_duration_s + 0.1
    )
    assert decision.state == ClusterCollectionState.PROBING_BOTTOM
    assert decision.grasp_attempt_index == 0


def test_gripper_rejection_does_not_count_and_recovers_to_start_depth() -> None:
    mission = scanning_mission()
    now = acquire(mission, center_y=0.75)
    mission.state = ClusterCollectionState.APPROACHING
    mission.tracked_cluster = replace(
        mission.tracked_cluster,
        center_x_ratio=0.50,
        center_y_ratio=0.75,
    )
    for frame_id in range(4, 9):
        decision = mission._step_approaching(
            observation(frame_id, depth=0.35), now + 0.1
        )
        now += 0.1
    assert decision.gripper_action == "open"

    decision = mission.acknowledge_gripper(
        "open", False, "servo output rejected", observation(9, depth=0.35), now
    )
    assert decision.state == ClusterCollectionState.RETURNING
    assert decision.motion.is_neutral()
    assert mission.total_grasp_attempt_count == 0
    assert mission.gripper_failure_reason == "机械爪 open 被拒绝: servo output rejected"


def test_clearance_aborts_if_depth_increases_below_recorded_bottom() -> None:
    mission = ClusterCollectionMission(BASE)
    first = observation(0, depth=0.20)
    mission.start(first, descent_command=0.60, now=0.0)
    mission.bottom_depth_m = 0.50
    mission.target_depth_m = 0.35
    mission._enter(ClusterCollectionState.CLEARING_BOTTOM, first, 1.0)
    decision = mission.step(observation(0, depth=0.56), 1.1)
    assert decision.state == ClusterCollectionState.ABORTED
    assert decision.motion.is_neutral()


def test_rejected_close_ack_recovers_without_counting_attempt() -> None:
    mission = scanning_mission()
    mission.awaiting_gripper_action = "close"
    mission.state = ClusterCollectionState.CLOSING_GRIPPER
    decision = mission.acknowledge_gripper(
        "close", False, "denied", observation(1), 1.0
    )
    assert decision.state == ClusterCollectionState.RETURNING
    assert decision.motion.is_neutral()
    assert mission.total_grasp_attempt_count == 0


def test_zero_finish_never_descends_to_reach_start_depth() -> None:
    mission = ClusterCollectionMission(BASE)
    mission.start(observation(0, depth=0.20), descent_command=0.60, now=0.0)
    decision = mission.request_normal_finish(
        observation(1, depth=0.10), 1.0, "operator"
    )
    assert decision.motion.is_neutral()
    decision = mission.step(observation(1, depth=0.10), 2.1)
    assert decision.state == ClusterCollectionState.COMPLETE
    assert decision.motion.vertical == 0.0
