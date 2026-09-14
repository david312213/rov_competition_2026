"""Offline regressions for optional depth cutoff and gross-depth outliers."""

from dataclasses import replace
from pathlib import Path

import pytest

from rov_competition.cluster_collection import (
    ClusterCollectionConfig,
    ClusterCollectionError,
    ClusterCollectionMission,
    ClusterCollectionState as State,
    load_cluster_collection_config,
)
from rov_competition.domain import MissionObservation


def sample(depth, frame=0, valid=True):
    return MissionObservation(
        frame_id=frame, detections=(), frame_width=640, frame_height=480,
        perception_valid=True, depth_valid=valid, depth_m=depth,
        attitude_valid=True, yaw_deg=0.0,
    )


def begin(depth=0.05, **changes):
    config = replace(ClusterCollectionConfig(maximum_operation_depth_m=None), **changes)
    mission = ClusterCollectionMission(config, stop_after_approach=True)
    mission.start(sample(depth), descent_command=0.60, now=0.0)
    return mission


def test_active_yaml_disables_only_absolute_depth_cutoff():
    root = Path(__file__).resolve().parents[1]
    config = load_cluster_collection_config(
        root / "ros2_ws/src/rov_competition/config/cluster_collection.yaml"
    )
    assert config.maximum_operation_depth_m is None
    assert config.bottom_probe_timeout_s == 45.0
    assert config.bottom_detection_stable_s == 3.0
    assert config.depth_anomaly_timeout_s == 2.0


@pytest.mark.parametrize("limit", [0.0, -1.0, float("inf"), float("nan"), True, False])
def test_bad_depth_limit_rejected(limit):
    with pytest.raises(ClusterCollectionError):
        ClusterCollectionConfig(maximum_operation_depth_m=limit)


@pytest.mark.parametrize("field", [
    "maximum_depth_step_m", "depth_recovery_stable_s", "depth_anomaly_timeout_s",
    "bottom_probe_timeout_s",
])
@pytest.mark.parametrize("value", [0.0, -1.0, float("inf"), float("nan")])
def test_depth_safety_cannot_be_disabled_with_bad_numbers(field, value):
    with pytest.raises(ClusterCollectionError):
        replace(ClusterCollectionConfig(), **{field: value})


def test_recovery_must_fit_within_abort_timeout():
    with pytest.raises(ClusterCollectionError):
        ClusterCollectionConfig(depth_recovery_stable_s=2.0)


def test_enabled_absolute_cutoff_still_stops():
    mission = begin(19.7, maximum_operation_depth_m=20.0)
    decision = mission.step(sample(20.1, 1), 1.0)
    assert decision.state == State.ABORTED
    assert "硬上限" in decision.message
    assert decision.motion.is_neutral()


def test_disabled_cutoff_does_not_stop_at_fixed_metre_value():
    mission = begin(19.7)
    decision = mission.step(sample(20.1, 1), 1.0)
    assert decision.state == State.PROBING_BOTTOM
    assert decision.motion.vertical == -0.60


def test_2552_spike_is_not_motion_or_bottom_and_does_not_rebase():
    mission = begin(0.78)
    for now in [0.1, 0.5, 1.0]:
        decision = mission.step(sample(25.52, 1), now)
        assert decision.state == State.PROBING_BOTTOM
        assert decision.motion.is_neutral()
        assert decision.gripper_action is None
        assert mission.bottom_depth_m is None
        assert mission.last_plausible_depth_m == 0.78
        assert not mission.depth_history
    decision = mission.step(sample(25.52, 2), 2.2)
    assert decision.state == State.ABORTED
    assert decision.motion.is_neutral()


def test_automatic_recovery_requires_new_confirmation_window():
    mission = begin(0.05)
    mission.step(sample(0.4, 1), 0.5)
    mission.step(sample(0.78, 2), 1.0)
    assert mission.step(sample(25.52, 3), 1.1).motion.is_neutral()
    for now in [1.2, 1.4, 1.6]:
        decision = mission.step(sample(0.78, 4), now)
        assert decision.motion.is_neutral()
        assert mission.bottom_depth_m is None
    decision = mission.step(sample(0.78, 5), 1.8)
    assert decision.motion.vertical == -0.60
    assert mission.bottom_stable_duration_s == 0.0
    decision = mission.step(sample(0.78, 6), 3.9)
    assert decision.state == State.PROBING_BOTTOM
    decision = mission.step(sample(0.78, 7), 4.9)
    assert decision.state == State.CLEARING_BOTTOM
    assert mission.bottom_depth_m == pytest.approx(0.78)


def test_repeated_glitches_do_not_reset_abort_deadline():
    mission = begin(0.78)
    mission.step(sample(25.52), 0.1)
    mission.step(sample(0.78), 0.2)
    mission.step(sample(25.52), 0.4)
    mission.step(sample(0.78), 0.5)
    mission.step(sample(25.52), 0.8)
    decision = mission.step(sample(25.52), 2.2)
    assert decision.state == State.ABORTED


def test_probe_timeout_still_applies_during_anomaly_hold():
    mission = begin(0.78)
    assert mission.step(sample(25.52), 44.9).state == State.PROBING_BOTTOM
    decision = mission.step(sample(25.52), 45.0)
    assert decision.state == State.ABORTED
    assert "触底等待超时" in decision.message
    assert decision.motion.is_neutral()


def test_frozen_start_depth_is_not_bottom_and_times_out():
    mission = begin(0.05)
    for now in range(1, 45):
        decision = mission.step(sample(0.05), float(now))
        assert decision.state == State.PROBING_BOTTOM
        assert mission.bottom_depth_m is None
    assert mission.step(sample(0.05), 45.0).state == State.ABORTED


def test_automatic_bottom_at_15m_without_operator_confirmation():
    mission = begin(0.05)
    for frame, depth in enumerate([0.4, 0.8, 1.2, 1.5], 1):
        decision = mission.step(sample(depth, frame), frame * 0.5)
        assert decision.state == State.PROBING_BOTTOM
    for now in [2.5, 3.0, 4.0, 4.9]:
        decision = mission.step(sample(1.5, 5), now)
        assert decision.state == State.PROBING_BOTTOM
    decision = mission.step(sample(1.5, 6), 5.1)
    assert decision.state == State.CLEARING_BOTTOM
    assert mission.bottom_depth_m == pytest.approx(1.5)
    assert decision.target_depth_m == pytest.approx(1.35)
    assert decision.gripper_action is None


@pytest.mark.parametrize("depth,valid", [(float("nan"), True), (0.8, False)])
def test_invalid_depth_remains_a_hard_abort(depth, valid):
    mission = begin()
    decision = mission.step(sample(depth, valid=valid), 0.1)
    assert decision.state == State.ABORTED
    assert decision.motion.is_neutral()
