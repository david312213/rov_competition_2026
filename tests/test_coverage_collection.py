"""所有数值均为合成测试输入，不能作为实艇标定数据。"""
import math
from dataclasses import replace
from pathlib import Path

import pytest

from rov_competition.coverage_collection import CoverageConfig, CoverageCollectionMission, load_coverage_config, semicircle_waypoints
from rov_competition.cluster_collection import ClusterCollectionConfig, ClusterCollectionError, ClusterCollectionState as S
from rov_competition.domain import MissionObservation, Detection, BoundingBox


@pytest.fixture
def cfg():
    return CoverageConfig(diameter_m=10, wall_clearance_m=.5, edge_margin_m=.5,
        lane_spacing_m=1, waypoint_tolerance_m=.1, search_command=.2, search_speed_mps=.5,
        far_speed_mps=.3, near_speed_mps=.15, heading_tolerance_deg=4, heading_gain=.01,
        turn_timeout_s=10, segment_timeout_s=100, max_tick_gap_s=.5, max_heading_step_deg=45,
        max_estimated_travel_m=100, max_local_travel_m=2, revisit_suppression_m=.6,
        local_attempt_limit=2, local_timeout_s=30, reobserve_timeout_s=2,
        observation_depth_m=1, depth_tolerance_m=.02, descent_command=.1, lift_command=.2,
        descent_timeout_s=10, depth_settle_s=.2, grasp_descent_m=.1, transfer_timeout_s=10,
        mission_timeout_s=120, maximum_operation_depth_m=5, image_yaw_sign=1,
        aim_x_ratio=.5, descent_line_y_ratio=.7, far_center_y_ratio=.55)


def obs(frame=0, depth=1, yaw=0, detections=()):
    return MissionObservation(frame, detections, 1000, 1000, True, True, depth, True, yaw)


def target(x=500, y=720):
    return Detection(1, 'scallop', .9, BoundingBox(x-20, y-20, x+20, y+20))


def mission(cfg):
    m = CoverageCollectionMission(ClusterCollectionConfig(), cfg)
    m.start(obs(), descent_command=.1, now=0)
    m.step(obs(1), .1)
    m.step(obs(2), .31)
    assert m.state == S.ROUTE_SEARCH
    return m


def test_geometry_inside_semicircle_and_shortens(cfg):
    points = semicircle_waypoints(cfg)
    lengths = []
    for a, b in zip(points[::2], points[1::2]):
        assert a[0] == b[0]
        lengths.append(abs(b[1]-a[1]))
        for x,y,_ in (a,b):
            assert math.hypot(x,y-5) == pytest.approx(4.5)
    assert lengths == sorted(lengths, reverse=True)
    assert points[2][1] < points[1][1]  # 换行需要同时改变y，不能只横移


@pytest.mark.parametrize('field,value', [('search_speed_mps', None), ('lane_spacing_m', 0),
    ('local_attempt_limit', 1.5), ('image_yaw_sign', True), ('field_calibrated', 'false'),
    ('observation_depth_m', 5), ('max_tick_gap_s', float('nan'))])
def test_invalid_config(cfg, field, value):
    with pytest.raises(ClusterCollectionError):
        replace(cfg, **{field:value})


def test_template_cannot_start():
    path = Path(__file__).resolve().parents[1] / 'ros2_ws/src/rov_competition/config/coverage.example.yaml'
    with pytest.raises(ClusterCollectionError):
        load_coverage_config(path)


def test_initial_descent_is_bounded_not_bottom_probe(cfg):
    m = CoverageCollectionMission(ClusterCollectionConfig(), cfg)
    m.start(obs(depth=.3), descent_command=.1, now=0)
    decision = m.step(obs(1, depth=.4), .1)
    assert m.state == S.INITIAL_DESCENT
    assert decision.motion.vertical == -.1
    assert m.target_depth_m == 1


def test_heading_wrap_and_route_motion(cfg):
    m = mission(cfg)
    m.initial_heading = 359
    decision = m.step(obs(3, yaw=1), .4)
    assert decision.motion.forward == .2
    assert decision.motion.yaw < 0
    m.step(obs(4, yaw=1), .5)
    assert m.estimated_travel == pytest.approx(.05)


def test_single_target_and_repeat_frames(cfg):
    m = mission(cfg)
    for i,t in [(3,.4),(3,.5),(3,.6)]:
        m.step(obs(i, detections=(target(),)), t)
    assert m.state == S.ROUTE_SEARCH
    assert m.last_speed == 0
    m.step(obs(4,detections=(target(),)), .7)
    m.step(obs(5,detections=(target(),)), .8)
    assert m.state == S.ALIGNING
    assert m.tracked_cluster.count == 1


def test_disjoint_targets_never_become_empty_group_center(cfg):
    m = mission(cfg)
    detections = (target(300),target(700))
    for i in range(3,6):
        m.step(obs(i,detections=detections), i*.1+.1)
    assert m.tracked_cluster.center_x_ratio in (.3,.7)


def test_pause_does_not_add_distance(cfg):
    m = mission(cfg)
    m.step(obs(3), .4)
    m.delay_timers(5)
    m.step(obs(4), 5.5)
    assert m.estimated_travel == 0


def test_cycle_gap_aborts(cfg):
    m = mission(cfg)
    assert m.step(obs(3), 2).state == S.ABORTED


def test_retry_limit_returns_to_route_without_reacquiring(cfg):
    m = mission(cfg)
    m.local_start = .2
    m.local_attempts = 2
    m._enter(S.LOCAL_REOBSERVE, obs(), .31)
    d = m.step(obs(3,detections=(target(),)), .4)
    assert d.state == S.LIFTING_AFTER_GRASP
    assert m.local_start is None
    assert m.suppressed_until_travel > m.estimated_travel


def test_local_time_limit(cfg):
    m = mission(replace(cfg, local_timeout_s=.2))
    m.local_start = .1
    m._enter(S.LOCAL_REOBSERVE, obs(), .31)
    assert m.step(obs(3), .4).state == S.LIFTING_AFTER_GRASP


def test_transfer_requires_explicit_confirmation(cfg):
    m = mission(cfg)
    m._enter(S.TRANSFER_TO_NET, obs(), .31)
    assert m.step(obs(3), .4).gripper_action is None
    d = m.confirm_transfer(obs(3), .4)
    assert d.gripper_action == 'reopen'
    m.acknowledge_gripper('reopen', True, 'ok', obs(4), .5)
    assert m.state == S.LOCAL_REOBSERVE
    assert m.confirm_transfer(obs(4), .5) is None


def test_return_timeout_not_reset_by_mission_deadline(cfg):
    m = mission(replace(cfg, mission_timeout_s=.5))
    m.step(obs(3, depth=1.2), .5)
    assert m.state == S.RETURNING
    entered = m.state_started_at
    m.step(obs(4, depth=1.2), .6)
    assert m.state_started_at == entered
    assert m.last_speed == 0


def test_gripper_failure_recovers(cfg):
    m = mission(cfg)
    m.awaiting_gripper_action = 'open'
    d = m.acknowledge_gripper('open', False, 'failed', obs(), .4)
    assert d.state == S.RETURNING
    assert m.total_grasp_attempt_count == 0


def test_grasp_does_not_advance_route_index_but_tracks_displacement(cfg):
    m = mission(cfg)
    m._enter(S.APPROACHING, obs(), .31)
    m.tracked_cluster = m._single(obs(detections=(target(y=400),)))
    m.smoothed_center = (.5,.4)
    m.last_target_seen_at = .31
    m.local_start = .31
    for i,t in [(3,.4),(4,.5)]:
        m.step(obs(i,detections=(target(y=400),)), t)
    assert m.point_index == 1
    assert m.estimated_travel == pytest.approx(.03)


def test_full_two_attempts_reobserve_then_leave(cfg):
    m = mission(cfg)
    now = .31
    frame = 2
    transfers = 0
    for _ in range(250):
        now += .1
        frame += 1
        depth = m.grasp_target if m.state in (S.GRASP_DESCENDING,S.CLOSING_GRIPPER,S.GRASP_HOLDING) else 1
        d = m.step(obs(frame, depth=depth, detections=(target(),)), now)
        if m.state == S.TRANSFER_TO_NET:
            transfers += 1
            d = m.confirm_transfer(obs(frame), now)
        if d.gripper_action:
            m.acknowledge_gripper(d.gripper_action, True, 'synthetic ack', obs(frame, depth=depth), now)
        if m.completed_cluster_count:
            break
    assert m.total_grasp_attempt_count == 2
    assert transfers == 2
    assert m.completed_cluster_count == 1
    assert m.point_index == 1


def test_entire_route_reaches_completion_with_synthetic_ideal_motion(cfg):
    m = mission(cfg)
    now, yaw = .31, 0.0
    visited = set()
    for frame in range(3, 2000):
        now += .1
        if m.point_index < len(m.points):
            x,y,_ = m.points[m.point_index]
            desired = math.degrees(math.atan2(-(x-m.x), y-m.y)) % 360
            delta = (desired-yaw+180) % 360-180
            yaw = (yaw + max(-20,min(20,delta))) % 360
        decision = m.step(obs(frame,yaw=yaw), now)
        visited.add(m.point_index)
        assert m.state != S.ABORTED
        if m.state == S.COMPLETE:
            break
    assert m.state == S.COMPLETE
    assert m.point_index == len(m.points)
    assert visited == set(range(1,len(m.points)+1))


def test_stale_frames_neutral_then_abort(cfg):
    m = mission(cfg)
    for t in (.6,.9,1.2,1.5):
        decision = m.step(obs(2),t)
    assert decision.motion.forward == 0
    for i in range(16,57):
        decision = m.step(obs(2),i/10)
    assert decision.state == S.ABORTED


def test_no_target_reobserve_leaves_and_transfer_timeout_finishes(cfg):
    m = mission(replace(cfg,reobserve_timeout_s=.1, transfer_timeout_s=.1))
    m.local_start = .31
    m._enter(S.LOCAL_REOBSERVE,obs(),.31)
    assert m.step(obs(3),.5).state == S.LIFTING_AFTER_GRASP
    m._enter(S.TRANSFER_TO_NET,obs(),.5)
    decision = m.step(obs(4),.7)
    assert decision.state == S.RETURNING
    assert decision.gripper_action is None
