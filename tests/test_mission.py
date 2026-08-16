"""候选版自主抓取状态机的纯软件测试。"""

from dataclasses import replace
from pathlib import Path

import pytest

from rov_competition.config import load_autonomy_config
from rov_competition.domain import (
    BoundingBox,
    Detection,
    GripperAction,
    MissionObservation,
    MissionOutcome,
    MissionState,
)
from rov_competition.mission import AutonomousGraspMission, signed_yaw_delta_deg

PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "rov_competition"
BASE = load_autonomy_config(PACKAGE / "config" / "autonomy.yaml").mission
TEST_CONFIG = replace(
    BASE,
    allow_autonomous_mission=True,
    allow_open_loop_horizontal_motion=True,
    horizontal_motion_confirmed=True,
    image_control_confirmed=True,
    grasp_thresholds_confirmed=True,
    image_yaw_sign=1,
    image_vertical_sign=1,
    detection_confirmation_frames=2,
    alignment_confirmation_frames=2,
    grasp_area_confirmation_frames=3,
    descent_settle_s=0.10,
    ascent_settle_s=0.10,
    gripper_hold_s=0.10,
)


def detection(
    *,
    label: str = "echinus",
    confidence: float = 0.9,
    left: float = 280,
    top: float = 296,
    right: float = 360,
    bottom: float = 376,
) -> Detection:
    """创建 640x480 画面中默认对准 (0.50, 0.70) 的框。"""

    return Detection(
        class_id=0,
        label=label,
        confidence=confidence,
        box=BoundingBox(left=left, top=top, right=right, bottom=bottom),
    )


def observation(
    frame_id: int,
    *,
    detections=(),
    depth: float = 1.30,
    yaw: float = 350.0,
    perception_valid: bool = True,
) -> MissionObservation:
    """创建一个同时含图像、深度和航向的观测。"""

    return MissionObservation(
        frame_id=frame_id,
        detections=tuple(detections),
        frame_width=640,
        frame_height=480,
        perception_valid=perception_valid,
        depth_valid=True,
        depth_m=depth,
        attitude_valid=True,
        yaw_deg=yaw,
    )


def new_mission(*, labels=("echinus",), config=TEST_CONFIG) -> AutonomousGraspMission:
    """创建一个方向已在测试中明确的任务。"""

    return AutonomousGraspMission(config, labels)


def reach_scanning(task: AutonomousGraspMission) -> tuple[float, int]:
    """打开机械爪并完成从 1.00m 到 1.30m 的相对下潜。"""

    first = observation(0, depth=1.00)
    decision = task.start(first, 0.0)
    assert decision.state == MissionState.PREPARING
    assert decision.gripper == GripperAction.OPEN
    decision = task.acknowledge_gripper(
        GripperAction.OPEN, True, "accepted", first, 0.01
    )
    assert decision.state == MissionState.DESCENDING
    assert decision.target_depth_m == pytest.approx(1.30)
    task.step(observation(1, depth=1.30), 0.02)
    decision = task.step(observation(2, depth=1.30), 0.13)
    assert decision.state == MissionState.SCANNING
    return 0.13, 2


def reach_approaching(task: AutonomousGraspMission) -> tuple[float, int]:
    """在扫描中连续确认目标，再连续对准。"""

    now, frame = reach_scanning(task)
    target = detection(label=sorted(task.target_labels)[0])
    for _ in range(task.config.detection_confirmation_frames):
        now += 0.05
        frame += 1
        decision = task.step(observation(frame, detections=(target,)), now)
    assert decision.state == MissionState.ALIGNING
    for _ in range(task.config.alignment_confirmation_frames):
        now += 0.05
        frame += 1
        decision = task.step(observation(frame, detections=(target,)), now)
    assert decision.state == MissionState.APPROACHING
    return now, frame


def test_yaw_delta_handles_359_to_zero_wrap() -> None:
    """360° 扫描不能在北向跨零时倒退 358°。"""

    assert signed_yaw_delta_deg(359.0, 1.0) == pytest.approx(2.0)
    assert signed_yaw_delta_deg(1.0, 359.0) == pytest.approx(-2.0)


def test_relative_descent_is_start_depth_plus_point_three_metres() -> None:
    """状态1使用启动时深度 +0.30m，不把 0.30m 当绝对目标。"""

    task = new_mission()
    first = observation(0, depth=2.10)
    task.start(first, 0.0)
    decision = task.acknowledge_gripper(
        GripperAction.OPEN, True, "accepted", first, 0.01
    )
    assert decision.target_depth_m == pytest.approx(2.40)
    decision = task.step(observation(1, depth=2.10), 0.02)
    assert decision.motion.vertical < 0.0
    assert abs(decision.motion.vertical) <= TEST_CONFIG.descent_max_command


def test_scan_accumulates_a_real_360_degrees_across_zero() -> None:
    """状态2必须依据真实航向累计一整圈。"""

    task = new_mission()
    now, frame = reach_scanning(task)
    for index in range(1, 19):
        now += 0.05
        frame += 1
        yaw = (350.0 + index * 20.0) % 360.0
        decision = task.step(observation(frame, yaw=yaw), now)
    assert decision.state == MissionState.ADVANCING
    assert decision.motion.is_neutral()


def test_advance_uses_derived_duration_and_returns_to_scan() -> None:
    """0.40/0.30 自动得到约 1.33s，结束后回到新一轮扫描。"""

    task = new_mission()
    now, frame = reach_scanning(task)
    task._enter(MissionState.ADVANCING, now, observation(frame))
    decision = task.step(
        observation(frame + 1), now + task.config.advance_duration_s + 0.01
    )
    assert task.config.advance_duration_s == pytest.approx(4.0 / 3.0)
    assert decision.state == MissionState.SCANNING
    assert decision.search_cycle == 2


def test_acquisition_selects_largest_box_before_confidence() -> None:
    """多个目标出现时先选框面积最大者，面积相同才比置信度。"""

    task = new_mission(labels=("echinus", "scallop"))
    now, frame = reach_scanning(task)
    small_high = detection(confidence=0.99)
    large_low = detection(
        label="scallop", confidence=0.55, left=220, top=250, right=420, bottom=422
    )
    decision = task.step(
        observation(frame + 1, detections=(small_high, large_low)), now + 0.05
    )
    assert decision.selected_target == large_low
    assert decision.motion.is_neutral()


def test_reusing_one_frame_cannot_satisfy_grasp_confirmation() -> None:
    """20Hz 控制循环重复使用同一张图时不得增加连续帧计数。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    near = detection(left=160, top=240, right=480, bottom=432)
    frame += 1
    for index in range(4):
        now += 0.05
        decision = task.step(observation(frame, detections=(near,)), now)
    assert decision.state == MissionState.APPROACHING
    assert decision.gripper == GripperAction.NONE


def test_close_requires_alignment_class_k_and_three_new_frames() -> None:
    """对准、类别 k 和三张新图必须同时满足才请求闭爪。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    near = detection(left=160, top=240, right=480, bottom=432)  # 面积比 0.20
    for _ in range(task.config.grasp_area_confirmation_frames):
        now += 0.05
        frame += 1
        decision = task.step(observation(frame, detections=(near,)), now)
    assert decision.state == MissionState.GRASPING
    assert decision.motion.is_neutral()
    assert decision.gripper == GripperAction.CLOSE
    assert decision.target_area_ratio == pytest.approx(0.20)
    assert decision.grasp_area_threshold == pytest.approx(0.18)


def test_each_class_uses_its_own_grasp_threshold() -> None:
    """类别阈值不得被一个全局 k 覆盖。"""

    thresholds = dict(TEST_CONFIG.grasp_area_ratios)
    thresholds["starfish"] = 0.25
    task = new_mission(
        labels=("starfish",),
        config=replace(TEST_CONFIG, grasp_area_ratios=thresholds),
    )
    now, frame = reach_approaching(task)
    area_twenty_percent = detection(
        label="starfish", left=160, top=240, right=480, bottom=432
    )
    for _ in range(4):
        now += 0.05
        frame += 1
        decision = task.step(
            observation(frame, detections=(area_twenty_percent,)), now
        )
    assert decision.state == MissionState.APPROACHING
    assert decision.gripper == GripperAction.NONE
    assert decision.grasp_area_threshold == pytest.approx(0.25)


def test_locked_target_is_not_replaced_by_a_new_larger_box() -> None:
    """锁定后的跟踪依据 IoU/中心距离，不再按全局最大框瞬间切换。"""

    task = new_mission(labels=("echinus", "scallop"))
    now, frame = reach_approaching(task)
    original = detection(left=276, top=292, right=364, bottom=380)
    distracting = detection(
        label="scallop", left=0, top=0, right=300, bottom=300, confidence=0.99
    )
    decision = task.step(
        observation(frame + 1, detections=(distracting, original)), now + 0.05
    )
    assert decision.selected_target == original
    assert decision.state == MissionState.APPROACHING


def test_same_class_box_that_jumps_across_frame_cannot_trigger_close() -> None:
    """同一类别的另一个物体也不能用大跳变顶替原锁定。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    far_jump = detection(left=0, top=0, right=100, bottom=100)
    decision = task.step(
        observation(frame + 1, detections=(far_jump,)), now + 0.05
    )
    assert decision.motion.is_neutral()
    assert decision.gripper == GripperAction.NONE


def test_nearly_full_screen_box_aborts_instead_of_blind_grasp() -> None:
    """框异常接近整屏时视觉定位不再可信，必须中止。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    huge = detection(left=0, top=0, right=640, bottom=480)
    decision = task.step(observation(frame + 1, detections=(huge,)), now + 0.05)
    assert decision.state == MissionState.ABORTED
    assert decision.gripper == GripperAction.NONE
    assert decision.motion.is_neutral()


def test_lost_target_runs_left_60_then_right_120_and_returns_to_scan() -> None:
    """状态4/5丢框后先停，再左 60°、右 120°，失败则回到 360° 扫描。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    now += task.config.target_lost_timeout_s + 0.01
    frame += 1
    decision = task.step(observation(frame), now)
    assert decision.state == MissionState.REACQUIRING
    assert decision.motion.is_neutral()

    now += task.config.reacquire_grace_s + 0.01
    frame += 1
    decision = task.step(observation(frame, yaw=350.0), now)
    assert decision.motion.yaw < 0.0
    for yaw in (320.0, 290.0):
        now += 0.1
        frame += 1
        decision = task.step(observation(frame, yaw=yaw), now)
    assert decision.motion.is_neutral()  # 左转结束的切段停车。
    for yaw in (330.0, 10.0, 50.0):
        now += 0.1
        frame += 1
        decision = task.step(observation(frame, yaw=yaw), now)
    assert decision.state == MissionState.SCANNING
    assert decision.motion.is_neutral()


def test_gripper_acceptance_then_ascent_and_normal_completion() -> None:
    """闭爪服务接受后才开始保持，随后上升到 0.30m 回收深度。"""

    task = new_mission()
    now, frame = reach_approaching(task)
    near = detection(left=160, top=240, right=480, bottom=432)
    for _ in range(task.config.grasp_area_confirmation_frames):
        now += 0.05
        frame += 1
        decision = task.step(observation(frame, detections=(near,)), now)
    decision = task.acknowledge_gripper(
        GripperAction.CLOSE,
        True,
        "accepted only",
        observation(frame, detections=(near,)),
        now + 0.01,
    )
    assert decision.gripper_command_accepted is True
    assert task.grasp_attempts == 1
    decision = task.step(
        observation(frame + 1, detections=(near,), depth=1.30), now + 0.12
    )
    assert decision.state == MissionState.ASCENDING
    assert decision.outcome == MissionOutcome.GRASP_COMMANDED
    assert decision.motion.is_neutral()
    decision = task.step(observation(frame + 2, depth=1.00), now + 0.17)
    assert decision.motion.vertical > 0.0
    task.step(observation(frame + 3, depth=0.30), now + 0.22)
    decision = task.step(observation(frame + 4, depth=0.30), now + 0.33)
    assert decision.state == MissionState.COMPLETE
    assert decision.motion.is_neutral()


def test_rejected_gripper_service_aborts_neutrally() -> None:
    """开爪或闭爪被网关拒绝时不得继续任务。"""

    task = new_mission()
    first = observation(0, depth=1.0)
    task.start(first, 0.0)
    decision = task.acknowledge_gripper(
        GripperAction.OPEN, False, "gate closed", first, 0.01
    )
    assert decision.state == MissionState.ABORTED
    assert decision.outcome == MissionOutcome.ABORTED
    assert decision.motion.is_neutral()


def test_one_search_cycle_without_target_returns_to_recovery_depth() -> None:
    """达到最大搜索轮数时不漫无边际遍历，而是上升回收。"""

    task = new_mission(config=replace(TEST_CONFIG, maximum_search_cycles=1))
    now, frame = reach_scanning(task)
    for index in range(1, 19):
        now += 0.05
        frame += 1
        task.step(observation(frame, yaw=(350 + 20 * index) % 360), now)
    decision = task.step(
        observation(frame + 1), now + task.config.advance_duration_s + 0.01
    )
    assert decision.state == MissionState.ASCENDING
    assert decision.outcome == MissionOutcome.NO_TARGET
    assert decision.motion.is_neutral()


def test_invalid_perception_aborts_every_active_state() -> None:
    """相机故障不得沿用旧框继续运动。"""

    task = new_mission()
    now, frame = reach_scanning(task)
    decision = task.step(
        observation(frame + 1, perception_valid=False), now + 0.05
    )
    assert decision.state == MissionState.ABORTED
    assert decision.motion.is_neutral()


@pytest.mark.parametrize(
    "state",
    [
        MissionState.PREPARING,
        MissionState.DESCENDING,
        MissionState.SCANNING,
        MissionState.ADVANCING,
        MissionState.ALIGNING,
        MissionState.APPROACHING,
        MissionState.REACQUIRING,
        MissionState.GRASPING,
        MissionState.ASCENDING,
    ],
)
def test_external_abort_is_neutral_from_every_active_state(state: MissionState) -> None:
    """无论任务在哪个活动状态，中止决策都不得遗留非中位指令。"""

    task = new_mission()
    obs = observation(0, depth=1.0)
    task.start(obs, 0.0)
    task._enter(state, 0.01, obs)
    decision = task.abort("测试中止", obs, 0.02)
    assert decision.state == MissionState.ABORTED
    assert decision.outcome == MissionOutcome.ABORTED
    assert decision.motion.is_neutral()
    assert decision.gripper == GripperAction.NONE
