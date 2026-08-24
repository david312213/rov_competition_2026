"""单轴与转向实机工具的纯软件安全测试。"""

import csv

import pytest
import yaml

from rov_competition.commissioning import (
    CommissioningError,
    TurnProgressTracker,
    build_axis_test_plan,
    build_turn_test_plan,
)
from rov_competition.grasp_calibration import (
    CalibrationSample,
    CalibrationTelemetry,
    GraspCalibrationError,
    GraspCalibrationSession,
    build_calibration_summary,
    calculate_box_metrics,
    ordered_calibration_targets,
)


def test_axis_preview_never_produces_motion() -> None:
    """不带 --execute 时，即使计划中有 0.05 也必须标记为不执行。"""

    plan = build_axis_test_plan(
        "forward", 0.05, 0.3, command_limit=0.10, execute=False
    )
    assert plan.produces_motion is False
    assert plan.motion.forward == pytest.approx(0.05)


@pytest.mark.parametrize(
    ("action", "axis", "sign"),
    [
        ("forward", "forward", 1),
        ("backward", "forward", -1),
        ("left", "lateral", -1),
        ("right", "lateral", 1),
        ("up", "vertical", 1),
        ("down", "vertical", -1),
    ],
)
def test_six_axis_actions_map_to_exactly_one_axis(action: str, axis: str, sign: int) -> None:
    """每次测试只能产生一个非中位轴。"""

    plan = build_axis_test_plan(action, 0.05, 0.3, command_limit=0.10, execute=True)
    values = {
        "forward": plan.motion.forward,
        "lateral": plan.motion.lateral,
        "vertical": plan.motion.vertical,
        "yaw": plan.motion.yaw,
    }
    assert values[axis] == pytest.approx(sign * 0.05)
    assert sum(value != 0.0 for value in values.values()) == 1


def test_axis_value_and_duration_cannot_bypass_limits() -> None:
    """幅值和时长超界不会被静默截断成“看似可用”。"""

    with pytest.raises(CommissioningError):
        build_axis_test_plan("forward", 0.11, 0.3, command_limit=0.10, execute=True)
    with pytest.raises(CommissioningError):
        build_axis_test_plan("forward", 0.05, 5.1, command_limit=0.10, execute=True)


def test_turn_preview_and_angle_range() -> None:
    """转向默认只预览，且支持的边界为 1°..360°。"""

    preview = build_turn_test_plan(
        "right", 360.0, 0.05, command_limit=0.10, execute=False
    )
    assert preview.produces_motion is False
    assert preview.motion.yaw == pytest.approx(0.05)
    with pytest.raises(CommissioningError):
        build_turn_test_plan("right", 361.0, 0.05, command_limit=0.10, execute=True)


def test_turn_tracker_accumulates_right_turn_across_zero() -> None:
    """350° -> 10° -> 30° 应累计右转 40°。"""

    plan = build_turn_test_plan(
        "right", 40.0, 0.05, command_limit=0.10, execute=True
    )
    tracker = TurnProgressTracker(plan)
    tracker.update(350.0, 0.0)
    tracker.update(10.0, 0.1)
    assert tracker.update(30.0, 0.2) == pytest.approx(40.0)
    assert tracker.complete(tolerance_deg=1.0)


def test_turn_tracker_rejects_opposite_direction_jump_and_no_progress() -> None:
    """方向相反、航向跳变或长时间没有进展都必须中止。"""

    plan = build_turn_test_plan(
        "right", 90.0, 0.05, command_limit=0.10, execute=True
    )
    opposite = TurnProgressTracker(plan)
    opposite.update(100.0, 0.0)
    with pytest.raises(CommissioningError, match="相反"):
        opposite.update(95.0, 0.1)

    jump = TurnProgressTracker(plan, maximum_yaw_step_deg=45.0)
    jump.update(0.0, 0.0)
    with pytest.raises(CommissioningError, match="跳变"):
        jump.update(90.0, 0.1)

    stalled = TurnProgressTracker(plan, progress_timeout_s=1.0)
    stalled.update(0.0, 0.0)
    with pytest.raises(CommissioningError, match="无进展"):
        stalled.update(0.0, 1.1)


def _calibration_sample(
    sample_id: int,
    *,
    center_x: float,
    center_y: float,
    area_ratio: float,
    outcome: str = "success",
) -> CalibrationSample:
    """构造一张 1000x1000 画面中的方形抓取框样本。"""

    side = area_ratio**0.5
    target = calculate_box_metrics(
        class_id=0,
        label="echinus",
        confidence=0.9,
        frame_width=1000,
        frame_height=1000,
        left=(center_x - side / 2.0) * 1000,
        top=(center_y - side / 2.0) * 1000,
        right=(center_x + side / 2.0) * 1000,
        bottom=(center_y + side / 2.0) * 1000,
    )
    return CalibrationSample(
        sample_id=sample_id,
        utc_time="2026-08-25T00:00:00+00:00",
        detection_stamp_s=float(sample_id),
        image_stamp_s=float(sample_id) + 0.01,
        image_time_delta_s=0.01,
        target=target,
        telemetry=CalibrationTelemetry(depth_m=0.8, yaw_deg=30.0),
        screenshot_file=f"screenshots/{sample_id:04d}.jpg",
        outcome=outcome,
    )


def test_grasp_calibration_box_math_clips_and_filters_targets() -> None:
    """标定只能使用画面内的可抓类别，且默认选最大框。"""

    large = calculate_box_metrics(
        class_id=0,
        label="echinus",
        confidence=0.8,
        frame_width=200,
        frame_height=100,
        left=-10,
        top=10,
        right=110,
        bottom=90,
    )
    ignored = calculate_box_metrics(
        class_id=4,
        label="waterweeds",
        confidence=0.99,
        frame_width=200,
        frame_height=100,
        left=0,
        top=0,
        right=200,
        bottom=100,
    )
    small = calculate_box_metrics(
        class_id=1,
        label="scallop",
        confidence=0.9,
        frame_width=200,
        frame_height=100,
        left=50,
        top=25,
        right=100,
        bottom=75,
    )

    assert large.left == 0.0
    assert large.center_x_ratio == pytest.approx(0.275)
    assert large.area_ratio == pytest.approx(0.44)
    ordered = ordered_calibration_targets(
        (small, ignored, large), ("echinus", "scallop")
    )
    assert [target.label for target in ordered] == ["echinus", "scallop"]

    with pytest.raises(GraspCalibrationError, match="有效面积"):
        calculate_box_metrics(
            class_id=0,
            label="echinus",
            confidence=0.9,
            frame_width=100,
            frame_height=100,
            left=80,
            top=10,
            right=20,
            bottom=90,
        )


def test_grasp_calibration_summary_uses_successes_and_requires_enough_evidence() -> None:
    """建议只由足够的成功抓取产生，失败样本不会拉偏中位数。"""

    samples = [
        _calibration_sample(
            index,
            center_x=center_x,
            center_y=center_y,
            area_ratio=area,
        )
        for index, (center_x, center_y, area) in enumerate(
            zip(
                (0.48, 0.49, 0.50, 0.51, 0.52),
                (0.68, 0.69, 0.70, 0.71, 0.72),
                (0.08, 0.09, 0.10, 0.11, 0.12),
            ),
            start=1,
        )
    ]
    samples.append(
        _calibration_sample(
            6,
            center_x=0.2,
            center_y=0.2,
            area_ratio=0.30,
            outcome="failure",
        )
    )

    summary = build_calibration_summary(
        samples, ("echinus",), minimum_success_samples=5
    )
    suggestion = summary["autonomy_yaml_suggestion"]
    assert isinstance(suggestion, dict)
    assert suggestion["field_tuning"]["grasp_aim_x_ratio"] == pytest.approx(0.5)
    assert suggestion["field_tuning"]["grasp_aim_y_ratio"] == pytest.approx(0.7)
    assert suggestion["field_tuning"]["grasp_area_ratio"]["echinus"] == pytest.approx(
        0.09
    )
    assert summary["outcome_counts"]["failure"] == 1

    insufficient = build_calibration_summary(
        samples[:2], ("echinus",), minimum_success_samples=5
    )
    assert insufficient["autonomy_yaml_suggestion"] is None
    assert insufficient["warnings"]


def test_grasp_calibration_session_writes_evidence_without_editing_config(
    tmp_path,
) -> None:
    """CSV/JPEG/建议要原子落盘，但绝不能自动改 autonomy.yaml。"""

    autonomy_config = tmp_path / "autonomy.yaml"
    original_config = "field_tuning:\n  grasp_aim_x_ratio: 0.50\n"
    autonomy_config.write_text(original_config, encoding="utf-8")
    session = GraspCalibrationSession(
        tmp_path / "session",
        target_labels=("echinus",),
        autonomy_config_path=autonomy_config,
        minimum_success_samples=3,
    )
    target = _calibration_sample(
        1,
        center_x=0.5,
        center_y=0.7,
        area_ratio=0.1,
    ).target
    first = session.capture(
        target=target,
        telemetry=CalibrationTelemetry(depth_m=0.75, flight_mode="ALT_HOLD"),
        detection_stamp_s=10.0,
        image_stamp_s=10.01,
        jpeg_data=b"\xff\xd8test-jpeg\xff\xd9",
    )
    assert first.outcome == "pending"
    with pytest.raises(GraspCalibrationError, match="Y/N"):
        session.capture(
            target=target,
            telemetry=CalibrationTelemetry(),
            detection_stamp_s=11.0,
            image_stamp_s=11.0,
            jpeg_data=b"\xff\xd8second\xff\xd9",
        )
    with pytest.raises(GraspCalibrationError, match="闭爪"):
        session.mark_grasp_outcome("success")
    session.record_gripper_result(
        "close",
        accepted=True,
        acknowledgement="飞控已接受闭爪序列",
    )
    session.mark_grasp_outcome("success")
    session.capture_bad_pose(
        target=target,
        telemetry=CalibrationTelemetry(),
        detection_stamp_s=12.0,
        image_stamp_s=12.0,
        jpeg_data=b"\xff\xd8bad-pose\xff\xd9",
    )
    session.finish()

    with session.csv_path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [row["outcome"] for row in rows] == ["success", "bad_pose"]
    assert rows[0]["gripper_action"] == "close"
    assert rows[0]["gripper_command_accepted"] == "true"
    assert rows[0]["gripper_ack"] == "飞控已接受闭爪序列"
    assert (session.session_directory / rows[0]["screenshot_file"]).is_file()
    recommendation = yaml.safe_load(session.suggestion_path.read_text(encoding="utf-8"))
    assert recommendation["review_required"] is True
    assert recommendation["sample_counts"]["success"] == 1
    assert recommendation["sample_counts"]["bad_pose"] == 1
    assert "echinus" in recommendation["per_target"]
    # 运行时只有一份 session.json；标定模块不得另造重复摘要。
    assert not (session.session_directory / "grasp_summary.json").exists()
    assert not (session.session_directory / "grasp_session.json").exists()
    assert autonomy_config.read_text(encoding="utf-8") == original_config


def test_rejected_close_cannot_be_marked_as_physical_success(tmp_path) -> None:
    """服务拒绝不是抓取结果；Y/N 只能跟在被接受的闭爪命令后。"""

    session = GraspCalibrationSession(
        tmp_path / "session",
        target_labels=("echinus",),
        minimum_success_samples=3,
    )
    target = _calibration_sample(
        1,
        center_x=0.5,
        center_y=0.7,
        area_ratio=0.1,
    ).target
    session.capture(
        target=target,
        telemetry=CalibrationTelemetry(depth_m=0.8, flight_mode="ALT_HOLD"),
        detection_stamp_s=20.0,
        image_stamp_s=20.0,
        jpeg_data=b"\xff\xd8sample\xff\xd9",
        frame_id=42,
        manual_power=0.12,
        command_forward=0.12,
        gripper_profile="dalian",
    )
    session.record_gripper_result(
        "close",
        accepted=False,
        acknowledgement="网关拒绝",
    )
    with pytest.raises(GraspCalibrationError, match="未获网关接受"):
        session.mark_grasp_outcome("success")

    with session.csv_path.open(encoding="utf-8", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert row["frame_id"] == "42"
    assert row["manual_power"] == "0.120000"
    assert row["command_forward"] == "0.120000"
    assert row["gripper_profile"] == "dalian"
    assert row["outcome"] == "pending"
