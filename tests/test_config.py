"""八推进器控制配置的路径解析和硬安全门测试。"""

from pathlib import Path

import pytest
from rov_competition.config import (
    ConfigurationError,
    ControlProfile,
    ControlProtocol,
    load_autonomy_config,
    load_dataset_config,
    load_robot_config,
)
from rov_competition.gripper_test import write_candidate_config

PROJECT = Path(__file__).resolve().parents[1]
PACKAGE = PROJECT / "ros2_ws" / "src" / "rov_competition"
ROBOT_EXAMPLE = PACKAGE / "config" / "robot.example.yaml"
DATASET_EXAMPLE = PACKAGE / "config" / "dataset.example.yaml"


def test_dataset_template_has_no_software_depth_limit() -> None:
    """数据采集模板只限制键盘幅值，不包含软件深度上限。"""

    dataset = load_dataset_config(DATASET_EXAMPLE)
    assert dataset.initial_command == pytest.approx(0.20)
    assert dataset.maximum_command == pytest.approx(0.80)
    assert dataset.maximum_attitude_age_s == pytest.approx(3.0)
    assert dataset.maximum_status_age_s == pytest.approx(2.0)
    errors = dataset.readiness_errors(load_robot_config(ROBOT_EXAMPLE))
    assert not any("深度" in error or "depth" in error.lower() for error in errors)
    assert "depth_safety" not in DATASET_EXAMPLE.read_text(encoding="utf-8")


def test_legacy_dataset_status_timeout_is_raised_to_desktop_minimum(
    tmp_path: Path,
) -> None:
    """旧现场配置的 0.75 秒阈值也应自动获得 2 秒调度余量。"""

    source = DATASET_EXAMPLE.read_text(encoding="utf-8")
    legacy = tmp_path / "dataset.yaml"
    legacy.write_text(
        source.replace("maximum_status_age_s: 2.0", "maximum_status_age_s: 0.75"),
        encoding="utf-8",
    )
    assert load_dataset_config(legacy).maximum_status_age_s == pytest.approx(2.0)


def test_example_robot_configuration_is_safe_and_explicit() -> None:
    """仓库模板必须默认禁止动作，且尚未猜测实艇机架。"""

    config = load_robot_config(ROBOT_EXAMPLE)
    assert config.allow_live_actuation is False
    assert config.allow_ros_arming is False
    assert config.allow_gripper_actuation is False
    assert config.expected_frame_config is None
    assert config.expected_motor_count == 8
    assert config.control_protocol == ControlProtocol.MANUAL_CONTROL
    assert config.control_profile == ControlProfile.COMMISSIONING
    assert config.command_limit == pytest.approx(0.80)
    assert config.gripper.profile == "dalian"
    assert config.gripper.output_channels == (12,)
    assert config.gripper.calibrated is False
    assert config.gripper.allow_extended_pwm is False
    output = config.gripper.outputs[0]
    assert output.open_sweep.values() == (
        1775,
        1800,
        1825,
        1850,
        1875,
        1900,
    )
    assert output.close_sweep.values() == tuple(range(1900, 1600, -25))
    assert config.rc_override.channels["forward"] == 5
    assert set(config.axis_directions) == {
        "forward",
        "lateral",
        "vertical",
        "yaw",
    }


def test_gripper_permission_requires_explicit_calibration(tmp_path: Path) -> None:
    """历史曲线未做本艇实测前，不得仅靠权限开关驱动机械爪。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "unsafe_gripper.yaml"
    path.write_text(
        source.replace("expected_frame_config: null", "expected_frame_config: 2")
        .replace("allow_live_actuation: false", "allow_live_actuation: true")
        .replace("allow_gripper_actuation: false", "allow_gripper_actuation: true"),
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="gripper.calibrated"):
        load_robot_config(path)


def test_legacy_one_shot_gripper_config_remains_readable_but_uncalibrated(
    tmp_path: Path,
) -> None:
    """旧实艇 robot.yaml 不应妨碍遥测启动，但不能绕过新版标定门。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    start = source.index("gripper:\n")
    end = source.index("\ndepth:\n", start)
    legacy = source[:start] + (
        "gripper:\n"
        "  output_channel: 12\n"
        "  open_pwm: 1650\n"
        "  close_pwm: 1900\n"
    ) + source[end + 1 :]
    path = tmp_path / "legacy_robot.yaml"
    path.write_text(legacy, encoding="utf-8")
    config = load_robot_config(path)
    assert config.gripper.calibrated is False
    assert config.gripper.profile == "legacy"
    assert config.gripper.outputs[0].open_sweep.values() == (1650,)
    assert config.gripper.outputs[0].close_sweep.values() == (1900,)


def test_rst_profile_preserves_exact_two_output_commands(tmp_path: Path) -> None:
    """RST 候选必须忠实保留 AUX3/AUX2 的两路历史值。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "rst_robot.yaml"
    path.write_text(
        source.replace('active_profile: "dalian"', 'active_profile: "rst"'),
        encoding="utf-8",
    )
    config = load_robot_config(path)
    assert config.gripper.profile == "rst"
    assert config.gripper.output_channels == (11, 10)
    assert config.gripper.steps_for("open") == (((11, 950), (10, 1050)),)
    assert config.gripper.steps_for("close") == (((11, 1450), (10, 500)),)
    assert config.gripper.uses_extended_pwm
    assert not config.gripper.allow_extended_pwm


def test_rst_extended_pwm_requires_a_separate_explicit_gate(tmp_path: Path) -> None:
    """仅标定并开启爪子仍不能偷偷发送 RST 的 500us 值。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "unsafe_rst.yaml"
    path.write_text(
        source.replace('active_profile: "dalian"', 'active_profile: "rst"')
        .replace("expected_frame_config: null", "expected_frame_config: 2")
        .replace("allow_live_actuation: false", "allow_live_actuation: true")
        .replace("allow_gripper_actuation: false", "allow_gripper_actuation: true")
        .replace("calibrated: false", "calibrated: true"),
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="allow_extended_pwm"):
        load_robot_config(path)


def test_autonomy_model_path_and_hash_are_stable() -> None:
    """自主配置必须解析到工程内复制并校验过的模型。"""

    config = load_autonomy_config(PACKAGE / "config" / "autonomy.yaml")
    assert config.detector.model_path.name == "seafood_yolo26x.pt"
    assert config.detector.model_path.is_file()
    assert config.detector.expected_sha256 == (
        "300eb3d98ae586c1c5b26b87b3a1baf6450bf6ef367acad4fc5129622ce20a23"
    )
    assert config.detector.device == "0"
    assert config.mission.descent_delta_m == pytest.approx(0.30)
    assert config.mission.advance_target_distance_m == pytest.approx(0.40)
    assert config.mission.advance_duration_s == pytest.approx(4.0 / 3.0)
    assert set(config.mission.grasp_area_ratios) == {
        "echinus",
        "holothurian",
        "scallop",
        "starfish",
    }
    assert config.mission.image_yaw_sign is None
    assert config.mission.readiness_errors(config.detector.class_names[:4])


def test_missing_axis_direction_is_rejected(tmp_path: Path) -> None:
    """四轴中任何一轴缺失时，连接飞控前就应报错。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "bad.yaml"
    path.write_text(source.replace("    lateral: 1\n", ""), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="directions"):
        load_robot_config(path)


def test_string_false_cannot_bypass_actuation_gate(tmp_path: Path) -> None:
    """带引号的 ``false`` 不能利用 Python 真值规则开启硬件输出。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "unsafe_bool.yaml"
    path.write_text(
        source.replace("allow_live_actuation: false", 'allow_live_actuation: "false"'),
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="布尔值"):
        load_robot_config(path)


def test_duplicate_rc_motion_channels_are_rejected(tmp_path: Path) -> None:
    """RC 兼容表中两个轴映射到同一通道必须报错。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "duplicate_channel.yaml"
    path.write_text(
        source.replace("    lateral: 6", "    lateral: 5"), encoding="utf-8"
    )
    with pytest.raises(ConfigurationError, match="复用"):
        load_robot_config(path)


def test_live_output_requires_a_selected_eight_thruster_frame(tmp_path: Path) -> None:
    """没有根据实物选定 FRAME_CONFIG 2/3 前，不得打开真实输出。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    path = tmp_path / "frame_unknown.yaml"
    path.write_text(
        source.replace("allow_live_actuation: false", "allow_live_actuation: true"),
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="expected_frame_config"):
        load_robot_config(path)


def test_ros_arming_cannot_be_enabled_without_live_output(tmp_path: Path) -> None:
    """ROS 解锁权限不能独立于真实输出权限打开。"""

    source = ROBOT_EXAMPLE.read_text(encoding="utf-8")
    source = source.replace("expected_frame_config: null", "expected_frame_config: 2")
    source = source.replace("allow_ros_arming: false", "allow_ros_arming: true")
    path = tmp_path / "arming_without_output.yaml"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(ConfigurationError, match="allow_ros_arming"):
        load_robot_config(path)


def test_class_names_must_be_a_list_not_a_string(tmp_path: Path) -> None:
    """类别配置写成单个字符串时，不能被错误拆成一个个字符。"""

    source = (PACKAGE / "config" / "autonomy.yaml").read_text(encoding="utf-8")
    start = source.index("  class_names:")
    end = source.index("\n\nmission:", start)
    path = tmp_path / "bad_classes.yaml"
    path.write_text(
        source[:start] + '  class_names: "echinus"' + source[end:],
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="字符串列表"):
        load_autonomy_config(path)


def test_advance_duration_is_derived_not_duplicated(tmp_path: Path) -> None:
    """改变估计速度后，前进时间必须自动由距离/速度重算。"""

    source = (PACKAGE / "config" / "autonomy.yaml").read_text(encoding="utf-8")
    path = tmp_path / "slower.yaml"
    path.write_text(
        source.replace("advance_estimated_speed_mps: 0.30", "advance_estimated_speed_mps: 0.20"),
        encoding="utf-8",
    )
    assert load_autonomy_config(path).mission.advance_duration_s == pytest.approx(2.0)


def test_live_readiness_lists_every_uncalibrated_gate() -> None:
    """默认候选包必须一次列出权限、三项标定和两个图像方向。"""

    mission = load_autonomy_config(PACKAGE / "config" / "autonomy.yaml").mission
    errors = mission.readiness_errors(("echinus", "holothurian", "scallop", "starfish"))
    assert len(errors) == 7
    assert any("image_yaw_sign" in item for item in errors)
    assert any("grasp_thresholds_confirmed" in item for item in errors)


@pytest.mark.parametrize(
    ("profile", "outputs", "extended"),
    (
        ("dalian", (12,), False),
        ("rst", (11, 10), True),
    ),
)
def test_gripper_candidate_config_selects_profile_without_editing_source(
    tmp_path: Path,
    profile: str,
    outputs: tuple[int, ...],
    extended: bool,
) -> None:
    """候选工具只写安全副本，并强制关闭正常实机的三道门。"""

    original = ROBOT_EXAMPLE.read_bytes()
    destination = tmp_path / f"{profile}.yaml"
    config = write_candidate_config(ROBOT_EXAMPLE, profile, destination)

    assert config.gripper.profile == profile
    assert config.gripper.output_channels == outputs
    assert config.gripper.uses_extended_pwm is extended
    assert config.allow_live_actuation is False
    assert config.allow_ros_arming is False
    assert config.allow_gripper_actuation is False
    assert ROBOT_EXAMPLE.read_bytes() == original
