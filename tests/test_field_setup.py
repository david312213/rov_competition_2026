"""明日联调的权重、机械爪证据和会话配置测试。"""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
from pathlib import Path

import pytest
import yaml

from rov_competition import field_setup as field_setup_module
from rov_competition.field_setup import (
    BALANCED_TIMEOUT_CONFIRMATION,
    BALANCED_TIMEOUT_VALUES,
    FieldSetupError,
    WeightInspection,
    activate_gripper,
    apply_balanced_timeouts,
    field_paths,
    inspect_balanced_timeouts,
    inspect_weight,
    install_weight,
    main,
    rollback_weight,
    sha256_file,
    validate_gripper_result,
    verify_active_weight,
    verify_balanced_timeouts,
    verify_gripper_activation,
    write_runtime_robot_config,
)


PROJECT = Path(__file__).resolve().parents[1]
PACKAGE = PROJECT / "ros2_ws/src/rov_competition"
ROBOT_EXAMPLE = PACKAGE / "config/robot.example.yaml"
AUTONOMY_EXAMPLE = PACKAGE / "config/autonomy.yaml"
DATASET_EXAMPLE = PACKAGE / "config/dataset.example.yaml"
TARGETS = ("echinus", "holothurian", "scallop", "starfish")


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _make_project(tmp_path: Path) -> Path:
    project = tmp_path / "ROV 现场工程"
    package = project / "ros2_ws/src/rov_competition"
    (project / "config").mkdir(parents=True)
    (package / "config").mkdir(parents=True)
    (package / "models").mkdir(parents=True)
    shutil.copy2(ROBOT_EXAMPLE, project / "config/robot.yaml")
    shutil.copy2(AUTONOMY_EXAMPLE, package / "config/autonomy.yaml")
    shutil.copy2(DATASET_EXAMPLE, package / "config/dataset.example.yaml")
    return project


def _write_active_weight(project: Path, payload: bytes = b"old-weight") -> None:
    paths = field_paths(project)
    paths.active_weight.write_bytes(payload)
    data = yaml.safe_load(paths.source_autonomy.read_text(encoding="utf-8"))
    data["detector"]["expected_sha256"] = _sha(payload)
    data["detector"]["class_names"] = [*TARGETS, "waterweeds"]
    paths.source_autonomy.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def _enable_verified_motion_gates(project: Path) -> None:
    """模拟已完成八推预检的实艇 robot.yaml。"""

    path = field_paths(project).robot_config
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    data["control"]["expected_frame_config"] = 2
    data["safety"]["allow_live_actuation"] = True
    data["safety"]["allow_ros_arming"] = True
    path.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def _inspection(path: Path) -> WeightInspection:
    return WeightInspection(
        source=path.resolve(),
        sha256=sha256_file(path),
        class_names=(*TARGETS, "extra_target"),
        device_name="Fake RTX",
    )


def _write_gripper_result(
    project: Path,
    *,
    profile: str = "dalian",
    outcome: str = "passed",
    opened: bool = True,
    closed: bool = True,
    ack: str = "0",
) -> Path:
    session = project / "output/gripper_tests" / f"session_{profile}"
    session.mkdir(parents=True, exist_ok=True)
    commands = session / "commands.csv"
    with commands.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=("action", "step", "output_channel", "pwm", "ack_result"),
        )
        writer.writeheader()
        writer.writerow(
            {
                "action": "open",
                "step": 0,
                "output_channel": 12,
                "pwm": 1650,
                "ack_result": ack,
            }
        )
        writer.writerow(
            {
                "action": "close",
                "step": 0,
                "output_channel": 12,
                "pwm": 1900,
                "ack_result": ack,
            }
        )
    result = session / "result.json"
    result.write_text(
        json.dumps(
            {
                "profile": profile,
                "outcome": outcome,
                "actual_opened": opened,
                "actual_closed": closed,
                "files": {"commands": commands.name},
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return result


class _FakeCuda:
    def __init__(self, available: bool = True) -> None:
        self.available = available

    def is_available(self) -> bool:
        return self.available

    def get_device_name(self, index: int) -> str:
        assert index == 0
        return "Fake RTX"


class _FakeTorch:
    def __init__(self, available: bool = True) -> None:
        self.cuda = _FakeCuda(available)


class _FakeNumpy:
    uint8 = "uint8"

    @staticmethod
    def zeros(shape: tuple[int, int, int], dtype: object) -> object:
        assert shape == (640, 640, 3)
        assert dtype == "uint8"
        return object()


class _FakeYolo:
    names = {index: name for index, name in enumerate((*TARGETS, "other"))}

    def __init__(self, path: str) -> None:
        assert Path(path).is_file()

    def predict(self, **kwargs: object) -> list[object]:
        assert kwargs["device"] == "0"
        assert kwargs["imgsz"] == 640
        return []


def test_weight_inspection_requires_cuda_and_all_grasp_targets(tmp_path: Path) -> None:
    source = tmp_path / "可信 best.pt"
    source.write_bytes(b"model")
    inspected = inspect_weight(
        source,
        yolo_factory=_FakeYolo,
        torch_module=_FakeTorch(),
        numpy_module=_FakeNumpy(),
    )
    assert inspected.sha256 == _sha(b"model")
    assert inspected.class_names[:4] == TARGETS
    assert inspected.device_name == "Fake RTX"

    with pytest.raises(FieldSetupError, match="CUDA"):
        inspect_weight(
            source,
            yolo_factory=_FakeYolo,
            torch_module=_FakeTorch(False),
            numpy_module=_FakeNumpy(),
        )

    class MissingTargetYolo(_FakeYolo):
        names = {0: "echinus", 1: "scallop"}

    with pytest.raises(FieldSetupError, match="holothurian"):
        inspect_weight(
            source,
            yolo_factory=MissingTargetYolo,
            torch_module=_FakeTorch(),
            numpy_module=_FakeNumpy(),
        )

    def broken_model(_: str) -> object:
        raise RuntimeError("权重文件损坏")

    with pytest.raises(FieldSetupError, match="加载或 CUDA 试推理失败"):
        inspect_weight(
            source,
            yolo_factory=broken_model,
            torch_module=_FakeTorch(),
            numpy_module=_FakeNumpy(),
        )


def test_weight_install_is_atomic_and_rollback_swaps_model_and_config(
    tmp_path: Path,
) -> None:
    project = _make_project(tmp_path)
    _write_active_weight(project)
    source = tmp_path / "含空格的新 best.pt"
    source.write_bytes(b"new-weight")
    result = install_weight(
        project,
        _inspection(source),
        confirmation="INSTALL NEW WEIGHT",
    )
    paths = field_paths(project)
    assert paths.active_weight.read_bytes() == b"new-weight"
    assert paths.previous_weight.read_bytes() == b"old-weight"
    assert result["sha256"] == _sha(b"new-weight")
    local = yaml.safe_load(paths.local_autonomy.read_text(encoding="utf-8"))
    assert local["detector"]["model_path"] == str(paths.active_weight.resolve())
    assert local["detector"]["class_names"] == [*TARGETS, "extra_target"]

    rolled_back = rollback_weight(project, confirmation="ROLLBACK WEIGHT")
    assert paths.active_weight.read_bytes() == b"old-weight"
    assert paths.previous_weight.read_bytes() == b"new-weight"
    assert rolled_back["sha256"] == _sha(b"old-weight")
    assert verify_active_weight(project)["sha256"] == _sha(b"old-weight")


def test_failed_weight_copy_does_not_replace_current_model(tmp_path: Path) -> None:
    project = _make_project(tmp_path)
    _write_active_weight(project)
    source = tmp_path / "bad.pt"
    source.write_bytes(b"new-but-bad")
    bad = WeightInspection(
        source=source,
        sha256="0" * 64,
        class_names=TARGETS,
        device_name="Fake RTX",
    )
    with pytest.raises(FieldSetupError, match="哈希变化"):
        install_weight(project, bad, confirmation="INSTALL NEW WEIGHT")
    paths = field_paths(project)
    assert paths.active_weight.read_bytes() == b"old-weight"
    assert not paths.local_autonomy.exists()


def test_weight_install_preserves_existing_local_tuning(tmp_path: Path) -> None:
    project = _make_project(tmp_path)
    _write_active_weight(project)
    paths = field_paths(project)
    local = yaml.safe_load(paths.source_autonomy.read_text(encoding="utf-8"))
    local["detector"]["model_path"] = str(paths.active_weight.resolve())
    local["detector"]["confidence_threshold"] = 0.31
    local["field_tuning"]["descent_delta_m"] = 0.42
    paths.local_autonomy.write_text(
        yaml.safe_dump(local, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    source = tmp_path / "new.pt"
    source.write_bytes(b"new-weight")
    install_weight(
        project,
        _inspection(source),
        confirmation="INSTALL NEW WEIGHT",
    )
    updated = yaml.safe_load(paths.local_autonomy.read_text(encoding="utf-8"))
    assert updated["detector"]["confidence_threshold"] == pytest.approx(0.31)
    assert updated["field_tuning"]["descent_delta_m"] == pytest.approx(0.42)
    assert updated["detector"]["expected_sha256"] == _sha(b"new-weight")


def test_failure_after_weight_replace_restores_previous_active_files(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """即使最后的激活记录写入失败，旧模型仍必须可用。"""

    project = _make_project(tmp_path)
    _write_active_weight(project)
    source = tmp_path / "new.pt"
    source.write_bytes(b"new-weight")

    def fail_record(*_: object, **__: object) -> None:
        raise OSError("模拟磁盘写入失败")

    monkeypatch.setattr(field_setup_module, "_atomic_yaml", fail_record)
    with pytest.raises(OSError, match="磁盘"):
        install_weight(
            project,
            _inspection(source),
            confirmation="INSTALL NEW WEIGHT",
        )
    paths = field_paths(project)
    assert paths.active_weight.read_bytes() == b"old-weight"
    assert not paths.local_autonomy.exists()


def test_gripper_activation_requires_complete_human_and_ack_evidence(
    tmp_path: Path,
) -> None:
    project = _make_project(tmp_path)
    bad = _write_gripper_result(project, ack="4")
    original = field_paths(project).robot_config.read_bytes()
    with pytest.raises(FieldSetupError, match="COMMAND_ACK"):
        activate_gripper(
            project,
            bad,
            profile="dalian",
            confirmation="ACTIVATE DALIAN GRIPPER",
        )
    assert field_paths(project).robot_config.read_bytes() == original


def test_status_reports_missing_robot_config_without_crashing(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    project = _make_project(tmp_path)
    field_paths(project).robot_config.unlink()
    assert main(["--project-dir", str(project), "status"]) == 0
    assert "robot: NOT READY" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("profile", "confirmation"),
    (
        ("dalian", "ACTIVATE DALIAN GRIPPER"),
        ("rst", "ACTIVATE RST GRIPPER"),
    ),
)
def test_passed_gripper_can_be_activated_and_manual_runtime_checks_evidence(
    tmp_path: Path,
    profile: str,
    confirmation: str,
) -> None:
    project = _make_project(tmp_path)
    _enable_verified_motion_gates(project)
    result = _write_gripper_result(project, profile=profile)
    evidence = validate_gripper_result(project, result, expected_profile=profile)
    assert evidence.command_count == 2
    record = activate_gripper(
        project,
        result,
        profile=profile,
        confirmation=confirmation,
    )
    assert Path(str(record["backup_path"])).is_file()
    valid, reason = verify_gripper_activation(project)
    assert valid, reason

    robot = yaml.safe_load(field_paths(project).robot_config.read_text(encoding="utf-8"))
    assert robot["gripper"]["active_profile"] == profile
    assert robot["gripper"]["profiles"][profile]["calibrated"] is True
    assert robot["safety"]["allow_gripper_actuation"] is True
    if profile == "rst":
        assert robot["gripper"]["profiles"][profile]["allow_extended_pwm"] is True

    automatic = tmp_path / f"auto-{profile}.yaml"
    enabled, _ = write_runtime_robot_config(project, automatic, mode="auto")
    assert enabled is False
    assert yaml.safe_load(automatic.read_text())["safety"][
        "allow_gripper_actuation"
    ] is False

    manual = tmp_path / f"manual-{profile}.yaml"
    enabled, _ = write_runtime_robot_config(project, manual, mode="manual")
    assert enabled is True
    assert yaml.safe_load(manual.read_text())["safety"][
        "allow_gripper_actuation"
    ] is True

    # 证据被改动后，人工标定仍能记位置，但 C/O 必须被关闭。
    result.write_text(result.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    invalid_manual = tmp_path / f"invalid-manual-{profile}.yaml"
    enabled, reason = write_runtime_robot_config(
        project, invalid_manual, mode="manual"
    )
    assert enabled is False
    assert "发生变化" in reason


def test_passed_rst_activation_upgrades_legacy_robot_config_with_backup(
    tmp_path: Path,
) -> None:
    """RST 实物通过后可升级 rc1 旧配置，但必须先留原文件备份。"""

    project = _make_project(tmp_path)
    _enable_verified_motion_gates(project)
    paths = field_paths(project)
    robot = yaml.safe_load(paths.robot_config.read_text(encoding="utf-8"))
    robot["gripper"] = {
        "output_channel": 12,
        "open_pwm": 1650,
        "close_pwm": 1900,
    }
    paths.robot_config.write_text(
        yaml.safe_dump(robot, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    legacy_bytes = paths.robot_config.read_bytes()
    result = _write_gripper_result(project, profile="rst")

    record = activate_gripper(
        project,
        result,
        profile="rst",
        confirmation="ACTIVATE RST GRIPPER",
    )

    backup = Path(str(record["backup_path"]))
    assert backup.read_bytes() == legacy_bytes
    upgraded = yaml.safe_load(paths.robot_config.read_text(encoding="utf-8"))
    assert upgraded["gripper"]["active_profile"] == "rst"
    assert set(upgraded["gripper"]["profiles"]) == {"dalian", "rst"}
    assert upgraded["gripper"]["profiles"]["rst"]["calibrated"] is True
    assert upgraded["gripper"]["profiles"]["rst"]["allow_extended_pwm"] is True
    assert upgraded["safety"]["allow_gripper_actuation"] is True
    valid, reason = verify_gripper_activation(project)
    assert valid, reason


def _write_legacy_local_timeouts(project: Path) -> tuple[bytes, bytes, bytes]:
    """写入现场旧阈值，用于验证 Git 忽略配置的迁移。"""

    paths = field_paths(project)
    robot = yaml.safe_load(paths.robot_config.read_text(encoding="utf-8"))
    robot["mavlink"]["heartbeat_stale_timeout_s"] = 1.5
    robot["mavlink"]["telemetry_stale_timeout_s"] = 1.0
    robot["control"]["maximum_command_age_s"] = 0.25
    robot["control"]["command_timeout_s"] = 0.5
    robot["safety"]["maximum_pilot_input_timeout_s"] = 3.0
    paths.robot_config.write_text(
        yaml.safe_dump(robot, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    dataset = yaml.safe_load(paths.dataset_template.read_text(encoding="utf-8"))
    dataset["safety"]["maximum_telemetry_age_s"] = 0.75
    dataset["safety"]["maximum_status_age_s"] = 2.0
    # 现场旧文件还可能完全没有后来新增的姿态连续无效阈值。
    dataset["safety"].pop("maximum_attitude_age_s", None)
    paths.dataset_config.write_text(
        yaml.safe_dump(dataset, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )

    autonomy = yaml.safe_load(paths.source_autonomy.read_text(encoding="utf-8"))
    autonomy["mission"]["target_lost_timeout_s"] = 0.8
    autonomy["mission"]["reacquire_grace_s"] = 0.5
    autonomy["mission"]["maximum_heartbeat_age_s"] = 1.5
    autonomy["mission"]["maximum_message_age_s"] = 0.8
    autonomy["mission"].pop("perception_hold_timeout_s", None)
    autonomy["mission"]["maximum_perception_age_s"] = 0.75
    autonomy["mission"]["maximum_control_status_age_s"] = 0.75
    paths.local_autonomy.write_text(
        yaml.safe_dump(autonomy, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return (
        paths.robot_config.read_bytes(),
        paths.dataset_config.read_bytes(),
        paths.local_autonomy.read_bytes(),
    )


def test_balanced_timeout_apply_requires_exact_confirmation_and_keeps_files(
    tmp_path: Path,
) -> None:
    project = _make_project(tmp_path)
    original_robot, original_dataset, original_autonomy = (
        _write_legacy_local_timeouts(project)
    )
    paths = field_paths(project)

    with pytest.raises(FieldSetupError, match="确认词"):
        apply_balanced_timeouts(project, confirmation="APPLY")

    assert paths.robot_config.read_bytes() == original_robot
    assert paths.dataset_config.read_bytes() == original_dataset
    assert paths.local_autonomy.read_bytes() == original_autonomy
    assert not paths.timing_record.exists()


def test_balanced_timeout_apply_backs_up_updates_and_verifies_local_configs(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    project = _make_project(tmp_path)
    original_robot, original_dataset, original_autonomy = (
        _write_legacy_local_timeouts(project)
    )
    paths = field_paths(project)

    before = inspect_balanced_timeouts(project)
    assert before["robot.control.command_timeout_s"][2] is True
    assert before["dataset.safety.maximum_attitude_age_s"] == (
        None,
        5.0,
        False,
    )
    assert all(
        not matched
        for name, (_, _, matched) in before.items()
        if name
        not in {
            "robot.control.command_timeout_s",
            "autonomy.mission.perception_hold_timeout_s",
        }
    )

    record = apply_balanced_timeouts(
        project,
        confirmation=BALANCED_TIMEOUT_CONFIRMATION,
    )

    robot_backup = Path(str(record["robot_backup"]))
    dataset_backup = Path(str(record["dataset_backup"]))
    autonomy_backup = Path(str(record["autonomy_backup"]))
    assert robot_backup.read_bytes() == original_robot
    assert dataset_backup.read_bytes() == original_dataset
    assert autonomy_backup.read_bytes() == original_autonomy
    assert paths.timing_record.is_file()
    assert record["values"] == BALANCED_TIMEOUT_VALUES
    assert record["unchanged_flight_controller_parameters"] == [
        "FS_PILOT_TIMEOUT",
        "FS_PILOT_INPUT",
        "FS_GCS_ENABLE",
    ]

    after = inspect_balanced_timeouts(project)
    assert all(matched for _, _, matched in after.values())
    valid, reason = verify_balanced_timeouts(project)
    assert valid, reason

    robot = yaml.safe_load(paths.robot_config.read_text(encoding="utf-8"))
    assert robot["control"]["command_timeout_s"] == pytest.approx(0.5)
    assert robot["safety"]["maximum_pilot_input_timeout_s"] == pytest.approx(3.0)
    autonomy = yaml.safe_load(paths.local_autonomy.read_text(encoding="utf-8"))
    assert autonomy["mission"]["perception_hold_timeout_s"] == pytest.approx(1.0)
    assert autonomy["mission"]["maximum_perception_age_s"] == pytest.approx(5.0)

    assert main(["--project-dir", str(project), "status"]) == 0
    assert "timing: OK" in capsys.readouterr().out


def test_balanced_timeout_failed_verification_rolls_back_both_configs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = _make_project(tmp_path)
    original_robot, original_dataset, original_autonomy = (
        _write_legacy_local_timeouts(project)
    )
    paths = field_paths(project)

    monkeypatch.setattr(
        field_setup_module,
        "verify_balanced_timeouts",
        lambda _project: (False, "模拟校验失败"),
    )
    with pytest.raises(FieldSetupError, match="模拟校验失败"):
        apply_balanced_timeouts(
            project,
            confirmation=BALANCED_TIMEOUT_CONFIRMATION,
        )

    assert paths.robot_config.read_bytes() == original_robot
    assert paths.dataset_config.read_bytes() == original_dataset
    assert paths.local_autonomy.read_bytes() == original_autonomy
    assert not paths.timing_record.exists()
