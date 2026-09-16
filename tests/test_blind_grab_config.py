from pathlib import Path

import pytest
import yaml

from blind_grab_test_helpers import config_document
from rov_competition.blind_grab_config import (
    BlindGrabConfigurationError, VisionSettings, load_blind_grab_config, package_config_path,
)
from rov_competition.blind_grab_helpers import prepare_helper_processes


def save(tmp_path, document):
    path = tmp_path / "blind.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_unfilled_template_reports_all_actuator_and_timing_inputs_without_guessing():
    with pytest.raises(BlindGrabConfigurationError) as error:
        load_blind_grab_config(package_config_path("blind_grab.yaml"))
    for name in ["search.lane_forward_duration_s", "grab.advance_duration_s", "grab.release_duration_s",
                 "actions.open_gripper", "actions.close_gripper", "actions.arm_to_basket", "actions.arm_to_grasp"]:
        assert name in str(error.value)


def test_template_contains_confirmed_vehicle_arm_and_gripper_setpoints():
    document = yaml.safe_load(package_config_path("blind_grab.yaml").read_text(encoding="utf-8"))
    actions = document["actions"]
    assert actions["open_gripper"]["outputs"] == [{"output_channel": 11, "pwm": 900}]
    assert actions["close_gripper"]["outputs"] == [{"output_channel": 11, "pwm": 1800}]
    assert actions["arm_to_basket"]["outputs"] == [{"output_channel": 10, "pwm": 900}]
    assert actions["arm_to_grasp"]["outputs"] == [{"output_channel": 10, "pwm": 1800}]


def test_old_software_permission_flags_and_command_limit_do_not_gate_blind_configuration(tmp_path):
    document = config_document()
    document["safety"] = {"allow_live_actuation": False, "allow_ros_arming": False, "allow_gripper_actuation": False}
    document["control"] = {"profile": "commissioning", "command_limit": .001, "expected_frame_config": None}
    c = load_blind_grab_config(save(tmp_path, document))
    assert c.mission.forward_command == .23
    assert c.mission.grabs_per_batch == 3
    assert c.mission.fallback_after_s == 30
    assert c.vision.confidence == .18
    assert c.vision.target_labels == ("scallop",)


def test_multi_output_gripper_and_separate_arm_mapping_are_preserved(tmp_path):
    document = config_document()
    document["actions"]["open_gripper"]["outputs"].append({"output_channel": 22, "pwm": 700})
    document["actions"]["close_gripper"]["outputs"].append({"output_channel": 22, "pwm": 500})
    c = load_blind_grab_config(save(tmp_path, document))
    assert [p.output_channel for p in c.mission.close_gripper.outputs] == [20, 22]
    assert [p.pwm for p in c.mission.close_gripper.outputs] == [1900, 500]
    assert c.mission.arm_to_basket.outputs[0].output_channel == 21


def test_relative_vision_paths_are_relative_to_the_selected_config(tmp_path):
    document = config_document()
    document["vision"]["autonomy_config"] = "vision.yaml"
    c = load_blind_grab_config(save(tmp_path, document))
    assert c.vision.autonomy_config == tmp_path / "vision.yaml"
    assert c.vision.robot_config == package_config_path("robot.example.yaml")


def test_perception_threshold_and_model_location_are_synced_without_editing_source(tmp_path):
    source = tmp_path / "source" / "autonomy.yaml"
    source.parent.mkdir()
    original = {
        "detector": {"confidence_threshold": .45, "model_path": "../models/custom.pt", "expected_sha256": "unchanged"},
        "mission": {"allow_autonomous_mission": False},
    }
    source.write_text(yaml.safe_dump(original))
    settings = VisionSettings(autonomy_config=source, record_video=True)
    processes = prepare_helper_processes(settings, tmp_path / "session")
    resolved = yaml.safe_load((tmp_path / "session/perception_autonomy.yaml").read_text())
    assert resolved["detector"]["confidence_threshold"] == .18
    assert resolved["detector"]["model_path"] == str((tmp_path / "models/custom.pt").resolve())
    assert resolved["detector"]["expected_sha256"] == "unchanged"
    assert yaml.safe_load(source.read_text()) == original
    assert {p.name for p in processes} == {"video_bridge", "perception", "recorder", "viewer"}
    commands = {p.name: p.command(1) for p in processes}
    assert "--no-qgc" in commands["video_bridge"]
    assert "perception_only.launch.py" in commands["perception"]
    assert not any("rov_vehicle" in arg for command in commands.values() for arg in command)
    recorder = next(p for p in processes if p.name == "recorder")
    assert recorder.command(1) != recorder.command(2)
    assert any("video_0002.mkv" in arg for arg in recorder.command(2))
