import pytest
import yaml
from blind_grab_test_helpers import config_document
from rov_competition.blind_grab_config import (
    BlindGrabConfigurationError,
    VisionSettings,
    load_blind_grab_config,
    package_config_path,
)
from rov_competition.blind_grab_helpers import prepare_helper_processes


def save(tmp_path, document):
    path = tmp_path / "blind.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def test_vehicle_template_contains_permanent_dive_route_and_action_timing():
    config = load_blind_grab_config(package_config_path("blind_grab.yaml"))
    mission = config.mission
    assert mission.initial_descent_command == pytest.approx(-.8)
    assert mission.initial_bottom_stable_s == 3
    assert mission.initial_bottom_tolerance_m == pytest.approx(.05)
    assert mission.initial_minimum_descent_m == pytest.approx(.10)
    assert mission.initial_fallback_s == 10
    assert mission.ascent_command == pytest.approx(.8)
    assert mission.ascent_duration_s == 2
    assert mission.repeat_descent_command == pytest.approx(-.8)
    assert mission.repeat_descent_duration_s == 5
    assert mission.route_forward_command == pytest.approx(.23)
    assert mission.route_step_duration_s == 5
    assert mission.route_steps_per_lane == 4
    assert mission.shift_duration_s == 4.5
    assert mission.turn_duration_s == 11.5
    assert mission.advance_duration_s == 1
    assert mission.release_duration_s == 2
    assert mission.open_gripper.duration_s == .5
    assert mission.close_gripper.duration_s == .5
    assert mission.arm_to_basket.duration_s == 1.8
    assert mission.arm_to_grasp.duration_s == 2
    assert config.official_ros.enabled
    assert (config.official_ros.server_ip, config.official_ros.server_port) == (
        "api.bjetone.com", 40197,
    )


def test_template_contains_confirmed_vehicle_arm_and_gripper_setpoints():
    document = yaml.safe_load(
        package_config_path("blind_grab.yaml").read_text(encoding="utf-8")
    )
    actions = document["actions"]
    assert actions["open_gripper"]["outputs"] == [{"output_channel": 11, "pwm": 730}]
    assert actions["close_gripper"]["outputs"] == [{"output_channel": 11, "pwm": 575}]
    assert actions["arm_to_basket"]["outputs"] == [{"output_channel": 10, "pwm": 1810}]
    assert actions["arm_to_grasp"]["outputs"] == [{"output_channel": 10, "pwm": 710}]


def test_old_permission_flags_and_visual_trigger_values_do_not_gate_mission(tmp_path):
    document = config_document()
    document["safety"] = {
        "allow_live_actuation": False,
        "allow_ros_arming": False,
        "allow_gripper_actuation": False,
    }
    document["control"] = {
        "profile": "commissioning",
        "command_limit": .001,
        "expected_frame_config": None,
    }
    document["trigger"] = {
        "required_boxes": 99,
        "fallback_after_s": 9999,
    }
    loaded = load_blind_grab_config(save(tmp_path, document))
    assert loaded.mission.route_forward_command == .23
    assert loaded.mission.initial_fallback_s == 10
    assert loaded.vision.confidence == .18
    assert loaded.vision.target_labels == ("scallop",)


def test_legacy_local_config_without_new_sections_gets_new_defaults(tmp_path):
    document = config_document()
    document.pop("vertical")
    document.pop("route")
    document["search"] = {
        "lane_forward_duration_s": 20.0,
        "forward_command": .25,
        "shift_command": .21,
        "shift_duration_s": 4.6,
        "turn_command": .22,
        "turn_duration_s": 11.6,
    }
    document["trigger"] = {
        "required_boxes": 4,
        "confirmation_frames": 5,
        "fallback_after_s": 30,
    }
    document["grab"]["grabs_per_batch"] = 10
    loaded = load_blind_grab_config(save(tmp_path, document))
    mission = loaded.mission
    assert mission.initial_descent_command == pytest.approx(-.8)
    assert mission.initial_fallback_s == 10
    assert mission.ascent_duration_s == 2
    assert mission.repeat_descent_duration_s == 5
    assert mission.route_steps_per_lane == 4
    assert mission.route_step_duration_s == 5
    assert mission.route_forward_command == .25
    assert mission.shift_command == .21
    assert mission.turn_command == .22


def test_multi_output_gripper_and_separate_arm_mapping_are_preserved(tmp_path):
    document = config_document()
    document["actions"]["open_gripper"]["outputs"].append(
        {"output_channel": 22, "pwm": 700}
    )
    document["actions"]["close_gripper"]["outputs"].append(
        {"output_channel": 22, "pwm": 500}
    )
    loaded = load_blind_grab_config(save(tmp_path, document))
    assert [point.output_channel for point in loaded.mission.close_gripper.outputs] == [20, 22]
    assert [point.pwm for point in loaded.mission.close_gripper.outputs] == [1900, 500]
    assert loaded.mission.arm_to_basket.outputs[0].output_channel == 21


def test_invalid_route_value_is_rejected_before_connecting(tmp_path):
    document = config_document()
    document["route"]["steps_per_lane"] = 0
    with pytest.raises(BlindGrabConfigurationError, match="route.steps_per_lane"):
        load_blind_grab_config(save(tmp_path, document))


def test_relative_vision_paths_are_relative_to_selected_config(tmp_path):
    document = config_document()
    document["vision"]["autonomy_config"] = "vision.yaml"
    loaded = load_blind_grab_config(save(tmp_path, document))
    assert loaded.vision.autonomy_config == tmp_path / "vision.yaml"
    assert loaded.vision.robot_config == package_config_path("robot.example.yaml")


def test_perception_threshold_model_and_restartable_viewer_are_prepared(tmp_path):
    source = tmp_path / "source" / "autonomy.yaml"
    source.parent.mkdir()
    original = {
        "detector": {
            "confidence_threshold": .45,
            "model_path": "../models/custom.pt",
            "expected_sha256": "unchanged",
        },
        "mission": {"allow_autonomous_mission": False},
    }
    source.write_text(yaml.safe_dump(original))
    settings = VisionSettings(autonomy_config=source, record_video=True)
    processes = prepare_helper_processes(
        settings, tmp_path / "session",
        "rtmp://api.bjetone.com/ros/40197",
    )
    resolved = yaml.safe_load((tmp_path / "session/perception_autonomy.yaml").read_text())
    assert resolved["detector"]["confidence_threshold"] == .18
    assert resolved["detector"]["model_path"] == str((tmp_path / "models/custom.pt").resolve())
    assert resolved["detector"]["expected_sha256"] == "unchanged"
    assert yaml.safe_load(source.read_text()) == original
    assert {process.name for process in processes} == {
        "video_bridge", "perception", "recorder", "viewer",
    }
    assert all(process.restart for process in processes)
    commands = {process.name: process.command(1) for process in processes}
    assert "--no-qgc" in commands["video_bridge"]
    assert commands["video_bridge"][-2:] == [
        "--rtmp", "rtmp://api.bjetone.com/ros/40197",
    ]
    assert "perception_only.launch.py" in commands["perception"]
    assert not any(
        "rov_vehicle" in argument
        for command in commands.values()
        for argument in command
    )
    recorder = next(process for process in processes if process.name == "recorder")
    assert recorder.command(1) != recorder.command(2)
    assert any("video_0002.mkv" in argument for argument in recorder.command(2))
