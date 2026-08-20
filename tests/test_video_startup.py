"""一键视频入口必须保持与飞控完全隔离。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
LAUNCH = PROJECT / "ros2_ws/src/rov_competition/launch/video_test.launch.py"
SCRIPT = PROJECT / "scripts/start_video_test.sh"
DATASET_SCRIPT = PROJECT / "scripts/start_dataset_collection.sh"
RECORD_ONLY_SCRIPT = PROJECT / "scripts/start_dataset_recording.sh"
GAMEPAD_README = PROJECT / "docs/手柄采集README.md"
KEYBOARD_README = PROJECT / "docs/键盘采集README.md"
REQUIREMENTS = PROJECT / "requirements.txt"


def test_video_launch_contains_only_stream_and_perception_nodes() -> None:
    """测试 launch 可以分流和识别，但不得启动飞控网关。"""

    source = LAUNCH.read_text(encoding="utf-8")
    assert "ExecuteProcess(" in source
    assert '"rov_stream_bridge"' in source
    assert 'executable="rov_autonomy"' in source
    assert "rov_vehicle" not in source
    assert "set_armed" not in source
    assert "mission/start" not in source
    assert '"gstreamer": True' in source
    assert '"udp_mpegts": False' in source
    assert '"display_window": False' in source


def test_one_click_script_uses_fixed_separate_video_ports() -> None:
    """一键入口固定使用 5600/5701/5702，并且没有任何解锁命令。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert "source_port:=5600" in source
    assert "qgc_port:=5701" in source
    assert "ai_port:=5702" in source
    assert "set_enabled" not in source
    assert "set_armed" not in source
    assert "emergency_stop" not in source
    assert "enable_actuation:=true" not in source
    assert "enable_ros_arming:=true" not in source
    assert "rov_vehicle_gateway" in source


def test_visual_dependencies_keep_the_ubuntu_22_04_numpy_abi() -> None:
    """部署依赖必须阻止 NumPy 2 与旧 ABI OpenCV 混装。"""

    source = REQUIREMENTS.read_text(encoding="utf-8")
    assert "numpy==1.26.4" in source
    assert "opencv-python==4.11.0.86" in source


def test_dataset_one_click_entry_has_fixed_routes_and_no_yolo_or_gripper() -> None:
    """采集入口编排 MAVLink/视频两条链，但不启动识别和机械爪。"""

    source = DATASET_SCRIPT.read_text(encoding="utf-8")
    for port in ("14550", "14551", "14552", "5600", "5701", "5702"):
        assert port in source
    assert "rov_dataset_drive" in source
    assert "enable_actuation:=true" in source
    assert "enable_ros_arming:=true" in source
    assert "--no-inference" in source
    assert "ros2 run rov_competition rov_autonomy" not in source
    assert "competition.launch.py" not in source
    assert "set_gripper" not in source
    assert "seafood_yolo26x.pt" not in source
    assert "import ultralytics" not in source.lower()
    assert 'QGC 仍在监听 ${MAVLINK_SOURCE_PORT}' in source
    assert (
        'for port in "${ROS_MAVLINK_PORT}" "${VIDEO_SOURCE_PORT}" '
        '"${RECORD_VIDEO_PORT}"' in source
    )


def test_gamepad_recording_entry_never_touches_the_control_chain() -> None:
    """手柄模式只复制和录制视频，不得接管 MAVLink 或 ROS 控制。"""

    source = RECORD_ONLY_SCRIPT.read_text(encoding="utf-8")
    for port in ("5600", "5701", "5702"):
        assert port in source
    for forbidden in (
        "14550",
        "14551",
        "14552",
        "mavproxy.py",
        "rov_vehicle",
        "rov_dataset_drive",
        "rov_autonomy",
        "set_armed",
        "set_enabled",
        "emergency_stop",
        "set_gripper",
        "ultralytics",
    ):
        assert forbidden not in source.lower()
    assert "rov_dataset_record" in source
    assert "rov_stream_bridge" in source
    assert "--no-inference" in source
    assert "QGC_EXECUTABLE" in source
    assert "flatpak run org.mavlink.qgroundcontrol" in source


def test_dataset_readmes_expose_one_command_daily_entries() -> None:
    """两种采集方式都必须有独立中文说明和唯一日常入口。"""

    gamepad = GAMEPAD_README.read_text(encoding="utf-8")
    keyboard = KEYBOARD_README.read_text(encoding="utf-8")
    assert "一键启动" in gamepad
    assert "./scripts/start_dataset_recording.sh" in gamepad
    assert "不会让机器人停车、上浮或上锁" in gamepad
    assert "一键启动" in keyboard
    assert "./scripts/start_dataset_collection.sh" in keyboard
    assert "START DATASET ROV" in keyboard
    assert "正常结束一定按 0" in keyboard
