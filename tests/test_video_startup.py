"""一键视频入口必须保持与飞控完全隔离。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
LAUNCH = PROJECT / "ros2_ws/src/rov_competition/launch/video_test.launch.py"
SCRIPT = PROJECT / "scripts/start_video_test.sh"
DATASET_SCRIPT = PROJECT / "scripts/start_dataset_collection.sh"
RECORD_ONLY_SCRIPT = PROJECT / "scripts/start_dataset_recording.sh"
SEARCH_SCRIPT = PROJECT / "scripts/start_search_approach_test.sh"
GRASP_POSITION_SCRIPT = PROJECT / "scripts/start_grasp_position_test.sh"
GRIPPER_TEST_SCRIPT = PROJECT / "scripts/start_gripper_test.sh"
ROS_CHECK_SCRIPT = PROJECT / "scripts/check_ros.sh"
SEARCH_RUNTIME = (
    PROJECT
    / "ros2_ws/src/rov_competition/rov_competition/search_approach_runtime.py"
)
DATASET_RUNTIME = PROJECT / "ros2_ws/src/rov_competition/rov_competition/dataset_drive.py"
PERCEPTION_LAUNCH = (
    PROJECT / "ros2_ws/src/rov_competition/launch/perception_only.launch.py"
)
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
    assert '"perception_only": True' in source


def test_one_click_script_uses_fixed_separate_video_ports() -> None:
    """QGC 直连 5600，只读 YOLO 使用 5700 -> 5702。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert "source_port:=5700" in source
    assert "ai_port:=5702" in source
    assert "--no-qgc" in LAUNCH.read_text(encoding="utf-8")
    assert "set_enabled" not in source
    assert "set_armed" not in source
    assert "emergency_stop" not in source
    assert "enable_actuation:=true" not in source
    assert "enable_ros_arming:=true" not in source
    assert "rov_vehicle_gateway" not in source


def test_plain_video_entry_is_perception_only_without_legacy_calibration() -> None:
    """普通视频入口只能看框；抓取标定必须使用独立安全入口。"""

    source = SCRIPT.read_text(encoding="utf-8")
    for removed in (
        "--calibrate",
        "rov_grasp_calibrate",
        "telemetry.launch.py",
        "output/grasp_calibration",
        "set_gripper",
        "set_armed",
        "set_enabled",
        "emergency_stop",
    ):
        assert removed not in source


def test_visual_dependencies_keep_the_ubuntu_22_04_numpy_abi() -> None:
    """部署依赖必须阻止 NumPy 2 与旧 ABI OpenCV 混装。"""

    source = REQUIREMENTS.read_text(encoding="utf-8")
    assert "numpy==1.26.4" in source
    assert "opencv-python==4.11.0.86" in source
    assert "mavproxy" not in source.lower()


def test_dataset_one_click_entry_has_fixed_routes_and_no_yolo_or_gripper() -> None:
    """键盘采集直接使用 BlueOS 双路输出，不再启动 MAVProxy。"""

    source = DATASET_SCRIPT.read_text(encoding="utf-8")
    for port in ("14550", "14551", "5600", "5700", "5704"):
        assert port in source
    for removed in ("14552", "5701", "5703", "mavproxy.py"):
        assert removed not in source.lower()
    assert "rov_dataset_drive" in source
    assert "enable_actuation:=true" in source
    assert "enable_ros_arming:=true" in source
    assert "--no-inference" in source
    assert "ros2 run rov_competition rov_autonomy" not in source
    assert "competition.launch.py" not in source
    assert "set_gripper" not in source
    assert "seafood_yolo26x.pt" not in source
    assert "import ultralytics" not in source.lower()
    assert "--no-qgc" in source


def test_gamepad_recording_entry_never_touches_the_control_chain() -> None:
    """手柄模式只复制和录制视频，不得接管 MAVLink 或 ROS 控制。"""

    source = RECORD_ONLY_SCRIPT.read_text(encoding="utf-8")
    for port in ("5600", "5700", "5704"):
        assert port in source
    for forbidden in (
        "14550",
        "14551",
        "14552",
        "5701",
        "5703",
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


def test_search_one_click_starts_all_required_rc2_components() -> None:
    """搜索测试使用默认 QGC 端口、独立 ROS/视频输出和完整证据链。"""

    source = SEARCH_SCRIPT.read_text(encoding="utf-8")
    for port in ("14550", "14551", "5600", "5700", "5702", "5704"):
        assert port in source
    for removed in ("14552", "5701", "5703", "mavproxy"):
        assert removed not in source.lower()
    assert "rov_vehicle" in source
    assert "rov_stream_bridge" in source
    assert "perception_only.launch.py" in source
    assert "rov_search_approach_test" in source
    assert "--no-qgc" in source
    assert "START SEARCH TEST" not in source  # 确认词由运行节点校验。
    assert '--workflow "${WORKFLOW}"' in source
    assert "output/search_tests" in source
    assert "output/grasp_tests" in source


def test_grasp_position_wrapper_selects_manual_workflow() -> None:
    """人工标定入口只能是共用状态机的显式 manual 工作流。"""

    source = GRASP_POSITION_SCRIPT.read_text(encoding="utf-8")
    assert "start_search_approach_test.sh" in source
    assert "--workflow manual_grasp_calibration" in source
    assert "rov_grasp_calibrate" not in source


def test_gripper_candidate_entry_is_disarmed_and_has_two_profiles() -> None:
    """机械爪候选台架工具不得启动网关或执行解锁。"""

    source = GRIPPER_TEST_SCRIPT.read_text(encoding="utf-8")
    assert '"dalian"' in source
    assert '"rst"' in source
    assert "rov_gripper_test" in source
    # 可以只读检查是否有旧网关节点，但绝不能启动飞控网关。
    assert "ros2 run rov_competition rov_vehicle" not in source
    assert "telemetry.launch.py" not in source
    assert "set_armed" not in source
    assert "enable_actuation" not in source
    assert "推进器" in source


def test_perception_only_launch_cannot_start_the_production_mission() -> None:
    source = PERCEPTION_LAUNCH.read_text(encoding="utf-8")
    assert '"perception_only": True' in source
    assert '"gstreamer": True' in source
    assert 'default_value="5702"' in source
    assert "rov_vehicle" not in source


def test_search_runtime_reuses_safety_node_with_its_own_ros_name() -> None:
    """搜索节点必须正确复用键盘安全节点，不能因构造参数数量而启动失败。"""

    dataset_source = DATASET_RUNTIME.read_text(encoding="utf-8")
    search_source = SEARCH_RUNTIME.read_text(encoding="utf-8")
    assert 'node_name: str = "rov_dataset_drive"' in dataset_source
    assert 'node_name="rov_search_approach_test"' in search_source


def test_manual_runtime_contains_all_stop_and_calibration_keys() -> None:
    """人工标定入口必须同时保留失焦回中、正常回收和急停路径。"""

    source = SEARCH_RUNTIME.read_text(encoding="utf-8")
    for required in (
        "WINDOWFOCUSLOST",
        "pygame.K_ESCAPE",
        "pygame.K_0",
        "pygame.K_SPACE",
        "pygame.K_g",
        "pygame.K_c",
        "pygame.K_o",
        "pygame.K_y",
        "pygame.K_n",
        "pygame.K_b",
        "pygame.K_r",
        "manual_control.clear()",
        "new_perception_frame = observation.frame_id > mission.last_frame_id",
        "and new_perception_frame",
        "mission.request_normal_finish",
        "node.emergency_stop()",
    ):
        assert required in source


def test_gripper_service_rejection_is_recorded_before_safety_shutdown() -> None:
    """服务拒绝不能被通用封装吞掉，否则 CSV 会误记为无响应。"""

    source = SEARCH_RUNTIME.read_text(encoding="utf-8")
    command_body = source.split("def command_gripper", 1)[1].split(
        "def wait_for_perception", 1
    )[0]
    assert "future.result()" in command_body
    assert "bool(result.success)" in command_body
    assert "self._call(" not in command_body


def test_ros_check_covers_search_executable_launch_and_config() -> None:
    """Ubuntu 完整验收必须检查新增的现场入口。"""

    source = ROS_CHECK_SCRIPT.read_text(encoding="utf-8")
    assert "rov_search_approach_test" in source
    assert "start_search_approach_test.sh" in source
    assert "perception_only.launch.py" in source
    assert "search_test.yaml" in source
    assert "rov_gripper_test" in source
    assert "start_grasp_position_test.sh" in source
    assert "start_gripper_test.sh" in source


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
