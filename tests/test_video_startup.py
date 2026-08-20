"""一键视频入口必须保持与飞控完全隔离。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
LAUNCH = PROJECT / "ros2_ws/src/rov_competition/launch/video_test.launch.py"
SCRIPT = PROJECT / "scripts/start_video_test.sh"


def test_video_launch_contains_only_stream_and_perception_nodes() -> None:
    """测试 launch 可以分流和识别，但不得启动飞控网关。"""

    source = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="rov_stream_bridge"' in source
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
