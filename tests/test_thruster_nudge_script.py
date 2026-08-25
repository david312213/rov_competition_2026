"""推进器消音点动入口必须保持为一次性人工动作。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "scripts/nudge_thrusters_once.sh"


def test_nudge_script_is_fixed_low_power_one_shot() -> None:
    """必须复用一次单轴测试，固定低功率和一秒，不得周期点动。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert 'VALUE="0.05"' in source
    assert 'DURATION_S="1.0"' in source
    assert "rov_axis_test" in source
    assert '--confirm "MOVE ROV"' in source
    assert 'CONFIRMATION="NUDGE ROV ONCE"' in source
    assert "nohup" not in source
    assert "systemd" not in source
    assert source.count("rov_axis_test") == 1


def test_nudge_script_owns_the_complete_ros_gateway_lifecycle() -> None:
    """一条命令应完成网关、许可、解锁、动作和最终上锁。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert "telemetry.launch.py" in source
    assert "enable_actuation:=true" in source
    assert "enable_ros_arming:=true" in source
    assert "/rov/control/set_enabled" in source
    assert "/rov/control/set_armed" in source
    assert "{arm: true, confirmation: 'ARM ROV'}" in source
    assert "{arm: false, confirmation: ''}" in source
    assert "best_effort_lock" in source
    assert "stop_gateway" in source
    assert "14551" in source


def test_nudge_script_reuses_only_an_idle_gateway() -> None:
    """已有网关可复用，但 WASD/自主节点和已解锁状态仍必须拒绝。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert 'REUSE_EXISTING_GATEWAY=false' in source
    assert "复用现有 ROS/MAVLink 飞控网关" in source
    assert "rov_dataset_drive" in source
    assert "rov_search_approach_test" in source
    assert "'armed: false'" in source
    assert 'CONTROL_TOUCHED=false' in source
    assert 'if [[ "${CONTROL_TOUCHED}" == true ]]' in source


def test_nudge_script_documents_real_motion_and_automatic_disarm() -> None:
    """现场提示不能把点动描述成无运动或永久消音。"""

    source = SCRIPT.read_text(encoding="utf-8")
    assert "不能保证艇体完全不移动" in source
    assert "蜂鸣还会再次出现" in source
    assert "点动完成；再次确认上锁" in source
