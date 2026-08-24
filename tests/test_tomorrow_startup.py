"""明日联调总向导的静态安全边界。"""

from __future__ import annotations

import subprocess
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT / "scripts/start_tomorrow_test.sh"
SEARCH = PROJECT / "scripts/start_search_approach_test.sh"
VIDEO = PROJECT / "scripts/start_video_test.sh"
ROS_CHECK = PROJECT / "scripts/check_ros.sh"


def test_tomorrow_wizard_exposes_menu_and_every_named_subcommand() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for command in ("status", "qgc", "gripper", "weights", "search", "calibrate"):
        assert f"{command})" in source
    assert "start_gripper_test.sh" in source
    assert "start_search_approach_test.sh" in source
    assert "start_grasp_position_test.sh" in source
    assert "ACTIVATE DALIAN GRIPPER" in source
    assert "ACTIVATE RST GRIPPER" in source
    assert "INSTALL NEW WEIGHT" in source
    assert "ROLLBACK WEIGHT" in source


def test_tomorrow_help_runs_without_ros_or_hardware() -> None:
    completed = subprocess.run(
        ["bash", str(SCRIPT), "--help"],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert completed.returncode == 0
    assert "start_tomorrow_test.sh qgc" in completed.stdout
    assert "start_tomorrow_test.sh calibrate" in completed.stdout


def test_qgc_stage_documents_defaults_and_never_kills_qgc() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    for route in (
        "192.168.2.1:14550",
        "192.168.2.1:14551",
        "192.168.2.1:5600",
        "192.168.2.1:5700",
    ):
        assert route in source
    for free_port in ("14551", "5700", "5702", "5704"):
        assert f'require_free_port "${{port}}"' in source or free_port in source
    assert "QGC READY" in source
    assert "receive_one_udp_packet 14551" in source
    assert "receive_one_udp_packet 5700" in source
    assert "pkill" not in source
    assert "killall" not in source


def test_search_and_video_prefer_local_weight_configuration() -> None:
    for path in (SEARCH, VIDEO):
        source = path.read_text(encoding="utf-8")
        assert "config/autonomy.local.yaml" in source
        assert "weights verify" in source
    search = SEARCH.read_text(encoding="utf-8")
    assert "runtime-config" in search
    assert '--mode "${RUNTIME_MODE}"' in search
    assert 'RUNTIME_MODE="auto"' in search
    assert 'RUNTIME_MODE="manual"' in search
    assert 'robot_config:="${RUNTIME_ROBOT_CONFIG}"' in search


def test_ros_check_requires_new_cli_and_master_script() -> None:
    source = ROS_CHECK.read_text(encoding="utf-8")
    assert "rov_field_setup" in source
    assert "start_tomorrow_test.sh" in source
