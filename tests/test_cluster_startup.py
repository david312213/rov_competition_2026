"""群体收集一键入口与 ROS 接口的静态边界。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
WRAPPER = PROJECT / "scripts/start_cluster_collection_test.sh"
COMMON = PROJECT / "scripts/start_search_approach_test.sh"
SETUP = PROJECT / "ros2_ws/src/rov_competition/setup.py"
STATUS = PROJECT / "ros2_ws/src/rov_interfaces/msg/MissionStatus.msg"


def test_cluster_one_click_uses_existing_default_port_pipeline() -> None:
    wrapper = WRAPPER.read_text(encoding="utf-8")
    common = COMMON.read_text(encoding="utf-8")
    assert "--workflow cluster_collection" in wrapper
    for port in ("14550", "14551", "5600", "5700", "5702", "5704"):
        assert port in common
    assert "output/cluster_collection_tests" in common
    assert "rov_cluster_collection_test" in common
    assert 'RUNTIME_MODE="manual"' in common
    assert 'VIEW_BASE_TOPIC="/rov/cluster_image"' in common
    assert 'until ros2 topic list' in common
    assert '"${topic} compressed"' in common


def test_cluster_cli_and_status_fields_are_installed() -> None:
    assert "rov_cluster_collection_test" in SETUP.read_text(encoding="utf-8")
    source = STATUS.read_text(encoding="utf-8")
    for field in (
        "has_cluster",
        "visible_scallop_count",
        "locked_cluster_count",
        "cluster_center_x_ratio",
        "cluster_center_y_ratio",
        "cluster_union_area_ratio",
        "grasp_attempt_index",
        "completed_cluster_count",
        "descent_trigger_mode",
    ):
        assert field in source
