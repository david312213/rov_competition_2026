"""群体收集一键入口与 ROS 接口的静态边界。"""

from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
WRAPPER = PROJECT / "scripts/start_cluster_collection_test.sh"
COMMON = PROJECT / "scripts/start_search_approach_test.sh"
SETUP = PROJECT / "ros2_ws/src/rov_competition/setup.py"
STATUS = PROJECT / "ros2_ws/src/rov_interfaces/msg/MissionStatus.msg"
RUNTIME = PROJECT / "ros2_ws/src/rov_competition/rov_competition/cluster_collection_runtime.py"


def test_upward_neutral_bypass_flag_is_scoped_to_cluster_sessions() -> None:
    common = COMMON.read_text(encoding="utf-8")
    assert (
        'RUNTIME_EXTRA_ARGS=()\n'
        'if [[ "${WORKFLOW}" == "cluster_approach" || "${WORKFLOW}" == "cluster_collection" ]]; then\n'
        '  RUNTIME_EXTRA_ARGS+=(--upward-neutral-bypass-slew)\n'
        'fi'
    ) in common
    assert '"${RUNTIME_EXTRA_ARGS[@]}"' in common


def test_cluster_one_click_uses_existing_default_port_pipeline() -> None:
    wrapper = WRAPPER.read_text(encoding="utf-8")
    common = COMMON.read_text(encoding="utf-8")
    assert "--workflow cluster_approach" in wrapper
    assert "cluster_collection" in common
    for port in ("14550", "14551", "5600", "5700", "5702", "5704"):
        assert port in common
    assert "output/cluster_collection_tests" in common
    assert "rov_cluster_collection_test" in common
    assert 'RUNTIME_MODE="manual"' in common
    assert 'VIEW_BASE_TOPIC="/rov/cluster_image"' in common
    assert "cluster_collection_targets.yaml" in common
    assert 'targets_config:="${ACTIVE_TARGETS_CONFIG}"' in common
    assert 'until ros2 topic list' in common
    assert '"${topic} compressed"' in common


def test_cluster_wrapper_defaults_to_no_gripper_search_approach() -> None:
    wrapper = WRAPPER.read_text(encoding="utf-8")
    common = COMMON.read_text(encoding="utf-8")
    runtime = RUNTIME.read_text(encoding="utf-8")

    assert "模式：无机械爪群体搜索—接近；不会发送机械爪或盲抓下降命令。" in wrapper
    assert "--workflow cluster_approach" in wrapper
    assert "cluster_approach" in common
    assert "--stop-after-approach" in common
    assert 'RUNTIME_MODE="auto"' in common
    assert "--stop-after-approach" in runtime
    assert "allow_gripper=not args.stop_after_approach" in runtime
    assert "无机械爪模式异常产生机械爪动作；已禁止发送该命令" in runtime
    assert "无机械爪接近完成后正常上锁未确认；" in runtime
    assert "node.publish_neutral()" in runtime


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
