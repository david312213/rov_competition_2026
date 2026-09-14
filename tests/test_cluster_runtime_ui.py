"""不连接 ROS/飞控的鼠标焦点、启动确认与控制热路径回归测试。"""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from rov_competition.cluster_runtime_support import LoopTiming, focus_pause_reason
from rov_competition.cluster_window_bridge import WindowInput


PACKAGE = Path(__file__).resolve().parents[1] / "ros2_ws/src/rov_competition/rov_competition"
RUNTIME = PACKAGE / "cluster_collection_runtime.py"
PG = SimpleNamespace(ACTIVEEVENT=1, WINDOWFOCUSLOST=2, WINDOWMINIMIZED=3,
                     WINDOWHIDDEN=4, WINDOWLEAVE=5, WINDOWENTER=6,
                     QUIT=7, KEYDOWN=8, K_ESCAPE=9, K_RETURN=10, K_KP_ENTER=11)


@pytest.mark.parametrize("event", [
    SimpleNamespace(type=1, gain=0, state=1),
    SimpleNamespace(type=1, gain=1, state=2),
    SimpleNamespace(type=1, gain=1, state=4),
    SimpleNamespace(type=5), SimpleNamespace(type=6),
])
def test_mouse_leave_and_focus_gain_do_not_pause(event):
    assert focus_pause_reason(PG, event) is None


@pytest.mark.parametrize("event", [
    SimpleNamespace(type=1, gain=0, state=2),
    SimpleNamespace(type=1, gain=0, state=3),
    SimpleNamespace(type=1, gain=0, state=4),
    SimpleNamespace(type=1, gain=0, state=7),
    SimpleNamespace(type=1, gain=0),
    SimpleNamespace(type=2), SimpleNamespace(type=3), SimpleNamespace(type=4),
])
def test_keyboard_loss_minimize_and_unknown_loss_still_pause(event):
    assert focus_pause_reason(PG, event)


def test_timing_detects_blocking_stage_without_changing_control():
    now = [10.0]
    timing = LoopTiming(clock=lambda: now[0])
    now[0] += 0.01
    timing.mark("ros_callbacks")
    now[0] += 0.6
    timing.mark("disk")
    message = timing.warning()
    assert "disk=0.600s" in message
    assert "运动看门狗未放宽" in message
    assert timing.warning() is None
    now[0] += 3
    timing.begin()
    now[0] += 0.01
    assert timing.warning() is None


def _load_wait_function(namespace):
    tree = ast.parse(RUNTIME.read_text(encoding="utf-8"))
    function = next(item for item in tree.body if isinstance(item, ast.FunctionDef)
                    and item.name == "_wait_for_operator_start")
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(RUNTIME), "exec"), namespace)
    return namespace[function.name]


def _run_wait(states, *, perception_age=0.0, prearm_error=None):
    calls = []
    states = iter(states)
    window = SimpleNamespace(poll=lambda: next(states), alive=True, created_at=0.0,
                             show=lambda lines: None)
    node = SimpleNamespace(spin=lambda timeout: calls.append("spin"),
                           wait_for_perception=lambda **kw: calls.append("perception"),
                           perception_age_s=lambda: perception_age)
    ns = {"rclpy": SimpleNamespace(ok=lambda: True), "time": SimpleNamespace(monotonic=lambda: 1),
          "DatasetDriveError": RuntimeError, "focus_pause_reason": focus_pause_reason,
          "_prearm_error": lambda *a, **kw: prearm_error}
    _load_wait_function(ns)(window, node, None, allow_gripper=False)
    return calls


def test_start_wait_ignores_stale_focus_and_requires_new_enter():
    calls = _run_wait([None, WindowInput(1.0, True, True, 0, False, False, "等待启动"),
                       WindowInput(1.0, True, False, 1, False, False, "")])
    assert calls == ["spin", "spin", "spin", "perception"]


@pytest.mark.parametrize("reason", ["控制窗口被关闭", "Esc 急停"])
def test_cancel_in_start_window_does_not_arm(reason):
    with pytest.raises(RuntimeError, match="未解锁"):
        _run_wait([WindowInput(1.0, True, True, 0, False, True, reason)])


def test_unfocused_enter_does_not_start():
    with pytest.raises(RuntimeError, match="取消启动"):
        _run_wait([WindowInput(1.0, False, False, 1, False, False, ""),
                   WindowInput(1.0, False, True, 1, False, True, "Esc")])


def test_focus_lost_after_enter_in_same_batch_does_not_start():
    with pytest.raises(RuntimeError, match="取消启动"):
        _run_wait([WindowInput(1.0, False, True, 1, False, False, "键盘失焦"),
                   WindowInput(1.0, False, True, 1, False, True, "Esc")])


def test_stale_perception_or_failed_prearm_does_not_start():
    with pytest.raises(RuntimeError, match="检测帧已过期"):
        _run_wait([WindowInput(1.0, True, False, 1, False, False, "")], perception_age=0.6)
    with pytest.raises(RuntimeError, match="故障"):
        _run_wait([WindowInput(1.0, True, False, 1, False, False, "")], prearm_error="故障")


def test_control_path_does_not_subscribe_decode_or_publish_jpeg():
    source = RUNTIME.read_text(encoding="utf-8")
    assert 'capture_images=False, node_name="rov_cluster_collection_test"' in source
    for expensive_operation in ("cv2.imdecode", "cv2.imencode", "publish_cluster_overlay", "CompressedImage"):
        assert expensive_operation not in source
    assert '"-m", "rov_competition.cluster_overlay"' in source
    assert "_stop_overlay(overlay_process)" in source
    assert source.index("descent_power, start_depth = _interactive_parameters(") < source.index("window = ClusterWindowBridge(")
    assert source.index("start_enter_seq = _wait_for_operator_start(") < source.index("        node.arm()")
    assert "window_gate.evaluate(" in source
    for forbidden in ("pygame.", "import pygame", "font.render", "display.flip", "clock.tick"):
        assert forbidden not in source


def test_only_new_decisions_are_published_and_safety_checks_remain():
    source = RUNTIME.read_text(encoding="utf-8")
    loop = source[source.index("        while rclpy.ok():"):]
    assert loop.index("error = _active_error(") < loop.index("decision = mission.step(")
    assert loop.index("node.publish(decision.motion)") < loop.index("logger.write_frame(")
    for safety in ("action.error", "action.pause_reason", "原始视频录像停止增长",
                   "PerceptionFreshness.ABORT", "node.emergency_stop()"):
        assert safety in source
    assert "Thread(" not in source  # no blind keepalive of the last movement


def test_overlay_has_no_actuation_and_keeps_original_image_timestamp():
    source = (PACKAGE / "cluster_overlay.py").read_text(encoding="utf-8")
    assert '"/rov/cluster_image/compressed"' in source
    assert "message.header = image.header" in source
    assert "os.getppid() != self.parent_pid" in source
    for forbidden in ("create_client(", "MavlinkVehicle", "pymavlink", "/rov/control/command"):
        assert forbidden not in source
