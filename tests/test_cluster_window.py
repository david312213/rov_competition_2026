"""独立窗口/IPC 的故障注入；无 ROS、无 MAVLink、无机器人控制。"""

from dataclasses import replace
import os
from pathlib import Path
import socket
import time
from types import SimpleNamespace

import pytest

from rov_competition.cluster_window import apply_input
from rov_competition.cluster_window_bridge import (
    ClusterWindowBridge, WindowGate, WindowInput, send_packet, receive_packets,
)


PG = SimpleNamespace(ACTIVEEVENT=1, WINDOWFOCUSLOST=2, WINDOWLEAVE=5, QUIT=7,
                     KEYDOWN=8, K_ESCAPE=9, K_RETURN=10, K_KP_ENTER=11,
                     K_SPACE=12, K_0=13, K_KP0=14)
READY = WindowInput(10.0, True, False, 1, False, False, "")


def test_stall_hold_abort_and_no_automatic_resume():
    gate = WindowGate(1)
    assert not gate.evaluate(READY, now=10.9, alive=True).pause_reason
    assert gate.evaluate(READY, now=11.1, alive=True).pause_reason
    assert gate.evaluate(READY, now=13.1, alive=True).error
    recovered = replace(READY, sent_at=13.2)
    assert not gate.evaluate(recovered, now=13.2, alive=True).resume
    assert gate.evaluate(replace(recovered, enter_seq=2), now=13.2, alive=True).resume
    assert not gate.evaluate(replace(recovered, enter_seq=2), now=13.2, alive=True).resume


def test_dead_ui_estop_missing_and_keyboard_loss_fail_closed():
    gate = WindowGate(1)
    assert gate.evaluate(READY, now=10, alive=False).error
    assert gate.evaluate(None, now=10, alive=True).error
    assert gate.evaluate(replace(READY, estop=True, reason="Esc"), now=10, alive=True).error == "Esc"
    assert gate.evaluate(replace(READY, focused=False), now=10, alive=True).pause_reason
    assert gate.evaluate(replace(READY, paused=True, reason="Space"), now=10, alive=True).pause_reason == "Space"
    assert gate.evaluate(replace(READY, finish_requested=True), now=10, alive=True).finish


def test_enter_space_focus_and_stop_intents_are_latched():
    state = dict(focused=True, paused=False, enter_seq=0, finish_requested=False, estop=False, reason="")
    apply_input(PG, [SimpleNamespace(type=1, gain=0, state=1), SimpleNamespace(type=5)], state, focused=True)
    assert not state["paused"]
    apply_input(PG, [SimpleNamespace(type=8, key=12)], state, focused=True)
    assert state["paused"]
    apply_input(PG, [SimpleNamespace(type=8, key=10)], state, focused=False)
    assert state["paused"] and state["enter_seq"] == 0
    apply_input(PG, [SimpleNamespace(type=8, key=10)], state, focused=True)
    assert not state["paused"] and state["enter_seq"] == 1
    apply_input(PG, [SimpleNamespace(type=8, key=13), SimpleNamespace(type=8, key=9)], state, focused=True)
    apply_input(PG, [SimpleNamespace(type=8, key=10)], state, focused=True)
    assert state["finish_requested"] and state["estop"]  # Enter cannot undo stop


def test_input_parser_does_not_refresh_old_timestamps_or_accept_malformed_data():
    packet = dict(kind="input", sent_at=5.0, focused=True, paused=False, enter_seq=1,
                  finish_requested=False, estop=False, reason="")
    assert WindowInput.parse(packet, 10).sent_at == 5.0
    for patch in ({"sent_at": float("nan")}, {"sent_at": 11.0}, {"focused": "true"},
                  {"enter_seq": True}, {"enter_seq": -1}, {"kind": "view"}):
        assert WindowInput.parse({**packet, **patch}, 10) is None


@pytest.mark.skipif(os.name != "posix", reason="Ubuntu private datagram IPC")
def test_full_display_buffer_does_not_block_sender():
    parent, child = socket.socketpair(type=socket.SOCK_DGRAM)
    try:
        parent.setblocking(False)
        child.setblocking(False)
        started = time.monotonic()
        results = [send_packet(parent, {"kind": "view", "lines": ["x" * 200]}) for _ in range(1000)]
        assert not all(results)
        assert time.monotonic() - started < 0.5
        assert receive_packets(child)
        assert send_packet(parent, {"kind": "view", "lines": ["latest"]})
    finally:
        parent.close()
        child.close()


# Inject slow actual Pygame drawing in a disposable child, never in the controller.
SLOW_GUI = r'''
import os, socket, sys, time
import pygame
from rov_competition.cluster_window import run_window, draw_window
real_get = pygame.event.get
entered = False
def events():
    global entered
    result = real_get()
    if not entered:
        entered = True
        result.append(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_RETURN))
    return result
pygame.event.get = events
pygame.key.get_focused = lambda: True
def slow(*args):
    draw_window(*args)
    time.sleep(0.85)
channel = socket.socket(fileno=int(sys.argv[1]))
channel.setblocking(False)
try:
    run_window(channel, pygame, parent_alive=lambda: os.getppid() == int(sys.argv[2]), draw=slow)
finally:
    channel.close()
'''


@pytest.mark.skipif(os.name != "posix", reason="Ubuntu subprocess window")
def test_slow_drawing_isolated_from_parent_and_window_death_detected(tmp_path, monkeypatch):
    pytest.importorskip("pygame")
    import rov_competition.cluster_window_bridge as bridge_module
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    real_popen = bridge_module.subprocess.Popen
    def slow_child(args, **kwargs):
        fd = args[args.index("--ipc-fd") + 1]
        pid = args[args.index("--parent-pid") + 1]
        return real_popen([args[0], "-c", SLOW_GUI, fd, pid], **kwargs)
    monkeypatch.setattr(bridge_module.subprocess, "Popen", slow_child)
    window = ClusterWindowBridge(tmp_path / "slow_window.log")
    try:
        deadline = time.monotonic() + 10
        while window.poll() is None and time.monotonic() < deadline:
            assert window.alive, (tmp_path / "slow_window.log").read_text()
            time.sleep(0.01)
        assert window.state is not None
        gate = WindowGate(window.state.enter_seq)
        ticks, ages = [], []
        protective_holds = 0
        started = time.monotonic()
        while time.monotonic() - started < 3.0:
            tick = time.monotonic()
            state = window.poll()
            action = gate.evaluate(state, now=tick, alive=window.alive)
            assert not action.error
            # A loaded host can add scheduling/drawing time to the injected sleep.
            # In that case require the documented neutral hold, not an unsafe bypass.
            age = tick - state.sent_at
            if age > 1.0:
                assert action.pause_reason
                protective_holds += 1
            else:
                assert not action.pause_reason
            window.show(["OFFLINE ONLY - render delay 0.85s"])
            ticks.append(time.monotonic())
            ages.append(tick - state.sent_at)
            time.sleep(max(0.0, 0.05 - (time.monotonic() - tick)))
        gaps = [b - a for a, b in zip(ticks, ticks[1:])]
        assert max(ages) > 0.7  # the injected paint blockage actually occurred
        assert len(ticks) >= 50 and max(gaps) < 0.45
        print(f"GUI ISOLATION: draw_delay=0.85s parent_ticks={len(ticks)} max_gap={max(gaps):.3f}s protective_holds={protective_holds}")
        window.process.kill()
        window.process.wait(timeout=2)
        assert gate.evaluate(window.poll(), now=time.monotonic(), alive=window.alive).error
    finally:
        window.close()


def test_gui_and_ipc_have_no_robot_control_path():
    package = Path(__file__).resolve().parents[1] / "ros2_ws/src/rov_competition/rov_competition"
    for name in ("cluster_window.py", "cluster_window_bridge.py"):
        source = (package / name).read_text(encoding="utf-8")
        for forbidden in ("import rclpy", "pymavlink", "create_publisher", "MotionCommand"):
            assert forbidden not in source


@pytest.mark.skipif(os.name != "posix", reason="Ubuntu subprocess window")
def test_real_window_entrypoint_starts_disarmed_and_closes(tmp_path, monkeypatch):
    pytest.importorskip("pygame")
    monkeypatch.setenv("SDL_VIDEODRIVER", "dummy")
    monkeypatch.setenv("SDL_AUDIODRIVER", "dummy")
    window = ClusterWindowBridge(tmp_path / "window.log")
    try:
        deadline = time.monotonic() + 10
        while window.poll() is None and time.monotonic() < deadline:
            assert window.alive, (tmp_path / "window.log").read_text()
            time.sleep(0.01)
        assert window.state is not None
        assert window.state.paused and window.state.enter_seq == 0
        window.show(["DISARMED OFFLINE SMOKE TEST"])
    finally:
        window.close()
    assert not window.alive
