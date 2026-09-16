import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest
import yaml

from blind_grab_test_helpers import config, config_document
from rov_competition.blind_grab import BlindGrabMission, BlindState
from rov_competition.blind_grab_config import VisionSettings
from rov_competition.blind_grab_helpers import HelperProcess, OptionalHelpers
from rov_competition.blind_grab_runtime import DetectionBuffer, run_control_loop


class VirtualStop:
    def __init__(self, end=45, hook=None):
        self.now = 0.0
        self.end = end
        self.hook = hook
        self.stopped = False

    def is_set(self):
        return self.stopped

    def set(self):
        self.stopped = True

    def wait(self, duration):
        self.now = round(self.now + duration, 9)
        if self.now >= self.end:
            self.set()
        if self.hook is not None:
            self.hook(self.now)


class CaptureOutput:
    def __init__(self, fail=False):
        self.commands = []
        self.fail = fail

    def set_desired(self, motion, servos):
        self.commands.append((motion, servos))
        if self.fail:
            raise OSError("simulated output unavailable")


def message(stamp, labels_and_scores):
    return SimpleNamespace(
        stamp=SimpleNamespace(sec=stamp, nanosec=0),
        detections=[SimpleNamespace(label=label, confidence=confidence) for label, confidence in labels_and_scores],
    )


def test_detection_buffer_counts_only_selected_labels_at_the_synced_confidence():
    buffer = DetectionBuffer(VisionSettings())
    assert buffer.accept(message(10, [("scallop", .18)] * 4 + [("scallop", .17), ("waterweeds", .99)]), 0)
    assert buffer.latest().count == 4
    assert buffer.latest().frame_id == 1
    assert not buffer.accept(message(10, [("scallop", .99)] * 9), .5)
    assert not buffer.accept(message(9, []), .6)
    assert buffer.latest().received_at == 0
    assert buffer.latest().count == 4
    assert buffer.accept(message(11, []), 1.0)
    assert buffer.latest().count == 0
    assert buffer.latest().frame_id == 2
    assert not buffer.stream_lost(1.999, 1.0)
    assert buffer.stream_lost(2.0, 1.0)


def test_detection_stream_loss_immediately_latches_permanent_blind_grab():
    buffer = DetectionBuffer(VisionSettings())
    assert buffer.accept(message(1, []), 0.0)
    stop = VirtualStop(end=1.1)
    mission = BlindGrabMission(config(fallback_after_s=1000))
    run_control_loop(mission, buffer, CaptureOutput(), stop,
                     clock=lambda: stop.now, report=lambda s: None)
    assert mission.permanent
    assert mission.state is BlindState.GRABBING
    assert mission.batch_index == 1


def test_detection_source_exception_latches_without_waiting_for_fallback_timer():
    class FailedSource:
        def latest(self):
            raise RuntimeError("detection transport disconnected")

    stop = VirtualStop(end=.1)
    mission = BlindGrabMission(config(fallback_after_s=1000))
    run_control_loop(mission, FailedSource(), CaptureOutput(), stop,
                     clock=lambda: stop.now, report=lambda s: None)
    assert mission.permanent
    assert mission.state is BlindState.GRABBING
    assert mission.batch_index == 1


@pytest.mark.parametrize("source_fails,output_fails", [(False, False), (True, False), (False, True), (True, True)])
def test_main_loop_keeps_running_and_falls_back_when_components_fail(source_fails, output_fails):
    class Source:
        def latest(self):
            if source_fails:
                raise RuntimeError("model/camera unavailable")
            return None
    stop = VirtualStop()
    output = CaptureOutput(fail=output_fails)
    mission = BlindGrabMission(config())
    run_control_loop(mission, Source(), output, stop, clock=lambda: stop.now, report=lambda s: None)
    assert mission.permanent
    assert mission.state is BlindState.GRABBING
    assert mission.grasp_command_count >= 2
    assert len(output.commands) == 900


@pytest.mark.parametrize("failed_role", ["perception", "video_bridge", "recorder", "viewer"])
@pytest.mark.parametrize("failure", ["startup", "exit"])
def test_auxiliary_process_failure_never_stops_control(tmp_path, failed_role, failure):
    class Process:
        def __init__(self, role):
            self.role = role
        def poll(self):
            return 1 if failure == "exit" and self.role == failed_role else None
    def popen(command, **kwargs):
        if failure == "startup" and command[0] == failed_role:
            raise OSError(f"{failed_role} failed to start")
        return Process(command[0])
    (tmp_path / "logs").mkdir()
    stop = VirtualStop()
    helpers = OptionalHelpers(VisionSettings(), tmp_path, stop, popen=popen, report=lambda s: None)
    helpers.processes = [HelperProcess(role, lambda n, role=role: [role], restart=role != "viewer")
                         for role in ["perception", "video_bridge", "recorder", "viewer"]]
    stop.hook = helpers.tick
    mission = BlindGrabMission(config())
    try:
        run_control_loop(mission, SimpleNamespace(latest=lambda: None), CaptureOutput(), stop,
                         clock=lambda: stop.now, report=lambda s: None)
        assert mission.permanent
        assert mission.grasp_command_count >= 2
        failed = next(c for c in helpers.processes if c.name == failed_role)
        assert failed.attempts >= (1 if failed_role == "viewer" else 2)
    finally:
        for child in helpers.processes:
            if child.log is not None:
                child.log.close()


def python_environment():
    env = dict(os.environ)
    root = Path(__file__).resolve().parents[1] / "ros2_ws/src/rov_competition"
    env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
    return env


def test_dry_run_does_not_load_ros_mavlink_or_start_helpers(tmp_path):
    document = config_document()
    document["vision"]["start_helpers"] = True
    path = tmp_path / "valid.yaml"
    path.write_text(yaml.safe_dump(document))
    code = (
        "import sys; from rov_competition.blind_grab_runtime import main; "
        f"assert main(['--config', {str(path)!r}, '--dry-run']) == 0; "
        "assert 'rclpy' not in sys.modules; assert 'pymavlink' not in sys.modules"
    )
    result = subprocess.run([sys.executable, "-c", code], env=python_environment(), cwd=tmp_path,
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "output").exists()


@pytest.mark.parametrize("stop_signal,wire_version", [(signal.SIGINT, "1"), (signal.SIGTERM, "2"), (signal.SIGHUP, "1")])
def test_real_loopback_mavlink_and_process_signal_cleanup(tmp_path, stop_signal, wire_version):
    """只向测试自身的127.0.0.1临时UDP端口发送；从不连接实艇。"""
    pytest.importorskip("pymavlink")
    from pymavlink.dialects.v20 import ardupilotmega
    decoder = ardupilotmega.MAVLink(None)
    decoder.robust_parsing = True
    packets = []
    document = config_document()
    document["trigger"]["fallback_after_s"] = .15  # 缩短仅此测试的虚拟现场时序。
    document["grab"]["advance_duration_s"] = .10
    document["grab"]["release_duration_s"] = .10
    for action in document["actions"].values():
        action["duration_s"] = .10
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.settimeout(.10)
    port = sock.getsockname()[1]
    document["mavlink"] = {"connection_uri": f"udpout:127.0.0.1:{port}"}
    path = tmp_path / "loopback_only.yaml"
    path.write_text(yaml.safe_dump(document))
    env = python_environment()
    env.pop("MAVLINK20", None)
    if wire_version == "2":
        env["MAVLINK20"] = "1"
    log_path = tmp_path / "process.log"
    process = None
    try:
        with log_path.open("wb") as log:
            process = subprocess.Popen(
                [sys.executable, "-m", "rov_competition.blind_grab_runtime",
                 "--config", str(path), "--no-helpers"],
                env=env, cwd=tmp_path, stdout=log, stderr=subprocess.STDOUT,
            )
            started = time.monotonic()
            while time.monotonic() - started < 8.0:
                try:
                    data, _ = sock.recvfrom(65535)
                    packets.extend(decoder.parse_buffer(data) or [])
                except socket.timeout:
                    pass
                has_grab = any(p.get_type() == "COMMAND_LONG" and p.param2 == 1900 for p in packets)
                if has_grab and time.monotonic() - started >= 1.0:
                    break
                assert process.poll() is None, log_path.read_text()
            assert any(p.get_type() == "MANUAL_CONTROL" and p.x > 0 for p in packets), log_path.read_text()
            process.send_signal(stop_signal)
            assert process.wait(timeout=8) == 0, log_path.read_text()
        while True:
            try:
                data, _ = sock.recvfrom(65535)
                packets.extend(decoder.parse_buffer(data) or [])
            except socket.timeout:
                break
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        sock.close()
    motion = [p for p in packets if p.get_type() == "MANUAL_CONTROL"]
    assert len(motion) >= 5
    assert all((p.x, p.y, p.z, p.r) == (0, 0, 500, 0) for p in motion[-3:])
    release = [p for p in packets if p.get_type() == "RC_CHANNELS_OVERRIDE"]
    assert release and all(getattr(release[-1], f"chan{i}_raw") == 0 for i in range(1, 9))
    if wire_version == "2":
        assert all(getattr(release[-1], f"chan{i}_raw") == 65534 for i in range(9, 19))
    assert all(p.command == 183 for p in packets if p.get_type() == "COMMAND_LONG")
    assert "permanent=true" in log_path.read_text()
