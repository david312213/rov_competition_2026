import os
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from blind_grab_test_helpers import config, config_document
from rov_competition.blind_grab import BlindGrabMission, BlindState, DetectionCount
from rov_competition.blind_grab_config import VisionSettings
from rov_competition.blind_grab_helpers import HelperProcess, OptionalHelpers
from rov_competition.blind_grab_runtime import DetectionBuffer, run_control_loop
from rov_competition.blind_grab_telemetry import BlindTelemetrySnapshot


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
        detections=[
            SimpleNamespace(label=label, confidence=confidence)
            for label, confidence in labels_and_scores
        ],
    )


def no_depth():
    return BlindTelemetrySnapshot()


def fast_config(**changes):
    from dataclasses import replace

    c = config(
        initial_fallback_s=.15,
        ascent_duration_s=.10,
        route_step_duration_s=.10,
        repeat_descent_duration_s=.10,
        shift_duration_s=.10,
        turn_duration_s=.10,
        advance_duration_s=.10,
        release_duration_s=.10,
    )
    c = replace(
        c,
        **{
            name: replace(getattr(c, name), duration_s=.10)
            for name in (
                "open_gripper", "close_gripper", "arm_to_basket", "arm_to_grasp",
            )
        },
    )
    return replace(c, **changes)


def test_detection_buffer_counts_boxes_for_display_only():
    buffer = DetectionBuffer(VisionSettings())
    assert buffer.accept(
        message(10, [("scallop", .18)] * 4 + [("scallop", .17), ("waterweeds", .99)]),
        0,
    )
    assert buffer.latest().count == 4
    assert buffer.latest().frame_id == 1
    assert not buffer.accept(message(10, [("scallop", .99)] * 9), .5)
    assert not buffer.accept(message(9, []), .6)
    assert buffer.latest().received_at == 0
    assert buffer.accept(message(11, []), 1.0)
    assert buffer.latest().count == 0
    assert buffer.latest().frame_id == 2


def test_detection_exception_does_not_skip_initial_descent_or_change_motion():
    class FailedSource:
        def latest(self):
            raise RuntimeError("detection transport disconnected")

    stop = VirtualStop(end=.10)
    mission = BlindGrabMission(config(initial_fallback_s=10.0))
    output = CaptureOutput()
    run_control_loop(
        mission,
        FailedSource(),
        no_depth,
        output,
        stop,
        clock=lambda: stop.now,
        report=lambda text: None,
    )
    assert mission.state is BlindState.INITIAL_DESCENT
    assert output.commands
    assert all(command.vertical == pytest.approx(-.415) for command, _ in output.commands)


def test_many_boxes_and_no_boxes_produce_identical_command_timeline():
    outputs = []
    for visible_count in (0, 100):
        stop = VirtualStop(end=3.0)

        class DetectionSource:
            def __init__(self, timer, count):
                self.timer = timer
                self.count = count

            def latest(self):
                return DetectionCount(
                    frame_id=round(self.timer.now * 20),
                    received_at=self.timer.now,
                    count=self.count,
                )

        output = CaptureOutput()
        run_control_loop(
            BlindGrabMission(fast_config()),
            DetectionSource(stop, visible_count),
            no_depth,
            output,
            stop,
            clock=lambda timer=stop: timer.now,
            report=lambda text: None,
        )
        outputs.append(output.commands)
    assert outputs[0] == outputs[1]


def test_pressure_depth_can_confirm_first_bottom_inside_runtime():
    stop = VirtualStop(end=3.25)

    def depth_source():
        value = 1.0 if stop.now == 0 else 1.20
        return BlindTelemetrySnapshot(depth_m=value, depth_at=stop.now)

    mission = BlindGrabMission(config(initial_fallback_s=10.0))
    run_control_loop(
        mission,
        SimpleNamespace(latest=lambda: None),
        depth_source,
        CaptureOutput(),
        stop,
        clock=lambda: stop.now,
        report=lambda text: None,
    )
    assert mission.bottom_source == "pressure"
    assert mission.state is BlindState.GRABBING


@pytest.mark.parametrize(
    "detection_fails,depth_fails,output_fails",
    [
        (False, False, False),
        (True, False, False),
        (False, True, False),
        (False, False, True),
        (True, True, True),
    ],
)
def test_main_loop_keeps_timing_when_optional_sources_or_output_fail(
    detection_fails, depth_fails, output_fails,
):
    class DetectionSource:
        def latest(self):
            if detection_fails:
                raise RuntimeError("model/camera unavailable")

    def depth_source():
        if depth_fails:
            raise RuntimeError("telemetry unavailable")
        return BlindTelemetrySnapshot()

    stop = VirtualStop(end=5.0)
    output = CaptureOutput(fail=output_fails)
    mission = BlindGrabMission(fast_config())
    run_control_loop(
        mission,
        DetectionSource(),
        depth_source,
        output,
        stop,
        clock=lambda: stop.now,
        report=lambda text: None,
    )
    assert mission.completed_cycles >= 4
    assert mission.state in set(BlindState)
    assert len(output.commands) == 100


@pytest.mark.parametrize("failed_role", ["perception", "video_bridge", "recorder", "viewer"])
@pytest.mark.parametrize("failure", ["startup", "exit"])
def test_auxiliary_process_failure_restarts_and_never_stops_control(
    tmp_path, failed_role, failure,
):
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
    stop = VirtualStop(end=12.0)
    helpers = OptionalHelpers(
        VisionSettings(), tmp_path, stop, popen=popen, report=lambda text: None,
    )
    helpers.processes = [
        HelperProcess(role, lambda n, role=role: [role])
        for role in ["perception", "video_bridge", "recorder", "viewer"]
    ]
    stop.hook = helpers.tick
    mission = BlindGrabMission(fast_config())
    try:
        run_control_loop(
            mission,
            SimpleNamespace(latest=lambda: None),
            no_depth,
            CaptureOutput(),
            stop,
            clock=lambda: stop.now,
            report=lambda text: None,
        )
        assert mission.completed_cycles > 5
        failed = next(child for child in helpers.processes if child.name == failed_role)
        assert failed.attempts >= 2
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
    result = subprocess.run(
        [sys.executable, "-c", code],
        env=python_environment(),
        cwd=tmp_path,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert not (tmp_path / "output").exists()


@pytest.mark.parametrize(
    "stop_signal,wire_version",
    [(signal.SIGINT, "1"), (signal.SIGTERM, "2"), (signal.SIGHUP, "1")],
)
def test_real_loopback_mavlink_and_process_signal_cleanup(
    tmp_path, stop_signal, wire_version,
):
    """只向测试自身的127.0.0.1临时UDP端口发送；从不连接实艇。"""
    pytest.importorskip("pymavlink")
    from pymavlink.dialects.v20 import ardupilotmega

    decoder = ardupilotmega.MAVLink(None)
    decoder.robust_parsing = True
    packets = []
    document = config_document()
    document["vertical"].update(
        initial_fallback_s=.15,
        ascent_duration_s=.10,
        repeat_descent_duration_s=.10,
    )
    document["route"].update(
        step_duration_s=.10,
        shift_duration_s=.10,
        turn_duration_s=.10,
    )
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
                [
                    sys.executable,
                    "-m",
                    "rov_competition.blind_grab_runtime",
                    "--config",
                    str(path),
                    "--no-helpers",
                ],
                env=env,
                cwd=tmp_path,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            started = time.monotonic()
            while time.monotonic() - started < 8.0:
                try:
                    data, _ = sock.recvfrom(65535)
                    packets.extend(decoder.parse_buffer(data) or [])
                except TimeoutError:
                    pass
                has_close = any(
                    packet.get_type() == "COMMAND_LONG" and packet.param2 == 1900
                    for packet in packets
                )
                motion = [
                    packet for packet in packets
                    if packet.get_type() == "MANUAL_CONTROL"
                ]
                has_descent = any(packet.z < 500 for packet in motion)
                has_forward = any(packet.x > 0 for packet in motion)
                if has_close and has_descent and has_forward and time.monotonic() - started >= 1.0:
                    break
                assert process.poll() is None, log_path.read_text()
            motion = [packet for packet in packets if packet.get_type() == "MANUAL_CONTROL"]
            assert any(packet.z < 500 for packet in motion), (
                [packet.z for packet in motion], log_path.read_text()
            )
            assert any(packet.x > 0 for packet in motion), log_path.read_text()
            process.send_signal(stop_signal)
            assert process.wait(timeout=8) == 0, log_path.read_text()
        while True:
            try:
                data, _ = sock.recvfrom(65535)
                packets.extend(decoder.parse_buffer(data) or [])
            except TimeoutError:
                break
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        sock.close()
    motion = [packet for packet in packets if packet.get_type() == "MANUAL_CONTROL"]
    assert len(motion) >= 5
    assert all((packet.x, packet.y, packet.z, packet.r) == (0, 0, 500, 0) for packet in motion[-3:])
    release = [packet for packet in packets if packet.get_type() == "RC_CHANNELS_OVERRIDE"]
    assert release and all(getattr(release[-1], f"chan{i}_raw") == 0 for i in range(1, 9))
    if wire_version == "2":
        assert all(getattr(release[-1], f"chan{i}_raw") == 65534 for i in range(9, 19))
    assert all(packet.command == 183 for packet in packets if packet.get_type() == "COMMAND_LONG")
    log_text = log_path.read_text()
    assert "visual_control=false" in log_text
    assert "bottom=timer" in log_text
