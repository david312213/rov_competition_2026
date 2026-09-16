from dataclasses import replace
import math
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

from rov_competition.blind_grab_config import MavlinkSettings, OfficialRosSettings
from rov_competition.blind_grab_official import (
    OfficialForwarderSupervisor,
    applied_motion,
    command_acceleration,
    forwarder_command,
    official_robot_data,
)
from rov_competition.blind_grab_official_mavlink import OfficialTelemetryMavlinkOutput
from rov_competition.blind_grab_telemetry import BlindTelemetrySnapshot
from rov_competition.domain import MotionCommand


ROOT = Path(__file__).resolve().parents[1]


def test_copied_official_package_preserves_interface_fields_and_subscribed_topics():
    fields = [
        line.strip() for line in (
            ROOT / "ros2_ws/src/ros2_topic_forwarding/msg/RobotDataMessage.msg"
        ).read_text().splitlines() if line.strip()
    ]
    assert fields == [
        "float64 roll", "float64 yaw", "float64 cabin_hmi", "float64 pitch",
        "float64 longitude", "float64 latitude", "float64 depth", "float64 speed",
        "float64 cabin_temp", "int64 times", "float64 magnetic_field",
        "float64 accelerated_speed", "float64 cabin_pres", "float64 battery_vol",
        "float64 claw_cur",
    ]
    source = (ROOT / "ros2_ws/src/ros2_topic_forwarding/src/topic_forwarding.cpp").read_text()
    for topic in [
        '"/imu"', '"/cmd_vel"', '"/cmd_accel"', '"/joy"',
        '"/magnetometer"', '"/pressure"', '"/robot_data"',
    ]:
        assert topic in source
    assert 'j.dump() + "\\r\\n"' in source


def test_official_motion_matches_mavlink_directions_and_acceleration_is_a_derivative():
    current = applied_motion(
        MotionCommand(forward=.23, lateral=-.2, vertical=.1, yaw=.4),
        (-1, 1, -1, 1),
    )
    assert current == (-.23, -.2, -.1, .4)
    assert command_acceleration(current, (0, 0, 0, 0), .5) == pytest.approx(
        (-.46, -.4, -.2, .8)
    )
    assert command_acceleration(current, None, 0) == (0, 0, 0, 0)


def test_robot_data_uses_fresh_real_values_and_zeros_unavailable_or_stale_fields():
    snapshot = BlindTelemetrySnapshot(
        roll_rad=math.radians(5), pitch_rad=math.radians(-3), yaw_rad=math.radians(45),
        attitude_at=10,
        acceleration_m_s2=(3, 4, 0), acceleration_at=10,
        magnetic_field_t=(0, 3e-5, 4e-5), magnetic_field_at=10,
        depth_m=2.2, depth_at=10,
        latitude_deg=38.9, longitude_deg=121.6, position_at=10,
        speed_m_s=.7, speed_at=10,
        battery_voltage_v=16.4, battery_at=10,
    )
    fresh = official_robot_data(snapshot, now=12, wall_time=1234.9, stale_s=5)
    assert (fresh.roll, fresh.pitch, fresh.yaw) == pytest.approx((5, -3, 45))
    assert (fresh.latitude, fresh.longitude, fresh.depth, fresh.speed) == pytest.approx(
        (38.9, 121.6, 2.2, .7)
    )
    assert fresh.accelerated_speed == 5
    assert fresh.magnetic_field == pytest.approx(5e-5)
    assert fresh.battery_vol == 16.4
    assert fresh.times == 1234
    assert fresh.cabin_hmi == fresh.cabin_temp == fresh.cabin_pres == fresh.claw_cur == 0
    stale = official_robot_data(snapshot, now=16, wall_time=2000, stale_s=5)
    assert stale.roll == stale.depth == stale.battery_vol == stale.latitude == 0


def test_forwarder_command_uses_official_executable_and_supplied_platform_endpoint():
    settings = replace(OfficialRosSettings(), server_ip="example.test", server_port=4567)
    command = forwarder_command(settings)
    assert command[:4] == ["ros2", "run", "ros2_topic_forwarding", "topic_forwarding"]
    assert "server_ip:=example.test" in command
    assert "server_port:=4567" in command


class FakeMav:
    def __init__(self):
        self.motion = []
        self.servo = []
        self.heartbeat = []

    def manual_control_send(self, *args):
        self.motion.append(args)

    def command_long_send(self, *args):
        self.servo.append(args)

    def heartbeat_send(self, *args):
        self.heartbeat.append(args)

    def rc_channels_override_send(self, *args):
        pass


class FakeMessage(SimpleNamespace):
    def get_type(self):
        return self.message_type

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.messages = [
            FakeMessage(message_type="ATTITUDE", roll=0.1, pitch=0.2, yaw=0.3,
                        rollspeed=0.0, pitchspeed=0.0, yawspeed=0.0),
            FakeMessage(message_type="AHRS2", altitude=-1.5),
            FakeMessage(message_type="SYS_STATUS", voltage_battery=15000),
        ]

    def recv_match(self, *, blocking):
        assert not blocking
        return self.messages.pop(0) if self.messages else None

    def close(self):
        pass


def test_mavlink_output_collects_telemetry_without_waiting_or_gating_commands():
    master = FakeMaster()
    output = OfficialTelemetryMavlinkOutput(
        MavlinkSettings(), OfficialRosSettings(),
        connection_factory=lambda *args, **kwargs: master,
        report=lambda text: None,
    )
    assert output.publish(MotionCommand(forward=.23), (), 1.0)
    snapshot = output.telemetry_snapshot()
    assert snapshot.roll_rad == .1
    assert snapshot.depth_m == 1.5
    assert snapshot.battery_voltage_v == 15
    assert master.mav.motion


class ExitedProcess:
    next_pid = 900000

    def __init__(self):
        self.pid = ExitedProcess.next_pid
        ExitedProcess.next_pid += 1
        self.signals = []

    def poll(self):
        return 1

    def send_signal(self, value):
        self.signals.append(value)

    def wait(self, timeout=None):
        return 1

    def terminate(self):
        pass

    def kill(self):
        pass


def test_forwarder_exit_is_restarted_without_affecting_caller(tmp_path):
    processes = []

    def popen(*args, **kwargs):
        process = ExitedProcess()
        processes.append(process)
        return process

    settings = replace(OfficialRosSettings(), forwarder_restart_s=.01)
    supervisor = OfficialForwarderSupervisor(
        settings, tmp_path / "forwarder.log", popen=popen, report=lambda text: None,
    )
    supervisor.start()
    deadline = time.monotonic() + 1.0
    while len(processes) < 2 and time.monotonic() < deadline:
        time.sleep(.01)
    supervisor.shutdown()
    assert len(processes) >= 2
    assert supervisor.attempts >= 2
