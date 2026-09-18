from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rov_competition.blind_grab_config import MavlinkSettings, OfficialRosSettings
from rov_competition.blind_grab_official import OfficialRosPublisher
from rov_competition.domain import MotionCommand
from rov_competition.official_data_only_mavlink import PassiveMavlinkTelemetry
from rov_competition.official_data_only_runtime import main


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "ros2_ws/src/rov_competition/config/blind_grab.yaml"


class FakeMessage(SimpleNamespace):
    def get_type(self):
        return self.message_type

    def get_srcSystem(self):
        return 1

    def get_srcComponent(self):
        return 1


class FakeMaster:
    def __init__(self, messages=()):
        self.messages = list(messages)
        self.recv_calls = 0
        self.close_calls = 0
        self.mav = SimpleNamespace(
            manual_control_send=Mock(name="manual_control_send"),
            command_long_send=Mock(name="command_long_send"),
            rc_channels_override_send=Mock(name="rc_channels_override_send"),
            heartbeat_send=Mock(name="heartbeat_send"),
            set_mode_send=Mock(name="set_mode_send"),
            command_int_send=Mock(name="command_int_send"),
        )

    def recv_match(self, *, blocking):
        assert blocking is False
        self.recv_calls += 1
        return self.messages.pop(0) if self.messages else None

    def close(self):
        self.close_calls += 1


def assert_no_flight_controller_writes(master):
    master.mav.manual_control_send.assert_not_called()
    master.mav.command_long_send.assert_not_called()
    master.mav.rc_channels_override_send.assert_not_called()
    master.mav.heartbeat_send.assert_not_called()
    master.mav.set_mode_send.assert_not_called()
    master.mav.command_int_send.assert_not_called()


def test_passive_mavlink_reads_real_telemetry_and_only_closes_connection():
    master = FakeMaster([
        FakeMessage(
            message_type="ATTITUDE",
            roll=0.1,
            pitch=0.2,
            yaw=0.3,
            rollspeed=0.01,
            pitchspeed=0.02,
            yawspeed=0.03,
        ),
        FakeMessage(message_type="AHRS2", altitude=-1.4),
        FakeMessage(message_type="SYS_STATUS", voltage_battery=15400),
    ])
    factory = Mock(return_value=master)
    source = PassiveMavlinkTelemetry(
        MavlinkSettings(),
        OfficialRosSettings(),
        connection_factory=factory,
        report=lambda text: None,
    )

    assert source.poll(5.0) == 3
    snapshot = source.telemetry_snapshot()
    assert snapshot.roll_rad == 0.1
    assert snapshot.depth_m == 1.4
    assert snapshot.battery_voltage_v == 15.4
    assert source.shutdown() == []
    assert master.recv_calls == 4
    assert master.close_calls == 1
    assert_no_flight_controller_writes(master)
    factory.assert_called_once_with(
        "udpin:0.0.0.0:14551",
        baud=115200,
        source_system=255,
        source_component=191,
        autoreconnect=False,
    )


def test_passive_mavlink_receive_error_closes_and_retries_without_writes():
    broken = FakeMaster()
    broken.recv_match = Mock(side_effect=OSError("link lost"))
    recovered = FakeMaster([FakeMessage(message_type="AHRS2", altitude=-2.0)])
    factory = Mock(side_effect=[broken, recovered])
    source = PassiveMavlinkTelemetry(
        MavlinkSettings(reconnect_interval_s=1.0),
        OfficialRosSettings(),
        connection_factory=factory,
        report=lambda text: None,
    )

    assert source.poll(10.0) == 0
    assert broken.close_calls == 1
    assert source.poll(10.5) == 0
    assert factory.call_count == 1
    assert source.poll(11.0) == 1
    assert source.telemetry_snapshot().depth_m == 2.0
    source.shutdown()
    assert recovered.close_calls == 1
    assert_no_flight_controller_writes(broken)
    assert_no_flight_controller_writes(recovered)


class FakeNode:
    def __init__(self):
        self.created = []

    def create_publisher(self, message_type, topic, qos):
        self.created.append((message_type, topic, qos))
        return object()


def test_data_only_ros_publisher_does_not_create_or_update_motion_topics():
    publisher = OfficialRosPublisher(
        OfficialRosSettings(),
        MavlinkSettings(),
        telemetry_source=lambda: None,
        publish_motion=False,
        node_name="rov_official_data_only",
        report=lambda text: None,
    )
    types = {
        "RobotDataMessage": object(),
        "Imu": object(),
        "MagneticField": object(),
        "FluidPressure": object(),
    }
    node = FakeNode()
    created = publisher._create_publishers(node, types)

    assert tuple(created) == ("robot_data", "imu", "magnetometer", "pressure")
    assert publisher.published_topics == (
        "/robot_data", "/imu", "/magnetometer", "/pressure",
    )
    assert {topic for _, topic, _ in node.created}.isdisjoint({"/cmd_vel", "/cmd_accel"})
    publisher.set_desired(MotionCommand(forward=1.0, vertical=1.0))
    assert publisher._desired() == MotionCommand.neutral()


def test_data_only_dry_run_declares_no_control_or_helpers(capsys):
    assert main(["--config", str(CONFIG), "--dry-run"]) == 0
    output = capsys.readouterr().out
    assert '"flight_controller_writes": false' in output
    assert '"motion_publishers_created": false' in output
    assert '"starts_blind_grab": false' in output
    assert '"starts_video_or_yolo": false' in output
