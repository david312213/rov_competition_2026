from dataclasses import replace

import pytest

from rov_competition.blind_grab import ServoSetpoint
from rov_competition.blind_grab_config import MavlinkSettings
from rov_competition.blind_grab_mavlink import BlindMavlinkOutput
from rov_competition.domain import MotionCommand


class FakeMav:
    def __init__(self):
        self.motion = []
        self.servo = []
        self.heartbeat = []
        self.release = []
        self.fail_motion = False
        self.only_eight_channels = False

    def manual_control_send(self, *values):
        if self.fail_motion:
            raise OSError("simulated send failure")
        self.motion.append(values)

    def command_long_send(self, *values):
        self.servo.append(values)

    def heartbeat_send(self, *values):
        self.heartbeat.append(values)

    def rc_channels_override_send(self, *values):
        if self.only_eight_channels and len(values) > 10:
            raise TypeError("MAVLink1 signature")
        self.release.append(values)


class FakeMaster:
    def __init__(self):
        self.mav = FakeMav()
        self.closed = False
        self.recv_count = 0
        self.endless_receive = False
        self.receive_error = False

    def recv_match(self, *, blocking):
        assert blocking is False
        self.recv_count += 1
        if self.receive_error:
            raise OSError("simulated receive failure")
        return object() if self.endless_receive else None

    def close(self):
        self.closed = True

    def wait_heartbeat(self, *args, **kwargs):
        raise AssertionError("blind output must not wait for heartbeat")


def adapter(master=None, settings=None):
    master = master or FakeMaster()
    return BlindMavlinkOutput(settings or MavlinkSettings(), connection_factory=lambda *a, **k: master, report=lambda s: None), master


def test_output_without_telemetry_or_arming_feedback_uses_configured_raw_values():
    output, master = adapter(settings=replace(MavlinkSettings(), directions=(-1, 1, -1, 1)))
    servos = (ServoSetpoint(20, 1100), ServoSetpoint(21, 1800))
    assert output.publish(MotionCommand(forward=.23, lateral=-.2, vertical=.2, yaw=.15), servos, 0)
    assert master.mav.motion == [(1, -230, -200, 400, 150, 0)]
    assert [p[4:6] for p in master.mav.servo] == [(20, 1100), (21, 1800)]
    assert all(p[2] == 183 for p in master.mav.servo)
    assert master.recv_count == 1


def test_motion_streams_every_tick_and_servo_poses_repeat_or_change_without_ack():
    output, master = adapter()
    pose = (ServoSetpoint(20, 1100),)
    for now in [0, .05, .10, .25]:
        assert output.publish(MotionCommand(forward=.23), pose, now)
    assert len(master.mav.motion) == 4
    assert len(master.mav.servo) == 2
    assert output.publish(MotionCommand.neutral(), (ServoSetpoint(20, 1900),), .26)
    assert len(master.mav.servo) == 3
    assert master.mav.servo[-1][5] == 1900


def test_a_backlog_of_incoming_packets_cannot_starve_motion_output():
    output, master = adapter()
    master.endless_receive = True
    assert output.publish(MotionCommand(forward=.23), (), 0)
    assert master.recv_count == 32
    assert len(master.mav.motion) == 1


def test_connect_failure_is_retried_without_latching_an_abort():
    calls = []
    master = FakeMaster()
    def factory(*args, **kwargs):
        calls.append((args, kwargs))
        if len(calls) == 1:
            raise OSError("temporarily unavailable")
        return master
    output = BlindMavlinkOutput(MavlinkSettings(), connection_factory=factory, report=lambda s: None)
    assert not output.publish(MotionCommand(forward=.23), (), 0)
    assert not output.publish(MotionCommand(forward=.23), (), .99)
    assert len(calls) == 1
    assert output.publish(MotionCommand(forward=.23), (), 1)
    assert len(calls) == 2
    assert not output.closed


@pytest.mark.parametrize("failure", ["motion", "receive"])
def test_send_and_receive_errors_reconnect_and_resend_the_current_pose(failure):
    first, second = FakeMaster(), FakeMaster()
    first.mav.fail_motion = failure == "motion"
    first.receive_error = failure == "receive"
    masters = iter([first, second])
    output = BlindMavlinkOutput(MavlinkSettings(), connection_factory=lambda *a, **k: next(masters), report=lambda s: None)
    pose = (ServoSetpoint(20, 1900), ServoSetpoint(21, 1800))
    assert not output.publish(MotionCommand.neutral(), pose, 0)
    assert first.closed
    assert output.publish(MotionCommand.neutral(), pose, 1)
    assert [p[4:6] for p in second.mav.servo] == [(20, 1900), (21, 1800)]


def test_manual_shutdown_neutralizes_then_releases_and_never_reconnects():
    output, master = adapter()
    output.publish(MotionCommand(forward=.23), (ServoSetpoint(20, 1100),), 0)
    assert output.shutdown() == []
    assert master.mav.motion[-3:] == [(1, 0, 0, 500, 0, 0)] * 3
    assert master.mav.release == [(1, 1, *([0] * 8 + [65534] * 10))]
    assert master.closed
    assert output.shutdown() == []
    assert not output.publish(MotionCommand(forward=.23), (), 1)
    assert len(master.mav.servo) == 1


def test_shutdown_still_releases_if_neutral_send_fails():
    output, master = adapter()
    output.publish(MotionCommand(forward=.23), (), 0)
    master.mav.fail_motion = True
    assert len(output.shutdown()) == 3
    assert master.mav.release
    assert master.closed


def test_shutdown_supports_mavlink1_eight_channel_signature():
    output, master = adapter()
    master.mav.only_eight_channels = True
    output.publish(MotionCommand(forward=.23), (), 0)
    assert output.shutdown() == []
    assert master.mav.release == [(1, 1, *([0] * 8))]
