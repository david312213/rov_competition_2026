import math
from types import SimpleNamespace

import pytest

from rov_competition.blind_grab_telemetry import BlindTelemetryCollector


class Message(SimpleNamespace):
    def __init__(self, message_type, *, source_system=1, source_component=1, **fields):
        super().__init__(**fields)
        self.message_type = message_type
        self.source_system = source_system
        self.source_component = source_component

    def get_type(self):
        return self.message_type

    def get_srcSystem(self):
        return self.source_system

    def get_srcComponent(self):
        return self.source_component


def test_collector_extracts_real_attitude_depth_battery_position_and_speed():
    collector = BlindTelemetryCollector()
    assert collector.update(
        Message(
            "ATTITUDE",
            roll=math.radians(10),
            pitch=math.radians(-5),
            yaw=math.radians(90),
            rollspeed=.1,
            pitchspeed=.2,
            yawspeed=.3,
        ),
        1.0,
    )
    assert collector.update(Message("AHRS2", altitude=-2.4), 1.1)
    assert collector.update(Message("SYS_STATUS", voltage_battery=16800), 1.2)
    assert collector.update(
        Message("GLOBAL_POSITION_INT", lat=389123456, lon=1216123456, vx=30, vy=40, vz=0),
        1.3,
    )
    snapshot = collector.snapshot()
    assert math.degrees(snapshot.roll_rad) == pytest.approx(10)
    assert math.degrees(snapshot.pitch_rad) == pytest.approx(-5)
    assert math.degrees(snapshot.yaw_rad) == pytest.approx(90)
    assert snapshot.angular_velocity_rad_s == pytest.approx((.1, .2, .3))
    assert snapshot.depth_m == pytest.approx(2.4)
    assert snapshot.battery_voltage_v == pytest.approx(16.8)
    assert snapshot.latitude_deg == pytest.approx(38.9123456)
    assert snapshot.longitude_deg == pytest.approx(121.6123456)
    assert snapshot.speed_m_s == pytest.approx(.5)


def test_collector_converts_imu_magnetic_and_pressure_units_for_ros_messages():
    collector = BlindTelemetryCollector()
    assert collector.update(
        Message(
            "SCALED_IMU",
            xacc=1000,
            yacc=0,
            zacc=-1000,
            xgyro=100,
            ygyro=-200,
            zgyro=300,
            xmag=500,
            ymag=-250,
            zmag=100,
        ),
        2.0,
    )
    assert collector.update(Message("SCALED_PRESSURE", press_abs=1013.25), 2.1)
    snapshot = collector.snapshot()
    assert snapshot.acceleration_m_s2 == pytest.approx((9.80665, 0, -9.80665))
    assert snapshot.angular_velocity_rad_s == pytest.approx((.1, -.2, .3))
    assert snapshot.magnetic_field_t == pytest.approx((5e-5, -2.5e-5, 1e-5))
    assert snapshot.pressure_pa == pytest.approx(101325)


def test_collector_ignores_other_vehicle_and_malformed_messages_without_erasing_data():
    collector = BlindTelemetryCollector(target_system=1, target_component=1)
    collector.update(Message("SYS_STATUS", voltage_battery=12000), 1.0)
    assert not collector.update(
        Message("SYS_STATUS", source_system=2, voltage_battery=24000), 2.0,
    )
    assert not collector.update(object(), 3.0)
    assert collector.snapshot().battery_voltage_v == 12.0
    assert collector.snapshot().last_message_at == 1.0
