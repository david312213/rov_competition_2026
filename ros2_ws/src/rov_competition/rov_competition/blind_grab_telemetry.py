"""从盲抓专用 MAVLink 链路旁路提取官方展示所需的真实遥测。"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
from typing import Any


STANDARD_GRAVITY_M_S2 = 9.80665
GAUSS_TO_TESLA = 1.0e-4
MILLIGAUSS_TO_TESLA = 1.0e-7


@dataclass(frozen=True)
class BlindTelemetrySnapshot:
    """只保存实际收到的值；``None`` 表示从未获得该字段。"""

    last_message_at: float | None = None
    roll_rad: float | None = None
    pitch_rad: float | None = None
    yaw_rad: float | None = None
    attitude_at: float | None = None
    angular_velocity_rad_s: tuple[float, float, float] | None = None
    angular_velocity_at: float | None = None
    acceleration_m_s2: tuple[float, float, float] | None = None
    acceleration_at: float | None = None
    magnetic_field_t: tuple[float, float, float] | None = None
    magnetic_field_at: float | None = None
    pressure_pa: float | None = None
    pressure_at: float | None = None
    depth_m: float | None = None
    depth_at: float | None = None
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    position_at: float | None = None
    speed_m_s: float | None = None
    speed_at: float | None = None
    battery_voltage_v: float | None = None
    battery_at: float | None = None


class BlindTelemetryCollector:
    """解析常见ArduSub消息；深度只供首次判底，其余字段供官方展示。"""

    def __init__(
        self,
        *,
        target_system: int = 1,
        target_component: int = 1,
        depth_message: str = "AHRS2",
        depth_field: str = "altitude",
        depth_multiplier: float = -1.0,
        depth_offset_m: float = 0.0,
    ) -> None:
        self.target_system = int(target_system)
        self.target_component = int(target_component)
        self.depth_message = str(depth_message).upper()
        self.depth_field = str(depth_field)
        self.depth_multiplier = float(depth_multiplier)
        self.depth_offset_m = float(depth_offset_m)
        self._lock = threading.Lock()
        self._snapshot = BlindTelemetrySnapshot()

    @staticmethod
    def _source(message: Any, method: str) -> int | None:
        getter = getattr(message, method, None)
        try:
            return int(getter()) if callable(getter) else None
        except (TypeError, ValueError):
            return None

    def _from_target(self, message: Any) -> bool:
        system = self._source(message, "get_srcSystem")
        component = self._source(message, "get_srcComponent")
        return (
            (system is None or system == self.target_system)
            and (
                component is None
                or self.target_component == 0
                or component == self.target_component
            )
        )

    @staticmethod
    def _number(message: Any, field: str, multiplier: float = 1.0) -> float | None:
        try:
            value = float(getattr(message, field)) * multiplier
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        return value if math.isfinite(value) else None

    @staticmethod
    def _vector(
        message: Any,
        fields: tuple[str, str, str],
        multiplier: float = 1.0,
    ) -> tuple[float, float, float] | None:
        values = tuple(BlindTelemetryCollector._number(message, field, multiplier) for field in fields)
        if any(value is None for value in values):
            return None
        return values  # type: ignore[return-value]

    @staticmethod
    def _quaternion_to_euler(
        w: float, x: float, y: float, z: float,
    ) -> tuple[float, float, float]:
        sin_roll_cos_pitch = 2.0 * (w * x + y * z)
        cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sin_pitch)
        sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
        cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
        return roll, pitch, yaw

    def update(self, message: Any, now: float) -> bool:
        """解析一条消息。未知、损坏或其他载具的消息直接忽略。"""

        getter = getattr(message, "get_type", None)
        if not callable(getter) or not self._from_target(message):
            return False
        try:
            message_type = str(getter()).upper()
        except Exception:
            return False
        if not message_type or message_type == "BAD_DATA":
            return False

        changes: dict[str, Any] = {"last_message_at": float(now)}

        if message_type == self.depth_message:
            raw_depth = self._number(message, self.depth_field)
            if raw_depth is not None:
                depth = raw_depth * self.depth_multiplier + self.depth_offset_m
                if math.isfinite(depth):
                    changes.update(depth_m=depth, depth_at=float(now))

        if message_type == "ATTITUDE":
            attitude = self._vector(message, ("roll", "pitch", "yaw"))
            if attitude is not None:
                changes.update(
                    roll_rad=attitude[0], pitch_rad=attitude[1], yaw_rad=attitude[2],
                    attitude_at=float(now),
                )
            angular = self._vector(message, ("rollspeed", "pitchspeed", "yawspeed"))
            if angular is not None:
                changes.update(angular_velocity_rad_s=angular, angular_velocity_at=float(now))
        elif message_type == "ATTITUDE_QUATERNION":
            quaternion = tuple(self._number(message, field) for field in ("q1", "q2", "q3", "q4"))
            if all(value is not None for value in quaternion):
                roll, pitch, yaw = self._quaternion_to_euler(*quaternion)  # type: ignore[arg-type]
                changes.update(
                    roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
                    attitude_at=float(now),
                )
            angular = self._vector(message, ("rollspeed", "pitchspeed", "yawspeed"))
            if angular is not None:
                changes.update(angular_velocity_rad_s=angular, angular_velocity_at=float(now))
        elif message_type == "HIGHRES_IMU":
            acceleration = self._vector(message, ("xacc", "yacc", "zacc"))
            angular = self._vector(message, ("xgyro", "ygyro", "zgyro"))
            magnetic = self._vector(message, ("xmag", "ymag", "zmag"), GAUSS_TO_TESLA)
            pressure = self._number(message, "abs_pressure", 100.0)
            if acceleration is not None:
                changes.update(acceleration_m_s2=acceleration, acceleration_at=float(now))
            if angular is not None:
                changes.update(angular_velocity_rad_s=angular, angular_velocity_at=float(now))
            if magnetic is not None:
                changes.update(magnetic_field_t=magnetic, magnetic_field_at=float(now))
            if pressure is not None and pressure > 0.0:
                changes.update(pressure_pa=pressure, pressure_at=float(now))
        elif message_type == "RAW_IMU" or message_type.startswith("SCALED_IMU"):
            acceleration = self._vector(
                message, ("xacc", "yacc", "zacc"), STANDARD_GRAVITY_M_S2 / 1000.0,
            )
            angular = self._vector(message, ("xgyro", "ygyro", "zgyro"), 1.0 / 1000.0)
            magnetic = self._vector(message, ("xmag", "ymag", "zmag"), MILLIGAUSS_TO_TESLA)
            if acceleration is not None:
                changes.update(acceleration_m_s2=acceleration, acceleration_at=float(now))
            if angular is not None:
                changes.update(angular_velocity_rad_s=angular, angular_velocity_at=float(now))
            if magnetic is not None:
                changes.update(magnetic_field_t=magnetic, magnetic_field_at=float(now))
        elif message_type.startswith("SCALED_PRESSURE"):
            pressure = self._number(message, "press_abs", 100.0)
            if pressure is not None and pressure > 0.0:
                changes.update(pressure_pa=pressure, pressure_at=float(now))
        elif message_type == "SYS_STATUS":
            voltage_mv = self._number(message, "voltage_battery")
            if voltage_mv is not None and 0.0 <= voltage_mv < 65535.0:
                changes.update(battery_voltage_v=voltage_mv / 1000.0, battery_at=float(now))
        elif message_type == "BATTERY_STATUS":
            values = getattr(message, "voltages", ())
            try:
                cells = [float(value) for value in values if 0.0 < float(value) < 65535.0]
            except (TypeError, ValueError, OverflowError):
                cells = []
            if cells:
                changes.update(battery_voltage_v=sum(cells) / 1000.0, battery_at=float(now))
        elif message_type == "GLOBAL_POSITION_INT":
            latitude = self._number(message, "lat", 1.0e-7)
            longitude = self._number(message, "lon", 1.0e-7)
            if latitude is not None and longitude is not None:
                changes.update(
                    latitude_deg=latitude, longitude_deg=longitude, position_at=float(now),
                )
            velocity = self._vector(message, ("vx", "vy", "vz"), 0.01)
            if velocity is not None:
                changes.update(speed_m_s=math.sqrt(sum(value * value for value in velocity)), speed_at=float(now))
        elif message_type == "GPS_RAW_INT":
            fix_type = int(self._number(message, "fix_type") or 0)
            latitude = self._number(message, "lat", 1.0e-7)
            longitude = self._number(message, "lon", 1.0e-7)
            if fix_type >= 2 and latitude is not None and longitude is not None:
                changes.update(
                    latitude_deg=latitude, longitude_deg=longitude, position_at=float(now),
                )
            velocity_cm_s = self._number(message, "vel")
            if velocity_cm_s is not None and 0.0 <= velocity_cm_s < 65535.0:
                changes.update(speed_m_s=velocity_cm_s * 0.01, speed_at=float(now))
        elif message_type == "LOCAL_POSITION_NED":
            velocity = self._vector(message, ("vx", "vy", "vz"))
            if velocity is not None:
                changes.update(speed_m_s=math.sqrt(sum(value * value for value in velocity)), speed_at=float(now))
        elif message_type == "VFR_HUD":
            speed = self._number(message, "groundspeed")
            if speed is not None and speed >= 0.0:
                changes.update(speed_m_s=speed, speed_at=float(now))

        with self._lock:
            self._snapshot = replace(self._snapshot, **changes)
        return True

    def snapshot(self) -> BlindTelemetrySnapshot:
        with self._lock:
            return self._snapshot
