"""在持续发送盲抓命令时旁路收集首次判底和官方ROS遥测。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .blind_grab import ServoSetpoint
from .blind_grab_config import MavlinkSettings, OfficialRosSettings
from .blind_grab_mavlink import BlindMavlinkOutput, MAV_CMD_DO_SET_SERVO
from .blind_grab_telemetry import BlindTelemetryCollector, BlindTelemetrySnapshot
from .domain import MotionCommand


class OfficialTelemetryMavlinkOutput(BlindMavlinkOutput):
    """保持盲抓发送语义，并解析已经到达的MAVLink深度及展示遥测。"""

    def __init__(
        self,
        config: MavlinkSettings,
        official: OfficialRosSettings,
        *,
        connection_factory: Callable[..., Any] | None = None,
        report: Callable[[str], None] = print,
    ) -> None:
        super().__init__(config, connection_factory=connection_factory, report=report)
        self.telemetry = BlindTelemetryCollector(
            target_system=config.target_system,
            target_component=config.target_component,
            depth_message=official.depth_message,
            depth_field=official.depth_field,
            depth_multiplier=official.depth_multiplier,
            depth_offset_m=official.depth_offset_m,
        )
        self._next_telemetry_error_report_at = 0.0

    def telemetry_snapshot(self) -> BlindTelemetrySnapshot:
        return self.telemetry.snapshot()

    def _collect(self, message: Any, now: float) -> None:
        try:
            self.telemetry.update(message, now)
        except Exception as exc:
            # 遥测仅供平台展示；单条坏消息不能让控制链路重连或停机。
            if now >= self._next_telemetry_error_report_at:
                self._next_telemetry_error_report_at = now + 5.0
                try:
                    self._report(f"[官方ROS遥测] 忽略无法解析的MAVLink消息: {exc}")
                except Exception:
                    pass

    def publish(
        self,
        motion: MotionCommand,
        servos: tuple[ServoSetpoint, ...],
        now: float,
    ) -> bool:
        if self.closed:
            return False
        if self._master is None and not self._connect(now):
            return False
        try:
            for _ in range(32):
                message = self._master.recv_match(blocking=False)
                if message is None:
                    break
                self._collect(message, now)
            if now >= self._next_heartbeat_at:
                self._master.mav.heartbeat_send(6, 8, 0, 0, 4)
                self._next_heartbeat_at = now + 1.0
            self._send_motion(motion)
            if servos != self._last_servos or now >= self._next_servo_at:
                for target in servos:
                    self._master.mav.command_long_send(
                        self.config.target_system,
                        self.config.target_component,
                        MAV_CMD_DO_SET_SERVO,
                        0,
                        target.output_channel,
                        target.pwm,
                        0,
                        0,
                        0,
                        0,
                        0,
                    )
                self._last_servos = servos
                self._next_servo_at = now + self.config.servo_refresh_s
            self.last_error = ""
            return True
        except Exception as exc:
            self._communication_error(exc, now)
            return False
