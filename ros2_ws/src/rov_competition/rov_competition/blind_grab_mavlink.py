"""盲抓的独立 MAVLink 输出，不调用 MavlinkVehicle 或 ROS 控制网关。

发送 MANUAL_CONTROL 和 MAV_CMD_DO_SET_SERVO。只处理通信重试，不检查
视觉、遥测、模式、深度、解锁许可或 ACK；也不发送解锁、模式或参数修改。
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .blind_grab import ServoSetpoint
from .blind_grab_config import MavlinkSettings
from .domain import MotionCommand


MAV_CMD_DO_SET_SERVO = 183


class BlindMavlinkOutput:
    """单个 I/O 线程拥有该对象和连接；控制线程只交付最新动作目标。"""

    def __init__(
        self,
        config: MavlinkSettings,
        *,
        connection_factory: Callable[..., Any] | None = None,
        report: Callable[[str], None] = print,
    ) -> None:
        self.config = config
        self._factory = connection_factory
        self._report = report
        self._master: Any = None
        self._next_connection_at = 0.0
        self._next_heartbeat_at = 0.0
        self._next_servo_at = 0.0
        self._last_servos: tuple[ServoSetpoint, ...] | None = None
        self._next_error_report_at = 0.0
        self.last_error = ""
        self.closed = False

    @property
    def connected(self) -> bool:
        return self._master is not None

    def _connect(self, now: float) -> bool:
        if self.closed or now < self._next_connection_at:
            return False
        try:
            if self._factory is None:
                from pymavlink import mavutil
                self._factory = mavutil.mavlink_connection
            self._master = self._factory(
                self.config.connection_uri,
                baud=self.config.baud,
                source_system=self.config.source_system,
                source_component=self.config.source_component,
                autoreconnect=False,
            )
            self._next_heartbeat_at = self._next_servo_at = 0.0
            self._last_servos = None
            self.last_error = ""
            return True
        except Exception as exc:
            self._communication_error(exc, now)
            return False

    def _report_error(self, message: str, now: float) -> None:
        self.last_error = message
        if now >= self._next_error_report_at:
            self._next_error_report_at = now + 2.0
            try:
                self._report(f"[盲抓通信] {message}；继续循环并重试")
            except Exception:
                pass

    def _communication_error(self, exc: Exception, now: float) -> None:
        self._report_error(f"{type(exc).__name__}: {exc}", now)
        master, self._master = self._master, None
        self._next_connection_at = now + self.config.reconnect_interval_s
        if master is not None:
            try:
                master.close()
            except Exception:
                pass

    def _send_motion(self, motion: MotionCommand) -> None:
        forward, lateral, vertical, yaw = self.config.directions
        self._master.mav.manual_control_send(
            self.config.target_system,
            round(motion.forward * forward * 1000),
            round(motion.lateral * lateral * 1000),
            round(500 + motion.vertical * vertical * 500),
            round(motion.yaw * yaw * 1000),
            0,
        )

    def publish(
        self, motion: MotionCommand, servos: tuple[ServoSetpoint, ...], now: float,
    ) -> bool:
        if self.closed:
            return False
        if self._master is None and not self._connect(now):
            return False
        try:
            # udpin 通过接收报文获知回包端点。读取有限条消息，不等待心跳/ACK，
            # 不以消息内容或年龄决定是否发送动作。
            for _ in range(32):
                if self._master.recv_match(blocking=False) is None:
                    break
            if now >= self._next_heartbeat_at:
                self._master.mav.heartbeat_send(6, 8, 0, 0, 4)
                self._next_heartbeat_at = now + 1.0
            self._send_motion(motion)
            if servos != self._last_servos or now >= self._next_servo_at:
                for target in servos:
                    self._master.mav.command_long_send(
                        self.config.target_system, self.config.target_component,
                        MAV_CMD_DO_SET_SERVO, 0, target.output_channel, target.pwm,
                        0, 0, 0, 0, 0,
                    )
                self._last_servos = servos
                self._next_servo_at = now + self.config.servo_refresh_s
            self.last_error = ""
            return True
        except Exception as exc:
            self._communication_error(exc, now)
            return False

    def shutdown(self) -> list[str]:
        """只在人工关闭时调用；不等待 ACK，不追加开爪、转臂或解锁动作。"""
        if self.closed:
            return []
        self.closed = True
        errors: list[str] = []
        if self._master is None:
            return errors
        # 即使一类报文发送失败，仍分别尝试归中、释放和关闭连接。
        for _ in range(3):
            try:
                self._send_motion(MotionCommand.neutral())
            except Exception as exc:
                errors.append(f"归中: {exc}")
        try:
            # MAVLink: 通道1..8以0释放，扩展通道9..18以65534释放。
            try:
                self._master.mav.rc_channels_override_send(
                    self.config.target_system, self.config.target_component,
                    *([0] * 8 + [65534] * 10),
                )
            except TypeError:
                # pymavlink 的 MAVLink1 方言只提供前八个通道参数。
                self._master.mav.rc_channels_override_send(
                    self.config.target_system, self.config.target_component,
                    *([0] * 8),
                )
        except Exception as exc:
            errors.append(f"释放控制: {exc}")
        try:
            self._master.close()
        except Exception as exc:
            errors.append(f"关闭连接: {exc}")
        self._master = None
        return errors
