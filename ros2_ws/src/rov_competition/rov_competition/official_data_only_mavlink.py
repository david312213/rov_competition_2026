"""只读 MAVLink 遥测入口，不向飞控发送任何报文。"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from .blind_grab_config import MavlinkSettings, OfficialRosSettings
from .blind_grab_telemetry import BlindTelemetryCollector, BlindTelemetrySnapshot


class PassiveMavlinkTelemetry:
    """轮询独立遥测端点；连接、运行和关闭阶段都不写 MAVLink。"""

    def __init__(
        self,
        config: MavlinkSettings,
        official: OfficialRosSettings,
        *,
        connection_factory: Callable[..., Any] | None = None,
        report: Callable[[str], None] = print,
    ) -> None:
        self.config = config
        self._factory = connection_factory
        self._report = report
        self._master: Any = None
        self._next_connection_at = 0.0
        self._next_error_report_at = 0.0
        self.last_error = ""
        self.closed = False
        self.telemetry = BlindTelemetryCollector(
            target_system=config.target_system,
            target_component=config.target_component,
            depth_message=official.depth_message,
            depth_field=official.depth_field,
            depth_multiplier=official.depth_multiplier,
            depth_offset_m=official.depth_offset_m,
        )

    @property
    def connected(self) -> bool:
        return self._master is not None

    def telemetry_snapshot(self) -> BlindTelemetrySnapshot:
        return self.telemetry.snapshot()

    def _report_error(self, message: str, now: float) -> None:
        self.last_error = message
        if now >= self._next_error_report_at:
            self._next_error_report_at = now + 2.0
            try:
                self._report(f"[只读MAVLink遥测] {message}；稍后重试")
            except Exception:
                pass

    def _close_connection(self) -> list[str]:
        master, self._master = self._master, None
        if master is None:
            return []
        try:
            master.close()
        except Exception as exc:
            return [f"关闭连接: {exc}"]
        return []

    def _communication_error(self, exc: Exception, now: float) -> None:
        self._report_error(f"{type(exc).__name__}: {exc}", now)
        self._close_connection()
        self._next_connection_at = now + self.config.reconnect_interval_s

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
            self.last_error = ""
            return True
        except Exception as exc:
            self._communication_error(exc, now)
            return False

    def poll(self, now: float, *, max_messages: int = 64) -> int:
        """读取当前已到达的消息；不等待消息，也不发送心跳或请求。"""

        if self.closed:
            return 0
        if self._master is None and not self._connect(now):
            return 0
        received = 0
        try:
            for _ in range(max_messages):
                message = self._master.recv_match(blocking=False)
                if message is None:
                    break
                received += 1
                try:
                    self.telemetry.update(message, now)
                except Exception as exc:
                    self._report_error(f"忽略无法解析的消息: {exc}", now)
            self.last_error = ""
            return received
        except Exception as exc:
            self._communication_error(exc, now)
            return 0

    def shutdown(self) -> list[str]:
        """人工关闭时只关闭本地连接；不发送归中、释放、模式或解锁报文。"""

        if self.closed:
            return []
        self.closed = True
        return self._close_connection()
