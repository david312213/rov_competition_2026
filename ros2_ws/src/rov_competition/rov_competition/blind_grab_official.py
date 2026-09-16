"""持续盲抓到比赛官方 ROS2 话题及 TCP 转发节点的独立数据链。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any

from .blind_grab_config import MavlinkSettings, OfficialRosSettings
from .blind_grab_telemetry import BlindTelemetrySnapshot
from .domain import MotionCommand


@dataclass(frozen=True)
class OfficialRobotData:
    roll: float = 0.0
    yaw: float = 0.0
    cabin_hmi: float = 0.0
    pitch: float = 0.0
    longitude: float = 0.0
    latitude: float = 0.0
    depth: float = 0.0
    speed: float = 0.0
    cabin_temp: float = 0.0
    times: int = 0
    magnetic_field: float = 0.0
    accelerated_speed: float = 0.0
    cabin_pres: float = 0.0
    battery_vol: float = 0.0
    claw_cur: float = 0.0


def _fresh(value: Any, updated_at: float | None, now: float, stale_s: float) -> Any:
    if value is None or updated_at is None or now - updated_at > stale_s:
        return None
    return value


def _magnitude(vector: tuple[float, float, float] | None) -> float:
    if vector is None:
        return 0.0
    return math.sqrt(sum(value * value for value in vector))


def official_robot_data(
    snapshot: BlindTelemetrySnapshot,
    *,
    now: float,
    wall_time: float,
    stale_s: float,
) -> OfficialRobotData:
    """生成官方无有效位消息；不可用字段按该接口惯例发送 0。"""

    attitude_fresh = _fresh(True, snapshot.attitude_at, now, stale_s) is not None
    roll = snapshot.roll_rad if attitude_fresh else None
    pitch = snapshot.pitch_rad if attitude_fresh else None
    yaw = snapshot.yaw_rad if attitude_fresh else None
    position_fresh = _fresh(True, snapshot.position_at, now, stale_s) is not None
    acceleration = _fresh(snapshot.acceleration_m_s2, snapshot.acceleration_at, now, stale_s)
    magnetic = _fresh(snapshot.magnetic_field_t, snapshot.magnetic_field_at, now, stale_s)
    return OfficialRobotData(
        roll=math.degrees(roll) if roll is not None else 0.0,
        yaw=math.degrees(yaw) if yaw is not None else 0.0,
        pitch=math.degrees(pitch) if pitch is not None else 0.0,
        longitude=snapshot.longitude_deg if position_fresh and snapshot.longitude_deg is not None else 0.0,
        latitude=snapshot.latitude_deg if position_fresh and snapshot.latitude_deg is not None else 0.0,
        depth=float(_fresh(snapshot.depth_m, snapshot.depth_at, now, stale_s) or 0.0),
        speed=float(_fresh(snapshot.speed_m_s, snapshot.speed_at, now, stale_s) or 0.0),
        times=int(wall_time),
        magnetic_field=_magnitude(magnetic),
        accelerated_speed=_magnitude(acceleration),
        battery_vol=float(
            _fresh(snapshot.battery_voltage_v, snapshot.battery_at, now, stale_s) or 0.0
        ),
    )


def applied_motion(
    motion: MotionCommand,
    directions: tuple[int, int, int, int],
) -> tuple[float, float, float, float]:
    """返回与实际 MANUAL_CONTROL 方向一致的 x/y/z/yaw 指令。"""

    return tuple(value * direction for value, direction in zip(
        (motion.forward, motion.lateral, motion.vertical, motion.yaw), directions,
    ))  # type: ignore[return-value]


def command_acceleration(
    current: tuple[float, float, float, float],
    previous: tuple[float, float, float, float] | None,
    duration_s: float,
) -> tuple[float, float, float, float]:
    if previous is None or duration_s <= 0.0 or not math.isfinite(duration_s):
        return (0.0, 0.0, 0.0, 0.0)
    return tuple((value - old) / duration_s for value, old in zip(current, previous))  # type: ignore[return-value]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def forwarder_command(settings: OfficialRosSettings) -> list[str]:
    """使用官方包的原可执行程序和参数名，不经过旧控制网关。"""

    return [
        "ros2",
        "run",
        "ros2_topic_forwarding",
        "topic_forwarding",
        "--ros-args",
        "--log-level",
        "warn",
        "-p",
        f"server_ip:={settings.server_ip}",
        "-p",
        f"server_port:={settings.server_port}",
    ]


class OfficialRosPublisher:
    """发布官方话题；ROS 初始化或发布失败后独立重试。"""

    def __init__(
        self,
        settings: OfficialRosSettings,
        mavlink: MavlinkSettings,
        telemetry_source: Callable[[], BlindTelemetrySnapshot],
        *,
        report: Callable[[str], None] = print,
    ) -> None:
        self.settings = settings
        self.mavlink = mavlink
        self.telemetry_source = telemetry_source
        self.report = report
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name="blind-grab-official-ros", daemon=True,
        )
        self._lock = threading.Lock()
        self._motion = MotionCommand.neutral()
        self.ready = False
        self.last_error = ""
        self._started = False
        self._next_telemetry_report_at = 0.0

    def set_desired(self, motion: MotionCommand) -> None:
        with self._lock:
            self._motion = motion

    def _desired(self) -> MotionCommand:
        with self._lock:
            return self._motion

    def start(self) -> None:
        if self.settings.enabled and not self._started:
            self.thread.start()
            self._started = True

    @staticmethod
    def _fill_twist(message: Any, values: tuple[float, float, float, float]) -> None:
        message.linear.x, message.linear.y, message.linear.z = values[:3]
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = values[3]

    @staticmethod
    def _fill_accel(message: Any, values: tuple[float, float, float, float]) -> None:
        message.linear.x, message.linear.y, message.linear.z = values[:3]
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = values[3]

    def _publish_sensor_topics(
        self,
        node: Any,
        publishers: dict[str, Any],
        types: dict[str, Any],
        snapshot: BlindTelemetrySnapshot,
        now: float,
    ) -> None:
        stale = self.settings.telemetry_stale_s
        stamp = node.get_clock().now().to_msg()
        attitude = _fresh(True, snapshot.attitude_at, now, stale)
        angular = _fresh(snapshot.angular_velocity_rad_s, snapshot.angular_velocity_at, now, stale)
        acceleration = _fresh(snapshot.acceleration_m_s2, snapshot.acceleration_at, now, stale)
        if attitude is not None or angular is not None or acceleration is not None:
            message = types["Imu"]()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            if attitude is not None:
                x, y, z, w = euler_to_quaternion(
                    snapshot.roll_rad or 0.0,
                    snapshot.pitch_rad or 0.0,
                    snapshot.yaw_rad or 0.0,
                )
                message.orientation.x = x
                message.orientation.y = y
                message.orientation.z = z
                message.orientation.w = w
            else:
                message.orientation.w = 1.0
                message.orientation_covariance[0] = -1.0
            if angular is not None:
                message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = angular
            else:
                message.angular_velocity_covariance[0] = -1.0
            if acceleration is not None:
                message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = acceleration
            else:
                message.linear_acceleration_covariance[0] = -1.0
            publishers["imu"].publish(message)

        magnetic = _fresh(snapshot.magnetic_field_t, snapshot.magnetic_field_at, now, stale)
        if magnetic is not None:
            message = types["MagneticField"]()
            message.header.stamp = stamp
            message.header.frame_id = "base_link"
            message.magnetic_field.x, message.magnetic_field.y, message.magnetic_field.z = magnetic
            publishers["magnetometer"].publish(message)

        pressure = _fresh(snapshot.pressure_pa, snapshot.pressure_at, now, stale)
        if pressure is not None:
            message = types["FluidPressure"]()
            message.header.stamp = stamp
            message.header.frame_id = "pressure_link"
            message.fluid_pressure = pressure
            message.variance = 0.0
            publishers["pressure"].publish(message)

    def _publish_robot_data(
        self,
        publisher: Any,
        message_type: Any,
        snapshot: BlindTelemetrySnapshot,
        now: float,
    ) -> None:
        values = official_robot_data(
            snapshot,
            now=now,
            wall_time=time.time(),
            stale_s=self.settings.telemetry_stale_s,
        )
        message = message_type()
        for name, value in vars(values).items():
            setattr(message, name, value)
        publisher.publish(message)

    def _run_context(self) -> None:
        import rclpy
        from geometry_msgs.msg import Accel, Twist
        from rclpy.context import Context
        from rclpy.signals import SignalHandlerOptions
        from ros2_topic_forwarding.msg import RobotDataMessage
        from sensor_msgs.msg import FluidPressure, Imu, MagneticField

        context = Context()
        node = None
        try:
            rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
            from rclpy.node import Node
            node = Node("rov_blind_grab_official_data", context=context)
            publishers = {
                "cmd_vel": node.create_publisher(Twist, "/cmd_vel", 10),
                "cmd_accel": node.create_publisher(Accel, "/cmd_accel", 10),
                "robot_data": node.create_publisher(RobotDataMessage, "/robot_data", 10),
                "imu": node.create_publisher(Imu, "/imu", 10),
                "magnetometer": node.create_publisher(MagneticField, "/magnetometer", 10),
                "pressure": node.create_publisher(FluidPressure, "/pressure", 10),
            }
            types = {
                "Twist": Twist,
                "Accel": Accel,
                "RobotDataMessage": RobotDataMessage,
                "Imu": Imu,
                "MagneticField": MagneticField,
                "FluidPressure": FluidPressure,
            }
            self.ready = True
            self.last_error = ""
            self.report(
                "[官方ROS] 已发布 /cmd_vel /cmd_accel /robot_data /imu "
                "/magnetometer /pressure"
            )
            command_period = 1.0 / self.settings.command_rate_hz
            robot_period = 1.0 / self.settings.robot_data_rate_hz
            previous: tuple[float, float, float, float] | None = None
            previous_at: float | None = None
            next_robot_at = 0.0
            while not self.stop_event.is_set() and context.ok():
                started = time.monotonic()
                current = applied_motion(self._desired(), self.mavlink.directions)
                acceleration = command_acceleration(
                    current,
                    previous,
                    0.0 if previous_at is None else started - previous_at,
                )
                twist = Twist()
                accel = Accel()
                self._fill_twist(twist, current)
                self._fill_accel(accel, acceleration)
                publishers["cmd_vel"].publish(twist)
                publishers["cmd_accel"].publish(accel)
                previous, previous_at = current, started
                if started >= next_robot_at:
                    try:
                        snapshot = self.telemetry_source()
                    except Exception as exc:
                        snapshot = BlindTelemetrySnapshot()
                        if started >= self._next_telemetry_report_at:
                            self._next_telemetry_report_at = started + 5.0
                            self.report(
                                f"[官方ROS] 遥测快照读取失败: {exc}；继续发布运动和空值状态"
                            )
                    self._publish_robot_data(
                        publishers["robot_data"], RobotDataMessage, snapshot, started,
                    )
                    self._publish_sensor_topics(node, publishers, types, snapshot, started)
                    next_robot_at = started + robot_period
                self.stop_event.wait(max(0.0, command_period - (time.monotonic() - started)))

            # 平台端最后再看到几帧归中；此处只报告，不参与飞控控制。
            for _ in range(3):
                twist = Twist()
                accel = Accel()
                publishers["cmd_vel"].publish(twist)
                publishers["cmd_accel"].publish(accel)
                time.sleep(0.02)
        finally:
            self.ready = False
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            try:
                context.try_shutdown()
            except Exception:
                pass

    def _run(self) -> None:
        next_report_at = 0.0
        while not self.stop_event.is_set():
            try:
                self._run_context()
                if not self.stop_event.is_set():
                    raise RuntimeError("ROS上下文已关闭")
            except Exception as exc:
                self.ready = False
                self.last_error = f"{type(exc).__name__}: {exc}"
                now = time.monotonic()
                if now >= next_report_at:
                    next_report_at = now + 5.0
                    try:
                        self.report(f"[官方ROS] {self.last_error}；盲抓继续，稍后重试")
                    except Exception:
                        pass
            self.stop_event.wait(self.settings.forwarder_restart_s)

    def shutdown(self) -> None:
        if not self.settings.enabled or not self._started:
            return
        self.set_desired(MotionCommand.neutral())
        self.stop_event.set()
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            self.report("[官方ROS] 发布线程尚未返回；不阻塞盲抓主进程关闭")


class OfficialForwarderSupervisor:
    """监督官方 C++ TCP 转发节点；退出或启动失败只触发重试。"""

    def __init__(
        self,
        settings: OfficialRosSettings,
        log_path: Path,
        *,
        popen: Callable[..., Any] = subprocess.Popen,
        report: Callable[[str], None] = print,
    ) -> None:
        self.settings = settings
        self.log_path = log_path
        self.popen = popen
        self.report = report
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name="blind-grab-official-forwarder", daemon=True,
        )
        self.process: Any = None
        self.attempts = 0
        self.last_error = ""
        self._started = False

    def start(self) -> None:
        if self.settings.enabled and not self._started:
            self.thread.start()
            self._started = True

    @staticmethod
    def _stop_process(process: Any) -> None:
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
        except Exception:
            try:
                process.send_signal(signal.SIGINT)
            except Exception:
                pass
        try:
            process.wait(timeout=2.0)
            return
        except Exception:
            pass
        try:
            process.terminate()
            process.wait(timeout=1.0)
            return
        except Exception:
            pass
        try:
            process.kill()
        except Exception:
            pass

    def _run(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        next_report_at = 0.0
        with self.log_path.open("ab", buffering=0) as log:
            while not self.stop_event.is_set():
                try:
                    self.attempts += 1
                    self.process = self.popen(
                        forwarder_command(self.settings),
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    self.last_error = ""
                    self.report(
                        f"[官方ROS] TCP转发节点已启动 -> "
                        f"{self.settings.server_ip}:{self.settings.server_port}"
                    )
                    while not self.stop_event.wait(0.5):
                        code = self.process.poll()
                        if code is not None:
                            raise RuntimeError(f"官方转发节点退出，code={code}")
                except Exception as exc:
                    self.last_error = f"{type(exc).__name__}: {exc}"
                    now = time.monotonic()
                    if now >= next_report_at:
                        next_report_at = now + 5.0
                        try:
                            self.report(
                                f"[官方ROS] {self.last_error}；盲抓继续，转发节点稍后重启"
                            )
                        except Exception:
                            pass
                finally:
                    self._stop_process(self.process)
                    self.process = None
                self.stop_event.wait(self.settings.forwarder_restart_s)

    def shutdown(self) -> None:
        if not self.settings.enabled or not self._started:
            return
        self.stop_event.set()
        self._stop_process(self.process)
        self.thread.join(timeout=3.0)
        if self.thread.is_alive():
            self.report("[官方ROS] 转发监督线程尚未返回；不阻塞主进程关闭")


class OfficialDataService:
    """统一管理官方话题发布和官方 TCP 转发，但不拥有盲抓控制生命周期。"""

    def __init__(
        self,
        settings: OfficialRosSettings,
        mavlink: MavlinkSettings,
        telemetry_source: Callable[[], BlindTelemetrySnapshot],
        session_directory: Path,
        *,
        report: Callable[[str], None] = print,
    ) -> None:
        self.settings = settings
        self.report = report
        self.publisher = OfficialRosPublisher(
            settings, mavlink, telemetry_source, report=report,
        )
        self.forwarder = OfficialForwarderSupervisor(
            settings,
            session_directory / "official_ros_forwarder.log",
            report=report,
        )

    def start(self) -> None:
        if not self.settings.enabled:
            return
        for name, component in (
            ("TCP转发监督", self.forwarder),
            ("话题发布", self.publisher),
        ):
            try:
                component.start()
            except Exception as exc:
                self.report(f"[官方ROS] {name}启动失败: {exc}；盲抓继续")

    def set_desired(self, motion: MotionCommand, servos: tuple[Any, ...] = ()) -> None:
        del servos
        self.publisher.set_desired(motion)

    def shutdown(self) -> None:
        if not self.settings.enabled:
            return
        # 先让官方平台接收归中，再关闭TCP转发子进程。
        for name, component in (
            ("话题发布", self.publisher),
            ("TCP转发监督", self.forwarder),
        ):
            try:
                component.shutdown()
            except Exception as exc:
                self.report(f"[官方ROS] {name}关闭异常: {exc}；继续清理其他支路")
