"""单一 MAVLink 飞控适配器、双控制后端和实艇安全检查。

Python 只发送前后、横移、升沉和偏航意图。八个推进器的混控、姿态控制和物理
输出均由 ArduSub 负责；本模块不会实现或绕过飞控的八电机分配。
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

from .config import ControlProtocol, RobotConfig
from .domain import GripperAction, MotionCommand, TelemetrySnapshot
from .preflight import (
    MAV_AUTOPILOT_ARDUPILOTMEGA,
    MAV_TYPE_SUBMARINE,
    PreflightReport,
    build_preflight_report,
)

# MAVLink common.xml 的稳定编号。使用常量可让假飞控测试不依赖本机安装 pymavlink。
MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_REQUEST_MESSAGE = 512
MAVLINK_MSG_ID_AUTOPILOT_VERSION = 148
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_IN_PROGRESS = 5
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_STATE_ACTIVE = 4
GRIPPER_TEST_CONFIRMATION = "TEST GRIPPER"
EXTENDED_PWM_CONFIRMATION = "ALLOW EXTENDED PWM"


class VehicleError(RuntimeError):
    """飞控连接、命令发送或安全条件不满足。"""


class MavlinkVehicle:
    """线程安全地持有唯一 MAVLink 连接并执行经过门控的动作。"""

    def __init__(
        self,
        config: RobotConfig,
        *,
        connection_factory: Callable[..., Any] | None = None,
    ) -> None:
        """保存配置但不连接，也不在导入模块时产生硬件副作用。"""

        self.config = config
        self._connection_factory = connection_factory
        self._master: Any = None
        self._lock = threading.RLock()
        self._telemetry = TelemetrySnapshot()
        self._last_motion_command_at: float | None = None
        self._last_output_at: float | None = None
        self._last_output_command = MotionCommand.neutral()
        self._gripper_action = GripperAction.NONE
        self._gripper_pending_steps: list[tuple[tuple[int, int], ...]] = []
        self._gripper_next_step_at: float | None = None
        self._preflight_report: PreflightReport | None = None
        self._closed = False

    @property
    def connected(self) -> bool:
        """返回当前是否持有有效连接对象。"""

        return self._master is not None and not self._closed

    @property
    def preflight_report(self) -> PreflightReport | None:
        """返回最近一次只读参数预检报告。"""

        return self._preflight_report

    @property
    def preflight_passed(self) -> bool:
        """只有已经运行且关键项全部通过时才返回 ``True``。"""

        return self._preflight_report is not None and self._preflight_report.passed

    def current_telemetry(self) -> TelemetrySnapshot:
        """返回当前缓存的遥测副本，不读取连接。"""

        return TelemetrySnapshot(**vars(self._telemetry))

    def connect(self) -> None:
        """连接飞控并等待心跳；本步骤本身不会发送执行器命令。"""

        if self.connected:
            return
        if self._connection_factory is None:
            try:
                from pymavlink import mavutil
            except ImportError as exc:
                raise VehicleError("缺少 pymavlink，无法连接飞控") from exc
            self._connection_factory = mavutil.mavlink_connection

        try:
            self._master = self._connection_factory(
                self.config.connection_uri,
                baud=self.config.baud,
                source_system=self.config.source_system,
                source_component=self.config.source_component,
            )
            deadline = time.monotonic() + self.config.heartbeat_timeout_s
            heartbeat = None
            last_identity: tuple[int, int] | None = None
            while time.monotonic() < deadline:
                candidate = self._master.wait_heartbeat(
                    timeout=min(1.0, max(0.0, deadline - time.monotonic()))
                )
                if candidate is None:
                    continue
                autopilot_type = int(getattr(candidate, "autopilot", -1))
                vehicle_type = int(getattr(candidate, "type", -1))
                last_identity = (autopilot_type, vehicle_type)
                if (
                    autopilot_type == MAV_AUTOPILOT_ARDUPILOTMEGA
                    and vehicle_type == MAV_TYPE_SUBMARINE
                ):
                    heartbeat = candidate
                    break
        except Exception as exc:
            self._master = None
            raise VehicleError(f"MAVLink 连接失败: {exc}") from exc
        if heartbeat is None:
            self._master = None
            raise VehicleError(
                f"{self.config.heartbeat_timeout_s:.1f} 秒内未收到 ArduSub 心跳；"
                f"最后身份={last_identity}"
            )
        source_system = self._message_source_system(heartbeat)
        source_component = self._message_source_component(heartbeat)
        if source_system is not None:
            self._master.target_system = source_system
        if source_component is not None:
            self._master.target_component = source_component
        self._closed = False
        now = time.monotonic()
        self._telemetry.last_heartbeat_monotonic = now
        self._telemetry.last_message_monotonic = now
        if hasattr(heartbeat, "get_type"):
            self._update_telemetry(heartbeat, now)

    def close(self) -> None:
        """先回中并释放 RC override，再关闭连接。

        上层网关负责在退出前请求上锁；本方法不隐藏执行解锁/上锁操作。
        """

        if self._master is None:
            self.cancel_gripper()
            self._closed = True
            return
        errors: list[str] = []
        try:
            self.cancel_gripper()
            if self.config.allow_live_actuation:
                try:
                    self.stop_motion(force=True)
                except VehicleError as exc:
                    errors.append(f"关闭前回中失败: {exc}")
                try:
                    self.release_control()
                except VehicleError as exc:
                    errors.append(f"关闭前释放控制失败: {exc}")
        finally:
            close_method = getattr(self._master, "close", None)
            if callable(close_method):
                close_method()
            self._master = None
            self._closed = True
        if errors:
            raise VehicleError("; ".join(errors))

    def run_preflight(self) -> PreflightReport:
        """只读飞控身份和参数，生成会门控真实执行的预检报告。"""

        if not self.connected:
            raise VehicleError("尚未连接飞控，无法执行参数预检")
        firmware_version = self._request_autopilot_version()
        parameters = self._fetch_all_parameters(self.config.preflight_timeout_s)
        report = build_preflight_report(
            self.config,
            target_system=int(getattr(self._master, "target_system", 0)),
            target_component=int(getattr(self._master, "target_component", 0)),
            autopilot_type=self._telemetry.autopilot_type,
            vehicle_type=self._telemetry.vehicle_type,
            firmware_version=firmware_version or self._telemetry.firmware_version,
            parameters=parameters,
            board_version=self._telemetry.board_version,
            vendor_id=self._telemetry.vendor_id,
            product_id=self._telemetry.product_id,
        )
        self._preflight_report = report
        return report

    def send_gcs_heartbeat(self) -> None:
        """以 1 Hz 发送控制端心跳，供飞控 GCS failsafe 监视。"""

        if not self.connected:
            raise VehicleError("尚未连接飞控")
        with self._lock:
            try:
                self._master.mav.heartbeat_send(
                    MAV_TYPE_GCS,
                    MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    MAV_STATE_ACTIVE,
                    3,
                )
            except Exception as exc:
                raise VehicleError(f"GCS 心跳发送失败: {exc}") from exc

    def send_motion(
        self,
        command: MotionCommand,
        *,
        now: float | None = None,
    ) -> MotionCommand:
        """限幅、限斜率后用固定后端发送四自由度运动意图。

        Returns:
            实际发送的归一化命令。调用者可据此显示斜坡后的真实软件输出。
        """

        self._require_live_actuation()
        self._require_preflight()
        self._require_fresh_heartbeat()
        self._require_vehicle_armed()
        self._require_allowed_mode()
        current = time.monotonic() if now is None else now
        try:
            limited = command.limited(self.config.command_limit)
            shaped = self._apply_slew_limit(limited, current)
        except ValueError as exc:
            self.stop_motion(force=True)
            self._last_motion_command_at = None
            raise VehicleError(f"运动指令非法，已回中: {exc}") from exc

        if self.config.control_protocol == ControlProtocol.MANUAL_CONTROL:
            self._send_manual_control(shaped)
        else:
            self._send_rc_motion(shaped)
        self._last_motion_command_at = current
        self._last_output_at = current
        self._last_output_command = shaped
        return shaped

    def stop_motion(self, *, force: bool = False) -> None:
        """发送协议对应的中位值，并清除运动看门狗时间戳。"""

        if not self.connected:
            return
        if not force:
            self._require_live_actuation()
        neutral = MotionCommand.neutral()
        if self.config.control_protocol == ControlProtocol.MANUAL_CONTROL:
            self._send_manual_control(neutral)
        else:
            self._send_rc_motion(neutral)
        now = time.monotonic()
        self._last_output_at = now
        self._last_output_command = neutral
        self._last_motion_command_at = None

    def release_control(self) -> None:
        """正常交还控制权；RC 后端会发送 MAVLink 规定的逐通道释放值。"""

        if not self.connected:
            return
        if self.config.control_protocol == ControlProtocol.MANUAL_CONTROL:
            # MANUAL_CONTROL 没有单独的 release 消息；回中后停止发送即可。
            self._send_manual_control(MotionCommand.neutral())
            return
        values = [65535] * 18
        for channel in self.config.rc_override.channels.values():
            # 通道 1..8 用 0 释放；9..18 用 UINT16_MAX-1 释放。
            values[channel - 1] = 0 if channel <= 8 else 65534
        self._send_rc_override(values)

    def stop_disarm_and_release(self) -> list[str]:
        """依次回中、正常上锁并释放控制，尽量执行每一步。

        某一步失败不会跳过后续保护动作。返回的错误列表为空表示
        三步均得到本地确认；非空时现场必须立即使用物理断电。
        """

        self._require_live_actuation()
        # 急停或正常结束都必须先停止后续渐变步骤，避免上锁过程中又发出
        # 一个迟到的机械爪 PWM。这里不会猜测“安全 PWM”，只停止发送。
        self.cancel_gripper()
        errors: list[str] = []
        try:
            self.stop_motion(force=True)
        except VehicleError as exc:
            errors.append(f"回中失败: {exc}")
        try:
            self.set_armed(False)
        except VehicleError as exc:
            errors.append(f"上锁失败: {exc}")
        try:
            self.release_control()
        except VehicleError as exc:
            errors.append(f"释放控制失败: {exc}")
        return errors

    def command_timed_out(self, now: float | None = None) -> bool:
        """最近一次运动命令超过配置时间时返回 ``True``。"""

        if self._last_motion_command_at is None:
            return False
        current = time.monotonic() if now is None else now
        return current - self._last_motion_command_at > self.config.command_timeout_s

    def heartbeat_is_fresh(self) -> bool:
        """飞控心跳存在且年龄未超过配置阈值。"""

        age = self._heartbeat_age()
        return age is not None and age <= self.config.heartbeat_stale_timeout_s

    def set_armed(self, armed: bool) -> None:
        """请求正常解锁或上锁，等待 COMMAND_ACK 和心跳状态确认。

        该方法永远把 ``param2`` 设为 0，不提供绕过预解锁检查的强制入口。
        """

        if not self.connected:
            raise VehicleError("尚未连接飞控")
        if armed:
            if not self.config.allow_ros_arming:
                raise VehicleError("ROS 解锁未通过 YAML 和启动参数授权")
            self._require_live_actuation()
            self._require_preflight()
            self._require_fresh_heartbeat()
            self._require_allowed_mode()
            if self._telemetry.armed is True:
                return
            if self._telemetry.armed is not False:
                raise VehicleError("飞控解锁状态未知，拒绝解锁")
            self.stop_motion(force=True)
        elif self._telemetry.armed is False:
            return

        with self._lock:
            try:
                self._master.mav.command_long_send(
                    self._master.target_system,
                    self._master.target_component,
                    MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1 if armed else 0,
                    0,  # 禁止使用 21196 强制绕过安全检查。
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                raise VehicleError(
                    f"{'解锁' if armed else '上锁'}命令发送失败: {exc}"
                ) from exc
        self._wait_for_armed_state(armed, self.config.arm_ack_timeout_s)

    @property
    def gripper_active(self) -> bool:
        """机械爪开闭序列尚未发送完成时返回 ``True``。"""

        with self._lock:
            return self._gripper_action != GripperAction.NONE

    def set_gripper(
        self, action: GripperAction, *, now: float | None = None
    ) -> tuple[int, float]:
        """开始一段非阻塞机械爪序列，返回节拍数和预计发送时长。

        旧工程在循环中 ``sleep`` 会让 ROS 网关停止处理遥测、看门狗和急停。
        新实现只立即发送第一节拍，其余节拍由 20 Hz 网关定时器调用
        :meth:`update_gripper` 继续发送。同一节拍可以包含 RST 的左右两路输出。
        """

        if action == GripperAction.NONE:
            return (0, 0.0)
        self._require_live_actuation()
        if not self.config.allow_gripper_actuation:
            raise VehicleError("机械爪真实输出未单独授权")
        if not self.config.gripper.calibrated:
            raise VehicleError("机械爪动作曲线尚未完成实机标定")
        if (
            self.config.gripper.uses_extended_pwm
            and not self.config.gripper.allow_extended_pwm
        ):
            raise VehicleError("机械爪档案包含扩展 PWM，但尚未单独授权")
        self._require_preflight()
        self._require_fresh_heartbeat()
        self._require_vehicle_armed()
        self._require_allowed_mode()
        steps = self.config.gripper.steps_for(action.value)
        current = time.monotonic() if now is None else now
        with self._lock:
            if self._gripper_action != GripperAction.NONE:
                raise VehicleError(
                    f"机械爪正在执行 {self._gripper_action.value}，拒绝重叠动作"
                )
            try:
                self._send_gripper_step(steps[0])
            except Exception as exc:
                raise VehicleError(f"机械爪命令发送失败: {exc}") from exc
            self._gripper_action = action
            self._gripper_pending_steps = list(steps[1:])
            self._gripper_next_step_at = (
                current + self.config.gripper.step_interval_s
                if self._gripper_pending_steps
                else None
            )
            if not self._gripper_pending_steps:
                self._gripper_action = GripperAction.NONE
        duration_s = max(0, len(steps) - 1) * self.config.gripper.step_interval_s
        return (len(steps), duration_s)

    def run_disarmed_gripper_test(
        self,
        action: GripperAction,
        *,
        confirmation: str,
        extended_pwm_confirmation: str = "",
        ack_timeout_s: float = 2.0,
        sleep_fn: Callable[[float], None] = time.sleep,
    ) -> tuple[dict[str, object], ...]:
        """在推进器物理断开的台架上，以上锁状态试验候选爪档案。

        这是一条与正常任务机械爪完全分开的调试通道：

        - 要求飞控明确上锁，且三个真实输出配置门全部关闭；
        - 要求只读预检证明输出未被 Motor1–Motor8 占用，AUX 已是 PWM；
        - 每一个 ``MAV_CMD_DO_SET_SERVO`` 都必须收到匹配 ACK；
        - 不会解锁、不写参数、不会把候选档案自动标记为已标定。

        它只供 ``rov_gripper_test`` 命令使用，不得从自主任务调用。
        """

        if action not in {GripperAction.OPEN, GripperAction.CLOSE}:
            raise VehicleError("机械爪候选测试只允许 open/close")
        if confirmation != GRIPPER_TEST_CONFIRMATION:
            raise VehicleError("机械爪台架测试确认词错误")
        if (
            self.config.allow_live_actuation
            or self.config.allow_ros_arming
            or self.config.allow_gripper_actuation
        ):
            raise VehicleError("候选爪测试必须使用三道真实输出门全关闭的配置副本")
        if self.config.gripper.uses_extended_pwm and (
            extended_pwm_confirmation != EXTENDED_PWM_CONFIRMATION
        ):
            raise VehicleError("该档案含扩展 PWM，第二确认词错误")
        if not self.connected:
            raise VehicleError("尚未连接飞控")
        self._require_preflight()
        # 预检后会等待操作员阅读安全提示并按 Enter。等待期间该命令
        # 没有后台 ROS 定时器读取 MAVLink，因此内存中的心跳时间戳可能
        # 过期，即使 UDP 接收队列里已经有新心跳。发送任何爪子指令前
        # 先非阻塞吸收最新遥测，然后再依据新状态检查心跳和上锁。
        self.poll_telemetry()
        self._require_fresh_heartbeat()
        if self._telemetry.armed is not False:
            raise VehicleError("飞控必须明确上锁，拒绝机械爪候选测试")

        checks = {check.name: check for check in self._preflight_report.checks}
        required_checks = ["机械爪输出可由 MAVLink 控制"]
        if any(9 <= output <= 14 for output in self.config.gripper.output_channels):
            required_checks.append("机械爪 AUX 输出已启用 PWM")
        for name in required_checks:
            check = checks.get(name)
            if check is None or not check.passed:
                observed = "报告中缺失" if check is None else check.observed
                raise VehicleError(f"候选爪预检未通过: {name} ({observed})")

        if not math.isfinite(ack_timeout_s) or ack_timeout_s <= 0.0:
            raise VehicleError("ACK 超时必须是正有限数")

        records: list[dict[str, object]] = []
        steps = self.config.gripper.steps_for(action.value)
        for step_index, step in enumerate(steps, start=1):
            # 每个节拍前重新吸收遥测，一旦飞控意外解锁就停止。
            self.poll_telemetry()
            self._require_fresh_heartbeat()
            if self._telemetry.armed is not False:
                raise VehicleError("测试期间飞控不再是上锁状态，立即停止")
            for output_channel, pwm in step:
                with self._lock:
                    try:
                        self._send_gripper_pwm(output_channel, pwm)
                    except Exception as exc:
                        raise VehicleError(f"候选爪命令发送失败: {exc}") from exc
                ack_result = self._wait_for_command_ack(
                    MAV_CMD_DO_SET_SERVO,
                    ack_timeout_s,
                )
                records.append(
                    {
                        "step": step_index,
                        "output_channel": output_channel,
                        "pwm": pwm,
                        "ack_result": ack_result,
                    }
                )
            if step_index < len(steps):
                sleep_fn(self.config.gripper.step_interval_s)
        return tuple(records)

    def update_gripper(self, *, now: float | None = None) -> bool:
        """到达下一节拍时发送该帧全部输出；否则返回 ``False``。"""

        current = time.monotonic() if now is None else now
        with self._lock:
            if self._gripper_action == GripperAction.NONE:
                return False
            if self._gripper_next_step_at is None or current < self._gripper_next_step_at:
                return False

        # 每一步都重新检查飞控状态。序列期间一旦断链、上锁或权限失效，
        # 不再发送剩余值，让上层网关进入锁定急停。
        self._require_live_actuation()
        self._require_preflight()
        self._require_fresh_heartbeat()
        self._require_vehicle_armed()
        self._require_allowed_mode()
        with self._lock:
            if not self._gripper_pending_steps:
                self._finish_gripper_sequence()
                return False
            step = self._gripper_pending_steps.pop(0)
            try:
                self._send_gripper_step(step)
            except Exception as exc:
                self._finish_gripper_sequence()
                raise VehicleError(f"机械爪序列发送失败: {exc}") from exc
            if self._gripper_pending_steps:
                # 调度变慢时不补发一串积压命令，始终从真实发送时刻重新计时。
                self._gripper_next_step_at = (
                    current + self.config.gripper.step_interval_s
                )
            else:
                self._finish_gripper_sequence()
            return True

    def cancel_gripper(self) -> None:
        """取消尚未发送的步骤；不臆测舵机的中位或反向动作。"""

        with self._lock:
            self._finish_gripper_sequence()

    def _finish_gripper_sequence(self) -> None:
        """清空机械爪调度状态；调用方已经持有可重入锁。"""

        self._gripper_action = GripperAction.NONE
        self._gripper_pending_steps.clear()
        self._gripper_next_step_at = None

    def _send_gripper_step(self, step: tuple[tuple[int, int], ...]) -> None:
        """依次发送同一节拍内的一路或多路绝对 SERVO 输出。"""

        for output_channel, pwm in step:
            self._send_gripper_pwm(output_channel, pwm)

    def _send_gripper_pwm(self, output_channel: int, pwm: int) -> None:
        """向给定绝对 SERVO 输出发送一个 MAVLink PWM 值。"""

        self._master.mav.command_long_send(
            self._master.target_system,
            self._master.target_component,
            MAV_CMD_DO_SET_SERVO,
            0,
            output_channel,
            pwm,
            0,
            0,
            0,
            0,
            0,
        )

    def poll_telemetry(self, maximum_messages: int = 100) -> TelemetrySnapshot:
        """非阻塞读取有限条 MAVLink 消息并返回真实遥测快照。"""

        if not self.connected:
            raise VehicleError("尚未连接飞控")
        for _ in range(maximum_messages):
            message = self._master.recv_match(blocking=False)
            if message is None:
                break
            if not self._message_is_from_target(message):
                continue
            self._update_telemetry(message, time.monotonic())
        return self.current_telemetry()

    def _apply_slew_limit(self, target: MotionCommand, now: float) -> MotionCommand:
        """限制相邻输出变化，避免第一次点动直接产生阶跃。"""

        if self._last_output_at is None:
            delta_time = 0.05
        else:
            delta_time = max(0.0, min(now - self._last_output_at, 1.0))
        maximum_step = self.config.slew_rate_per_s * delta_time

        def approach(previous: float, requested: float) -> float:
            change = max(-maximum_step, min(maximum_step, requested - previous))
            return previous + change

        previous = self._last_output_command
        return MotionCommand(
            forward=approach(previous.forward, target.forward),
            lateral=approach(previous.lateral, target.lateral),
            vertical=approach(previous.vertical, target.vertical),
            yaw=approach(previous.yaw, target.yaw),
        )

    def _send_manual_control(self, command: MotionCommand) -> None:
        """映射为 ArduSub 的 x/y/z/r，其中 z=500 表示升沉中位。"""

        directions = self.config.axis_directions
        x = round(command.forward * directions["forward"] * 1000)
        y = round(command.lateral * directions["lateral"] * 1000)
        z = round(500 + command.vertical * directions["vertical"] * 500)
        r = round(command.yaw * directions["yaw"] * 1000)
        with self._lock:
            try:
                self._master.mav.manual_control_send(
                    self._master.target_system,
                    x,
                    y,
                    max(0, min(1000, z)),
                    r,
                    0,
                )
            except Exception as exc:
                raise VehicleError(f"MANUAL_CONTROL 发送失败: {exc}") from exc

    def _send_rc_motion(self, command: MotionCommand) -> None:
        """按飞控 RC 标定把归一化运动意图转换为 RC override。"""

        values = [65535] * 18
        rc = self.config.rc_override
        for name, value in (
            ("forward", command.forward),
            ("lateral", command.lateral),
            ("vertical", command.vertical),
            ("yaw", command.yaw),
        ):
            directed = value * self.config.axis_directions[name]
            channel = rc.channels[name]
            minimum_pwm, neutral_pwm, maximum_pwm = self._actual_rc_calibration(channel)
            span = (
                maximum_pwm - neutral_pwm
                if directed >= 0
                else neutral_pwm - minimum_pwm
            )
            pwm = round(neutral_pwm + directed * span)
            values[channel - 1] = max(minimum_pwm, min(maximum_pwm, pwm))
        self._send_rc_override(values)

    def _actual_rc_calibration(self, channel: int) -> tuple[int, int, int]:
        """从最近一次只读飞控报告返回指定 RC 通道的 MIN/TRIM/MAX。"""

        if self._preflight_report is None:
            raise VehicleError("RC override 回中失败：尚无只读参数报告")
        parameters = self._preflight_report.parameters
        names = (f"RC{channel}_MIN", f"RC{channel}_TRIM", f"RC{channel}_MAX")
        try:
            numbers = tuple(round(float(parameters[name])) for name in names)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise VehicleError(
                f"RC{channel} 缺少有效 MIN/TRIM/MAX，无法安全回中"
            ) from exc
        minimum_pwm, neutral_pwm, maximum_pwm = numbers
        if not 800 <= minimum_pwm < neutral_pwm < maximum_pwm <= 2200:
            raise VehicleError(
                f"RC{channel} 标定不合法: {minimum_pwm}/{neutral_pwm}/{maximum_pwm}"
            )
        return minimum_pwm, neutral_pwm, maximum_pwm

    def _send_rc_override(self, values: list[int]) -> None:
        """以 MAVLink 2 的 18 通道格式发送 RC override。"""

        if len(values) != 18:
            raise VehicleError("RC override 必须包含 18 个字段")
        with self._lock:
            try:
                self._master.mav.rc_channels_override_send(
                    self._master.target_system,
                    self._master.target_component,
                    *values,
                )
            except Exception as exc:
                raise VehicleError(f"RC override 发送失败: {exc}") from exc

    def _wait_for_armed_state(self, desired: bool, timeout_s: float) -> None:
        """处理命令确认并以 HEARTBEAT 中的 armed 位作为最终事实。"""

        deadline = time.monotonic() + timeout_s
        accepted = False
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self._master.recv_match(
                blocking=True,
                timeout=min(0.2, remaining),
            )
            if message is None:
                continue
            if not self._message_is_from_target(message):
                continue
            message_type = message.get_type()
            self._update_telemetry(message, time.monotonic())
            if (
                message_type == "COMMAND_ACK"
                and int(message.command) == MAV_CMD_COMPONENT_ARM_DISARM
            ):
                result = int(message.result)
                if result == MAV_RESULT_ACCEPTED:
                    accepted = True
                elif result == MAV_RESULT_IN_PROGRESS:
                    # IN_PROGRESS 不是最终成功，继续等待 ACCEPTED。
                    continue
                else:
                    suffix = (
                        f"；飞控消息: {self._telemetry.last_status_text}"
                        if self._telemetry.last_status_text
                        else ""
                    )
                    raise VehicleError(
                        f"飞控拒绝{'解锁' if desired else '上锁'}，ACK={result}{suffix}"
                    )
            # 必须先收到匹配的成功 ACK，再以后续 HEARTBEAT 的
            # armed 位作为飞控真实状态证据。
            if (
                accepted
                and message_type == "HEARTBEAT"
                and self._telemetry.armed == desired
            ):
                return
        suffix = (
            f"；飞控消息: {self._telemetry.last_status_text}"
            if self._telemetry.last_status_text
            else ""
        )
        raise VehicleError(f"等待飞控{'解锁' if desired else '上锁'}确认超时{suffix}")

    def _wait_for_command_ack(self, command: int, timeout_s: float) -> int:
        """等待一个特定 MAVLink 命令的最终 ACK。"""

        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            message = self._master.recv_match(
                blocking=True,
                timeout=min(0.2, remaining),
            )
            if message is None or not self._message_is_from_target(message):
                continue
            self._update_telemetry(message, time.monotonic())
            if message.get_type() != "COMMAND_ACK":
                continue
            if int(getattr(message, "command", -1)) != int(command):
                continue
            result = int(getattr(message, "result", -1))
            if result == MAV_RESULT_IN_PROGRESS:
                continue
            if result != MAV_RESULT_ACCEPTED:
                raise VehicleError(f"飞控拒绝 MAVLink 命令 {command}，ACK={result}")
            return result
        raise VehicleError(f"等待 MAVLink 命令 {command} ACK 超时")

    def _request_autopilot_version(self) -> str | None:
        """请求 AUTOPILOT_VERSION；旧固件不响应时返回 ``None``。"""

        try:
            self._master.mav.command_long_send(
                self._master.target_system,
                self._master.target_component,
                MAV_CMD_REQUEST_MESSAGE,
                0,
                MAVLINK_MSG_ID_AUTOPILOT_VERSION,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        # 版本请求是可选增强；pymavlink 后端可能抛出不同异常类型，
        # 但参数预检仍必须继续，并在报告中标记版本缺失。
        except Exception:  # noqa: BLE001
            return None
        deadline = time.monotonic() + min(3.0, self.config.preflight_timeout_s)
        while time.monotonic() < deadline:
            message = self._master.recv_match(blocking=True, timeout=0.2)
            if message is None:
                continue
            if not self._message_is_from_target(message):
                continue
            self._update_telemetry(message, time.monotonic())
            if message.get_type() == "AUTOPILOT_VERSION":
                return self._telemetry.firmware_version
        return None

    def _fetch_all_parameters(self, timeout_s: float) -> dict[str, float]:
        """发送 PARAM_REQUEST_LIST 并在固定期限内收集飞控参数。"""

        try:
            self._master.mav.param_request_list_send(
                self._master.target_system,
                self._master.target_component,
            )
        except Exception as exc:
            raise VehicleError(f"参数列表请求失败: {exc}") from exc

        parameters: dict[str, float] = {}
        indexes: set[int] = set()
        expected_count: int | None = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            message = self._master.recv_match(blocking=True, timeout=0.25)
            if message is None:
                continue
            if not self._message_is_from_target(message):
                continue
            self._update_telemetry(message, time.monotonic())
            if message.get_type() != "PARAM_VALUE":
                continue
            name = self._decode_param_id(message.param_id)
            if name:
                parameters[name] = float(message.param_value)
            index = int(getattr(message, "param_index", -1))
            count = int(getattr(message, "param_count", -1))
            if index >= 0:
                indexes.add(index)
            if count > 0:
                expected_count = count
            if expected_count is not None and len(indexes) >= expected_count:
                break
        if not parameters:
            raise VehicleError("预检未收到任何 PARAM_VALUE；确认 MAVLink 端点为双向")
        if expected_count is None:
            raise VehicleError("参数列表没有声明 param_count，无法证明报告完整")
        if len(indexes) < expected_count:
            raise VehicleError(
                f"参数列表不完整: 收到 {len(indexes)}/{expected_count}；"
                "检查链路丢包或增大 mavlink.preflight_timeout_s 后重试"
            )
        return parameters

    @staticmethod
    def _decode_param_id(value: Any) -> str:
        """兼容 bytes 和 str 两种 pymavlink PARAM_VALUE 名称表示。"""

        if isinstance(value, bytes):
            return value.decode("ascii", errors="ignore").rstrip("\x00").strip()
        return str(value).rstrip("\x00").strip()

    @staticmethod
    def _message_source_system(message: Any) -> int | None:
        """读取 MAVLink 消息源 system id；测试替身缺少方法时返回 ``None``。"""

        getter = getattr(message, "get_srcSystem", None)
        return int(getter()) if callable(getter) else None

    @staticmethod
    def _message_source_component(message: Any) -> int | None:
        """读取 MAVLink 消息源 component id。"""

        getter = getattr(message, "get_srcComponent", None)
        return int(getter()) if callable(getter) else None

    def _message_is_from_target(self, message: Any) -> bool:
        """只接受当前目标飞控消息，忽略同一路由上的 QGC 或其他载具。"""

        source_system = self._message_source_system(message)
        source_component = self._message_source_component(message)
        target_system = int(getattr(self._master, "target_system", 0))
        target_component = int(getattr(self._master, "target_component", 0))
        system_matches = source_system is None or source_system == target_system
        component_matches = (
            source_component is None
            or target_component == 0
            or source_component == target_component
        )
        return system_matches and component_matches

    def _require_live_actuation(self) -> None:
        """检查配置和 launch 合并后的真实输出硬门。"""

        if not self.config.allow_live_actuation:
            raise VehicleError("真实推进器输出已禁用")
        if not self.connected:
            raise VehicleError("尚未连接飞控")

    def _require_preflight(self) -> None:
        """拒绝未运行或存在关键失败的实艇预检。"""

        if self._preflight_report is None:
            raise VehicleError("尚未完成只读飞控预检")
        if not self._preflight_report.passed:
            names = ", ".join(
                check.name for check in self._preflight_report.critical_failures
            )
            raise VehicleError(f"飞控预检未通过: {names}")

    def _heartbeat_age(self) -> float | None:
        """返回飞控心跳年龄；尚未收到时返回 ``None``。"""

        timestamp = self._telemetry.last_heartbeat_monotonic
        if timestamp is None:
            return None
        return max(0.0, time.monotonic() - timestamp)

    def _require_fresh_heartbeat(self) -> None:
        """心跳过期时拒绝动作；完整断链由飞控 failsafe 负责。"""

        if not self.heartbeat_is_fresh():
            raise VehicleError("飞控心跳超时，拒绝执行")

    def _require_vehicle_armed(self) -> None:
        """仅在飞控明确报告已解锁时允许运动。"""

        if self._telemetry.armed is not True:
            raise VehicleError("飞控尚未解锁，拒绝执行")

    def _require_allowed_mode(self) -> None:
        """只允许 YAML 明确列出的飞控模式接收 ROS 动作。"""

        mode = (self._telemetry.flight_mode or "").upper()
        if mode not in self.config.allowed_flight_modes:
            allowed = ", ".join(self.config.allowed_flight_modes)
            raise VehicleError(
                f"飞控模式 {mode or 'UNKNOWN'} 不允许控制；允许模式: {allowed}"
            )

    def _update_telemetry(self, message: Any, now: float) -> None:
        """把常用 MAVLink 消息更新到单一真实状态缓存。"""

        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return
        self._telemetry.last_message_monotonic = now
        if message_type == "HEARTBEAT":
            self._telemetry.last_heartbeat_monotonic = now
            self._telemetry.armed = bool(
                int(getattr(message, "base_mode", 0)) & MAV_MODE_FLAG_SAFETY_ARMED
            )
            self._telemetry.system_status = int(getattr(message, "system_status", 0))
            self._telemetry.autopilot_type = int(getattr(message, "autopilot", -1))
            self._telemetry.vehicle_type = int(getattr(message, "type", -1))
            mode_mapping_method = getattr(self._master, "mode_mapping", None)
            mode_mapping = (
                mode_mapping_method() if callable(mode_mapping_method) else {}
            )
            custom_mode = int(getattr(message, "custom_mode", -1))
            reverse_mapping = {
                value: key for key, value in (mode_mapping or {}).items()
            }
            self._telemetry.flight_mode = reverse_mapping.get(
                custom_mode, str(custom_mode)
            )
            return
        if message_type == "AUTOPILOT_VERSION":
            self._telemetry.firmware_version = self._format_firmware_version(
                int(getattr(message, "flight_sw_version", 0))
            )
            self._telemetry.board_version = int(getattr(message, "board_version", 0))
            self._telemetry.vendor_id = int(getattr(message, "vendor_id", 0))
            self._telemetry.product_id = int(getattr(message, "product_id", 0))
            return
        if message_type == "STATUSTEXT":
            text = getattr(message, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            self._telemetry.last_status_text = str(text).rstrip("\x00").strip()
            return
        if message_type == "ATTITUDE":
            self._telemetry.roll_deg = math.degrees(float(message.roll))
            self._telemetry.pitch_deg = math.degrees(float(message.pitch))
            self._telemetry.yaw_deg = math.degrees(float(message.yaw))
            self._telemetry.attitude_updated_monotonic = now
            return
        if message_type == self.config.depth_message and hasattr(
            message, self.config.depth_field
        ):
            raw_depth = float(getattr(message, self.config.depth_field))
            depth_m = (
                raw_depth * self.config.depth_multiplier + self.config.depth_offset_m
            )
            self._telemetry.depth_m = depth_m if math.isfinite(depth_m) else None
            self._telemetry.depth_updated_monotonic = now
            return
        if message_type == "SYS_STATUS":
            voltage_mv = int(getattr(message, "voltage_battery", 65535))
            current_centiamp = int(getattr(message, "current_battery", -1))
            remaining = int(getattr(message, "battery_remaining", -1))
            self._telemetry.battery_voltage_v = (
                None if voltage_mv == 65535 else voltage_mv / 1000.0
            )
            self._telemetry.battery_current_a = (
                None if current_centiamp < 0 else current_centiamp / 100.0
            )
            self._telemetry.battery_remaining_pct = None if remaining < 0 else remaining
            self._telemetry.battery_updated_monotonic = now

    @staticmethod
    def _format_firmware_version(encoded: int) -> str:
        """解析 AUTOPILOT_VERSION.flight_sw_version 的四段版本号。"""

        major = (encoded >> 24) & 0xFF
        minor = (encoded >> 16) & 0xFF
        patch = (encoded >> 8) & 0xFF
        release_type = encoded & 0xFF
        return f"{major}.{minor}.{patch} (type {release_type})"

    def __enter__(self) -> MavlinkVehicle:  # noqa: PYI034
        """进入上下文时连接飞控。"""

        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        """退出上下文时回中并关闭连接。"""

        self.close()
