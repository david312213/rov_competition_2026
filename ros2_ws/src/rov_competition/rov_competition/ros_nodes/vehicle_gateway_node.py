"""ROS 2 实艇控制网关。

该节点是唯一允许持有 MAVLink 连接的 ROS 进程。它把带来源和时间戳的
四轴运动意图交给 ArduSub，由飞控完成八推进器混控。真实输出、ROS
解锁和机械爪分别受独立安全门控；急停一旦触发，必须显式复位。
"""

from __future__ import annotations

import math
import time
from dataclasses import replace
from pathlib import Path

import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ControlStatus, NormalizedMotionCommand, RobotTelemetry
from rov_interfaces.srv import SetArmed, SetGripper
from std_msgs.msg import Bool, Float32, Int16, String
from std_srvs.srv import SetBool, Trigger

from rov_competition.config import RobotConfig, load_robot_config
from rov_competition.domain import ControlState, GripperAction, MotionCommand
from rov_competition.safety import is_fresh_telemetry, validate_command_envelope
from rov_competition.vehicle import MavlinkVehicle, VehicleError

ARM_CONFIRMATION = "ARM ROV"
COMMAND_FUTURE_TOLERANCE_S = 0.10


def _number_or_zero(value: float | None) -> float:
    """ROS 数值字段不能存 ``None``，无效值写 0 并由 valid 标志解释。"""

    return 0.0 if value is None or not math.isfinite(float(value)) else float(value)


def _valid_number(value: float | None) -> bool:
    """只有非空有限数值才能作为有效真实遥测发布。"""

    return value is not None and math.isfinite(float(value))


class VehicleGatewayNode(Node):
    """验证命令、维护安全状态机、执行 MAVLink 命令并发布遥测。"""

    def __init__(self) -> None:
        """连接飞控并建立控制、遥测、服务和看门狗接口。"""

        super().__init__("rov_vehicle_gateway")
        self.declare_parameter("robot_config", "config/robot.example.yaml")
        self.declare_parameter("enable_actuation", False)
        self.declare_parameter("enable_ros_arming", False)
        self.declare_parameter("preflight_output_dir", "output/preflight")

        config_path = str(self.get_parameter("robot_config").value)
        original_config = load_robot_config(config_path)
        launch_actuation = self._boolean_parameter("enable_actuation")
        launch_arming = self._boolean_parameter("enable_ros_arming")

        # 每个有效权限都是 YAML 与启动参数的交集。启动参数不能把 YAML
        # 中的 false “覆盖”成 true，从而防止一条误输命令直接打开推进器。
        effective_actuation = original_config.allow_live_actuation and launch_actuation
        effective_arming = (
            original_config.allow_ros_arming and launch_arming and effective_actuation
        )
        effective_gripper = (
            original_config.allow_gripper_actuation and effective_actuation
        )
        self._config: RobotConfig = replace(
            original_config,
            allow_live_actuation=effective_actuation,
            allow_ros_arming=effective_arming,
            allow_gripper_actuation=effective_gripper,
        )

        self._state = ControlState.LOCKED
        self._runtime_enabled = False
        self._estop_latched = False
        self._armed_by_ros = False
        self._reason = "等待运行时控制许可"
        self._last_command = MotionCommand.neutral()
        self._last_command_source = ""
        self._last_warning: tuple[str, float] | None = None
        self._destroying = False

        self._vehicle = MavlinkVehicle(self._config)
        self._vehicle.connect()
        self._run_startup_preflight_if_required()

        self._telemetry_publisher = self.create_publisher(
            RobotTelemetry, "/rov/telemetry", 10
        )
        self._status_publisher = self.create_publisher(
            ControlStatus, "/rov/control/status", 10
        )
        self._scalar_publishers = {
            "depth": self.create_publisher(Float32, "/rov/depth_m", 10),
            "roll": self.create_publisher(Float32, "/rov/roll_deg", 10),
            "pitch": self.create_publisher(Float32, "/rov/pitch_deg", 10),
            "yaw": self.create_publisher(Float32, "/rov/yaw_deg", 10),
            "voltage": self.create_publisher(Float32, "/rov/battery_voltage_v", 10),
            "current": self.create_publisher(Float32, "/rov/battery_current_a", 10),
            "remaining": self.create_publisher(Int16, "/rov/battery_remaining_pct", 10),
            "armed": self.create_publisher(Bool, "/rov/armed", 10),
            "mode": self.create_publisher(String, "/rov/flight_mode", 10),
        }

        self._motion_subscription = self.create_subscription(
            NormalizedMotionCommand,
            "/rov/control/command",
            self._handle_motion,
            10,
        )
        self._gripper_service = self.create_service(
            SetGripper, "/rov/control/set_gripper", self._handle_gripper
        )
        self._enable_service = self.create_service(
            SetBool, "/rov/control/set_enabled", self._handle_set_enabled
        )
        self._arm_service = self.create_service(
            SetArmed, "/rov/control/set_armed", self._handle_set_armed
        )
        self._estop_service = self.create_service(
            Trigger, "/rov/control/emergency_stop", self._handle_emergency_stop
        )
        self._reset_estop_service = self.create_service(
            Trigger,
            "/rov/control/reset_emergency_stop",
            self._handle_reset_emergency_stop,
        )

        self._telemetry_timer = self.create_timer(0.10, self._publish_snapshot)
        self._status_timer = self.create_timer(0.20, self._publish_control_status)
        self._watchdog_timer = self.create_timer(0.05, self._run_watchdog)
        self._gcs_heartbeat_timer = self.create_timer(1.0, self._send_gcs_heartbeat)

        output_state = "ENABLED" if effective_actuation else "DISABLED"
        arm_state = "ENABLED" if effective_arming else "DISABLED"
        self.get_logger().info(
            f"MAVLink 已连接；真实输出 {output_state}；ROS 解锁 {arm_state}；"
            f"协议 {self._config.control_protocol.value}；"
            f"机械爪档案 {self._config.gripper.profile}"
        )

    def _boolean_parameter(self, name: str) -> bool:
        """严格读取布尔启动参数，拒绝字符串 ``"true"`` 造成歧义。"""

        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise VehicleError(f"{name} 必须是布尔值 true 或 false")
        return value

    def _run_startup_preflight_if_required(self) -> None:
        """仅当本次启动可能真实输出时，在启动阶段执行只读预检。"""

        if not self._config.allow_live_actuation:
            return
        try:
            report = self._vehicle.run_preflight()
            output_directory = Path(
                str(self.get_parameter("preflight_output_dir").value)
            )
            json_path, markdown_path = report.write(output_directory)
            self.get_logger().info(
                f"只读预检报告已保存: {json_path} 和 {markdown_path}"
            )
            if report.passed:
                self._reason = "飞控预检通过，等待运行时控制许可"
            else:
                names = ", ".join(check.name for check in report.critical_failures)
                self._reason = f"飞控预检未通过: {names}"
                self.get_logger().error(self._reason)
        except (OSError, VehicleError) as exc:
            # 节点仍保留只读遥测和状态话题，但真实动作必然被门控拒绝。
            self._reason = f"飞控预检执行失败: {exc}"
            self.get_logger().error(self._reason)

    def _handle_motion(self, message: NormalizedMotionCommand) -> None:
        """验证来源、时间戳和数值，然后发送四轴运动意图。"""

        error = self._validate_command_message(message)
        if error is not None:
            self._reject_motion(error)
            return
        command = MotionCommand(
            forward=float(message.forward),
            lateral=float(message.lateral),
            vertical=float(message.vertical),
            yaw=float(message.yaw),
        ).limited(self._config.command_limit)
        # 即使当前还没解锁，也要记住合法发布者的最新意图。
        # 这样一个已在持续发非中位的节点不能趁解锁瞬间动作。
        self._last_command = command
        self._last_command_source = message.source
        if not self._runtime_enabled:
            self._reject_motion("运行时控制许可尚未开启")
            return
        if self._estop_latched or self._state == ControlState.ESTOPPED:
            self._reject_motion("急停锁定尚未复位")
            return

        snapshot = self._vehicle.current_telemetry()
        if snapshot.armed is not True:
            self._reject_motion("飞控尚未解锁")
            return
        if not self._armed_by_ros:
            self._reject_motion("本次解锁未经过 ROS 确认词服务")
            return

        try:
            sent = self._vehicle.send_motion(command)
        except VehicleError as exc:
            self._enter_emergency_stop(f"运动命令执行失败: {exc}")
            return

        self._last_command = sent
        self._last_command_source = message.source
        self._reason = "运动命令正常" if not sent.is_neutral() else "命令已回中"
        self._state = (
            ControlState.ACTIVE if not sent.is_neutral() else ControlState.READY
        )

    def _validate_command_message(self, message: NormalizedMotionCommand) -> str | None:
        """返回命令拒绝原因；完全合法时返回 ``None``。"""

        values = (
            float(message.forward),
            float(message.lateral),
            float(message.vertical),
            float(message.yaw),
        )
        stamp_s = float(message.stamp.sec) + float(message.stamp.nanosec) / 1e9
        now_s = self.get_clock().now().nanoseconds / 1e9
        return validate_command_envelope(
            source=message.source,
            expected_source=self._config.allowed_command_source,
            values=values,
            stamp_s=stamp_s,
            now_s=now_s,
            maximum_age_s=self._config.maximum_command_age_s,
            future_tolerance_s=COMMAND_FUTURE_TOLERANCE_S,
        )

    def _reject_motion(self, reason: str) -> None:
        """记录拒绝原因；已解锁时将非法输入升级为锁定急停。"""

        snapshot = self._vehicle.current_telemetry()
        if self._runtime_enabled and snapshot.armed is True:
            self._enter_emergency_stop(reason)
        else:
            self._reason = reason
            self._warn_throttled(reason)

    def _handle_gripper(
        self, request: SetGripper.Request, response: SetGripper.Response
    ) -> SetGripper.Response:
        """验证时间戳、来源和独立权限，并返回明确接受结果。

        服务成功只说明 MAVLink 机械爪命令已发送；没有指位反馈或
        力传感器时，网关不声称机械爪已物理抓牢。
        """

        stamp_s = float(request.stamp.sec) + float(request.stamp.nanosec) / 1e9
        now_s = self.get_clock().now().nanoseconds / 1e9
        error = validate_command_envelope(
            source=request.source,
            expected_source=self._config.allowed_command_source,
            values=(0.0,),
            stamp_s=stamp_s,
            now_s=now_s,
            maximum_age_s=self._config.maximum_command_age_s,
            future_tolerance_s=COMMAND_FUTURE_TOLERANCE_S,
        )
        if error is not None:
            return self._reject_gripper(error.replace("运动命令", "机械爪命令"), response)
        if not self._runtime_enabled:
            return self._reject_gripper("运行时控制许可尚未开启", response)
        if self._estop_latched or self._state == ControlState.ESTOPPED:
            return self._reject_gripper("急停锁定尚未复位", response)
        snapshot = self._vehicle.current_telemetry()
        if snapshot.armed is not True or not self._armed_by_ros:
            return self._reject_gripper("机械爪动作要求飞控已由 ROS 正常解锁", response)
        if int(request.action) == int(SetGripper.Request.OPEN):
            action = GripperAction.OPEN
        elif int(request.action) == int(SetGripper.Request.CLOSE):
            action = GripperAction.CLOSE
        else:
            return self._reject_gripper(
                f"无效机械爪动作枚举: {int(request.action)}", response
            )
        try:
            step_count, duration_s = self._vehicle.set_gripper(action)
        except VehicleError as exc:
            self._enter_emergency_stop(f"机械爪执行失败: {exc}")
            response.success = False
            response.message = self._reason
            return response
        self._reason = (
            f"机械爪档案 {self._config.gripper.profile} 的 {action.value} 序列已开始："
            f"{step_count} 个节拍，"
            f"预计 {duration_s:.2f}s 发完（无位置/抓牢反馈）"
        )
        response.success = True
        response.message = self._reason
        return response

    def _reject_gripper(
        self, reason: str, response: SetGripper.Response
    ) -> SetGripper.Response:
        """拒绝机械爪请求；活动解锁期的非法请求升级为急停。"""

        snapshot = self._vehicle.current_telemetry()
        if self._runtime_enabled and snapshot.armed is True:
            self._enter_emergency_stop(reason)
            reason = self._reason
        else:
            self._reason = reason
            self._warn_throttled(reason)
        response.success = False
        response.message = reason
        return response

    def _handle_set_enabled(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """开启或关闭运行时许可；开启时飞控必须明确处于上锁状态。"""

        if not request.data:
            errors = self._return_to_locked("运行时控制许可已关闭")
            response.success = not errors
            response.message = self._reason if not errors else "; ".join(errors)
            return response

        error = self._readiness_error(require_disarmed=True)
        if error is not None:
            response.success = False
            response.message = error
            self._reason = error
            return response
        if not self._last_command.is_neutral():
            response.success = False
            response.message = "最新的合法运动意图不是中位，拒绝开启运行时许可"
            self._reason = response.message
            return response
        try:
            # 在上锁状态先发一次回中，建立明确的软件起点。
            self._vehicle.stop_motion(force=True)
        except VehicleError as exc:
            response.success = False
            response.message = f"初始回中失败: {exc}"
            self._reason = response.message
            return response
        self._runtime_enabled = True
        self._armed_by_ros = False
        self._last_command = MotionCommand.neutral()
        self._state = ControlState.READY
        self._reason = "运行时许可已开启，飞控仍处于上锁状态"
        response.success = True
        response.message = self._reason
        return response

    def _handle_set_armed(
        self, request: SetArmed.Request, response: SetArmed.Response
    ) -> SetArmed.Response:
        """使用普通 MAVLink 解锁/上锁命令，并等待 ACK 与心跳确认。"""

        if not request.arm:
            errors = self._return_to_locked("ROS 已请求正常上锁")
            response.success = not errors
            response.message = self._reason if not errors else "; ".join(errors)
            return response

        if request.confirmation != ARM_CONFIRMATION:
            response.success = False
            response.message = f"确认词错误；解锁必须完整输入 {ARM_CONFIRMATION!r}"
            self._reason = response.message
            return response
        if not self._runtime_enabled:
            response.success = False
            response.message = "请先调用 /rov/control/set_enabled 开启运行时许可"
            self._reason = response.message
            return response
        if not self._last_command.is_neutral():
            response.success = False
            response.message = "当前运动命令不是中位，拒绝解锁"
            self._reason = response.message
            return response
        error = self._readiness_error(require_disarmed=True)
        if error is not None:
            response.success = False
            response.message = error
            self._reason = error
            return response
        try:
            self._vehicle.set_armed(True)
        except VehicleError as exc:
            response.success = False
            response.message = str(exc)
            self._reason = response.message
            return response
        self._armed_by_ros = True
        self._state = ControlState.READY
        self._reason = "飞控已解锁，等待中位或点动命令"
        response.success = True
        response.message = self._reason
        return response

    def _handle_emergency_stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """立即回中、尝试正常上锁并锁住 ROS 控制。"""

        errors = self._enter_emergency_stop("人工调用 ROS 急停")
        response.success = not errors
        response.message = (
            self._reason if not errors else self._physical_cutoff_message(errors)
        )
        return response

    def _handle_reset_emergency_stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """只在已上锁、心跳新鲜且预检通过时复位急停锁。"""

        if not self._estop_latched:
            response.success = True
            response.message = "急停未锁定，无需复位"
            return response
        snapshot = self._vehicle.current_telemetry()
        if snapshot.armed is not False:
            response.success = False
            response.message = "飞控必须明确上锁后才能复位急停"
            return response
        if not self._vehicle.heartbeat_is_fresh():
            response.success = False
            response.message = "飞控心跳不新鲜，拒绝复位急停"
            return response
        if not self._vehicle.preflight_passed:
            response.success = False
            response.message = "只读飞控预检未通过，拒绝复位急停"
            return response
        self._estop_latched = False
        self._runtime_enabled = False
        self._armed_by_ros = False
        self._state = ControlState.LOCKED
        self._reason = "急停已复位，需要重新开启运行时许可"
        response.success = True
        response.message = self._reason
        return response

    def _readiness_error(self, *, require_disarmed: bool) -> str | None:
        """检查打开控制或解锁前的共同条件。"""

        if not self._config.allow_live_actuation:
            return "真实输出未同时通过 YAML 和启动参数授权"
        if self._estop_latched:
            return "急停锁定尚未复位"
        if not self._vehicle.preflight_passed:
            return "只读飞控预检未通过"
        if not self._vehicle.heartbeat_is_fresh():
            return "飞控心跳不新鲜"
        snapshot = self._vehicle.current_telemetry()
        mode = (snapshot.flight_mode or "").upper()
        if mode not in self._config.allowed_flight_modes:
            return (
                f"飞控模式 {mode or 'UNKNOWN'} 不允许；必须为 "
                f"{', '.join(self._config.allowed_flight_modes)}"
            )
        if require_disarmed and snapshot.armed is not False:
            return "打开许可或解锁前，飞控必须明确处于上锁状态"
        return None

    def _return_to_locked(self, reason: str) -> list[str]:
        """正常回中、上锁和释放控制，最终进入 ``LOCKED``。"""

        errors: list[str] = []
        if self._config.allow_live_actuation:
            errors.extend(self._vehicle.stop_disarm_and_release())
        self._runtime_enabled = False
        self._armed_by_ros = False
        self._last_command = MotionCommand.neutral()
        self._state = ControlState.LOCKED
        self._reason = reason if not errors else self._physical_cutoff_message(errors)
        return errors

    def _enter_emergency_stop(self, reason: str) -> list[str]:
        """先锁住软件状态，再尝试回中、上锁和释放控制。"""

        self._estop_latched = True
        self._runtime_enabled = False
        self._armed_by_ros = False
        self._state = ControlState.ESTOPPED
        self._last_command = MotionCommand.neutral()
        errors: list[str] = []
        if self._config.allow_live_actuation:
            errors.extend(self._vehicle.stop_disarm_and_release())
        self._reason = reason if not errors else self._physical_cutoff_message(errors)
        if errors:
            self.get_logger().error(self._reason)
        else:
            self.get_logger().warning(f"急停已锁定: {reason}")
        return errors

    @staticmethod
    def _physical_cutoff_message(errors: list[str]) -> str:
        """在软件停车无法得到飞控确认时给出无歧义的现场指令。"""

        return "; ".join(errors) + "；无法确认软件停车，立即使用物理急停/断电"

    def _run_watchdog(self) -> None:
        """监视故障，并在已解锁待命时持续维持明确的中位输入。"""

        if self._estop_latched or not self._runtime_enabled:
            return
        snapshot = self._vehicle.current_telemetry()
        if snapshot.armed is True and not self._armed_by_ros:
            self._enter_emergency_stop("飞控在未经 ROS 确认词的情况下解锁")
            return
        if snapshot.armed is False and self._state == ControlState.ACTIVE:
            self._enter_emergency_stop("活动控制期间飞控意外上锁")
            return
        if snapshot.armed is False and self._armed_by_ros:
            self._runtime_enabled = False
            self._armed_by_ros = False
            self._state = ControlState.LOCKED
            self._reason = "飞控已从 ROS 控制中上锁，需重新开启许可"
            return
        if snapshot.armed is True and not self._vehicle.heartbeat_is_fresh():
            self._enter_emergency_stop("飞控心跳超时")
            return
        mode = (snapshot.flight_mode or "").upper()
        if snapshot.armed is True and mode not in self._config.allowed_flight_modes:
            self._enter_emergency_stop(f"飞控模式变为 {mode or 'UNKNOWN'}")
            return
        if self._vehicle.gripper_active:
            try:
                self._vehicle.update_gripper()
            except VehicleError as exc:
                self._enter_emergency_stop(f"机械爪渐变中止: {exc}")
                return
        if self._state == ControlState.ACTIVE and self._vehicle.command_timed_out():
            self._enter_emergency_stop("运动命令超时")
            return
        if (
            self._state == ControlState.READY
            and snapshot.armed is True
            and self._armed_by_ros
        ):
            # FS_PILOT_INPUT 会把“长时间没有 Pilot 输入”视为断链。
            # 操作员在解锁后输入下一条点动命令可能需要几秒，因此网关在
            # READY 状态以 20 Hz 保持中位。ACTIVE 状态绝不在这里续命：
            # 非中位发布者消失后仍由上面的 0.5 s 看门狗急停并上锁。
            try:
                self._vehicle.stop_motion(force=True)
            except VehicleError as exc:
                self._enter_emergency_stop(f"待命中位发送失败: {exc}")

    def _send_gcs_heartbeat(self) -> None:
        """ROS 持有运行时许可时才按 1 Hz 发送 GCS 心跳。

        锁定或急停后立即停发，一方面避免干扰 QGC 自己的 GCS
        failsafe，另一方面让软件上锁失败时仍可由飞控断链保护兜底。
        """

        if not self._runtime_enabled:
            return
        try:
            self._vehicle.send_gcs_heartbeat()
        except VehicleError as exc:
            if self._runtime_enabled:
                self._enter_emergency_stop(f"GCS 心跳发送失败: {exc}")
            else:
                self._warn_throttled(str(exc))

    def _publish_snapshot(self) -> None:
        """读取 MAVLink 消息并发布带有效性标志的真实遥测。"""

        try:
            snapshot = self._vehicle.poll_telemetry()
        except VehicleError as exc:
            if self._runtime_enabled:
                self._enter_emergency_stop(f"遥测读取失败: {exc}")
            else:
                self._warn_throttled(str(exc))
            return

        now = time.monotonic()
        stale_timeout = self._config.telemetry_stale_timeout_s
        depth_fresh = is_fresh_telemetry(
            snapshot.depth_updated_monotonic, now, stale_timeout
        )
        attitude_fresh = is_fresh_telemetry(
            snapshot.attitude_updated_monotonic, now, stale_timeout
        )
        battery_fresh = is_fresh_telemetry(
            snapshot.battery_updated_monotonic, now, stale_timeout
        )
        message = RobotTelemetry()
        message.stamp = self.get_clock().now().to_msg()
        message.valid_depth = depth_fresh and _valid_number(snapshot.depth_m)
        message.depth_m = _number_or_zero(snapshot.depth_m)
        attitude = (snapshot.roll_deg, snapshot.pitch_deg, snapshot.yaw_deg)
        message.valid_attitude = attitude_fresh and all(
            _valid_number(item) for item in attitude
        )
        message.roll_deg = _number_or_zero(snapshot.roll_deg)
        message.pitch_deg = _number_or_zero(snapshot.pitch_deg)
        message.yaw_deg = _number_or_zero(snapshot.yaw_deg)
        message.valid_battery_voltage = battery_fresh and _valid_number(
            snapshot.battery_voltage_v
        )
        message.battery_voltage_v = _number_or_zero(snapshot.battery_voltage_v)
        message.valid_battery_current = battery_fresh and _valid_number(
            snapshot.battery_current_a
        )
        message.battery_current_a = _number_or_zero(snapshot.battery_current_a)
        message.valid_battery_remaining = battery_fresh and _valid_number(
            snapshot.battery_remaining_pct
        )
        message.battery_remaining_pct = int(snapshot.battery_remaining_pct or 0)
        message.valid_heartbeat = self._vehicle.heartbeat_is_fresh()
        message.armed = bool(snapshot.armed)
        message.flight_mode = snapshot.flight_mode or "unknown"
        message.system_status = int(snapshot.system_status or 0)
        message.heartbeat_age_s = (
            0.0
            if snapshot.last_heartbeat_monotonic is None
            else max(0.0, now - snapshot.last_heartbeat_monotonic)
        )
        message.message_age_s = (
            0.0
            if snapshot.last_message_monotonic is None
            else max(0.0, now - snapshot.last_message_monotonic)
        )
        self._telemetry_publisher.publish(message)
        self._publish_scalar_topics(
            snapshot,
            depth_fresh=depth_fresh,
            attitude_fresh=attitude_fresh,
            battery_fresh=battery_fresh,
        )

    def _publish_scalar_topics(
        self,
        snapshot,
        *,
        depth_fresh: bool,
        attitude_fresh: bool,
        battery_fresh: bool,
    ) -> None:
        """将有效遥测拆成便于现场 ``ros2 topic echo`` 的独立话题。"""

        groups = (
            (depth_fresh, (("depth", snapshot.depth_m),)),
            (
                attitude_fresh,
                (
                    ("roll", snapshot.roll_deg),
                    ("pitch", snapshot.pitch_deg),
                    ("yaw", snapshot.yaw_deg),
                ),
            ),
            (
                battery_fresh,
                (
                    ("voltage", snapshot.battery_voltage_v),
                    ("current", snapshot.battery_current_a),
                ),
            ),
        )
        for fresh, values in groups:
            if not fresh:
                continue
            for key, value in values:
                if _valid_number(value):
                    self._scalar_publishers[key].publish(Float32(data=float(value)))
        if battery_fresh and _valid_number(snapshot.battery_remaining_pct):
            self._scalar_publishers["remaining"].publish(
                Int16(data=int(snapshot.battery_remaining_pct))
            )
        if snapshot.armed is not None:
            self._scalar_publishers["armed"].publish(Bool(data=bool(snapshot.armed)))
        if snapshot.flight_mode is not None:
            self._scalar_publishers["mode"].publish(String(data=snapshot.flight_mode))

    def _publish_control_status(self) -> None:
        """发布可以直接作为现场验收证据的控制门控状态。"""

        snapshot = self._vehicle.current_telemetry()
        message = ControlStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.state = self._state.value
        message.protocol = self._config.control_protocol.value
        message.command_source = self._last_command_source
        message.configuration_allows_actuation = self._config.allow_live_actuation
        message.configuration_allows_ros_arming = self._config.allow_ros_arming
        message.configuration_allows_gripper = self._config.allow_gripper_actuation
        message.runtime_enabled = self._runtime_enabled
        message.preflight_passed = self._vehicle.preflight_passed
        message.emergency_stop_latched = self._estop_latched
        message.armed_by_ros = self._armed_by_ros
        message.armed = bool(snapshot.armed)
        message.flight_mode = snapshot.flight_mode or "UNKNOWN"
        message.command_limit = float(self._config.command_limit)
        message.reason = self._reason
        self._status_publisher.publish(message)

    def _warn_throttled(self, message: str, interval_s: float = 2.0) -> None:
        """同一告警在指定时间内只打印一次，避免高频话题淹没根因。"""

        now = time.monotonic()
        if (
            self._last_warning is None
            or self._last_warning[0] != message
            or now - self._last_warning[1] >= interval_s
        ):
            self.get_logger().warning(message)
            self._last_warning = (message, now)

    def destroy_node(self) -> bool:
        """网关退出时尝试回中、正常上锁、释放控制并关闭连接。"""

        if self._destroying:
            return super().destroy_node()
        self._destroying = True
        if self._config.allow_live_actuation:
            self._enter_emergency_stop("飞控网关正在退出")
        try:
            self._vehicle.close()
        except VehicleError as exc:
            self.get_logger().error(f"关闭飞控连接失败: {exc}；请使用物理断电")
        return super().destroy_node()


def main() -> None:
    """启动实艇控制网关；``Ctrl+C`` 同样会进入锁定急停流程。"""

    rclpy.init()
    node: VehicleGatewayNode | None = None
    try:
        node = VehicleGatewayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning("收到 Ctrl+C，正在停车并上锁")
    except VehicleError as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"rov_vehicle 启动失败: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
