"""ROS 2 自主节点：感知循环与 20 Hz 控制循环彻底分离。

YOLO 推理帧率可能随 GPU 负载波动，但飞控网关的命令看门狗不应因此
断流。本节点用一个循环专职取帧/推理，另一个固定 20 Hz 循环读取
最新快照、执行状态机并发布运动意图。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, replace
from typing import Any

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rov_interfaces.msg import (
    ControlStatus,
    MissionStatus,
    NormalizedMotionCommand,
    RobotTelemetry,
    TargetDetection,
    TargetDetectionArray,
)
from rov_interfaces.srv import SetArmed, SetGripper
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rov_competition.annotated_video import AnnotatedRtpPublisher, AnnotatedVideoError
from rov_competition.config import ControlProfile, load_autonomy_config, load_robot_config
from rov_competition.detector import DetectorError, YoloDetector, draw_competition_overlay
from rov_competition.domain import (
    GripperAction,
    MissionDecision,
    MissionObservation,
    MissionState,
    MotionCommand,
)
from rov_competition.mission import AutonomousGraspMission
from rov_competition.safety import (
    AutonomyControlStatus,
    AutonomyTelemetryStatus,
    PerceptionFreshness,
    autonomy_control_error,
    autonomy_safety_error,
    classify_perception_age,
)
from rov_competition.targets import load_target_config
from rov_competition.video import (
    GStreamerVideoSource,
    OpenCvVideoSource,
    VideoSourceError,
    build_udp_mpegts_url,
    build_udp_rtp_h264_gstreamer_pipeline,
    should_retry_live_video_interruption,
)


@dataclass(frozen=True)
class PerceptionSnapshot:
    """感知线程提供给控制线程的不可变快照。"""

    frame_id: int
    detections: tuple
    width: int
    height: int
    received_monotonic: float


def _source_value(value: str, *, udp_mpegts: bool) -> str | int:
    """解析摄像头编号、录像路径或 MPEG-TS UDP 端口。"""

    if udp_mpegts and value.isdecimal():
        return build_udp_mpegts_url(int(value))
    return int(value) if value.isdecimal() else value


class AutonomyNode(Node):
    """持续识别和显示；只在全部安全门通过后启动真实任务。"""

    def __init__(self) -> None:
        """加载配置、模型和视频，建立 ROS 接口。"""

        super().__init__("rov_autonomy")
        self.declare_parameter("robot_config", "config/robot.example.yaml")
        self.declare_parameter("autonomy_config", "config/autonomy.yaml")
        self.declare_parameter("targets_config", "config/targets.yaml")
        self.declare_parameter("video_source", "0")
        self.declare_parameter("gstreamer", False)
        self.declare_parameter("udp_mpegts", False)
        self.declare_parameter("display_window", False)
        self.declare_parameter("annotated_rtp_host", "")
        self.declare_parameter("annotated_rtp_port", 0)
        self.declare_parameter("command_source", "autonomy")
        # 搜索—接近水池测试复用本节点的 GPU 感知链，但任务状态与
        # 运动指令由独立 commissioning 节点发布。该模式不发布冲突的
        # /rov/mission/status，也不开放正式自主启动/中止服务。
        self.declare_parameter("perception_only", False)

        robot_path = str(self.get_parameter("robot_config").value)
        autonomy_path = str(self.get_parameter("autonomy_config").value)
        targets_path = str(self.get_parameter("targets_config").value)
        video_source = str(self.get_parameter("video_source").value)
        use_gstreamer = self._boolean_parameter("gstreamer")
        use_udp_mpegts = self._boolean_parameter("udp_mpegts")
        self._display_window = self._boolean_parameter("display_window")
        self._perception_only = self._boolean_parameter("perception_only")
        if use_gstreamer and use_udp_mpegts:
            raise ValueError("gstreamer 和 udp_mpegts 不能同时启用")
        if self._display_window:
            raise ValueError(
                "display_window 必须保持 false；请订阅 "
                "/rov/annotated_image/compressed 查看带框画面"
            )

        self._robot_config = load_robot_config(robot_path)
        self._config = load_autonomy_config(autonomy_path)
        self._target_config = load_target_config(targets_path)
        self._command_source = str(self.get_parameter("command_source").value).strip()
        if self._command_source != ControlProfile.AUTONOMY.value:
            raise ValueError("自主节点的 command_source 必须固定为 'autonomy'")
        unknown_targets = set(self._target_config.graspable_labels) - set(
            self._config.detector.class_names
        )
        if unknown_targets:
            raise ValueError(f"允许抓取列表包含模型未知类别: {sorted(unknown_targets)}")

        self._annotated_rtp_host = str(
            self.get_parameter("annotated_rtp_host").value
        ).strip()
        self._annotated_rtp_port = int(self.get_parameter("annotated_rtp_port").value)
        if bool(self._annotated_rtp_host) != (self._annotated_rtp_port != 0):
            raise ValueError("annotated_rtp_host 和 annotated_rtp_port 必须同时启用或禁用")
        self._annotated_writer: AnnotatedRtpPublisher | None = None
        if self._annotated_rtp_host:
            if not 1 <= self._annotated_rtp_port <= 65535:
                raise ValueError("annotated_rtp_port 必须在 1..65535")
            self._annotated_writer = AnnotatedRtpPublisher(
                self._annotated_rtp_host, self._annotated_rtp_port
            )

        try:
            self._detector = YoloDetector(self._config.detector)
            self._mission = AutonomousGraspMission(
                self._config.mission, self._target_config.graspable_labels
            )
            if use_gstreamer:
                pipeline = (
                    build_udp_rtp_h264_gstreamer_pipeline(int(video_source))
                    if video_source.isdecimal()
                    else video_source
                )
                self._video = GStreamerVideoSource(pipeline)
            else:
                self._video = OpenCvVideoSource(
                    _source_value(video_source, udp_mpegts=use_udp_mpegts)
                )
            self._video.open()
        except Exception:
            if self._annotated_writer is not None:
                self._annotated_writer.close()
            raise

        self._motion_publisher = self.create_publisher(
            NormalizedMotionCommand, "/rov/control/command", 10
        )
        self._detection_publisher = self.create_publisher(
            TargetDetectionArray, "/rov/detections", 10
        )
        self._image_publisher = self.create_publisher(
            CompressedImage, "/rov/annotated_image/compressed", 2
        )
        self._state_publisher = self.create_publisher(String, "/rov/mission_state", 10)
        self._mission_status_publisher = self.create_publisher(
            MissionStatus, "/rov/mission/status", 10
        )

        self._state_lock = threading.RLock()
        self._active = False
        self._failed = False
        self._destroying = False
        self._live_video_source = use_gstreamer
        self._video_recovery_pending = False
        self._last_video_warning_monotonic = 0.0
        self._terminal_action_started = False
        self._perception_hold_started_at: float | None = None
        self._frame_id = 0
        self._perception: PerceptionSnapshot | None = None
        self._telemetry_status: AutonomyTelemetryStatus | None = None
        self._control_status: AutonomyControlStatus | None = None
        self._latest_decision = MissionDecision(
            state=MissionState.IDLE,
            motion=MotionCommand.neutral(),
            message="识别显示中；自主运动未启动",
        )
        self._gripper_future: Any = None
        self._gripper_action = GripperAction.NONE
        self._frame_count = 0
        self._inference_started = time.monotonic()

        frame_group = MutuallyExclusiveCallbackGroup()
        control_group = MutuallyExclusiveCallbackGroup()
        telemetry_group = ReentrantCallbackGroup()
        service_group = MutuallyExclusiveCallbackGroup()
        self._telemetry_subscription = self.create_subscription(
            RobotTelemetry,
            "/rov/telemetry",
            self._handle_telemetry,
            10,
            callback_group=telemetry_group,
        )
        self._control_status_subscription = self.create_subscription(
            ControlStatus,
            "/rov/control/status",
            self._handle_control_status,
            10,
            callback_group=telemetry_group,
        )
        self._start_services = [] if self._perception_only else [
            self.create_service(
                Trigger,
                name,
                self._handle_start,
                callback_group=service_group,
            )
            for name in ("/rov/mission/start", "/rov/autonomy/start")
        ]
        self._abort_services = [] if self._perception_only else [
            self.create_service(
                Trigger,
                name,
                self._handle_abort,
                callback_group=service_group,
            )
            for name in ("/rov/mission/abort", "/rov/autonomy/abort")
        ]
        self._gripper_client = self.create_client(
            SetGripper, "/rov/control/set_gripper", callback_group=service_group
        )
        self._estop_client = self.create_client(
            Trigger, "/rov/control/emergency_stop", callback_group=service_group
        )
        self._arm_client = self.create_client(
            SetArmed, "/rov/control/set_armed", callback_group=service_group
        )
        self._frame_timer = self.create_timer(
            0.01, self._process_frame, callback_group=frame_group
        )
        self._control_timer = self.create_timer(
            0.05, self._control_tick, callback_group=control_group
        )
        self.get_logger().info(
            "Model/video ready; detection is active; "
            + (
                "perception-only mode; this node cannot start autonomous motion"
                if self._perception_only
                else "autonomous motion is NOT started"
            )
        )

    def _boolean_parameter(self, name: str) -> bool:
        """严格读取 ROS 布尔参数。"""

        value = self.get_parameter(name).value
        if not isinstance(value, bool):
            raise TypeError(f"{name} 必须是布尔值 true 或 false")
        return value

    def _handle_start(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """在配置、感知、遥测和网关门控全部通过后开始任务。"""

        with self._state_lock:
            if self._active:
                response.success = False
                response.message = "自主任务已经在运行"
                return response
            errors = list(
                self._config.mission.readiness_errors(
                    self._target_config.graspable_labels
                )
            )
            if self._robot_config.control_profile != ControlProfile.AUTONOMY:
                errors.append("robot.yaml 的 control.profile 不是 autonomy")
            now = time.monotonic()
            observation = self._build_observation(now)
            if observation is None:
                errors.append("尚未收到新鲜相机帧和完整遥测")
            telemetry_error = autonomy_safety_error(
                self._telemetry_status, self._config.mission, now=now
            )
            if telemetry_error is not None:
                errors.append(telemetry_error)
            control_error = self._control_status_error(now)
            if control_error is not None:
                errors.append(control_error)
            if not self._gripper_client.service_is_ready():
                errors.append("机械爪确认服务 /rov/control/set_gripper 不可用")
            if errors:
                response.success = False
                response.message = "自主启动被拒绝：" + "；".join(dict.fromkeys(errors))
                return response

            assert observation is not None
            self._mission = AutonomousGraspMission(
                self._config.mission, self._target_config.graspable_labels
            )
            self._terminal_action_started = False
            self._perception_hold_started_at = None
            self._active = True
            decision = self._mission.start(observation, now)
            self._publish_decision(decision, observation, publish_motion=True)
            if decision.gripper != GripperAction.NONE:
                self._request_gripper(decision.gripper, observation, now)
            response.success = decision.state != MissionState.ABORTED
            response.message = decision.message
            return response

    def _handle_abort(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """操作员中止：立即发布回中，并请求网关急停上锁。"""

        with self._state_lock:
            now = time.monotonic()
            observation = self._build_observation(now)
            decision = self._mission.abort("操作员或安全员中止任务", observation, now)
            self._active = False
            self._publish_decision(decision, observation, publish_motion=True)
            self._finish_terminal(decision)
            response.success = True
            response.message = decision.message
            return response

    def _handle_telemetry(self, message: RobotTelemetry) -> None:
        """缓存任务启动和持续运行所需的真实飞控状态。"""

        status = AutonomyTelemetryStatus(
            heartbeat_valid=bool(message.valid_heartbeat),
            heartbeat_age_s=float(message.heartbeat_age_s),
            message_age_s=float(message.message_age_s),
            armed=bool(message.armed),
            depth_valid=bool(message.valid_depth),
            depth_m=float(message.depth_m),
            attitude_valid=bool(message.valid_attitude),
            yaw_deg=float(message.yaw_deg),
            flight_mode=str(message.flight_mode),
            received_monotonic=time.monotonic(),
        )
        with self._state_lock:
            self._telemetry_status = status

    def _handle_control_status(self, message: ControlStatus) -> None:
        """缓存网关门控、预检、解锁和模式状态。"""

        status = AutonomyControlStatus(
            state=str(message.state),
            command_source=str(message.command_source),
            configuration_allows_actuation=bool(
                message.configuration_allows_actuation
            ),
            configuration_allows_ros_arming=bool(
                message.configuration_allows_ros_arming
            ),
            configuration_allows_gripper=bool(
                message.configuration_allows_gripper
            ),
            runtime_enabled=bool(message.runtime_enabled),
            preflight_passed=bool(message.preflight_passed),
            emergency_stop_latched=bool(message.emergency_stop_latched),
            armed_by_ros=bool(message.armed_by_ros),
            armed=bool(message.armed),
            flight_mode=str(message.flight_mode),
            received_monotonic=time.monotonic(),
        )
        with self._state_lock:
            self._control_status = status

    def _control_status_error(self, now: float) -> str | None:
        """返回网关不允许自主控制的第一个原因。"""

        return autonomy_control_error(
            self._control_status,
            self._config.mission,
            command_source=self._command_source,
            now=now,
        )

    def _build_observation(self, now: float) -> MissionObservation | None:
        """合并最新感知和飞控遥测；缺失任意一方时返回空。"""

        perception = self._perception
        telemetry = self._telemetry_status
        if perception is None or telemetry is None:
            return None
        perception_age = now - perception.received_monotonic
        perception_valid = (
            0.0 <= perception_age <= self._config.mission.maximum_perception_age_s
        )
        return MissionObservation(
            frame_id=perception.frame_id,
            detections=perception.detections,
            frame_width=perception.width,
            frame_height=perception.height,
            perception_valid=perception_valid,
            depth_valid=telemetry.depth_valid,
            depth_m=telemetry.depth_m,
            attitude_valid=telemetry.attitude_valid,
            yaw_deg=telemetry.yaw_deg,
        )

    def _perception_age_s(self, now: float) -> float:
        """返回最新检测帧年龄；尚无任何帧时视为无限大。"""

        if self._perception is None:
            return float("inf")
        return max(0.0, now - self._perception.received_monotonic)

    def _control_tick(self) -> None:
        """固定 20 Hz 执行安全检查、机械爪响应和状态机。"""

        with self._state_lock:
            now = time.monotonic()
            observation = self._build_observation(now)
            self._poll_gripper_future(observation, now)
            if not self._active:
                if not self._perception_only:
                    self._publish_mission_status(self._latest_decision, observation)
                return
            telemetry_error = autonomy_safety_error(
                self._telemetry_status, self._config.mission, now=now
            )
            control_error = self._control_status_error(now)
            perception_age = self._perception_age_s(now)
            perception_state = classify_perception_age(
                perception_age,
                hold_timeout_s=self._config.mission.perception_hold_timeout_s,
                abort_timeout_s=self._config.mission.maximum_perception_age_s,
            )
            if observation is None:
                error = "感知或遥测快照缺失"
            elif perception_state == PerceptionFreshness.ABORT:
                error = (
                    f"感知图像已过期 {perception_age:.2f}s，超过 "
                    f"{self._config.mission.maximum_perception_age_s:.2f}s"
                )
            else:
                error = telemetry_error or control_error
            if error is not None:
                decision = self._mission.abort(f"运行安全检查失败: {error}", observation, now)
                self._active = False
                self._publish_decision(decision, observation, publish_motion=True)
                self._finish_terminal(decision)
                return

            assert observation is not None
            if perception_state == PerceptionFreshness.HOLD:
                if self._perception_hold_started_at is None:
                    self._perception_hold_started_at = now
                    self.get_logger().warning(
                        f"感知暂停 {perception_age:.2f}s；四轴回中并冻结任务计时"
                    )
                decision = replace(
                    self._latest_decision,
                    motion=MotionCommand.neutral(),
                    message=(
                        f"感知暂停 {perception_age:.2f}s，四轴回中等待；"
                        f"{self._config.mission.maximum_perception_age_s:.1f}s 后硬中止"
                    ),
                )
                self._publish_decision(decision, observation, publish_motion=True)
                return
            if self._perception_hold_started_at is not None:
                paused_duration_s = now - self._perception_hold_started_at
                self._mission.delay_timers(paused_duration_s)
                self._perception_hold_started_at = None
                self.get_logger().info(
                    f"感知恢复；任务计时顺延 {paused_duration_s:.2f}s"
                )
            decision = self._mission.step(observation, now)
            self._publish_decision(decision, observation, publish_motion=True)
            if decision.gripper != GripperAction.NONE:
                self._request_gripper(decision.gripper, observation, now)
            if decision.state in {MissionState.COMPLETE, MissionState.ABORTED}:
                self._active = False
                self._finish_terminal(decision)

    def _request_gripper(
        self, action: GripperAction, observation: MissionObservation, now: float
    ) -> None:
        """只发送一个带时间戳和来源的机械爪服务请求。"""

        if self._gripper_future is not None:
            decision = self._mission.abort("上一个机械爪请求尚未结束", observation, now)
            self._active = False
            self._publish_decision(decision, observation, publish_motion=True)
            self._finish_terminal(decision)
            return
        if not self._gripper_client.service_is_ready():
            decision = self._mission.abort("机械爪服务不可用", observation, now)
            self._active = False
            self._publish_decision(decision, observation, publish_motion=True)
            self._finish_terminal(decision)
            return
        request = SetGripper.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.source = self._command_source
        request.action = (
            SetGripper.Request.OPEN
            if action == GripperAction.OPEN
            else SetGripper.Request.CLOSE
        )
        self._gripper_action = action
        self._gripper_future = self._gripper_client.call_async(request)

    def _poll_gripper_future(
        self, observation: MissionObservation | None, now: float
    ) -> None:
        """在 20 Hz 循环中非阻塞地取回机械爪响应。"""

        future = self._gripper_future
        if future is None or not future.done():
            return
        action = self._gripper_action
        self._gripper_future = None
        self._gripper_action = GripperAction.NONE
        if observation is None:
            decision = self._mission.abort("机械爪响应到达时观测已丢失", None, now)
        else:
            try:
                result = future.result()
                accepted = bool(result.success)
                message = str(result.message)
            except Exception as exc:  # ROS 客户端可返回多种中间件异常。
                accepted = False
                message = f"机械爪服务异常: {exc}"
            decision = self._mission.acknowledge_gripper(
                action, accepted, message, observation, now
            )
        self._publish_decision(decision, observation, publish_motion=True)
        if decision.state == MissionState.ABORTED:
            self._active = False
            self._finish_terminal(decision)

    def _process_frame(self) -> None:
        """感知循环：读图、YOLO 推理、更新快照并发布带框画面。"""

        try:
            ok, frame = self._video.read()
        except VideoSourceError as exc:
            self._handle_video_interruption(str(exc))
            return
        if not ok:
            self._handle_video_interruption("相机断流或录像结束")
            return
        if self._video_recovery_pending:
            self._video_recovery_pending = False
            self.get_logger().info("YOLO 视频已经自动恢复")
        try:
            detections = self._detector.detect(frame)
        except DetectorError as exc:
            self._fail_safe(str(exc))
            return
        height, width = frame.shape[:2]
        now = time.monotonic()
        with self._state_lock:
            self._frame_id += 1
            self._perception = PerceptionSnapshot(
                frame_id=self._frame_id,
                detections=tuple(detections),
                width=width,
                height=height,
                received_monotonic=now,
            )
            decision = self._latest_decision

        self._frame_count += 1
        elapsed = max(1e-6, now - self._inference_started)
        annotated = draw_competition_overlay(
            frame,
            detections,
            mission_state=decision.state.value,
            mission_message=decision.message,
            fps=self._frame_count / elapsed,
            selected_target=decision.selected_target,
            aim_point=(
                self._config.mission.grasp_aim_x_ratio,
                self._config.mission.grasp_aim_y_ratio,
            ),
            target_area_ratio=decision.target_area_ratio,
            grasp_area_threshold=decision.grasp_area_threshold,
            current_depth_m=decision.current_depth_m,
            scan_progress_deg=decision.scan_progress_deg,
            simulation=False,
        )
        self._publish_detections(detections, width, height, decision)
        self._publish_image(annotated)
        self._display_image(annotated)
        self._publish_annotated_rtp(annotated)

    def _handle_video_interruption(self, reason: str) -> None:
        """只读测试自动重连直播；真实任务仍按原安全策略立即中止。"""

        with self._state_lock:
            mission_active = self._active
        if not should_retry_live_video_interruption(
            live_stream=self._live_video_source,
            mission_active=mission_active,
        ):
            # 活动任务不再因一帧读取失败立刻急停。控制循环会在 1s 后
            # 回中冻结，并在配置的 5s 硬阈值后中止；录像结束也遵循同一规则。
            if mission_active:
                now = time.monotonic()
                if now - self._last_video_warning_monotonic >= 1.0:
                    self._last_video_warning_monotonic = now
                    self.get_logger().warning(
                        f"{reason}；等待视频恢复，控制循环将按感知年龄回中/中止"
                    )
                return
            self._fail_safe(reason)
            return

        now = time.monotonic()
        should_log = (
            not self._video_recovery_pending
            or now - self._last_video_warning_monotonic >= 10.0
        )
        self._video_recovery_pending = True
        if should_log:
            self._last_video_warning_monotonic = now
            if mission_active:
                self.get_logger().warning(
                    f"{reason}；正在重建 YOLO 解码管线，"
                    "活动任务已按感知年龄回中/计时"
                )
            else:
                self.get_logger().warning(
                    f"{reason}；自主任务未启动，正在重建 "
                    "YOLO 解码管线，QGC 不受影响"
                )

        try:
            self._video.release()
            self._video.open()
        except VideoSourceError as exc:
            if should_log:
                self.get_logger().warning(f"YOLO 视频重连尚未成功: {exc}")

    def _publish_decision(
        self,
        decision: MissionDecision,
        observation: MissionObservation | None,
        *,
        publish_motion: bool,
    ) -> None:
        """发布运动意图、兼容状态和结构化任务状态。"""

        self._latest_decision = decision
        if publish_motion:
            command = NormalizedMotionCommand()
            command.stamp = self.get_clock().now().to_msg()
            command.source = self._command_source
            command.forward = float(decision.motion.forward)
            command.lateral = float(decision.motion.lateral)
            command.vertical = float(decision.motion.vertical)
            command.yaw = float(decision.motion.yaw)
            self._motion_publisher.publish(command)
        self._state_publisher.publish(String(data=decision.state.value))
        self._publish_mission_status(decision, observation)

    def _publish_mission_status(
        self, decision: MissionDecision, observation: MissionObservation | None
    ) -> None:
        """发布可直接 echo/bag 的任务指标，不用日志文字反推状态。"""

        message = MissionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.active = self._active
        message.state = decision.state.value
        message.outcome = decision.outcome.value
        message.message = decision.message
        message.frame_id = 0 if observation is None else int(observation.frame_id)
        message.has_target = decision.selected_target is not None
        message.target_label = (
            "" if decision.selected_target is None else decision.selected_target.label
        )
        message.target_area_ratio = float(decision.target_area_ratio or 0.0)
        message.grasp_area_threshold = float(decision.grasp_area_threshold or 0.0)
        message.horizontal_error = float(decision.horizontal_error or 0.0)
        message.vertical_error = float(decision.vertical_error or 0.0)
        message.valid_depth = decision.current_depth_m is not None
        message.current_depth_m = float(decision.current_depth_m or 0.0)
        message.target_depth_m = float(decision.target_depth_m or 0.0)
        message.valid_scan_progress = decision.scan_progress_deg is not None
        message.scan_progress_deg = float(decision.scan_progress_deg or 0.0)
        message.estimated_advance_distance_m = float(
            decision.estimated_advance_distance_m or 0.0
        )
        message.search_cycle = int(decision.search_cycle)
        message.gripper_command_accepted = decision.gripper_command_accepted
        self._mission_status_publisher.publish(message)

    def _publish_detections(
        self, detections, width: int, height: int, decision: MissionDecision
    ) -> None:
        """发布当前新帧的全部结构化目标框。"""

        message = TargetDetectionArray()
        message.stamp = self.get_clock().now().to_msg()
        message.image_width = width
        message.image_height = height
        message.mission_state = decision.state.value
        message.mission_message = decision.message
        for item in detections:
            target = TargetDetection()
            target.class_id = item.class_id
            target.label = item.label
            target.confidence = item.confidence
            target.left = item.box.left
            target.top = item.box.top
            target.right = item.box.right
            target.bottom = item.box.bottom
            message.detections.append(target)
        self._detection_publisher.publish(message)

    def _publish_image(self, frame: Any) -> None:
        """JPEG 压缩后发布裁判可见的带框第一视角。"""

        import cv2

        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            self._fail_safe("标注画面 JPEG 编码失败")
            return
        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self._image_publisher.publish(message)

    def _display_image(self, frame: Any) -> None:
        """在操作主屏显示带框第一视角。"""

        if not self._display_window:
            return
        import cv2

        cv2.imshow("ROV autonomous recognition", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._fail_safe("操作员关闭自主识别窗口")

    def _publish_annotated_rtp(self, frame: Any) -> None:
        """将同一张带框画面发送到裁判视频桥。"""

        if self._annotated_writer is None:
            return
        try:
            self._annotated_writer.write(frame)
        except AnnotatedVideoError as exc:
            self._fail_safe(str(exc))

    def _finish_terminal(self, decision: MissionDecision) -> None:
        """中止时请求急停；正常完成时请求普通上锁/释放。"""

        if self._terminal_action_started:
            return
        self._terminal_action_started = True
        if decision.state == MissionState.ABORTED:
            if self._estop_client.service_is_ready():
                self._estop_client.call_async(Trigger.Request())
            else:
                self.get_logger().error(
                    "急停服务不可用；网关命令看门狗应会上锁，现场立即准备物理断电"
                )
            return
        if decision.state == MissionState.COMPLETE:
            if self._arm_client.service_is_ready():
                request = SetArmed.Request()
                request.arm = False
                request.confirmation = ""
                self._arm_client.call_async(request)
            else:
                self.get_logger().error(
                    "正常上锁服务不可用；立即使用 QGC 上锁并准备物理断电"
                )

    def _fail_safe(self, reason: str) -> None:
        """相机、推理或画面管线故障时停止感知并中止真实任务。"""

        with self._state_lock:
            if self._failed:
                return
            self._failed = True
            self.get_logger().error(reason)
            if self._active:
                now = time.monotonic()
                observation = self._build_observation(now)
                decision = self._mission.abort(reason, observation, now)
                self._active = False
                self._publish_decision(decision, observation, publish_motion=True)
                self._finish_terminal(decision)
            self._frame_timer.cancel()

    def destroy_node(self) -> bool:
        """退出时最后发送一次回中，活动任务同时请求急停。"""

        if self._destroying:
            return super().destroy_node()
        self._destroying = True
        if hasattr(self, "_mission") and self._active:
            now = time.monotonic()
            observation = self._build_observation(now)
            decision = self._mission.abort("自主节点退出", observation, now)
            self._active = False
            self._publish_decision(decision, observation, publish_motion=True)
            self._finish_terminal(decision)
        if hasattr(self, "_video"):
            self._video.release()
        if self._annotated_writer is not None:
            self._annotated_writer.close()
        if getattr(self, "_display_window", False):
            import cv2

            cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    """用三线程执行器启动感知、控制与 ROS 回调。"""

    rclpy.init()
    node: AutonomyNode | None = None
    executor: MultiThreadedExecutor | None = None
    try:
        node = AutonomyNode()
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.spin()
    except (
        AnnotatedVideoError,
        KeyboardInterrupt,
        DetectorError,
        TypeError,
        VideoSourceError,
        ValueError,
    ) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"自主节点启动失败: {exc}")
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
