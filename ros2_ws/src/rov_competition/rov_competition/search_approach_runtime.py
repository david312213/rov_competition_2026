"""ROS 2 搜索—接近与抓取位置标定运行时。

YOLO 感知由 ``perception_only.launch.py`` 独立运行，本节点订阅结构化检测、
使用真实深度/航向执行纯 Python 状态机，并通过 commissioning 来源发布
四轴运动意图。自动流程不调用机械爪；人工标定流程只有在档案已经实机
确认并通过三道权限门时，才允许操作员显式按键开合机械爪。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
import traceback
from collections import deque
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rov_interfaces.msg import MissionStatus, TargetDetectionArray
from rov_interfaces.srv import SetGripper
from sensor_msgs.msg import CompressedImage

from .config import (
    ConfigurationError,
    ControlProfile,
    load_autonomy_config,
    load_dataset_config,
    load_robot_config,
)
from .dataset_drive import (
    DatasetDriveError,
    DatasetDriveNode,
    _active_error,
    _current_depth,
    _event_key_name,
    _prearm_error,
    _publish_neutral_with_health_check,
    _wait_for_gateway_state,
)
from .dataset_recording import RtpMkvRecorder, _git_commit, _sha256
from .domain import BoundingBox, Detection, MissionObservation, MotionCommand
from .grasp_calibration import (
    BoxMetrics,
    CalibrationTelemetry,
    GraspCalibrationError,
    GraspCalibrationSession,
    calculate_box_metrics,
    ordered_calibration_targets,
)
from .search_approach import (
    ManualCalibrationControl,
    SearchApproachMission,
    SearchTestDecision,
    SearchTestError,
    SearchTestState,
    SearchWorkflow,
    load_search_approach_config,
)
from .targets import load_target_config


CONFIRMATION = "START SEARCH TEST"
ARM_CONFIRMATION = "ARM ROV"
SOURCE = "commissioning"
MAXIMUM_PERCEPTION_AGE_S = 1.0
DETECTION_MAXIMUM_AGE_S = 0.50
IMAGE_MAXIMUM_AGE_S = 0.50
IMAGE_MATCH_TOLERANCE_S = 0.20


def _stamp_seconds(stamp: object) -> float:
    """把 ROS 时间戳转成秒，供检测与截图严格配对。"""

    return float(getattr(stamp, "sec")) + float(getattr(stamp, "nanosec")) / 1e9


@dataclass(frozen=True)
class DetectionSnapshot:
    stamp_s: float
    received_monotonic: float
    frame_id: int
    targets: tuple[BoxMetrics, ...]


@dataclass(frozen=True)
class ImageSnapshot:
    stamp_s: float
    received_monotonic: float
    jpeg_data: bytes


def _package_config_path(name: str) -> str:
    """优先返回 ROS 安装后的配置，源码环境则回退到包目录。"""

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return str(Path(__file__).resolve().parents[1] / "config" / name)
    try:
        root = Path(get_package_share_directory("rov_competition"))
    except PackageNotFoundError:
        root = Path(__file__).resolve().parents[1]
    return str(root / "config" / name)


def build_parser() -> argparse.ArgumentParser:
    """创建默认只预览的参数解析器。"""

    parser = argparse.ArgumentParser(
        description="ROV 360°搜索—对准—接近水池测试"
    )
    parser.add_argument("--robot-config", default=_package_config_path("robot.example.yaml"))
    parser.add_argument("--dataset-config", default=_package_config_path("dataset.example.yaml"))
    parser.add_argument("--autonomy-config", default=_package_config_path("autonomy.yaml"))
    parser.add_argument("--targets-config", default=_package_config_path("targets.yaml"))
    parser.add_argument("--search-config", default=_package_config_path("search_test.yaml"))
    parser.add_argument("--session-dir")
    parser.add_argument("--project-dir", default=str(Path.cwd()))
    parser.add_argument("--record-port", type=int, default=5704)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument(
        "--workflow",
        choices=tuple(item.value for item in SearchWorkflow),
        default=SearchWorkflow.AUTO_APPROACH.value,
        help="稳定对准后自动接近，或进入人工抓取位置标定",
    )
    parser.add_argument("--execute", action="store_true")
    return parser


class SearchSessionLogger:
    """保存状态、运动指令、每帧检测和会话元数据。"""

    EVENT_FIELDS = (
        "utc_time",
        "monotonic_s",
        "state",
        "message",
        "frame_id",
        "forward",
        "lateral",
        "vertical",
        "yaw",
        "depth_m",
        "yaw_deg",
        "target_label",
        "target_area_ratio",
        "horizontal_error",
        "scan_progress_deg",
        "search_cycle",
        "manual_power",
        "gripper_event",
    )
    DETECTION_FIELDS = (
        "utc_time",
        "frame_id",
        "label",
        "confidence",
        "left",
        "top",
        "right",
        "bottom",
        "area_ratio",
    )

    def __init__(
        self,
        directory: Path,
        *,
        project_directory: Path,
        robot_config: Path,
        search_config: Path,
        autonomy_config: Path,
        workflow: SearchWorkflow,
        gripper_profile: str,
    ) -> None:
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "logs").mkdir(exist_ok=True)
        self.events_path = directory / "events.csv"
        self.detections_path = directory / "detections.csv"
        self.session_path = directory / "session.json"
        self._events_file = self.events_path.open("w", encoding="utf-8", newline="")
        self._detections_file = self.detections_path.open(
            "w", encoding="utf-8", newline=""
        )
        self._events = csv.DictWriter(self._events_file, fieldnames=self.EVENT_FIELDS)
        self._detections = csv.DictWriter(
            self._detections_file, fieldnames=self.DETECTION_FIELDS
        )
        self._events.writeheader()
        self._detections.writeheader()
        self.metadata: dict[str, object] = {
            "mode": workflow.value,
            "version": "0.2.0rc2",
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "finished_at_utc": None,
            "outcome": "starting",
            "detail": "",
            "git_commit": _git_commit(project_directory),
            "robot_config": str(robot_config),
            "robot_config_sha256": _sha256(robot_config),
            "search_config": str(search_config),
            "search_config_sha256": _sha256(search_config),
            "autonomy_config": str(autonomy_config),
            "autonomy_config_sha256": _sha256(autonomy_config),
            "start_depth_m": None,
            "relative_descent_m": None,
            "target_depth_m": None,
            "descent_maximum_command": None,
            "end_depth_m": None,
            "video_file": None,
            "gripper_profile": gripper_profile,
            "grasp_sample_counts": None,
        }
        self._write_metadata()

    def set_start_parameters(
        self,
        *,
        start_depth_m: float,
        relative_descent_m: float,
        target_depth_m: float,
        descent_maximum_command: float,
    ) -> None:
        self.metadata.update(
            {
                "start_depth_m": start_depth_m,
                "relative_descent_m": relative_descent_m,
                "target_depth_m": target_depth_m,
                "descent_maximum_command": descent_maximum_command,
            }
        )
        self._write_metadata()

    def write_decision(
        self,
        decision: SearchTestDecision,
        observation: MissionObservation,
        *,
        motion: MotionCommand | None = None,
        manual_power: float | None = None,
        gripper_event: str = "",
    ) -> None:
        target = decision.selected_target
        actual_motion = decision.motion if motion is None else motion
        self._events.writerow(
            {
                "utc_time": datetime.now(timezone.utc).isoformat(),
                "monotonic_s": f"{time.monotonic():.6f}",
                "state": decision.state.value,
                "message": decision.message,
                "frame_id": observation.frame_id,
                "forward": f"{actual_motion.forward:.6f}",
                "lateral": f"{actual_motion.lateral:.6f}",
                "vertical": f"{actual_motion.vertical:.6f}",
                "yaw": f"{actual_motion.yaw:.6f}",
                "depth_m": f"{observation.depth_m:.6f}",
                "yaw_deg": f"{observation.yaw_deg:.6f}",
                "target_label": "" if target is None else target.label,
                "target_area_ratio": "" if decision.target_area_ratio is None else f"{decision.target_area_ratio:.8f}",
                "horizontal_error": "" if decision.horizontal_error is None else f"{decision.horizontal_error:.8f}",
                "scan_progress_deg": "" if decision.scan_progress_deg is None else f"{decision.scan_progress_deg:.6f}",
                "search_cycle": decision.search_cycle,
                "manual_power": "" if manual_power is None else f"{manual_power:.6f}",
                "gripper_event": gripper_event,
            }
        )
        self._events_file.flush()

    def write_detections(self, observation: MissionObservation) -> None:
        utc_time = datetime.now(timezone.utc).isoformat()
        if not observation.detections:
            self._detections.writerow(
                {"utc_time": utc_time, "frame_id": observation.frame_id, "label": ""}
            )
        for item in observation.detections:
            self._detections.writerow(
                {
                    "utc_time": utc_time,
                    "frame_id": observation.frame_id,
                    "label": item.label,
                    "confidence": f"{item.confidence:.8f}",
                    "left": f"{item.box.left:.3f}",
                    "top": f"{item.box.top:.3f}",
                    "right": f"{item.box.right:.3f}",
                    "bottom": f"{item.box.bottom:.3f}",
                    "area_ratio": f"{item.box.area_ratio(observation.frame_width, observation.frame_height):.8f}",
                }
            )
        self._detections_file.flush()

    def finish(
        self,
        *,
        outcome: str,
        detail: str,
        end_depth_m: float | None,
        video_path: Path | None,
        grasp_sample_counts: dict[str, int] | None = None,
    ) -> None:
        if self._events_file.closed:
            return
        self.metadata.update(
            {
                "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                "outcome": outcome,
                "detail": detail,
                "end_depth_m": end_depth_m,
                "video_file": str(video_path) if video_path else None,
                "grasp_sample_counts": grasp_sample_counts,
            }
        )
        self._write_metadata()
        self._events_file.close()
        self._detections_file.close()

    def _write_metadata(self) -> None:
        temporary = self.session_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.session_path)


class SearchApproachNode(DatasetDriveNode):
    """增加检测、带框截图、任务状态和显式机械爪服务。"""

    def __init__(self) -> None:
        super().__init__(source=SOURCE, node_name="rov_search_approach_test")
        self.detection_received_at: float | None = None
        self.detection_frame_id = 0
        self.detections: tuple[Detection, ...] = ()
        self.frame_width = 0
        self.frame_height = 0
        self.latest_detection: DetectionSnapshot | None = None
        self.images: deque[ImageSnapshot] = deque(maxlen=20)
        self.gripper_command_accepted = False
        self.last_gripper_message = ""
        self.create_subscription(
            TargetDetectionArray, "/rov/detections", self._handle_detections, 10
        )
        self.create_subscription(
            CompressedImage,
            "/rov/annotated_image/compressed",
            self._handle_image,
            5,
        )
        self.gripper_client = self.create_client(
            SetGripper, "/rov/control/set_gripper"
        )
        self.mission_status_publisher = self.create_publisher(
            MissionStatus, "/rov/mission/status", 10
        )

    def _handle_detections(self, message: TargetDetectionArray) -> None:
        self.detection_frame_id += 1
        self.detection_received_at = time.monotonic()
        self.frame_width = int(message.image_width)
        self.frame_height = int(message.image_height)
        self.detections = tuple(
            Detection(
                class_id=int(item.class_id),
                label=str(item.label),
                confidence=float(item.confidence),
                box=BoundingBox(
                    float(item.left),
                    float(item.top),
                    float(item.right),
                    float(item.bottom),
                ),
            )
            for item in message.detections
        )
        metrics: list[BoxMetrics] = []
        for item in message.detections:
            try:
                metrics.append(
                    calculate_box_metrics(
                        class_id=int(item.class_id),
                        label=str(item.label),
                        confidence=float(item.confidence),
                        frame_width=int(message.image_width),
                        frame_height=int(message.image_height),
                        left=float(item.left),
                        top=float(item.top),
                        right=float(item.right),
                        bottom=float(item.bottom),
                    )
                )
            except GraspCalibrationError:
                continue
        self.latest_detection = DetectionSnapshot(
            stamp_s=_stamp_seconds(message.stamp),
            received_monotonic=self.detection_received_at,
            frame_id=self.detection_frame_id,
            targets=tuple(metrics),
        )

    def _handle_image(self, message: CompressedImage) -> None:
        """缓存少量最新 JPEG，只用于保存与检测同帧的证据。"""

        if message.data:
            self.images.append(
                ImageSnapshot(
                    stamp_s=_stamp_seconds(message.header.stamp),
                    received_monotonic=time.monotonic(),
                    jpeg_data=bytes(message.data),
                )
            )

    def calibration_capture_arguments(
        self,
        *,
        allowed_labels: tuple[str, ...],
        selected_target: Detection | None,
        manual_power: float,
        motion: MotionCommand,
        gripper_profile: str,
    ) -> dict[str, object]:
        """取得同一检测时刻的框、截图、遥测和人工控制证据。"""

        snapshot = self.latest_detection
        now = time.monotonic()
        if (
            snapshot is None
            or now - snapshot.received_monotonic > DETECTION_MAXIMUM_AGE_S
        ):
            raise GraspCalibrationError("没有 0.5 秒内的新鲜检测框")
        ordered = ordered_calibration_targets(snapshot.targets, allowed_labels)
        if not ordered:
            raise GraspCalibrationError("当前帧没有允许抓取的目标框")
        target = ordered[0]
        if selected_target is not None:
            old_x, old_y = selected_target.box.center()
            old_x /= max(1, self.frame_width)
            old_y /= max(1, self.frame_height)
            same_label = [item for item in ordered if item.label == selected_target.label]
            if same_label:
                target = min(
                    same_label,
                    key=lambda item: math.hypot(
                        item.center_x_ratio - old_x,
                        item.center_y_ratio - old_y,
                    ),
                )
        if not self.images:
            raise GraspCalibrationError("尚未收到 /rov/annotated_image/compressed")
        image = min(self.images, key=lambda item: abs(item.stamp_s - snapshot.stamp_s))
        if now - image.received_monotonic > IMAGE_MAXIMUM_AGE_S:
            raise GraspCalibrationError("带框截图已经过期")
        if abs(image.stamp_s - snapshot.stamp_s) > IMAGE_MATCH_TOLERANCE_S:
            raise GraspCalibrationError(
                "检测框与截图时间差超过 0.20 秒，拒绝保存错帧证据"
            )
        telemetry = self.telemetry
        calibration_telemetry = CalibrationTelemetry()
        if telemetry is not None:
            calibration_telemetry = CalibrationTelemetry(
                depth_m=(float(telemetry.depth_m) if telemetry.valid_depth else None),
                roll_deg=(float(telemetry.roll_deg) if telemetry.valid_attitude else None),
                pitch_deg=(float(telemetry.pitch_deg) if telemetry.valid_attitude else None),
                yaw_deg=(float(telemetry.yaw_deg) if telemetry.valid_attitude else None),
                flight_mode=str(telemetry.flight_mode),
                armed=bool(telemetry.armed) if telemetry.valid_heartbeat else None,
            )
        return {
            "target": target,
            "telemetry": calibration_telemetry,
            "detection_stamp_s": snapshot.stamp_s,
            "image_stamp_s": image.stamp_s,
            "jpeg_data": image.jpeg_data,
            "frame_id": snapshot.frame_id,
            "manual_power": manual_power,
            "command_forward": motion.forward,
            "command_lateral": motion.lateral,
            "command_vertical": motion.vertical,
            "command_yaw": motion.yaw,
            "gripper_profile": gripper_profile,
        }

    def command_gripper(self, action: str) -> tuple[bool, str]:
        """显式调用开爪/闭爪服务，并返回是否接受及网关说明。"""

        request = SetGripper.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.source = SOURCE
        if action == "open":
            request.action = SetGripper.Request.OPEN
        elif action == "close":
            request.action = SetGripper.Request.CLOSE
        else:
            raise DatasetDriveError(f"未知机械爪动作: {action}")
        # 这里不使用 DatasetDriveNode._call：那个通用封装会把
        # response.success=false 立即变成异常，而标定记录必须先
        # 保存网关的“拒绝+原因”证据。等待期间仍以 20 Hz 回中，
        # 避免 0.5s 命令看门狗误判控制者失联。
        if not self.gripper_client.wait_for_service(timeout_sec=1.0):
            raise DatasetDriveError("服务不可用: /rov/control/set_gripper")
        future = self.gripper_client.call_async(request)
        deadline = time.monotonic() + 5.0
        next_neutral = 0.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.spin(0.02)
            now = time.monotonic()
            if now >= next_neutral:
                self.publish(MotionCommand.neutral())
                next_neutral = now + 0.05
        if not future.done():
            raise DatasetDriveError("服务超时: /rov/control/set_gripper")
        try:
            result = future.result()
        except Exception as exc:
            raise DatasetDriveError(
                f"服务异常 /rov/control/set_gripper: {exc}"
            ) from exc
        if result is None:
            raise DatasetDriveError("服务无返回内容: /rov/control/set_gripper")
        self.gripper_command_accepted = bool(result.success)
        self.last_gripper_message = str(result.message)
        return self.gripper_command_accepted, self.last_gripper_message

    def wait_for_perception(self, timeout_s: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.05)
            if self.detection_received_at is not None and self.frame_width > 0:
                return
        raise DatasetDriveError("15 秒内未收到 /rov/detections")

    def observation(self, now: float | None = None) -> MissionObservation:
        current = time.monotonic() if now is None else now
        telemetry = self.telemetry
        if telemetry is None or self.detection_received_at is None:
            raise DatasetDriveError("感知或遥测快照缺失")
        age = current - self.detection_received_at
        return MissionObservation(
            frame_id=self.detection_frame_id,
            detections=self.detections,
            frame_width=self.frame_width,
            frame_height=self.frame_height,
            perception_valid=0.0 <= age <= MAXIMUM_PERCEPTION_AGE_S,
            depth_valid=bool(telemetry.valid_depth),
            depth_m=float(telemetry.depth_m),
            attitude_valid=bool(telemetry.valid_attitude),
            yaw_deg=float(telemetry.yaw_deg),
        )

    def perception_age_s(self) -> float:
        if self.detection_received_at is None:
            return math.inf
        return time.monotonic() - self.detection_received_at

    def publish_mission_status(
        self, decision: SearchTestDecision, observation: MissionObservation
    ) -> None:
        message = MissionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.active = decision.state not in {
            SearchTestState.IDLE,
            SearchTestState.COMPLETE,
            SearchTestState.ABORTED,
        }
        message.state = decision.state.value
        message.outcome = decision.outcome
        message.message = decision.message
        message.frame_id = int(observation.frame_id)
        message.has_target = decision.selected_target is not None
        message.target_label = "" if decision.selected_target is None else decision.selected_target.label
        message.target_area_ratio = float(decision.target_area_ratio or 0.0)
        message.grasp_area_threshold = 0.10
        message.horizontal_error = float(decision.horizontal_error or 0.0)
        message.vertical_error = 0.0
        message.valid_depth = bool(observation.depth_valid)
        message.current_depth_m = float(observation.depth_m)
        message.target_depth_m = float(decision.target_depth_m or 0.0)
        message.valid_scan_progress = decision.scan_progress_deg is not None
        message.scan_progress_deg = float(decision.scan_progress_deg or 0.0)
        message.estimated_advance_distance_m = 0.0
        message.search_cycle = int(decision.search_cycle)
        message.gripper_command_accepted = self.gripper_command_accepted
        self.mission_status_publisher.publish(message)


def _prompt_float(prompt: str, default: float, minimum: float, maximum: float) -> float:
    """交互读取一个有限浮点数，空行使用默认值。"""

    raw = input(f"{prompt}[默认 {default:.2f}]：").strip()
    try:
        value = default if not raw else float(raw)
    except ValueError as exc:
        raise DatasetDriveError(f"无法读取数值: {raw!r}") from exc
    if not math.isfinite(value) or not minimum <= value <= maximum:
        raise DatasetDriveError(
            f"{prompt.strip()} 必须在 {minimum:.2f}..{maximum:.2f}"
        )
    return value


def _draw_window(
    pygame: object,
    screen: object,
    font: object,
    decision: SearchTestDecision,
    *,
    workflow: SearchWorkflow,
    paused: bool,
    perception_age_s: float,
    manual_control: ManualCalibrationControl,
    calibration: GraspCalibrationSession | None,
    gripper_profile: str,
    gripper_enabled: bool,
    operator_message: str,
) -> None:
    """绘制一个不遮挡 QGC 的高对比控制面板。

    窗口只呈现状态和键位，带框图像仍由 ROS 图像话题提供。
    这样不会在主控制循环内再做一次 JPEG 解码和缩放。
    """

    screen.fill((17, 24, 39))
    target = decision.selected_target
    manual = workflow == SearchWorkflow.MANUAL_GRASP_CALIBRATION
    counts = calibration.counts() if calibration is not None else {}
    lines = [
        (
            "ROV GRASP POSITION CALIBRATION"
            if manual
            else "ROV SEARCH-APPROACH POOL TEST"
        ),
        "SPACE neutral/pause | 0 return+disarm | ESC/close emergency stop",
    ]
    if manual:
        lines.extend(
            [
                "W/S forward/back | A/D left/right | 1/2 turn | UP/DOWN depth",
                "+/- power | Enter/G save | C close | O open | Y/N result | B bad | R rescan",
                (
                    f"power={manual_control.strength:.2f}  "
                    f"held={'+'.join(sorted(manual_control.held_keys)) or 'none'}  "
                    f"gripper={gripper_profile} ({'enabled' if gripper_enabled else 'DISABLED'})"
                ),
                (
                    "samples="
                    f"success:{counts.get('success', 0)} "
                    f"failure:{counts.get('failure', 0)} "
                    f"bad:{counts.get('bad_pose', 0)} "
                    f"pending:{counts.get('pending', 0)}"
                ),
            ]
        )
    else:
        lines.append("Automatic: descend -> scan -> align -> approach -> return")
    lines.extend(
        [
            f"state={decision.state.value}  cycle={decision.search_cycle}/3  paused={paused}",
            f"depth={decision.current_depth_m or 0.0:.2f} m  target_depth={decision.target_depth_m or 0.0:.2f} m",
            f"frame_age={perception_age_s:.3f}s  target={target.label if target else 'none'}",
            operator_message[:120] if operator_message else decision.message[:120],
        ]
    )
    for index, line in enumerate(lines):
        color = (235, 245, 255) if index < len(lines) - 1 else (255, 205, 80)
        screen.blit(font.render(line, True, color), (22, 16 + index * 34))
    pygame.display.flip()


def _interactive_parameters(
    node: SearchApproachNode,
    maximum_depth_m: float,
) -> tuple[float, float, float, float]:
    if not sys.stdin.isatty():
        raise DatasetDriveError("真实测试必须在交互终端运行")
    observation = node.observation()
    if not observation.depth_valid or not math.isfinite(observation.depth_m):
        raise DatasetDriveError("启动前深度反馈无效")
    relative = _prompt_float("相对下潜距离（米）", 0.30, 0.01, maximum_depth_m)
    descent_power = _prompt_float("下潜最大 power ", 0.20, 0.10, 0.40)
    start_depth = observation.depth_m
    target_depth = start_depth + relative
    if target_depth > maximum_depth_m:
        raise DatasetDriveError(
            f"目标深度 {target_depth:.2f} m 超过配置上限 {maximum_depth_m:.2f} m"
        )
    print("\n本次测试参数：")
    print(f"  启动深度：{start_depth:.2f} m")
    print(f"  相对下潜：{relative:.2f} m")
    print(f"  目标深度：{target_depth:.2f} m")
    print(f"  下潜最大 power：{descent_power:.2f}")
    print(
        "\n解锁前确认：ROV 已浸没、危险区无人、ALT_HOLD、"
        "QGC 遥测和 5600 画面正常、QGC 可人工上锁、安全员可断电。"
    )
    typed = input(f"全部满足后完整输入 {CONFIRMATION!r}: ").strip()
    if typed != CONFIRMATION:
        raise DatasetDriveError("确认词不匹配，未开启控制")
    return relative, descent_power, start_depth, target_depth


def main(argv: list[str] | None = None) -> int:
    """预览配置，或执行一次可中止、可追溯的搜索测试。"""

    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    workflow = SearchWorkflow(args.workflow)
    manual_mode = workflow == SearchWorkflow.MANUAL_GRASP_CALIBRATION
    try:
        robot = load_robot_config(args.robot_config)
        dataset = load_dataset_config(args.dataset_config)
        autonomy = load_autonomy_config(args.autonomy_config)
        targets = load_target_config(args.targets_config)
        search = load_search_approach_config(args.search_config)
    except (ConfigurationError, SearchTestError, ValueError) as exc:
        print(f"配置错误: {exc}")
        return 2

    # dataset.yaml 在此只提供链路新鲜度和 ALT_HOLD 安全阈值。搜索测试的
    # 最大输出为 0.40，不能被键盘采集的 maximum_command=0.80 误挡住。
    errors = list(
        search.readiness_errors(
            robot_command_limit=robot.command_limit,
            workflow=workflow,
        )
    )
    if robot.control_profile != ControlProfile.COMMISSIONING:
        errors.append("robot.yaml 的 profile 必须是 commissioning")
    if "ALT_HOLD" not in robot.allowed_flight_modes:
        errors.append("robot.yaml 必须允许 ALT_HOLD")
    if not robot.allow_live_actuation:
        errors.append("robot.yaml 尚未允许真实输出")
    if not robot.allow_ros_arming:
        errors.append("robot.yaml 尚未允许 ROS 解锁")
    if not manual_mode and robot.allow_gripper_actuation:
        errors.append("自动接近测试必须关闭机械爪权限")
    unknown = set(targets.graspable_labels) - set(autonomy.detector.class_names)
    if unknown:
        errors.append(f"目标列表包含模型未知类别: {sorted(unknown)}")
    if errors:
        print("禁止真实搜索测试：")
        for error in dict.fromkeys(errors):
            print(f"  - {error}")
        return 2
    if not args.execute:
        print("配置预览通过；未初始化 ROS、未录像、不会解锁。")
        if manual_mode:
            print(
                "稳定对准后停止自动运动，进入 WASD 人工抓取位置标定。"
            )
            if not robot.allow_gripper_actuation:
                print("机械爪权限当前关闭；C/O 将被禁用，其他标定功能可用。")
        else:
            print("扫描 power=0.20，无目标前进 power=0.40/2.0s，面积 0.10 停止。")
        return 0
    if not args.session_dir:
        print("真实执行必须使用 --session-dir")
        return 2

    try:
        import pygame
    except ImportError:
        print("缺少 pygame，请重新执行安装脚本。")
        return 2

    session_dir = Path(args.session_dir).expanduser().resolve()
    logger: SearchSessionLogger | None = None
    calibration: GraspCalibrationSession | None = None
    recorder: RtpMkvRecorder | None = None
    node: SearchApproachNode | None = None
    pygame_started = False
    control_opened = False
    normal_complete = False
    video_path: Path | None = None
    outcome = "failed_before_arm"
    detail = "启动未完成"
    result_code = 2
    operator_message = ""

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = SearchApproachNode()
        node.wait_for_initial_data(timeout_s=10.0)
        node.wait_for_perception(timeout_s=20.0)
        error = _prearm_error(
            node,
            dataset,
            require_disarmed=True,
            allow_gripper=manual_mode,
        )
        if error is not None:
            raise DatasetDriveError(error)
        if node.perception_age_s() > MAXIMUM_PERCEPTION_AGE_S:
            raise DatasetDriveError("启动前感知已过期")

        pygame.init()
        pygame_started = True
        screen = pygame.display.set_mode((1120, 390 if manual_mode else 320))
        pygame.display.set_caption(
            "ROV Grasp Position Calibration"
            if manual_mode
            else "ROV Search-Approach Pool Test"
        )
        font = pygame.font.Font(None, 27)
        screen.fill((25, 25, 25))
        pygame.display.flip()

        logger = SearchSessionLogger(
            session_dir,
            project_directory=Path(args.project_dir).expanduser().resolve(),
            robot_config=Path(args.robot_config).expanduser().resolve(),
            search_config=Path(args.search_config).expanduser().resolve(),
            autonomy_config=Path(args.autonomy_config).expanduser().resolve(),
            workflow=workflow,
            gripper_profile=robot.gripper.profile,
        )
        if manual_mode:
            calibration = GraspCalibrationSession(
                session_dir,
                target_labels=targets.graspable_labels,
                display_names=dict(targets.display_names),
                autonomy_config_path=args.autonomy_config,
                targets_config_path=args.targets_config,
                minimum_success_samples=search.minimum_success_samples,
                allow_existing_directory=True,
            )
        recorder = RtpMkvRecorder(
            session_dir,
            source_port=args.record_port,
            payload_type=args.payload_type,
        )
        recorder.start()
        recorder.wait_until_receiving(timeout_s=10.0, pump=lambda: node.spin(0.0))
        relative, descent_power, start_depth, target_depth = _interactive_parameters(
            node, search.maximum_operation_depth_m
        )
        logger.set_start_parameters(
            start_depth_m=start_depth,
            relative_descent_m=relative,
            target_depth_m=target_depth,
            descent_maximum_command=descent_power,
        )

        for _ in range(5):
            node.publish(MotionCommand.neutral())
            node.spin(0.02)
        node.set_enabled(True)
        control_opened = True
        _wait_for_gateway_state(
            node,
            lambda: bool(node.status and node.status.runtime_enabled),
            "运行时许可",
        )
        node.arm()
        _wait_for_gateway_state(
            node,
            lambda: bool(node.status and node.status.armed and node.status.armed_by_ros),
            "ROS 解锁",
        )
        error = _active_error(node, dataset, allow_gripper=manual_mode)
        if error is not None:
            raise DatasetDriveError(error)

        mission = SearchApproachMission(
            search,
            targets.graspable_labels,
            workflow=workflow,
        )
        manual_control = ManualCalibrationControl(search)
        observation = node.observation()
        decision = mission.start(
            observation,
            relative_descent_m=relative,
            descent_maximum_command=descent_power,
            now=time.monotonic(),
        )
        node.publish(decision.motion)
        node.publish_mission_status(decision, observation)
        logger.write_decision(decision, observation)
        last_logged_frame = -1
        paused = False
        pause_started_at: float | None = None
        last_manual_motion = MotionCommand.neutral()
        last_state = decision.state
        gripper_event = ""
        clock = pygame.time.Clock()
        next_publish = time.monotonic()

        while rclpy.ok():
            normal_finish_requested = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise DatasetDriveError("控制窗口被关闭")
                if event.type in {
                    getattr(pygame, "WINDOWFOCUSLOST", -1),
                } or (event.type == pygame.ACTIVEEVENT and getattr(event, "gain", 1) == 0):
                    if not paused:
                        pause_started_at = time.monotonic()
                    paused = True
                    manual_control.clear()
                    operator_message = "窗口失去焦点，已回中；回到窗口后按 Enter 恢复"
                    node.publish(MotionCommand.neutral())
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        raise DatasetDriveError("Esc 急停")
                    if event.key in {pygame.K_0, pygame.K_KP0}:
                        normal_finish_requested = True
                    if event.key == pygame.K_SPACE:
                        manual_control.clear()
                        node.publish(MotionCommand.neutral())
                        if decision.state != SearchTestState.MANUAL_CALIBRATION:
                            if not paused:
                                pause_started_at = time.monotonic()
                            paused = True
                            operator_message = "Space 已暂停自动运动；按 Enter 恢复"
                        else:
                            operator_message = "Space 已回中，保持当前深度"
                    if event.key in {pygame.K_RETURN, pygame.K_KP_ENTER} and paused:
                        if node.perception_age_s() <= MAXIMUM_PERCEPTION_AGE_S:
                            if pause_started_at is not None:
                                mission.delay_timers(time.monotonic() - pause_started_at)
                            paused = False
                            pause_started_at = None
                            operator_message = "已恢复"
                        continue

                    in_manual = (
                        manual_mode
                        and decision.state == SearchTestState.MANUAL_CALIBRATION
                    )
                    key_name = _event_key_name(pygame, event.key)
                    if in_manual and key_name is not None:
                        manual_control.press(key_name)
                        operator_message = f"按住 {key_name} 人工精调"
                    elif in_manual and event.key in {
                        pygame.K_PLUS,
                        pygame.K_EQUALS,
                        pygame.K_KP_PLUS,
                    }:
                        operator_message = (
                            f"power -> {manual_control.adjust(1):.2f}"
                        )
                    elif in_manual and event.key in {
                        pygame.K_MINUS,
                        pygame.K_KP_MINUS,
                    }:
                        operator_message = (
                            f"power -> {manual_control.adjust(-1):.2f}"
                        )
                    elif in_manual and event.key in {
                        pygame.K_RETURN,
                        pygame.K_KP_ENTER,
                        pygame.K_g,
                    }:
                        if calibration is None:
                            operator_message = "保存失败：标定会话未初始化"
                            continue
                        manual_control.clear()
                        node.publish(MotionCommand.neutral())
                        try:
                            sample = calibration.capture(
                                **node.calibration_capture_arguments(
                                    allowed_labels=targets.graspable_labels,
                                    selected_target=decision.selected_target,
                                    manual_power=manual_control.strength,
                                    motion=last_manual_motion,
                                    gripper_profile=robot.gripper.profile,
                                )
                            )
                        except GraspCalibrationError as exc:
                            operator_message = f"保存失败：{exc}"
                            continue
                        operator_message = (
                            f"已保存样本 {sample.sample_id}；现在可按 C 闭爪"
                        )
                        gripper_event = f"sample_saved:{sample.sample_id}"
                    elif in_manual and event.key == pygame.K_b:
                        if calibration is None:
                            operator_message = "记录失败：标定会话未初始化"
                            continue
                        manual_control.clear()
                        node.publish(MotionCommand.neutral())
                        try:
                            sample = calibration.capture_bad_pose(
                                **node.calibration_capture_arguments(
                                    allowed_labels=targets.graspable_labels,
                                    selected_target=decision.selected_target,
                                    manual_power=manual_control.strength,
                                    motion=last_manual_motion,
                                    gripper_profile=robot.gripper.profile,
                                )
                            )
                        except GraspCalibrationError as exc:
                            operator_message = f"记录失败：{exc}"
                            continue
                        operator_message = f"已记录不合格位置 {sample.sample_id}"
                        gripper_event = f"bad_pose:{sample.sample_id}"
                    elif in_manual and event.key == pygame.K_c:
                        if calibration is None or calibration.pending_sample is None:
                            operator_message = "闭爪被拒绝：请先按 Enter/G 保存位置"
                            continue
                        if not robot.allow_gripper_actuation:
                            operator_message = (
                                "闭爪被拒绝：先完成候选档案实测并开启机械爪权限"
                            )
                            continue
                        manual_control.clear()
                        node.publish(MotionCommand.neutral())
                        accepted, acknowledgement = node.command_gripper("close")
                        calibration.record_gripper_result(
                            "close",
                            accepted=accepted,
                            acknowledgement=acknowledgement,
                        )
                        operator_message = (
                            "闭爪命令已被网关接受；观察实物后按 Y/N"
                            if accepted
                            else f"闭爪被网关拒绝：{acknowledgement}"
                        )
                        gripper_event = f"close:{acknowledgement}"
                    elif in_manual and event.key == pygame.K_o:
                        if not robot.allow_gripper_actuation:
                            operator_message = (
                                "开爪被拒绝：先完成候选档案实测并开启机械爪权限"
                            )
                            continue
                        manual_control.clear()
                        node.publish(MotionCommand.neutral())
                        accepted, acknowledgement = node.command_gripper("open")
                        operator_message = (
                            "开爪命令已被网关接受"
                            if accepted
                            else f"开爪被网关拒绝：{acknowledgement}"
                        )
                        gripper_event = f"open:{acknowledgement}"
                    elif in_manual and event.key in {pygame.K_y, pygame.K_n}:
                        if calibration is None:
                            operator_message = "结果记录失败：标定会话未初始化"
                            continue
                        outcome_name = "success" if event.key == pygame.K_y else "failure"
                        try:
                            sample = calibration.mark_grasp_outcome(outcome_name)
                        except GraspCalibrationError as exc:
                            operator_message = f"结果记录被拒绝：{exc}"
                            continue
                        operator_message = (
                            f"样本 {sample.sample_id} 已标记 "
                            f"{'SUCCESS' if outcome_name == 'success' else 'FAILURE'}"
                        )
                        gripper_event = f"outcome:{outcome_name}"
                    elif in_manual and event.key == pygame.K_r:
                        if calibration is not None and calibration.pending_sample is not None:
                            operator_message = "重新搜索被拒绝：请先用 Y/N 完成当前样本"
                            continue
                        manual_control.clear()
                        decision = mission.request_rescan(
                            node.observation(), time.monotonic()
                        )
                        operator_message = "正在回到本轮搜索深度，随后重新扫描"
                if event.type == pygame.KEYUP:
                    key_name = _event_key_name(pygame, event.key)
                    if key_name is not None:
                        manual_control.release(key_name)

            node.spin(0.0)
            error = _active_error(node, dataset, allow_gripper=manual_mode)
            if error is not None:
                raise DatasetDriveError(error)
            if node.perception_age_s() > MAXIMUM_PERCEPTION_AGE_S:
                raise DatasetDriveError("超过 1.0 秒没有新鲜检测帧")
            if not recorder.is_stream_fresh(maximum_idle_s=3.0):
                raise DatasetDriveError("原始视频录像停止增长")
            observation = node.observation()
            now = time.monotonic()
            # 人工精调也不能在停帧时沿用旧画面继续走。
            # 状态机稍后会消费这个帧号，因此必须先记下本轮
            # 是否真的收到新检测帧。
            new_perception_frame = observation.frame_id > mission.last_frame_id
            if observation.frame_id != last_logged_frame:
                logger.write_detections(observation)
                last_logged_frame = observation.frame_id

            if normal_finish_requested and decision.state != SearchTestState.RETURNING:
                manual_control.clear()
                decision = mission.request_normal_finish(
                    observation, now, "操作员按 0 正常结束"
                )
            elif paused:
                # 暂停时不让状态机的扫描/前进继续消耗时间，只发中位。
                decision = SearchTestDecision(
                    state=decision.state,
                    motion=MotionCommand.neutral(),
                    message="窗口失去焦点，已暂停；回到窗口后按 Enter 恢复",
                    selected_target=decision.selected_target,
                    target_area_ratio=decision.target_area_ratio,
                    horizontal_error=decision.horizontal_error,
                    current_depth_m=observation.depth_m,
                    target_depth_m=decision.target_depth_m,
                    scan_progress_deg=decision.scan_progress_deg,
                    search_cycle=decision.search_cycle,
                    outcome=decision.outcome,
                )
            else:
                decision = mission.step(observation, now)

            if decision.state != SearchTestState.MANUAL_CALIBRATION:
                manual_control.clear()
            actual_motion = decision.motion
            if (
                manual_mode
                and decision.state == SearchTestState.MANUAL_CALIBRATION
                and not paused
                and observation.perception_valid
                and new_perception_frame
            ):
                actual_motion = manual_control.motion()
                if not actual_motion.is_neutral():
                    last_manual_motion = actual_motion
            if last_state != decision.state:
                operator_message = decision.message
                last_state = decision.state

            if now >= next_publish:
                node.publish(actual_motion)
                published_decision = replace(decision, motion=actual_motion)
                node.publish_mission_status(published_decision, observation)
                logger.write_decision(
                    published_decision,
                    observation,
                    motion=actual_motion,
                    manual_power=(manual_control.strength if manual_mode else None),
                    gripper_event=gripper_event,
                )
                gripper_event = ""
                next_publish = now + 0.05
            _draw_window(
                pygame,
                screen,
                font,
                decision,
                workflow=workflow,
                paused=paused,
                perception_age_s=node.perception_age_s(),
                manual_control=manual_control,
                calibration=calibration,
                gripper_profile=robot.gripper.profile,
                gripper_enabled=robot.allow_gripper_actuation,
                operator_message=operator_message,
            )
            if decision.state == SearchTestState.ABORTED:
                raise DatasetDriveError(decision.message)
            if decision.state == SearchTestState.COMPLETE:
                break
            clock.tick(60)

        node.publish_neutral()
        video_path, recording_error = recorder.finalize(
            pump=lambda: _publish_neutral_with_health_check(
                node,
                dataset,
                allow_gripper=manual_mode,
            )
        )
        if recording_error is not None:
            raise DatasetDriveError(f"录像封装失败: {recording_error}")
        disarm_error = node.request_normal_disarm(timeout_s=5.0)
        if disarm_error is not None:
            raise DatasetDriveError(f"正常上锁未确认: {disarm_error}")
        _wait_for_gateway_state(
            node, lambda: bool(node.status and not node.status.armed), "上锁"
        )
        normal_complete = True
        control_opened = False
        outcome = "completed"
        detail = decision.message
        result_code = 0
    except KeyboardInterrupt:
        outcome = "emergency_stopped"
        detail = "Ctrl+C 急停"
        result_code = 130
    except (DatasetDriveError, SearchTestError) as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = str(exc)
        print(f"\n搜索测试中止: {detail}")
        result_code = 2
    except GraspCalibrationError as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = str(exc)
        print(f"\n抓取位置标定中止: {detail}")
        result_code = 2
    except Exception as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = f"未预见异常 {type(exc).__name__}: {exc}"
        try:
            trace_path = session_dir / "logs/search_test_traceback.log"
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_path.write_text(traceback.format_exc(), encoding="utf-8")
        except OSError:
            pass
        print(f"\n搜索测试中止: {detail}")
        result_code = 2
    finally:
        if node is not None and rclpy.ok() and control_opened and not normal_complete:
            try:
                node.publish_neutral()
            except Exception:
                pass
            estop_error = node.emergency_stop()
            if estop_error is not None:
                detail += f"；急停服务未确认: {estop_error}"
                print("无法确认软件急停，立即用 QGC 上锁或物理断电。")
            control_opened = False
        if recorder is not None and video_path is None:
            finalized, recording_error = recorder.finalize()
            video_path = finalized
            if recording_error is not None:
                detail += f"；录像封装: {recording_error}"
        end_depth = _current_depth(node) if node is not None else None
        if calibration is not None:
            calibration.finish()
        if logger is not None:
            logger.finish(
                outcome=outcome,
                detail=detail,
                end_depth_m=end_depth,
                video_path=video_path,
                grasp_sample_counts=(
                    calibration.counts() if calibration is not None else None
                ),
            )
        if pygame_started:
            pygame.quit()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"会话目录: {session_dir}")
    if normal_complete:
        label = "抓取位置标定" if manual_mode else "搜索—接近测试"
        print(f"{label}完成，ROV 已回到启动深度附近并正常上锁。")
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
