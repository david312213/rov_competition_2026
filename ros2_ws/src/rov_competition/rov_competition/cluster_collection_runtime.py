"""ROS 2 群体盲抓与跳跃式离底搜索运行时。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
import traceback
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rov_interfaces.msg import MissionStatus
from sensor_msgs.msg import CompressedImage

from .cluster_collection import (
    ClusterCollectionConfig,
    ClusterCollectionDecision,
    ClusterCollectionError,
    ClusterCollectionMission,
    ClusterCollectionState,
    ClusterStateReporter,
    cluster_scallops,
    load_cluster_collection_config,
)
from .config import (
    ConfigurationError,
    ControlProfile,
    load_autonomy_config,
    load_dataset_config,
    load_robot_config,
)
from .dataset_drive import (
    DatasetDriveError,
    _active_error,
    _current_depth,
    _prearm_error,
    _publish_neutral_with_health_check,
    _wait_for_gateway_state,
)
from .dataset_recording import RtpMkvRecorder, _git_commit, _sha256
from .domain import MissionObservation, MotionCommand
from .safety import PerceptionFreshness, classify_perception_age
from .search_approach_runtime import (
    ARM_CONFIRMATION,
    SearchApproachNode,
    _package_config_path,
    _prompt_float,
)


CONFIRMATION = "START CLUSTER COLLECTION"
SOURCE = "commissioning"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROV 群体盲抓与跳跃式离底搜索测试"
    )
    parser.add_argument("--robot-config", default=_package_config_path("robot.example.yaml"))
    parser.add_argument("--dataset-config", default=_package_config_path("dataset.example.yaml"))
    parser.add_argument("--autonomy-config", default=_package_config_path("autonomy.yaml"))
    parser.add_argument(
        "--cluster-config", default=_package_config_path("cluster_collection.yaml")
    )
    parser.add_argument("--session-dir")
    parser.add_argument("--project-dir", default=str(Path.cwd()))
    parser.add_argument("--record-port", type=int, default=5704)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--execute", action="store_true")
    return parser


class ClusterSessionLogger:
    """保存运动、原始检测、群体和机械爪尝试证据。"""

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
        "visible_scallop_count",
        "locked_cluster_count",
        "cluster_center_x_ratio",
        "cluster_center_y_ratio",
        "cluster_union_area_ratio",
        "grasp_attempt_index",
        "completed_cluster_count",
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
    )
    CLUSTER_FIELDS = (
        "utc_time",
        "frame_id",
        "state",
        "cluster_index",
        "locked",
        "visible_scallop_count",
        "locked_cluster_count",
        "center_x_ratio",
        "center_y_ratio",
        "union_area_ratio",
        "total_confidence",
        "descent_trigger",
    )
    GRASP_FIELDS = (
        "utc_time",
        "monotonic_s",
        "attempt_index",
        "completed_cluster_count",
        "action",
        "accepted",
        "acknowledgement",
        "depth_m",
        "center_x_ratio",
        "center_y_ratio",
        "union_area_ratio",
        "note",
    )

    def __init__(
        self,
        directory: Path,
        *,
        project_directory: Path,
        robot_config: Path,
        autonomy_config: Path,
        cluster_config: Path,
        gripper_profile: str,
    ) -> None:
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "logs").mkdir(exist_ok=True)
        self.session_path = directory / "session.json"
        self._files = {
            "events": (directory / "events.csv").open("w", encoding="utf-8", newline=""),
            "detections": (directory / "detections.csv").open("w", encoding="utf-8", newline=""),
            "clusters": (directory / "clusters.csv").open("w", encoding="utf-8", newline=""),
            "grasps": (directory / "grasp_attempts.csv").open("w", encoding="utf-8", newline=""),
        }
        self._writers = {
            "events": csv.DictWriter(self._files["events"], fieldnames=self.EVENT_FIELDS),
            "detections": csv.DictWriter(
                self._files["detections"], fieldnames=self.DETECTION_FIELDS
            ),
            "clusters": csv.DictWriter(self._files["clusters"], fieldnames=self.CLUSTER_FIELDS),
            "grasps": csv.DictWriter(self._files["grasps"], fieldnames=self.GRASP_FIELDS),
        }
        for writer in self._writers.values():
            writer.writeheader()
        self.metadata: dict[str, object] = {
            "mode": "cluster_collection_test",
            "version": "0.2.0rc2",
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "finished_at_utc": None,
            "outcome": "starting",
            "detail": "",
            "git_commit": _git_commit(project_directory),
            "robot_config": str(robot_config),
            "robot_config_sha256": _sha256(robot_config),
            "autonomy_config": str(autonomy_config),
            "autonomy_config_sha256": _sha256(autonomy_config),
            "cluster_config": str(cluster_config),
            "cluster_config_sha256": _sha256(cluster_config),
            "gripper_profile": gripper_profile,
            "start_depth_m": None,
            "descent_command": None,
            "completed_cluster_count": 0,
            "grasp_attempt_count": 0,
            "video_file": None,
            "limitations": [
                "pressure depth is not altitude above bottom",
                "short forward moves have no terrain-ranging feedback",
                "accepted gripper command is not physical grasp confirmation",
                "TRANSFER_TO_NET is a timed no-op placeholder",
            ],
        }
        self._write_metadata()

    def set_started(self, start_depth_m: float, descent_command: float) -> None:
        self.metadata.update(
            {"start_depth_m": start_depth_m, "descent_command": descent_command}
        )
        self._write_metadata()

    def write_decision(
        self,
        decision: ClusterCollectionDecision,
        observation: MissionObservation,
        *,
        gripper_event: str = "",
    ) -> None:
        cluster = decision.cluster
        self._writers["events"].writerow(
            {
                "utc_time": datetime.now(timezone.utc).isoformat(),
                "monotonic_s": f"{time.monotonic():.6f}",
                "state": decision.state.value,
                "message": decision.message,
                "frame_id": observation.frame_id,
                "forward": f"{decision.motion.forward:.6f}",
                "lateral": f"{decision.motion.lateral:.6f}",
                "vertical": f"{decision.motion.vertical:.6f}",
                "yaw": f"{decision.motion.yaw:.6f}",
                "depth_m": f"{observation.depth_m:.6f}",
                "yaw_deg": f"{observation.yaw_deg:.6f}",
                "visible_scallop_count": decision.visible_scallop_count,
                "locked_cluster_count": 0 if cluster is None else cluster.count,
                "cluster_center_x_ratio": "" if cluster is None else f"{cluster.center_x_ratio:.8f}",
                "cluster_center_y_ratio": "" if cluster is None else f"{cluster.center_y_ratio:.8f}",
                "cluster_union_area_ratio": "" if cluster is None else f"{cluster.union_area_ratio:.8f}",
                "grasp_attempt_index": decision.grasp_attempt_index,
                "completed_cluster_count": decision.completed_cluster_count,
                "gripper_event": gripper_event,
            }
        )
        self._files["events"].flush()

    def write_frame(
        self,
        decision: ClusterCollectionDecision,
        observation: MissionObservation,
        *,
        config: ClusterCollectionConfig,
    ) -> None:
        utc = datetime.now(timezone.utc).isoformat()
        if not observation.detections:
            self._writers["detections"].writerow(
                {"utc_time": utc, "frame_id": observation.frame_id, "label": ""}
            )
        for item in observation.detections:
            self._writers["detections"].writerow(
                {
                    "utc_time": utc,
                    "frame_id": observation.frame_id,
                    "label": item.label,
                    "confidence": f"{item.confidence:.8f}",
                    "left": f"{item.box.left:.3f}",
                    "top": f"{item.box.top:.3f}",
                    "right": f"{item.box.right:.3f}",
                    "bottom": f"{item.box.bottom:.3f}",
                }
            )
        clusters = cluster_scallops(
            observation.detections,
            observation.frame_width,
            observation.frame_height,
            label=config.target_label,
            link_distance_ratio=config.cluster_link_distance_ratio,
        )
        locked_index: int | None = None
        if decision.cluster is not None and clusters:
            locked_index = min(
                range(len(clusters)),
                key=lambda index: math.hypot(
                    clusters[index].center_x_ratio
                    - decision.cluster.center_x_ratio,
                    clusters[index].center_y_ratio
                    - decision.cluster.center_y_ratio,
                ),
            )
        rows = list(enumerate(clusters)) or [(None, None)]
        for index, cluster in rows:
            self._writers["clusters"].writerow(
                {
                    "utc_time": utc,
                    "frame_id": observation.frame_id,
                    "state": decision.state.value,
                    "cluster_index": "" if index is None else index,
                    "locked": bool(index is not None and index == locked_index),
                    "visible_scallop_count": decision.visible_scallop_count,
                    "locked_cluster_count": (
                        0 if decision.cluster is None else decision.cluster.count
                    ),
                    "center_x_ratio": (
                        "" if cluster is None else f"{cluster.center_x_ratio:.8f}"
                    ),
                    "center_y_ratio": (
                        "" if cluster is None else f"{cluster.center_y_ratio:.8f}"
                    ),
                    "union_area_ratio": (
                        "" if cluster is None else f"{cluster.union_area_ratio:.8f}"
                    ),
                    "total_confidence": (
                        "" if cluster is None else f"{cluster.total_confidence:.8f}"
                    ),
                    "descent_trigger": decision.descent_trigger_mode,
                }
            )
        self._files["detections"].flush()
        self._files["clusters"].flush()

    def write_gripper(
        self,
        decision: ClusterCollectionDecision,
        observation: MissionObservation,
        *,
        action: str,
        accepted: bool,
        acknowledgement: str,
    ) -> None:
        cluster = decision.cluster
        self._writers["grasps"].writerow(
            {
                "utc_time": datetime.now(timezone.utc).isoformat(),
                "monotonic_s": f"{time.monotonic():.6f}",
                "attempt_index": decision.grasp_attempt_index,
                "completed_cluster_count": decision.completed_cluster_count,
                "action": action,
                "accepted": accepted,
                "acknowledgement": acknowledgement,
                "depth_m": f"{observation.depth_m:.6f}",
                "center_x_ratio": "" if cluster is None else f"{cluster.center_x_ratio:.8f}",
                "center_y_ratio": "" if cluster is None else f"{cluster.center_y_ratio:.8f}",
                "union_area_ratio": "" if cluster is None else f"{cluster.union_area_ratio:.8f}",
                "note": "ACK only; physical grasp success is not inferred",
            }
        )
        self._files["grasps"].flush()

    def finish(
        self,
        *,
        outcome: str,
        detail: str,
        end_depth_m: float | None,
        video_path: Path | None,
        completed_cluster_count: int,
        grasp_attempt_count: int,
    ) -> None:
        if self._files["events"].closed:
            return
        self.metadata.update(
            {
                "finished_at_utc": datetime.now(timezone.utc).isoformat(),
                "outcome": outcome,
                "detail": detail,
                "end_depth_m": end_depth_m,
                "video_file": None if video_path is None else str(video_path),
                "completed_cluster_count": completed_cluster_count,
                "grasp_attempt_count": grasp_attempt_count,
            }
        )
        self._write_metadata()
        for stream in self._files.values():
            stream.close()

    def _write_metadata(self) -> None:
        temporary = self.session_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.session_path)


class ClusterCollectionNode(SearchApproachNode):
    """为群体测试增加状态字段和独立带群中心画面。"""

    def __init__(self, config: ClusterCollectionConfig) -> None:
        super().__init__(capture_images=True, node_name="rov_cluster_collection_test")
        self.cluster_config = config
        self.cluster_image_publisher = self.create_publisher(
            CompressedImage, "/rov/cluster_image/compressed", 2
        )
        self._last_overlay_stamp = -1.0
        self._last_overlay_at = 0.0

    def publish_cluster_status(
        self, decision: ClusterCollectionDecision, observation: MissionObservation
    ) -> None:
        message = MissionStatus()
        message.stamp = self.get_clock().now().to_msg()
        message.active = decision.state not in {
            ClusterCollectionState.IDLE,
            ClusterCollectionState.COMPLETE,
            ClusterCollectionState.ABORTED,
        }
        message.state = decision.state.value
        message.outcome = decision.outcome
        message.message = decision.message
        message.frame_id = int(observation.frame_id)
        message.valid_depth = bool(observation.depth_valid)
        message.current_depth_m = float(observation.depth_m)
        message.target_depth_m = float(decision.target_depth_m or 0.0)
        message.valid_scan_progress = decision.scan_progress_deg is not None
        message.scan_progress_deg = float(decision.scan_progress_deg or 0.0)
        message.search_cycle = int(decision.search_cycle)
        message.gripper_command_accepted = bool(decision.gripper_command_accepted)
        cluster = decision.cluster
        message.has_cluster = cluster is not None
        message.visible_scallop_count = int(decision.visible_scallop_count)
        message.locked_cluster_count = 0 if cluster is None else int(cluster.count)
        message.cluster_center_x_ratio = 0.0 if cluster is None else float(cluster.center_x_ratio)
        message.cluster_center_y_ratio = 0.0 if cluster is None else float(cluster.center_y_ratio)
        message.cluster_union_area_ratio = 0.0 if cluster is None else float(cluster.union_area_ratio)
        message.grasp_attempt_index = int(decision.grasp_attempt_index)
        message.completed_cluster_count = int(decision.completed_cluster_count)
        message.descent_trigger_mode = decision.descent_trigger_mode
        self.mission_status_publisher.publish(message)

    def publish_cluster_overlay(self, decision: ClusterCollectionDecision) -> None:
        """在原有 YOLO 带框 JPEG 上叠加锁定群、群中心和抓取线。"""

        if not self.images:
            return
        snapshot = self.images[-1]
        now = time.monotonic()
        if snapshot.stamp_s == self._last_overlay_stamp or now - self._last_overlay_at < 0.08:
            return
        try:
            import cv2
            import numpy as np

            frame = cv2.imdecode(
                np.frombuffer(snapshot.jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                return
            height, width = frame.shape[:2]
            line_y = int(self.cluster_config.descent_line_y_ratio * height)
            cv2.line(frame, (0, line_y), (width - 1, line_y), (255, 0, 255), 2)
            cv2.putText(
                frame,
                "CLUSTER DESCENT LINE",
                (12, max(22, line_y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cluster = decision.cluster
            if cluster is not None:
                for item in cluster.detections:
                    cv2.rectangle(
                        frame,
                        (int(item.box.left), int(item.box.top)),
                        (int(item.box.right), int(item.box.bottom)),
                        (0, 165, 255),
                        4,
                    )
                center = (
                    int(cluster.center_x_ratio * width),
                    int(cluster.center_y_ratio * height),
                )
                cv2.drawMarker(
                    frame,
                    center,
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    36,
                    3,
                )
            cv2.putText(
                frame,
                (
                    f"CLUSTER state={decision.state.value} visible="
                    f"{decision.visible_scallop_count} locked="
                    f"{0 if cluster is None else cluster.count} grasp="
                    f"{decision.grasp_attempt_index}/3 groups="
                    f"{decision.completed_cluster_count}"
                ),
                (12, height - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
            message = CompressedImage()
            message.header.stamp = self.get_clock().now().to_msg()
            message.format = "jpeg"
            message.data = encoded.tobytes()
            self.cluster_image_publisher.publish(message)
            self._last_overlay_stamp = snapshot.stamp_s
            self._last_overlay_at = now
        except Exception as exc:
            self.get_logger().warning(f"群体带框画面发布失败: {exc}")


def _draw_window(
    pygame: object,
    screen: object,
    font: object,
    decision: ClusterCollectionDecision,
    *,
    paused: bool,
    perception_age_s: float,
) -> None:
    screen.fill((17, 24, 39))
    cluster = decision.cluster
    lines = [
        "ROV CLUSTER COLLECTION TEST",
        "SPACE neutral/pause | ENTER resume | 0 return+disarm | ESC/close emergency stop",
        (
            f"state={decision.state.value} paused={paused} "
            f"frame_age={perception_age_s:.2f}s"
        ),
        (
            f"scallops={decision.visible_scallop_count} "
            f"locked={0 if cluster is None else cluster.count} "
            f"grasp={decision.grasp_attempt_index}/3 "
            f"groups={decision.completed_cluster_count}"
        ),
        (
            "cluster=none"
            if cluster is None
            else (
                f"center=({cluster.center_x_ratio:.3f}, {cluster.center_y_ratio:.3f}) "
                f"union={cluster.union_area_ratio:.4f} trigger={decision.descent_trigger_mode}"
            )
        ),
        decision.message[:135],
    ]
    for index, line in enumerate(lines):
        color = (255, 205, 80) if index == len(lines) - 1 else (235, 245, 255)
        screen.blit(font.render(line, True, color), (20, 18 + index * 38))
    pygame.display.flip()


def _interactive_parameters(
    node: ClusterCollectionNode,
    config: ClusterCollectionConfig,
    command_limit: float,
) -> tuple[float, float]:
    if not sys.stdin.isatty():
        raise DatasetDriveError("真实群体收集测试必须在交互终端运行")
    observation = node.observation()
    if not observation.depth_valid or not math.isfinite(observation.depth_m):
        raise DatasetDriveError("启动前深度反馈无效")
    ceiling = min(0.80, command_limit)
    descent = _prompt_float("触底固定下潜 power ", min(0.60, ceiling), 0.10, ceiling)
    print("\n本次群体收集参数：")
    print(f"  启动深度：{observation.depth_m:.2f} m")
    print(f"  触底下潜 power：{descent:.2f}")
    print(f"  每次触底后上浮：{config.bottom_clearance_m:.2f} m")
    print(f"  群体门槛：{config.minimum_cluster_count} 个 scallop，最近 5 帧至少 3 帧")
    print(f"  下降判据：{config.descent_trigger.value}")
    print(f"  每群机械爪动作：{config.grabs_per_cluster} 次")
    print("  TRANSFER_TO_NET 当前只等待和记录，没有网兜机械动作。")
    print("  机械爪 ACK 只表示飞控接受命令，不代表已真实抓到扇贝。")
    print("\n必须确认：ROV 已浸没，危险区无人，ALT_HOLD，")
    print("QGC 有实时遥测/5600 画面/人工上锁，RST 爪已实艇开闭验证。")
    typed = input(f"全部满足后完整输入 {CONFIRMATION!r}: ").strip()
    if typed != CONFIRMATION:
        raise DatasetDriveError("确认词不匹配，未开启控制")
    return descent, float(observation.depth_m)


def main(argv: list[str] | None = None) -> int:
    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        robot = load_robot_config(args.robot_config)
        dataset = load_dataset_config(args.dataset_config)
        autonomy = load_autonomy_config(args.autonomy_config)
        cluster_config = load_cluster_collection_config(args.cluster_config)
    except (ConfigurationError, ClusterCollectionError, ValueError) as exc:
        print(f"配置错误: {exc}")
        return 2

    errors = list(
        cluster_config.readiness_errors(robot_command_limit=robot.command_limit)
    )
    if robot.control_profile != ControlProfile.COMMISSIONING:
        errors.append("robot.yaml 的 profile 必须是 commissioning")
    if "ALT_HOLD" not in robot.allowed_flight_modes:
        errors.append("robot.yaml 必须允许 ALT_HOLD")
    if not robot.allow_live_actuation:
        errors.append("robot.yaml 尚未允许真实输出")
    if not robot.allow_ros_arming:
        errors.append("robot.yaml 尚未允许 ROS 解锁")
    if not robot.allow_gripper_actuation:
        errors.append("robot.yaml 尚未允许机械爪输出")
    if robot.gripper.profile != "rst":
        errors.append("群体收集只允许已实测的 rst 机械爪档案")
    if not robot.gripper.calibrated:
        errors.append("rst 机械爪档案尚未标记为实艇标定通过")
    if cluster_config.target_label not in autonomy.detector.class_names:
        errors.append(
            f"模型类别不包含 {cluster_config.target_label!r}"
        )
    if errors:
        print("禁止真实群体收集测试：")
        for error in dict.fromkeys(errors):
            print(f"  - {error}")
        return 2
    if not args.execute:
        print("群体收集配置预览通过；未初始化 ROS、未解锁、未动爪。")
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
    logger: ClusterSessionLogger | None = None
    recorder: RtpMkvRecorder | None = None
    node: ClusterCollectionNode | None = None
    mission: ClusterCollectionMission | None = None
    pygame_started = False
    control_opened = False
    normal_complete = False
    gripper_failure_recovered = False
    video_path: Path | None = None
    outcome = "failed_before_arm"
    detail = "启动未完成"
    result_code = 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ClusterCollectionNode(cluster_config)
        node.wait_for_initial_data(timeout_s=10.0)
        node.wait_for_perception(timeout_s=20.0)
        error = _prearm_error(node, dataset, require_disarmed=True, allow_gripper=True)
        if error is not None:
            raise DatasetDriveError(error)

        pygame.init()
        pygame_started = True
        screen = pygame.display.set_mode((1180, 285))
        pygame.display.set_caption("ROV Cluster Collection Test")
        font = pygame.font.Font(None, 27)

        logger = ClusterSessionLogger(
            session_dir,
            project_directory=Path(args.project_dir).expanduser().resolve(),
            robot_config=Path(args.robot_config).expanduser().resolve(),
            autonomy_config=Path(args.autonomy_config).expanduser().resolve(),
            cluster_config=Path(args.cluster_config).expanduser().resolve(),
            gripper_profile=robot.gripper.profile,
        )
        recorder = RtpMkvRecorder(
            session_dir, source_port=args.record_port, payload_type=args.payload_type
        )
        recorder.start()
        recorder.wait_until_receiving(timeout_s=10.0, pump=lambda: node.spin(0.0))
        descent_power, start_depth = _interactive_parameters(
            node, cluster_config, robot.command_limit
        )
        selected_errors = cluster_config.readiness_errors(
            robot_command_limit=robot.command_limit,
            descent_command=descent_power,
        )
        if selected_errors:
            raise DatasetDriveError("; ".join(selected_errors))
        logger.set_started(start_depth, descent_power)

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
        error = _active_error(node, dataset, allow_gripper=True)
        if error is not None:
            raise DatasetDriveError(error)

        mission = ClusterCollectionMission(cluster_config)
        observation = node.observation()
        started_at = time.monotonic()
        decision = mission.start(
            observation, descent_command=descent_power, now=started_at
        )
        reporter = ClusterStateReporter()
        reporter.report(decision, observation, started_at)
        node.publish(decision.motion)
        node.publish_cluster_status(decision, observation)
        logger.write_decision(decision, observation)
        last_logged_frame = -1
        paused = False
        pause_started_at: float | None = None
        perception_hold_started_at: float | None = None
        next_publish = time.monotonic()
        gripper_event = ""
        clock = pygame.time.Clock()

        while rclpy.ok():
            normal_finish_requested = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise DatasetDriveError("控制窗口被关闭")
                if event.type in {getattr(pygame, "WINDOWFOCUSLOST", -1)} or (
                    event.type == pygame.ACTIVEEVENT and getattr(event, "gain", 1) == 0
                ):
                    if not paused:
                        pause_started_at = time.monotonic()
                    paused = True
                    node.publish(MotionCommand.neutral())
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        raise DatasetDriveError("Esc 急停")
                    if event.key in {pygame.K_0, pygame.K_KP0}:
                        normal_finish_requested = True
                    if event.key == pygame.K_SPACE:
                        if not paused:
                            pause_started_at = time.monotonic()
                        paused = True
                        node.publish(MotionCommand.neutral())
                    if event.key in {pygame.K_RETURN, pygame.K_KP_ENTER} and paused:
                        if node.perception_age_s() <= cluster_config.perception_hold_timeout_s:
                            if pause_started_at is not None:
                                mission.delay_timers(time.monotonic() - pause_started_at)
                            paused = False
                            pause_started_at = None

            node.spin(0.0)
            error = _active_error(node, dataset, allow_gripper=True)
            if error is not None:
                raise DatasetDriveError(error)
            if not recorder.is_stream_fresh(maximum_idle_s=5.0):
                raise DatasetDriveError("原始视频录像停止增长")
            perception_age = node.perception_age_s()
            perception_state = classify_perception_age(
                perception_age,
                hold_timeout_s=cluster_config.perception_hold_timeout_s,
                abort_timeout_s=cluster_config.perception_abort_timeout_s,
            )
            if perception_state == PerceptionFreshness.ABORT:
                raise DatasetDriveError(
                    f"超过 {cluster_config.perception_abort_timeout_s:.1f}s "
                    "没有新鲜检测帧"
                )
            observation = node.observation()
            now = time.monotonic()
            if now - started_at >= cluster_config.mission_timeout_s:
                normal_finish_requested = True
            perception_holding = perception_state == PerceptionFreshness.HOLD
            if perception_holding and not paused:
                if perception_hold_started_at is None:
                    perception_hold_started_at = now
            elif perception_hold_started_at is not None:
                mission.delay_timers(now - perception_hold_started_at)
                perception_hold_started_at = None

            log_new_frame = observation.frame_id != last_logged_frame
            if normal_finish_requested and decision.state != ClusterCollectionState.RETURNING:
                decision = mission.request_normal_finish(
                    observation,
                    now,
                    "15 分钟时限已到"
                    if now - started_at >= cluster_config.mission_timeout_s
                    else "操作员按 0 正常结束",
                )
            elif paused:
                decision = replace(
                    decision,
                    motion=MotionCommand.neutral(),
                    message="Space/窗口失焦已回中暂停；回到窗口后按 Enter 恢复",
                    current_depth_m=observation.depth_m,
                )
            elif perception_holding:
                decision = replace(
                    decision,
                    motion=MotionCommand.neutral(),
                    message=f"检测帧暂停 {perception_age:.2f}s，已回中等待",
                    current_depth_m=observation.depth_m,
                )
            else:
                decision = mission.step(observation, now)

            if decision.gripper_action is not None:
                logical_action = decision.gripper_action
                service_action = "open" if logical_action == "reopen" else logical_action
                node.publish(MotionCommand.neutral())
                try:
                    accepted, acknowledgement = node.command_gripper(service_action)
                except DatasetDriveError as exc:
                    logger.write_gripper(
                        decision,
                        observation,
                        action=logical_action,
                        accepted=False,
                        acknowledgement=str(exc),
                    )
                    failure_observation = node.observation()
                    if not (
                        failure_observation.depth_valid
                        and math.isfinite(failure_observation.depth_m)
                    ):
                        raise DatasetDriveError(
                            f"{exc}；当前深度反馈无效，不能执行盲目回收"
                        ) from exc
                    gripper_event = f"{logical_action}:failed:{exc}"
                    decision = mission.request_gripper_failure_recovery(
                        failure_observation,
                        time.monotonic(),
                        str(exc),
                    )
                    gripper_failure_recovered = True
                    observation = failure_observation
                    accepted = False
                    acknowledgement = str(exc)
                else:
                    logger.write_gripper(
                        decision,
                        observation,
                        action=logical_action,
                        accepted=accepted,
                        acknowledgement=acknowledgement,
                    )
                    gripper_event = f"{logical_action}:{acknowledgement}"
                    acknowledgement_observation = node.observation()
                    if not accepted and not (
                        acknowledgement_observation.depth_valid
                        and math.isfinite(acknowledgement_observation.depth_m)
                    ):
                        raise DatasetDriveError(
                            f"机械爪 {logical_action} 被拒绝且当前深度反馈无效，"
                            "不能执行盲目回收"
                        )
                    decision = mission.acknowledge_gripper(
                        logical_action,
                        accepted,
                        acknowledgement,
                        acknowledgement_observation,
                        time.monotonic(),
                    )
                    observation = acknowledgement_observation
                    if not accepted:
                        gripper_failure_recovered = True

            # 先让状态机消费这张新帧，再把检测和它导致的群体状态
            # 写在同一行记录中，避免 clusters.csv 落后一帧。
            if log_new_frame:
                logger.write_frame(decision, observation, config=cluster_config)
                last_logged_frame = observation.frame_id

            reporter.report(decision, observation, now)
            if now >= next_publish:
                node.publish(decision.motion)
                node.publish_cluster_status(decision, observation)
                logger.write_decision(
                    decision, observation, gripper_event=gripper_event
                )
                gripper_event = ""
                next_publish = now + 0.05
            node.publish_cluster_overlay(decision)
            _draw_window(
                pygame,
                screen,
                font,
                decision,
                paused=paused,
                perception_age_s=perception_age,
            )
            if decision.state == ClusterCollectionState.ABORTED:
                raise DatasetDriveError(decision.message)
            if decision.state == ClusterCollectionState.COMPLETE:
                break
            clock.tick(60)

        node.publish_neutral()
        video_path, recording_error = recorder.finalize(
            pump=lambda: _publish_neutral_with_health_check(
                node, dataset, allow_gripper=True
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
        if gripper_failure_recovered:
            outcome = "gripper_failed_recovered"
            detail = mission.gripper_failure_reason or decision.message
            result_code = 2
        else:
            outcome = "completed"
            detail = decision.message
            result_code = 0
    except KeyboardInterrupt:
        outcome = "emergency_stopped"
        detail = "Ctrl+C 急停"
        result_code = 130
    except (DatasetDriveError, ClusterCollectionError) as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = str(exc)
        print(f"\n[群体收集中止] {detail}", flush=True)
        result_code = 2
    except Exception as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = f"未预见异常 {type(exc).__name__}: {exc}"
        try:
            trace_path = session_dir / "logs/cluster_collection_traceback.log"
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_path.write_text(traceback.format_exc(), encoding="utf-8")
        except OSError:
            pass
        print(f"\n[群体收集中止] {detail}", flush=True)
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
        if recorder is not None and video_path is None:
            video_path, recording_error = recorder.finalize()
            if recording_error is not None:
                detail += f"；录像封装: {recording_error}"
        if logger is not None:
            logger.finish(
                outcome=outcome,
                detail=detail,
                end_depth_m=_current_depth(node) if node is not None else None,
                video_path=video_path,
                completed_cluster_count=(
                    0 if mission is None else mission.completed_cluster_count
                ),
                grasp_attempt_count=(
                    0 if mission is None else mission.total_grasp_attempt_count
                ),
            )
        if pygame_started:
            pygame.quit()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"会话目录: {session_dir}", flush=True)
    if normal_complete:
        if gripper_failure_recovered:
            print("机械爪故障后已正常回收并上锁；本次任务未成功。", flush=True)
        else:
            print("群体收集测试已正常回收并上锁。", flush=True)
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
