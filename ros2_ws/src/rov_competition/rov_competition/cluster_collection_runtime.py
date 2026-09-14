"""ROS 2 群体盲抓与跳跃式离底搜索运行时。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
import traceback
import uuid
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rov_interfaces.msg import MissionStatus, GripperExecutionStatus
from rov_interfaces.srv import SetGripper
from .target_grasp import TargetGraspMission

from .cluster_collection import (
    ClusterCollectionConfig,
    ClusterCollectionDecision,
    ClusterCollectionError,
    ClusterCollectionMission,
    ClusterCollectionState,
    ClusterStateReporter,
    DescentTrigger,
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
from .cluster_runtime_support import LoopTiming
from .cluster_window_bridge import ClusterWindowBridge, WindowGate
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
    parser.add_argument(
        "--stop-after-approach",
        action="store_true",
        help="达到群体接近判据后只回中、上锁，不发送机械爪或盲抓下降命令",
    )
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
        "selected_target", "target_bottom_ratio", "descent_vertical_only",
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
        stop_after_approach: bool = False,
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
            "mode": (
                "cluster_search_approach_no_gripper"
                if stop_after_approach
                else "cluster_collection_test"
            ),
            "stop_after_approach": stop_after_approach,
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
                "servo completion means command sequence plus calibrated wait, not position sensing",
                *(
                    [
                        "no-gripper mode stops at the cluster approach criterion",
                        "no gripper command or blind-grasp descent is permitted",
                    ]
                    if stop_after_approach
                    else []
                ),
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
                "selected_target": repr(decision.selected_target),
                "target_bottom_ratio": decision.target_bottom_ratio,
                "descent_vertical_only": decision.descent_vertical_only,
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
        super().__init__(capture_images=False, node_name="rov_cluster_collection_test")
        self.cluster_config = config
        self.execution_feedback = None
        self.execution_future = None
        self.execution_id = None
        self.create_subscription(GripperExecutionStatus, "/rov/control/gripper_execution",
                                 self._execution_feedback, 10)

    def _execution_feedback(self, message):
        if message.request_id != self.execution_id or message.source != SOURCE:
            return
        age = (self.get_clock().now().nanoseconds / 1e9
               - message.stamp.sec - message.stamp.nanosec / 1e9)
        if not math.isfinite(age) or not -0.1 <= age <= 1.0:
            return
        self.execution_feedback = (message, time.monotonic())

    def start_execution(self, mission, action, gripper, now):
        if not self.gripper_client.service_is_ready():
            raise DatasetDriveError("机械爪服务不可用；不重发")
        request = SetGripper.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.source = SOURCE
        request.action = {"open": request.OPEN, "close": request.CLOSE,
                          "raise": request.RAISE, "reset": request.RESET}[action]
        request.request_id = uuid.uuid4().hex
        self.execution_id = request.request_id
        self.execution_feedback = None
        duration = len(gripper.steps_for(action)) * (gripper.step_interval_s + 2.0)
        mission.bind_execution(request.request_id, now, duration + gripper.wait_for(action) + 5.0)
        self.execution_future = self.gripper_client.call_async(request)

    def poll_execution(self, mission, observation, now, *, allow_transition):
        if mission.execution_request_id is None:
            return None
        future = self.execution_future
        if future is not None and future.done():
            try:
                response = future.result()
            except Exception as exc:
                raise DatasetDriveError(f"机械爪请求失败（不重发）: {exc}") from exc
            if response is None or response.request_id != mission.execution_request_id or not response.success:
                raise DatasetDriveError(f"机械爪请求拒绝或响应标识不匹配: {getattr(response, 'message', '')}")
        # Never consume completion before the matching service acceptance.
        feedback = self.execution_feedback
        if feedback is not None and now - feedback[1] <= 1.0:
            message = feedback[0]
            stamp = message.stamp.sec + message.stamp.nanosec / 1e9
            result = mission.execution_status(message.request_id, message.action, message.state,
                observation, now, stamp=stamp,
                allow_transition=allow_transition and future is not None and future.done())
            if result is not None:
                return result
        if now >= mission.execution_deadline or now - mission.execution_status_at > 1.0:
            return mission.abort("机械动作超时或执行状态过期", observation)
        return None

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
        target = decision.selected_target
        message.has_selected_target = target is not None
        message.descent_vertical_only = decision.descent_vertical_only
        if target is not None:
            message.selected_confidence = float(target.confidence)
            message.selected_left = float(target.box.left / observation.frame_width)
            message.selected_right = float(target.box.right / observation.frame_width)
            message.selected_top = float(target.box.top / observation.frame_height)
            message.selected_bottom = float(target.box.bottom / observation.frame_height)
        self.mission_status_publisher.publish(message)

def _window_lines(
    decision: ClusterCollectionDecision,
    *,
    paused: bool,
    perception_age_s: float,
) -> list[str]:
    cluster = decision.cluster
    lines = [
        "ROV CLUSTER COLLECTION TEST | Mouse leave OK; keyboard focus loss pauses",
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
    return lines


def _wait_for_operator_start(window, node, dataset, *, allow_gripper):
    """终端确认后仍保持上锁；只在控制窗口收到新 Enter 才允许解锁。"""

    print("参数已确认，尚未解锁。点击 ROV Cluster Collection Test 窗口并按 Enter 开始。", flush=True)
    print("鼠标移出不会暂停；切换键盘焦点或最小化仍回中暂停。", flush=True)
    next_update = 0.0
    while rclpy.ok():
        state = window.poll()
        if not window.alive or (state is not None and state.estop):
            raise DatasetDriveError("操作员取消启动或窗口启动失败；未解锁")
        if state is None and time.monotonic() - window.created_at > 15.0:
            raise DatasetDriveError("控制窗口启动超时；未解锁")
        if state is not None and time.monotonic() - state.sent_at > 3.0:
            raise DatasetDriveError("控制窗口无响应；未解锁")
        node.spin(0.02)
        error = _prearm_error(node, dataset, require_disarmed=True, allow_gripper=allow_gripper)
        if error is not None:
            raise DatasetDriveError(error)
        if time.monotonic() >= next_update:
            window.show([
                "DISARMED - READY TO START",
                "Click THIS control window, then press ENTER to arm and start.",
                "Mouse leave OK; keyboard focus loss/minimize pauses.",
                "SPACE pause | ENTER resume | 0 return+disarm | ESC/close emergency stop",
            ])
            next_update = time.monotonic() + 0.10
        if (state is not None and state.enter_seq > 0 and state.focused and not state.paused
                and 0.0 <= time.monotonic() - state.sent_at <= 0.5):
            node.wait_for_perception(timeout_s=2.0)
            if node.perception_age_s() > 0.5:
                raise DatasetDriveError("启动时检测帧已过期；未解锁")
            return state.enter_seq
    raise DatasetDriveError("ROS 已关闭；未解锁")


def _stop_overlay(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def _interactive_parameters(
    node: ClusterCollectionNode,
    config: ClusterCollectionConfig,
    command_limit: float,
    *,
    stop_after_approach: bool,
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
    if config.descent_trigger == DescentTrigger.TARGET_BOTTOM_LINE:
        print(f"  单目标框下缘触发线：{config.target_bottom_line_y_ratio:.0%}；新帧 {config.descent_required_hits}/{config.descent_window_frames}")
        print(f"  抓取斜降：前进 {config.grasp_forward_command:.2f}，下降 {config.grasp_descent_command:.2f}，硬超时 {config.grasp_descent_timeout_s:.1f}s")
        if not stop_after_approach:
            print("  底部闭爪→抬爪→张爪落料→闭爪→复位完成→艇体上浮；三次仅代表动作循环。")
            print("  必须已人工确认抬爪在标定复位位置；禁止 QGC/其他程序同时发送舵机动作。")
    if config.maximum_operation_depth_m is None:
        print("  固定米数停止条件：关闭；仍依赖可信深度的稳定时间推测触底。")
    else:
        print(f"  作业最大深度：{config.maximum_operation_depth_m:.2f} m")
    print(
        f"  自动触底：至少下潜 {config.bottom_detection_minimum_descent_m:.2f} m，"
        f"深度变化不超过 {config.bottom_detection_depth_tolerance_m:.2f} m，"
        f"稳定 {config.bottom_detection_stable_s:.1f} s。"
    )
    print(
        f"  下潜超时：{config.bottom_probe_timeout_s:.1f} s；"
        f"深度跳变回中复核，{config.depth_anomaly_timeout_s:.1f} s 未恢复则中止。"
    )
    print(f"  每次触底后上浮：{config.bottom_clearance_m:.2f} m")
    print(
        f"  离底上浮 power：{config.bottom_clearance_up_command:.2f}（单档跨过飞控死区；"
        "不追加补推，达到目标后立即回中）。"
    )
    print(f"  群体门槛：{config.minimum_cluster_count} 个 scallop，最近 5 帧至少 3 帧")
    print(f"  下降判据：{config.descent_trigger.value}")
    if stop_after_approach:
        confirmation = "START CLUSTER APPROACH WITHOUT GRIPPER"
        print("  模式：无机械爪群体搜索—接近")
        print("  达到接近判据后仅发布全零控制并请求上锁。")
        print("  不会发送机械爪或盲抓下降命令。")
        print("\n必须确认：ROV 已浸没，危险区无人，ALT_HOLD，")
        print("QGC 有实时遥测/5600 画面/人工上锁。")
    else:
        confirmation = CONFIRMATION
        print(f"  每群机械爪动作：{config.grabs_per_cluster} 次")
        print("  TRANSFER_TO_NET 当前只等待和记录，没有网兜机械动作。")
        print("  机械爪 ACK 只表示飞控接受命令，不代表已真实抓到扇贝。")
        print("\n必须确认：ROV 已浸没，危险区无人，ALT_HOLD，")
        print("QGC 有实时遥测/5600 画面/人工上锁，RST 爪已实艇开闭验证。")
    typed = input(f"全部满足后完整输入 {confirmation!r}: ").strip()
    if typed != confirmation:
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
    if os.environ.get("ROV_REQUIRE_DUAL_GRASP") == "1" and (
        args.stop_after_approach or cluster_config.descent_trigger != DescentTrigger.TARGET_BOTTOM_LINE
    ):
        errors.append("完整双自由度入口必须使用 target_bottom_line 抓取模式")
    if robot.control_profile != ControlProfile.COMMISSIONING:
        errors.append("robot.yaml 的 profile 必须是 commissioning")
    if "ALT_HOLD" not in robot.allowed_flight_modes:
        errors.append("robot.yaml 必须允许 ALT_HOLD")
    if not robot.allow_live_actuation:
        errors.append("robot.yaml 尚未允许真实输出")
    if not robot.allow_ros_arming:
        errors.append("robot.yaml 尚未允许 ROS 解锁")
    if not args.stop_after_approach:
        if not robot.allow_gripper_actuation:
            errors.append("robot.yaml 尚未允许机械爪输出")
        expected_profile = "dual_dof" if cluster_config.descent_trigger == DescentTrigger.TARGET_BOTTOM_LINE else "rst"
        if robot.gripper.profile != expected_profile:
            errors.append(f"当前流程必须使用 {expected_profile} 档案；禁止复用旧联动输出")
        if not robot.gripper.calibrated:
            errors.append("机械爪档案尚未标记为实艇标定通过")
        if expected_profile == "dual_dof":
            from .field_setup import verify_gripper_activation, field_paths
            from .dual_gripper_calibration import signature
            valid, reason = verify_gripper_activation(args.project_dir)
            if not valid:
                errors.append(f"双自由度标定证据未通过: {reason}")
            else:
                tested = load_robot_config(field_paths(args.project_dir).robot_config).gripper
                if signature(tested) != signature(robot.gripper):
                    errors.append("会话舵机参数与已标定档案不一致")
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
        if args.stop_after_approach:
            print("无机械爪群体搜索—接近配置预览通过；未初始化 ROS、未解锁、未动爪。")
        else:
            print("群体收集配置预览通过；未初始化 ROS、未解锁、未动爪。")
        return 0
    if not args.session_dir:
        print("真实执行必须使用 --session-dir")
        return 2
    session_dir = Path(args.session_dir).expanduser().resolve()
    logger: ClusterSessionLogger | None = None
    recorder: RtpMkvRecorder | None = None
    node: ClusterCollectionNode | None = None
    mission: ClusterCollectionMission | None = None
    overlay_process: subprocess.Popen | None = None
    window: ClusterWindowBridge | None = None
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
        error = _prearm_error(
            node,
            dataset,
            require_disarmed=True,
            allow_gripper=not args.stop_after_approach,
        )
        if error is not None:
            raise DatasetDriveError(error)

        logger = ClusterSessionLogger(
            session_dir,
            project_directory=Path(args.project_dir).expanduser().resolve(),
            robot_config=Path(args.robot_config).expanduser().resolve(),
            autonomy_config=Path(args.autonomy_config).expanduser().resolve(),
            cluster_config=Path(args.cluster_config).expanduser().resolve(),
            gripper_profile=robot.gripper.profile,
            stop_after_approach=args.stop_after_approach,
        )
        with (session_dir / "logs/cluster_overlay.log").open("a", encoding="utf-8") as overlay_log:
            overlay_process = subprocess.Popen(
                [sys.executable, "-m", "rov_competition.cluster_overlay",
                 "--line-ratio", str(cluster_config.target_bottom_line_y_ratio
                    if cluster_config.descent_trigger == DescentTrigger.TARGET_BOTTOM_LINE
                    else cluster_config.descent_line_y_ratio),
                 "--parent-pid", str(os.getpid())],
                stdout=overlay_log, stderr=subprocess.STDOUT,
            )
        recorder = RtpMkvRecorder(
            session_dir, source_port=args.record_port, payload_type=args.payload_type
        )
        recorder.start()
        recorder.wait_until_receiving(timeout_s=10.0, pump=lambda: node.spin(0.0))
        descent_power, start_depth = _interactive_parameters(
            node,
            cluster_config,
            robot.command_limit,
            stop_after_approach=args.stop_after_approach,
        )
        selected_errors = cluster_config.readiness_errors(
            robot_command_limit=robot.command_limit,
            descent_command=descent_power,
        )
        if selected_errors:
            raise DatasetDriveError("; ".join(selected_errors))
        # 窗口在独立进程中创建，终端输入完成前绝不解锁。
        window = ClusterWindowBridge(session_dir / "logs/cluster_window.log")
        start_enter_seq = _wait_for_operator_start(
            window, node, dataset,
            allow_gripper=not args.stop_after_approach,
        )
        window_gate = WindowGate(start_enter_seq)
        start_depth = node.observation().depth_m
        if not node.observation().depth_valid or not math.isfinite(start_depth):
            raise DatasetDriveError("启动时深度无效；未解锁")
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
        before_arm = window_gate.evaluate(window.poll(), now=time.monotonic(), alive=window.alive)
        if before_arm.error or before_arm.pause_reason:
            raise DatasetDriveError(f"解锁前窗口未就绪: {before_arm.error or before_arm.pause_reason}")
        node.arm()
        _wait_for_gateway_state(
            node,
            lambda: bool(node.status and node.status.armed and node.status.armed_by_ros),
            "ROS 解锁",
        )
        error = _active_error(
            node, dataset, allow_gripper=not args.stop_after_approach
        )
        if error is not None:
            raise DatasetDriveError(error)

        mission_type = TargetGraspMission if cluster_config.descent_trigger == DescentTrigger.TARGET_BOTTOM_LINE else ClusterCollectionMission
        mission = mission_type(
            cluster_config, stop_after_approach=args.stop_after_approach
        )
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
        pause_reason = ""
        pause_started_at: float | None = None
        perception_hold_started_at: float | None = None
        next_publish = time.monotonic()
        gripper_event = ""
        last_execution_logged = None
        next_window_update = 0.0
        timing = LoopTiming()

        while rclpy.ok():
            timing.begin()
            action = window_gate.evaluate(window.poll(), now=time.monotonic(), alive=window.alive)
            if action.error:
                raise DatasetDriveError(action.error)
            normal_finish_requested = action.finish
            if action.pause_reason:
                if not paused:
                    pause_started_at = time.monotonic()
                paused = True
                pause_reason = action.pause_reason
                node.publish(MotionCommand.neutral())
            elif action.resume and paused:
                if node.perception_age_s() <= cluster_config.perception_hold_timeout_s:
                    if pause_started_at is not None:
                        mission.delay_timers(time.monotonic() - pause_started_at)
                    paused = False
                    pause_reason = ""
                    pause_started_at = None
            timing.mark("window_ipc")
            node.spin(0.0)
            timing.mark("ros_callbacks")
            error = _active_error(
                node, dataset, allow_gripper=not args.stop_after_approach
            )
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
            execution_decision = None
            if isinstance(mission, TargetGraspMission):
                execution_decision = node.poll_execution(mission, observation, now,
                    allow_transition=not (paused or perception_holding or normal_finish_requested))
                if node.execution_feedback is not None:
                    feedback = node.execution_feedback[0]
                    event_key = (feedback.request_id, feedback.state)
                    if event_key != last_execution_logged:
                        last_execution_logged = event_key
                        gripper_event = f"request={feedback.request_id} action={feedback.action} state={feedback.state}"
                        logger.write_gripper(decision, observation, action=feedback.action,
                            accepted=feedback.state in {"accepted", "running", "completed"},
                            acknowledgement=gripper_event + ":" + feedback.message)
            if execution_decision is not None:
                decision = execution_decision
            elif normal_finish_requested and decision.state != ClusterCollectionState.RETURNING:
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
                    message=f"{pause_reason}，已回中暂停；回到控制窗口按 Enter 恢复",
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

            if args.stop_after_approach and decision.gripper_action is not None:
                raise DatasetDriveError(
                    "无机械爪模式异常产生机械爪动作；已禁止发送该命令"
                )
            if isinstance(mission, TargetGraspMission):
                if decision.gripper_action is not None and not (paused or perception_holding):
                    if mission.execution_request_id is None:
                        node.publish(MotionCommand.neutral())
                        node.start_execution(mission, decision.gripper_action, robot.gripper, now)
                        gripper_event = f"{decision.gripper_action}:request={mission.execution_request_id}"
                        logger.write_gripper(decision, observation, action=decision.gripper_action,
                            accepted=False, acknowledgement=gripper_event + ":pending")
                    decision = replace(decision, gripper_action=None)
            elif decision.gripper_action is not None:
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

            timing.mark("health_and_decision")
            # 先发本轮新决定，再做磁盘日志和窗口绘制；没有后台重播旧指令。
            publish_now = time.monotonic()
            publish_due = publish_now >= next_publish
            if publish_due:
                node.publish(decision.motion)
                node.publish_cluster_status(decision, observation)
                next_publish += 0.05
                if next_publish <= publish_now:
                    next_publish = publish_now + 0.05
            timing.mark("command_publish")
            # 先让状态机消费这张新帧，再把检测和它导致的群体状态
            # 写在同一行记录中，避免 clusters.csv 落后一帧。
            if log_new_frame:
                logger.write_frame(decision, observation, config=cluster_config)
                last_logged_frame = observation.frame_id

            reporter.report(decision, observation, now)
            if publish_due:
                logger.write_decision(
                    decision, observation, gripper_event=gripper_event
                )
                gripper_event = ""
            timing.mark("disk_and_console")
            if time.monotonic() >= next_window_update:
                window.show(_window_lines(
                    decision,
                    paused=paused, perception_age_s=perception_age,
                ))
                next_window_update = time.monotonic() + 0.10
            timing.mark("window_ipc_send")
            warning = timing.warning()
            if warning is not None:
                print(warning, flush=True)
            if decision.state == ClusterCollectionState.ABORTED:
                raise DatasetDriveError(decision.message)
            if decision.state == ClusterCollectionState.COMPLETE:
                break
            time.sleep(max(0.0, 1.0 / 60.0 - (time.monotonic() - timing.started_at)))

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
            if args.stop_after_approach:
                # 上锁拒绝后不恢复状态机；再次连续回中，随后 finally 请求急停锁存。
                node.publish_neutral()
                raise DatasetDriveError(
                    "无机械爪接近完成后正常上锁未确认；"
                    f"已持续发布全零控制: {disarm_error}"
                )
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
        if window is not None:
            window.close()
        _stop_overlay(overlay_process)
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
