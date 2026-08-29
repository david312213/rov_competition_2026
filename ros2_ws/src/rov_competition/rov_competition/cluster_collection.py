"""群体盲抓与跳跃式离底搜索的纯 Python 状态机。

本模块不导入 ROS、YOLO、Pygame 或 MAVLink。它只接收结构化
检测和遥测，输出归一化运动意图与待确认的机械爪动作。
压力深度只用于记录水面以下深度；没有下视测距时，本流程
只能通过“重新触底 -> 上浮固定距离”近似建立离底搜索深度。
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .bottom_clearance import update_bottom_clearance, update_bottom_probe
from .domain import BoundingBox, Detection, MissionObservation, MotionCommand
from .mission import signed_yaw_delta_deg


class ClusterCollectionError(RuntimeError):
    """群体收集配置或状态无法安全继续。"""


class DescentTrigger(str, Enum):
    """从靠近切换为盲抓下降的判据。"""

    IMAGE_LINE = "image_line"
    UNION_AREA = "union_area"
    BOTH = "both"


class ClusterCollectionState(str, Enum):
    """群体收集测试的所有状态。"""

    IDLE = "idle"
    PROBING_BOTTOM = "probing_bottom"
    CLEARING_BOTTOM = "clearing_bottom"
    SCANNING = "scanning"
    SEARCH_ADVANCING = "search_advancing"
    ALIGNING = "aligning"
    APPROACHING = "approaching"
    VERIFYING_LOSS = "verifying_loss"
    OPENING_GRIPPER = "opening_gripper"
    GRASP_DESCENDING = "grasp_descending"
    CLOSING_GRIPPER = "closing_gripper"
    GRASP_HOLDING = "grasp_holding"
    LIFTING_AFTER_GRASP = "lifting_after_grasp"
    TRANSFER_TO_NET = "transfer_to_net"
    REOPENING_GRIPPER = "reopening_gripper"
    INTER_GRAB_ADVANCING = "inter_grab_advancing"
    LEAVING_CLUSTER = "leaving_cluster"
    RETURNING = "returning"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass(frozen=True)
class ClusterCollectionConfig:
    """跳跃式搜索、群体靠近与三次盲抓参数。"""

    maximum_operation_depth_m: float = 20.0
    mission_timeout_s: float = 900.0

    bottom_detection_stable_s: float = 3.0
    bottom_detection_depth_tolerance_m: float = 0.05
    bottom_detection_minimum_descent_m: float = 0.10
    bottom_neutral_confirmation_s: float = 1.0
    bottom_probe_timeout_s: float = 45.0
    bottom_clearance_m: float = 0.15
    bottom_clearance_tolerance_m: float = 0.02
    bottom_clearance_up_command: float = 0.40
    bottom_clearance_settle_s: float = 1.0
    bottom_clearance_timeout_s: float = 15.0

    perception_hold_timeout_s: float = 1.0
    perception_abort_timeout_s: float = 5.0
    tracking_command_hold_s: float = 0.20

    scan_yaw_command: float = 0.40
    scan_angle_deg: float = 360.0
    scan_tolerance_deg: float = 3.0
    scan_timeout_s: float = 120.0
    maximum_yaw_step_deg: float = 45.0
    search_advance_command: float = 0.40
    search_advance_duration_s: float = 2.0

    target_label: str = "scallop"
    minimum_cluster_count: int = 6
    cluster_link_distance_ratio: float = 0.18
    acquisition_window_frames: int = 5
    acquisition_required_hits: int = 3
    cluster_center_smoothing_alpha: float = 0.35
    maximum_tracking_jump_ratio: float = 0.25

    aim_x_ratio: float = 0.50
    image_yaw_sign: int = 1
    horizontal_tolerance: float = 0.08
    realign_threshold: float = 0.12
    alignment_frames: int = 3
    yaw_gain: float = 0.80
    minimum_yaw_command: float = 0.10
    maximum_yaw_command: float = 0.20
    approach_yaw_command: float = 0.10
    far_center_y_ratio: float = 0.55
    descent_line_y_ratio: float = 0.70
    far_forward_command: float = 0.20
    near_forward_command: float = 0.10

    descent_trigger: DescentTrigger = DescentTrigger.IMAGE_LINE
    union_area_threshold: float | None = None
    descent_window_frames: int = 5
    descent_required_hits: int = 4
    loss_required_frames: int = 5
    loss_confirmation_s: float = 0.80

    grabs_per_cluster: int = 3
    gripper_motion_wait_s: float = 1.50
    transfer_to_net_wait_s: float = 1.00
    inter_grab_forward_command: float = 0.20
    inter_grab_duration_s: float = 0.50
    leave_cluster_forward_command: float = 0.40
    leave_cluster_duration_s: float = 2.0

    return_gain: float = 1.00
    return_minimum_command: float = 0.40
    return_maximum_command: float = 0.60
    return_tolerance_m: float = 0.05
    return_settle_s: float = 1.0
    return_timeout_s: float = 60.0

    def __post_init__(self) -> None:
        positive = (
            self.maximum_operation_depth_m,
            self.mission_timeout_s,
            self.bottom_detection_stable_s,
            self.bottom_detection_depth_tolerance_m,
            self.bottom_detection_minimum_descent_m,
            self.bottom_neutral_confirmation_s,
            self.bottom_probe_timeout_s,
            self.bottom_clearance_m,
            self.bottom_clearance_tolerance_m,
            self.bottom_clearance_up_command,
            self.bottom_clearance_settle_s,
            self.bottom_clearance_timeout_s,
            self.perception_hold_timeout_s,
            self.perception_abort_timeout_s,
            self.tracking_command_hold_s,
            self.scan_yaw_command,
            self.scan_angle_deg,
            self.scan_tolerance_deg,
            self.scan_timeout_s,
            self.maximum_yaw_step_deg,
            self.search_advance_command,
            self.search_advance_duration_s,
            self.cluster_link_distance_ratio,
            self.cluster_center_smoothing_alpha,
            self.maximum_tracking_jump_ratio,
            self.horizontal_tolerance,
            self.realign_threshold,
            self.yaw_gain,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.approach_yaw_command,
            self.far_center_y_ratio,
            self.descent_line_y_ratio,
            self.far_forward_command,
            self.near_forward_command,
            self.loss_confirmation_s,
            self.gripper_motion_wait_s,
            self.transfer_to_net_wait_s,
            self.inter_grab_forward_command,
            self.inter_grab_duration_s,
            self.leave_cluster_forward_command,
            self.leave_cluster_duration_s,
            self.return_gain,
            self.return_minimum_command,
            self.return_maximum_command,
            self.return_tolerance_m,
            self.return_settle_s,
            self.return_timeout_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ClusterCollectionError("群体收集数值参数必须是大于 0 的有限数")
        if not self.target_label.strip():
            raise ClusterCollectionError("群体目标类别不能为空")
        if self.perception_abort_timeout_s <= self.perception_hold_timeout_s:
            raise ClusterCollectionError("感知硬中止时间必须大于回中等待时间")
        if self.tracking_command_hold_s >= self.perception_hold_timeout_s:
            raise ClusterCollectionError("跟踪命令保持时间必须小于感知回中时间")
        if self.bottom_detection_depth_tolerance_m >= self.bottom_detection_minimum_descent_m:
            raise ClusterCollectionError("触底深度容差必须小于最小真实下潜量")
        if self.bottom_neutral_confirmation_s >= self.bottom_detection_stable_s:
            raise ClusterCollectionError("疑似触底回中时间必须小于完整确认时间")
        if not 0.05 <= self.bottom_clearance_m <= 0.30:
            raise ClusterCollectionError("离底距离必须在 0.05..0.30 m")
        if self.bottom_clearance_tolerance_m >= self.bottom_clearance_m:
            raise ClusterCollectionError("离底容差必须小于离底距离")
        if not 0.0 < self.horizontal_tolerance < self.realign_threshold < 1.0:
            raise ClusterCollectionError("水平容差必须小于重新对准阈值")
        if not 0.0 < self.far_center_y_ratio < self.descent_line_y_ratio < 1.0:
            raise ClusterCollectionError("靠近线必须满足 0 < far_y < descent_y < 1")
        if not 0.0 < self.aim_x_ratio < 1.0:
            raise ClusterCollectionError("水平矄准点必须在画面内")
        if self.image_yaw_sign not in {-1, 1}:
            raise ClusterCollectionError("image_yaw_sign 必须是 -1 或 1")
        if self.minimum_yaw_command > self.maximum_yaw_command:
            raise ClusterCollectionError("最小偏航指令不能大于最大值")
        if self.return_minimum_command > self.return_maximum_command:
            raise ClusterCollectionError("回收最小指令不能大于最大值")
        if not 0.0 < self.cluster_center_smoothing_alpha <= 1.0:
            raise ClusterCollectionError("群中心平滑系数必须在 (0, 1]")
        if self.scan_tolerance_deg >= self.scan_angle_deg:
            raise ClusterCollectionError("扫描容差必须小于扫描角度")
        if self.maximum_yaw_step_deg > 180.0:
            raise ClusterCollectionError("航向单次跳变上限不能超过 180°")
        if self.near_forward_command > self.far_forward_command:
            raise ClusterCollectionError("近距离前进指令不能大于远距离指令")
        integer_values = (
            self.minimum_cluster_count,
            self.acquisition_window_frames,
            self.acquisition_required_hits,
            self.alignment_frames,
            self.descent_window_frames,
            self.descent_required_hits,
            self.loss_required_frames,
            self.grabs_per_cluster,
        )
        if any(value <= 0 for value in integer_values):
            raise ClusterCollectionError("帧数、数量和抓取次数必须是正整数")
        if self.acquisition_required_hits > self.acquisition_window_frames:
            raise ClusterCollectionError("群体确认命中数不能超过窗口帧数")
        if self.descent_required_hits > self.descent_window_frames:
            raise ClusterCollectionError("下降确认命中数不能超过窗口帧数")
        if self.grabs_per_cluster != 3:
            raise ClusterCollectionError("当前群体测试固定每群抓取 3 次")
        commands = (
            self.bottom_clearance_up_command,
            self.scan_yaw_command,
            self.search_advance_command,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.approach_yaw_command,
            self.far_forward_command,
            self.near_forward_command,
            self.inter_grab_forward_command,
            self.leave_cluster_forward_command,
            self.return_minimum_command,
            self.return_maximum_command,
        )
        if any(value > 1.0 for value in commands):
            raise ClusterCollectionError("归一化运动指令不能超过 1.0")
        if self.descent_trigger in {DescentTrigger.UNION_AREA, DescentTrigger.BOTH}:
            if self.union_area_threshold is None:
                raise ClusterCollectionError("面积下降判据缺少 union_area_threshold")
        if self.union_area_threshold is not None and not 0.0 < self.union_area_threshold < 1.0:
            raise ClusterCollectionError("union_area_threshold 必须在 (0, 1)")

    def readiness_errors(
        self, *, robot_command_limit: float, descent_command: float | None = None
    ) -> tuple[str, ...]:
        required = max(
            self.bottom_clearance_up_command,
            self.scan_yaw_command,
            self.search_advance_command,
            self.maximum_yaw_command,
            self.far_forward_command,
            self.inter_grab_forward_command,
            self.leave_cluster_forward_command,
            self.return_maximum_command,
        )
        if descent_command is not None:
            if not math.isfinite(descent_command) or not 0.10 <= descent_command <= 0.80:
                return ("下潜 power 必须在 0.10..0.80",)
            required = max(required, descent_command)
        if robot_command_limit + 1e-9 < required:
            return (
                f"robot.yaml command_limit={robot_command_limit:.2f} "
                f"小于群体收集所需 {required:.2f}",
            )
        return ()


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ClusterCollectionError(f"{name} 必须是 YAML 映射")
    return value


def load_cluster_collection_config(path: str | Path) -> ClusterCollectionConfig:
    """读取并严格校验群体收集配置。"""

    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise ClusterCollectionError(f"配置文件不存在: {config_path}")
    with config_path.open("r", encoding="utf-8") as stream:
        root = yaml.safe_load(stream) or {}
    section = _mapping(_mapping(root, "root").get("cluster_collection", {}), "cluster_collection")
    values = dict(section)
    try:
        values["descent_trigger"] = DescentTrigger(
            str(values.get("descent_trigger", DescentTrigger.IMAGE_LINE.value))
        )
        if values.get("union_area_threshold") is not None:
            values["union_area_threshold"] = float(values["union_area_threshold"])
        return ClusterCollectionConfig(**values)
    except (TypeError, ValueError) as exc:
        raise ClusterCollectionError(f"群体收集配置无效: {exc}") from exc


@dataclass(frozen=True)
class ScallopCluster:
    """单帧中一个空间连通的扇贝群。"""

    detections: tuple[Detection, ...]
    center_x_ratio: float
    center_y_ratio: float
    union_area_ratio: float
    total_confidence: float

    @property
    def count(self) -> int:
        return len(self.detections)


def exact_union_area_ratio(
    boxes: Sequence[BoundingBox], frame_width: int, frame_height: int
) -> float:
    """返回多个像素框的精确联合面积占比，不重复计算重叠区。"""

    if frame_width <= 0 or frame_height <= 0 or not boxes:
        return 0.0
    rectangles = [
        (
            max(0.0, min(float(frame_width), box.left)),
            max(0.0, min(float(frame_height), box.top)),
            max(0.0, min(float(frame_width), box.right)),
            max(0.0, min(float(frame_height), box.bottom)),
        )
        for box in boxes
        if box.right > box.left and box.bottom > box.top
    ]
    xs = sorted({value for rect in rectangles for value in (rect[0], rect[2])})
    area = 0.0
    for left, right in zip(xs, xs[1:]):
        if right <= left:
            continue
        intervals = sorted(
            (top, bottom)
            for rect_left, top, rect_right, bottom in rectangles
            if rect_left < right and rect_right > left
        )
        if not intervals:
            continue
        merged_height = 0.0
        start, end = intervals[0]
        for next_start, next_end in intervals[1:]:
            if next_start <= end:
                end = max(end, next_end)
            else:
                merged_height += max(0.0, end - start)
                start, end = next_start, next_end
        merged_height += max(0.0, end - start)
        area += (right - left) * merged_height
    return area / float(frame_width * frame_height)


def _cluster_from_detections(
    detections: Sequence[Detection], frame_width: int, frame_height: int
) -> ScallopCluster:
    centers = [item.box.center() for item in detections]
    return ScallopCluster(
        detections=tuple(detections),
        center_x_ratio=statistics.median(x for x, _ in centers) / frame_width,
        center_y_ratio=statistics.median(y for _, y in centers) / frame_height,
        union_area_ratio=exact_union_area_ratio(
            [item.box for item in detections], frame_width, frame_height
        ),
        total_confidence=sum(float(item.confidence) for item in detections),
    )


def cluster_scallops(
    detections: Sequence[Detection],
    frame_width: int,
    frame_height: int,
    *,
    label: str = "scallop",
    link_distance_ratio: float = 0.18,
) -> tuple[ScallopCluster, ...]:
    """用归一化框中心距离的连通分量形成群体。"""

    if frame_width <= 0 or frame_height <= 0:
        return ()
    items = [item for item in detections if item.label == label]
    if not items:
        return ()
    centers = [
        (item.box.center()[0] / frame_width, item.box.center()[1] / frame_height)
        for item in items
    ]
    remaining = set(range(len(items)))
    groups: list[ScallopCluster] = []
    while remaining:
        seed = remaining.pop()
        component = {seed}
        queue = [seed]
        while queue:
            current = queue.pop()
            cx, cy = centers[current]
            adjacent = {
                index
                for index in remaining
                if math.hypot(centers[index][0] - cx, centers[index][1] - cy)
                <= link_distance_ratio
            }
            remaining.difference_update(adjacent)
            component.update(adjacent)
            queue.extend(adjacent)
        groups.append(
            _cluster_from_detections(
                [items[index] for index in sorted(component)],
                frame_width,
                frame_height,
            )
        )
    return tuple(groups)


def select_cluster(
    clusters: Sequence[ScallopCluster], *, minimum_count: int, aim_x_ratio: float
) -> ScallopCluster | None:
    """数量优先，再按总置信度和距中心距离选群。"""

    eligible = [item for item in clusters if item.count >= minimum_count]
    if not eligible:
        return None
    return max(
        eligible,
        key=lambda item: (
            item.count,
            item.total_confidence,
            -abs(item.center_x_ratio - aim_x_ratio),
        ),
    )


@dataclass(frozen=True)
class ClusterCollectionDecision:
    """状态机一个控制周期的完整输出。"""

    state: ClusterCollectionState
    motion: MotionCommand
    message: str
    cluster: ScallopCluster | None = None
    visible_scallop_count: int = 0
    scan_progress_deg: float | None = None
    search_cycle: int = 0
    grasp_attempt_index: int = 0
    completed_cluster_count: int = 0
    descent_trigger_mode: str = DescentTrigger.IMAGE_LINE.value
    current_depth_m: float | None = None
    target_depth_m: float | None = None
    horizontal_error: float | None = None
    gripper_action: str | None = None
    gripper_command_accepted: bool = False
    outcome: str = "running"


class ClusterCollectionMission:
    """触底、离底搜索、群体靠近和三次盲抓状态机。"""

    def __init__(self, config: ClusterCollectionConfig) -> None:
        self.config = config
        self.state = ClusterCollectionState.IDLE
        self.outcome = "none"
        self.message = "群体收集未启动"
        self.start_depth_m: float | None = None
        self.bottom_depth_m: float | None = None
        self.target_depth_m: float | None = None
        self.descent_command = 0.60
        self.state_started_at = 0.0
        self.mission_started_at = 0.0
        self.settled_since: float | None = None
        self.clearance_reference_depth_m: float | None = None
        self.depth_history: deque[tuple[float, float]] = deque()
        self.probe_start_depth_m: float | None = None
        self.bottom_stable_duration_s = 0.0
        self.bottom_depth_span_m = 0.0
        self.probe_purpose = "search"
        self.clearance_purpose = "search"

        self.scan_progress_deg = 0.0
        self.previous_yaw_deg: float | None = None
        self.search_cycle = 0
        self.last_frame_id = -1
        self.last_new_frame_at = 0.0
        self.candidate_center: tuple[float, float] | None = None
        self.candidate_history: deque[bool] = deque(
            maxlen=config.acquisition_window_frames
        )
        self.tracked_cluster: ScallopCluster | None = None
        self.smoothed_center: tuple[float, float] | None = None
        self.last_target_seen_at: float | None = None
        self.consecutive_misses = 0
        self.aligned_frames = 0
        self.descent_history: deque[bool] = deque(
            maxlen=config.descent_window_frames
        )
        self.last_tracking_decision: ClusterCollectionDecision | None = None
        self.state_before_loss: ClusterCollectionState | None = None

        self.grasp_attempts_completed = 0
        self.current_grasp_attempt_index = 0
        self.total_grasp_attempt_count = 0
        self.completed_cluster_count = 0
        self.awaiting_gripper_action: str | None = None
        self.last_gripper_accepted = False
        self.gripper_failure_reason: str | None = None
        self.last_visible_scallop_count = 0

    def start(
        self,
        observation: MissionObservation,
        *,
        descent_command: float,
        now: float,
    ) -> ClusterCollectionDecision:
        self._validate_observation(observation, require_attitude=False)
        if self.state not in {ClusterCollectionState.IDLE, ClusterCollectionState.COMPLETE}:
            raise ClusterCollectionError("群体收集任务已在运行")
        errors = self.config.readiness_errors(
            robot_command_limit=1.0, descent_command=descent_command
        )
        if errors:
            raise ClusterCollectionError(errors[0])
        self.start_depth_m = float(observation.depth_m)
        self.descent_command = float(descent_command)
        self.mission_started_at = float(now)
        self.search_cycle = 0
        self.grasp_attempts_completed = 0
        self.current_grasp_attempt_index = 0
        self.total_grasp_attempt_count = 0
        self.completed_cluster_count = 0
        self.gripper_failure_reason = None
        self.outcome = "running"
        self.last_frame_id = observation.frame_id
        self.last_new_frame_at = float(now)
        self._clear_cluster_lock()
        self._start_probe(observation, now, purpose="search")
        return self._decision(
            MotionCommand.neutral(), observation, "已记录启动深度，开始首次触底标定"
        )

    def step(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        if self.state in {
            ClusterCollectionState.IDLE,
            ClusterCollectionState.COMPLETE,
            ClusterCollectionState.ABORTED,
        }:
            return self._decision(MotionCommand.neutral(), observation, self.message)
        try:
            self._validate_observation(observation, require_attitude=False)
        except ClusterCollectionError as exc:
            return self.abort(str(exc), observation)
        if not math.isfinite(now):
            return self.abort("控制时间无效", observation)
        if now - self.mission_started_at >= self.config.mission_timeout_s:
            return self.request_normal_finish(observation, now, "15 分钟任务时限已到")
        if observation.depth_m >= self.config.maximum_operation_depth_m:
            return self.abort(
                f"深度 {observation.depth_m:.2f}m 已达作业硬上限", observation
            )
        if observation.frame_id < self.last_frame_id:
            return self.abort("图像帧号倒退", observation)
        new_frame = observation.frame_id > self.last_frame_id
        if new_frame:
            self.last_frame_id = observation.frame_id
            self.last_new_frame_at = float(now)
            self.last_visible_scallop_count = sum(
                1
                for item in observation.detections
                if item.label == self.config.target_label
            )

        if self.awaiting_gripper_action is not None:
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"等待机械爪 {self.awaiting_gripper_action} ACK",
            )
        if self.state in {
            ClusterCollectionState.PROBING_BOTTOM,
            ClusterCollectionState.GRASP_DESCENDING,
        }:
            return self._step_probing_bottom(observation, now)
        if self.state in {
            ClusterCollectionState.CLEARING_BOTTOM,
            ClusterCollectionState.LIFTING_AFTER_GRASP,
        }:
            return self._step_clearing_bottom(observation, now)
        if self.state == ClusterCollectionState.RETURNING:
            return self._step_returning(observation, now)
        if self.state == ClusterCollectionState.GRASP_HOLDING:
            return self._step_grasp_holding(observation, now)
        if self.state == ClusterCollectionState.TRANSFER_TO_NET:
            return self._step_transfer(observation, now)
        if self.state == ClusterCollectionState.INTER_GRAB_ADVANCING:
            return self._step_inter_grab_advance(observation, now)
        if self.state == ClusterCollectionState.LEAVING_CLUSTER:
            return self._step_leaving_cluster(observation, now)

        if not observation.perception_valid:
            return self._decision(MotionCommand.neutral(), observation, "没有新鲜图像，回中等待")
        if not new_frame:
            if self.state == ClusterCollectionState.SCANNING:
                return self._step_scanning(observation, now, consume_frame=False)
            if self.state == ClusterCollectionState.SEARCH_ADVANCING:
                return self._step_search_advance(observation, now, consume_frame=False)
            if (
                self.state in {ClusterCollectionState.ALIGNING, ClusterCollectionState.APPROACHING}
                and self.last_tracking_decision is not None
                and now - self.last_new_frame_at <= self.config.tracking_command_hold_s
            ):
                return self.last_tracking_decision
            return self._decision(
                MotionCommand.neutral(), observation, "等待下一张新检测帧，保持停车"
            )

        if self.state in {
            ClusterCollectionState.SCANNING,
            ClusterCollectionState.SEARCH_ADVANCING,
        } and self.tracked_cluster is None:
            acquired = self._update_acquisition(observation, now)
            if acquired is not None:
                return acquired
        elif self.state in {
            ClusterCollectionState.ALIGNING,
            ClusterCollectionState.APPROACHING,
            ClusterCollectionState.VERIFYING_LOSS,
        }:
            transition = self._update_tracking(observation, now)
            if transition is not None:
                return transition

        if self.state == ClusterCollectionState.SCANNING:
            return self._step_scanning(observation, now, consume_frame=True)
        if self.state == ClusterCollectionState.SEARCH_ADVANCING:
            return self._step_search_advance(observation, now, consume_frame=True)
        if self.state == ClusterCollectionState.ALIGNING:
            return self._step_aligning(observation, now)
        if self.state == ClusterCollectionState.APPROACHING:
            return self._step_approaching(observation, now)
        if self.state == ClusterCollectionState.VERIFYING_LOSS:
            return self._decision(MotionCommand.neutral(), observation, "群体漏检确认中，保持停车")
        return self.abort(f"未处理状态: {self.state.value}", observation)

    def acknowledge_gripper(
        self,
        action: str,
        accepted: bool,
        message: str,
        observation: MissionObservation,
        now: float,
    ) -> ClusterCollectionDecision:
        """消费一次机械爪 ACK；拒绝时立即转为中止。"""

        if action != self.awaiting_gripper_action:
            return self.abort(f"意外的机械爪 ACK: {action}", observation)
        self.awaiting_gripper_action = None
        self.last_gripper_accepted = bool(accepted)
        if not accepted:
            return self.request_gripper_failure_recovery(
                observation,
                now,
                f"机械爪 {action} 被拒绝: {message}",
            )
        if action == "open":
            self._start_probe(observation, now, purpose="grasp")
            return self._decision(
                MotionCommand.neutral(), observation, "开爪 ACK 已接受，开始垂直下降盲抓"
            )
        if action == "close":
            self.grasp_attempts_completed += 1
            self.total_grasp_attempt_count += 1
            self._enter(ClusterCollectionState.GRASP_HOLDING, observation, now)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"闭爪 ACK 已接受；记录第 {self.grasp_attempts_completed}/"
                f"{self.config.grabs_per_cluster} 次抓取尝试",
            )
        if action == "reopen":
            if self.grasp_attempts_completed < self.config.grabs_per_cluster:
                self._enter(ClusterCollectionState.INTER_GRAB_ADVANCING, observation, now)
                return self._decision(
                    MotionCommand.neutral(), observation, "已重新开爪，准备短移后下一次盲抓"
                )
            self.completed_cluster_count += 1
            self._clear_cluster_lock()
            self._enter(ClusterCollectionState.LEAVING_CLUSTER, observation, now)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"本群 {self.config.grabs_per_cluster} 次抓取动作已完成，准备离开当前区域",
            )
        return self.abort(f"未知机械爪 ACK 动作: {action}", observation)

    def request_gripper_failure_recovery(
        self,
        observation: MissionObservation,
        now: float,
        reason: str,
    ) -> ClusterCollectionDecision:
        """机械爪失败但深度仍有效时，先回收到启动深度再上锁。

        是否仍具备有效深度反馈由 ROS 运行层在调用前确认；一旦回收期间
        遥测失效，运行层仍会升级为急停，绝不盲目上浮。
        """

        self._validate_observation(observation, require_attitude=False)
        self.awaiting_gripper_action = None
        self.last_gripper_accepted = False
        self.gripper_failure_reason = reason
        return self.request_normal_finish(
            observation,
            now,
            f"{reason}；水平运动已停止，正在回收到启动深度",
        )

    def request_normal_finish(
        self, observation: MissionObservation, now: float, reason: str = "操作员按 0 正常结束"
    ) -> ClusterCollectionDecision:
        self._validate_observation(observation, require_attitude=False)
        self.awaiting_gripper_action = None
        self._enter(ClusterCollectionState.RETURNING, observation, now)
        # 如果当前已比启动深度更浅，只在原地等待稳定；绝不为了
        # “回到启动深度”而重新下潜。这里从正常结束请求时开始计稳定时间。
        if (
            self.start_depth_m is not None
            and observation.depth_m - self.start_depth_m
            <= self.config.return_tolerance_m
        ):
            self.settled_since = float(now)
        return self._decision(MotionCommand.neutral(), observation, reason)

    def abort(
        self, reason: str, observation: MissionObservation | None = None
    ) -> ClusterCollectionDecision:
        self.state = ClusterCollectionState.ABORTED
        self.outcome = "aborted"
        self.message = reason
        self.awaiting_gripper_action = None
        return self._decision(MotionCommand.neutral(), observation, reason)

    def delay_timers(self, duration_s: float) -> None:
        if not math.isfinite(duration_s) or duration_s < 0.0:
            raise ClusterCollectionError("暂停时长必须是非负有限数")
        self.state_started_at += duration_s
        if self.settled_since is not None:
            self.settled_since += duration_s
        if self.last_target_seen_at is not None:
            self.last_target_seen_at += duration_s
        self.depth_history = deque(
            (sample_time + duration_s, depth)
            for sample_time, depth in self.depth_history
        )

    def _start_probe(
        self, observation: MissionObservation, now: float, *, purpose: str
    ) -> None:
        self.probe_purpose = purpose
        self.probe_start_depth_m = float(observation.depth_m)
        self.depth_history.clear()
        self.depth_history.append((float(now), float(observation.depth_m)))
        self.bottom_stable_duration_s = 0.0
        self.bottom_depth_span_m = 0.0
        state = (
            ClusterCollectionState.GRASP_DESCENDING
            if purpose == "grasp"
            else ClusterCollectionState.PROBING_BOTTOM
        )
        self._enter(state, observation, now)

    def _step_probing_bottom(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        if self.probe_start_depth_m is None:
            return self.abort("触底基准深度缺失", observation)
        if now - self.state_started_at >= self.config.bottom_probe_timeout_s:
            return self.abort("下潜触底等待超时", observation)
        probe = update_bottom_probe(
            self.depth_history,
            now=now,
            current_depth_m=observation.depth_m,
            probe_start_depth_m=self.probe_start_depth_m,
            stable_duration_required_s=self.config.bottom_detection_stable_s,
            stable_depth_tolerance_m=self.config.bottom_detection_depth_tolerance_m,
            minimum_descent_m=self.config.bottom_detection_minimum_descent_m,
            neutral_confirmation_s=self.config.bottom_neutral_confirmation_s,
        )
        self.bottom_stable_duration_s = probe.stable_duration_s
        self.bottom_depth_span_m = probe.depth_span_m
        if probe.confirmed:
            assert probe.bottom_depth_m is not None
            bottom = probe.bottom_depth_m
            self.bottom_depth_m = bottom
            if self.probe_purpose == "grasp":
                self._request_gripper(
                    ClusterCollectionState.CLOSING_GRIPPER,
                    "close",
                    observation,
                    now,
                )
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    f"盲抓下降已触底 {bottom:.2f}m，升沉回中并请求闭爪",
                    gripper_action="close",
                )
            self.target_depth_m = max(
                float(self.start_depth_m or 0.0), bottom - self.config.bottom_clearance_m
            )
            self.clearance_purpose = "search"
            self._enter(ClusterCollectionState.CLEARING_BOTTOM, observation, now)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"已记录底深 {bottom:.2f}m，上浮 {self.config.bottom_clearance_m:.2f}m",
            )
        return self._decision(
            MotionCommand(vertical=0.0 if probe.hold_neutral else -self.descent_command),
            observation,
            f"下潜触底：深度={observation.depth_m:.2f}m，"
            f"已下潜={max(0.0, probe.descended_m):.2f}m，"
            f"稳定={self.bottom_stable_duration_s:.1f}/"
            f"{self.config.bottom_detection_stable_s:.1f}s，"
            + (
                "ALT_HOLD 回中确认"
                if probe.hold_neutral
                else f"power=-{self.descent_command:.2f}"
            ),
        )

    def _step_clearing_bottom(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        if self.bottom_depth_m is None or self.target_depth_m is None:
            return self.abort("离底基准缺失", observation)
        if now - self.state_started_at >= self.config.bottom_clearance_timeout_s:
            return self.abort("触底后离底上浮超时", observation)
        clearance = update_bottom_clearance(
            now=now,
            current_depth_m=observation.depth_m,
            bottom_depth_m=self.bottom_depth_m,
            target_depth_m=self.target_depth_m,
            bottom_depth_tolerance_m=self.config.bottom_detection_depth_tolerance_m,
            clearance_tolerance_m=self.config.bottom_clearance_tolerance_m,
            settle_duration_s=self.config.bottom_clearance_settle_s,
            up_command=self.config.bottom_clearance_up_command,
            settled_since=self.settled_since,
            reference_depth_m=self.clearance_reference_depth_m,
        )
        self.settled_since = clearance.settled_since
        self.clearance_reference_depth_m = clearance.reference_depth_m
        if clearance.unsafe_depth_increase:
            return self.abort("离底时深度反而增大，拒绝继续施力", observation)
        if clearance.reached_target:
            if clearance.settled:
                if self.clearance_purpose == "after_grasp":
                    self._enter(ClusterCollectionState.TRANSFER_TO_NET, observation, now)
                    return self._decision(
                        MotionCommand.neutral(),
                        observation,
                        "闭爪离底已稳定，进入网兜转运预留状态",
                    )
                self.search_cycle += 1
                self._clear_cluster_lock()
                self._enter(ClusterCollectionState.SCANNING, observation, now)
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    f"已离底 {max(0.0, self.bottom_depth_m - observation.depth_m):.2f}m "
                    "并由 ALT_HOLD 稳定，开始 360° 扫描",
                )
            return self._decision(MotionCommand.neutral(), observation, "已到离底深度，等待 ALT_HOLD 稳定")
        return self._decision(
            MotionCommand(vertical=clearance.vertical_command),
            observation,
            f"上浮离底：{observation.depth_m:.2f} -> {self.target_depth_m:.2f}m",
        )

    def _step_scanning(
        self, observation: MissionObservation, now: float, *, consume_frame: bool
    ) -> ClusterCollectionDecision:
        if not observation.attitude_valid:
            return self.abort("扫描时姿态遥测无效", observation)
        if now - self.state_started_at >= self.config.scan_timeout_s:
            return self.abort("360° 扫描超时", observation)
        if self.previous_yaw_deg is None:
            self.previous_yaw_deg = observation.yaw_deg
        else:
            delta = signed_yaw_delta_deg(self.previous_yaw_deg, observation.yaw_deg)
            self.previous_yaw_deg = observation.yaw_deg
            if abs(delta) > self.config.maximum_yaw_step_deg:
                return self.abort(f"航向单次跳变 {delta:.1f}°", observation)
            self.scan_progress_deg += max(0.0, delta)
        if self.scan_progress_deg >= self.config.scan_angle_deg - self.config.scan_tolerance_deg:
            self._enter(ClusterCollectionState.SEARCH_ADVANCING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "360° 未确认 6 个扇贝群，准备前进 2s")
        return self._decision(
            MotionCommand(yaw=self.config.scan_yaw_command),
            observation,
            f"离底扫描 {self.scan_progress_deg:.1f}/{self.config.scan_angle_deg:.0f}°",
        )

    def _step_search_advance(
        self, observation: MissionObservation, now: float, *, consume_frame: bool
    ) -> ClusterCollectionDecision:
        elapsed = now - self.state_started_at
        if elapsed >= self.config.search_advance_duration_s:
            self._start_probe(observation, now, purpose="search")
            return self._decision(
                MotionCommand.neutral(), observation, "无目标前进完成，重新触底标定地形"
            )
        return self._decision(
            MotionCommand(forward=self.config.search_advance_command),
            observation,
            f"无群体前进 {elapsed:.1f}/{self.config.search_advance_duration_s:.1f}s；"
            "本段无下视测距，无法连续跟随起伏地形",
        )

    def _update_acquisition(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision | None:
        clusters = cluster_scallops(
            observation.detections,
            observation.frame_width,
            observation.frame_height,
            label=self.config.target_label,
            link_distance_ratio=self.config.cluster_link_distance_ratio,
        )
        candidate = select_cluster(
            clusters,
            minimum_count=self.config.minimum_cluster_count,
            aim_x_ratio=self.config.aim_x_ratio,
        )
        if candidate is None:
            self.candidate_history.append(False)
            return None
        center = (candidate.center_x_ratio, candidate.center_y_ratio)
        if (
            self.candidate_center is not None
            and math.hypot(center[0] - self.candidate_center[0], center[1] - self.candidate_center[1])
            > self.config.maximum_tracking_jump_ratio
        ):
            self.candidate_history.clear()
        self.candidate_center = center
        self.candidate_history.append(True)
        if sum(self.candidate_history) < self.config.acquisition_required_hits:
            return None
        self.tracked_cluster = candidate
        self.smoothed_center = center
        self.last_target_seen_at = now
        self.consecutive_misses = 0
        self.aligned_frames = 0
        self.descent_history.clear()
        self._enter(ClusterCollectionState.ALIGNING, observation, now)
        return self._step_aligning(observation, now)

    def _update_tracking(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision | None:
        if self.smoothed_center is None:
            return self.abort("群体跟踪中心缺失", observation)
        clusters = cluster_scallops(
            observation.detections,
            observation.frame_width,
            observation.frame_height,
            label=self.config.target_label,
            link_distance_ratio=self.config.cluster_link_distance_ratio,
        )
        match = None
        if clusters:
            match = min(
                clusters,
                key=lambda item: math.hypot(
                    item.center_x_ratio - self.smoothed_center[0],
                    item.center_y_ratio - self.smoothed_center[1],
                ),
            )
            distance = math.hypot(
                match.center_x_ratio - self.smoothed_center[0],
                match.center_y_ratio - self.smoothed_center[1],
            )
            if distance > self.config.maximum_tracking_jump_ratio:
                match = None
        if match is not None:
            alpha = self.config.cluster_center_smoothing_alpha
            self.smoothed_center = (
                (1.0 - alpha) * self.smoothed_center[0] + alpha * match.center_x_ratio,
                (1.0 - alpha) * self.smoothed_center[1] + alpha * match.center_y_ratio,
            )
            self.tracked_cluster = ScallopCluster(
                detections=match.detections,
                center_x_ratio=self.smoothed_center[0],
                center_y_ratio=self.smoothed_center[1],
                union_area_ratio=match.union_area_ratio,
                total_confidence=match.total_confidence,
            )
            self.last_target_seen_at = now
            self.consecutive_misses = 0
            if self.state == ClusterCollectionState.VERIFYING_LOSS:
                self._enter(ClusterCollectionState.ALIGNING, observation, now)
                return self._step_aligning(observation, now)
            return None

        self.consecutive_misses += 1
        if self.state != ClusterCollectionState.VERIFYING_LOSS:
            self.state_before_loss = self.state
            self._enter(ClusterCollectionState.VERIFYING_LOSS, observation, now)
            return self._decision(
                MotionCommand.neutral(), observation, "群体第 1 帧漏检，立即停车但保留目标锁"
            )
        elapsed = math.inf if self.last_target_seen_at is None else now - self.last_target_seen_at
        if (
            self.consecutive_misses >= self.config.loss_required_frames
            and elapsed >= self.config.loss_confirmation_s
        ):
            self._clear_cluster_lock()
            self._enter(ClusterCollectionState.SCANNING, observation, now)
            return self._decision(
                MotionCommand.neutral(), observation, "连续新帧与时间均达阈值，确认丢失原群并重新扫描"
            )
        return self._decision(
            MotionCommand.neutral(),
            observation,
            f"群体漏检 {self.consecutive_misses}/{self.config.loss_required_frames}，"
            f"{elapsed:.2f}/{self.config.loss_confirmation_s:.2f}s，保持停车",
        )

    def _step_aligning(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        cluster = self.tracked_cluster
        if cluster is None:
            return self.abort("对准时群体锁缺失", observation)
        error = cluster.center_x_ratio - self.config.aim_x_ratio
        if abs(error) <= self.config.horizontal_tolerance:
            self.aligned_frames += 1
            if self.aligned_frames >= self.config.alignment_frames:
                self.descent_history.clear()
                self._enter(ClusterCollectionState.APPROACHING, observation, now)
                decision = self._decision(
                    MotionCommand.neutral(), observation, "群中心已连续居中，开始靠近"
                )
                self.last_tracking_decision = decision
                return decision
            decision = self._decision(
                MotionCommand.neutral(), observation, "群中心已进入容差，等待 3 帧稳定"
            )
            self.last_tracking_decision = decision
            return decision
        self.aligned_frames = 0
        magnitude = min(self.config.maximum_yaw_command, abs(error) * self.config.yaw_gain)
        magnitude = max(self.config.minimum_yaw_command, magnitude)
        yaw = self.config.image_yaw_sign * math.copysign(magnitude, error)
        side = "右" if error > 0 else "左"
        turn = "右转" if yaw > 0 else "左转"
        decision = self._decision(
            MotionCommand(yaw=yaw),
            observation,
            f"群中心在画面{side}侧，执行{turn}对准",
        )
        self.last_tracking_decision = decision
        return decision

    def _descent_ready(self, cluster: ScallopCluster, error: float) -> bool:
        centered = abs(error) <= self.config.horizontal_tolerance
        line_ready = cluster.center_y_ratio >= self.config.descent_line_y_ratio
        area_ready = (
            self.config.union_area_threshold is not None
            and cluster.union_area_ratio >= self.config.union_area_threshold
        )
        if self.config.descent_trigger == DescentTrigger.IMAGE_LINE:
            return centered and line_ready
        if self.config.descent_trigger == DescentTrigger.UNION_AREA:
            return centered and area_ready
        return centered and line_ready and area_ready

    def _step_approaching(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        cluster = self.tracked_cluster
        if cluster is None:
            return self.abort("靠近时群体锁缺失", observation)
        error = cluster.center_x_ratio - self.config.aim_x_ratio
        if abs(error) > self.config.realign_threshold:
            self.aligned_frames = 0
            self._enter(ClusterCollectionState.ALIGNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "水平偏差过大，停止前进并重新对准")
        ready = self._descent_ready(cluster, error)
        self.descent_history.append(ready)
        if (
            len(self.descent_history) == self.config.descent_window_frames
            and sum(self.descent_history) >= self.config.descent_required_hits
        ):
            self.grasp_attempts_completed = 0
            self.current_grasp_attempt_index = 1
            self._request_gripper(
                ClusterCollectionState.OPENING_GRIPPER, "open", observation, now
            )
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"下降判据 {self.config.descent_trigger.value} 连续达标，请求开爪",
                gripper_action="open",
            )
        yaw = max(
            -self.config.approach_yaw_command,
            min(
                self.config.approach_yaw_command,
                self.config.image_yaw_sign * error * self.config.yaw_gain,
            ),
        )
        if abs(error) > self.config.horizontal_tolerance:
            forward = 0.0
        elif cluster.center_y_ratio < self.config.far_center_y_ratio:
            forward = self.config.far_forward_command
        elif cluster.center_y_ratio < self.config.descent_line_y_ratio:
            forward = self.config.near_forward_command
        else:
            forward = 0.0
        decision = self._decision(
            MotionCommand(forward=forward, yaw=yaw),
            observation,
            f"靠近扇贝群：center_y={cluster.center_y_ratio:.3f}/"
            f"{self.config.descent_line_y_ratio:.3f}，union={cluster.union_area_ratio:.4f}，"
            f"下降确认={sum(self.descent_history)}/{self.config.descent_window_frames}",
        )
        self.last_tracking_decision = decision
        return decision

    def _step_grasp_holding(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        elapsed = now - self.state_started_at
        if elapsed < self.config.gripper_motion_wait_s:
            return self._decision(
                MotionCommand.neutral(), observation, f"等待闭爪机械动作 {elapsed:.1f}s"
            )
        if self.bottom_depth_m is None:
            return self.abort("抓取后离底基准缺失", observation)
        self.target_depth_m = max(
            float(self.start_depth_m or 0.0), self.bottom_depth_m - self.config.bottom_clearance_m
        )
        self.clearance_purpose = "after_grasp"
        self._enter(ClusterCollectionState.LIFTING_AFTER_GRASP, observation, now)
        return self._decision(MotionCommand.neutral(), observation, "闭爪等待完成，保持闭爪上浮离底")

    def _step_transfer(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        elapsed = now - self.state_started_at
        if elapsed < self.config.transfer_to_net_wait_s:
            return self._decision(
                MotionCommand.neutral(), observation, "TRANSFER_TO_NET 预留：当前只回中等待和记录"
            )
        self._request_gripper(
            ClusterCollectionState.REOPENING_GRIPPER, "reopen", observation, now
        )
        return self._decision(
            MotionCommand.neutral(), observation, "转运预留结束，请求重新开爪", gripper_action="reopen"
        )

    def _step_inter_grab_advance(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        elapsed = now - self.state_started_at
        if elapsed >= self.config.inter_grab_duration_s:
            self.current_grasp_attempt_index = self.grasp_attempts_completed + 1
            self._request_gripper(
                ClusterCollectionState.OPENING_GRIPPER, "open", observation, now
            )
            return self._decision(
                MotionCommand.neutral(), observation, "抓取间短移完成，再次请求开爪", gripper_action="open"
            )
        return self._decision(
            MotionCommand(forward=self.config.inter_grab_forward_command),
            observation,
            f"抓取间短移 {elapsed:.1f}/{self.config.inter_grab_duration_s:.1f}s",
        )

    def _step_leaving_cluster(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        elapsed = now - self.state_started_at
        if elapsed >= self.config.leave_cluster_duration_s:
            self.current_grasp_attempt_index = 0
            self._start_probe(observation, now, purpose="search")
            return self._decision(
                MotionCommand.neutral(), observation, "已离开当前群体，重新触底标定并搜索下一群"
            )
        return self._decision(
            MotionCommand(forward=self.config.leave_cluster_forward_command),
            observation,
            f"离开已处理群体 {elapsed:.1f}/{self.config.leave_cluster_duration_s:.1f}s",
        )

    def _step_returning(
        self, observation: MissionObservation, now: float
    ) -> ClusterCollectionDecision:
        if self.start_depth_m is None:
            return self.abort("回收启动深度缺失", observation)
        if now - self.state_started_at >= self.config.return_timeout_s:
            return self.abort("回到启动深度超时", observation)
        error = observation.depth_m - self.start_depth_m
        if error <= self.config.return_tolerance_m:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= self.config.return_settle_s:
                self.state = ClusterCollectionState.COMPLETE
                self.outcome = "completed"
                self.message = "已回到启动深度，等待 ROS 正常上锁"
                return self._decision(MotionCommand.neutral(), observation, self.message)
            return self._decision(MotionCommand.neutral(), observation, "已进入回收容差，等待稳定")
        self.settled_since = None
        vertical = min(
            self.config.return_maximum_command,
            max(self.config.return_minimum_command, error * self.config.return_gain),
        )
        return self._decision(
            MotionCommand(vertical=max(0.0, vertical)),
            observation,
            f"正常回收 {observation.depth_m:.2f} -> {self.start_depth_m:.2f}m",
        )

    def _request_gripper(
        self,
        state: ClusterCollectionState,
        action: str,
        observation: MissionObservation,
        now: float,
    ) -> None:
        self.awaiting_gripper_action = action
        self._enter(state, observation, now)

    def _enter(
        self, state: ClusterCollectionState, observation: MissionObservation, now: float
    ) -> None:
        self.state = state
        self.state_started_at = float(now)
        self.settled_since = None
        if state == ClusterCollectionState.SCANNING:
            self.scan_progress_deg = 0.0
            self.previous_yaw_deg = observation.yaw_deg
            self.candidate_history.clear()
            self.candidate_center = None
        if state == ClusterCollectionState.ALIGNING:
            self.aligned_frames = 0
        if state == ClusterCollectionState.APPROACHING:
            self.descent_history.clear()

    def _clear_cluster_lock(self) -> None:
        self.candidate_center = None
        self.candidate_history.clear()
        self.tracked_cluster = None
        self.smoothed_center = None
        self.last_target_seen_at = None
        self.consecutive_misses = 0
        self.aligned_frames = 0
        self.descent_history.clear()
        self.last_tracking_decision = None
        self.state_before_loss = None

    def _decision(
        self,
        motion: MotionCommand,
        observation: MissionObservation | None,
        message: str,
        *,
        gripper_action: str | None = None,
    ) -> ClusterCollectionDecision:
        self.message = message
        cluster = self.tracked_cluster
        current_depth = None if observation is None else float(observation.depth_m)
        horizontal_error = (
            None
            if cluster is None
            else cluster.center_x_ratio - self.config.aim_x_ratio
        )
        return ClusterCollectionDecision(
            state=self.state,
            motion=motion,
            message=message,
            cluster=cluster,
            visible_scallop_count=self.last_visible_scallop_count,
            scan_progress_deg=(
                self.scan_progress_deg
                if self.state == ClusterCollectionState.SCANNING
                else None
            ),
            search_cycle=self.search_cycle,
            grasp_attempt_index=self.current_grasp_attempt_index,
            completed_cluster_count=self.completed_cluster_count,
            descent_trigger_mode=self.config.descent_trigger.value,
            current_depth_m=current_depth,
            target_depth_m=self.target_depth_m,
            horizontal_error=horizontal_error,
            gripper_action=gripper_action,
            gripper_command_accepted=self.last_gripper_accepted,
            outcome=self.outcome,
        )

    def _validate_observation(
        self, observation: MissionObservation, *, require_attitude: bool
    ) -> None:
        if not observation.depth_valid or not math.isfinite(observation.depth_m):
            raise ClusterCollectionError("深度遥测无效")
        if require_attitude and (
            not observation.attitude_valid or not math.isfinite(observation.yaw_deg)
        ):
            raise ClusterCollectionError("航向遥测无效")
        if observation.frame_width <= 0 or observation.frame_height <= 0:
            raise ClusterCollectionError("图像尺寸无效")


STATE_DISPLAY_NAMES = {
    ClusterCollectionState.IDLE: "等待启动",
    ClusterCollectionState.PROBING_BOTTOM: "搜索前触底标定",
    ClusterCollectionState.CLEARING_BOTTOM: "上浮 15cm 离底",
    ClusterCollectionState.SCANNING: "360° 群体扫描",
    ClusterCollectionState.SEARCH_ADVANCING: "无群体前进",
    ClusterCollectionState.ALIGNING: "群中心偏航对准",
    ClusterCollectionState.APPROACHING: "靠近扇贝群",
    ClusterCollectionState.VERIFYING_LOSS: "原群漏检确认",
    ClusterCollectionState.OPENING_GRIPPER: "开爪确认",
    ClusterCollectionState.GRASP_DESCENDING: "盲抓下降触底",
    ClusterCollectionState.CLOSING_GRIPPER: "闭爪确认",
    ClusterCollectionState.GRASP_HOLDING: "等待闭爪动作",
    ClusterCollectionState.LIFTING_AFTER_GRASP: "闭爪离底",
    ClusterCollectionState.TRANSFER_TO_NET: "网兜转运预留",
    ClusterCollectionState.REOPENING_GRIPPER: "重新开爪",
    ClusterCollectionState.INTER_GRAB_ADVANCING: "抓取间短移",
    ClusterCollectionState.LEAVING_CLUSTER: "离开已处理群体",
    ClusterCollectionState.RETURNING: "回到启动深度",
    ClusterCollectionState.COMPLETE: "任务完成",
    ClusterCollectionState.ABORTED: "任务中止",
}


class ClusterStateReporter:
    """输出状态进入事件和最多 1Hz 的现场进度。"""

    def __init__(self) -> None:
        self.last_state: ClusterCollectionState | None = None
        self.last_report_at: float | None = None

    def report(
        self,
        decision: ClusterCollectionDecision,
        observation: MissionObservation,
        now: float,
    ) -> None:
        if decision.state != self.last_state:
            if self.last_state is not None:
                print(
                    f"[状态切换] {self.last_state.name} -> {decision.state.name} | "
                    f"{decision.message}",
                    flush=True,
                )
            print(
                f"[进入状态] {decision.state.name}"
                f"（{STATE_DISPLAY_NAMES[decision.state]}）| {decision.message}",
                flush=True,
            )
            self.last_state = decision.state
            self.last_report_at = now
            return
        if self.last_report_at is None or now - self.last_report_at >= 1.0:
            self.last_report_at = now
            cluster = decision.cluster
            cluster_text = (
                "cluster=none"
                if cluster is None
                else (
                    f"cluster={cluster.count} | center=({cluster.center_x_ratio:.3f},"
                    f"{cluster.center_y_ratio:.3f}) | union={cluster.union_area_ratio:.4f}"
                )
            )
            print(
                f"[状态进度] {decision.state.name} | depth={observation.depth_m:.2f}m | "
                f"{cluster_text} | grasp={decision.grasp_attempt_index}/3 | "
                f"groups={decision.completed_cluster_count} | {decision.message}",
                flush=True,
            )
