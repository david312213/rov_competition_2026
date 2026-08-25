"""搜索—接近水池测试的纯 Python 状态机。

这个模块不导入 ROS、YOLO、Pygame 或 MAVLink，因此可在不连实艇的
电脑上验证搜索、漏检确认和回收逻辑。它本身不发送机械爪命令；人工
标定运行时只在操作员显式按 C 后调用机械爪服务。它不替换
:mod:`rov_competition.mission` 中的正式自主抓取状态机。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Mapping, Sequence

import yaml

from .domain import Detection, MissionObservation, MotionCommand
from .dataset_control import MOVEMENT_KEYS, motion_from_keys
from .mission import signed_yaw_delta_deg


class SearchTestError(RuntimeError):
    """搜索测试配置、观测或状态不能安全继续。"""


class SearchTestState(str, Enum):
    """搜索—接近测试的状态。"""

    IDLE = "idle"
    DESCENDING = "descending"
    SCANNING = "scanning"
    ADVANCING = "advancing"
    ALIGNING = "aligning"
    APPROACHING = "approaching"
    MANUAL_CALIBRATION = "manual_calibration"
    VERIFYING_LOSS = "verifying_loss"
    RESETTING_FOR_RESCAN = "resetting_for_rescan"
    RETURNING = "returning"
    COMPLETE = "complete"
    ABORTED = "aborted"


class SearchWorkflow(str, Enum):
    """稳定对准后的两种互斥流程。"""

    AUTO_APPROACH = "auto_approach"
    MANUAL_GRASP_CALIBRATION = "manual_grasp_calibration"


@dataclass(frozen=True)
class SearchApproachConfig:
    """水池测试中可验证的控制、跟踪和超时参数。"""

    maximum_operation_depth_m: float = 20.0
    descent_tolerance_m: float = 0.05
    descent_gain: float = 0.8
    descent_minimum_command: float = 0.10
    descent_slowdown_distance_m: float = 0.20
    descent_settle_s: float = 1.0
    descent_timeout_s: float = 45.0

    scan_yaw_command: float = 0.20
    scan_angle_deg: float = 360.0
    scan_tolerance_deg: float = 3.0
    scan_timeout_s: float = 120.0
    maximum_yaw_step_deg: float = 45.0
    maximum_search_cycles: int = 3

    advance_forward_command: float = 0.40
    advance_duration_s: float = 2.0

    aim_x_ratio: float = 0.50
    horizontal_tolerance: float = 0.08
    realign_threshold: float = 0.12
    alignment_frames: int = 3
    yaw_gain: float = 0.8
    minimum_yaw_command: float = 0.10
    maximum_yaw_command: float = 0.20

    far_area_ratio: float = 0.07
    stop_area_ratio: float = 0.10
    far_forward_command: float = 0.20
    near_forward_command: float = 0.10
    approach_yaw_command: float = 0.10
    maximum_target_area_ratio: float = 0.80

    acquisition_window_frames: int = 5
    acquisition_required_hits: int = 3
    loss_required_frames: int = 3
    loss_confirmation_s: float = 0.30
    finish_window_frames: int = 5
    finish_required_hits: int = 4
    maximum_tracking_jump_ratio: float = 0.25
    minimum_tracking_iou: float = 0.05

    manual_initial_command: float = 0.10
    manual_minimum_command: float = 0.05
    manual_maximum_command: float = 0.40
    manual_command_step: float = 0.01
    minimum_success_samples: int = 5

    return_gain: float = 0.50
    return_maximum_command: float = 0.20
    return_tolerance_m: float = 0.05
    return_settle_s: float = 1.0
    return_timeout_s: float = 60.0

    def __post_init__(self) -> None:
        """在创建状态机前拒绝互相矛盾的参数。"""

        positive = (
            self.maximum_operation_depth_m,
            self.descent_tolerance_m,
            self.descent_gain,
            self.descent_minimum_command,
            self.descent_slowdown_distance_m,
            self.descent_settle_s,
            self.descent_timeout_s,
            self.scan_yaw_command,
            self.scan_angle_deg,
            self.scan_tolerance_deg,
            self.scan_timeout_s,
            self.maximum_yaw_step_deg,
            self.advance_forward_command,
            self.advance_duration_s,
            self.horizontal_tolerance,
            self.realign_threshold,
            self.yaw_gain,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.far_area_ratio,
            self.stop_area_ratio,
            self.far_forward_command,
            self.near_forward_command,
            self.approach_yaw_command,
            self.maximum_target_area_ratio,
            self.loss_confirmation_s,
            self.maximum_tracking_jump_ratio,
            self.minimum_tracking_iou,
            self.manual_initial_command,
            self.manual_minimum_command,
            self.manual_maximum_command,
            self.manual_command_step,
            self.return_gain,
            self.return_maximum_command,
            self.return_tolerance_m,
            self.return_settle_s,
            self.return_timeout_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise SearchTestError("搜索测试的数值参数必须是大于 0 的有限数")
        commands = (
            self.descent_minimum_command,
            self.scan_yaw_command,
            self.advance_forward_command,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.far_forward_command,
            self.near_forward_command,
            self.approach_yaw_command,
            self.return_maximum_command,
            self.manual_initial_command,
            self.manual_minimum_command,
            self.manual_maximum_command,
            self.manual_command_step,
        )
        if any(value > 1.0 for value in commands):
            raise SearchTestError("归一化运动指令不能超过 1.0")
        if not 0.0 < self.far_area_ratio < self.stop_area_ratio < 1.0:
            raise SearchTestError("框面积阈值必须满足 0 < far < stop < 1")
        if self.stop_area_ratio >= self.maximum_target_area_ratio:
            raise SearchTestError("停止面积必须小于异常大框阈值")
        if not 0.0 < self.horizontal_tolerance < self.realign_threshold < 1.0:
            raise SearchTestError("水平容差必须小于重新对准阈值")
        if not 0.0 < self.aim_x_ratio < 1.0:
            raise SearchTestError("画面瞄准点必须位于画面内")
        if self.minimum_yaw_command > self.maximum_yaw_command:
            raise SearchTestError("最小偏航指令不能大于最大值")
        integer_values = (
            self.maximum_search_cycles,
            self.alignment_frames,
            self.acquisition_window_frames,
            self.acquisition_required_hits,
            self.loss_required_frames,
            self.finish_window_frames,
            self.finish_required_hits,
            self.minimum_success_samples,
        )
        if any(value <= 0 for value in integer_values):
            raise SearchTestError("帧数和搜索轮数必须是正整数")
        if self.acquisition_required_hits > self.acquisition_window_frames:
            raise SearchTestError("目标确认命中数不能超过窗口帧数")
        if self.finish_required_hits > self.finish_window_frames:
            raise SearchTestError("完成命中数不能超过窗口帧数")

        if not (
            self.manual_minimum_command
            <= self.manual_initial_command
            <= self.manual_maximum_command
        ):
            raise SearchTestError("人工精调初始 power 必须位于最小值与最大值之间")

    def readiness_errors(
        self,
        *,
        robot_command_limit: float,
        workflow: SearchWorkflow = SearchWorkflow.AUTO_APPROACH,
        descent_maximum_command: float | None = None,
    ) -> tuple[str, ...]:
        """列出 robot.yaml 不足以执行测试的原因。"""

        required = max(
            self.advance_forward_command,
            self.scan_yaw_command,
            self.far_forward_command,
            self.maximum_yaw_command,
            self.return_maximum_command,
        )
        if workflow == SearchWorkflow.MANUAL_GRASP_CALIBRATION:
            required = max(required, self.manual_maximum_command)
        if descent_maximum_command is not None:
            if (
                not math.isfinite(descent_maximum_command)
                or not 0.10 <= descent_maximum_command <= 0.80
            ):
                return ("下潜最大 power 必须在 0.10..0.80",)
            required = max(required, descent_maximum_command)
        if robot_command_limit + 1e-9 < required:
            return (
                f"robot.yaml command_limit={robot_command_limit:.2f} 小于测试所需 {required:.2f}",
            )
        return ()


@dataclass
class ManualCalibrationControl:
    """人工标定阶段的按键与 power；不依赖 ROS 或 Pygame。"""

    config: SearchApproachConfig
    held_keys: set[str] = field(default_factory=set)
    strength: float = field(init=False)

    def __post_init__(self) -> None:
        self.strength = self.config.manual_initial_command

    def press(self, key: str) -> bool:
        value = str(key).lower()
        if value not in MOVEMENT_KEYS:
            return False
        before = len(self.held_keys)
        self.held_keys.add(value)
        return len(self.held_keys) != before

    def release(self, key: str) -> bool:
        value = str(key).lower()
        if value not in self.held_keys:
            return False
        self.held_keys.remove(value)
        return True

    def clear(self) -> None:
        self.held_keys.clear()

    def adjust(self, direction: int) -> float:
        if direction not in (-1, 1):
            raise SearchTestError("power 调整方向只能是 -1 或 1")
        value = self.strength + direction * self.config.manual_command_step
        self.strength = round(
            max(
                self.config.manual_minimum_command,
                min(self.config.manual_maximum_command, value),
            ),
            6,
        )
        return self.strength

    def motion(self) -> MotionCommand:
        return motion_from_keys(
            self.held_keys,
            self.strength,
            self.config.manual_maximum_command,
        )


@dataclass(frozen=True)
class SearchTestDecision:
    """状态机在一个 20 Hz 控制周期的决定。"""

    state: SearchTestState
    motion: MotionCommand
    message: str
    selected_target: Detection | None = None
    target_area_ratio: float | None = None
    horizontal_error: float | None = None
    current_depth_m: float | None = None
    target_depth_m: float | None = None
    scan_progress_deg: float | None = None
    search_cycle: int = 0
    outcome: str = "running"


def _mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise SearchTestError(f"{name} 必须是 YAML 映射")
    return value


def load_search_approach_config(path: str | Path) -> SearchApproachConfig:
    """读取简单的水池测试配置。"""

    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise SearchTestError(f"搜索测试配置不存在: {config_path}")
    content = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    section = _mapping(_mapping(content, "配置根节点").get("search_test"), "search_test")
    defaults = SearchApproachConfig()
    values: dict[str, object] = {}
    for field_name in defaults.__dataclass_fields__:
        raw = section.get(field_name, getattr(defaults, field_name))
        if isinstance(getattr(defaults, field_name), int):
            if isinstance(raw, bool) or int(raw) != float(raw):
                raise SearchTestError(f"search_test.{field_name} 必须是整数")
            values[field_name] = int(raw)
        else:
            values[field_name] = float(raw)
    return SearchApproachConfig(**values)


class SearchApproachMission:
    """共用搜索与对准，并分流到自动接近或人工标定的测试状态机。"""

    def __init__(
        self,
        config: SearchApproachConfig,
        target_labels: Sequence[str],
        workflow: SearchWorkflow = SearchWorkflow.AUTO_APPROACH,
    ) -> None:
        labels = frozenset(str(label).strip() for label in target_labels if str(label).strip())
        if not labels:
            raise SearchTestError("搜索测试至少需要一个允许目标类别")
        self.config = config
        self.target_labels = labels
        self.workflow = SearchWorkflow(workflow)
        self.state = SearchTestState.IDLE
        self.start_depth_m: float | None = None
        self.target_depth_m: float | None = None
        self.descent_maximum_command = 0.20
        self.state_started_at = 0.0
        self.settled_since: float | None = None
        self.search_cycle = 0
        self.scan_progress_deg = 0.0
        self.previous_yaw_deg: float | None = None
        self.last_frame_id = -1
        self.candidate: Detection | None = None
        self.candidate_history: deque[bool] = deque(
            maxlen=config.acquisition_window_frames
        )
        self.tracked: Detection | None = None
        self.last_target_seen_at: float | None = None
        self.consecutive_misses = 0
        self.aligned_frames = 0
        self.finish_history: deque[bool] = deque(maxlen=config.finish_window_frames)
        self.message = "测试未启动"
        self.outcome = "none"

    def start(
        self,
        observation: MissionObservation,
        *,
        relative_descent_m: float,
        descent_maximum_command: float,
        now: float,
    ) -> SearchTestDecision:
        """记录启动深度并进入相对下潜。"""

        if self.state not in {SearchTestState.IDLE, SearchTestState.COMPLETE}:
            raise SearchTestError("搜索测试已在运行")
        self._validate_observation(observation, require_perception=True)
        if not math.isfinite(relative_descent_m) or relative_descent_m <= 0.0:
            raise SearchTestError("相对下潜距离必须大于 0")
        if (
            not math.isfinite(descent_maximum_command)
            or not 0.10 <= descent_maximum_command <= 0.80
        ):
            raise SearchTestError("下潜最大 power 必须在 0.10..0.80")
        target_depth = observation.depth_m + relative_descent_m
        if target_depth > self.config.maximum_operation_depth_m:
            raise SearchTestError(
                f"目标深度 {target_depth:.2f} m 超过配置上限 "
                f"{self.config.maximum_operation_depth_m:.2f} m"
            )
        self.start_depth_m = float(observation.depth_m)
        self.target_depth_m = target_depth
        self.descent_maximum_command = float(descent_maximum_command)
        self.outcome = "running"
        self.last_frame_id = observation.frame_id
        self._clear_target()
        self._enter(SearchTestState.DESCENDING, observation, now)
        return self._decision(MotionCommand.neutral(), observation, "已记录启动深度，开始相对下潜")

    def request_normal_finish(
        self, observation: MissionObservation, now: float, reason: str = "操作员正常结束"
    ) -> SearchTestDecision:
        """由 Space 或接近成功进入有深度反馈的回收。"""

        self._validate_observation(observation, require_perception=False)
        if self.state in {SearchTestState.COMPLETE, SearchTestState.ABORTED}:
            return self._decision(MotionCommand.neutral(), observation, self.message)
        self._enter(SearchTestState.RETURNING, observation, now)
        self.message = reason
        return self._decision(MotionCommand.neutral(), observation, reason)

    def request_rescan(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        """人工标定时先回到本轮搜索深度，再重新执行完整扫描。"""

        if self.workflow != SearchWorkflow.MANUAL_GRASP_CALIBRATION:
            raise SearchTestError("只有人工抓取标定流程支持重新搜索")
        if self.state != SearchTestState.MANUAL_CALIBRATION:
            raise SearchTestError("只有进入人工精调后才能重新搜索")
        self._validate_observation(observation, require_perception=False)
        if self.target_depth_m is None:
            raise SearchTestError("本轮搜索深度缺失")
        self._clear_target()
        self._enter(SearchTestState.RESETTING_FOR_RESCAN, observation, now)
        return self._decision(
            MotionCommand.neutral(), observation, "先回到本轮搜索深度，再重新扫描"
        )

    def abort(
        self, reason: str, observation: MissionObservation | None = None
    ) -> SearchTestDecision:
        """中止任务并只返回中位；实际急停由 ROS 层请求。"""

        self.state = SearchTestState.ABORTED
        self.outcome = "aborted"
        self.message = reason
        return self._decision(MotionCommand.neutral(), observation, reason)

    def delay_timers(self, paused_duration_s: float) -> None:
        """失去窗口焦点后将所有超时基准向后平移。

        暂停期间持续发送中位，不应被计入“前进 2 秒”、扫描超时、
        漏检 0.30 秒或深度稳定时间。
        """

        if not math.isfinite(paused_duration_s) or paused_duration_s < 0.0:
            raise SearchTestError("暂停时长必须是非负有限数")
        self.state_started_at += paused_duration_s
        if self.settled_since is not None:
            self.settled_since += paused_duration_s
        if self.last_target_seen_at is not None:
            self.last_target_seen_at += paused_duration_s

    def step(self, observation: MissionObservation, now: float) -> SearchTestDecision:
        """使用一份最新观测执行一个控制周期。"""

        if self.state in {SearchTestState.IDLE, SearchTestState.COMPLETE, SearchTestState.ABORTED}:
            return self._decision(MotionCommand.neutral(), observation, self.message)
        try:
            self._validate_observation(observation, require_perception=False)
        except SearchTestError as exc:
            return self.abort(str(exc), observation)
        if not math.isfinite(now):
            return self.abort("控制时间无效", observation)

        if observation.frame_id < self.last_frame_id:
            return self.abort("图像帧编号倒退", observation)
        new_frame = observation.frame_id > self.last_frame_id
        if new_frame:
            self.last_frame_id = observation.frame_id

        if self.state == SearchTestState.DESCENDING:
            return self._step_descending(observation, now)
        if self.state == SearchTestState.RETURNING:
            return self._step_returning(observation, now)
        if self.state == SearchTestState.RESETTING_FOR_RESCAN:
            return self._step_resetting_for_rescan(observation, now)
        if not observation.perception_valid:
            return self._decision(MotionCommand.neutral(), observation, "没有新鲜图像，回中等待")
        if not new_frame:
            return self._decision(MotionCommand.neutral(), observation, "图像帧未更新，回中等待")

        if self.state == SearchTestState.MANUAL_CALIBRATION:
            return self._step_manual_calibration(observation, now)

        target_transition = self._update_target(observation, now)
        if target_transition is not None:
            return target_transition

        if self.state == SearchTestState.SCANNING:
            return self._step_scanning(observation, now)
        if self.state == SearchTestState.ADVANCING:
            return self._step_advancing(observation, now)
        if self.state == SearchTestState.ALIGNING:
            return self._step_aligning(observation, now)
        if self.state == SearchTestState.APPROACHING:
            return self._step_approaching(observation, now)
        if self.state == SearchTestState.VERIFYING_LOSS:
            return self._decision(MotionCommand.neutral(), observation, "漏检确认中，保持停车")
        return self.abort(f"未处理状态: {self.state.value}", observation)

    def _step_resetting_for_rescan(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        """回到本轮下潜完成时的深度；这里允许小幅上升或下潜。"""

        if self.target_depth_m is None:
            return self.abort("重新搜索深度缺失", observation)
        if now - self.state_started_at >= self.config.return_timeout_s:
            return self.abort("回到本轮搜索深度超时", observation)
        error = self.target_depth_m - observation.depth_m
        if abs(error) <= self.config.descent_tolerance_m:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= self.config.descent_settle_s:
                self.search_cycle = 1
                self._enter(SearchTestState.SCANNING, observation, now)
                return self._decision(
                    MotionCommand.neutral(), observation, "已回到本轮搜索深度，重新扫描"
                )
            return self._decision(
                MotionCommand.neutral(), observation, "已进入本轮搜索深度容差，等待稳定"
            )
        self.settled_since = None
        magnitude = min(
            self.config.return_maximum_command,
            max(self.config.descent_minimum_command, abs(error) * self.config.descent_gain),
        )
        vertical = -magnitude if error > 0.0 else magnitude
        return self._decision(
            MotionCommand(vertical=vertical),
            observation,
            f"返回搜索深度 {observation.depth_m:.2f} -> {self.target_depth_m:.2f} m",
        )

    def _step_descending(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        if self.target_depth_m is None:
            return self.abort("下潜目标深度缺失", observation)
        if now - self.state_started_at >= self.config.descent_timeout_s:
            return self.abort("相对下潜超时", observation)
        error = self.target_depth_m - observation.depth_m
        if abs(error) <= self.config.descent_tolerance_m:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= self.config.descent_settle_s:
                self.search_cycle = 1
                self._enter(SearchTestState.SCANNING, observation, now)
                return self._decision(MotionCommand.neutral(), observation, "下潜完成，开始向右扫描")
            return self._decision(MotionCommand.neutral(), observation, "进入深度容差，等待稳定")
        self.settled_since = None
        # “下潜最大 power”应该真正在距离目标较远时生效。旧式为
        # abs(error) * descent_gain：默认相对下潜 0.30m 时，即使输入
        # 0.40，实际也只会发 0.24。现在在距目标大于等于减速距离时
        # 发送选定的最大值；进入最后一段后按剩余距离线性减速。
        slowdown_ratio = min(
            1.0,
            abs(error) / self.config.descent_slowdown_distance_m,
        )
        magnitude = self.descent_maximum_command * slowdown_ratio
        magnitude = max(self.config.descent_minimum_command, magnitude)
        # vertical > 0 是上升；目标更深时发负值，过深时允许小幅上升纠正。
        vertical = -magnitude if error > 0.0 else magnitude
        return self._decision(
            MotionCommand(vertical=vertical),
            observation,
            f"下潜 {observation.depth_m:.2f}/{self.target_depth_m:.2f} m",
        )

    def _step_scanning(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        if now - self.state_started_at >= self.config.scan_timeout_s:
            return self.abort("航向360°扫描超时", observation)
        if self.previous_yaw_deg is None:
            self.previous_yaw_deg = observation.yaw_deg
        else:
            delta = signed_yaw_delta_deg(self.previous_yaw_deg, observation.yaw_deg)
            self.previous_yaw_deg = observation.yaw_deg
            if abs(delta) > self.config.maximum_yaw_step_deg:
                return self.abort(f"航向单次跳变 {delta:.1f}°，拒绝继续扫描", observation)
            # 右转已由 WASD 实测为 yaw > 0，只累计同向航向变化。
            self.scan_progress_deg += max(0.0, delta)
        if self.scan_progress_deg >= self.config.scan_angle_deg - self.config.scan_tolerance_deg:
            self._enter(SearchTestState.ADVANCING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "360°扫描无目标，准备前进 2 秒")
        return self._decision(
            MotionCommand(yaw=self.config.scan_yaw_command),
            observation,
            f"向右扫描 {self.scan_progress_deg:.1f}°/360°",
        )

    def _step_advancing(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        elapsed = now - self.state_started_at
        if elapsed >= self.config.advance_duration_s:
            if self.search_cycle >= self.config.maximum_search_cycles:
                return self.request_normal_finish(observation, now, "完成最大搜索轮数，回收")
            self.search_cycle += 1
            self._enter(SearchTestState.SCANNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "前进段完成，开始新一轮扫描")
        return self._decision(
            MotionCommand(forward=self.config.advance_forward_command),
            observation,
            f"无目标前进 {elapsed:.1f}/{self.config.advance_duration_s:.1f} s",
        )

    def _step_aligning(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        target = self.tracked
        if target is None:
            return self.abort("对准时目标锁缺失", observation)
        error = self._horizontal_error(target, observation)
        area = target.box.area_ratio(observation.frame_width, observation.frame_height)
        if abs(error) <= self.config.horizontal_tolerance:
            self.aligned_frames += 1
            if self.aligned_frames >= self.config.alignment_frames:
                self.finish_history.clear()
                if self.workflow == SearchWorkflow.MANUAL_GRASP_CALIBRATION:
                    self._enter(SearchTestState.MANUAL_CALIBRATION, observation, now)
                    return self._decision(
                        MotionCommand.neutral(),
                        observation,
                        "目标已稳定居中，自动运动停止；进入 WASD 人工抓取标定",
                        target,
                        area,
                        error,
                    )
                self._enter(SearchTestState.APPROACHING, observation, now)
                return self._decision(MotionCommand.neutral(), observation, "目标已稳定居中，开始接近")
            return self._decision(MotionCommand.neutral(), observation, "目标居中，等待多帧稳定")
        self.aligned_frames = 0
        magnitude = min(self.config.maximum_yaw_command, abs(error) * self.config.yaw_gain)
        magnitude = max(self.config.minimum_yaw_command, magnitude)
        # 画面左侧 error<0 -> 键2的右转 yaw>0；右侧则键1左转。
        yaw = magnitude if error < 0.0 else -magnitude
        return self._decision(
            MotionCommand(yaw=yaw), observation, "仅用偏航对准目标", target, area, error
        )

    def _step_approaching(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        target = self.tracked
        if target is None:
            return self.abort("接近时目标锁缺失", observation)
        error = self._horizontal_error(target, observation)
        area = target.box.area_ratio(observation.frame_width, observation.frame_height)
        if abs(error) > self.config.realign_threshold:
            self.aligned_frames = 0
            self._enter(SearchTestState.ALIGNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "偏差过大，停车重新对准", target, area, error)

        ready = area >= self.config.stop_area_ratio and abs(error) <= self.config.horizontal_tolerance
        self.finish_history.append(ready)
        if (
            len(self.finish_history) == self.config.finish_window_frames
            and sum(self.finish_history) >= self.config.finish_required_hits
        ):
            return self.request_normal_finish(observation, now, "框面积与居中连续达标，测试成功回收")

        yaw = max(-self.config.approach_yaw_command, min(self.config.approach_yaw_command, -error * self.config.yaw_gain))
        # 一旦面积达到停止阈值，第一帧就停止前进；后续只原地确认 4/5 帧，
        # 避免为了“确认稳定”继续向目标冲近。
        if area >= self.config.stop_area_ratio:
            return self._decision(
                MotionCommand(yaw=yaw),
                observation,
                f"面积已达阈值，原地确认 {sum(self.finish_history)}/{self.config.finish_window_frames}",
                target,
                area,
                error,
            )
        forward = (
            self.config.far_forward_command
            if area < self.config.far_area_ratio
            else self.config.near_forward_command
        )
        return self._decision(
            MotionCommand(forward=forward, yaw=yaw),
            observation,
            f"接近目标，面积 {area:.3f}/{self.config.stop_area_ratio:.3f}",
            target,
            area,
            error,
        )

    def _step_manual_calibration(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        """更新当前目标证据，但运动始终由运行时的人工按键决定。"""

        target = self._tracked_target(observation)
        if target is None:
            target = self._largest_allowed(observation)
        if target is None:
            self.consecutive_misses += 1
            return self._decision(
                MotionCommand.neutral(),
                observation,
                "人工精调中：本帧没有匹配框；不能保存样本，但可等待目标恢复",
            )
        self.tracked = target
        self.last_target_seen_at = now
        self.consecutive_misses = 0
        area = target.box.area_ratio(observation.frame_width, observation.frame_height)
        error = self._horizontal_error(target, observation)
        return self._decision(
            MotionCommand.neutral(),
            observation,
            "人工精调中：按住键移动，Enter/G 保存候选位置",
            target,
            area,
            error,
        )

    def _step_returning(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        if self.start_depth_m is None:
            return self.abort("回收启动深度缺失", observation)
        if now - self.state_started_at >= self.config.return_timeout_s:
            return self.abort("回到启动深度超时", observation)
        error = observation.depth_m - self.start_depth_m
        # 已经比启动深度浅时只等待稳定，绝不为“追目标深度”再下潜。
        if error <= self.config.return_tolerance_m:
            if self.settled_since is None:
                self.settled_since = now
            if now - self.settled_since >= self.config.return_settle_s:
                self.state = SearchTestState.COMPLETE
                self.outcome = "completed"
                self.message = "已回到启动深度，等待 ROS 正常上锁"
                return self._decision(MotionCommand.neutral(), observation, self.message)
            return self._decision(MotionCommand.neutral(), observation, "已进入回收容差，等待稳定")
        self.settled_since = None
        vertical = min(self.config.return_maximum_command, error * self.config.return_gain)
        return self._decision(
            MotionCommand(vertical=max(0.0, vertical)),
            observation,
            f"回收 {observation.depth_m:.2f} -> {self.start_depth_m:.2f} m",
        )

    def _update_target(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision | None:
        """只在新帧中更新候选、跟踪、漏检和锁定。"""

        if self.tracked is None:
            target = self._largest_allowed(observation)
            if target is None:
                self.candidate_history.append(False)
            elif self.candidate is None or self._same_target(self.candidate, target, observation):
                self.candidate = target
                self.candidate_history.append(True)
            else:
                self.candidate = target
                self.candidate_history.clear()
                self.candidate_history.append(True)
            if (
                self.candidate is not None
                and sum(self.candidate_history) >= self.config.acquisition_required_hits
            ):
                self.tracked = self.candidate
                self.last_target_seen_at = now
                self.consecutive_misses = 0
                self.aligned_frames = 0
                self.finish_history.clear()
                self._enter(SearchTestState.ALIGNING, observation, now)
                area = self.tracked.box.area_ratio(observation.frame_width, observation.frame_height)
                error = self._horizontal_error(self.tracked, observation)
                return self._decision(MotionCommand.neutral(), observation, "5帧中至少3帧命中，目标已锁定", self.tracked, area, error)
            return None

        target = self._tracked_target(observation)
        if target is not None:
            self.tracked = target
            self.last_target_seen_at = now
            self.consecutive_misses = 0
            if self.state == SearchTestState.VERIFYING_LOSS:
                self.aligned_frames = 0
                self._enter(SearchTestState.ALIGNING, observation, now)
                area = target.box.area_ratio(observation.frame_width, observation.frame_height)
                return self._decision(MotionCommand.neutral(), observation, "目标恢复，重新对准", target, area, self._horizontal_error(target, observation))
            return None

        self.consecutive_misses += 1
        if self.state in {SearchTestState.ALIGNING, SearchTestState.APPROACHING}:
            self._enter(SearchTestState.VERIFYING_LOSS, observation, now)
        if (
            self.consecutive_misses >= self.config.loss_required_frames
            and self.last_target_seen_at is not None
            and now - self.last_target_seen_at >= self.config.loss_confirmation_s
        ):
            self._clear_target()
            if self.search_cycle >= self.config.maximum_search_cycles:
                return self.request_normal_finish(observation, now, "目标确认丢失且搜索轮数用尽，回收")
            self.search_cycle += 1
            self._enter(SearchTestState.SCANNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "连续3帧且超过0.30秒未匹配，确认丢失并重新扫描")
        return self._decision(MotionCommand.neutral(), observation, f"目标暂时漏检 {self.consecutive_misses}/3，立即停车确认")

    def _largest_allowed(self, observation: MissionObservation) -> Detection | None:
        candidates = [
            item
            for item in observation.detections
            if item.label in self.target_labels and self._valid_detection(item, observation, False)
        ]
        return max(
            candidates,
            key=lambda item: (
                item.box.area_ratio(observation.frame_width, observation.frame_height),
                item.confidence,
            ),
            default=None,
        )

    def _tracked_target(self, observation: MissionObservation) -> Detection | None:
        if self.tracked is None:
            return None
        candidates = [
            item
            for item in observation.detections
            if item.label == self.tracked.label
            and self._valid_detection(item, observation, False)
        ]
        if not candidates:
            return None
        target = max(
            candidates,
            key=lambda item: (
                self.tracked.box.intersection_over_union(item.box),
                -self._center_jump(self.tracked, item, observation),
                item.confidence,
            ),
        )
        return target if self._same_target(self.tracked, target, observation) else None

    def _same_target(
        self, previous: Detection, current: Detection, observation: MissionObservation
    ) -> bool:
        if previous.label != current.label:
            return False
        iou = previous.box.intersection_over_union(current.box)
        jump = self._center_jump(previous, current, observation)
        return iou >= self.config.minimum_tracking_iou or jump <= self.config.maximum_tracking_jump_ratio

    @staticmethod
    def _center_jump(
        previous: Detection, current: Detection, observation: MissionObservation
    ) -> float:
        old_x, old_y = previous.box.center()
        new_x, new_y = current.box.center()
        diagonal = math.hypot(observation.frame_width, observation.frame_height)
        return math.hypot(new_x - old_x, new_y - old_y) / max(diagonal, 1.0)

    def _valid_detection(
        self, detection: Detection, observation: MissionObservation, allow_oversized: bool
    ) -> bool:
        values = (
            detection.confidence,
            detection.box.left,
            detection.box.top,
            detection.box.right,
            detection.box.bottom,
        )
        if any(not math.isfinite(value) for value in values):
            return False
        if not 0.0 <= detection.confidence <= 1.0:
            return False
        if observation.frame_width <= 0 or observation.frame_height <= 0:
            return False
        if (
            detection.box.width() <= 0.0
            or detection.box.height() <= 0.0
            or detection.box.left < 0.0
            or detection.box.top < 0.0
            or detection.box.right > observation.frame_width
            or detection.box.bottom > observation.frame_height
        ):
            return False
        area = detection.box.area_ratio(observation.frame_width, observation.frame_height)
        return allow_oversized or area <= self.config.maximum_target_area_ratio

    def _horizontal_error(
        self, target: Detection, observation: MissionObservation
    ) -> float:
        center_x, _ = target.box.center()
        return center_x / observation.frame_width - self.config.aim_x_ratio

    def _clear_target(self) -> None:
        self.candidate = None
        self.candidate_history.clear()
        self.tracked = None
        self.last_target_seen_at = None
        self.consecutive_misses = 0
        self.aligned_frames = 0
        self.finish_history.clear()

    def _enter(
        self, state: SearchTestState, observation: MissionObservation, now: float
    ) -> None:
        self.state = state
        self.state_started_at = now
        self.settled_since = None
        if state == SearchTestState.SCANNING:
            self.scan_progress_deg = 0.0
            self.previous_yaw_deg = observation.yaw_deg
            self.candidate = None
            self.candidate_history.clear()
        elif state == SearchTestState.ALIGNING:
            self.aligned_frames = 0
        elif state == SearchTestState.MANUAL_CALIBRATION:
            self.finish_history.clear()
        elif state == SearchTestState.RETURNING:
            self.finish_history.clear()

    @staticmethod
    def _validate_observation(
        observation: MissionObservation, *, require_perception: bool
    ) -> None:
        if not observation.depth_valid or not math.isfinite(observation.depth_m):
            raise SearchTestError("深度反馈无效")
        if not observation.attitude_valid or not math.isfinite(observation.yaw_deg):
            raise SearchTestError("航向反馈无效")
        if observation.frame_id < 0:
            raise SearchTestError("图像帧编号无效")
        if require_perception and not observation.perception_valid:
            raise SearchTestError("启动时没有新鲜图像")

    def _decision(
        self,
        motion: MotionCommand,
        observation: MissionObservation | None,
        message: str,
        selected_target: Detection | None = None,
        area: float | None = None,
        horizontal_error: float | None = None,
    ) -> SearchTestDecision:
        self.message = message
        return SearchTestDecision(
            state=self.state,
            motion=motion,
            message=message,
            selected_target=selected_target if selected_target is not None else self.tracked,
            target_area_ratio=area,
            horizontal_error=horizontal_error,
            current_depth_m=(observation.depth_m if observation is not None else None),
            target_depth_m=(
                self.start_depth_m
                if self.state == SearchTestState.RETURNING
                else self.target_depth_m
            ),
            scan_progress_deg=(
                self.scan_progress_deg if self.state == SearchTestState.SCANNING else None
            ),
            search_cycle=self.search_cycle,
            outcome=self.outcome,
        )
