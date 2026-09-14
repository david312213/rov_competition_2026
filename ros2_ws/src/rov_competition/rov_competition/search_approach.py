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
from typing import Callable, Mapping, Sequence

import yaml

from .bottom_clearance import update_bottom_clearance, update_bottom_probe
from .domain import Detection, MissionObservation, MotionCommand
from .dataset_control import MOVEMENT_KEYS, motion_from_keys
from .mission import signed_yaw_delta_deg


class SearchTestError(RuntimeError):
    """搜索测试配置、观测或状态不能安全继续。"""


class SearchTestState(str, Enum):
    """搜索—接近测试的状态。"""

    IDLE = "idle"
    DESCENDING = "descending"
    CLEARING_BOTTOM = "clearing_bottom"
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
    # 搜索测试不再使用人工输入的目标深度。持续下潜期间，
    # 深度在 bottom_detection_depth_tolerance_m 范围内连续
    # bottom_detection_stable_s 没有明显变化，才把当前深度记为池底。
    bottom_detection_stable_s: float = 3.0
    bottom_detection_depth_tolerance_m: float = 0.05
    # 至少观察到一小段真实下潜，避免深度传感器从启动起
    # 就卡死时，程序在水面误判“已触底”并开始旋转。
    bottom_detection_minimum_descent_m: float = 0.10
    # 先在持续下潜指令下观察到一段深度平台，才认为“可能触底”；
    # 达到该时间后立即把升沉回中，由 ALT_HOLD 定深完成剩余确认，
    # 避免在完整 3 秒确认窗口内一直向池底施力。
    bottom_neutral_confirmation_s: float = 1.0
    # 确认触底后不直接在池底定深扫描。先上浮一个可调间隙，
    # 减小艇体压底产生的法向力和水平旋转摩擦。
    bottom_clearance_m: float = 0.10
    bottom_clearance_tolerance_m: float = 0.02
    bottom_clearance_up_command: float = 0.40
    bottom_clearance_settle_s: float = 1.0
    bottom_clearance_timeout_s: float = 15.0
    descent_tolerance_m: float = 0.05
    descent_gain: float = 0.8
    descent_minimum_command: float = 0.10
    descent_settle_s: float = 1.0
    descent_timeout_s: float = 45.0

    # 检测帧暂停更新时，状态机会先回中等待；只有连续
    # 超过此时间才把它判定为真正的感知断流并中止任务。
    perception_hold_timeout_s: float = 1.0
    perception_abort_timeout_s: float = 5.0
    # 检测帧与 20 Hz 输出不同步时，短时沿用上一张新帧计算出的
    # “纯偏航”对准命令，避免命令在真正发布前就被重复帧回中覆盖。
    tracking_command_hold_s: float = 0.20

    scan_yaw_command: float = 0.40
    scan_angle_deg: float = 360.0
    scan_report_step_deg: float = 15.0
    scan_tolerance_deg: float = 3.0
    scan_timeout_s: float = 120.0
    maximum_yaw_step_deg: float = 45.0
    maximum_search_cycles: int = 3

    advance_forward_command: float = 0.40
    advance_duration_s: float = 2.0

    aim_x_ratio: float = 0.50
    aim_y_ratio: float = 0.70
    # +1: 前视非镜像画面，左框左转/右框右转；-1: 镜像画面。
    image_yaw_sign: int = 1
    # -1: 框在瞄准点上方时上升；+1: 竖直画面/执行器映射反向。
    image_vertical_sign: int = -1
    horizontal_tolerance: float = 0.08
    vertical_tolerance: float = 0.10
    realign_threshold: float = 0.12
    vertical_realign_threshold: float = 0.15
    alignment_frames: int = 3
    yaw_gain: float = 0.8
    minimum_yaw_command: float = 0.10
    maximum_yaw_command: float = 0.20
    vertical_gain: float = 0.8
    minimum_vertical_command: float = 0.30
    maximum_vertical_command: float = 0.40

    far_area_ratio: float = 0.07
    stop_area_ratio: float = 0.10
    far_forward_command: float = 0.20
    near_forward_command: float = 0.10
    approach_yaw_command: float = 0.10
    approach_vertical_command: float = 0.30
    maximum_target_area_ratio: float = 0.80

    acquisition_window_frames: int = 5
    acquisition_required_hits: int = 3
    loss_required_frames: int = 5
    loss_confirmation_s: float = 0.80
    # 对准时允许极短的 YOLO 空框抖动。宽限内只沿用上一条纯偏航
    # 命令；自动接近阶段仍在第一张空框立即停车。
    alignment_loss_grace_frames: int = 2
    alignment_loss_grace_s: float = 0.20
    finish_window_frames: int = 5
    finish_required_hits: int = 4
    maximum_tracking_jump_ratio: float = 0.25
    minimum_tracking_iou: float = 0.05

    manual_initial_command: float = 0.10
    manual_minimum_command: float = 0.05
    manual_maximum_command: float = 0.40
    manual_command_step: float = 0.01
    minimum_success_samples: int = 5

    return_gain: float = 1.00
    return_minimum_command: float = 0.40
    return_maximum_command: float = 0.60
    return_tolerance_m: float = 0.05
    return_settle_s: float = 1.0
    return_timeout_s: float = 60.0

    def __post_init__(self) -> None:
        """在创建状态机前拒绝互相矛盾的参数。"""

        positive = (
            self.maximum_operation_depth_m,
            self.bottom_detection_stable_s,
            self.bottom_detection_depth_tolerance_m,
            self.bottom_detection_minimum_descent_m,
            self.bottom_neutral_confirmation_s,
            self.bottom_clearance_m,
            self.bottom_clearance_tolerance_m,
            self.bottom_clearance_up_command,
            self.bottom_clearance_settle_s,
            self.bottom_clearance_timeout_s,
            self.descent_tolerance_m,
            self.descent_gain,
            self.descent_minimum_command,
            self.descent_settle_s,
            self.descent_timeout_s,
            self.perception_hold_timeout_s,
            self.perception_abort_timeout_s,
            self.tracking_command_hold_s,
            self.scan_yaw_command,
            self.scan_angle_deg,
            self.scan_report_step_deg,
            self.scan_tolerance_deg,
            self.scan_timeout_s,
            self.maximum_yaw_step_deg,
            self.advance_forward_command,
            self.advance_duration_s,
            self.horizontal_tolerance,
            self.vertical_tolerance,
            self.realign_threshold,
            self.vertical_realign_threshold,
            self.yaw_gain,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.vertical_gain,
            self.minimum_vertical_command,
            self.maximum_vertical_command,
            self.far_area_ratio,
            self.stop_area_ratio,
            self.far_forward_command,
            self.near_forward_command,
            self.approach_yaw_command,
            self.approach_vertical_command,
            self.maximum_target_area_ratio,
            self.loss_confirmation_s,
            self.alignment_loss_grace_s,
            self.maximum_tracking_jump_ratio,
            self.minimum_tracking_iou,
            self.manual_initial_command,
            self.manual_minimum_command,
            self.manual_maximum_command,
            self.manual_command_step,
            self.return_gain,
            self.return_minimum_command,
            self.return_maximum_command,
            self.return_tolerance_m,
            self.return_settle_s,
            self.return_timeout_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise SearchTestError("搜索测试的数值参数必须是大于 0 的有限数")
        if self.perception_abort_timeout_s <= self.perception_hold_timeout_s:
            raise SearchTestError("感知中止时间必须大于回中等待时间")
        if self.tracking_command_hold_s >= self.perception_hold_timeout_s:
            raise SearchTestError("对准命令短时保持必须小于感知回中等待时间")
        if (
            self.bottom_detection_depth_tolerance_m
            >= self.bottom_detection_minimum_descent_m
        ):
            raise SearchTestError("触底深度变化容差必须小于最小实际下潜量")
        if self.bottom_detection_stable_s >= self.descent_timeout_s:
            raise SearchTestError("触底稳定时间必须小于下潜超时")
        if self.bottom_neutral_confirmation_s >= self.bottom_detection_stable_s:
            raise SearchTestError("触底回中确认时间必须小于完整触底稳定时间")
        if self.bottom_clearance_tolerance_m >= self.bottom_clearance_m:
            raise SearchTestError("离底深度容差必须小于离底距离")
        if not 0.05 <= self.bottom_clearance_m <= 0.20:
            raise SearchTestError("离底距离必须在 0.05..0.20 m")
        if self.bottom_clearance_settle_s >= self.bottom_clearance_timeout_s:
            raise SearchTestError("离底稳定时间必须小于离底超时")
        if self.scan_report_step_deg > self.scan_angle_deg:
            raise SearchTestError("扫描进度提示步长不能大于扫描总角度")
        if self.alignment_loss_grace_s >= self.loss_confirmation_s:
            raise SearchTestError("对准漏检宽限时间必须小于确认丢失时间")
        if self.return_minimum_command > self.return_maximum_command:
            raise SearchTestError("回收最小指令不能大于最大指令")
        commands = (
            self.descent_minimum_command,
            self.bottom_clearance_up_command,
            self.scan_yaw_command,
            self.advance_forward_command,
            self.minimum_yaw_command,
            self.maximum_yaw_command,
            self.far_forward_command,
            self.near_forward_command,
            self.approach_yaw_command,
            self.return_minimum_command,
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
        if not 0.0 < self.aim_x_ratio < 1.0 or not 0.0 < self.aim_y_ratio < 1.0:
            raise SearchTestError("画面瞄准点必须位于画面内")
        if self.image_yaw_sign not in {-1, 1}:
            raise SearchTestError("image_yaw_sign 必须是 +1 或 -1")
        if self.image_vertical_sign not in {-1, 1}:
            raise SearchTestError("image_vertical_sign 必须是 +1 或 -1")
        if not 0.0 < self.vertical_tolerance < self.vertical_realign_threshold < 1.0:
            raise SearchTestError("纵向容差必须小于纵向重新对准阈值")
        if self.minimum_yaw_command > self.maximum_yaw_command:
            raise SearchTestError("最小偏航指令不能大于最大值")
        if self.minimum_vertical_command > self.maximum_vertical_command:
            raise SearchTestError("最小升沉指令不能大于最大值")
        integer_values = (
            self.maximum_search_cycles,
            self.alignment_frames,
            self.acquisition_window_frames,
            self.acquisition_required_hits,
            self.loss_required_frames,
            self.alignment_loss_grace_frames,
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
        if self.alignment_loss_grace_frames >= self.loss_required_frames:
            raise SearchTestError("对准漏检宽限帧数必须小于确认丢失帧数")

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
        descent_command: float | None = None,
    ) -> tuple[str, ...]:
        """列出 robot.yaml 不足以执行测试的原因。"""

        required = max(
            self.advance_forward_command,
            self.bottom_clearance_up_command,
            self.scan_yaw_command,
            self.far_forward_command,
            self.maximum_yaw_command,
            self.return_maximum_command,
        )
        if workflow == SearchWorkflow.MANUAL_GRASP_CALIBRATION:
            required = max(required, self.manual_maximum_command)
        if descent_command is not None:
            if (
                not math.isfinite(descent_command)
                or not 0.10 <= descent_command <= 0.80
            ):
                return ("下潜 power 必须在 0.10..0.80",)
            required = max(required, descent_command)
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


STATE_DISPLAY_NAMES = {
    SearchTestState.IDLE: "等待启动",
    SearchTestState.DESCENDING: "下潜探底",
    SearchTestState.CLEARING_BOTTOM: "离底上浮并定深",
    SearchTestState.SCANNING: "360°扫描",
    SearchTestState.ADVANCING: "无目标前进",
    SearchTestState.ALIGNING: "偏航对准",
    SearchTestState.APPROACHING: "自动接近",
    SearchTestState.MANUAL_CALIBRATION: "人工抓取标定",
    SearchTestState.VERIFYING_LOSS: "漏检确认",
    SearchTestState.RESETTING_FOR_RESCAN: "回到搜索深度",
    SearchTestState.RETURNING: "回收到启动深度",
    SearchTestState.COMPLETE: "任务完成",
    SearchTestState.ABORTED: "任务中止",
}


def _terminal_print(message: str) -> None:
    """状态日志必须立即刷新，便于现场人员边看边判断。"""

    print(message, flush=True)


class SearchStateReporter:
    """把 20Hz 状态机压缩成进入事件和 1Hz 可读进度。"""

    def __init__(
        self,
        mission: "SearchApproachMission",
        *,
        emit: Callable[[str], None] = _terminal_print,
    ) -> None:
        self._mission = mission
        self._emit = emit
        self._last_state: SearchTestState | None = None
        self._last_progress_at: float | None = None
        self._last_bottom_second = 0
        self._bottom_neutral_reported = False
        self._last_scan_milestone_deg = 0.0
        self._terminal_reported = False

    def report(
        self,
        decision: SearchTestDecision,
        observation: MissionObservation,
        now: float,
        *,
        manual_power: float | None = None,
    ) -> None:
        """状态变化立即输出，同一状态最多每秒输出一次进度。"""

        state_changed = decision.state != self._last_state
        if state_changed:
            if (
                self._last_state == SearchTestState.DESCENDING
                and decision.state == SearchTestState.CLEARING_BOTTOM
            ):
                self._emit(
                    "[状态进度] DESCENDING | "
                    f"深度={observation.depth_m:.2f}m | power=0.00 | "
                    f"疑似触底稳定={int(self._mission.config.bottom_detection_stable_s)}/"
                    f"{int(self._mission.config.bottom_detection_stable_s)}s | "
                    f"深度波动={self._mission.bottom_depth_span_m:.3f}m"
                )
            if self._last_state is not None:
                self._emit(
                    f"[状态切换] {self._last_state.name} -> {decision.state.name} | "
                    f"{decision.message}"
                )
            self._emit(
                f"[进入状态] {decision.state.name}"
                f"（{STATE_DISPLAY_NAMES[decision.state]}）| "
                f"{self._entry_detail(decision, manual_power)}"
            )
            self._last_state = decision.state
            self._last_progress_at = now
            if decision.state == SearchTestState.DESCENDING:
                self._bottom_neutral_reported = False
            if decision.state == SearchTestState.SCANNING:
                self._last_scan_milestone_deg = 0.0

        if decision.state == SearchTestState.DESCENDING:
            current_second = min(
                int(self._mission.config.bottom_detection_stable_s),
                int(self._mission.bottom_stable_duration_s + 1e-6),
            )
            if current_second < self._last_bottom_second:
                self._emit(
                    "[计时重置] DESCENDING | 深度重新变化，"
                    f"触底稳定计时 {self._last_bottom_second}s -> {current_second}s"
                )
            self._last_bottom_second = current_second
            if (
                not self._bottom_neutral_reported
                and self._mission.bottom_stable_duration_s
                >= self._mission.config.bottom_neutral_confirmation_s
                and decision.motion.is_neutral()
            ):
                self._emit(
                    "[探底动作] DESCENDING | 疑似触底已稳定 "
                    f"{self._mission.bottom_stable_duration_s:.1f}s，"
                    "升沉回中，由 ALT_HOLD 定深完成确认"
                )
                self._bottom_neutral_reported = True
        elif state_changed:
            self._last_bottom_second = 0

        if decision.state == SearchTestState.SCANNING:
            step = self._mission.config.scan_report_step_deg
            milestone = math.floor(self._mission.scan_progress_deg / step) * step
            if milestone > self._last_scan_milestone_deg:
                self._last_scan_milestone_deg = milestone
                self._emit(
                    "[扫描里程碑] SCANNING | "
                    f"已完成 {milestone:.0f}/{self._mission.config.scan_angle_deg:.0f}°"
                )

        if decision.state in {SearchTestState.COMPLETE, SearchTestState.ABORTED}:
            if not self._terminal_reported:
                prefix = "任务完成" if decision.state == SearchTestState.COMPLETE else "任务中止"
                self._emit(f"[{prefix}] {decision.message}")
                self._terminal_reported = True
            return

        if state_changed:
            return
        if self._last_progress_at is not None:
            if now - self._last_progress_at < 1.0:
                return
        self._emit(
            f"[状态进度] {decision.state.name} | "
            f"{self._progress_detail(decision, observation, now, manual_power)}"
        )
        self._last_progress_at = now

    def _entry_detail(
        self, decision: SearchTestDecision, manual_power: float | None
    ) -> str:
        if decision.state == SearchTestState.DESCENDING:
            return f"power={self._mission.descent_command:.2f}"
        if decision.state == SearchTestState.CLEARING_BOTTOM:
            bottom = self._mission.bottom_depth_m
            target = self._mission.target_depth_m
            return (
                f"池底={'--' if bottom is None else f'{bottom:.2f}m'}，"
                f"离底={self._mission.config.bottom_clearance_m:.2f}m，"
                f"目标={'--' if target is None else f'{target:.2f}m'}，"
                f"up_power=+{self._mission.config.bottom_clearance_up_command:.2f}"
            )
        if decision.state == SearchTestState.SCANNING:
            return (
                f"连续右转 yaw_power={self._mission.config.scan_yaw_command:.2f}，"
                f"每 {self._mission.config.scan_report_step_deg:.0f}° 报告一次"
            )
        if decision.state == SearchTestState.ADVANCING:
            return (
                f"前进 power={self._mission.config.advance_forward_command:.2f}, "
                f"时长={self._mission.config.advance_duration_s:.1f}s"
            )
        if decision.state == SearchTestState.MANUAL_CALIBRATION:
            return f"人工 power={manual_power or self._mission.config.manual_initial_command:.2f}"
        return decision.message

    def _progress_detail(
        self,
        decision: SearchTestDecision,
        observation: MissionObservation,
        now: float,
        manual_power: float | None,
    ) -> str:
        elapsed = max(0.0, now - self._mission.state_started_at)
        state = decision.state
        if (
            "暂停" in decision.message
            or "回中等待" in decision.message
            or "短暂漏检" in decision.message
        ):
            return decision.message
        if state == SearchTestState.DESCENDING:
            stable = min(
                self._mission.bottom_stable_duration_s,
                self._mission.config.bottom_detection_stable_s,
            )
            return (
                f"深度={observation.depth_m:.2f}m | power={abs(decision.motion.vertical):.2f} | "
                f"疑似触底稳定={int(stable + 1e-6)}/"
                f"{int(self._mission.config.bottom_detection_stable_s)}s | "
                f"深度波动={self._mission.bottom_depth_span_m:.3f}m"
            )
        if state == SearchTestState.CLEARING_BOTTOM:
            target = self._mission.target_depth_m
            bottom = self._mission.bottom_depth_m
            lifted = (
                0.0
                if bottom is None
                else max(0.0, bottom - observation.depth_m)
            )
            settled = (
                0.0
                if self._mission.settled_since is None
                else max(0.0, now - self._mission.settled_since)
            )
            return (
                f"深度={observation.depth_m:.2f}m | "
                f"目标={'--' if target is None else f'{target:.2f}m'} | "
                f"已离底={lifted:.2f}/{self._mission.config.bottom_clearance_m:.2f}m | "
                f"稳定={settled:.1f}/{self._mission.config.bottom_clearance_settle_s:.1f}s | "
                f"vertical={decision.motion.vertical:+.2f}"
            )
        if state == SearchTestState.SCANNING:
            return (
                f"扫描={self._mission.scan_progress_deg:.1f}/"
                f"{self._mission.config.scan_angle_deg:.0f}° | "
                f"轮次={self._mission.search_cycle}/"
                f"{self._mission.config.maximum_search_cycles} | "
                f"yaw_power={decision.motion.yaw:+.2f}"
            )
        if state == SearchTestState.ADVANCING:
            return (
                f"计时={elapsed:.1f}/{self._mission.config.advance_duration_s:.1f}s | "
                f"forward={decision.motion.forward:+.2f}"
            )
        if state == SearchTestState.ALIGNING:
            error = decision.horizontal_error
            return (
                f"横向误差={'--' if error is None else f'{error:+.3f}'} | "
                f"稳定帧={self._mission.aligned_frames}/"
                f"{self._mission.config.alignment_frames} | yaw={decision.motion.yaw:+.2f}"
            )
        if state == SearchTestState.APPROACHING:
            area = decision.target_area_ratio
            return (
                f"面积={'--' if area is None else f'{area:.3f}'}/"
                f"{self._mission.config.stop_area_ratio:.3f} | "
                f"forward={decision.motion.forward:+.2f} | yaw={decision.motion.yaw:+.2f}"
            )
        if state == SearchTestState.VERIFYING_LOSS:
            since_seen = (
                0.0
                if self._mission.last_target_seen_at is None
                else max(0.0, now - self._mission.last_target_seen_at)
            )
            return (
                f"漏检={self._mission.consecutive_misses}/"
                f"{self._mission.config.loss_required_frames}帧 | "
                f"时间={since_seen:.2f}/{self._mission.config.loss_confirmation_s:.2f}s | 已停车"
            )
        if state in {SearchTestState.RESETTING_FOR_RESCAN, SearchTestState.RETURNING}:
            target = decision.target_depth_m
            settled = 0.0 if self._mission.settled_since is None else now - self._mission.settled_since
            return (
                f"深度={observation.depth_m:.2f}m | "
                f"目标={'--' if target is None else f'{target:.2f}m'} | "
                f"稳定={max(0.0, settled):.1f}s | vertical={decision.motion.vertical:+.2f}"
            )
        if state == SearchTestState.MANUAL_CALIBRATION:
            area = decision.target_area_ratio
            return (
                f"power={manual_power or 0.0:.2f} | "
                f"框面积={'--' if area is None else f'{area:.3f}'} | "
                f"误差={'--' if decision.horizontal_error is None else f'{decision.horizontal_error:+.3f}'}"
            )
        return f"持续={elapsed:.1f}s | {decision.message}"


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
        self.bottom_depth_m: float | None = None
        self.target_depth_m: float | None = None
        self.descent_command = 0.20
        self.descent_depth_history: deque[tuple[float, float]] = deque()
        self.bottom_stable_duration_s = 0.0
        self.bottom_depth_span_m = 0.0
        self.state_started_at = 0.0
        self.settled_since: float | None = None
        self.clearance_settle_reference_depth_m: float | None = None
        self.search_cycle = 0
        self.scan_progress_deg = 0.0
        self.previous_yaw_deg: float | None = None
        self.last_frame_id = -1
        self.last_new_frame_at = 0.0
        self.last_alignment_decision: SearchTestDecision | None = None
        self.state_before_loss: SearchTestState | None = None
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
        descent_command: float,
        now: float,
    ) -> SearchTestDecision:
        """记录启动深度，持续下潜并等待深度平台判定触底。"""

        if self.state not in {SearchTestState.IDLE, SearchTestState.COMPLETE}:
            raise SearchTestError("搜索测试已在运行")
        self._validate_observation(observation, require_perception=True)
        if (
            not math.isfinite(descent_command)
            or not 0.10 <= descent_command <= 0.80
        ):
            raise SearchTestError("下潜 power 必须在 0.10..0.80")
        if observation.depth_m >= self.config.maximum_operation_depth_m:
            raise SearchTestError(
                f"启动深度 {observation.depth_m:.2f} m 已达到配置上限 "
                f"{self.config.maximum_operation_depth_m:.2f} m"
            )
        self.start_depth_m = float(observation.depth_m)
        # 触底前不存在人工设定的目标深度。确认深度平台后，
        # bottom_depth_m 记录池底，target_depth_m 记录离底后的
        # 本轮搜索深度，供扫描和 R 重搜使用。
        self.bottom_depth_m = None
        self.target_depth_m = None
        self.descent_command = float(descent_command)
        self.descent_depth_history.clear()
        self.descent_depth_history.append((float(now), float(observation.depth_m)))
        self.bottom_stable_duration_s = 0.0
        self.bottom_depth_span_m = 0.0
        self.clearance_settle_reference_depth_m = None
        self.outcome = "running"
        self.last_frame_id = observation.frame_id
        self.last_new_frame_at = float(now)
        self.last_alignment_decision = None
        self.state_before_loss = None
        self._clear_target()
        self._enter(SearchTestState.DESCENDING, observation, now)
        return self._decision(
            MotionCommand.neutral(),
            observation,
            "已记录启动深度，开始下潜并自动判定触底",
        )

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
        漏检确认时间或深度稳定时间。
        """

        if not math.isfinite(paused_duration_s) or paused_duration_s < 0.0:
            raise SearchTestError("暂停时长必须是非负有限数")
        self.state_started_at += paused_duration_s
        if self.settled_since is not None:
            self.settled_since += paused_duration_s
        if self.descent_depth_history:
            self.descent_depth_history = deque(
                (sample_time + paused_duration_s, depth_m)
                for sample_time, depth_m in self.descent_depth_history
            )
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
            self.last_new_frame_at = float(now)

        if self.state == SearchTestState.DESCENDING:
            return self._step_descending(observation, now)
        if self.state == SearchTestState.CLEARING_BOTTOM:
            return self._step_clearing_bottom(observation, now)
        if self.state == SearchTestState.RETURNING:
            return self._step_returning(observation, now)
        if self.state == SearchTestState.RESETTING_FOR_RESCAN:
            return self._step_resetting_for_rescan(observation, now)
        if not observation.perception_valid:
            return self._decision(MotionCommand.neutral(), observation, "没有新鲜图像，回中等待")
        if not new_frame:
            # 控制循环固定为 20 Hz，YOLO 帧率不必与它严格同频。只要
            # 最近检测帧仍在外层的 freshness 窗口内，扫描和无目标
            # 前进就应连续执行，不能在两张新帧之间一转一停。
            # 目标确认、漏检、对准和接近仍只消费真正的新帧，绝不把
            # 同一张旧框重复计数或据此继续靠近。
            if self.state == SearchTestState.SCANNING:
                return self._step_scanning(observation, now)
            if self.state == SearchTestState.ADVANCING:
                return self._step_advancing(observation, now)
            if (
                self.state == SearchTestState.ALIGNING
                and self.last_alignment_decision is not None
                and self.last_target_seen_at is not None
                and now - self.last_new_frame_at
                <= self.config.tracking_command_hold_s
                and now - self.last_target_seen_at
                <= self.config.alignment_loss_grace_s
            ):
                cached = self.last_alignment_decision
                return self._decision(
                    cached.motion,
                    observation,
                    "等待下一张新检测帧，短时保持最近偏航命令",
                    cached.selected_target,
                    cached.target_area_ratio,
                    cached.horizontal_error,
                )
            return self._decision(
                MotionCommand.neutral(), observation, "等待下一张新检测帧，保持停车"
            )

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
        if (
            self.start_depth_m is None
            or not self.descent_depth_history
        ):
            return self.abort("触底判定基准缺失", observation)
        if now - self.state_started_at >= self.config.descent_timeout_s:
            return self.abort("下潜触底等待超时", observation)
        if observation.depth_m >= self.config.maximum_operation_depth_m:
            return self.abort(
                f"深度 {observation.depth_m:.2f} m 已达到作业上限 "
                f"{self.config.maximum_operation_depth_m:.2f} m",
                observation,
            )

        probe = update_bottom_probe(
            self.descent_depth_history,
            now=now,
            current_depth_m=observation.depth_m,
            probe_start_depth_m=self.start_depth_m,
            stable_duration_required_s=self.config.bottom_detection_stable_s,
            stable_depth_tolerance_m=self.config.bottom_detection_depth_tolerance_m,
            minimum_descent_m=self.config.bottom_detection_minimum_descent_m,
            neutral_confirmation_s=self.config.bottom_neutral_confirmation_s,
        )
        self.bottom_stable_duration_s = probe.stable_duration_s
        self.bottom_depth_span_m = probe.depth_span_m
        if probe.confirmed:
            # 记下实测触底深度。进入扫描前先回中，不让推进器
            # 在状态切换的这一帧继续向池底施力。
            assert probe.bottom_depth_m is not None
            bottom_depth_m = probe.bottom_depth_m
            self.bottom_depth_m = float(bottom_depth_m)
            # 深度以向下为正，因此离底目标是池底深度减去间隙。
            # 不让这个阶段浮到本次启动深度之上。
            self.target_depth_m = max(
                float(self.start_depth_m),
                float(bottom_depth_m) - self.config.bottom_clearance_m,
            )
            self._enter(SearchTestState.CLEARING_BOTTOM, observation, now)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"深度连续 {self.config.bottom_detection_stable_s:.1f}s "
                f"无明显变化，记录疑似触底 {bottom_depth_m:.2f} m，"
                f"先上浮 {bottom_depth_m - self.target_depth_m:.2f} m 解除压底",
            )

        # 与键盘工具按住“↓”一致地下潜，直到在持续推力下观察到
        # 足够长的疑似平台。随后立即把 vertical 回中，由 ALT_HOLD
        # 定住当前深度，剩余时间只用于确认平台没有消失。若深度重新
        # 明显变化，稳定计时会归零，程序才恢复下潜指令。
        vertical = 0.0 if probe.hold_neutral else -self.descent_command
        progress_note = (
            f"已下潜 {max(0.0, probe.descended_m):.2f} m"
            if probe.enough_descent
            else (
                f"已下潜 {max(0.0, probe.descended_m):.2f}/"
                f"{self.config.bottom_detection_minimum_descent_m:.2f} m"
            )
        )
        return self._decision(
            MotionCommand(vertical=vertical),
            observation,
            f"下潜探底：深度 {observation.depth_m:.2f} m，{progress_note}，"
            f"稳定 {self.bottom_stable_duration_s:.1f}/{self.config.bottom_detection_stable_s:.1f}s，"
            f"变化 {probe.depth_span_m:.3f}/{self.config.bottom_detection_depth_tolerance_m:.3f}m，"
            + ("ALT_HOLD 定深确认" if probe.hold_neutral else "继续下潜"),
        )

    def _step_clearing_bottom(
        self, observation: MissionObservation, now: float
    ) -> SearchTestDecision:
        """触底后上浮一个小间隙，稳定定深后再开始扫描。

        这个阶段只允许上浮或回中，即使超调到目标深度之上，
        也不会为了追准数值再次下潜压向池底。
        """

        if self.bottom_depth_m is None or self.target_depth_m is None:
            return self.abort("离底基准深度缺失", observation)
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
            reference_depth_m=self.clearance_settle_reference_depth_m,
        )
        self.settled_since = clearance.settled_since
        self.clearance_settle_reference_depth_m = clearance.reference_depth_m
        if clearance.unsafe_depth_increase:
            return self.abort("离底时深度反而增大，拒绝继续施力", observation)
        if clearance.reached_target:
            if clearance.settled:
                self.search_cycle = 1
                self._enter(SearchTestState.SCANNING, observation, now)
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    f"已离底 {max(0.0, self.bottom_depth_m - observation.depth_m):.2f} m "
                    "并稳定定深，开始向右扫描",
                )
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"已到离底目标 {self.target_depth_m:.2f} m，"
                "升沉回中等待 ALT_HOLD 稳定",
            )

        return self._decision(
            MotionCommand(vertical=clearance.vertical_command),
            observation,
            f"触底后上浮离底：{observation.depth_m:.2f} -> "
            f"{self.target_depth_m:.2f} m",
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
            decision = self._decision(
                MotionCommand.neutral(),
                observation,
                "目标居中，等待多帧稳定",
                target,
                area,
                error,
            )
            self.last_alignment_decision = decision
            return decision
        self.aligned_frames = 0
        magnitude = min(self.config.maximum_yaw_command, abs(error) * self.config.yaw_gain)
        magnitude = max(self.config.minimum_yaw_command, magnitude)
        # 已验收的艇体映射：yaw<0 是键1左转，yaw>0 是键2右转。
        # 前视非镜像画面中，目标在左就应左转，在右就应右转。
        yaw = self.config.image_yaw_sign * math.copysign(magnitude, error)
        target_side = "画面右侧" if error > 0.0 else "画面左侧"
        turn_direction = "右转" if yaw > 0.0 else "左转"
        decision = self._decision(
            MotionCommand(yaw=yaw),
            observation,
            f"目标在{target_side}，执行{turn_direction}对准",
            target,
            area,
            error,
        )
        self.last_alignment_decision = decision
        return decision

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

        yaw = max(
            -self.config.approach_yaw_command,
            min(
                self.config.approach_yaw_command,
                self.config.image_yaw_sign * error * self.config.yaw_gain,
            ),
        )
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
        vertical = min(
            self.config.return_maximum_command,
            max(self.config.return_minimum_command, error * self.config.return_gain),
        )
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
                # 锁定目标的这张新帧已经是有效观测，应立即
                # 计算并缓存偏航决定。如果先额外发一帧中位，
                # 60 Hz 状态循环可能在 20 Hz 发布时机前把真正的
                # 对准命令覆盖掉。
                decision = self._step_aligning(observation, now)
                if decision.state == SearchTestState.ALIGNING:
                    decision = self._decision(
                        decision.motion,
                        observation,
                        "5帧中至少3帧命中，目标已锁定；开始偏航对准",
                        decision.selected_target,
                        decision.target_area_ratio,
                        decision.horizontal_error,
                    )
                    self.last_alignment_decision = decision
                return decision
            return None

        target = self._tracked_target(observation)
        if target is not None:
            self.tracked = target
            self.last_target_seen_at = now
            self.consecutive_misses = 0
            if self.state == SearchTestState.VERIFYING_LOSS:
                # 恢复帧本身就参与对准并立即给出偏航决定，不能先回中
                # 一帧，否则“命中一帧、漏一帧”时永远没有实际动作。
                if self.state_before_loss == SearchTestState.APPROACHING:
                    self.aligned_frames = 0
                self.state = SearchTestState.ALIGNING
                self.state_started_at = now
                self.settled_since = None
                self.state_before_loss = None
                return self._step_aligning(observation, now)
            return None

        self.consecutive_misses += 1
        since_seen_s = (
            math.inf
            if self.last_target_seen_at is None
            else max(0.0, now - self.last_target_seen_at)
        )
        if (
            self.state == SearchTestState.ALIGNING
            and self.consecutive_misses <= self.config.alignment_loss_grace_frames
            and since_seen_s <= self.config.alignment_loss_grace_s
        ):
            cached = self.last_alignment_decision
            motion = MotionCommand.neutral() if cached is None else cached.motion
            return self._decision(
                motion,
                observation,
                f"目标短暂漏检 {self.consecutive_misses}/"
                f"{self.config.alignment_loss_grace_frames}，"
                "短时保持最近偏航；仍未恢复将停车确认",
                self.tracked,
                (None if cached is None else cached.target_area_ratio),
                (None if cached is None else cached.horizontal_error),
            )
        if self.state in {SearchTestState.ALIGNING, SearchTestState.APPROACHING}:
            self.state_before_loss = self.state
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
            return self._decision(
                MotionCommand.neutral(),
                observation,
                f"连续{self.config.loss_required_frames}帧且超过"
                f"{self.config.loss_confirmation_s:.2f}秒未匹配，确认丢失并重新扫描",
            )
        return self._decision(
            MotionCommand.neutral(),
            observation,
            f"目标暂时漏检 {self.consecutive_misses}/"
            f"{self.config.loss_required_frames}，立即停车确认",
        )

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
        self.last_alignment_decision = None
        self.state_before_loss = None

    def _enter(
        self, state: SearchTestState, observation: MissionObservation, now: float
    ) -> None:
        self.state = state
        self.state_started_at = now
        self.settled_since = None
        self.clearance_settle_reference_depth_m = None
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
