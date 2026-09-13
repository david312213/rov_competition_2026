"""人工辅助的连续蛇形搜寻核心，不含抓取、机械爪或转运。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math

from .bottom_clearance import update_bottom_clearance, update_bottom_probe
from .domain import Detection, MissionObservation, MotionCommand


class SearchState(str, Enum):
    PAUSED = "paused"
    PROBING_BOTTOM = "probing_bottom"
    CLEARING_BOTTOM = "clearing_bottom"
    SEARCHING = "searching"
    LANE_SHIFTING = "lane_shifting"
    LANE_TURNING = "lane_turning"
    CORRECTING = "correcting"
    TARGET_HELD = "target_held"
    SURFACING = "surfacing"
    FAULT = "fault"


class SearchAction(str, Enum):
    """操作员短按动作；不需要持续按键。"""

    TOUCH_AND_START = "T"
    PAUSE_TOGGLE = "P"
    CORRECT_LEFT = "A"
    CORRECT_RIGHT = "D"
    NEXT_LANE = "N"
    SURFACE = "0"


@dataclass(frozen=True)
class SearchConfig:
    """连续搜寻的显式配置。

    运动参数来自本次实艇测量；触底判据沿用原 ``search_test.yaml`` 的
    基线，尚未作为已验证的实艇结论。
    """

    target_label: str
    target_confidence: float
    target_required_hits: int
    target_minimum_iou: float
    stop_on_stable_target: bool
    forward_command: float
    shift_command: float
    shift_duration_s: float
    small_yaw_left_command: float
    small_yaw_left_duration_s: float
    small_yaw_right_command: float
    small_yaw_right_duration_s: float
    turn_command: float
    turn_duration_s: float
    descent_command: float
    ascent_command: float
    probe_timeout_s: float
    minimum_descent_m: float
    stable_depth_tolerance_m: float
    stable_duration_s: float
    neutral_confirmation_s: float
    clearance_m: float
    clearance_tolerance_m: float
    clearance_settle_s: float
    clearance_timeout_s: float
    lane_forward_duration_s: float
    search_duration_s: float

    def __post_init__(self) -> None:
        if self.target_label not in {"scallop", "snail"}:
            raise ValueError("搜寻目标只能是 scallop 或 snail")
        if not 0.0 < self.target_confidence <= 1.0:
            raise ValueError("目标置信度必须在 (0, 1]")
        if self.target_required_hits < 1:
            raise ValueError("目标连续命中帧数至少为 1")
        if not 0.0 <= self.target_minimum_iou <= 1.0:
            raise ValueError("目标 IoU 必须在 [0, 1]")
        if not isinstance(self.stop_on_stable_target, bool):
            raise ValueError("stop_on_stable_target 必须为布尔值")
        non_motion = {
            "target_label", "target_required_hits", "target_confidence",
            "target_minimum_iou", "stop_on_stable_target",
        }
        for name, value in vars(self).items():
            if name not in non_motion and (
                not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} 必须为正的有限数")
        for name in (
            "forward_command", "shift_command", "small_yaw_left_command",
            "small_yaw_right_command", "turn_command", "descent_command",
            "ascent_command",
        ):
            if getattr(self, name) > 1.0:
                raise ValueError(f"{name} 不能大于 1")


@dataclass(frozen=True)
class SearchDecision:
    state: SearchState
    motion: MotionCommand
    message: str
    lane_index: int
    lane_direction: str
    target: Detection | None = None


class SemicircleSearchMission:
    """半圆核心区的定时蛇形搜寻。

    没有位置估计、罗盘闭环或自动边界判断。每条搜寻带到时后自动横移、
    定时大角度翻转；``N`` 只保留为提前换带的备用键。是否因稳定发现目标
    停车由 ``stop_on_stable_target`` 配置决定。
    """

    def __init__(self, config: SearchConfig) -> None:
        self.config = config
        self.state = SearchState.PAUSED
        self.lane_index = 0
        self.forward_direction = True
        self.state_started_at = 0.0
        self.bottom_samples: deque[tuple[float, float]] = deque()
        self.bottom_depth_m: float | None = None
        self.target_depth_m: float | None = None
        self.probe_start_depth_m: float | None = None
        self.settled_since: float | None = None
        self.clearance_reference_depth_m: float | None = None
        self.last_frame_id: int | None = None
        self.previous_box: Detection | None = None
        self.target_hits = 0
        self.target: Detection | None = None
        self.correction_motion = MotionCommand.neutral()
        self.height_established = False
        self.search_started_at: float | None = None
        self.lane_started_at: float | None = None
        self.paused_from_state: SearchState | None = None
        self.paused_at: float | None = None

    def step(self, observation: MissionObservation, now: float, action: SearchAction | None = None) -> SearchDecision:
        if not math.isfinite(now):
            return self._fault("控制时间无效")
        if action is SearchAction.SURFACE:
            self.state = SearchState.SURFACING
        if self.state is SearchState.SURFACING:
            return self._out(MotionCommand(vertical=self.config.ascent_command), "终止搜寻：持续上浮；只能由外部安全流程结束")
        if self.state is SearchState.FAULT:
            return self._out(MotionCommand.neutral(), "搜寻故障：已停止自主运动")
        if self.state is SearchState.TARGET_HELD:
            return self._out(MotionCommand.neutral(), "目标已锁定：等待独立抓取策略")
        if action is SearchAction.TOUCH_AND_START:
            return self._start_bottom_probe(observation, now)
        if action is SearchAction.PAUSE_TOGGLE:
            return self._toggle_pause(now)
        if action in {SearchAction.CORRECT_LEFT, SearchAction.CORRECT_RIGHT}:
            return self._start_correction(action, now)
        if action is SearchAction.NEXT_LANE:
            return self._start_next_lane(now)
        if self.state is SearchState.PAUSED:
            return self._out(MotionCommand.neutral(), "已暂停：按 T 触底定高后开始搜寻")
        if not observation.depth_valid:
            return self._fault("深度数据无效")
        if self.state is SearchState.PROBING_BOTTOM:
            return self._probe_bottom(observation, now)
        if self.state is SearchState.CLEARING_BOTTOM:
            return self._clear_bottom(observation, now)
        if self.state is SearchState.CORRECTING:
            return self._continue_correction(now)
        if self.state is SearchState.LANE_SHIFTING:
            return self._continue_lane_shift(now)
        if self.state is SearchState.LANE_TURNING:
            return self._continue_lane_turn(now)
        return self._search(observation, now)

    def _start_bottom_probe(self, observation: MissionObservation, now: float) -> SearchDecision:
        if not observation.depth_valid:
            return self._fault("无法触底定高：深度数据无效")
        self.state = SearchState.PROBING_BOTTOM
        self.state_started_at = now
        self.probe_start_depth_m = observation.depth_m
        self.bottom_samples.clear()
        self.bottom_depth_m = self.target_depth_m = None
        self.settled_since = self.clearance_reference_depth_m = None
        self.target = None
        self._clear_target_tracking()
        return self._out(MotionCommand(vertical=-self.config.descent_command), "触底定高：开始有限下潜")

    def _toggle_pause(self, now: float) -> SearchDecision:
        if self.state is SearchState.PAUSED:
            if not self.height_established:
                return self._out(MotionCommand.neutral(), "尚未定高：请先按 T 触底并上浮")
            assert self.paused_from_state is not None and self.paused_at is not None
            pause_duration = now - self.paused_at
            self.state = self.paused_from_state
            self.state_started_at += pause_duration
            if self.search_started_at is not None:
                self.search_started_at += pause_duration
            if self.lane_started_at is not None:
                self.lane_started_at += pause_duration
            self.paused_from_state = None
            self.paused_at = None
            return self._out(MotionCommand.neutral(), "解除暂停：等待下一控制周期恢复")
        self.paused_from_state = self.state
        self.paused_at = now
        self.state = SearchState.PAUSED
        self.state_started_at = now
        return self._out(MotionCommand.neutral(), "搜寻已暂停")

    def _start_correction(self, action: SearchAction, now: float) -> SearchDecision:
        if self.state is not SearchState.SEARCHING:
            return self._out(MotionCommand.neutral(), "仅可在持续搜寻时进行方向微调")
        self.state = SearchState.CORRECTING
        self.state_started_at = now
        self.correction_motion = MotionCommand(yaw=(-self.config.small_yaw_left_command if action is SearchAction.CORRECT_LEFT else self.config.small_yaw_right_command))
        return self._out(self.correction_motion, "人工方向微调中")

    def _start_next_lane(self, now: float) -> SearchDecision:
        if self.state is not SearchState.SEARCHING:
            return self._out(MotionCommand.neutral(), "仅可在持续搜寻时换带")
        self.state = SearchState.LANE_SHIFTING
        self.state_started_at = now
        return self._out(self._lane_shift_motion(), "换带：横移中")

    def _probe_bottom(self, observation: MissionObservation, now: float) -> SearchDecision:
        assert self.probe_start_depth_m is not None
        if now - self.state_started_at >= self.config.probe_timeout_s:
            return self._fault("触底未确认：下潜超时")
        result = update_bottom_probe(
            self.bottom_samples, now=now, current_depth_m=observation.depth_m,
            probe_start_depth_m=self.probe_start_depth_m,
            stable_duration_required_s=self.config.stable_duration_s,
            stable_depth_tolerance_m=self.config.stable_depth_tolerance_m,
            minimum_descent_m=self.config.minimum_descent_m,
            neutral_confirmation_s=self.config.neutral_confirmation_s,
        )
        if result.confirmed:
            assert result.bottom_depth_m is not None
            self.bottom_depth_m = result.bottom_depth_m
            self.target_depth_m = self.bottom_depth_m - self.config.clearance_m
            self.state = SearchState.CLEARING_BOTTOM
            self.state_started_at = now
            return self._out(MotionCommand(vertical=self.config.ascent_command), "触底确认：开始上浮到观察高度")
        motion = MotionCommand.neutral() if result.hold_neutral else MotionCommand(vertical=-self.config.descent_command)
        return self._out(motion, "触底定高中")

    def _clear_bottom(self, observation: MissionObservation, now: float) -> SearchDecision:
        assert self.bottom_depth_m is not None and self.target_depth_m is not None
        if now - self.state_started_at >= self.config.clearance_timeout_s:
            return self._fault("离底失败：上浮超时")
        result = update_bottom_clearance(
            now=now, current_depth_m=observation.depth_m, bottom_depth_m=self.bottom_depth_m,
            target_depth_m=self.target_depth_m, bottom_depth_tolerance_m=self.config.stable_depth_tolerance_m,
            clearance_tolerance_m=self.config.clearance_tolerance_m,
            settle_duration_s=self.config.clearance_settle_s, up_command=self.config.ascent_command,
            settled_since=self.settled_since, reference_depth_m=self.clearance_reference_depth_m,
        )
        self.settled_since = result.settled_since
        self.clearance_reference_depth_m = result.reference_depth_m
        if result.unsafe_depth_increase:
            return self._fault("离底阶段仍在继续下潜")
        if result.settled:
            self.height_established = True
            self.state = SearchState.SEARCHING
            self.state_started_at = now
            self.search_started_at = now
            self.lane_started_at = now
            return self._out(self._cruise_motion(), "定高完成：开始持续前进搜寻")
        return self._out(MotionCommand(vertical=result.vertical_command), "上浮到观察高度中")

    def _continue_correction(self, now: float) -> SearchDecision:
        duration = self.config.small_yaw_left_duration_s if self.correction_motion.yaw < 0 else self.config.small_yaw_right_duration_s
        if now - self.state_started_at < duration:
            return self._out(self.correction_motion, "人工方向微调中")
        self.state = SearchState.SEARCHING
        return self._out(self._cruise_motion(), "方向微调完成：恢复持续前进")

    def _continue_lane_shift(self, now: float) -> SearchDecision:
        if now - self.state_started_at < self.config.shift_duration_s:
            return self._out(self._lane_shift_motion(), "换带：横移中")
        self.state = SearchState.LANE_TURNING
        self.state_started_at = now
        return self._out(self._lane_turn_motion(), "换带：大角度翻转中")

    def _continue_lane_turn(self, now: float) -> SearchDecision:
        if now - self.state_started_at < self.config.turn_duration_s:
            return self._out(self._lane_turn_motion(), "换带：大角度翻转中")
        self.lane_index += 1
        self.forward_direction = not self.forward_direction
        self.state = SearchState.SEARCHING
        self.lane_started_at = now
        return self._out(self._cruise_motion(), "进入下一条搜寻带：恢复持续前进")

    def _search(self, observation: MissionObservation, now: float) -> SearchDecision:
        assert self.search_started_at is not None and self.lane_started_at is not None
        if now - self.search_started_at >= self.config.search_duration_s:
            self.state = SearchState.SURFACING
            return self._out(
                MotionCommand(vertical=self.config.ascent_command),
                "搜寻倒计时结束：自动持续上浮",
            )
        if now - self.lane_started_at >= self.config.lane_forward_duration_s:
            self.state = SearchState.LANE_SHIFTING
            self.state_started_at = now
            return self._out(self._lane_shift_motion(), "本带定时结束：自动横移换带")
        if self.config.stop_on_stable_target and observation.frame_id != self.last_frame_id:
            self.last_frame_id = observation.frame_id
            candidates = [d for d in observation.detections if d.label == self.config.target_label and d.confidence >= self.config.target_confidence]
            if candidates:
                candidate = max(candidates, key=lambda item: item.confidence)
                same_target = self.previous_box is not None and candidate.box.intersection_over_union(self.previous_box.box) >= self.config.target_minimum_iou
                self.target_hits = self.target_hits + 1 if same_target else 1
                self.previous_box = candidate
                if self.target_hits >= self.config.target_required_hits:
                    self.target = candidate
                    self.state = SearchState.TARGET_HELD
                    return self._out(MotionCommand.neutral(), "稳定发现目标：停车等待独立抓取策略")
            else:
                self._clear_target_tracking()
        return self._out(self._cruise_motion(), "持续前进搜寻")

    def _clear_target_tracking(self) -> None:
        self.previous_box = None
        self.target_hits = 0

    def _cruise_motion(self) -> MotionCommand:
        return MotionCommand(forward=self.config.forward_command)

    def _lane_shift_motion(self) -> MotionCommand:
        return MotionCommand(lateral=(-self.config.shift_command if self.forward_direction else self.config.shift_command))

    def _lane_turn_motion(self) -> MotionCommand:
        return MotionCommand(yaw=(-self.config.turn_command if self.forward_direction else self.config.turn_command))

    def _fault(self, message: str) -> SearchDecision:
        self.state = SearchState.FAULT
        return self._out(MotionCommand.neutral(), message)

    def _out(self, motion: MotionCommand, message: str) -> SearchDecision:
        return SearchDecision(self.state, motion, message, self.lane_index, "forward" if self.forward_direction else "reverse", self.target)
