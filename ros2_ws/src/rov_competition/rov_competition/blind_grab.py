"""永久沉底蛇形盲抓的纯状态机。

首次启动使用压力深度平台或固定十秒兜底进入第一次抓取。之后只依赖
配置动作和单调时钟，永久重复抓取、上潜、蛇形移动和定时下潜。
视觉检测仅供画面与日志显示，不参与本状态机的任何转换。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from enum import Enum

from .bottom_clearance import update_bottom_probe
from .domain import MotionCommand


@dataclass(frozen=True)
class ServoSetpoint:
    output_channel: int
    pwm: int


@dataclass(frozen=True)
class ServoAction:
    """一次姿态指令；允许一个动作同时控制多路舵机。"""

    outputs: tuple[ServoSetpoint, ...]
    duration_s: float


@dataclass(frozen=True)
class BlindGrabConfig:
    advance_duration_s: float
    open_gripper: ServoAction
    close_gripper: ServoAction
    arm_to_basket: ServoAction
    arm_to_grasp: ServoAction
    release_duration_s: float
    initial_descent_command: float = -0.415
    initial_bottom_stable_s: float = 3.0
    initial_bottom_tolerance_m: float = 0.05
    initial_minimum_descent_m: float = 0.10
    initial_fallback_s: float = 10.0
    ascent_command: float = 0.415
    ascent_duration_s: float = 2.0
    repeat_descent_command: float = -0.415
    repeat_descent_duration_s: float = 5.0
    route_forward_command: float = 0.23
    route_step_duration_s: float = 5.0
    route_steps_per_lane: int = 4
    shift_command: float = 0.20
    shift_duration_s: float = 4.5
    turn_command: float = 0.20
    turn_duration_s: float = 11.5
    grab_forward_command: float = 0.23


@dataclass(frozen=True)
class DetectionCount:
    """仅供查看和日志显示；永远不会传入运动状态机。"""

    frame_id: int
    received_at: float
    count: int


@dataclass(frozen=True)
class DepthSample:
    """由MAVLink旁路提取的本机单调时间深度样本。"""

    received_at: float
    depth_m: float


class BlindState(str, Enum):
    INITIAL_DESCENT = "initial_descent"
    GRABBING = "grabbing"
    ASCENDING = "ascending"
    FORWARD = "forward"
    SHIFT = "shift"
    TURN = "turn"
    REPEAT_DESCENT = "repeat_descent"


class GrabPhase(str, Enum):
    OPEN = "open"
    ADVANCE = "advance"
    CLOSE = "close"
    TRANSFER = "transfer"
    RELEASE = "release"
    RETURN = "return"


@dataclass(frozen=True)
class BlindGrabDecision:
    state: BlindState
    motion: MotionCommand
    servos: tuple[ServoSetpoint, ...]
    phase: str
    phase_elapsed_s: float
    depth_m: float | None
    initial_descent_elapsed_s: float
    bottom_stable_s: float
    bottom_depth_span_m: float
    bottom_depth_m: float | None
    bottom_source: str
    lane_index: int
    segment_in_lane: int
    route_direction: int
    grasp_command_count: int
    completed_cycles: int


class BlindGrabMission:
    """启动即下潜，随后永久执行定时蛇形盲抓。"""

    def __init__(self, config: BlindGrabConfig) -> None:
        self.config = config
        self.state = BlindState.INITIAL_DESCENT
        self.grab_phase: GrabPhase | None = None
        self.lane_index = 0
        self.segment_in_lane = 0
        self.grasp_command_count = 0
        self.completed_cycles = 0
        self.bottom_source = "pending"
        self.bottom_depth_m: float | None = None
        self.bottom_stable_s = 0.0
        self.bottom_depth_span_m = 0.0
        self.initial_descent_elapsed_s = 0.0
        self._mission_started_at: float | None = None
        self._state_started_at: float | None = None
        self._phase_started_at: float | None = None
        self._initial_start_depth_m: float | None = None
        self._latest_depth_m: float | None = None
        self._last_depth_at: float | None = None
        self._depth_samples: deque[tuple[float, float]] = deque()

    @property
    def route_direction(self) -> int:
        return -1 if self.lane_index % 2 == 0 else 1

    def step(
        self,
        now: float,
        depth: DepthSample | None = None,
    ) -> BlindGrabDecision:
        if self._mission_started_at is None:
            self._mission_started_at = float(now)
            self._state_started_at = float(now)
        self._accept_depth(depth)

        if self.state is BlindState.INITIAL_DESCENT:
            self._advance_initial_descent(now)
        elif self.state is BlindState.GRABBING:
            self._advance_grab(now)
        elif self._state_elapsed(now) >= self._state_duration():
            self._advance_route(now)
        return self._decision(now)

    def _accept_depth(self, sample: DepthSample | None) -> None:
        if sample is None:
            return
        try:
            received_at = float(sample.received_at)
            depth_m = float(sample.depth_m)
        except (TypeError, ValueError, OverflowError):
            return
        if not math.isfinite(received_at) or not math.isfinite(depth_m):
            return
        if self._last_depth_at is not None and received_at <= self._last_depth_at:
            return
        self._last_depth_at = received_at
        self._latest_depth_m = depth_m
        if self.state is not BlindState.INITIAL_DESCENT:
            return
        if self._initial_start_depth_m is None:
            self._initial_start_depth_m = depth_m
        probe = update_bottom_probe(
            self._depth_samples,
            now=received_at,
            current_depth_m=depth_m,
            probe_start_depth_m=self._initial_start_depth_m,
            stable_duration_required_s=self.config.initial_bottom_stable_s,
            stable_depth_tolerance_m=self.config.initial_bottom_tolerance_m,
            minimum_descent_m=self.config.initial_minimum_descent_m,
            neutral_confirmation_s=self.config.initial_bottom_stable_s,
        )
        self.bottom_stable_s = probe.stable_duration_s
        self.bottom_depth_span_m = probe.depth_span_m
        if probe.confirmed:
            self.bottom_source = "pressure"
            self.bottom_depth_m = probe.bottom_depth_m

    def _advance_initial_descent(self, now: float) -> None:
        self.initial_descent_elapsed_s = self._state_elapsed(now)
        if self.bottom_source == "pressure":
            self._begin_grab(now)
            return
        if self._state_elapsed(now) >= self.config.initial_fallback_s:
            self.bottom_source = "timer"
            self.bottom_depth_m = self._latest_depth_m
            self._begin_grab(now)

    def _begin_grab(self, now: float) -> None:
        self.state = BlindState.GRABBING
        self._state_started_at = float(now)
        self._enter_grab_phase(GrabPhase.OPEN, now)

    def _enter_grab_phase(self, phase: GrabPhase, now: float) -> None:
        self.grab_phase = phase
        self._phase_started_at = float(now)
        if phase is GrabPhase.CLOSE:
            self.grasp_command_count += 1

    def _advance_grab(self, now: float) -> None:
        assert self.grab_phase is not None
        assert self._phase_started_at is not None
        durations = {
            GrabPhase.OPEN: self.config.open_gripper.duration_s,
            GrabPhase.ADVANCE: self.config.advance_duration_s,
            GrabPhase.CLOSE: self.config.close_gripper.duration_s,
            GrabPhase.TRANSFER: self.config.arm_to_basket.duration_s,
            GrabPhase.RELEASE: self.config.release_duration_s,
            GrabPhase.RETURN: self.config.arm_to_grasp.duration_s,
        }
        if now - self._phase_started_at < durations[self.grab_phase]:
            return
        phases = tuple(GrabPhase)
        if self.grab_phase is not GrabPhase.RETURN:
            self._enter_grab_phase(phases[phases.index(self.grab_phase) + 1], now)
            return
        self.completed_cycles += 1
        self.grab_phase = None
        self._enter_state(BlindState.ASCENDING, now)

    def _state_duration(self) -> float:
        durations = {
            BlindState.ASCENDING: self.config.ascent_duration_s,
            BlindState.FORWARD: self.config.route_step_duration_s,
            BlindState.SHIFT: self.config.shift_duration_s,
            BlindState.TURN: self.config.turn_duration_s,
            BlindState.REPEAT_DESCENT: self.config.repeat_descent_duration_s,
        }
        return durations[self.state]

    def _advance_route(self, now: float) -> None:
        if self.state is BlindState.ASCENDING:
            self._enter_state(BlindState.FORWARD, now)
        elif self.state is BlindState.FORWARD:
            self.segment_in_lane += 1
            if self.segment_in_lane >= self.config.route_steps_per_lane:
                self._enter_state(BlindState.SHIFT, now)
            else:
                self._enter_state(BlindState.REPEAT_DESCENT, now)
        elif self.state is BlindState.SHIFT:
            self._enter_state(BlindState.TURN, now)
        elif self.state is BlindState.TURN:
            self.lane_index += 1
            self.segment_in_lane = 0
            self._enter_state(BlindState.REPEAT_DESCENT, now)
        elif self.state is BlindState.REPEAT_DESCENT:
            self._begin_grab(now)

    def _enter_state(self, state: BlindState, now: float) -> None:
        self.state = state
        self._state_started_at = float(now)

    def _state_elapsed(self, now: float) -> float:
        if self._state_started_at is None:
            return 0.0
        return max(0.0, float(now) - self._state_started_at)

    def _decision(self, now: float) -> BlindGrabDecision:
        c = self.config
        motion = MotionCommand.neutral()
        claw = c.open_gripper.outputs
        arm = c.arm_to_grasp.outputs
        phase = self.state.value
        phase_elapsed_s = self._state_elapsed(now)

        if self.state is BlindState.INITIAL_DESCENT:
            motion = MotionCommand(vertical=c.initial_descent_command)
        elif self.state is BlindState.ASCENDING:
            motion = MotionCommand(vertical=c.ascent_command)
        elif self.state is BlindState.FORWARD:
            motion = MotionCommand(forward=c.route_forward_command)
        elif self.state is BlindState.SHIFT:
            motion = MotionCommand(lateral=self.route_direction * c.shift_command)
        elif self.state is BlindState.TURN:
            motion = MotionCommand(yaw=self.route_direction * c.turn_command)
        elif self.state is BlindState.REPEAT_DESCENT:
            motion = MotionCommand(vertical=c.repeat_descent_command)
        elif self.state is BlindState.GRABBING:
            assert self.grab_phase is not None
            assert self._phase_started_at is not None
            phase = self.grab_phase.value
            phase_elapsed_s = max(0.0, float(now) - self._phase_started_at)
            if self.grab_phase is GrabPhase.ADVANCE:
                motion = MotionCommand(forward=c.grab_forward_command)
            if self.grab_phase in {GrabPhase.CLOSE, GrabPhase.TRANSFER}:
                claw = c.close_gripper.outputs
            if self.grab_phase in {GrabPhase.TRANSFER, GrabPhase.RELEASE}:
                arm = c.arm_to_basket.outputs

        return BlindGrabDecision(
            state=self.state,
            motion=motion,
            servos=claw + arm,
            phase=phase,
            phase_elapsed_s=phase_elapsed_s,
            depth_m=self._latest_depth_m,
            initial_descent_elapsed_s=(
                self._state_elapsed(now)
                if self.state is BlindState.INITIAL_DESCENT
                else self.initial_descent_elapsed_s
            ),
            bottom_stable_s=self.bottom_stable_s,
            bottom_depth_span_m=self.bottom_depth_span_m,
            bottom_depth_m=self.bottom_depth_m,
            bottom_source=self.bottom_source,
            lane_index=self.lane_index,
            segment_in_lane=self.segment_in_lane,
            route_direction=self.route_direction,
            grasp_command_count=self.grasp_command_count,
            completed_cycles=self.completed_cycles,
        )
