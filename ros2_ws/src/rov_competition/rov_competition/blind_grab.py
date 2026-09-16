"""持续盲抓的纯状态机：只依赖新检测帧、配置动作和单调时钟。

搜索时序取自 semicircle_search.py 的 _cruise_motion / _lane_shift_motion /
_lane_turn_motion。独立保存有效搜索时间，停车和抓取不会消耗搜索段时间。
没有任务终局、遥测门控、抓取成功判定或累计抓取次数上限。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

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
    lane_forward_duration_s: float
    advance_duration_s: float
    open_gripper: ServoAction
    close_gripper: ServoAction
    arm_to_basket: ServoAction
    arm_to_grasp: ServoAction
    release_duration_s: float
    forward_command: float = 0.23
    shift_command: float = 0.20
    shift_duration_s: float = 4.5
    turn_command: float = 0.20
    turn_duration_s: float = 11.5
    grab_forward_command: float = 0.23
    required_boxes: int = 4
    confirmation_frames: int = 5
    confirmation_duration_s: float = 1.0
    missing_frame_timeout_s: float = 1.0
    fallback_after_s: float = 30.0
    grabs_per_batch: int = 10


@dataclass(frozen=True)
class DetectionCount:
    """frame_id 仅在新的检测消息到来时增加，received_at 使用本机单调时间。"""

    frame_id: int
    received_at: float
    count: int


class BlindState(str, Enum):
    SEARCHING = "searching"
    CONFIRMING = "confirming"
    GRABBING = "grabbing"


class SnakePhase(str, Enum):
    FORWARD = "forward"
    SHIFT = "shift"
    TURN = "turn"


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
    permanent: bool
    threshold: int
    box_count: int
    search_elapsed_s: float
    lane_index: int
    snake_phase: SnakePhase
    snake_elapsed_s: float
    batch_index: int
    grab_in_batch: int
    grasp_command_count: int
    completed_cycles: int
    completed_batches: int


class TimedSnake:
    """原定时蛇形路线；只累计真正处于搜索状态的时间。"""

    def __init__(self, config: BlindGrabConfig) -> None:
        self.config = config
        self.phase = SnakePhase.FORWARD
        self.elapsed_s = 0.0
        self.lane_index = 0

    def account(self, duration_s: float) -> None:
        self.elapsed_s += duration_s

    def motion(self) -> MotionCommand:
        durations = {
            SnakePhase.FORWARD: self.config.lane_forward_duration_s,
            SnakePhase.SHIFT: self.config.shift_duration_s,
            SnakePhase.TURN: self.config.turn_duration_s,
        }
        # 与原实现一样，每次最多推进一个阶段，不在迟到的 tick 中跳过动作。
        if self.elapsed_s >= durations[self.phase]:
            self.elapsed_s = 0.0
            if self.phase is SnakePhase.FORWARD:
                self.phase = SnakePhase.SHIFT
            elif self.phase is SnakePhase.SHIFT:
                self.phase = SnakePhase.TURN
            else:
                self.phase = SnakePhase.FORWARD
                self.lane_index += 1
        direction = -1.0 if self.lane_index % 2 == 0 else 1.0
        if self.phase is SnakePhase.SHIFT:
            return MotionCommand(lateral=direction * self.config.shift_command)
        if self.phase is SnakePhase.TURN:
            return MotionCommand(yaw=direction * self.config.turn_command)
        return MotionCommand(forward=self.config.forward_command)


class BlindGrabMission:
    """从搜索开始；4 框稳定触发一批，找框 30 秒后锁定永久盲抓。"""

    def __init__(self, config: BlindGrabConfig) -> None:
        self.config = config
        self.state = BlindState.SEARCHING
        self.snake = TimedSnake(config)
        self.permanent = False
        self.search_elapsed_s = 0.0
        self.batch_index = 0
        self.grab_in_batch = 0
        self.grasp_command_count = 0
        self.completed_cycles = 0
        self.completed_batches = 0
        self.grab_phase: GrabPhase | None = None
        self._phase_started_at = 0.0
        self._last_tick: float | None = None
        self._latest_frame: DetectionCount | None = None
        self._confirm_started_at = 0.0
        self._confirmation_frames = 0

    def force_permanent(self, now: float) -> None:
        """锁定永久盲抓；已经在抓取时只改变批后去向。"""

        if self.permanent:
            return
        self.permanent = True
        if self.state is not BlindState.GRABBING:
            self._begin_batch(now)

    def step(self, now: float, frame: DetectionCount | None = None) -> BlindGrabDecision:
        if self._last_tick is not None:
            elapsed = max(0.0, now - self._last_tick)
            if self.state is BlindState.SEARCHING:
                self.snake.account(elapsed)
            if self.state is not BlindState.GRABBING:
                self.search_elapsed_s += elapsed
        self._last_tick = now

        new_frame = False
        if frame is not None and (
            self._latest_frame is None or frame.frame_id > self._latest_frame.frame_id
        ):
            # 丢帧超过窗口后到来的新消息不能把断开的确认序列接起来。
            if self.state is BlindState.CONFIRMING and (
                self._latest_frame is None
                or frame.received_at - self._latest_frame.received_at
                >= self.config.missing_frame_timeout_s
            ):
                self._begin_confirmation(now)
            self._latest_frame = frame
            new_frame = True

        count = self._current_count(now)
        if self.state is BlindState.GRABBING:
            self._advance_grab(now, count)
        elif self.search_elapsed_s >= self.config.fallback_after_s:
            # 截止时间优先于同一 tick 的确认结果；门槛一旦为零永不恢复。
            self.permanent = True
            self._begin_batch(now)
        elif self.state is BlindState.SEARCHING:
            if count >= self.config.required_boxes:
                self._begin_confirmation(now)
                # 触发停车的那张图拍摄于停车前，不计入停车后的五张新图。
        else:
            if count < self.config.required_boxes:
                self.state = BlindState.SEARCHING
                self._confirmation_frames = 0
            elif new_frame and self._latest_frame is not None:
                if self._latest_frame.received_at > self._confirm_started_at:
                    self._confirmation_frames += 1
                if (
                    self._confirmation_frames >= self.config.confirmation_frames
                    and self._latest_frame.received_at - self._confirm_started_at
                    >= self.config.confirmation_duration_s
                ):
                    self._begin_batch(now)
        return self._decision(now)

    def _current_count(self, now: float) -> int:
        if self._latest_frame is None:
            return 0
        age = now - self._latest_frame.received_at
        if not 0.0 <= age < self.config.missing_frame_timeout_s:
            return 0
        return self._latest_frame.count

    def _begin_confirmation(self, now: float) -> None:
        self.state = BlindState.CONFIRMING
        self._confirm_started_at = now
        self._confirmation_frames = 0

    def _begin_batch(self, now: float) -> None:
        self.state = BlindState.GRABBING
        self.search_elapsed_s = 0.0
        self.batch_index += 1
        self.grab_in_batch = 1
        self._enter_grab_phase(GrabPhase.OPEN, now)

    def _enter_grab_phase(self, phase: GrabPhase, now: float) -> None:
        self.grab_phase = phase
        self._phase_started_at = now
        if phase is GrabPhase.CLOSE:
            # 统计的是安排闭爪指令的次数，不是 ACK 或实际抓获数。
            self.grasp_command_count += 1

    def _advance_grab(self, now: float, count: int) -> None:
        durations = {
            GrabPhase.OPEN: self.config.open_gripper.duration_s,
            GrabPhase.ADVANCE: self.config.advance_duration_s,
            GrabPhase.CLOSE: self.config.close_gripper.duration_s,
            GrabPhase.TRANSFER: self.config.arm_to_basket.duration_s,
            GrabPhase.RELEASE: self.config.release_duration_s,
            GrabPhase.RETURN: self.config.arm_to_grasp.duration_s,
        }
        assert self.grab_phase is not None
        if now - self._phase_started_at < durations[self.grab_phase]:
            return
        phases = tuple(GrabPhase)
        if self.grab_phase is not GrabPhase.RETURN:
            self._enter_grab_phase(phases[phases.index(self.grab_phase) + 1], now)
            return
        self.completed_cycles += 1
        if self.grab_in_batch < self.config.grabs_per_batch:
            self.grab_in_batch += 1
            self._enter_grab_phase(GrabPhase.OPEN, now)
            return
        self.completed_batches += 1
        if self.permanent:
            self._begin_batch(now)
            return
        self.grab_phase = None
        self.grab_in_batch = 0
        self.search_elapsed_s = 0.0
        if count >= self.config.required_boxes:
            self._begin_confirmation(now)
        else:
            self.state = BlindState.SEARCHING

    def _decision(self, now: float) -> BlindGrabDecision:
        c = self.config
        motion = MotionCommand.neutral()
        claw = c.open_gripper.outputs
        arm = c.arm_to_grasp.outputs
        phase = self.state.value
        if self.state is BlindState.SEARCHING:
            motion = self.snake.motion()
            phase = self.snake.phase.value
        elif self.state is BlindState.GRABBING:
            assert self.grab_phase is not None
            phase = self.grab_phase.value
            if self.grab_phase is GrabPhase.ADVANCE:
                motion = MotionCommand(forward=c.grab_forward_command)
            if self.grab_phase in {GrabPhase.CLOSE, GrabPhase.TRANSFER}:
                claw = c.close_gripper.outputs
            if self.grab_phase in {GrabPhase.TRANSFER, GrabPhase.RELEASE}:
                arm = c.arm_to_basket.outputs
        return BlindGrabDecision(
            state=self.state, motion=motion, servos=claw + arm, phase=phase,
            permanent=self.permanent,
            threshold=0 if self.permanent else c.required_boxes,
            box_count=self._current_count(now), search_elapsed_s=self.search_elapsed_s,
            lane_index=self.snake.lane_index, snake_phase=self.snake.phase,
            snake_elapsed_s=self.snake.elapsed_s, batch_index=self.batch_index,
            grab_in_batch=self.grab_in_batch, grasp_command_count=self.grasp_command_count,
            completed_cycles=self.completed_cycles, completed_batches=self.completed_batches,
        )
