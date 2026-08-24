"""不依赖 ROS、YOLO 和 MAVLink 的单目标自主抓取状态机。

该模块只接收 :class:`MissionObservation` 并输出归一化运动意图。它不知道
八个推进器的编号，也不直接发 PWM；混控由 ArduSub 完成。把核心逻辑与
硬件隔离，可以在不连实艇的电脑上完整测试正常路径、边界条件和安全退路。
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from .config import MissionConfig
from .domain import (
    Detection,
    GripperAction,
    MissionDecision,
    MissionObservation,
    MissionOutcome,
    MissionState,
    MotionCommand,
)


def _clamp(value: float, limit: float) -> float:
    """把有限数限制到对称区间。"""

    if not math.isfinite(value):
        raise ValueError("控制量必须是有限数")
    return max(-limit, min(limit, value))


def signed_yaw_delta_deg(previous: float, current: float) -> float:
    """返回航向从 ``previous`` 到 ``current`` 的最短带符号变化。

    ArduSub 航向通常在 ``0..360`` 间循环。该公式会把 ``359 -> 1``
    正确解释为 ``+2°``，而不是 ``-358°``。
    """

    if not math.isfinite(previous) or not math.isfinite(current):
        raise ValueError("航向必须是有限数")
    return (current - previous + 180.0) % 360.0 - 180.0


class AutonomousGraspMission:
    """赛前候选版的单目标自主任务。

    状态机的时间全部使用调用方传入的单调时钟。“连续 N 帧”只会在
    ``frame_id`` 增加时计数，因此 20 Hz 控制循环重复使用同一张图时，
    不会把一帧误检伪造成多帧确认。
    """

    def __init__(self, config: MissionConfig, target_labels: Sequence[str]) -> None:
        """创建任务对象，但不启动。"""

        self.config = config
        self.target_labels = frozenset(str(item).strip() for item in target_labels)
        if not self.target_labels or "" in self.target_labels:
            raise ValueError("自主抓取目标类别不能为空")
        missing = sorted(self.target_labels - set(config.grasp_area_ratios))
        if missing:
            raise ValueError(f"目标类别缺少抓取阈值: {', '.join(missing)}")

        self.state = MissionState.IDLE
        self._outcome = MissionOutcome.NONE
        self._message = "任务尚未启动"
        self._started_at: float | None = None
        self._state_started_at: float | None = None
        self._last_now: float | None = None
        self._last_frame_id: int | None = None

        self._start_depth_m: float | None = None
        self._target_depth_m: float | None = None
        self._settled_since: float | None = None

        self._candidate: Detection | None = None
        self._candidate_frames = 0
        self._tracked: Detection | None = None
        self._last_target_seen_at: float | None = None
        self._aligned_frames = 0
        self._grasp_ready_frames = 0
        self._grasp_attempts = 0

        self._last_yaw_deg: float | None = None
        self._turn_progress_deg = 0.0
        self._last_yaw_progress_at: float | None = None
        self._search_cycle = 0

        self._advance_elapsed_s = 0.0
        self._last_control_at: float | None = None

        self._reacquire_phase = ""
        self._gripper_waiting_for = GripperAction.NONE
        self._gripper_command_accepted = False
        self._gripper_accepted_at: float | None = None

    @property
    def grasp_attempts(self) -> int:
        """返回已被网关接受的闭爪命令次数。"""

        return self._grasp_attempts

    @property
    def is_active(self) -> bool:
        """任务处于非终局状态时返回 ``True``。"""

        return self.state not in {
            MissionState.IDLE,
            MissionState.COMPLETE,
            MissionState.ABORTED,
        }

    def start(self, observation: MissionObservation, now: float) -> MissionDecision:
        """开始任务，先停车并请求打开机械爪。

        该方法不会自动切换飞行模式，也不会自动解锁。这些条件应由
        ROS 节点在调用本方法前检查。
        """

        if self.is_active:
            return self._decision(
                MotionCommand.neutral(), observation, message="任务已在运行"
            )
        error = self._observation_error(observation)
        if error:
            return self._abort(error, observation, now)
        if not math.isfinite(now):
            raise ValueError("任务时间必须是有限数")

        self._reset(now, observation.frame_id)
        self._enter(MissionState.PREPARING, now, observation)
        self._gripper_waiting_for = GripperAction.OPEN
        return self._decision(
            MotionCommand.neutral(),
            observation,
            gripper=GripperAction.OPEN,
            message="准备：四轴回中，请求打开机械爪",
        )

    def acknowledge_gripper(
        self,
        action: GripperAction,
        accepted: bool,
        message: str,
        observation: MissionObservation,
        now: float,
    ) -> MissionDecision:
        """把机械爪服务响应交回状态机。

        ``accepted`` 只代表网关接受了命令，不代表已经物理抓牢。
        """

        if action != self._gripper_waiting_for or action == GripperAction.NONE:
            return self._abort("收到与当前状态不匹配的机械爪响应", observation, now)
        if not accepted:
            detail = message.strip() or "网关未说明原因"
            return self._abort(f"机械爪命令被拒绝: {detail}", observation, now)

        self._gripper_waiting_for = GripperAction.NONE
        self._gripper_command_accepted = True
        self._gripper_accepted_at = now
        if action == GripperAction.OPEN and self.state == MissionState.PREPARING:
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message="开爪渐变已接受，四轴保持中位等待发完",
            )
        if action == GripperAction.CLOSE and self.state == MissionState.GRASPING:
            self._grasp_attempts += 1
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message="闭爪命令已接受，静止保持（未声称已物理抓牢）",
            )
        return self._abort("机械爪响应到达时状态已改变", observation, now)

    def abort(
        self, reason: str, observation: MissionObservation | None, now: float
    ) -> MissionDecision:
        """外部中止入口；无论原状态都立即四轴回中。"""

        return self._abort(reason or "外部紧急中止", observation, now)

    def step(self, observation: MissionObservation, now: float) -> MissionDecision:
        """运行一次固定频率控制周期。"""

        if self.state == MissionState.IDLE:
            return self._decision(MotionCommand.neutral(), observation)
        if self.state == MissionState.ABORTED:
            return self._decision(MotionCommand.neutral(), observation)
        if self.state == MissionState.COMPLETE:
            return self._decision(MotionCommand.neutral(), observation)
        if self._started_at is None or self._state_started_at is None:
            return self._abort("状态机缺少启动时间", observation, now)
        if self._last_now is not None and now < self._last_now:
            return self._abort("单调时钟倒退", observation, now)
        self._last_now = now

        error = self._observation_error(observation)
        if error:
            return self._abort(error, observation, now)
        if now - self._started_at >= self.config.hard_mission_timeout_s:
            return self._abort("达到任务硬超时，执行急停", observation, now)
        if (
            now - self._started_at >= self.config.soft_mission_deadline_s
            and self.state
            not in {MissionState.GRASPING, MissionState.ASCENDING}
        ):
            return self._begin_ascent(
                observation,
                now,
                MissionOutcome.NO_TARGET,
                "达到软截止时间，主动上升回收",
            )

        new_frame = self._consume_new_frame(observation, now)
        if self.state == MissionState.ABORTED:
            return self._decision(MotionCommand.neutral(), observation)
        if self.state == MissionState.PREPARING:
            return self._step_preparing(observation, now)
        if self.state == MissionState.DESCENDING:
            return self._step_descending(observation, now)
        if self.state == MissionState.SCANNING:
            return self._step_scanning(observation, now, new_frame)
        if self.state == MissionState.ADVANCING:
            return self._step_advancing(observation, now, new_frame)
        if self.state == MissionState.ALIGNING:
            return self._step_aligning(observation, now, new_frame)
        if self.state == MissionState.APPROACHING:
            return self._step_approaching(observation, now, new_frame)
        if self.state == MissionState.REACQUIRING:
            return self._step_reacquiring(observation, now, new_frame)
        if self.state == MissionState.GRASPING:
            return self._step_grasping(observation, now)
        if self.state == MissionState.ASCENDING:
            return self._step_ascending(observation, now)
        return self._abort(f"未知任务状态: {self.state.value}", observation, now)

    def _reset(self, now: float, frame_id: int) -> None:
        """清空上一次任务的所有内部记忆。"""

        self._outcome = MissionOutcome.NONE
        self._message = ""
        self._started_at = now
        self._state_started_at = now
        self._last_now = now
        self._last_frame_id = frame_id
        self._start_depth_m = None
        self._target_depth_m = None
        self._settled_since = None
        self._clear_target_lock()
        self._grasp_attempts = 0
        self._last_yaw_deg = None
        self._turn_progress_deg = 0.0
        self._last_yaw_progress_at = now
        self._search_cycle = 0
        self._advance_elapsed_s = 0.0
        self._last_control_at = now
        self._reacquire_phase = ""
        self._gripper_waiting_for = GripperAction.NONE
        self._gripper_command_accepted = False
        self._gripper_accepted_at = None

    def _observation_error(self, observation: MissionObservation) -> str | None:
        """校验控制所依赖的相机、深度和航向观测。"""

        if not observation.perception_valid:
            return "相机或感知数据无效"
        if observation.frame_width <= 0 or observation.frame_height <= 0:
            return "相机画面尺寸无效"
        if not observation.depth_valid or not math.isfinite(observation.depth_m):
            return "深度遥测无效"
        if not 0.0 <= observation.depth_m <= self.config.maximum_operation_depth_m:
            return "深度遥测超出配置范围"
        if not observation.attitude_valid or not math.isfinite(observation.yaw_deg):
            return "航向遥测无效"
        if observation.frame_id < 0:
            return "图像帧编号无效"
        return None

    def _consume_new_frame(self, observation: MissionObservation, now: float) -> bool:
        """判断是否真的到达了一张新图像。"""

        if self._last_frame_id is None:
            self._last_frame_id = observation.frame_id
            return True
        if observation.frame_id < self._last_frame_id:
            self._abort("图像帧编号倒退", observation, now)
            return False
        if observation.frame_id == self._last_frame_id:
            return False
        self._last_frame_id = observation.frame_id
        return True

    def _step_preparing(
        self, observation: MissionObservation, now: float
    ) -> MissionDecision:
        """等待开爪请求被接受，并给渐变曲线留出完整时间。"""

        if not self._gripper_command_accepted:
            if self._elapsed(now) >= self.config.gripper_command_timeout_s:
                return self._abort("打开机械爪的服务响应超时", observation, now)
            return self._decision(
                MotionCommand.neutral(), observation, message="等待网关确认开爪命令"
            )
        if self._gripper_accepted_at is None:
            return self._abort("开爪接受时间丢失", observation, now)
        elapsed = now - self._gripper_accepted_at
        if elapsed < self.config.gripper_hold_s:
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message=(
                    f"开爪渐变发送中，四轴保持中位 "
                    f"{elapsed:.2f}/{self.config.gripper_hold_s:.2f}s"
                ),
            )

        # 开爪等待结束后再记录启动深度，避免在爪子扫动期间就开始下潜。
        self._start_depth_m = observation.depth_m
        self._target_depth_m = observation.depth_m + self.config.descent_delta_m
        if self._target_depth_m > self.config.maximum_operation_depth_m:
            return self._abort("相对下潜目标超过最大作业深度", observation, now)
        self._enter(MissionState.DESCENDING, now, observation)
        return self._decision(
            MotionCommand.neutral(),
            observation,
            message=(
                f"开爪等待结束；从 {self._start_depth_m:.2f} m "
                f"下潜到 {self._target_depth_m:.2f} m"
            ),
        )

    def _step_descending(
        self, observation: MissionObservation, now: float
    ) -> MissionDecision:
        """用真实深度反馈下潜到“启动深度 + 增量”。"""

        if self._target_depth_m is None:
            return self._abort("下潜目标深度未设置", observation, now)
        if self._elapsed(now) >= self.config.descent_timeout_s:
            return self._abort("相对下潜超时", observation, now)

        error_m = self._target_depth_m - observation.depth_m
        if abs(error_m) <= self.config.descent_tolerance_m:
            if self._settled_since is None:
                self._settled_since = now
            if now - self._settled_since >= self.config.descent_settle_s:
                self._search_cycle = 1
                self._enter(MissionState.SCANNING, now, observation)
                return self._decision(
                    MotionCommand.neutral(), observation, message="相对下潜完成，准备向右扫描"
                )
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message="已进入下潜深度容差，等待稳定",
            )

        self._settled_since = None
        # 领域约定 vertical > 0 表示上升；深度误差为正时需要下潜。
        vertical = -_clamp(
            error_m * self.config.descent_gain, self.config.descent_max_command
        )
        return self._decision(
            MotionCommand(vertical=vertical),
            observation,
            message=(
                f"相对下潜：{observation.depth_m:.2f} / "
                f"{self._target_depth_m:.2f} m"
            ),
        )

    def _step_scanning(
        self, observation: MissionObservation, now: float, new_frame: bool
    ) -> MissionDecision:
        """依据真实航向累计扫描角，而不是盲等固定时间。"""

        target = self._acquire_candidate(observation, new_frame)
        if target is not None:
            self._last_yaw_progress_at = now  # 确认目标期间不计“无转向进展”。
            if self._candidate_frames >= self.config.detection_confirmation_frames:
                self._lock_target(target, now)
                self._enter(MissionState.ALIGNING, now, observation)
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    target=target,
                    message=f"已连续确认目标 {target.label}，进入对准",
                )
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                message=(
                    f"扫描中确认 {target.label}: {self._candidate_frames}/"
                    f"{self.config.detection_confirmation_frames} 帧"
                ),
            )

        turn_error = self._update_turn_progress(
            observation.yaw_deg, self.config.scan_direction, now, moving=True
        )
        if turn_error:
            return self._abort(turn_error, observation, now)
        if self._elapsed(now) >= self.config.scan_timeout_s:
            return self._abort("360° 扫描超时", observation, now)
        if self._turn_progress_deg >= (
            self.config.scan_angle_deg - self.config.scan_tolerance_deg
        ):
            self._enter(MissionState.ADVANCING, now, observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message=(
                    f"第 {self._search_cycle} 轮扫描无目标，准备估算前进 "
                    f"{self.config.advance_target_distance_m:.2f} m"
                ),
            )
        return self._decision(
            MotionCommand(yaw=self.config.scan_direction * self.config.scan_yaw_command),
            observation,
            message=(
                f"第 {self._search_cycle} 轮向右扫描："
                f"{self._turn_progress_deg:.1f}/{self.config.scan_angle_deg:.0f}°"
            ),
        )

    def _step_advancing(
        self, observation: MissionObservation, now: float, new_frame: bool
    ) -> MissionDecision:
        """在无 DVL 条件下按“标定速度 × 时间”估算前进。"""

        target = self._acquire_candidate(observation, new_frame)
        if target is not None:
            self._last_control_at = now
            if self._candidate_frames >= self.config.detection_confirmation_frames:
                self._lock_target(target, now)
                self._enter(MissionState.ALIGNING, now, observation)
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    target=target,
                    message=f"前进途中确认目标 {target.label}，进入对准",
                )
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                message=f"前进途中发现候选目标，停车确认第 {self._candidate_frames} 帧",
            )

        if self._last_control_at is None:
            self._last_control_at = now
        delta_s = max(0.0, now - self._last_control_at)
        self._last_control_at = now
        self._advance_elapsed_s += delta_s
        if self._advance_elapsed_s >= self.config.advance_duration_s:
            if self._search_cycle >= self.config.maximum_search_cycles:
                return self._begin_ascent(
                    observation,
                    now,
                    MissionOutcome.NO_TARGET,
                    f"已完成 {self._search_cycle} 轮搜索仍无目标，上升回收",
                )
            self._search_cycle += 1
            self._enter(MissionState.SCANNING, now, observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message=f"估算前进完成，开始第 {self._search_cycle} 轮扫描",
            )
        estimated = min(
            self.config.advance_target_distance_m,
            self._advance_elapsed_s * self.config.advance_estimated_speed_mps,
        )
        return self._decision(
            MotionCommand(forward=self.config.advance_forward_command),
            observation,
            message=(
                f"开环估算前进 {estimated:.2f}/"
                f"{self.config.advance_target_distance_m:.2f} m（非位置测量）"
            ),
        )

    def _step_aligning(
        self, observation: MissionObservation, now: float, new_frame: bool
    ) -> MissionDecision:
        """前进和横移为零，用偏航/升沉将目标移到抓取瞄准点。"""

        target = self._tracked_target(observation)
        if target is None:
            return self._lost_target_decision(observation, now, "对准时丢失目标")
        self._tracked = target
        self._last_target_seen_at = now
        horizontal, vertical = self._position_errors(target, observation)
        aligned = self._is_aligned(horizontal, vertical)
        if new_frame:
            self._aligned_frames = self._aligned_frames + 1 if aligned else 0
        if self._aligned_frames >= self.config.alignment_confirmation_frames:
            self._grasp_ready_frames = 0
            self._enter(MissionState.APPROACHING, now, observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                horizontal_error=horizontal,
                vertical_error=vertical,
                message="目标已稳定对准，准备低速接近",
            )
        return self._decision(
            self._alignment_motion(horizontal, vertical),
            observation,
            target=target,
            horizontal_error=horizontal,
            vertical_error=vertical,
            message=f"对准：水平误差 {horizontal:+.3f}，垂直误差 {vertical:+.3f}",
        )

    def _step_approaching(
        self, observation: MissionObservation, now: float, new_frame: bool
    ) -> MissionDecision:
        """低速前进并小幅修正，达到分类阈值后请求闭爪。"""

        if self._elapsed(now) >= self.config.approach_timeout_s:
            return self._abort("接近目标超时", observation, now)
        target = self._tracked_target(observation)
        if target is None:
            return self._lost_target_decision(observation, now, "接近时丢失目标")
        self._tracked = target
        self._last_target_seen_at = now
        horizontal, vertical = self._position_errors(target, observation)
        area_ratio = target.box.area_ratio(
            observation.frame_width, observation.frame_height
        )
        if area_ratio > self.config.maximum_grasp_area_ratio:
            return self._abort("目标框异常接近整屏，禁止盲抓", observation, now)
        multiplier = self.config.approach_alignment_multiplier
        if (
            abs(horizontal) > self.config.horizontal_tolerance * multiplier
            or abs(vertical) > self.config.vertical_tolerance * multiplier
        ):
            self._aligned_frames = 0
            self._enter(MissionState.ALIGNING, now, observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                horizontal_error=horizontal,
                vertical_error=vertical,
                message="目标偏差过大，立即停止前进并重新对准",
            )

        threshold = self.config.grasp_area_ratios[target.label]
        ready = (
            self._is_aligned(horizontal, vertical)
            and threshold <= area_ratio <= self.config.maximum_grasp_area_ratio
        )
        if new_frame:
            self._grasp_ready_frames = self._grasp_ready_frames + 1 if ready else 0
        if self._grasp_ready_frames >= self.config.grasp_area_confirmation_frames:
            self._enter(MissionState.GRASPING, now, observation)
            self._gripper_waiting_for = GripperAction.CLOSE
            self._gripper_command_accepted = False
            self._gripper_accepted_at = None
            return self._decision(
                MotionCommand.neutral(),
                observation,
                gripper=GripperAction.CLOSE,
                target=target,
                horizontal_error=horizontal,
                vertical_error=vertical,
                message=(
                    f"{target.label} 框面积连续 "
                    f"{self.config.grasp_area_confirmation_frames} 帧达到 k，请求闭爪"
                ),
            )

        correction = self._alignment_motion(horizontal, vertical)
        # 接近阈值时自动降速，但不低于配置指令的 40%。
        distance_factor = max(0.4, min(1.0, (threshold - area_ratio) / threshold))
        return self._decision(
            MotionCommand(
                forward=self.config.approach_forward_command * distance_factor,
                vertical=correction.vertical,
                yaw=correction.yaw,
            ),
            observation,
            target=target,
            horizontal_error=horizontal,
            vertical_error=vertical,
            message=(
                f"低速接近：框面积 {area_ratio:.3f}，k={threshold:.3f}；"
                "速度为归一化指令，非 m/s 实测"
            ),
        )

    def _step_reacquiring(
        self, observation: MissionObservation, now: float, new_frame: bool
    ) -> MissionDecision:
        """丢框后先左转 60°，再右转 120°，全程按真实航向闭环。"""

        target = self._reacquire_candidate(observation, new_frame)
        if target is not None and self._candidate_frames >= self.config.detection_confirmation_frames:
            self._lock_target(target, now)
            self._enter(MissionState.ALIGNING, now, observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                message="重新找到原目标，统一返回对准状态",
            )
        if target is not None:
            self._last_yaw_progress_at = now
            return self._decision(
                MotionCommand.neutral(),
                observation,
                target=target,
                message=f"重新捕获候选确认 {self._candidate_frames} 帧",
            )

        if self._elapsed(now) >= self.config.reacquire_timeout_s:
            return self._abort("丢框重新捕获超时", observation, now)
        if self._reacquire_phase == "grace":
            if self._elapsed(now) < self.config.reacquire_grace_s:
                return self._decision(
                    MotionCommand.neutral(), observation, message="丢框后先停车，等待短暂恢复"
                )
            self._reacquire_phase = "left"
            self._reset_turn(observation.yaw_deg, now)

        if self._reacquire_phase == "left":
            error = self._update_turn_progress(observation.yaw_deg, -1, now, moving=True)
            if error:
                return self._abort(error, observation, now)
            if self._turn_progress_deg >= (
                self.config.reacquire_first_turn_deg - self.config.scan_tolerance_deg
            ):
                self._reacquire_phase = "right"
                self._reset_turn(observation.yaw_deg, now)
                return self._decision(
                    MotionCommand.neutral(), observation, message="左转 60° 完成，准备右转 120°"
                )
            return self._decision(
                MotionCommand(yaw=-self.config.reacquire_yaw_command),
                observation,
                message=(
                    f"丢框搜索：左转 {self._turn_progress_deg:.1f}/"
                    f"{self.config.reacquire_first_turn_deg:.0f}°"
                ),
            )

        if self._reacquire_phase == "right":
            error = self._update_turn_progress(observation.yaw_deg, 1, now, moving=True)
            if error:
                return self._abort(error, observation, now)
            if self._turn_progress_deg >= (
                self.config.reacquire_second_turn_deg - self.config.scan_tolerance_deg
            ):
                self._clear_target_lock()
                self._enter(MissionState.SCANNING, now, observation)
                return self._decision(
                    MotionCommand.neutral(),
                    observation,
                    message="60°/120° 恢复搜索未找到目标，回到 360° 扫描",
                )
            return self._decision(
                MotionCommand(yaw=self.config.reacquire_yaw_command),
                observation,
                message=(
                    f"丢框搜索：右转 {self._turn_progress_deg:.1f}/"
                    f"{self.config.reacquire_second_turn_deg:.0f}°"
                ),
            )
        return self._abort("重新捕获子状态无效", observation, now)

    def _step_grasping(
        self, observation: MissionObservation, now: float
    ) -> MissionDecision:
        """等待闭爪接受并保持静止。"""

        if not self._gripper_command_accepted:
            if self._elapsed(now) >= self.config.gripper_command_timeout_s:
                return self._abort("闭爪服务响应超时", observation, now)
            return self._decision(
                MotionCommand.neutral(), observation, message="等待网关确认闭爪命令"
            )
        if self._gripper_accepted_at is None:
            return self._abort("闭爪接受时间丢失", observation, now)
        if now - self._gripper_accepted_at >= self.config.gripper_hold_s:
            return self._begin_ascent(
                observation,
                now,
                MissionOutcome.GRASP_COMMANDED,
                "闭爪保持结束，上升到回收深度",
            )
        return self._decision(
            MotionCommand.neutral(),
            observation,
            message="闭爪命令已接受，机器人静止保持",
        )

    def _step_ascending(
        self, observation: MissionObservation, now: float
    ) -> MissionDecision:
        """依据真实深度上升至绝对回收深度。"""

        if self._elapsed(now) >= self.config.ascent_timeout_s:
            return self._abort("上升回收超时", observation, now)
        error_m = observation.depth_m - self.config.recovery_depth_m
        if error_m <= self.config.ascent_tolerance_m:
            # 若因浪涌略浅于目标，不为了“追 0.30 m”再向下动。
            if self._settled_since is None:
                self._settled_since = now
            if now - self._settled_since >= self.config.ascent_settle_s:
                self._enter(MissionState.COMPLETE, now, observation)
                self._message = (
                    "已到达回收深度；等待 ROS 节点正常上锁并释放控制"
                )
                return self._decision(MotionCommand.neutral(), observation)
            return self._decision(
                MotionCommand.neutral(),
                observation,
                message="已进入回收深度容差，等待稳定",
            )
        self._settled_since = None
        vertical = _clamp(
            error_m * self.config.ascent_gain, self.config.ascent_max_command
        )
        return self._decision(
            MotionCommand(vertical=vertical),
            observation,
            message=(
                f"上升回收：当前 {observation.depth_m:.2f} m，"
                f"目标 ≤ {self.config.recovery_depth_m:.2f} m"
            ),
        )

    def _begin_ascent(
        self,
        observation: MissionObservation,
        now: float,
        outcome: MissionOutcome,
        message: str,
    ) -> MissionDecision:
        """进入上升回收，同时记录最终结果原因。"""

        self._outcome = outcome
        self._target_depth_m = self.config.recovery_depth_m
        self._enter(MissionState.ASCENDING, now, observation)
        return self._decision(MotionCommand.neutral(), observation, message=message)

    def _lost_target_decision(
        self, observation: MissionObservation, now: float, reason: str
    ) -> MissionDecision:
        """丢框时立即停车，超过短暂容忍时间后进入扫角恢复。"""

        if (
            self._last_target_seen_at is not None
            and now - self._last_target_seen_at < self.config.target_lost_timeout_s
        ):
            return self._decision(
                MotionCommand.neutral(), observation, message=f"{reason}，立即停车等待"
            )
        self._candidate = None
        self._candidate_frames = 0
        self._enter(MissionState.REACQUIRING, now, observation)
        return self._decision(
            MotionCommand.neutral(), observation, message=f"{reason}，进入左 60°/右 120° 恢复搜索"
        )

    def _acquire_candidate(
        self, observation: MissionObservation, new_frame: bool
    ) -> Detection | None:
        """未锁定时选框面积最大的允许抓取目标。"""

        candidates = [
            item
            for item in observation.detections
            if item.label in self.target_labels
            and self._valid_detection(item, observation, allow_oversized=False)
        ]
        target = max(
            candidates,
            key=lambda item: (
                item.box.area_ratio(observation.frame_width, observation.frame_height),
                item.confidence,
            ),
            default=None,
        )
        if not new_frame:
            return target
        if target is None:
            self._candidate = None
            self._candidate_frames = 0
            return None
        if self._candidate is not None and self._same_physical_target(
            self._candidate, target, observation
        ):
            self._candidate_frames += 1
        else:
            self._candidate_frames = 1
        self._candidate = target
        return target

    def _reacquire_candidate(
        self, observation: MissionObservation, new_frame: bool
    ) -> Detection | None:
        """恢复搜索时优先保持已锁定类别和空间连续性。"""

        target = self._tracked_target(observation)
        if target is None and self._tracked is not None:
            # 转过 60° 后，原目标可能从画面另一侧重新进入，与丢失
            # 前的框既无 IoU，中心距离也可能较大。此时仍只允许原类别，
            # 并从新位置连续多帧确认；不用单帧结果直接恢复接近。
            same_label = [
                item
                for item in observation.detections
                if item.label == self._tracked.label
                and self._valid_detection(item, observation, allow_oversized=False)
            ]
            target = max(
                same_label,
                key=lambda item: (
                    item.box.area_ratio(
                        observation.frame_width, observation.frame_height
                    ),
                    item.confidence,
                ),
                default=None,
            )
        if target is None:
            if new_frame:
                self._candidate = None
                self._candidate_frames = 0
            return None
        if new_frame:
            if self._candidate is not None and self._same_physical_target(
                self._candidate, target, observation
            ):
                self._candidate_frames += 1
            else:
                self._candidate_frames = 1
            self._candidate = target
        return target

    def _tracked_target(self, observation: MissionObservation) -> Detection | None:
        """通过类别、IoU 和中心距离跟踪同一个物体。"""

        if self._tracked is None:
            return None
        candidates = [
            item
            for item in observation.detections
            if item.label == self._tracked.label
            and self._valid_detection(item, observation, allow_oversized=True)
        ]
        if not candidates:
            return None
        diagonal = math.hypot(observation.frame_width, observation.frame_height)

        def score(item: Detection) -> tuple[float, float, float]:
            iou = self._tracked.box.intersection_over_union(item.box)
            old_x, old_y = self._tracked.box.center()
            new_x, new_y = item.box.center()
            distance = math.hypot(new_x - old_x, new_y - old_y) / diagonal
            return (iou, -distance, item.confidence)

        target = max(candidates, key=score)
        iou = self._tracked.box.intersection_over_union(target.box)
        old_x, old_y = self._tracked.box.center()
        new_x, new_y = target.box.center()
        jump = math.hypot(new_x - old_x, new_y - old_y) / diagonal
        if iou < self.config.minimum_tracking_iou and jump > self.config.maximum_tracking_jump_ratio:
            return None
        return target

    def _same_physical_target(
        self,
        previous: Detection,
        current: Detection,
        observation: MissionObservation,
    ) -> bool:
        """判断两帧中的框是否可视为同一目标。"""

        if previous.label != current.label:
            return False
        iou = previous.box.intersection_over_union(current.box)
        old_x, old_y = previous.box.center()
        new_x, new_y = current.box.center()
        diagonal = math.hypot(observation.frame_width, observation.frame_height)
        jump = math.hypot(new_x - old_x, new_y - old_y) / diagonal
        return iou >= self.config.minimum_tracking_iou or jump <= self.config.maximum_tracking_jump_ratio

    def _valid_detection(
        self,
        detection: Detection,
        observation: MissionObservation,
        *,
        allow_oversized: bool,
    ) -> bool:
        """拒绝非法坐标、空框、非法置信度和获取阶段的异常大框。"""

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
        if detection.box.width() <= 0.0 or detection.box.height() <= 0.0:
            return False
        if (
            detection.box.left < 0.0
            or detection.box.top < 0.0
            or detection.box.right > observation.frame_width
            or detection.box.bottom > observation.frame_height
        ):
            return False
        area = detection.box.area_ratio(
            observation.frame_width, observation.frame_height
        )
        return allow_oversized or area <= self.config.maximum_grasp_area_ratio

    def _lock_target(self, target: Detection, now: float) -> None:
        """将已连续确认的框设为唯一跟踪目标。"""

        self._tracked = target
        self._candidate = target
        self._last_target_seen_at = now
        self._aligned_frames = 0
        self._grasp_ready_frames = 0

    def _clear_target_lock(self) -> None:
        """清除候选和已锁定目标。"""

        self._candidate = None
        self._candidate_frames = 0
        self._tracked = None
        self._last_target_seen_at = None
        self._aligned_frames = 0
        self._grasp_ready_frames = 0

    def _position_errors(
        self, target: Detection, observation: MissionObservation
    ) -> tuple[float, float]:
        """计算目标中心相对可调抓取瞄准点的归一化误差。"""

        center_x, center_y = target.box.center()
        aim_x = observation.frame_width * self.config.grasp_aim_x_ratio
        aim_y = observation.frame_height * self.config.grasp_aim_y_ratio
        horizontal = (center_x - aim_x) / (observation.frame_width / 2.0)
        vertical = (center_y - aim_y) / (observation.frame_height / 2.0)
        return horizontal, vertical

    def _is_aligned(self, horizontal: float, vertical: float) -> bool:
        """目标中心同时进入水平和垂直容差时返回真。"""

        return (
            abs(horizontal) <= self.config.horizontal_tolerance
            and abs(vertical) <= self.config.vertical_tolerance
        )

    def _alignment_motion(self, horizontal: float, vertical: float) -> MotionCommand:
        """把图像误差转换为已限幅的偏航与升沉意图。"""

        if self.config.image_yaw_sign is None or self.config.image_vertical_sign is None:
            # 真实启动门本应在到达此处前拒绝。这里再做一层防御，
            # 使直接调用状态机也不会猜测方向。
            return MotionCommand.neutral()
        return MotionCommand(
            yaw=_clamp(
                self.config.image_yaw_sign * horizontal * self.config.yaw_gain,
                self.config.maximum_yaw_command,
            ),
            vertical=_clamp(
                self.config.image_vertical_sign * vertical * self.config.vertical_gain,
                self.config.maximum_vertical_command,
            ),
        )

    def _reset_turn(self, yaw_deg: float, now: float) -> None:
        """从当前真实航向开始一段新的转角累计。"""

        self._last_yaw_deg = yaw_deg
        self._turn_progress_deg = 0.0
        self._last_yaw_progress_at = now

    def _update_turn_progress(
        self, yaw_deg: float, direction: int, now: float, *, moving: bool
    ) -> str | None:
        """更新转角进度，检测跳变、反向和无进展。"""

        if self._last_yaw_deg is None:
            self._reset_turn(yaw_deg, now)
            return None
        raw_delta = signed_yaw_delta_deg(self._last_yaw_deg, yaw_deg)
        self._last_yaw_deg = yaw_deg
        if abs(raw_delta) > self.config.maximum_yaw_step_deg:
            return f"航向单次跳变 {raw_delta:+.1f}°，停止转向"
        signed_progress = raw_delta * direction
        if signed_progress < -max(2.0, self.config.scan_tolerance_deg):
            return "实际航向变化与命令方向相反"
        if signed_progress > 0.05:
            self._turn_progress_deg += signed_progress
            self._last_yaw_progress_at = now
        if moving and self._last_yaw_progress_at is not None:
            if now - self._last_yaw_progress_at >= self.config.yaw_progress_timeout_s:
                return "偏航命令发出后航向长时间无进展"
        return None

    def _enter(
        self,
        state: MissionState,
        now: float,
        observation: MissionObservation,
    ) -> None:
        """统一进入状态，避免上一状态的计时器泄漏。"""

        self.state = state
        self._state_started_at = now
        self._settled_since = None
        self._message = ""
        if state in {MissionState.SCANNING, MissionState.REACQUIRING}:
            self._reset_turn(observation.yaw_deg, now)
        if state == MissionState.SCANNING:
            self._candidate = None
            self._candidate_frames = 0
        if state == MissionState.ADVANCING:
            self._advance_elapsed_s = 0.0
            self._last_control_at = now
            self._candidate = None
            self._candidate_frames = 0
        if state == MissionState.REACQUIRING:
            self._reacquire_phase = "grace"
            self._candidate = None
            self._candidate_frames = 0

    def _elapsed(self, now: float) -> float:
        """返回当前状态已持续的时间。"""

        return 0.0 if self._state_started_at is None else max(0.0, now - self._state_started_at)

    def _abort(
        self,
        reason: str,
        observation: MissionObservation | None,
        now: float,
    ) -> MissionDecision:
        """进入不可自动恢复的中止状态。"""

        self.state = MissionState.ABORTED
        self._state_started_at = now
        self._outcome = MissionOutcome.ABORTED
        self._message = reason.strip() or "自主任务中止"
        self._gripper_waiting_for = GripperAction.NONE
        return self._decision(MotionCommand.neutral(), observation)

    def _decision(
        self,
        motion: MotionCommand,
        observation: MissionObservation | None,
        *,
        gripper: GripperAction = GripperAction.NONE,
        target: Detection | None = None,
        horizontal_error: float | None = None,
        vertical_error: float | None = None,
        message: str | None = None,
    ) -> MissionDecision:
        """统一生成状态输出和结构化可观测字段。"""

        if message is not None:
            self._message = message
        selected = target if target is not None else self._tracked
        area_ratio: float | None = None
        threshold: float | None = None
        if selected is not None and observation is not None:
            area_ratio = selected.box.area_ratio(
                observation.frame_width, observation.frame_height
            )
            threshold = self.config.grasp_area_ratios.get(selected.label)
        estimated_distance = min(
            self.config.advance_target_distance_m,
            self._advance_elapsed_s * self.config.advance_estimated_speed_mps,
        )
        return MissionDecision(
            state=self.state,
            motion=motion.limited(0.10),
            gripper=gripper,
            selected_target=selected,
            message=self._message,
            grasp_attempts=self._grasp_attempts,
            outcome=self._outcome,
            target_area_ratio=area_ratio,
            grasp_area_threshold=threshold,
            horizontal_error=horizontal_error,
            vertical_error=vertical_error,
            current_depth_m=(
                observation.depth_m
                if observation is not None and observation.depth_valid
                else None
            ),
            target_depth_m=self._target_depth_m,
            scan_progress_deg=(
                self._turn_progress_deg
                if self.state in {MissionState.SCANNING, MissionState.REACQUIRING}
                else None
            ),
            estimated_advance_distance_m=estimated_distance,
            search_cycle=self._search_cycle,
            gripper_command_accepted=self._gripper_command_accepted,
        )
