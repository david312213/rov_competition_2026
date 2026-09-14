"""Single-target visual approach and sequential bottom-side deposit.

No hardware imports. Mechanical transitions require an execution-complete event,
not a service ACK. Even that event is not physical position/catch feedback.
"""
from __future__ import annotations

from dataclasses import replace
import math

from .cluster_collection import (
    ClusterCollectionMission, ClusterCollectionState as State,
    ClusterCollectionError, cluster_scallops, select_cluster,
)
from .domain import Detection, MotionCommand


def choose_target(detections, width, aim_x):
    return max(detections, key=lambda d: (
        d.confidence, -abs(d.box.center()[0] / width - aim_x),
        -d.box.left, -d.box.top,
    ))


def associate_target(previous, detections, width, height, max_jump=0.25):
    """Geometry, never confidence, preserves the lock. Ambiguity is a miss."""
    if previous is None or width <= 0 or height <= 0:
        return None
    px, py = previous.box.center()
    ranked = []
    for detection in detections:
        box = detection.box
        if detection.label != previous.label or box.width() <= 0 or box.height() <= 0:
            continue
        if not all(math.isfinite(v) for v in (box.left, box.top, box.right, box.bottom)):
            continue
        x, y = box.center()
        distance = math.hypot((x - px) / width, (y - py) / height)
        iou = previous.box.intersection_over_union(box)
        if distance <= max_jump and (iou >= 0.10 or distance <= 0.08):
            ranked.append((iou, -distance, detection))
    ranked.sort(key=lambda row: row[:2], reverse=True)
    if not ranked:
        return None
    if len(ranked) > 1 and (
        abs(ranked[0][0] - ranked[1][0]) < 0.05
        and abs(ranked[0][1] - ranked[1][1]) < 0.02
    ):
        return None
    return ranked[0][2]


class TargetGraspMission(ClusterCollectionMission):
    """Uses old search/clearance routines, but never the old blind deposit path."""
    def __init__(self, config, *, stop_after_approach=False):
        super().__init__(config, stop_after_approach=stop_after_approach)
        self.selected_target: Detection | None = None
        self.selected_frame_size = None
        self.target_bottom_ratio = None
        self.target_x_ratio = None
        self.descent_vertical_only = False
        self.drop_started_at = None
        self.group_reference = None
        self.reacquire_started_at = None
        self.execution_request_id = None
        self.execution_deadline = None
        self.execution_status_at = None
        self.execution_status_stamp = None
        self.completed_execution_ids = set()
        self.descent_crossing_latched = False
        self.grasp_started_at = None

    def _clear_cluster_lock(self):
        super()._clear_cluster_lock()
        self.selected_target = None
        self.selected_frame_size = None
        self.target_bottom_ratio = None
        self.target_x_ratio = None
        self.descent_crossing_latched = False

    def _set_target(self, target, observation):
        self.selected_target = target
        self.selected_frame_size = (observation.frame_width, observation.frame_height)
        self.target_x_ratio = target.box.center()[0] / observation.frame_width
        self.target_bottom_ratio = min(1.0, max(0.0, target.box.bottom / observation.frame_height))

    def _decision(self, motion, observation, message, **kwargs):
        decision = super()._decision(motion, observation, message, **kwargs)
        if self.target_x_ratio is not None:
            decision = replace(decision, horizontal_error=self.target_x_ratio - self.config.aim_x_ratio)
        return replace(decision, selected_target=self.selected_target,
            target_bottom_ratio=self.target_bottom_ratio,
            descent_vertical_only=self.descent_vertical_only)

    def _step_aligning(self, observation, now):
        if self.selected_target is None:
            if self.tracked_cluster is None:
                return self.abort("单目标对准缺少群体", observation)
            self._set_target(choose_target(self.tracked_cluster.detections,
                observation.frame_width, self.config.aim_x_ratio), observation)
            self.group_reference = (self.tracked_cluster.center_x_ratio, self.tracked_cluster.center_y_ratio)
        error = self.target_x_ratio - self.config.aim_x_ratio
        if abs(error) <= self.config.horizontal_tolerance:
            self.aligned_frames += 1
            if self.aligned_frames >= self.config.alignment_frames:
                self._enter(State.APPROACHING, observation, now)
            decision = self._decision(MotionCommand.neutral(), observation, "单目标居中，等待稳定/开始接近")
        else:
            self.aligned_frames = 0
            yaw = self.config.image_yaw_sign * math.copysign(max(
                self.config.minimum_yaw_command,
                min(self.config.maximum_yaw_command, abs(error) * self.config.yaw_gain)), error)
            decision = self._decision(MotionCommand(yaw=yaw), observation, "对准锁定扇贝")
        self.last_tracking_decision = decision
        return decision

    def _match(self, observation):
        if self.selected_frame_size != (observation.frame_width, observation.frame_height):
            return None
        return associate_target(self.selected_target, observation.detections,
            observation.frame_width, observation.frame_height, self.config.maximum_tracking_jump_ratio)

    def _update_tracking(self, observation, now):
        match = self._match(observation)
        if match is not None:
            self._set_target(match, observation)
            self.last_target_seen_at = now
            self.consecutive_misses = 0
            if self.state == State.VERIFYING_LOSS:
                self._enter(State.ALIGNING, observation, now)
                return self._step_aligning(observation, now)
            return None
        self.consecutive_misses += 1
        self.aligned_frames = 0
        self.descent_history.clear()
        if self.state != State.VERIFYING_LOSS:
            self._enter(State.VERIFYING_LOSS, observation, now)
        if (self.consecutive_misses >= self.config.loss_required_frames
                and now - (self.last_target_seen_at if self.last_target_seen_at is not None else now)
                >= self.config.loss_confirmation_s):
            self._clear_cluster_lock()
            self.grasp_attempts_completed = 0
            self._enter(State.SCANNING, observation, now)
        return self._decision(MotionCommand.neutral(), observation, "选中目标丢失，停车确认/重新搜索")

    def _step_approaching(self, observation, now):
        if self.selected_target is None:
            return self.abort("接近时单目标缺失", observation)
        error = self.target_x_ratio - self.config.aim_x_ratio
        if abs(error) > self.config.realign_threshold:
            self.descent_history.clear()
            self._enter(State.ALIGNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "偏差过大，重新对准")
        crossed = self.target_bottom_ratio >= self.config.target_bottom_line_y_ratio
        self.descent_crossing_latched |= crossed
        self.descent_history.append(crossed and abs(error) <= self.config.horizontal_tolerance)
        if (len(self.descent_history) == self.config.descent_window_frames
                and sum(self.descent_history) >= self.config.descent_required_hits
                and crossed and abs(error) <= self.config.horizontal_tolerance):
            if self.stop_after_approach:
                self._enter(State.COMPLETE, observation, now)
                return self._decision(MotionCommand.neutral(), observation, "下边缘接近达标；无爪模式停止")
            self.current_grasp_attempt_index = self.grasp_attempts_completed + 1
            return self._action("open", State.OPENING_GRIPPER, observation, now)
        forward = 0.0 if self.descent_crossing_latched or abs(error) > self.config.horizontal_tolerance else (
            self.config.far_forward_command if self.target_bottom_ratio < self.config.target_near_bottom_y_ratio
            else self.config.near_forward_command)
        yaw = max(-self.config.approach_yaw_command, min(self.config.approach_yaw_command,
            self.config.image_yaw_sign * error * self.config.yaw_gain))
        decision = self._decision(MotionCommand(forward=forward, yaw=yaw), observation,
            f"选中框下缘={self.target_bottom_ratio:.3f}，越线确认={sum(self.descent_history)}/{self.config.descent_window_frames}")
        self.last_tracking_decision = decision
        return decision

    def _action(self, action, state, observation, now):
        self._request_gripper(state, action, observation, now)
        self.execution_request_id = None
        self.execution_deadline = None
        self.execution_status_at = now
        self.execution_status_stamp = None
        return self._decision(MotionCommand.neutral(), observation,
            f"请求机械动作 {action}；等待序列完成和标定等待", gripper_action=action)

    def bind_execution(self, request_id, now, timeout_s):
        if self.awaiting_gripper_action is None or not request_id or timeout_s <= 0:
            raise ValueError("无待执行动作或无效请求")
        self.execution_request_id = request_id
        self.execution_deadline = now + timeout_s
        self.execution_status_at = now

    def execution_status(self, request_id, action, state, observation, now, *, stamp=None, allow_transition=True):
        if (request_id != self.execution_request_id or request_id in self.completed_execution_ids
                or action != self.awaiting_gripper_action):
            return None
        if stamp is not None:
            if self.execution_status_stamp is not None and stamp <= self.execution_status_stamp:
                return None
            self.execution_status_stamp = stamp
        self.execution_status_at = now
        if state in {"failed", "cancelled"}:
            return self.abort(f"机械动作 {action} {state}", observation)
        if state not in {"accepted", "running", "completed"}:
            return self.abort("未知机械执行状态", observation)
        if state != "completed" or not allow_transition:
            return None
        try:
            self._validate_observation(observation, require_attitude=False)
        except ClusterCollectionError as exc:
            return self.abort(str(exc), observation)
        depth_check = self._check_depth_sample(observation, now)
        if depth_check is not None:
            return depth_check
        self.completed_execution_ids.add(request_id)
        self.execution_request_id = None
        self.execution_deadline = None
        self.awaiting_gripper_action = None
        self.last_gripper_accepted = True
        return self._mechanical_complete(action, observation, now)

    def acknowledge_gripper(self, action, accepted, message, observation, now):
        if not accepted:
            return self.abort(f"机械动作被拒绝：{message}", observation)
        # Acceptance cannot trigger mechanical transitions in this workflow.
        self.last_gripper_accepted = True
        return self._decision(MotionCommand.neutral(), observation, "命令已接受，仍等待执行完成")

    def _mechanical_complete(self, action, observation, now):
        if self.state == State.OPENING_GRIPPER and action == "open":
            self._start_probe(observation, now, purpose="grasp")
            self.grasp_started_at = now
            target = self._match(observation)
            self.descent_vertical_only = target is None or target.box.bottom >= observation.frame_height - 1
            if target is not None:
                self._set_target(target, observation)
            return self._decision(MotionCommand.neutral(), observation, "开爪动作结束，开始斜向下降")
        if self.state == State.CLOSING_GRIPPER and action == "close":
            return self._action("raise", State.RAISING_CLAW, observation, now)
        if self.state == State.RAISING_CLAW and action == "raise":
            return self._action("open", State.RELEASING_CATCH, observation, now)
        if self.state == State.RELEASING_CATCH and action == "open":
            self._enter(State.WAITING_DROP, observation, now)
            self.drop_started_at = now
            return self._decision(MotionCommand.neutral(), observation, "张爪完成，等待落料（无入网传感器）")
        if self.state == State.CLOSING_FOR_RESET and action == "close":
            return self._action("reset", State.RESETTING_CLAW, observation, now)
        if self.state == State.RESETTING_CLAW and action == "reset":
            self.grasp_attempts_completed += 1
            self.total_grasp_attempt_count += 1
            self.clearance_purpose = "after_deposit"
            self.target_depth_m = max(float(self.start_depth_m or 0),
                self.bottom_depth_m - self.config.bottom_clearance_m)
            self._enter(State.CLEARING_BOTTOM, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "投放复位循环结束，开始机器人上浮离底")
        return self.abort("机械动作与当前阶段不匹配", observation)

    def _step_probing_bottom(self, observation, now):
        decision = super()._step_probing_bottom(observation, now)
        if decision.gripper_action is not None:
            # Base probe requests close. Register it with execution tracking.
            return self._action("close", State.CLOSING_GRIPPER, observation, now)
        if self.probe_purpose != "grasp" or self.state != State.GRASP_DESCENDING:
            return decision
        if not observation.perception_valid or now - self.last_new_frame_at > self.config.tracking_command_hold_s:
            return replace(decision, motion=MotionCommand.neutral(), message="等待新鲜检测帧")
        if self._descent_new_frame and not self.descent_vertical_only:
            target = self._match(observation)
            if target is None or target.box.bottom >= observation.frame_height - 1:
                self.descent_vertical_only = True
            else:
                self._set_target(target, observation)
        if decision.motion.vertical == 0:
            return decision
        error = 0.0 if self.target_x_ratio is None else self.target_x_ratio - self.config.aim_x_ratio
        forward = 0.0 if self.descent_vertical_only or abs(error) > self.config.horizontal_tolerance else self.config.grasp_forward_command
        yaw = 0.0 if self.descent_vertical_only else max(-self.config.approach_yaw_command,
            min(self.config.approach_yaw_command, self.config.image_yaw_sign * error * self.config.yaw_gain))
        return replace(decision, motion=MotionCommand(forward=forward, yaw=yaw,
            vertical=-self.config.grasp_descent_command), message=(
            "目标出画已锁存：仅垂直下降" if self.descent_vertical_only else "单目标斜向下降抓取"))

    def _step_clearing_bottom(self, observation, now):
        depositing = self.clearance_purpose == "after_deposit"
        reference = self.group_reference
        decision = super()._step_clearing_bottom(observation, now)
        if depositing and decision.state == State.SCANNING:
            if self.grasp_attempts_completed >= self.config.grabs_per_cluster:
                self.completed_cluster_count += 1
                self.grasp_attempts_completed = 0
                self._enter(State.LEAVING_CLUSTER, observation, now)
                return self._decision(MotionCommand.neutral(), observation, "三次投放循环完成，离开当前群体")
            self.group_reference = reference
            self.reacquire_started_at = now
            self.consecutive_misses = 0
            self._enter(State.REACQUIRING_GROUP, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "离底完成，重新观察当前群体")
        return decision

    def step(self, observation, now):
        if self.state in {State.COMPLETE, State.ABORTED, State.IDLE}:
            return super().step(observation, now)
        self._descent_new_frame = observation.frame_id > self.last_frame_id
        if self.state == State.GRASP_DESCENDING and now - (self.grasp_started_at if self.grasp_started_at is not None else self.state_started_at) >= self.config.grasp_descent_timeout_s:
            return self.abort("抓取下降超时", observation)
        if self.awaiting_gripper_action is not None:
            if self.execution_deadline is not None and now >= self.execution_deadline:
                return self.abort("机械动作执行超时", observation)
            if self.execution_request_id and now - self.execution_status_at > 1.0:
                return self.abort("机械动作状态反馈过期", observation)
        return super().step(observation, now)

    def _step_extension(self, observation, now, new_frame):
        if self.state in {State.WAITING_DROP, State.REACQUIRING_GROUP}:
            if not observation.perception_valid:
                return self._decision(MotionCommand.neutral(), observation, "等待新鲜视频，禁止下一机械动作")
            if self.state == State.WAITING_DROP:
                if now - self.drop_started_at >= self.config.transfer_to_net_wait_s:
                    return self._action("close", State.CLOSING_FOR_RESET, observation, now)
                return self._decision(MotionCommand.neutral(), observation, "底部投放落料等待")
            if new_frame:
                groups = cluster_scallops(observation.detections, observation.frame_width,
                    observation.frame_height, label=self.config.target_label,
                    link_distance_ratio=self.config.cluster_link_distance_ratio)
                nearby = [g for g in groups if self.group_reference is not None and math.hypot(
                    g.center_x_ratio - self.group_reference[0], g.center_y_ratio - self.group_reference[1])
                    <= self.config.maximum_tracking_jump_ratio]
                candidate = select_cluster(nearby, minimum_count=1, aim_x_ratio=self.config.aim_x_ratio)
                if candidate is not None:
                    center = (candidate.center_x_ratio, candidate.center_y_ratio)
                    if self.candidate_center is not None and math.dist(center, self.candidate_center) > self.config.maximum_tracking_jump_ratio:
                        self.candidate_history.clear()
                    self.candidate_center = center
                self.candidate_history.append(candidate is not None)
                if candidate is not None:
                    self.consecutive_misses = 0
                    self.reacquire_started_at = now
                    if sum(self.candidate_history) >= self.config.acquisition_required_hits:
                        self.tracked_cluster = candidate
                        self.smoothed_center = (candidate.center_x_ratio, candidate.center_y_ratio)
                        self.last_target_seen_at = now
                        self._enter(State.ALIGNING, observation, now)
                        return self._step_aligning(observation, now)
                else:
                    self.consecutive_misses += 1
                if self.consecutive_misses >= self.config.loss_required_frames and now - self.reacquire_started_at >= self.config.loss_confirmation_s:
                    self.grasp_attempts_completed = 0
                    self._clear_cluster_lock()
                    self._enter(State.SCANNING, observation, now)
            return self._decision(MotionCommand.neutral(), observation, "等待当前群体的新目标")
        return None

    def request_gripper_failure_recovery(self, observation, now, reason):
        return self.abort(reason, observation)

    def request_normal_finish(self, observation, now, reason):
        if not self.stop_after_approach and self.state in {
            State.OPENING_GRIPPER, State.GRASP_DESCENDING, State.CLOSING_GRIPPER,
            State.RAISING_CLAW, State.RELEASING_CATCH, State.WAITING_DROP,
            State.CLOSING_FOR_RESET, State.RESETTING_CLAW,
        }:
            return self.abort(f"{reason}；机械序列中禁止自动回收或复位", observation)
        return super().request_normal_finish(observation, now, reason)

    def delay_timers(self, duration_s):
        super().delay_timers(duration_s)
        # Hardware execution deadlines intentionally remain wall-clock based.
        if self.drop_started_at is not None:
            self.drop_started_at += max(0.0, duration_s)
