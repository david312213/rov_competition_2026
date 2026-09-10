"""半圆覆盖与单目标重试。位置仅为推算，不是定位传感器测量。"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import yaml

from .cluster_collection import (
    ClusterCollectionConfig, ClusterCollectionError, ClusterCollectionMission,
    ClusterCollectionState as State, ScallopCluster, DescentTrigger,
)
from .domain import MotionCommand
from .mission import signed_yaw_delta_deg


@dataclass(frozen=True)
class CoverageConfig:
    # 所有实艇数值必须现场填写；测试使用显式合成配置。
    diameter_m: float
    wall_clearance_m: float
    edge_margin_m: float
    lane_spacing_m: float
    waypoint_tolerance_m: float
    search_command: float
    search_speed_mps: float
    far_speed_mps: float
    near_speed_mps: float
    heading_tolerance_deg: float
    heading_gain: float
    turn_timeout_s: float
    segment_timeout_s: float
    max_tick_gap_s: float
    max_heading_step_deg: float
    max_estimated_travel_m: float
    max_local_travel_m: float
    revisit_suppression_m: float
    local_attempt_limit: int
    local_timeout_s: float
    reobserve_timeout_s: float
    observation_depth_m: float
    depth_tolerance_m: float
    descent_command: float
    lift_command: float
    descent_timeout_s: float
    depth_settle_s: float
    grasp_descent_m: float
    transfer_timeout_s: float
    mission_timeout_s: float
    maximum_operation_depth_m: float
    image_yaw_sign: int
    aim_x_ratio: float
    descent_line_y_ratio: float
    far_center_y_ratio: float
    field_calibrated: bool = False

    def __post_init__(self):
        for name, value in vars(self).items():
            if name in {'field_calibrated', 'image_yaw_sign'}:
                continue
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ClusterCollectionError(f'{name} 必须填写实测的正有限数')
        if type(self.field_calibrated) is not bool:
            raise ClusterCollectionError('field_calibrated 必须是布尔值')
        if type(self.local_attempt_limit) is not int:
            raise ClusterCollectionError('local_attempt_limit 必须是整数')
        if type(self.image_yaw_sign) is not int or self.image_yaw_sign not in (-1, 1):
            raise ClusterCollectionError('image_yaw_sign 必须经标定后填 -1 或 1')
        if not 0 < self.aim_x_ratio < 1 or not 0 < self.far_center_y_ratio < self.descent_line_y_ratio < 1:
            raise ClusterCollectionError('图像位置比例无效')
        if max(self.search_command, self.descent_command, self.lift_command) > 1:
            raise ClusterCollectionError('控制量不能超过 1')
        if self.observation_depth_m + self.grasp_descent_m >= self.maximum_operation_depth_m:
            raise ClusterCollectionError('观察深度加抓取下降量必须小于作业深度上限')
        if self.depth_tolerance_m >= self.grasp_descent_m:
            raise ClusterCollectionError('深度容差必须小于抓取下降量')
        if not self.depth_tolerance_m < self.observation_depth_m:
            raise ClusterCollectionError('观察深度必须大于深度容差')
        if not 0 < self.heading_tolerance_deg < self.max_heading_step_deg <= 180:
            raise ClusterCollectionError('航向阈值无效')
        usable = self.diameter_m / 2 - self.edge_margin_m
        if not 0 < self.wall_clearance_m < usable:
            raise ClusterCollectionError('避墙距离与半圆半径不兼容')
        if self.lane_spacing_m >= usable or self.waypoint_tolerance_m >= self.lane_spacing_m / 2:
            raise ClusterCollectionError('行距或航点容差过大')
        if usable / self.lane_spacing_m > 1000:
            raise ClusterCollectionError('搜索行数不能超过1000')
        if self.revisit_suppression_m <= self.waypoint_tolerance_m:
            raise ClusterCollectionError('离开观察点的抑制距离必须大于航点容差')


def load_coverage_config(path):
    try:
        root = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        if not isinstance(root, dict) or not isinstance(root.get('coverage'), dict):
            raise ClusterCollectionError('缺少 coverage 配置段')
        return CoverageConfig(**root['coverage'])
    except (TypeError, ValueError) as exc:
        raise ClusterCollectionError(f'半圆策略配置未完成: {exc}') from exc


def semicircle_waypoints(c: CoverageConfig):
    """x向离墙方向、y沿初始航向。圆心(0,R)，避开墙和弧边。

    连接相邻弦端点，不能只横移后立刻反向：弦端点的y也会变化。
    起点由操作员放在第一条弦的下端附近，并沿墙朝向另一端。
    """
    radius = c.diameter_m / 2
    inner = radius - c.edge_margin_m
    points = []
    x = c.wall_clearance_m
    lane = 0
    while x < inner:
        half = math.sqrt(inner * inner - x * x)
        low, high = radius - half, radius + half
        ends = (low, high) if lane % 2 == 0 else (high, low)
        points.extend((x, y, lane) for y in ends)
        lane += 1
        x = c.wall_clearance_m + lane * c.lane_spacing_m
    return tuple(points)


class CoverageCollectionMission(ClusterCollectionMission):
    """共用原网关决策接口和图像对准，覆盖/下降/重试由本类管理。"""

    def __init__(self, base: ClusterCollectionConfig, coverage: CoverageConfig):
        c = coverage
        super().__init__(replace(base, image_yaw_sign=c.image_yaw_sign,
            descent_trigger=DescentTrigger.IMAGE_LINE,
            aim_x_ratio=c.aim_x_ratio, descent_line_y_ratio=c.descent_line_y_ratio,
            far_center_y_ratio=c.far_center_y_ratio, mission_timeout_s=c.mission_timeout_s,
            maximum_operation_depth_m=c.maximum_operation_depth_m))
        self.coverage = c
        self.points = semicircle_waypoints(c)
        self.point_index = 1
        self.x, self.y, _ = self.points[0]
        self.initial_heading = 0.0
        self.last_tick = None
        self.last_yaw = None
        self.last_speed = 0.0
        self.estimated_travel = 0.0
        self.local_start = None
        self.local_travel_start = 0.0
        self.local_attempts = 0
        self.suppressed_until_travel = 0.0
        self.turn_started = None
        self.route_started = 0.0
        self.grasp_target = None
        self.lifting_to_leave = False

    def start(self, observation, *, descent_command, now):
        self._validate_observation(observation, require_attitude=True)
        if self.state != State.IDLE:
            raise ClusterCollectionError('半圆策略实例不能重复启动')
        if not math.isfinite(now):
            raise ClusterCollectionError('启动时间无效')
        if observation.depth_m >= self.coverage.maximum_operation_depth_m:
            raise ClusterCollectionError('启动深度超过限制')
        if observation.depth_m > self.coverage.observation_depth_m:
            raise ClusterCollectionError('起始深度已深于确认的观察深度')
        self.start_depth_m = observation.depth_m
        self.initial_heading = observation.yaw_deg
        self.last_yaw = observation.yaw_deg
        self.last_tick = now
        self.last_frame_id = observation.frame_id
        self.last_new_frame_at = now
        self.mission_started_at = now
        self.outcome = 'running'
        self._enter(State.INITIAL_DESCENT, observation, now)
        return self._decision(MotionCommand.neutral(), observation, '沿初始航向建立推算坐标；下降至人工确认的观察压力深度')

    def _decision(self, motion, observation, message, *, gripper_action=None):
        if motion.forward:
            if self.state in (State.ROUTE_SEARCH, State.LANE_SHIFT):
                self.last_speed = self.coverage.search_speed_mps
            elif self.tracked_cluster and self.tracked_cluster.center_y_ratio < self.config.far_center_y_ratio:
                self.last_speed = self.coverage.far_speed_mps
            else:
                self.last_speed = self.coverage.near_speed_mps
        else:
            self.last_speed = 0.0
        return super()._decision(motion, observation, message, gripper_action=gripper_action)

    def delay_timers(self, duration_s):
        super().delay_timers(duration_s)
        # 暂停不累计里程；局部停留/总任务使用墙钟时间，防止无限停留。
        if self.last_tick is not None:
            self.last_tick += duration_s
        self.last_speed = 0.0
        self.last_yaw = None
        self.route_started += duration_s
        if self.turn_started is not None:
            self.turn_started += duration_s

    def snapshot(self):
        return dict(position_source='command_time_dead_reckoning', x_estimated_m=self.x,
            y_estimated_m=self.y, waypoint_index=self.point_index,
            lane_index=self.points[min(self.point_index, len(self.points)-1)][2],
            travel_estimated_m=self.estimated_travel, local_attempts=self.local_attempts,
            state=self.state.value, selected_target=(None if self.tracked_cluster is None else
                dict(label=self.tracked_cluster.detections[0].label,
                     center_x_ratio=self.tracked_cluster.center_x_ratio,
                     center_y_ratio=self.tracked_cluster.center_y_ratio)))

    def step(self, observation, now):
        c = self.coverage
        if self.state in (State.IDLE, State.COMPLETE, State.ABORTED):
            return self._decision(MotionCommand.neutral(), observation, self.message)
        try:
            self._validate_observation(observation, require_attitude=True)
        except ClusterCollectionError as exc:
            return self.abort(str(exc), observation)
        dt = now - self.last_tick
        if not math.isfinite(now) or dt < 0 or dt > c.max_tick_gap_s:
            return self.abort('控制周期中断或时间倒退，推算不再可信', observation)
        if self.last_yaw is not None and abs(signed_yaw_delta_deg(self.last_yaw, observation.yaw_deg)) > c.max_heading_step_deg:
            return self.abort('航向跳变', observation)
        angle = math.radians(signed_yaw_delta_deg(self.initial_heading, observation.yaw_deg))
        distance = self.last_speed * dt
        self.x -= math.sin(angle) * distance
        self.y += math.cos(angle) * distance
        self.estimated_travel += distance
        self.last_tick, self.last_yaw = now, observation.yaw_deg
        if observation.depth_m >= c.maximum_operation_depth_m:
            return self.abort('达到作业深度硬限制', observation)
        if self.state == State.RETURNING:
            return self._step_returning(observation, now)
        if now - self.mission_started_at >= c.mission_timeout_s or self.estimated_travel >= c.max_estimated_travel_m:
            return self.request_normal_finish(observation, now, '总时间或推算路程达到上限')
        radius = c.diameter_m / 2
        if self.x < 0 or math.hypot(self.x, self.y-radius) > radius:
            return self.request_normal_finish(observation, now, '推算位置越过计划半圆；真实位置未知')
        if observation.frame_id < self.last_frame_id:
            return self.abort('图像帧号倒退', observation)
        new_frame = observation.frame_id > self.last_frame_id
        if new_frame:
            self.last_frame_id = observation.frame_id
            self.last_new_frame_at = now
        age = now - self.last_new_frame_at
        if age >= self.config.perception_abort_timeout_s:
            return self.abort('检测流超时', observation)
        if not observation.perception_valid or age >= self.config.perception_hold_timeout_s:
            return self._decision(MotionCommand.neutral(), observation, '检测流不新鲜，等待')
        if self.state == State.INITIAL_DESCENT:
            return self._depth_step(observation, now, c.observation_depth_m, State.ROUTE_SEARCH)
        if self.state == State.GRASP_DESCENDING:
            return self._depth_step(observation, now, self.grasp_target, State.CLOSING_GRIPPER)
        if self.state == State.LIFTING_AFTER_GRASP:
            return self._depth_step(observation, now, c.observation_depth_m,
                State.ROUTE_SEARCH if self.lifting_to_leave else State.TRANSFER_TO_NET)
        if self.state == State.GRASP_HOLDING:
            if now-self.state_started_at >= self.config.gripper_motion_wait_s:
                self.lifting_to_leave = False
                self._enter(State.LIFTING_AFTER_GRASP, observation, now)
            return self._decision(MotionCommand.neutral(), observation, '闭爪等待后回到观察深度')
        if self.state == State.TRANSFER_TO_NET:
            if now-self.state_started_at >= c.transfer_timeout_s:
                return self.request_normal_finish(observation, now, '转运确认超时，保持闭爪结束')
            return self._decision(MotionCommand.neutral(), observation, '等待人工完成入网兜；确认可开爪后按 T')
        if self.awaiting_gripper_action is not None:
            return self._decision(MotionCommand.neutral(), observation, '等待机械爪命令反馈')
        if self.local_start is not None and (
            now-self.local_start >= c.local_timeout_s or
            self.local_attempts >= c.local_attempt_limit or
            self.estimated_travel-self.local_travel_start >= c.max_local_travel_m
        ):
            return self._leave(observation, now)
        if self.state in (State.ROUTE_SEARCH, State.LANE_SHIFT):
            if now-self.route_started >= c.segment_timeout_s:
                return self.request_normal_finish(observation, now, '搜索段或候选确认超时')
            # 仅在搜索带上发现目标，换行连接段不偏离路线抓取。
            if self.state == State.ROUTE_SEARCH and new_frame and self.estimated_travel >= self.suppressed_until_travel:
                found = self._acquire_single(observation, now)
                if found is not None:
                    return found
            if self.candidate_history:
                return self._decision(MotionCommand.neutral(), observation, '停车确认候选目标，等待新帧')
            return self._route_step(observation, now)
        if self.state == State.LOCAL_REOBSERVE:
            if new_frame:
                found = self._acquire_single(observation, now)
                if found is not None:
                    return found
            if now-self.state_started_at >= c.reobserve_timeout_s:
                return self._leave(observation, now)
            return self._decision(MotionCommand.neutral(), observation, '重新观察并选择单个扇贝')
        if self.state in (State.ALIGNING, State.APPROACHING, State.VERIFYING_LOSS):
            if not new_frame:
                return self._decision(MotionCommand.neutral(), observation, '等待新帧，重复帧不累计确认')
            match = self._single(observation, tracking=True)
            if match is None:
                self.consecutive_misses += 1
                if self.consecutive_misses >= self.config.loss_required_frames and now-self.last_target_seen_at >= self.config.loss_confirmation_s:
                    self._clear_cluster_lock()
                    self._enter(State.LOCAL_REOBSERVE, observation, now)
                else:
                    self._enter(State.VERIFYING_LOSS, observation, now)
                return self._decision(MotionCommand.neutral(), observation, '目标漏检，停车重新观察')
            self.tracked_cluster = match
            self.smoothed_center = (match.center_x_ratio, match.center_y_ratio)
            self.last_target_seen_at = now
            self.consecutive_misses = 0
            if self.state == State.VERIFYING_LOSS:
                self._enter(State.ALIGNING, observation, now)
            if self.state == State.ALIGNING:
                return self._step_aligning(observation, now)
            decision = self._step_approaching(observation, now)
            if decision.gripper_action == 'open':
                self.local_attempts += 1
                self.current_grasp_attempt_index = self.local_attempts
                decision = replace(decision, grasp_attempt_index=self.local_attempts)
            return decision
        return self.abort('未知半圆策略状态', observation)

    def _single(self, observation, tracking=False):
        if observation.frame_width <= 0 or observation.frame_height <= 0:
            return None
        candidates = []
        for detection in observation.detections:
            if detection.label != self.config.target_label:
                continue
            x, y = detection.box.center()
            x, y = x/observation.frame_width, y/observation.frame_height
            area = detection.box.area_ratio(observation.frame_width, observation.frame_height)
            if not all(math.isfinite(v) for v in (x, y, area, detection.confidence)) or not (0 <= x <= 1 and 0 <= y <= 1 and area > 0):
                continue
            candidates.append(ScallopCluster((detection,), x, y, area, detection.confidence))
        self.last_visible_scallop_count = len(candidates)
        if not candidates:
            return None
        reference = self.smoothed_center if tracking else self.candidate_center
        if reference is not None:
            chosen = min(candidates, key=lambda v: math.hypot(v.center_x_ratio-reference[0], v.center_y_ratio-reference[1]))
            if math.hypot(chosen.center_x_ratio-reference[0], chosen.center_y_ratio-reference[1]) <= self.config.maximum_tracking_jump_ratio:
                return chosen
            if tracking:
                return None
        return min(candidates, key=lambda v: (abs(v.center_x_ratio-self.config.aim_x_ratio), -v.total_confidence))

    def _acquire_single(self, observation, now):
        match = self._single(observation)
        if match is None:
            self.candidate_history.clear()
            self.candidate_center = None
            return None
        center = (match.center_x_ratio, match.center_y_ratio)
        if self.candidate_center and math.dist(center, self.candidate_center) > self.config.maximum_tracking_jump_ratio:
            self.candidate_history.clear()
        self.candidate_center = center
        self.candidate_history.append(True)
        if len(self.candidate_history) < self.config.acquisition_required_hits:
            return None
        self.tracked_cluster = match
        self.smoothed_center = center
        self.last_target_seen_at = now
        if self.local_start is None:
            self.local_start = now
            self.local_travel_start = self.estimated_travel
            self.local_attempts = 0
        self._enter(State.ALIGNING, observation, now)
        return self._decision(MotionCommand.neutral(), observation, '已确认单个扇贝，暂停路线并开始局部抓取')

    def _route_step(self, observation, now):
        c = self.coverage
        if self.point_index >= len(self.points):
            return self.request_normal_finish(observation, now, '计划航点已按推算完成，不代表实际完整覆盖')
        x, y, lane = self.points[self.point_index]
        dx, dy = x-self.x, y-self.y
        if math.hypot(dx, dy) <= c.waypoint_tolerance_m:
            self.point_index += 1
            self.route_started = now
            self.turn_started = None
            self._clear_cluster_lock()
            self._enter(State.LANE_SHIFT if self.point_index % 2 == 0 else State.ROUTE_SEARCH, observation, now)
            return self._decision(MotionCommand.neutral(), observation, '到达推算航点，准备下一段')
        if now-self.route_started >= c.segment_timeout_s:
            return self.request_normal_finish(observation, now, '搜索段超时')
        heading = (self.initial_heading + math.degrees(math.atan2(-dx, dy))) % 360
        error = signed_yaw_delta_deg(observation.yaw_deg, heading)
        if abs(error) > c.heading_tolerance_deg:
            self.turn_started = now if self.turn_started is None else self.turn_started
            if now-self.turn_started >= c.turn_timeout_s:
                return self.abort('航向对准超时', observation)
            yaw = math.copysign(min(self.config.maximum_yaw_command,
                max(self.config.minimum_yaw_command, abs(error)*c.heading_gain)), error)
            motion = MotionCommand(yaw=yaw)
        else:
            self.turn_started = None
            motion = MotionCommand(forward=c.search_command, yaw=max(-self.config.maximum_yaw_command,
                min(self.config.maximum_yaw_command, error*c.heading_gain)))
        self.search_cycle = lane
        return self._decision(motion, observation,
            f'ESTIMATED lane={lane+1} waypoint={self.point_index} x={self.x:.2f} y={self.y:.2f} remaining={math.hypot(dx,dy):.2f}m')

    def _depth_step(self, observation, now, target, next_state):
        c = self.coverage
        if now-self.state_started_at >= c.descent_timeout_s:
            return self.abort('有限深度动作超时', observation)
        self.target_depth_m = target
        error = target-observation.depth_m
        if abs(error) <= c.depth_tolerance_m:
            self.settled_since = now if self.settled_since is None else self.settled_since
            if now-self.settled_since >= c.depth_settle_s:
                if next_state == State.CLOSING_GRIPPER:
                    self._request_gripper(next_state, 'close', observation, now)
                    return self._decision(MotionCommand.neutral(), observation, '有限下降完成，请求闭爪', gripper_action='close')
                self._enter(next_state, observation, now)
                if next_state == State.ROUTE_SEARCH:
                    self.route_started = now
            return self._decision(MotionCommand.neutral(), observation, '达到压力深度，等待稳定；不代表离底测距')
        self.settled_since = None
        return self._decision(MotionCommand(vertical=-c.descent_command if error > 0 else c.lift_command), observation, '按确认的压力深度执行有限垂直动作')

    def _leave(self, observation, now):
        self.completed_cluster_count += 1
        self.local_start = None
        self._clear_cluster_lock()
        self.suppressed_until_travel = self.estimated_travel + self.coverage.revisit_suppression_m
        self.lifting_to_leave = True
        self._enter(State.LIFTING_AFTER_GRASP, observation, now)
        return self._decision(MotionCommand.neutral(), observation, '本观察点结束；恢复观察深度后重新接近当前计划航点，短程内不再锁定目标')

    def acknowledge_gripper(self, action, accepted, message, observation, now):
        if action != self.awaiting_gripper_action:
            return self.abort('机械爪反馈与请求不匹配', observation)
        self.awaiting_gripper_action = None
        self.last_gripper_accepted = accepted
        self.last_speed = 0.0
        # 服务等待期间运行层保持回中，不能计入运动或判为循环失联。
        self.last_tick = now
        if not accepted:
            return self.request_gripper_failure_recovery(observation, now, message)
        if action == 'open':
            self.grasp_target = min(observation.depth_m, self.coverage.observation_depth_m) + self.coverage.grasp_descent_m
            self._enter(State.GRASP_DESCENDING, observation, now)
        elif action == 'close':
            self.total_grasp_attempt_count += 1
            self._enter(State.GRASP_HOLDING, observation, now)
        elif action == 'reopen':
            self._clear_cluster_lock()
            self._enter(State.LOCAL_REOBSERVE, observation, now)
        else:
            return self.abort('未知机械爪动作', observation)
        return self._decision(MotionCommand.neutral(), observation, '已收到机械爪反馈；不代表实物抓取成功')

    def confirm_transfer(self, observation, now):
        if self.state != State.TRANSFER_TO_NET:
            return None
        self._request_gripper(State.REOPENING_GRIPPER, 'reopen', observation, now)
        return self._decision(MotionCommand.neutral(), observation, '操作员确认转运完成并允许开爪', gripper_action='reopen')
