"""与硬件、ROS 和推理框架无关的领域数据类型。

把这些基础类型放在独立模块中，可以让任务状态机在没有飞控、摄像头、ROS 2
和 GPU 的电脑上进行单元测试。这样，控制逻辑的正确性不会依赖比赛现场设备。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


def _clamp(value: float, minimum: float, maximum: float) -> float:
    """把有限数值限制在闭区间内，拒绝 NaN/Inf。"""

    if not math.isfinite(value):
        raise ValueError("运动指令必须是有限数值")
    return max(minimum, min(maximum, value))


@dataclass(frozen=True)
class BoundingBox:
    """一帧图像中的目标框，坐标单位为像素。"""

    left: float
    top: float
    right: float
    bottom: float

    def width(self) -> float:
        """返回非负目标框宽度。"""

        return max(0.0, self.right - self.left)

    def height(self) -> float:
        """返回非负目标框高度。"""

        return max(0.0, self.bottom - self.top)

    def center(self) -> tuple[float, float]:
        """返回目标框中心像素坐标 ``(x, y)``。"""

        return ((self.left + self.right) / 2.0, (self.top + self.bottom) / 2.0)

    def area_ratio(self, frame_width: int, frame_height: int) -> float:
        """返回目标框占整帧面积的比例，非法帧尺寸返回 0。"""

        if frame_width <= 0 or frame_height <= 0:
            return 0.0
        return (self.width() * self.height()) / float(frame_width * frame_height)

    def intersection_over_union(self, other: BoundingBox) -> float:
        """返回与另一个框的 IoU，用于避免同类目标在连续帧中互相顶替。"""

        left = max(self.left, other.left)
        top = max(self.top, other.top)
        right = min(self.right, other.right)
        bottom = min(self.bottom, other.bottom)
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        union = self.width() * self.height() + other.width() * other.height() - intersection
        return 0.0 if union <= 0.0 else intersection / union


@dataclass(frozen=True)
class Detection:
    """一次目标检测结果。"""

    class_id: int
    label: str
    confidence: float
    box: BoundingBox


@dataclass(frozen=True)
class MotionCommand:
    """与具体 PWM 无关的四自由度运动指令。

    每个轴都使用 ``[-1, 1]`` 的归一化范围。方向正负最终由机器人配置文件中的
    ``direction`` 决定，状态机不直接写死通道号或 PWM 数值。
    """

    forward: float = 0.0
    lateral: float = 0.0
    vertical: float = 0.0
    yaw: float = 0.0

    def limited(self, maximum: float = 1.0) -> MotionCommand:
        """返回每个轴均限制到 ``[-maximum, maximum]`` 的安全副本。

        ``maximum`` 是运动意图的统一软件限幅，不是某个电机的输出比例。八路
        推进器的混控仍由飞控完成。
        """

        if not math.isfinite(maximum) or not 0.0 < maximum <= 1.0:
            raise ValueError("运动指令限幅必须位于 (0, 1]")

        return MotionCommand(
            forward=_clamp(self.forward, -maximum, maximum),
            lateral=_clamp(self.lateral, -maximum, maximum),
            vertical=_clamp(self.vertical, -maximum, maximum),
            yaw=_clamp(self.yaw, -maximum, maximum),
        )

    def is_neutral(self, tolerance: float = 1e-6) -> bool:
        """四个轴都接近零时返回 ``True``。"""

        return all(
            abs(value) <= tolerance
            for value in (self.forward, self.lateral, self.vertical, self.yaw)
        )

    @classmethod
    def neutral(cls) -> MotionCommand:
        """返回四个运动轴全部为零的停止指令。"""

        return cls()


class ControlState(str, Enum):
    """真实控制网关的运行时安全状态。"""

    LOCKED = "LOCKED"
    READY = "READY"
    ACTIVE = "ACTIVE"
    ESTOPPED = "ESTOPPED"


class GripperAction(str, Enum):
    """机械爪动作；``NONE`` 表示本周期不改变舵机状态。"""

    NONE = "none"
    OPEN = "open"
    CLOSE = "close"
    RAISE = "raise"
    RESET = "reset"


class MissionState(str, Enum):
    """自主抓取状态机的所有状态。"""

    IDLE = "idle"
    PREPARING = "preparing"
    DESCENDING = "descending"
    SCANNING = "scanning"
    ADVANCING = "advancing"
    ALIGNING = "aligning"
    APPROACHING = "approaching"
    REACQUIRING = "reacquiring"
    GRASPING = "grasping"
    ASCENDING = "ascending"
    COMPLETE = "complete"
    ABORTED = "aborted"


class MissionOutcome(str, Enum):
    """任务终局；``GRASP_COMMANDED`` 不冒充机械爪物理抓取反馈。"""

    NONE = "none"
    GRASP_COMMANDED = "grasp_commanded"
    NO_TARGET = "no_target"
    ABORTED = "aborted"


@dataclass(frozen=True)
class MissionObservation:
    """状态机一次控制周期可使用的全部观测。

    ``frame_id`` 必须只在取得一张新图像时递增。状态机据此保证连续三帧、
    连续五帧等判断不会把同一张图在 20 Hz 控制循环里重复计数。
    """

    frame_id: int
    detections: Sequence[Detection]
    frame_width: int
    frame_height: int
    perception_valid: bool
    depth_valid: bool
    depth_m: float
    attitude_valid: bool
    yaw_deg: float


@dataclass(frozen=True)
class MissionDecision:
    """状态机在一帧图像处理后给执行层的完整决定。"""

    state: MissionState
    motion: MotionCommand
    gripper: GripperAction = GripperAction.NONE
    selected_target: Detection | None = None
    message: str = ""
    grasp_attempts: int = 0
    outcome: MissionOutcome = MissionOutcome.NONE
    target_area_ratio: float | None = None
    grasp_area_threshold: float | None = None
    horizontal_error: float | None = None
    vertical_error: float | None = None
    current_depth_m: float | None = None
    target_depth_m: float | None = None
    scan_progress_deg: float | None = None
    estimated_advance_distance_m: float | None = None
    search_cycle: int = 0
    gripper_command_accepted: bool = False


@dataclass
class TelemetrySnapshot:
    """由真实 MAVLink 消息逐步更新的机器人状态快照。

    字段使用 ``None`` 表示飞控尚未提供该项数据。发布端不得用随机数或固定数值
    冒充实时数据，这一点直接对应规则中的 ROS 数据真实性要求。
    """

    depth_m: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    battery_voltage_v: float | None = None
    battery_current_a: float | None = None
    battery_remaining_pct: int | None = None
    armed: bool | None = None
    flight_mode: str | None = None
    system_status: int | None = None
    autopilot_type: int | None = None
    vehicle_type: int | None = None
    firmware_version: str | None = None
    board_version: int | None = None
    vendor_id: int | None = None
    product_id: int | None = None
    last_status_text: str | None = None
    last_message_monotonic: float | None = None
    last_heartbeat_monotonic: float | None = None
    depth_updated_monotonic: float | None = None
    attitude_updated_monotonic: float | None = None
    battery_updated_monotonic: float | None = None
