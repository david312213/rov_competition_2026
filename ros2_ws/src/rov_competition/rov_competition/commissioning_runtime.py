"""ROS 实机测试工具的共用运行时安全封装。"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ControlStatus, NormalizedMotionCommand, RobotTelemetry
from rov_interfaces.srv import SetArmed

from .domain import MotionCommand

PUBLISH_RATE_HZ = 20.0
NEUTRAL_REPEAT_COUNT = 10


class CommissioningNode(Node):
    """发布单一测试来源，并缓存网关/遥测状态。"""

    def __init__(self, name: str, source: str) -> None:
        """创建发布器、订阅和正常上锁客户端。"""

        super().__init__(name)
        self.source = source
        self.telemetry: RobotTelemetry | None = None
        self.telemetry_received_at: float | None = None
        self.status: ControlStatus | None = None
        self.status_received_at: float | None = None
        self.publisher = self.create_publisher(
            NormalizedMotionCommand, "/rov/control/command", 10
        )
        self.create_subscription(RobotTelemetry, "/rov/telemetry", self._telemetry, 10)
        self.create_subscription(ControlStatus, "/rov/control/status", self._status, 10)
        self.disarm_client = self.create_client(SetArmed, "/rov/control/set_armed")

    def _telemetry(self, message: RobotTelemetry) -> None:
        self.telemetry = message
        self.telemetry_received_at = time.monotonic()

    def _status(self, message: ControlStatus) -> None:
        self.status = message
        self.status_received_at = time.monotonic()

    def message(self, motion: MotionCommand) -> NormalizedMotionCommand:
        """生成带当前 ROS 时间戳和固定来源的命令。"""

        message = NormalizedMotionCommand()
        message.stamp = self.get_clock().now().to_msg()
        message.source = self.source
        message.forward = float(motion.forward)
        message.lateral = float(motion.lateral)
        message.vertical = float(motion.vertical)
        message.yaw = float(motion.yaw)
        return message

    def publish(self, motion: MotionCommand) -> None:
        """发布一条已由上层校验的运动意图。"""

        self.publisher.publish(self.message(motion))

    def wait_until_ready(self, timeout_s: float = 5.0) -> str | None:
        """等待 DDS、遥测和网关，完全健康时返回空。"""

        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.publisher.get_subscription_count() > 0
                and self.telemetry is not None
                and self.status is not None
            ):
                break
        if self.publisher.get_subscription_count() == 0:
            return "未发现 /rov/control/command 订阅者"
        if self.telemetry is None or self.status is None:
            return "未收齐 /rov/telemetry 和 /rov/control/status"
        return self.runtime_error()

    def runtime_error(
        self,
        maximum_age_s: float | None = None,
        *,
        maximum_telemetry_age_s: float = 1.5,
        maximum_status_age_s: float = 3.0,
    ) -> str | None:
        """每个执行周期重新检查数据新鲜度和网关状态。

        ``maximum_age_s`` 仅用于兼容旧调用点；新默认值将遥测和
        控制状态分开，避免 DDS/桌面调度抖动误中止。这不会改变
        网关的 0.5s 运动发布者看门狗。
        """

        if maximum_age_s is not None:
            maximum_telemetry_age_s = float(maximum_age_s)
            maximum_status_age_s = float(maximum_age_s)

        now = time.monotonic()
        if self.telemetry is None or self.telemetry_received_at is None:
            return "遥测缺失"
        if self.status is None or self.status_received_at is None:
            return "控制状态缺失"
        if now - self.telemetry_received_at > maximum_telemetry_age_s:
            return "遥测数据过期"
        if now - self.status_received_at > maximum_status_age_s:
            return "控制状态过期"
        if not self.telemetry.valid_heartbeat or not self.telemetry.valid_attitude:
            return "心跳或姿态遥测无效"
        if not self.status.preflight_passed:
            return "飞控只读预检未通过"
        if not self.status.runtime_enabled:
            return "运行时控制许可未开启"
        if self.status.emergency_stop_latched or self.status.state == "ESTOPPED":
            return "急停已锁定"
        if not self.status.armed or not self.status.armed_by_ros:
            return "飞控未经 ROS 正常解锁"
        if self.status.state not in {"READY", "ACTIVE"}:
            return f"网关状态不允许测试: {self.status.state}"
        if self.status.command_source and self.status.command_source != self.source:
            return f"网关当前命令来源不是 {self.source!r}"
        return None

    def publish_neutral(self) -> None:
        """连续发布中位，覆盖 DDS 队列中的非中位命令。"""

        period_s = 1.0 / PUBLISH_RATE_HZ
        for _ in range(NEUTRAL_REPEAT_COUNT):
            if not rclpy.ok():
                break
            self.publish(MotionCommand.neutral())
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period_s)

    def request_normal_disarm(self, timeout_s: float = 3.0) -> str | None:
        """请求网关正常回中、上锁并释放控制，绝不强制绕过检查。"""

        if not self.disarm_client.wait_for_service(timeout_sec=0.5):
            return "正常上锁服务不可用"
        request = SetArmed.Request()
        request.arm = False
        request.confirmation = ""
        future = self.disarm_client.call_async(request)
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return "正常上锁服务响应超时"
        try:
            result = future.result()
        except Exception as exc:
            return f"正常上锁服务异常: {exc}"
        return None if result.success else str(result.message)
