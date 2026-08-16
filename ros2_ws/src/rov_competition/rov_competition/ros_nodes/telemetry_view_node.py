"""ROS 2 节点：将真实遥测以易于裁判核验的窗口显示。"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rov_interfaces.msg import RobotTelemetry


def _value(valid: bool, number: float, unit: str = "") -> str:
    """格式化有效数值；无效数据明确显示 ``N/A``。"""

    if not valid or not math.isfinite(float(number)):
        return "N/A"
    return f"{number:.2f}{unit}"


class TelemetryViewNode(Node):
    """订阅组合遥测并生成黑底高对比度 ROS 数据面板。"""

    def __init__(self) -> None:
        """创建订阅与 10 Hz 显示定时器。"""

        super().__init__("rov_telemetry_view")
        self._latest: RobotTelemetry | None = None
        self._subscription = self.create_subscription(
            RobotTelemetry, "/rov/telemetry", self._handle_telemetry, 10
        )
        self._timer = self.create_timer(0.1, self._draw)

    def _handle_telemetry(self, message: RobotTelemetry) -> None:
        """缓存最近一条真实遥测。"""

        self._latest = message

    def _draw(self) -> None:
        """绘制状态面板；窗口由操作主屏幕一起 HDMI 镜像。"""

        import cv2
        import numpy as np

        canvas = np.zeros((620, 960, 3), dtype=np.uint8)
        cv2.putText(
            canvas,
            "ROV REAL-TIME ROS TELEMETRY",
            (35, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 230, 255),
            2,
            cv2.LINE_AA,
        )
        if self._latest is None:
            rows = [("STATUS", "WAITING FOR /rov/telemetry")]
        else:
            message = self._latest
            rows = [
                ("DEPTH", _value(message.valid_depth, message.depth_m, " m")),
                ("ROLL", _value(message.valid_attitude, message.roll_deg, " deg")),
                ("PITCH", _value(message.valid_attitude, message.pitch_deg, " deg")),
                ("YAW", _value(message.valid_attitude, message.yaw_deg, " deg")),
                (
                    "BATTERY",
                    _value(message.valid_battery_voltage, message.battery_voltage_v, " V"),
                ),
                (
                    "CURRENT",
                    _value(message.valid_battery_current, message.battery_current_a, " A"),
                ),
                (
                    "REMAINING",
                    "N/A"
                    if not message.valid_battery_remaining
                    else f"{message.battery_remaining_pct:d} %",
                ),
                ("ARMED", "YES" if message.valid_heartbeat and message.armed else "NO"),
                ("MODE", message.flight_mode if message.valid_heartbeat else "N/A"),
                ("HEARTBEAT AGE", f"{message.heartbeat_age_s:.2f} s"),
                ("MESSAGE AGE", f"{message.message_age_s:.2f} s"),
            ]
        for index, (name, value) in enumerate(rows):
            y = 105 + index * 43
            cv2.putText(
                canvas,
                f"{name:<16} {value}",
                (45, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.72,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        cv2.imshow("ROV ROS telemetry", canvas)
        cv2.waitKey(1)

    def destroy_node(self) -> bool:
        """销毁节点时关闭 OpenCV 窗口。"""

        import cv2

        cv2.destroyAllWindows()
        return super().destroy_node()


def main() -> None:
    """启动遥测展示窗口。"""

    rclpy.init()
    node = TelemetryViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
