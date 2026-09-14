"""独立的只读群体画面进程；没有控制发布者、解锁服务或 MAVLink 连接。

JPEG 解码/叠加/压缩和图像 DDS 发送不能占用运动控制循环。
显示只取最新帧；画面拥塞不得靠重发旧运动指令来掩盖。
"""

from __future__ import annotations

import argparse
from array import array
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rov_interfaces.msg import MissionStatus
from sensor_msgs.msg import CompressedImage


def render_overlay(jpeg_data: object, status: object | None, line_ratio: float) -> bytes | None:
    import cv2
    import numpy as np

    frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        return None
    height, width = frame.shape[:2]
    line_y = int(line_ratio * height)
    cv2.line(frame, (0, line_y), (width - 1, line_y), (255, 0, 255), 2)
    cv2.putText(frame, "CLUSTER DESCENT LINE", (12, max(22, line_y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 255), 2, cv2.LINE_AA)
    if status is not None and status.has_cluster:
        center = (int(status.cluster_center_x_ratio * width),
                  int(status.cluster_center_y_ratio * height))
        cv2.drawMarker(frame, center, (0, 0, 255), cv2.MARKER_CROSS, 36, 3)
    if status is not None and getattr(status, "has_selected_target", False):
        left, top = int(status.selected_left * width), int(status.selected_top * height)
        right, bottom = int(status.selected_right * width), int(status.selected_bottom * height)
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 255), 3)
        cv2.putText(frame, f"LOCK {status.selected_confidence:.2f} bottom={status.selected_bottom:.3f} "
                    f"vertical_only={status.descent_vertical_only}", (12, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    text = "WAITING FOR FRESH CLUSTER STATUS"
    if status is not None:
        text = (f"CLUSTER state={status.state} visible={status.visible_scallop_count} "
                f"locked={status.locked_cluster_count} grasp={status.grasp_attempt_index}/3 "
                f"groups={status.completed_cluster_count}")
    cv2.putText(frame, text, (12, height - 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.58, (255, 255, 255), 2, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return encoded.tobytes() if ok else None


class ClusterOverlayNode(Node):
    def __init__(self, *, line_ratio: float, parent_pid: int) -> None:
        super().__init__("rov_cluster_overlay")
        self.line_ratio = line_ratio
        self.parent_pid = parent_pid
        self.latest_image = None
        self.latest_status = None
        self.status_at = float("-inf")
        self.last_warning_at = float("-inf")
        # Best-effort depth=1 prevents a slow viewer from backpressuring this process.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(CompressedImage, "/rov/cluster_image/compressed", 1)
        self.create_subscription(CompressedImage, "/rov/annotated_image/compressed", self._image, qos)
        self.create_subscription(MissionStatus, "/rov/mission/status", self._status, qos)
        self.create_timer(0.10, self._render)

    def _image(self, message: CompressedImage) -> None:
        self.latest_image = message

    def _status(self, message: MissionStatus) -> None:
        self.latest_status = message
        self.status_at = time.monotonic()

    def _render(self) -> None:
        # The owner may be killed without running finally. Never remain orphaned.
        if self.parent_pid and os.getppid() != self.parent_pid:
            rclpy.shutdown()
            return
        image = self.latest_image
        if image is None:
            return
        self.latest_image = None
        status = self.latest_status if time.monotonic() - self.status_at <= 0.5 else None
        try:
            data = render_overlay(image.data, status, self.line_ratio)
            if data is None:
                return
            message = CompressedImage()
            message.header = image.header  # preserve capture stamp, not rendering time
            message.format = "jpeg"
            message.data = array("B", data)
            self.publisher.publish(message)
        except Exception as exc:
            if time.monotonic() - self.last_warning_at >= 2.0:
                self.last_warning_at = time.monotonic()
                self.get_logger().warning(f"群体画面发布失败（不影响控制）: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--line-ratio", type=float, required=True)
    parser.add_argument("--parent-pid", type=int, default=0)
    args = parser.parse_args()
    if not 0.0 <= args.line_ratio <= 1.0:
        parser.error("line-ratio must be in [0, 1]")
    import cv2
    cv2.setNumThreads(1)
    rclpy.init()
    node = ClusterOverlayNode(line_ratio=args.line_ratio, parent_pid=args.parent_pid)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
