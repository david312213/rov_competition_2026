"""YOLO26-x 感知适配器与比赛画面标注。

模块采用延迟导入，因此在未安装 Ultralytics/OpenCV 的电脑上仍可导入并测试任务
状态机。真正开始推理时会明确检查模型哈希、类别顺序和依赖，而不是悄悄换用
错误模型。
"""

from __future__ import annotations

import hashlib
import math
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from .config import DetectorConfig
from .domain import BoundingBox, Detection


class DetectorError(RuntimeError):
    """模型不存在、依赖缺失、类别不符或推理失败。"""


def sha256_file(path: str | Path, chunk_size: int = 1024 * 1024) -> str:
    """流式计算文件 SHA-256，避免把大模型一次读入内存。"""

    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


class YoloDetector:
    """把 Ultralytics YOLO 输出转换成工程统一的 :class:`Detection`。"""

    def __init__(self, config: DetectorConfig) -> None:
        """校验并加载模型。

        ``.pt`` 本质上是 Python/PyTorch 序列化文件，只应加载来源可信且哈希已经
        核对的权重。本工程记录了用户提供模型的 SHA-256，并拒绝哈希不一致文件。
        """

        self.config = config
        if not config.model_path.is_file():
            raise DetectorError(f"模型文件不存在: {config.model_path}")
        if config.expected_sha256:
            actual_hash = sha256_file(config.model_path)
            if actual_hash.lower() != config.expected_sha256.lower():
                raise DetectorError(
                    "模型 SHA-256 不一致，拒绝加载；"
                    f"期望 {config.expected_sha256}，实际 {actual_hash}"
                )

        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise DetectorError(
                "缺少 ultralytics；请在 Ubuntu 22.04 目标环境执行安装脚本"
            ) from exc

        try:
            self._model = YOLO(str(config.model_path))
        except Exception as exc:  # 推理框架会抛出多种后端异常，统一增加模型上下文。
            raise DetectorError(f"YOLO 模型加载失败: {exc}") from exc

        model_names = self._normalise_names(getattr(self._model, "names", {}))
        expected_names = tuple(config.class_names)
        if model_names != expected_names:
            raise DetectorError(
                "模型类别与配置不一致；"
                f"模型={model_names}，配置={expected_names}"
            )
        self.names = model_names

    def detect(self, frame: Any) -> list[Detection]:
        """对一帧 BGR 图像执行推理。

        Returns:
            置信度不低于配置阈值的目标列表。YOLO 自身负责缩放、NMS 和坐标还原。
        """

        if frame is None or getattr(frame, "size", 0) == 0:
            raise DetectorError("输入图像为空")
        device = None if self.config.device == "auto" else self.config.device
        try:
            results = self._model.predict(
                source=frame,
                conf=self.config.confidence_threshold,
                iou=self.config.iou_threshold,
                imgsz=self.config.image_size,
                device=device,
                verbose=False,
            )
        except Exception as exc:
            raise DetectorError(f"YOLO 推理失败: {exc}") from exc

        detections: list[Detection] = []
        if not results:
            return detections
        boxes = getattr(results[0], "boxes", None)
        if boxes is None:
            return detections

        for box in boxes:
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())
            left, top, right, bottom = (float(value) for value in box.xyxy[0].tolist())
            values = (confidence, left, top, right, bottom)
            if not all(math.isfinite(value) for value in values):
                continue
            if right <= left or bottom <= top:
                continue
            if not 0 <= class_id < len(self.names):
                continue
            detections.append(
                Detection(
                    class_id=class_id,
                    label=self.names[class_id],
                    confidence=confidence,
                    box=BoundingBox(left=left, top=top, right=right, bottom=bottom),
                )
            )
        return detections

    @staticmethod
    def _normalise_names(names: Any) -> tuple[str, ...]:
        """把 Ultralytics 的字典/列表类别表转换为有序元组。"""

        if isinstance(names, dict):
            try:
                return tuple(str(names[index]) for index in range(len(names)))
            except KeyError as exc:
                raise DetectorError("模型类别编号必须从 0 连续递增") from exc
        if isinstance(names, (list, tuple)):
            return tuple(str(item) for item in names)
        raise DetectorError(f"无法识别模型类别表类型: {type(names).__name__}")


def draw_competition_overlay(
    frame: Any,
    detections: Iterable[Detection],
    *,
    mission_state: str,
    mission_message: str,
    fps: float,
    selected_target: Detection | None = None,
    aim_point: tuple[float, float] = (0.5, 0.7),
    target_area_ratio: float | None = None,
    grasp_area_threshold: float | None = None,
    current_depth_m: float | None = None,
    scan_progress_deg: float | None = None,
    simulation: bool = False,
) -> Any:
    """绘制裁判可见的识别框、置信度和自主任务状态。

    规则明确要求自主抓取必须在屏幕上体现目标识别效果。本函数始终保留原始第一
    视角背景，并将目标框和状态叠加到同一画面，便于 HDMI 镜像和比赛录像取证。
    """

    try:
        import cv2
    except ImportError as exc:
        raise DetectorError("缺少 opencv-python，无法绘制比赛画面") from exc

    output = frame.copy()
    detection_list = list(detections)
    locked_item: Detection | None = None
    if selected_target is not None:
        same_label = [
            item for item in detection_list if item.label == selected_target.label
        ]
        if same_label:
            old_x, old_y = selected_target.box.center()
            locked_item = max(
                same_label,
                key=lambda item: (
                    selected_target.box.intersection_over_union(item.box),
                    -math.hypot(
                        item.box.center()[0] - old_x,
                        item.box.center()[1] - old_y,
                    ),
                ),
            )
    for item in detection_list:
        left, top = int(item.box.left), int(item.box.top)
        right, bottom = int(item.box.right), int(item.box.bottom)
        locked = item is locked_item
        color = (0, 140, 255) if locked else (0, 220, 0)
        cv2.rectangle(output, (left, top), (right, bottom), color, 4 if locked else 2)
        prefix = "LOCKED " if locked else ""
        text = f"{prefix}{item.label} {item.confidence:.2f}"
        cv2.putText(
            output,
            text,
            (left, max(24, top - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )

    aim_x = int(max(0.0, min(1.0, aim_point[0])) * output.shape[1])
    aim_y = int(max(0.0, min(1.0, aim_point[1])) * output.shape[0])
    cv2.drawMarker(
        output,
        (aim_x, aim_y),
        (255, 255, 0),
        markerType=cv2.MARKER_CROSS,
        markerSize=30,
        thickness=2,
    )
    cv2.putText(
        output,
        "GRASP AIM",
        (min(output.shape[1] - 115, aim_x + 10), max(20, aim_y - 10)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 0),
        1,
        cv2.LINE_AA,
    )

    panel_height = min(output.shape[0], 108)
    cv2.rectangle(output, (0, 0), (output.shape[1], panel_height), (0, 0, 0), -1)
    cv2.putText(
        output,
        f"STATE: {mission_state}  FPS: {fps:.1f}",
        (12, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    # OpenCV 内置 Hershey 字体不支持中文。中文详情仍发布在 ROS 消息和日志中，
    # 比赛窗口使用稳定可读的 ASCII 状态，避免现场出现方框或乱码。
    detail_text = (
        mission_message
        if mission_message.isascii()
        else f"DETECTIONS: {len(detection_list)}  ACTION: {mission_state.upper()}"
    )
    cv2.putText(
        output,
        detail_text[:100],
        (12, 53),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (0, 230, 255),
        2,
        cv2.LINE_AA,
    )
    metrics: list[str] = []
    if target_area_ratio is not None:
        metrics.append(f"AREA={target_area_ratio:.3f}")
    if grasp_area_threshold is not None:
        metrics.append(f"K={grasp_area_threshold:.3f}")
    if current_depth_m is not None:
        metrics.append(f"DEPTH={current_depth_m:.2f}m")
    if scan_progress_deg is not None:
        metrics.append(f"SCAN={scan_progress_deg:.1f}deg")
    cv2.putText(
        output,
        "  ".join(metrics) if metrics else "WAITING FOR MISSION METRICS",
        (12, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.54,
        (255, 210, 80),
        2,
        cv2.LINE_AA,
    )
    if simulation:
        cv2.putText(
            output,
            "SIMULATION - NOT REAL-VEHICLE EVIDENCE",
            (12, 104),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
    return output
