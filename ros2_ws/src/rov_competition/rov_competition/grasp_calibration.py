"""抓取位置标定的数据计算、证据保存和参数建议。

本模块不导入 ROS、OpenCV、YOLO 或 MAVLink。现场节点只负责把检测框、
遥测和截图交给这里；全部统计逻辑可以在没有实艇的电脑上单元测试。
"""

from __future__ import annotations

import csv
import math
import re
import statistics
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Sequence

import yaml


class GraspCalibrationError(RuntimeError):
    """检测框、样本状态或输出文件不满足标定要求。"""


@dataclass(frozen=True)
class BoxMetrics:
    """一个目标框在原始画面中的像素值和归一化几何量。"""

    class_id: int
    label: str
    confidence: float
    frame_width: int
    frame_height: int
    left: float
    top: float
    right: float
    bottom: float
    center_x_ratio: float
    center_y_ratio: float
    width_ratio: float
    height_ratio: float
    area_ratio: float


@dataclass(frozen=True)
class CalibrationTelemetry:
    """记录样本时可用的飞控状态；无效字段保持 ``None``。"""

    depth_m: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    yaw_deg: float | None = None
    flight_mode: str = ""
    armed: bool | None = None


@dataclass(frozen=True)
class CalibrationSample:
    """一次闭爪前姿态以及随后由操作员确认的物理结果。"""

    sample_id: int
    utc_time: str
    detection_stamp_s: float
    image_stamp_s: float
    image_time_delta_s: float
    target: BoxMetrics
    telemetry: CalibrationTelemetry
    screenshot_file: str
    frame_id: int = 0
    manual_power: float = 0.0
    command_forward: float = 0.0
    command_lateral: float = 0.0
    command_vertical: float = 0.0
    command_yaw: float = 0.0
    gripper_profile: str = ""
    gripper_action: str = "not_sent"
    gripper_command_accepted: bool | None = None
    gripper_ack: str = ""
    outcome: str = "pending"
    note: str = ""


VALID_FINAL_OUTCOMES = {"success", "failure", "bad_pose", "discarded"}
CSV_FIELDS = (
    "sample_id",
    "utc_time",
    "outcome",
    "note",
    "frame_id",
    "label",
    "class_id",
    "confidence",
    "frame_width",
    "frame_height",
    "left",
    "top",
    "right",
    "bottom",
    "center_x_ratio",
    "center_y_ratio",
    "width_ratio",
    "height_ratio",
    "area_ratio",
    "depth_m",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "flight_mode",
    "armed",
    "detection_stamp_s",
    "image_stamp_s",
    "image_time_delta_s",
    "screenshot_file",
    "manual_power",
    "command_forward",
    "command_lateral",
    "command_vertical",
    "command_yaw",
    "gripper_profile",
    "gripper_action",
    "gripper_command_accepted",
    "gripper_ack",
)


def calculate_box_metrics(
    *,
    class_id: int,
    label: str,
    confidence: float,
    frame_width: int,
    frame_height: int,
    left: float,
    top: float,
    right: float,
    bottom: float,
) -> BoxMetrics:
    """校验检测框并计算中心、宽高和面积占整帧的比例。"""

    width = int(frame_width)
    height = int(frame_height)
    if width <= 0 or height <= 0:
        raise GraspCalibrationError("检测画面宽高必须为正数")
    clean_label = str(label).strip()
    if not clean_label:
        raise GraspCalibrationError("检测类别不能为空")
    numeric = tuple(float(value) for value in (confidence, left, top, right, bottom))
    if not all(math.isfinite(value) for value in numeric):
        raise GraspCalibrationError("检测框包含 NaN/Inf")
    score, raw_left, raw_top, raw_right, raw_bottom = numeric
    if not 0.0 <= score <= 1.0:
        raise GraspCalibrationError("检测置信度必须位于 [0, 1]")

    clipped_left = min(float(width), max(0.0, raw_left))
    clipped_top = min(float(height), max(0.0, raw_top))
    clipped_right = min(float(width), max(0.0, raw_right))
    clipped_bottom = min(float(height), max(0.0, raw_bottom))
    box_width = clipped_right - clipped_left
    box_height = clipped_bottom - clipped_top
    if box_width <= 0.0 or box_height <= 0.0:
        raise GraspCalibrationError("检测框裁剪后没有有效面积")

    return BoxMetrics(
        class_id=int(class_id),
        label=clean_label,
        confidence=score,
        frame_width=width,
        frame_height=height,
        left=clipped_left,
        top=clipped_top,
        right=clipped_right,
        bottom=clipped_bottom,
        center_x_ratio=(clipped_left + clipped_right) / (2.0 * width),
        center_y_ratio=(clipped_top + clipped_bottom) / (2.0 * height),
        width_ratio=box_width / width,
        height_ratio=box_height / height,
        area_ratio=(box_width * box_height) / float(width * height),
    )


def ordered_calibration_targets(
    targets: Iterable[BoxMetrics], allowed_labels: Iterable[str]
) -> tuple[BoxMetrics, ...]:
    """只保留可抓类别，并按框面积、置信度从大到小排列。"""

    allowed = {str(label) for label in allowed_labels}
    return tuple(
        sorted(
            (target for target in targets if target.label in allowed),
            key=lambda target: (target.area_ratio, target.confidence),
            reverse=True,
        )
    )


def _percentile(values: Sequence[float], ratio: float) -> float:
    """使用线性插值计算小样本分位数。"""

    if not values:
        raise GraspCalibrationError("空样本不能计算分位数")
    if not 0.0 <= ratio <= 1.0:
        raise ValueError("分位数比例必须位于 [0, 1]")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * ratio
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _rounded(value: float | None, digits: int = 6) -> float | None:
    return None if value is None else round(float(value), digits)


def build_calibration_summary(
    samples: Sequence[CalibrationSample],
    target_labels: Sequence[str],
    *,
    minimum_success_samples: int = 5,
) -> dict[str, object]:
    """统计成功/失败样本并生成不会自动写回配置的参数建议。

    面积阈值使用成功样本的 25% 分位作为保守起点；瞄准点使用成功样本
    中位数。容差换算成正式状态机使用的“相对半幅画面误差”单位。
    """

    if minimum_success_samples < 3:
        raise ValueError("每类最少成功样本数不能小于 3")
    labels = tuple(dict.fromkeys(str(label) for label in target_labels))
    if not labels:
        raise ValueError("标定类别不能为空")

    profiles: dict[str, object] = {}
    sufficient_successes: list[CalibrationSample] = []
    warnings: list[str] = []
    suggested_areas: dict[str, float] = {}
    class_centers_x: list[float] = []
    class_centers_y: list[float] = []

    for label in labels:
        related = [sample for sample in samples if sample.target.label == label]
        successful = [sample for sample in related if sample.outcome == "success"]
        failures = [sample for sample in related if sample.outcome == "failure"]
        bad_poses = [sample for sample in related if sample.outcome == "bad_pose"]
        pending = [sample for sample in related if sample.outcome == "pending"]
        sufficient = len(successful) >= minimum_success_samples

        profile: dict[str, object] = {
            "total_samples": len(related),
            "success_count": len(successful),
            "failure_count": len(failures),
            "bad_pose_count": len(bad_poses),
            "pending_count": len(pending),
            "sufficient_for_suggestion": sufficient,
        }
        if successful:
            centers_x = [sample.target.center_x_ratio for sample in successful]
            centers_y = [sample.target.center_y_ratio for sample in successful]
            areas = [sample.target.area_ratio for sample in successful]
            median_x = statistics.median(centers_x)
            median_y = statistics.median(centers_y)
            area_q10 = _percentile(areas, 0.10)
            area_q25 = _percentile(areas, 0.25)
            area_q50 = statistics.median(areas)
            area_q90 = _percentile(areas, 0.90)
            profile.update(
                {
                    "median_center_x_ratio": _rounded(median_x),
                    "median_center_y_ratio": _rounded(median_y),
                    "success_area_ratio_p10": _rounded(area_q10),
                    "success_area_ratio_p50": _rounded(area_q50),
                    "success_area_ratio_p90": _rounded(area_q90),
                    "suggested_grasp_area_ratio": _rounded(area_q25),
                }
            )
            if sufficient:
                sufficient_successes.extend(successful)
                class_centers_x.append(median_x)
                class_centers_y.append(median_y)
                suggested_areas[label] = round(area_q25, 6)
                overlapping_failures = [
                    sample
                    for sample in failures
                    if sample.target.area_ratio >= area_q25
                ]
                profile["failure_count_at_or_above_suggested_area"] = len(
                    overlapping_failures
                )
                if overlapping_failures:
                    warnings.append(
                        f"{label} 有 {len(overlapping_failures)} 个失败样本的框"
                        "面积已达建议阈值；面积不能单独证明可以闭爪"
                    )
        if failures:
            profile.update(
                {
                    "failure_median_center_x_ratio": _rounded(
                        statistics.median(
                            sample.target.center_x_ratio for sample in failures
                        )
                    ),
                    "failure_median_center_y_ratio": _rounded(
                        statistics.median(
                            sample.target.center_y_ratio for sample in failures
                        )
                    ),
                    "failure_area_ratio_p50": _rounded(
                        statistics.median(
                            sample.target.area_ratio for sample in failures
                        )
                    ),
                }
            )
        if not sufficient:
            warnings.append(
                f"{label} 只有 {len(successful)} 个成功样本；"
                f"至少需要 {minimum_success_samples} 个才给出可粘贴建议"
            )
        profiles[label] = profile

    suggestion: dict[str, object] | None = None
    if sufficient_successes:
        # 先对每个类别求中位数，再跨类别求中位数，避免某一类样本数量特别多
        # 时把共同瞄准点完全拉向该类别。
        aim_x = statistics.median(class_centers_x)
        aim_y = statistics.median(class_centers_y)
        x_deviations = [
            abs(sample.target.center_x_ratio - aim_x)
            for sample in sufficient_successes
        ]
        y_deviations = [
            abs(sample.target.center_y_ratio - aim_y)
            for sample in sufficient_successes
        ]
        # mission.py 的误差以半幅画面为 1，因此全画面比例偏差要乘 2。
        horizontal_tolerance = min(
            0.20, max(0.02, 2.0 * _percentile(x_deviations, 0.90) * 1.25)
        )
        vertical_tolerance = min(
            0.25, max(0.02, 2.0 * _percentile(y_deviations, 0.90) * 1.25)
        )
        suggestion = {
            "field_tuning": {
                "grasp_aim_x_ratio": _rounded(aim_x, 4),
                "grasp_aim_y_ratio": _rounded(aim_y, 4),
                "grasp_area_ratio": {
                    label: _rounded(value, 4)
                    for label, value in suggested_areas.items()
                },
            },
            "mission": {
                "horizontal_tolerance": _rounded(horizontal_tolerance, 4),
                "vertical_tolerance": _rounded(vertical_tolerance, 4),
            },
        }
        if (
            max(class_centers_x) - min(class_centers_x) > 0.05
            or max(class_centers_y) - min(class_centers_y) > 0.05
        ):
            warnings.append(
                "不同类别的成功瞄准点相差超过画面 5%；"
                "请人工复核，必要时改为每类独立瞄准点"
            )

    outcome_counts = {
        outcome: sum(sample.outcome == outcome for sample in samples)
        for outcome in ("success", "failure", "bad_pose", "pending", "discarded")
    }
    return {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "review_required": True,
        "method": (
            "成功样本中心中位数；面积25%分位；中心绝对偏差90%分位加25%余量"
        ),
        "minimum_success_samples_per_class": minimum_success_samples,
        "outcome_counts": outcome_counts,
        "profiles": profiles,
        "autonomy_yaml_suggestion": suggestion,
        "warnings": warnings,
        "safety_note": (
            "建议值不会自动写入 autonomy.yaml；必须结合截图、失败样本和实艇"
            "空爪测试人工审核"
        ),
    }


def _safe_label(label: str) -> str:
    value = re.sub(r"[^A-Za-z0-9_-]+", "_", label).strip("_")
    return value or "target"


def _optional_number(value: float | None) -> str:
    if value is None or not math.isfinite(float(value)):
        return ""
    return f"{float(value):.6f}"


class GraspCalibrationSession:
    """原子保存截图、CSV 和参数建议；总会话由运行时统一记录。"""

    def __init__(
        self,
        session_directory: str | Path,
        *,
        target_labels: Sequence[str],
        display_names: dict[str, str] | None = None,
        autonomy_config_path: str | Path | None = None,
        targets_config_path: str | Path | None = None,
        minimum_success_samples: int = 5,
        allow_existing_directory: bool = False,
    ) -> None:
        self.session_directory = Path(session_directory).expanduser().resolve()
        self.screenshot_directory = self.session_directory / "screenshots"
        self.csv_path = self.session_directory / "grasp_samples.csv"
        self.suggestion_path = self.session_directory / "recommended_grasp.yaml"
        self.target_labels = tuple(target_labels)
        self.display_names = dict(display_names or {})
        self.minimum_success_samples = int(minimum_success_samples)
        self.samples: list[CalibrationSample] = []
        self.started_at_utc = datetime.now(timezone.utc).isoformat()
        self.finished_at_utc: str | None = None
        self.autonomy_config_path = (
            str(Path(autonomy_config_path).expanduser().resolve())
            if autonomy_config_path is not None
            else None
        )
        self.targets_config_path = (
            str(Path(targets_config_path).expanduser().resolve())
            if targets_config_path is not None
            else None
        )

        if self.minimum_success_samples < 3:
            raise ValueError("每类最少成功样本数不能小于 3")
        if not self.target_labels:
            raise ValueError("标定类别不能为空")
        if (
            not allow_existing_directory
            and self.session_directory.exists()
            and any(self.session_directory.iterdir())
        ):
            raise GraspCalibrationError(
                f"标定输出目录已经包含文件，拒绝覆盖: {self.session_directory}"
            )
        if allow_existing_directory:
            protected = (
                self.csv_path,
                self.suggestion_path,
            )
            if any(path.exists() for path in protected):
                raise GraspCalibrationError("本会话已包含抓取标定结果，拒绝覆盖")
        self.screenshot_directory.mkdir(parents=True, exist_ok=True)
        self._persist()

    @property
    def pending_sample(self) -> CalibrationSample | None:
        return next(
            (sample for sample in reversed(self.samples) if sample.outcome == "pending"),
            None,
        )

    def counts(self) -> dict[str, int]:
        return {
            outcome: sum(sample.outcome == outcome for sample in self.samples)
            for outcome in ("success", "failure", "bad_pose", "pending", "discarded")
        }

    def capture(
        self,
        *,
        target: BoxMetrics,
        telemetry: CalibrationTelemetry,
        detection_stamp_s: float,
        image_stamp_s: float,
        jpeg_data: bytes,
        frame_id: int = 0,
        manual_power: float = 0.0,
        command_forward: float = 0.0,
        command_lateral: float = 0.0,
        command_vertical: float = 0.0,
        command_yaw: float = 0.0,
        gripper_profile: str = "",
        note: str = "",
    ) -> CalibrationSample:
        """保存一次待确认的闭爪前姿态；同时只允许一个 pending 样本。"""

        if self.pending_sample is not None:
            raise GraspCalibrationError("请先用 Y/N 确认上一条抓取结果")
        if not jpeg_data or not jpeg_data.startswith(b"\xff\xd8"):
            raise GraspCalibrationError("没有取得与检测框匹配的 JPEG 截图")
        detection_time = float(detection_stamp_s)
        image_time = float(image_stamp_s)
        if not all(math.isfinite(value) for value in (detection_time, image_time)):
            raise GraspCalibrationError("检测或图像时间戳无效")

        sample_id = len(self.samples) + 1
        filename = f"{sample_id:04d}_{_safe_label(target.label)}.jpg"
        relative_path = Path("screenshots") / filename
        screenshot_path = self.session_directory / relative_path
        temporary_path = screenshot_path.with_suffix(".jpg.tmp")
        temporary_path.write_bytes(jpeg_data)
        temporary_path.replace(screenshot_path)

        sample = CalibrationSample(
            sample_id=sample_id,
            utc_time=datetime.now(timezone.utc).isoformat(),
            detection_stamp_s=detection_time,
            image_stamp_s=image_time,
            image_time_delta_s=abs(image_time - detection_time),
            target=target,
            telemetry=telemetry,
            screenshot_file=relative_path.as_posix(),
            frame_id=int(frame_id),
            manual_power=float(manual_power),
            command_forward=float(command_forward),
            command_lateral=float(command_lateral),
            command_vertical=float(command_vertical),
            command_yaw=float(command_yaw),
            gripper_profile=str(gripper_profile),
            note=str(note),
        )
        self.samples.append(sample)
        self._persist()
        return sample

    def record_gripper_result(
        self,
        action: str,
        *,
        accepted: bool,
        acknowledgement: str,
    ) -> CalibrationSample:
        """把机械爪命令和网关响应附加到当前待确认样本。"""

        pending = self.pending_sample
        if pending is None:
            raise GraspCalibrationError("请先按 Enter/G 保存闭爪前位置")
        clean_action = str(action).strip().lower()
        if clean_action not in {"open", "close"}:
            raise GraspCalibrationError(f"未知机械爪动作: {action!r}")
        updated = replace(
            pending,
            gripper_action=clean_action,
            gripper_command_accepted=bool(accepted),
            gripper_ack=str(acknowledgement),
        )
        self.samples[pending.sample_id - 1] = updated
        self._persist()
        return updated

    def mark_pending(self, outcome: str, *, note: str = "") -> CalibrationSample:
        """把最近的待确认姿态标记为成功、失败或丢弃。"""

        result = str(outcome).strip().lower()
        if result not in VALID_FINAL_OUTCOMES:
            raise GraspCalibrationError(f"未知标定结果: {outcome!r}")
        pending = self.pending_sample
        if pending is None:
            raise GraspCalibrationError("当前没有等待确认的抓取姿态")
        updated = replace(pending, outcome=result, note=str(note) or pending.note)
        self.samples[pending.sample_id - 1] = updated
        self._persist()
        return updated

    def mark_grasp_outcome(
        self, outcome: str, *, note: str = ""
    ) -> CalibrationSample:
        """只有闭爪命令被网关接受后，才允许标记实物抓取结果。"""

        pending = self.pending_sample
        if pending is None:
            raise GraspCalibrationError("当前没有等待确认的抓取姿态")
        if pending.gripper_action != "close":
            raise GraspCalibrationError("请先对当前样本发送闭爪命令 C")
        if pending.gripper_command_accepted is not True:
            raise GraspCalibrationError("闭爪命令未获网关接受，不能标记成功或失败")
        return self.mark_pending(outcome, note=note)

    def capture_bad_pose(self, **capture_arguments) -> CalibrationSample:
        """记录无需真正闭爪即可判断不合格的位置。"""

        self.capture(**capture_arguments)
        return self.mark_pending("bad_pose")

    def finish(self) -> None:
        """记录结束时间；未确认样本保留为 pending 且不参与建议。"""

        if self.finished_at_utc is None:
            self.finished_at_utc = datetime.now(timezone.utc).isoformat()
        self._persist()

    def _sample_row(self, sample: CalibrationSample) -> dict[str, object]:
        target = sample.target
        telemetry = sample.telemetry
        return {
            "sample_id": sample.sample_id,
            "utc_time": sample.utc_time,
            "outcome": sample.outcome,
            "note": sample.note,
            "frame_id": sample.frame_id,
            "label": target.label,
            "class_id": target.class_id,
            "confidence": f"{target.confidence:.6f}",
            "frame_width": target.frame_width,
            "frame_height": target.frame_height,
            "left": f"{target.left:.3f}",
            "top": f"{target.top:.3f}",
            "right": f"{target.right:.3f}",
            "bottom": f"{target.bottom:.3f}",
            "center_x_ratio": f"{target.center_x_ratio:.6f}",
            "center_y_ratio": f"{target.center_y_ratio:.6f}",
            "width_ratio": f"{target.width_ratio:.6f}",
            "height_ratio": f"{target.height_ratio:.6f}",
            "area_ratio": f"{target.area_ratio:.6f}",
            "depth_m": _optional_number(telemetry.depth_m),
            "roll_deg": _optional_number(telemetry.roll_deg),
            "pitch_deg": _optional_number(telemetry.pitch_deg),
            "yaw_deg": _optional_number(telemetry.yaw_deg),
            "flight_mode": telemetry.flight_mode,
            "armed": "" if telemetry.armed is None else str(telemetry.armed).lower(),
            "detection_stamp_s": f"{sample.detection_stamp_s:.9f}",
            "image_stamp_s": f"{sample.image_stamp_s:.9f}",
            "image_time_delta_s": f"{sample.image_time_delta_s:.6f}",
            "screenshot_file": sample.screenshot_file,
            "manual_power": f"{sample.manual_power:.6f}",
            "command_forward": f"{sample.command_forward:.6f}",
            "command_lateral": f"{sample.command_lateral:.6f}",
            "command_vertical": f"{sample.command_vertical:.6f}",
            "command_yaw": f"{sample.command_yaw:.6f}",
            "gripper_profile": sample.gripper_profile,
            "gripper_action": sample.gripper_action,
            "gripper_command_accepted": (
                ""
                if sample.gripper_command_accepted is None
                else str(sample.gripper_command_accepted).lower()
            ),
            "gripper_ack": sample.gripper_ack,
        }

    def _persist(self) -> None:
        summary = build_calibration_summary(
            self.samples,
            self.target_labels,
            minimum_success_samples=self.minimum_success_samples,
        )
        csv_temporary = self.csv_path.with_suffix(".csv.tmp")
        with csv_temporary.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self._sample_row(sample) for sample in self.samples)
        csv_temporary.replace(self.csv_path)

        self._atomic_text(
            self.suggestion_path,
            yaml.safe_dump(
                {
                    "review_required": True,
                    "started_at_utc": self.started_at_utc,
                    "finished_at_utc": self.finished_at_utc,
                    "minimum_success_samples": self.minimum_success_samples,
                    "sample_counts": self.counts(),
                    "source_configs": {
                        "autonomy": self.autonomy_config_path,
                        "targets": self.targets_config_path,
                    },
                    "per_target": summary["profiles"],
                    "autonomy_yaml_suggestion": summary[
                        "autonomy_yaml_suggestion"
                    ],
                    "warnings": summary["warnings"],
                    "safety_note": summary["safety_note"],
                },
                allow_unicode=True,
                sort_keys=False,
            ),
        )

    @staticmethod
    def _atomic_text(path: Path, content: str) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(content, encoding="utf-8")
        temporary.replace(path)
