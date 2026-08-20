"""视频逐帧导出核心；不依赖 ROS，也不会修改原视频。"""

from __future__ import annotations

import csv
import json
import math
import os
import re
import shutil
import statistics
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable

import cv2


class FrameExtractionError(RuntimeError):
    """视频无法读取、图片无法写入或磁盘空间不足。"""


@dataclass(frozen=True)
class VideoInfo:
    """从视频容器读取的基本信息。"""

    path: Path
    width: int
    height: int
    fps: float
    total_frames: int
    duration_s: float | None
    file_size_bytes: int


@dataclass(frozen=True)
class ExtractionOptions:
    """一次逐帧导出的不可变参数。"""

    video_path: Path
    output_directory: Path
    image_format: str = "jpg"
    jpeg_quality: int = 95
    # 0 表示全部帧；正数表示从完整视频中均匀选择该数量。
    frame_count: int = 0
    filename_prefix: str = "frame"
    minimum_free_bytes: int = 512 * 1024 * 1024

    def normalized_format(self) -> str:
        """统一格式名称，并拒绝无法保证写出的格式。"""

        image_format = self.image_format.lower().lstrip(".")
        if image_format == "jpeg":
            image_format = "jpg"
        if image_format not in {"jpg", "png"}:
            raise FrameExtractionError("图片格式只能是 JPG 或 PNG")
        return image_format

    def validate(self) -> None:
        """在创建任何输出文件前检查参数。"""

        if not self.video_path.expanduser().is_file():
            raise FrameExtractionError(f"视频不存在：{self.video_path}")
        self.normalized_format()
        if not 1 <= int(self.jpeg_quality) <= 100:
            raise FrameExtractionError("JPG 质量必须在 1 到 100 之间")
        if int(self.frame_count) < 0:
            raise FrameExtractionError("输出帧数不能小于 0；0 表示全部帧")
        if not re.fullmatch(r"[A-Za-z0-9_-]+", self.filename_prefix):
            raise FrameExtractionError(
                "文件名前缀只能包含英文、数字、下划线和短横线"
            )
        if int(self.minimum_free_bytes) < 0:
            raise FrameExtractionError("最小剩余磁盘空间不能为负数")


@dataclass(frozen=True)
class ExtractionProgress:
    """工作线程发送给界面的轻量进度快照。"""

    frames_scanned: int
    frames_written: int
    total_frames: int
    target_frames: int
    elapsed_s: float
    eta_s: float | None
    current_file: Path | None


@dataclass(frozen=True)
class ExtractionResult:
    """一次导出的最终结果。"""

    outcome: str
    frames_written: int
    output_directory: Path
    image_directory: Path
    elapsed_s: float
    detail: str


ProgressCallback = Callable[[ExtractionProgress], None]


def probe_video(path: str | Path) -> VideoInfo:
    """打开视频并读取尺寸、帧率、帧数和时长。

    OpenCV 返回的帧数只是容器声明值。真正导出时仍一直读取到解码器
    返回结束，不能只循环该数字，否则某些可变帧率视频会漏帧。
    """

    video_path = Path(path).expanduser().resolve()
    if not video_path.is_file():
        raise FrameExtractionError(f"视频不存在：{video_path}")

    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        capture.release()
        raise FrameExtractionError(f"无法打开视频：{video_path}")
    try:
        width = int(round(capture.get(cv2.CAP_PROP_FRAME_WIDTH)))
        height = int(round(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        raw_total = float(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        total_frames = (
            int(round(raw_total))
            if math.isfinite(raw_total) and raw_total > 0
            else 0
        )
        if not math.isfinite(fps) or fps <= 0:
            fps = 0.0

        # 有些容器不填写宽高，读取第一帧作为兜底；不修改原文件。
        if width <= 0 or height <= 0:
            ok, frame = capture.read()
            if not ok or frame is None:
                raise FrameExtractionError("视频已打开，但无法解码第一帧")
            height, width = frame.shape[:2]
        duration_s = (
            total_frames / fps if total_frames > 0 and fps > 0 else None
        )
        return VideoInfo(
            path=video_path,
            width=width,
            height=height,
            fps=fps,
            total_frames=total_frames,
            duration_s=duration_s,
            file_size_bytes=video_path.stat().st_size,
        )
    finally:
        capture.release()


def _safe_video_stem(path: Path) -> str:
    """生成简短、可读且适合作为目录名的视频名称。"""

    name = re.sub(r"[^\w.-]+", "_", path.stem, flags=re.UNICODE).strip("._")
    return name[:80] or "video"


def default_output_directory(
    video_path: str | Path,
    output_root: str | Path,
    *,
    now: datetime | None = None,
) -> Path:
    """为一次导出生成不会覆盖旧结果的独立目录。"""

    source = Path(video_path).expanduser().resolve()
    root = Path(output_root).expanduser().resolve()
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    base = root / f"{_safe_video_stem(source)}_{stamp}"
    candidate = base
    suffix = 2
    while candidate.exists():
        candidate = Path(f"{base}_{suffix}")
        suffix += 1
    return candidate


def uniform_frame_indices(
    total_frames: int, requested_frames: int
) -> range | tuple[int, ...]:
    """计算覆盖完整视频的均匀帧序号。

    ``requested_frames=0`` 表示全部帧。指定一帧时取中间；两帧及以上
    包含第一帧和最后一帧。计算只用整数，避免长视频浮点舍入造成重复。
    """

    total = int(total_frames)
    requested = int(requested_frames)
    if total < 0:
        raise FrameExtractionError("视频总帧数不能为负数")
    if requested < 0:
        raise FrameExtractionError("输出帧数不能小于 0")
    if total == 0:
        if requested == 0:
            return ()
        raise FrameExtractionError("视频未报告总帧数，只能选择 0（全部帧）")
    if requested == 0 or requested == total:
        return range(total)
    if requested > total:
        raise FrameExtractionError(
            f"输出帧数 {requested} 超过视频最大帧数 {total}"
        )
    if requested == 1:
        return ((total - 1) // 2,)

    denominator = requested - 1
    last_index = total - 1
    return tuple(
        (index * last_index + denominator // 2) // denominator
        for index in range(requested)
    )


def _encoding_parameters(image_format: str, jpeg_quality: int) -> tuple[str, list[int]]:
    """返回 OpenCV 编码扩展名和参数。"""

    if image_format == "jpg":
        return ".jpg", [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)]
    return ".png", [cv2.IMWRITE_PNG_COMPRESSION, 3]


def estimate_output_bytes(
    info: VideoInfo,
    *,
    image_format: str = "jpg",
    jpeg_quality: int = 95,
    frame_count: int = 0,
) -> int | None:
    """抽取最多三张代表帧，估算本次所选数量的磁盘占用。"""

    normalized = image_format.lower().lstrip(".")
    if normalized == "jpeg":
        normalized = "jpg"
    if normalized not in {"jpg", "png"} or info.total_frames <= 0:
        return None
    requested = int(frame_count)
    if requested < 0 or requested > info.total_frames:
        uniform_frame_indices(info.total_frames, requested)
    output_count = info.total_frames if requested == 0 else requested
    extension, parameters = _encoding_parameters(normalized, jpeg_quality)

    positions = {0}
    if info.total_frames > 2:
        positions.add(info.total_frames // 2)
        positions.add(max(0, info.total_frames - 2))

    capture = cv2.VideoCapture(str(info.path))
    sizes: list[int] = []
    try:
        for position in sorted(positions):
            capture.set(cv2.CAP_PROP_POS_FRAMES, float(position))
            ok, frame = capture.read()
            if not ok or frame is None:
                continue
            encoded_ok, encoded = cv2.imencode(extension, frame, parameters)
            if encoded_ok:
                sizes.append(int(encoded.nbytes))
    finally:
        capture.release()
    if not sizes:
        return None

    # 中位数比单张最大值更接近实际，再预留 20% 应对复杂场景和 CSV/JSON。
    representative = statistics.median(sizes)
    return int(representative * output_count * 1.20)


def _existing_ancestor(path: Path) -> Path:
    """找到可用于查询剩余空间的最近已存在父目录。"""

    candidate = path
    while not candidate.exists() and candidate != candidate.parent:
        candidate = candidate.parent
    return candidate


def available_disk_bytes(path: str | Path) -> int:
    """返回目标所在文件系统的当前可用字节数。"""

    target = _existing_ancestor(Path(path).expanduser().resolve())
    return int(shutil.disk_usage(target).free)


def _write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    """原子更新状态文件，避免突然断电只留下半个 JSON。"""

    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _write_frame_atomic(
    path: Path,
    frame,
    *,
    extension: str,
    parameters: list[int],
) -> None:
    """先编码到临时文件，再改名为正式图片。"""

    ok, encoded = cv2.imencode(extension, frame, parameters)
    if not ok:
        raise FrameExtractionError(f"图片编码失败：{path.name}")
    temporary = path.with_suffix(path.suffix + ".partial")
    try:
        encoded.tofile(str(temporary))
        temporary.replace(path)
    except OSError as exc:
        raise FrameExtractionError(f"写入图片失败：{path}：{exc}") from exc


def extract_all_frames(
    options: ExtractionOptions,
    *,
    cancel_event: threading.Event | None = None,
    progress_callback: ProgressCallback | None = None,
) -> ExtractionResult:
    """把全部帧或均匀选择的指定数量写入 ``images/``。

    取消时保留已经成功写出的图片，并在 ``extraction.json`` 中记录
    ``cancelled``；异常则记录 ``failed``。只有读到正常结尾才写
    ``completed``，因此半份数据不会伪装成完整结果。
    """

    options.validate()
    source = options.video_path.expanduser().resolve()
    output = options.output_directory.expanduser().resolve()
    image_directory = output / "images"
    if output.exists() and not output.is_dir():
        raise FrameExtractionError(f"输出路径不是目录：{output}")
    if output.exists() and any(output.iterdir()):
        raise FrameExtractionError(f"输出目录不是空目录：{output}")

    info = probe_video(source)
    requested_frames = int(options.frame_count)
    if info.total_frames <= 0 and requested_frames > 0:
        raise FrameExtractionError(
            "视频容器未报告总帧数，不能均匀分配指定数量；请选择 0 导出全部"
        )
    if info.total_frames > 0 and requested_frames > info.total_frames:
        raise FrameExtractionError(
            f"输出帧数 {requested_frames} 超过视频最大帧数 {info.total_frames}"
        )
    export_all = requested_frames == 0 or requested_frames == info.total_frames
    target_frames = (
        info.total_frames if export_all else requested_frames
    )
    selected_indices: set[int] | None = None
    if not export_all:
        selected_indices = set(
            uniform_frame_indices(info.total_frames, requested_frames)
        )
    image_format = options.normalized_format()
    extension, encoding_parameters = _encoding_parameters(
        image_format, options.jpeg_quality
    )
    estimated_bytes = estimate_output_bytes(
        info,
        image_format=image_format,
        jpeg_quality=options.jpeg_quality,
        frame_count=requested_frames,
    )
    free_bytes = available_disk_bytes(output)
    required_reserve = int(options.minimum_free_bytes)
    if estimated_bytes is not None and estimated_bytes + required_reserve > free_bytes:
        raise FrameExtractionError(
            "预计输出需要约 "
            f"{estimated_bytes / 1024**3:.2f} GiB，但目标磁盘只剩 "
            f"{free_bytes / 1024**3:.2f} GiB；请更换输出位置或释放空间"
        )

    output.mkdir(parents=True, exist_ok=True)
    image_directory.mkdir()
    status_path = output / "extraction.json"
    csv_path = output / "frames.csv"
    started_wall = datetime.now().astimezone().isoformat()
    started = time.monotonic()
    frames_scanned = 0
    frames_written = 0
    detail = ""
    outcome = "running"
    current_file: Path | None = None
    cancellation = cancel_event or threading.Event()

    status: dict[str, object] = {
        "outcome": outcome,
        "detail": detail,
        "source_video": str(source),
        "source_size_bytes": info.file_size_bytes,
        "started_at": started_wall,
        "finished_at": None,
        "width": info.width,
        "height": info.height,
        "fps": info.fps,
        "reported_total_frames": info.total_frames,
        "requested_frame_count": requested_frames,
        "target_frame_count": target_frames,
        "selection_mode": "all" if export_all else "uniform",
        "frames_scanned": 0,
        "frames_written": 0,
        "image_format": image_format,
        "jpeg_quality": options.jpeg_quality if image_format == "jpg" else None,
        "image_directory": str(image_directory),
        "estimated_output_bytes": estimated_bytes,
    }
    _write_json_atomic(status_path, status)

    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        status.update(outcome="failed", detail="导出阶段无法重新打开视频")
        _write_json_atomic(status_path, status)
        raise FrameExtractionError(str(status["detail"]))

    try:
        with csv_path.open("w", encoding="utf-8", newline="") as csv_stream:
            writer = csv.DictWriter(
                csv_stream,
                fieldnames=(
                    "image",
                    "source_frame_index",
                    "timestamp_s",
                ),
            )
            writer.writeheader()

            def emit_progress() -> None:
                """按已扫描帧计算进度；抽样时写盘数可能远小于扫描数。"""

                if progress_callback is None:
                    return
                elapsed = max(time.monotonic() - started, 1e-9)
                eta_s = None
                if info.total_frames > frames_scanned > 0:
                    eta_s = (
                        (info.total_frames - frames_scanned)
                        * elapsed
                        / frames_scanned
                    )
                progress_callback(
                    ExtractionProgress(
                        frames_scanned=frames_scanned,
                        frames_written=frames_written,
                        total_frames=info.total_frames,
                        target_frames=target_frames,
                        elapsed_s=elapsed,
                        eta_s=eta_s,
                        current_file=current_file,
                    )
                )

            while True:
                if cancellation.is_set():
                    outcome = "cancelled"
                    detail = "操作员取消；已导出的图片予以保留"
                    break

                ok, frame = capture.read()
                if not ok or frame is None:
                    outcome = "completed"
                    detail = (
                        "已读取到视频结尾"
                        if export_all
                        else f"已扫描完整视频并均匀导出 {frames_written} 帧"
                    )
                    emit_progress()
                    break

                source_index = frames_scanned
                should_write = (
                    selected_indices is None or source_index in selected_indices
                )
                if should_write:
                    filename = (
                        f"{options.filename_prefix}_{frames_written + 1:08d}"
                        f"{extension}"
                    )
                    current_file = image_directory / filename
                    _write_frame_atomic(
                        current_file,
                        frame,
                        extension=extension,
                        parameters=encoding_parameters,
                    )
                    timestamp_s = source_index / info.fps if info.fps > 0 else ""
                    writer.writerow(
                        {
                            "image": f"images/{filename}",
                            "source_frame_index": source_index,
                            "timestamp_s": (
                                f"{timestamp_s:.6f}" if timestamp_s != "" else ""
                            ),
                        }
                    )
                    frames_written += 1
                    if frames_written % 100 == 0:
                        csv_stream.flush()
                        if available_disk_bytes(output) < required_reserve:
                            raise FrameExtractionError(
                                "磁盘可用空间低于安全余量，已停止导出"
                            )

                frames_scanned += 1
                if (
                    frames_scanned == 1
                    or frames_scanned % 10 == 0
                    or frames_scanned == info.total_frames
                ):
                    emit_progress()

        # 指定数量时必须准确写出目标数；否则视频或容器信息不完整。
        if (
            outcome == "completed"
            and not export_all
            and frames_written != target_frames
        ):
            outcome = "failed"
            detail = (
                f"计划均匀导出 {target_frames} 帧，但实际只写出 "
                f"{frames_written} 帧；视频可能损坏或帧数信息不准确"
            )

        # 容器若声明了明显更多帧，通常意味着解码提前中断。保留图片但
        # 标为失败，防止团队误以为已经扫描了完整视频。
        if (
            outcome == "completed"
            and info.total_frames > 0
            and frames_scanned + max(2, int(info.total_frames * 0.02))
            < info.total_frames
        ):
            outcome = "failed"
            detail = (
                f"视频声明 {info.total_frames} 帧，但只解码出 "
                f"{frames_scanned} 帧；可能存在损坏或不支持的编码"
            )
    except (FrameExtractionError, OSError, cv2.error) as exc:
        outcome = "failed"
        detail = str(exc)
        raise FrameExtractionError(detail) from exc
    finally:
        capture.release()
        elapsed = time.monotonic() - started
        status.update(
            outcome=outcome,
            detail=detail,
            finished_at=datetime.now().astimezone().isoformat(),
            frames_scanned=frames_scanned,
            frames_written=frames_written,
            elapsed_s=elapsed,
        )
        _write_json_atomic(status_path, status)

    return ExtractionResult(
        outcome=outcome,
        frames_written=frames_written,
        output_directory=output,
        image_directory=image_directory,
        elapsed_s=time.monotonic() - started,
        detail=detail,
    )
