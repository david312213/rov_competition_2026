"""桌面抽帧工具的纯软件测试。"""

from __future__ import annotations

import csv
import hashlib
import json
import threading
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pytest

from rov_competition.frame_extractor import (
    ExtractionOptions,
    FrameExtractionError,
    default_output_directory,
    estimate_output_bytes,
    extract_all_frames,
    probe_video,
    uniform_frame_indices,
)


PROJECT = Path(__file__).resolve().parents[1]
GUI_SOURCE = (
    PROJECT
    / "ros2_ws/src/rov_competition/rov_competition/frame_extractor_gui.py"
)
LAUNCHER = PROJECT / "scripts/start_frame_extractor.sh"
SHORTCUT_INSTALLER = PROJECT / "scripts/install_frame_extractor_shortcut.sh"
REQUIREMENTS = PROJECT / "requirements.txt"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


@pytest.fixture
def sample_video(tmp_path: Path) -> Path:
    """创建包含七张不同画面的短 MJPEG 视频。"""

    path = tmp_path / "水池 测试.avi"
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"MJPG"),
        10.0,
        (96, 64),
    )
    if not writer.isOpened():
        pytest.skip("当前 OpenCV 构建不支持 MJPEG VideoWriter")
    for index in range(7):
        frame = np.zeros((64, 96, 3), dtype=np.uint8)
        frame[:, :] = (index * 20, 40 + index * 10, 180 - index * 15)
        cv2.putText(
            frame,
            str(index),
            (34, 42),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
        )
        writer.write(frame)
    writer.release()
    assert path.stat().st_size > 0
    return path


def test_probe_and_estimate_video(sample_video: Path) -> None:
    """界面所需信息和容量估算应来自真实可解码帧。"""

    info = probe_video(sample_video)
    assert info.width == 96
    assert info.height == 64
    assert info.fps == pytest.approx(10.0, rel=0.02)
    assert info.total_frames == 7
    assert info.duration_s == pytest.approx(0.7, rel=0.02)
    estimate = estimate_output_bytes(info, image_format="jpg", jpeg_quality=95)
    assert estimate is not None
    assert estimate > 0
    selected_estimate = estimate_output_bytes(
        info,
        image_format="jpg",
        jpeg_quality=95,
        frame_count=3,
    )
    assert selected_estimate is not None
    assert 0 < selected_estimate < estimate


def test_uniform_frame_indices_cover_the_whole_video() -> None:
    """少量帧必须覆盖完整时间范围，而不是只取开头。"""

    assert tuple(uniform_frame_indices(1000, 5)) == (0, 250, 500, 749, 999)
    assert tuple(uniform_frame_indices(10, 1)) == (4,)
    assert tuple(uniform_frame_indices(10, 2)) == (0, 9)
    assert tuple(uniform_frame_indices(7, 0)) == tuple(range(7))
    with pytest.raises(FrameExtractionError, match="超过视频最大帧数"):
        uniform_frame_indices(7, 8)


def test_extract_all_frames_preserves_source_and_writes_evidence(
    sample_video: Path,
    tmp_path: Path,
) -> None:
    """必须逐帧导出、顺序命名，并且不改原视频。"""

    before_hash = _sha256(sample_video)
    output = tmp_path / "output"
    progress = []
    result = extract_all_frames(
        ExtractionOptions(
            video_path=sample_video,
            output_directory=output,
            image_format="jpg",
            jpeg_quality=95,
            minimum_free_bytes=0,
        ),
        progress_callback=progress.append,
    )

    assert result.outcome == "completed"
    assert result.frames_written == 7
    images = sorted((output / "images").glob("*.jpg"))
    assert [path.name for path in images] == [
        f"frame_{index:08d}.jpg" for index in range(1, 8)
    ]
    assert cv2.imread(str(images[0])).shape == (64, 96, 3)
    assert _sha256(sample_video) == before_hash
    assert progress
    assert progress[-1].frames_scanned == 7
    assert progress[-1].frames_written == 7

    with (output / "frames.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 7
    assert rows[0]["source_frame_index"] == "0"
    assert rows[0]["timestamp_s"] == "0.000000"
    assert rows[-1]["timestamp_s"] == "0.600000"

    metadata = json.loads((output / "extraction.json").read_text(encoding="utf-8"))
    assert metadata["outcome"] == "completed"
    assert metadata["frames_written"] == 7
    assert metadata["source_video"] == str(sample_video.resolve())


def test_extract_requested_count_uses_uniform_source_frames(
    sample_video: Path,
    tmp_path: Path,
) -> None:
    """指定三帧时应导出首、中、尾，并保留原帧序号证据。"""

    output = tmp_path / "selected"
    progress = []
    result = extract_all_frames(
        ExtractionOptions(
            video_path=sample_video,
            output_directory=output,
            image_format="png",
            frame_count=3,
            minimum_free_bytes=0,
        ),
        progress_callback=progress.append,
    )

    assert result.outcome == "completed"
    assert result.frames_written == 3
    assert len(list((output / "images").glob("*.png"))) == 3
    with (output / "frames.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert [int(row["source_frame_index"]) for row in rows] == [0, 3, 6]
    assert progress[-1].frames_scanned == 7
    assert progress[-1].frames_written == 3
    assert progress[-1].target_frames == 3

    metadata = json.loads((output / "extraction.json").read_text(encoding="utf-8"))
    assert metadata["selection_mode"] == "uniform"
    assert metadata["requested_frame_count"] == 3
    assert metadata["target_frame_count"] == 3
    assert metadata["frames_scanned"] == 7
    assert metadata["frames_written"] == 3


def test_cancel_keeps_completed_directory_but_marks_it_incomplete(
    sample_video: Path,
    tmp_path: Path,
) -> None:
    """取消不能伪装成完整导出，也不能删除证据目录。"""

    cancellation = threading.Event()
    cancellation.set()
    output = tmp_path / "cancelled"
    result = extract_all_frames(
        ExtractionOptions(
            video_path=sample_video,
            output_directory=output,
            minimum_free_bytes=0,
        ),
        cancel_event=cancellation,
    )
    assert result.outcome == "cancelled"
    assert result.frames_written == 0
    assert not list((output / "images").iterdir())
    metadata = json.loads((output / "extraction.json").read_text(encoding="utf-8"))
    assert metadata["outcome"] == "cancelled"


def test_output_directory_is_unique_and_existing_data_is_never_overwritten(
    sample_video: Path,
    tmp_path: Path,
) -> None:
    """同一秒重复操作也必须生成新目录，并拒绝写入非空目录。"""

    fixed_time = datetime(2026, 8, 21, 12, 30, 0)
    first = default_output_directory(
        sample_video, tmp_path, now=fixed_time
    )
    first.mkdir()
    second = default_output_directory(
        sample_video, tmp_path, now=fixed_time
    )
    assert second != first
    assert second.name.endswith("_2")

    (first / "old.txt").write_text("old", encoding="utf-8")
    with pytest.raises(FrameExtractionError, match="不是空目录"):
        extract_all_frames(
            ExtractionOptions(
                video_path=sample_video,
                output_directory=first,
                minimum_free_bytes=0,
            )
        )
    assert (first / "old.txt").read_text(encoding="utf-8") == "old"


def test_gui_and_launcher_are_drag_drop_software_not_robot_control() -> None:
    """桌面入口应支持拖放，同时与实艇控制链保持隔离。"""

    gui = GUI_SOURCE.read_text(encoding="utf-8")
    launcher = LAUNCHER.read_text(encoding="utf-8")
    shortcut = SHORTCUT_INSTALLER.read_text(encoding="utf-8")
    requirements = REQUIREMENTS.read_text(encoding="utf-8")

    assert "drop_target_register" in gui
    assert "splitlist(event.data)" in gui
    assert "开始导出全部帧" in gui
    assert "frame_count_var" in gui
    assert "uniform_frame_indices" in gui
    assert "python -m" not in launcher
    assert "rov_competition.frame_extractor_gui" in launcher
    assert "/opt/ros" not in launcher
    assert "mavproxy" not in launcher.lower()
    assert "rov_vehicle" not in launcher
    assert "ROV 视频抽帧" in shortcut
    assert "tkinterdnd2==0.6.2" in requirements
