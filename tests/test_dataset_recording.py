"""原始 H.264 录像管线和数据集证据文件测试。"""

import csv
import json
from pathlib import Path

import pytest
from rov_competition import dataset_recording as dataset_recording_module
from rov_competition.dataset_recording import (
    DatasetRecordingError,
    DatasetSessionLogger,
    RtpMkvRecorder,
    VideoOnlySession,
    build_recording_pipeline,
)
from rov_competition.domain import MotionCommand


def test_recording_pipeline_remuxes_without_any_encoder(tmp_path: Path) -> None:
    """录像只解 RTP 封装并写 MKV，不允许二次编码拖慢 QGC。"""

    output = tmp_path / "video_raw.partial.mkv"
    command = build_recording_pipeline(
        source_port=5702,
        payload_type=96,
        output_path=output,
    )
    assert "port=5702" in command
    assert "rtph264depay" in command
    assert "h264parse" in command
    assert "matroskamux" in command
    assert f"location={output.resolve()}" in command
    joined = " ".join(command).lower()
    for encoder in ("x264enc", "openh264enc", "nvh264enc", "avenc_h264"):
        assert encoder not in joined


def test_recording_pipeline_rejects_invalid_udp_port(tmp_path: Path) -> None:
    """非法端口在创建子进程前就要报错。"""

    with pytest.raises(DatasetRecordingError, match="1..65535"):
        build_recording_pipeline(
            source_port=0,
            payload_type=96,
            output_path=tmp_path / "bad.mkv",
        )


def test_control_fault_interrupts_eos_wait_instead_of_hiding_it(tmp_path: Path) -> None:
    """封装期间的控制链故障必须停录像并交给上层急停。"""

    class FakeProcess:
        returncode = None
        terminated = False

        def poll(self):
            return self.returncode

        def send_signal(self, _signal):
            return None

        def terminate(self):
            self.terminated = True
            self.returncode = -15

        def wait(self, timeout=None):
            return self.returncode

        def kill(self):
            self.returncode = -9

    recorder = RtpMkvRecorder(tmp_path / "session")
    process = FakeProcess()
    recorder.process = process  # type: ignore[assignment]

    def failed_health_check() -> None:
        raise RuntimeError("飞控心跳过期")

    video, error = recorder.finalize(pump=failed_health_check)
    assert video is None
    assert error is not None and "飞控心跳过期" in error
    assert process.terminated


def test_recorder_that_exited_before_finalize_stays_partial(tmp_path: Path) -> None:
    """录像子进程提前退出时，残留文件不得改名为完整录像。"""

    class ExitedProcess:
        returncode = 1

        def poll(self):
            return self.returncode

    recorder = RtpMkvRecorder(tmp_path / "session")
    recorder.session_directory.mkdir(parents=True)
    recorder.partial_path.write_bytes(b"not a complete recording")
    recorder.process = ExitedProcess()  # type: ignore[assignment]

    video, error = recorder.finalize()
    assert video is None
    assert error is not None and "提前退出" in error
    assert recorder.partial_path.exists()
    assert not recorder.final_path.exists()


def test_session_logger_writes_csv_and_reproducible_metadata(tmp_path: Path) -> None:
    """events.csv 与 session.json 必须记录控制、深度和配置哈希。"""

    robot = tmp_path / "robot.yaml"
    dataset = tmp_path / "dataset.yaml"
    robot.write_text("robot: test\n", encoding="utf-8")
    dataset.write_text("dataset: test\n", encoding="utf-8")
    session = tmp_path / "session"
    logger = DatasetSessionLogger(
        session,
        project_directory=tmp_path,
        robot_config_path=robot,
        dataset_config_path=dataset,
    )
    logger.set_start_depth(0.20)
    logger.write(
        event="command",
        keys={"w", "2"},
        motion=MotionCommand(forward=0.03, yaw=0.03),
        depth_m=0.31,
        yaw_deg=123.4,
        flight_mode="ALT_HOLD",
        armed=True,
        message="test",
    )
    video = session / "video_raw.mkv"
    logger.finish(outcome="completed", detail="ok", video_path=video)

    metadata = json.loads((session / "session.json").read_text(encoding="utf-8"))
    assert metadata["outcome"] == "completed"
    assert metadata["start_depth_m"] == pytest.approx(0.20)
    assert "effective_depth_limit_m" not in metadata
    assert len(metadata["robot_config_sha256"]) == 64
    assert len(metadata["dataset_config_sha256"]) == 64

    with (session / "events.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["keys"] == "2+w"
    assert rows[0]["flight_mode"] == "ALT_HOLD"
    assert rows[0]["armed"] == "true"


def test_video_only_session_does_not_claim_unobserved_gamepad_data(
    tmp_path: Path,
) -> None:
    """只录像模式只记录可证明的文件，不伪造控制日志。"""

    session_directory = tmp_path / "session"
    session = VideoOnlySession(
        session_directory,
        project_directory=tmp_path,
    )
    video = session_directory / "video_raw.mkv"
    session.finish(outcome="completed", detail="operator stop", video_path=video)

    metadata = json.loads(
        (session_directory / "session.json").read_text(encoding="utf-8")
    )
    assert metadata["mode"] == "qgc_gamepad_video_only"
    assert metadata["outcome"] == "completed"
    assert "未观察或记录 QGC 手柄控制量" in metadata["control_note"]
    assert not (session_directory / "events.csv").exists()


def test_recording_growth_timeout_uses_relaxed_five_second_boundary(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """录像短暂不增长可宽限 5s，但越过边界仍报故障。"""

    class RunningProcess:
        @staticmethod
        def poll() -> None:
            return None

    recorder = RtpMkvRecorder(tmp_path / "session")
    recorder.process = RunningProcess()  # type: ignore[assignment]
    recorder._last_growth_at = 100.0
    monkeypatch.setattr(dataset_recording_module.time, "monotonic", lambda: 105.0)
    assert recorder.is_stream_fresh()

    monkeypatch.setattr(dataset_recording_module.time, "monotonic", lambda: 105.001)
    assert not recorder.is_stream_fresh()
