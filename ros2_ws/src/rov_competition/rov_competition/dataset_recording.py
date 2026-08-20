"""原始 RTP/H.264 数据集录像和会话证据文件。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import select
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Iterable

from .domain import MotionCommand


class DatasetRecordingError(RuntimeError):
    """录像管线或数据集证据文件无法安全继续。"""


def _port(value: int, name: str) -> int:
    """校验 UDP 端口。"""

    number = int(value)
    if not 1 <= number <= 65535:
        raise DatasetRecordingError(f"{name} 必须在 1..65535")
    return number


def build_recording_pipeline(
    *, source_port: int, payload_type: int, output_path: str | Path
) -> list[str]:
    """生成不重新编码的 RTP/H.264 -> MKV 参数列表。"""

    source_port = _port(source_port, "录像端口")
    if not 0 <= int(payload_type) <= 127:
        raise DatasetRecordingError("RTP payload type 必须在 0..127")
    path = Path(output_path).expanduser().resolve()
    return [
        "gst-launch-1.0",
        "-e",
        "udpsrc",
        f"port={source_port}",
        "caps=application/x-rtp,media=video,encoding-name=H264,"
        f"payload={int(payload_type)},clock-rate=90000",
        "!",
        "rtpjitterbuffer",
        "latency=50",
        "drop-on-latency=true",
        "!",
        "rtph264depay",
        "!",
        "h264parse",
        "config-interval=-1",
        "!",
        "matroskamux",
        "!",
        "filesink",
        f"location={path}",
        "sync=false",
    ]


class RtpMkvRecorder:
    """管理一个可收到 EOS 并安全封装 MKV 的 GStreamer 子进程。"""

    def __init__(
        self,
        session_directory: str | Path,
        *,
        source_port: int = 5702,
        payload_type: int = 96,
        stop_timeout_s: float = 8.0,
    ) -> None:
        self.session_directory = Path(session_directory).expanduser().resolve()
        self.partial_path = self.session_directory / "video_raw.partial.mkv"
        self.final_path = self.session_directory / "video_raw.mkv"
        self.log_path = self.session_directory / "logs" / "recorder.log"
        self.command = build_recording_pipeline(
            source_port=source_port,
            payload_type=payload_type,
            output_path=self.partial_path,
        )
        self.stop_timeout_s = float(stop_timeout_s)
        self.process: subprocess.Popen[bytes] | None = None
        self._log_stream = None
        self._last_size = 0
        self._last_growth_at: float | None = None
        self._finalized = False

    def start(self) -> None:
        """启动录像；不在此时假设已经收到视频。"""

        if self.process is not None:
            raise DatasetRecordingError("录像器不能重复启动")
        if shutil.which("gst-launch-1.0") is None:
            raise DatasetRecordingError("未找到 gst-launch-1.0")
        self.session_directory.mkdir(parents=True, exist_ok=True)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        if self.partial_path.exists() or self.final_path.exists():
            raise DatasetRecordingError("当前会话目录已存在录像文件")
        self._log_stream = self.log_path.open("wb")
        self.process = subprocess.Popen(
            self.command,
            stdout=self._log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self._last_growth_at = time.monotonic()

    def observe_progress(self, now: float | None = None) -> bool:
        """观察文件是否继续增长，增长时返回真。"""

        current_time = time.monotonic() if now is None else float(now)
        size = self.partial_path.stat().st_size if self.partial_path.exists() else 0
        if size > self._last_size:
            self._last_size = size
            self._last_growth_at = current_time
            return True
        return False

    def wait_until_receiving(
        self,
        timeout_s: float = 8.0,
        pump: Callable[[], None] | None = None,
    ) -> None:
        """在解锁前确认文件真正收到视频数据。"""

        if self.process is None:
            raise DatasetRecordingError("录像器尚未启动")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise DatasetRecordingError(
                    f"录像进程提前退出（{self.process.returncode}），请查看 {self.log_path}"
                )
            self.observe_progress()
            if self._last_size >= 4096:
                return
            if pump is not None:
                pump()
            time.sleep(0.05)
        raise DatasetRecordingError("视频文件 8 秒内没有增长，禁止解锁")

    def is_stream_fresh(self, maximum_idle_s: float = 3.0) -> bool:
        """检查录像进程存活且近期仍有新数据写入。"""

        if self.process is None or self.process.poll() is not None:
            return False
        now = time.monotonic()
        self.observe_progress(now)
        return (
            self._last_growth_at is not None
            and now - self._last_growth_at <= maximum_idle_s
        )

    def finalize(
        self, pump: Callable[[], None] | None = None
    ) -> tuple[Path | None, str | None]:
        """发送 SIGINT/EOS，验证 MKV，成功时去掉 partial 后缀。

        ``pump`` 用于飞控已解锁时继续发送中位并处理 ROS
        回调，避免等待封装的几秒内触发命令超时。
        """

        if self._finalized:
            return (self.final_path if self.final_path.is_file() else None, None)
        self._finalized = True
        error: str | None = None
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + self.stop_timeout_s
            while self.process.poll() is None and time.monotonic() < deadline:
                if pump is not None:
                    try:
                        pump()
                    except Exception as exc:
                        # 飞控健康检查比录像封装优先。回调报错时
                        # 立即停止录像子进程，让上层进入锁存急停。
                        self.process.terminate()
                        try:
                            self.process.wait(timeout=1.0)
                        except subprocess.TimeoutExpired:
                            self.process.kill()
                            self.process.wait(timeout=1.0)
                        error = f"录像封装期间控制链异常: {exc}"
                        break
                time.sleep(0.05)
            if error is None and self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=1.0)
                error = "录像器未在 EOS 超时内退出，已强制结束"
            elif error is None and self.process.returncode != 0:
                error = f"录像器退出码为 {self.process.returncode}"
        elif self.process is not None:
            # 第一次 finalize 时录像器已经消失，说明它在操作员
            # 按 0 之前就退出了。即使残留文件刚好能被 ffprobe
            # 打开，也不能将这次中断伪装成正常录像。
            error = f"录像器已提前退出（退出码 {self.process.returncode}）"

        if self._log_stream is not None:
            self._log_stream.close()
            self._log_stream = None

        if error is None:
            try:
                self._verify_partial_file()
                self.partial_path.replace(self.final_path)
            except (DatasetRecordingError, OSError) as exc:
                error = str(exc)
        return (self.final_path if error is None else None, error)

    def _verify_partial_file(self) -> None:
        """使用 ffprobe 确认封装后的文件包含 H.264 视频流。"""

        if not self.partial_path.is_file() or self.partial_path.stat().st_size < 4096:
            raise DatasetRecordingError("录像文件不存在或过小，保留 partial 文件")
        ffprobe = shutil.which("ffprobe")
        if ffprobe is None:
            raise DatasetRecordingError("未找到 ffprobe，无法验证录像")
        result = subprocess.run(
            [
                ffprobe,
                "-v",
                "error",
                "-select_streams",
                "v:0",
                "-show_entries",
                "stream=codec_name",
                "-of",
                "default=noprint_wrappers=1:nokey=1",
                str(self.partial_path),
            ],
            capture_output=True,
            text=True,
            timeout=8.0,
            check=False,
        )
        if result.returncode != 0 or "h264" not in result.stdout.lower():
            detail = result.stderr.strip() or result.stdout.strip() or "未识别到 H.264"
            raise DatasetRecordingError(f"ffprobe 录像验证失败: {detail}")


def _sha256(path: Path) -> str | None:
    """计算小型配置文件的 SHA-256；不存在时返回空。"""

    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git_commit(project_directory: Path) -> str | None:
    """读取当前 Git 提交，不修改仓库。"""

    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=project_directory,
        capture_output=True,
        text=True,
        check=False,
    )
    return result.stdout.strip() if result.returncode == 0 else None


class DatasetSessionLogger:
    """同步保存控制样本、按键事件和会话元数据。"""

    CSV_FIELDS = (
        "utc_time",
        "monotonic_s",
        "event",
        "keys",
        "forward",
        "lateral",
        "vertical",
        "yaw",
        "depth_m",
        "yaw_deg",
        "flight_mode",
        "armed",
        "message",
    )

    def __init__(
        self,
        session_directory: str | Path,
        *,
        project_directory: str | Path,
        robot_config_path: str | Path,
        dataset_config_path: str | Path,
    ) -> None:
        self.session_directory = Path(session_directory).expanduser().resolve()
        self.session_directory.mkdir(parents=True, exist_ok=True)
        (self.session_directory / "logs").mkdir(exist_ok=True)
        self.csv_path = self.session_directory / "events.csv"
        self.json_path = self.session_directory / "session.json"
        self._csv_stream = self.csv_path.open("w", encoding="utf-8", newline="")
        self._writer = csv.DictWriter(self._csv_stream, fieldnames=self.CSV_FIELDS)
        self._writer.writeheader()
        project = Path(project_directory).expanduser().resolve()
        robot_path = Path(robot_config_path).expanduser().resolve()
        dataset_path = Path(dataset_config_path).expanduser().resolve()
        self.metadata: dict[str, object] = {
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "finished_at_utc": None,
            "outcome": "running",
            "detail": "",
            "git_commit": _git_commit(project),
            "robot_config": str(robot_path),
            "robot_config_sha256": _sha256(robot_path),
            "dataset_config": str(dataset_path),
            "dataset_config_sha256": _sha256(dataset_path),
            "start_depth_m": None,
            "effective_depth_limit_m": None,
            "video_file": None,
        }
        self._write_json()

    def set_depths(self, start_depth_m: float, effective_limit_m: float) -> None:
        """记录启动深度和本次实际生效的深度上限。"""

        self.metadata["start_depth_m"] = float(start_depth_m)
        self.metadata["effective_depth_limit_m"] = float(effective_limit_m)
        self._write_json()

    def write(
        self,
        *,
        event: str,
        keys: Iterable[str],
        motion: MotionCommand,
        depth_m: float | None,
        yaw_deg: float | None,
        flight_mode: str,
        armed: bool,
        message: str = "",
    ) -> None:
        """写入一行可与视频时间对照的操作证据。"""

        def finite_or_blank(value: float | None) -> str:
            if value is None or not math.isfinite(float(value)):
                return ""
            return f"{float(value):.6f}"

        self._writer.writerow(
            {
                "utc_time": datetime.now(timezone.utc).isoformat(),
                "monotonic_s": f"{time.monotonic():.6f}",
                "event": event,
                "keys": "+".join(sorted(str(key) for key in keys)),
                "forward": f"{motion.forward:.6f}",
                "lateral": f"{motion.lateral:.6f}",
                "vertical": f"{motion.vertical:.6f}",
                "yaw": f"{motion.yaw:.6f}",
                "depth_m": finite_or_blank(depth_m),
                "yaw_deg": finite_or_blank(yaw_deg),
                "flight_mode": flight_mode,
                "armed": str(bool(armed)).lower(),
                "message": message,
            }
        )
        self._csv_stream.flush()

    def finish(
        self,
        *,
        outcome: str,
        detail: str,
        video_path: Path | None,
    ) -> None:
        """完成 CSV 和 JSON；多次调用也不会重复写入。"""

        if self._csv_stream.closed:
            return
        self.metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        self.metadata["outcome"] = outcome
        self.metadata["detail"] = detail
        self.metadata["video_file"] = str(video_path) if video_path else None
        self._write_json()
        self._csv_stream.close()

    def _write_json(self) -> None:
        """通过临时文件原子替换，避免突然断电留下半个 JSON。"""

        temporary = self.json_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.json_path)


class VideoOnlySession:
    """记录“QGC 手柄驾驶，本程序只录像”的最小会话信息。"""

    def __init__(
        self,
        session_directory: str | Path,
        *,
        project_directory: str | Path,
    ) -> None:
        self.session_directory = Path(session_directory).expanduser().resolve()
        self.session_directory.mkdir(parents=True, exist_ok=True)
        (self.session_directory / "logs").mkdir(exist_ok=True)
        self.json_path = self.session_directory / "session.json"
        project = Path(project_directory).expanduser().resolve()
        self.metadata: dict[str, object] = {
            "mode": "qgc_gamepad_video_only",
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "finished_at_utc": None,
            "outcome": "recording",
            "detail": "",
            "git_commit": _git_commit(project),
            "video_file": None,
            "control_note": (
                "本程序未连接 MAVLink，未观察或记录 QGC 手柄控制量"
            ),
        }
        self._write_json()

    def finish(
        self,
        *,
        outcome: str,
        detail: str,
        video_path: Path | None,
    ) -> None:
        """写入录像结果，不伪造手柄或飞控数据。"""

        self.metadata["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        self.metadata["outcome"] = outcome
        self.metadata["detail"] = detail
        self.metadata["video_file"] = str(video_path) if video_path else None
        self._write_json()

    def _write_json(self) -> None:
        """原子更新会话 JSON。"""

        temporary = self.json_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.json_path)


def build_video_only_parser() -> argparse.ArgumentParser:
    """创建不包含任何飞控参数的录像命令解析器。"""

    parser = argparse.ArgumentParser(
        description="只录制原始 RTP/H.264；不连 MAVLink，不控制 ROV。"
    )
    parser.add_argument("--session-dir", required=True)
    parser.add_argument("--project-dir", default=str(Path.cwd()))
    parser.add_argument("--record-port", type=int, default=5702)
    parser.add_argument("--payload-type", type=int, default=96)
    return parser


def video_only_main(argv: list[str] | None = None) -> int:
    """开始录像，按 Enter 或 Ctrl+C 后用 EOS 正常封装。"""

    args = build_video_only_parser().parse_args(argv)
    if not sys.stdin.isatty():
        print("只录像工具必须在交互终端运行。", file=sys.stderr)
        return 2

    session_directory = Path(args.session_dir).expanduser().resolve()
    session = VideoOnlySession(
        session_directory,
        project_directory=args.project_dir,
    )
    recorder = RtpMkvRecorder(
        session_directory,
        source_port=args.record_port,
        payload_type=args.payload_type,
    )
    detail = "录像尚未开始"
    outcome = "failed"
    result_code = 2
    video_path: Path | None = None

    try:
        recorder.start()
        recorder.wait_until_receiving(timeout_s=8.0)
        print("\n录像已开始；QGC 和手柄仍由操作员独立使用。")
        print("采集完成后，回到本终端按 Enter 停止录像。")
        print("Ctrl+C 也只会停止录像，不会向飞控发送命令。")
        while True:
            if not recorder.is_stream_fresh(maximum_idle_s=3.0):
                raise DatasetRecordingError("原始视频超过 3 秒未继续写入")
            ready, _, _ = select.select([sys.stdin], [], [], 0.25)
            if ready:
                sys.stdin.readline()
                detail = "操作员按 Enter 停止录像"
                outcome = "completed"
                result_code = 0
                break
    except KeyboardInterrupt:
        detail = "操作员按 Ctrl+C 停止录像"
        outcome = "completed"
        result_code = 0
        print()
    except (DatasetRecordingError, OSError, ValueError) as exc:
        detail = str(exc)
        outcome = "failed"
        result_code = 2
        print(f"\n录像中止: {detail}", file=sys.stderr)
    finally:
        video_path, finalize_error = recorder.finalize()
        if finalize_error is not None:
            detail = f"{detail}；封装验证失败: {finalize_error}"
            outcome = "failed"
            result_code = 2
        session.finish(
            outcome=outcome,
            detail=detail,
            video_path=video_path,
        )

    print(f"会话目录: {session_directory}")
    if video_path is not None:
        print(f"完整录像: {video_path}")
    else:
        print("录像未通过验证；如有残留数据将保留 partial 文件。")
    return result_code


if __name__ == "__main__":
    raise SystemExit(video_only_main())
