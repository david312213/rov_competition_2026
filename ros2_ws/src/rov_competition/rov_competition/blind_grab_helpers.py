"""独立视频/感知辅助进程；失败和退出均不会向主控制发送终止信号。"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Any

from .blind_grab_config import VisionSettings, package_config_path


@dataclass
class HelperProcess:
    name: str
    command: Callable[[int], list[str]]
    restart: bool = True
    process: Any = None
    log: Any = None
    attempts: int = 0
    next_start_at: float = 0.0
    finished: bool = False


def prepare_helper_processes(settings: VisionSettings, directory: Path) -> list[HelperProcess]:
    """制作本次感知配置；仅调整计数相关阈值，模型路径保持指向原权重。"""
    import yaml

    directory.mkdir(parents=True, exist_ok=True)
    (directory / "logs").mkdir(exist_ok=True)
    source = settings.autonomy_config or package_config_path("autonomy.yaml")
    content = yaml.safe_load(source.read_text(encoding="utf-8"))
    detector = content["detector"]
    detector["confidence_threshold"] = settings.confidence
    model = Path(detector["model_path"]).expanduser()
    detector["model_path"] = str(model if model.is_absolute() else (source.parent / model).resolve())
    autonomy_path = directory / "perception_autonomy.yaml"
    autonomy_path.write_text(yaml.safe_dump(content, allow_unicode=True, sort_keys=False), encoding="utf-8")
    targets_path = directory / "perception_targets.yaml"
    targets_path.write_text(yaml.safe_dump({
        "graspable_labels": list(settings.target_labels),
        "display_names": {name: name for name in settings.target_labels},
        "unsupported_scoring_targets": [],
    }, allow_unicode=True, sort_keys=False), encoding="utf-8")
    robot = settings.robot_config or package_config_path("robot.example.yaml")
    bridge = [
        "ros2", "run", "rov_competition", "rov_stream_bridge",
        "--source-port", str(settings.source_port), "--no-qgc",
        "--inference-host", "127.0.0.1", "--inference-port", str(settings.inference_port),
        "--record-host", "127.0.0.1", "--record-port", str(settings.record_port),
        "--no-display",
    ]
    perception = [
        "ros2", "launch", "rov_competition", "perception_only.launch.py",
        f"robot_config:={robot}", f"autonomy_config:={autonomy_path}",
        f"targets_config:={targets_path}", f"ai_port:={settings.inference_port}",
    ]
    processes = [
        HelperProcess("video_bridge", lambda attempt: list(bridge)),
        HelperProcess("perception", lambda attempt: list(perception)),
    ]
    if settings.record_video:
        from .dataset_recording import build_recording_pipeline
        processes.append(HelperProcess("recorder", lambda attempt: build_recording_pipeline(
            source_port=settings.record_port, payload_type=96,
            output_path=directory / f"video_{attempt:04d}.mkv",
        )))
    if settings.show_viewer:
        processes.append(HelperProcess("viewer", lambda attempt: [
            "ros2", "run", "rqt_image_view", "rqt_image_view",
            "/rov/annotated_image compressed",
        ], restart=False))
    return processes


class OptionalHelpers:
    """辅助进程在自己的线程中启动/重试，不等待第一帧、视频或录像数据。"""

    def __init__(
        self, settings: VisionSettings, directory: Path, stop_event: threading.Event,
        *, popen: Callable[..., Any] = subprocess.Popen,
        report: Callable[[str], None] = print,
    ) -> None:
        self.settings = settings
        self.directory = directory
        self.stop_event = stop_event
        self._popen = popen
        self._report = report
        self.processes: list[HelperProcess] = []
        self.thread: threading.Thread | None = None

    def start(self) -> None:
        if not self.settings.start_helpers:
            return
        self.thread = threading.Thread(target=self._run, name="blind-grab-helpers", daemon=True)
        self.thread.start()

    def _say(self, text: str) -> None:
        try:
            self._report(f"[盲抓辅助] {text}；主控制继续")
        except Exception:
            pass

    def tick(self, now: float) -> None:
        for child in self.processes:
            if self.stop_event.is_set():
                return
            if child.process is not None:
                try:
                    code = child.process.poll()
                except Exception as exc:
                    self._say(f"读取 {child.name} 状态失败: {exc}")
                    continue
                if code is None:
                    continue
                self._say(f"{child.name} 已退出，状态码 {code}")
                child.process = None
                if child.log is not None:
                    child.log.close()
                    child.log = None
                child.next_start_at = now + 5.0
                child.finished = not child.restart
            if child.finished or now < child.next_start_at:
                continue
            child.attempts += 1
            try:
                child.log = (self.directory / "logs" / f"{child.name}.log").open("ab")
                child.process = self._popen(
                    child.command(child.attempts), stdin=subprocess.DEVNULL,
                    stdout=child.log, stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except Exception as exc:
                if child.log is not None:
                    child.log.close()
                    child.log = None
                child.next_start_at = now + 5.0
                child.finished = not child.restart
                self._say(f"{child.name} 启动失败: {type(exc).__name__}: {exc}")

    def _run(self) -> None:
        try:
            while not self.stop_event.is_set():
                if not self.processes:
                    try:
                        self.processes = prepare_helper_processes(self.settings, self.directory)
                    except Exception as exc:
                        self._say(f"感知配置暂不可用: {type(exc).__name__}: {exc}")
                        self.stop_event.wait(5.0)
                        continue
                try:
                    self.tick(time.monotonic())
                except Exception as exc:
                    self._say(f"辅助进程管理异常: {type(exc).__name__}: {exc}")
                self.stop_event.wait(0.25)
        finally:
            self._close_owned_processes()

    def _close_owned_processes(self) -> None:
        running = [c for c in self.processes if c.process is not None]
        # SIGINT 让 GStreamer -e 封装录像；只处理本入口创建的进程组。
        for child in running:
            try:
                if child.process.poll() is None:
                    os.killpg(child.process.pid, signal.SIGINT)
            except (OSError, ProcessLookupError):
                pass
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and any(c.process.poll() is None for c in running):
            time.sleep(0.05)
        for child in running:
            try:
                if child.process.poll() is None:
                    os.killpg(child.process.pid, signal.SIGTERM)
                child.process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(child.process.pid, signal.SIGKILL)
                    child.process.wait(timeout=0.5)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            except OSError:
                pass
            if child.log is not None:
                child.log.close()
                child.log = None

    def join(self) -> None:
        if self.thread is not None:
            self.thread.join(timeout=6.0)
