"""持续盲抓入口：控制、MAVLink I/O、ROS 检测和辅助进程各自运行。"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import asdict, replace
from datetime import datetime
import json
from pathlib import Path
import signal
import threading
import time
from typing import Any
from uuid import uuid4

from .blind_grab import BlindGrabMission, DetectionCount
from .blind_grab_config import (
    BlindGrabConfigurationError, VisionSettings, load_blind_grab_config, package_config_path,
)
from .blind_grab_helpers import OptionalHelpers
from .blind_grab_mavlink import BlindMavlinkOutput
from .domain import MotionCommand


CONTROL_PERIOD_S = 0.05


def say(message: str) -> None:
    try:
        print(message, flush=True)
    except (OSError, ValueError):
        # 关闭查看日志的管道不会取消抓取。
        pass


class DetectionBuffer:
    """保存最后一个唯一检测帧；不把重复 ROS 消息刷新成新图。"""

    def __init__(self, settings: VisionSettings) -> None:
        self.settings = settings
        self._labels = frozenset(settings.target_labels)
        self._lock = threading.Lock()
        self._last_stamp: tuple[int, int] | None = None
        self._frame_id = 0
        self._latest: DetectionCount | None = None

    def accept(self, message: Any, now: float) -> bool:
        stamp = (int(message.stamp.sec), int(message.stamp.nanosec))
        count = sum(
            str(d.label) in self._labels and float(d.confidence) >= self.settings.confidence
            for d in message.detections
        )
        with self._lock:
            if self._last_stamp is not None and stamp <= self._last_stamp:
                return False
            self._last_stamp = stamp
            self._frame_id += 1
            self._latest = DetectionCount(self._frame_id, now, count)
        return True

    def latest(self) -> DetectionCount | None:
        with self._lock:
            return self._latest


class RosDetectionWorker:
    def __init__(self, buffer: DetectionBuffer, stop_event: threading.Event, report=say) -> None:
        self.buffer, self.stop_event, self.report = buffer, stop_event, report
        self.thread = threading.Thread(target=self._run, name="blind-grab-detections", daemon=True)
        self._next_callback_report_at = 0.0

    def start(self) -> None:
        self.thread.start()

    def _on_message(self, message: Any) -> None:
        now = time.monotonic()
        try:
            self.buffer.accept(message, now)
        except Exception as exc:
            if now >= self._next_callback_report_at:
                self._next_callback_report_at = now + 5.0
                self.report(f"[盲抓检测] 无法解析本帧: {exc}；找框计时继续")

    def _run(self) -> None:
        while not self.stop_event.is_set():
            node = context = executor = None
            try:
                # ROS 缺失、初始化失败或上下文关闭都只影响这个接收线程。
                import rclpy
                from rclpy.context import Context
                from rclpy.node import Node
                from rclpy.executors import SingleThreadedExecutor
                from rclpy.signals import SignalHandlerOptions
                from rov_interfaces.msg import TargetDetectionArray

                context = Context()
                rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
                node = Node("rov_blind_grab_detections", context=context)
                node.create_subscription(TargetDetectionArray, "/rov/detections", self._on_message, 1)
                executor = SingleThreadedExecutor(context=context)
                executor.add_node(node)
                while not self.stop_event.is_set() and context.ok():
                    executor.spin_once(timeout_sec=0.05)
                if not self.stop_event.is_set():
                    self.report("[盲抓检测] ROS 检测上下文已关闭，稍后重新连接；主控制继续")
            except Exception as exc:
                self.report(f"[盲抓检测] {type(exc).__name__}: {exc}；主控制继续，稍后重试")
            finally:
                if executor is not None:
                    try:
                        executor.shutdown(timeout_sec=0.2)
                    except Exception:
                        pass
                if node is not None:
                    try:
                        node.destroy_node()
                    except Exception:
                        pass
                if context is not None:
                    try:
                        context.try_shutdown()
                    except Exception:
                        pass
            self.stop_event.wait(5.0)

    def join(self) -> None:
        self.thread.join(timeout=1.0)


class MavlinkOutputWorker:
    """连接建立/发送异常不会阻塞状态机的30秒计时。"""

    def __init__(self, output: BlindMavlinkOutput, stop_event: threading.Event, report=say) -> None:
        self.output, self.stop_event, self.report = output, stop_event, report
        self._lock = threading.Lock()
        self._desired = (MotionCommand.neutral(), ())
        self.thread = threading.Thread(target=self._run, name="blind-grab-mavlink", daemon=True)

    def start(self) -> None:
        self.thread.start()

    def set_desired(self, motion: MotionCommand, servos: tuple) -> None:
        with self._lock:
            self._desired = (motion, servos)

    def _run(self) -> None:
        next_report = 0.0
        try:
            while not self.stop_event.is_set():
                started = time.monotonic()
                with self._lock:
                    motion, servos = self._desired
                if self.stop_event.is_set():
                    break
                try:
                    self.output.publish(motion, servos, started)
                except Exception as exc:
                    if started >= next_report:
                        next_report = started + 2.0
                        self.report(f"[盲抓输出] {type(exc).__name__}: {exc}；继续重试")
                self.stop_event.wait(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))
        finally:
            for error in self.output.shutdown():
                self.report(f"[手动关闭] {error}")

    def join(self) -> None:
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            self.report("[手动关闭] 通信线程尚未返回，进程退出将关闭连接；未确认归中报文发送")


def run_control_loop(
    mission: BlindGrabMission, source: Any, output: Any, stop_event: Any,
    *, clock: Callable[[], float] = time.monotonic,
    report: Callable[[str], None] = say,
) -> None:
    """可注入时钟、检测源和输出的20Hz主循环；不会等待辅助进程。"""
    next_report_at = 0.0
    next_error_report_at = 0.0
    previous_key = None
    while not stop_event.is_set():
        now = clock()
        try:
            frame = source.latest()
        except Exception as exc:
            frame = None
            if now >= next_error_report_at:
                report(f"[盲抓检测] {exc}；按未收到新帧继续")
                next_error_report_at = now + 2.0
        decision = mission.step(now, frame)
        if stop_event.is_set():
            break
        try:
            output.set_desired(decision.motion, decision.servos)
        except Exception as exc:
            if now >= next_error_report_at:
                report(f"[盲抓输出] {exc}；下个控制周期继续")
                next_error_report_at = now + 2.0
        key = (decision.state, decision.phase, decision.permanent, decision.batch_index, decision.grab_in_batch)
        if now >= next_report_at or key != previous_key:
            next_report_at, previous_key = now + 1.0, key
            report(
                f"[盲抓] state={decision.state.value} phase={decision.phase} "
                f"boxes={decision.box_count} threshold={decision.threshold} "
                f"permanent={str(decision.permanent).lower()} "
                f"search_wait={decision.search_elapsed_s:.2f}s lane={decision.lane_index + 1} "
                f"batch={decision.batch_index} grab={decision.grab_in_batch}/{mission.config.grabs_per_batch} "
                f"close_commands={decision.grasp_command_count} cycles={decision.completed_cycles}"
            )
        stop_event.wait(max(0.0, CONTROL_PERIOD_S - (clock() - now)))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="启动即执行的持续盲抓；4框确认，找框30秒后永久盲抓，Ctrl+C关闭。")
    parser.add_argument("--config", default=str(package_config_path("blind_grab.yaml")), help="已填写的盲抓参数文件")
    parser.add_argument("--no-helpers", action="store_true", help="不启动视频/YOLO/录像/查看器；仍接收已有 /rov/detections")
    parser.add_argument("--output-dir", default="output/blind_grab_sessions", help="辅助进程日志和可选录像的目录")
    parser.add_argument("--dry-run", action="store_true", help="只解析并显示配置，不连接MAVLink、不初始化ROS、不启动辅助进程")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_blind_grab_config(args.config)
    except (BlindGrabConfigurationError, ImportError) as exc:
        say(str(exc))
        return 2
    if args.no_helpers:
        config = replace(config, vision=replace(config.vision, start_helpers=False))
    if args.dry_run:
        say(json.dumps(asdict(config), default=str, ensure_ascii=False, indent=2))
        return 0

    stop_event = threading.Event()
    previous_handlers = {}

    def request_stop(signum: int, frame: Any) -> None:
        stop_event.set()

    handled_signals = (signal.SIGINT, signal.SIGTERM)
    if hasattr(signal, "SIGHUP"):
        handled_signals += (signal.SIGHUP,)
    for signum in handled_signals:
        previous_handlers[signum] = signal.signal(signum, request_stop)
    mission = BlindGrabMission(config.mission)
    buffer = DetectionBuffer(config.vision)
    detector = RosDetectionWorker(buffer, stop_event)
    transport = BlindMavlinkOutput(config.mavlink, report=say)
    output = MavlinkOutputWorker(transport, stop_event)
    directory = Path(args.output_dir).expanduser().resolve() / (
        f"{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}"
    )
    helpers = OptionalHelpers(config.vision, directory, stop_event, report=say)
    say("开始持续盲抓控制：直接搜索；不自动设置模式或解锁。Ctrl+C或SIGTERM关闭。")
    say(f"配置: {config.source_path}；辅助日志: {directory}")
    try:
        # 启动线程只派发工作，不等待ROS、模型、相机、录像或连接就绪。
        output.start()
        detector.start()
        helpers.start()
        run_control_loop(mission, buffer, output, stop_event)
    finally:
        stop_event.set()
        # 先归中/释放运动，再清理耗时的视觉和录像进程。
        output.join()
        detector.join()
        helpers.join()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    say(f"已关闭。安排闭爪指令 {mission.grasp_command_count} 次，完成动作循环 {mission.completed_cycles} 次；这些不是实际收获数。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
