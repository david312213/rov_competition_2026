"""与独立操作窗口进行有界、非阻塞通信；不依赖 ROS/Pygame，不重发运动。"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time


WINDOW_HOLD_S = 1.0
WINDOW_ABORT_S = 3.0
MAX_PACKET_BYTES = 4096


def send_packet(channel: socket.socket, packet: dict) -> bool:
    payload = json.dumps(packet, ensure_ascii=False, allow_nan=False).encode("utf-8")
    if len(payload) > MAX_PACKET_BYTES:
        raise ValueError("窗口消息超长")
    try:
        channel.send(payload)
        return True
    except BlockingIOError:
        return False  # display state / latched input is sent again, never block control


def receive_packets(channel: socket.socket) -> list[dict]:
    packets = []
    for _ in range(32):  # bounded work even if the peer is noisy
        try:
            payload = channel.recv(MAX_PACKET_BYTES + 1)
        except BlockingIOError:
            break
        if not payload or len(payload) > MAX_PACKET_BYTES:
            continue
        try:
            packet = json.loads(payload)
        except (ValueError, UnicodeDecodeError):
            continue
        if isinstance(packet, dict):
            packets.append(packet)
    return packets


@dataclass(frozen=True)
class WindowInput:
    sent_at: float
    focused: bool
    paused: bool
    enter_seq: int
    finish_requested: bool
    estop: bool
    reason: str

    @classmethod
    def parse(cls, packet: dict, now: float) -> WindowInput | None:
        if packet.get("kind") != "input":
            return None
        stamp = packet.get("sent_at")
        seq = packet.get("enter_seq")
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or stamp > now + 0.1 or type(seq) is not int or seq < 0):
            return None
        if any(type(packet.get(key)) is not bool for key in
               ("focused", "paused", "finish_requested", "estop")):
            return None
        reason = packet.get("reason", "")
        if not isinstance(reason, str):
            return None
        return cls(float(stamp), packet["focused"], packet["paused"], seq,
                   packet["finish_requested"], packet["estop"], reason[:160])


@dataclass(frozen=True)
class WindowAction:
    pause_reason: str = ""
    resume: bool = False
    finish: bool = False
    error: str = ""


class WindowGate:
    """短暂绘制抖动不拖住控制；窗口持续无响应则回中，失联则急停。"""

    def __init__(self, enter_seq: int = 0) -> None:
        self.last_enter_seq = enter_seq

    def evaluate(self, state: WindowInput | None, *, now: float, alive: bool) -> WindowAction:
        if not alive:
            return WindowAction(error="控制窗口被关闭或进程退出")
        if state is None:
            return WindowAction(error="控制窗口状态缺失")
        fresh_enter = state.enter_seq > self.last_enter_seq
        self.last_enter_seq = max(self.last_enter_seq, state.enter_seq)
        if state.estop:
            return WindowAction(error=state.reason or "Esc/关闭窗口急停")
        age = now - state.sent_at
        if age < -0.1 or age > WINDOW_ABORT_S:
            return WindowAction(error="控制窗口无响应超过 3 秒，急停")
        if age > WINDOW_HOLD_S:
            return WindowAction(pause_reason="控制窗口无响应，已回中；窗口恢复后按 Enter")
        if state.paused or not state.focused:
            return WindowAction(pause_reason=state.reason or "控制窗口失去键盘焦点",
                                finish=state.finish_requested)
        return WindowAction(resume=fresh_enter, finish=state.finish_requested)


class ClusterWindowBridge:
    def __init__(self, log_path: Path) -> None:
        # Private inherited descriptor; no open TCP/UDP control port.
        self.channel, peer = socket.socketpair(type=socket.SOCK_DGRAM)
        self.channel.setblocking(False)
        peer.setblocking(False)
        self.state: WindowInput | None = None
        self.created_at = time.monotonic()
        self.process = None
        environment = dict(os.environ)
        # A source-tree test/import may have changed sys.path without exporting it.
        # The child must load the same package version as this parent, not another install.
        package_root = str(Path(__file__).resolve().parents[1])
        environment["PYTHONPATH"] = os.pathsep.join(
            part for part in (package_root, environment.get("PYTHONPATH", "")) if part
        )
        try:
            with log_path.open("a", encoding="utf-8") as log:
                self.process = subprocess.Popen(
                    [sys.executable, "-m", "rov_competition.cluster_window",
                     "--ipc-fd", str(peer.fileno()), "--parent-pid", str(os.getpid())],
                    pass_fds=(peer.fileno(),), stdout=log, stderr=subprocess.STDOUT,
                    env=environment,
                )
        except BaseException:
            self.channel.close()
            raise
        finally:
            peer.close()

    @property
    def alive(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def poll(self) -> WindowInput | None:
        now = time.monotonic()
        for packet in receive_packets(self.channel):
            state = WindowInput.parse(packet, now)
            if state is not None and (self.state is None or state.sent_at >= self.state.sent_at):
                self.state = state
        return self.state

    def show(self, lines: list[str]) -> None:
        send_packet(self.channel, {"kind": "view", "sent_at": time.monotonic(),
                                   "lines": [line[:150] for line in lines[:6]]})

    def close(self) -> None:
        try:
            if self.alive:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2.0)
        finally:
            self.channel.close()
