"""群体测试的纯 Python 窗口门控与循环耗时诊断；不发送运动命令。"""

from __future__ import annotations

import time


def focus_pause_reason(pygame: object, event: object) -> str | None:
    """鼠标进出不是键盘失焦；保留切窗、最小化时的回中保护。"""

    if event.type == getattr(pygame, "WINDOWFOCUSLOST", -1):
        return "控制窗口失去键盘焦点"
    if event.type in {
        getattr(pygame, "WINDOWMINIMIZED", -2),
        getattr(pygame, "WINDOWHIDDEN", -3),
    }:
        return "控制窗口被最小化或隐藏"
    if event.type == pygame.ACTIVEEVENT and getattr(event, "gain", 1) == 0:
        # SDL legacy bitmask: mouse=1, keyboard=2, active/visible=4.
        state = getattr(event, "state", None)
        if state is None:
            return "无法确认窗口焦点状态"
        if state & 6:
            return "控制窗口失去键盘焦点或被最小化"
    return None


class LoopTiming:
    """只诊断、不延长看门狗，也不重复发送旧运动指令。"""

    def __init__(self, *, clock=time.monotonic) -> None:
        self.clock = clock
        self.last_warning_at = float("-inf")
        self.begin()

    def begin(self) -> None:
        self.started_at = self.marked_at = self.clock()
        self.stages: dict[str, float] = {}

    def mark(self, name: str) -> None:
        now = self.clock()
        self.stages[name] = now - self.marked_at
        self.marked_at = now

    def warning(self) -> str | None:
        now = self.clock()
        total = now - self.started_at
        if total < 0.20 or now - self.last_warning_at < 2.0:
            return None
        self.last_warning_at = now
        stages = ", ".join(f"{name}={value:.3f}s" for name, value in self.stages.items())
        return f"[控制循环耗时] total={total:.3f}s; {stages}; 运动看门狗未放宽"
