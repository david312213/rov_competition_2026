"""独立 Pygame 操作窗口：只处理显示和按键，不连接 ROS/飞控或发送运动指令。"""

from __future__ import annotations

import argparse
import os
import socket
import time

from .cluster_runtime_support import focus_pause_reason
from .cluster_window_bridge import receive_packets, send_packet


def apply_input(pygame, events, state: dict, *, focused: bool) -> None:
    """所有停止意图均锁存在状态里，发送缓冲满时也不会丢失。"""
    for event in events:
        if event.type == pygame.QUIT:
            state.update(estop=True, reason="控制窗口被关闭")
        reason = focus_pause_reason(pygame, event)
        if reason is not None:
            state.update(paused=True, reason=reason)
        if event.type != pygame.KEYDOWN:
            continue
        if event.key == pygame.K_ESCAPE:
            state.update(estop=True, reason="Esc 急停")
        elif event.key == pygame.K_SPACE:
            state.update(paused=True, reason="操作员按 Space")
        elif event.key in {pygame.K_0, pygame.K_KP0}:
            state["finish_requested"] = True
        elif event.key in {pygame.K_RETURN, pygame.K_KP_ENTER} and focused:
            state.update(paused=False, reason="", enter_seq=state["enter_seq"] + 1)
    state["focused"] = bool(focused)
    if not focused:
        state.update(paused=True, reason=state["reason"] or "控制窗口失去键盘焦点")


def draw_window(pygame, screen, font, lines: list[str]) -> None:
    screen.fill((17, 24, 39))
    for index, line in enumerate(lines[:6]):
        color = (255, 205, 80) if index == len(lines) - 1 else (235, 245, 255)
        screen.blit(font.render(line[:150], True, color), (20, 18 + index * 38))
    pygame.display.flip()


def run_window(channel, pygame, *, parent_alive, draw=draw_window) -> None:
    """允许注入慢 draw 做无硬件回归测试；这个进程永远没有运动发布器。"""
    pygame.init()
    try:
        screen = pygame.display.set_mode((1180, 285))
        pygame.display.set_caption("ROV Cluster Collection Test")
        font = pygame.font.Font(None, 27)
        state = dict(kind="input", sent_at=0.0, focused=False, paused=True,
                     enter_seq=0, finish_requested=False, estop=False, reason="等待 Enter 开始")
        lines = ["DISARMED - WAITING FOR START", "Click this window, then press ENTER to start.",
                 "Mouse leave OK; keyboard focus loss/minimize pauses.",
                 "SPACE pause | ENTER resume | 0 return+disarm | ESC/close emergency stop"]
        view_at = time.monotonic()
        next_draw = 0.0
        while parent_alive():
            started = time.monotonic()
            events = pygame.event.get()
            apply_input(pygame, events, state, focused=pygame.key.get_focused())
            state["sent_at"] = time.monotonic()
            send_packet(channel, state)
            if state["estop"]:
                # If the packet was dropped, process exit is independently fail-safe.
                return
            for packet in receive_packets(channel):
                if packet.get("kind") == "view" and isinstance(packet.get("lines"), list):
                    lines = [str(line)[:150] for line in packet["lines"][:6]]
                    view_at = time.monotonic()
            if time.monotonic() >= next_draw:
                displayed = list(lines)
                if time.monotonic() - view_at > 1.0:
                    displayed = ["CONTROL STATUS STALE - CHECK TERMINAL / QGC", *lines[:5]]
                draw(pygame, screen, font, displayed)
                next_draw = time.monotonic() + 0.10
            time.sleep(max(0.0, 0.05 - (time.monotonic() - started)))
    finally:
        pygame.quit()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ipc-fd", type=int, required=True)
    parser.add_argument("--parent-pid", type=int, required=True)
    args = parser.parse_args()
    import pygame
    channel = socket.socket(fileno=args.ipc_fd)
    channel.setblocking(False)
    try:
        run_window(channel, pygame, parent_alive=lambda: os.getppid() == args.parent_pid)
    finally:
        channel.close()


if __name__ == "__main__":
    main()
