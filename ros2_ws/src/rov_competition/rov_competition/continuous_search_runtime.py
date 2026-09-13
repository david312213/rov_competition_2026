"""连续自动蛇形搜寻的 ROS 运行时。

只负责触底、离底定高、定时蛇形搜索和稳定发现目标后的停车；不调用抓取、
机械爪或原有群体盲抓流程。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
import yaml

from .config import (
    ConfigurationError,
    ControlProfile,
    load_autonomy_config,
    load_dataset_config,
    load_robot_config,
)
from .dataset_drive import (
    DatasetDriveError,
    _active_error,
    _prearm_error,
    _wait_for_gateway_state,
)
from .dataset_recording import RtpMkvRecorder
from .domain import MotionCommand
from .search_approach_runtime import SearchApproachNode, _package_config_path
from .semicircle_search import SearchAction, SearchConfig, SearchState, SemicircleSearchMission


CONFIRMATION = "START CONTINUOUS SEARCH"


def _default_search_config_path() -> str:
    return _package_config_path("continuous_search.yaml")


def _load_config(path: str, *, lane_seconds: float, surface_minutes: float) -> SearchConfig:
    try:
        root = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        values = dict(root["continuous_search"])
    except (OSError, TypeError, ValueError, KeyError) as exc:
        raise ValueError(f"连续搜寻配置读取失败: {exc}") from exc
    values["lane_forward_duration_s"] = lane_seconds
    values["search_duration_s"] = surface_minutes * 60.0
    try:
        return SearchConfig(**values)
    except TypeError as exc:
        raise ValueError(f"连续搜寻配置字段不完整: {exc}") from exc


def _positive_input(prompt: str, supplied: float | None) -> float:
    value = supplied if supplied is not None else float(input(prompt).strip())
    if not math.isfinite(value) or value <= 0:
        raise ValueError("必须输入正的有限数")
    return value


def _event_action(pygame: object, key: int) -> SearchAction | None:
    return {
        pygame.K_t: SearchAction.TOUCH_AND_START,
        pygame.K_p: SearchAction.PAUSE_TOGGLE,
        pygame.K_a: SearchAction.CORRECT_LEFT,
        pygame.K_d: SearchAction.CORRECT_RIGHT,
        pygame.K_n: SearchAction.NEXT_LANE,
        pygame.K_0: SearchAction.SURFACE,
        pygame.K_KP0: SearchAction.SURFACE,
    }.get(key)


def _draw(
    pygame: object,
    screen: object,
    font: object,
    mission: SemicircleSearchMission,
    message: str,
    lane_seconds: float,
    surface_minutes: float,
) -> None:
    screen.fill((24, 27, 33))
    rows = (
        "ROV 连续自动蛇形搜寻（无抓取 / 无机械爪）",
        f"状态: {mission.state.value}   搜寻带: {mission.lane_index + 1}   方向: {'正向' if mission.forward_direction else '反向'}",
        f"每带前进: {lane_seconds:.1f} s   自动上浮倒计时: {surface_minutes:.2f} min（离底定高后开始）",
        "T 触底定高并开始  |  P 暂停/恢复  |  0 持续上浮  |  Esc 急停",
        "备用：A/D 定时微调，N 立即换带；默认流程会自动换带",
        message,
    )
    for index, row in enumerate(rows):
        surface = font.render(row, True, (236, 239, 244) if index != 5 else (253, 205, 79))
        screen.blit(surface, (26, 25 + 48 * index))
    pygame.display.flip()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="单键启动的连续自动蛇形搜寻；不执行抓取。")
    parser.add_argument("--robot-config", default=_package_config_path("robot.example.yaml"))
    parser.add_argument("--dataset-config", default=_package_config_path("dataset.example.yaml"))
    parser.add_argument("--autonomy-config", default=_package_config_path("autonomy.yaml"))
    parser.add_argument("--search-config", default=_default_search_config_path())
    parser.add_argument("--session-dir", help="由完整启动脚本创建的本次输出目录")
    parser.add_argument("--record-port", type=int, default=5704)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--lane-seconds", type=float)
    parser.add_argument("--surface-minutes", type=float)
    parser.add_argument("--execute", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        robot = load_robot_config(args.robot_config)
        dataset = load_dataset_config(args.dataset_config)
        autonomy = load_autonomy_config(args.autonomy_config)
        lane_seconds = _positive_input("每条搜寻带前进时长（秒）：", args.lane_seconds)
        surface_minutes = _positive_input("自动上浮倒计时（分钟）：", args.surface_minutes)
        search = _load_config(args.search_config, lane_seconds=lane_seconds, surface_minutes=surface_minutes)
    except (ConfigurationError, ValueError) as exc:
        print(f"配置错误: {exc}")
        return 2

    errors: list[str] = []
    if robot.control_profile is not ControlProfile.COMMISSIONING:
        errors.append("robot.yaml 的 profile 必须是 commissioning")
    if "ALT_HOLD" not in robot.allowed_flight_modes:
        errors.append("robot.yaml 必须允许 ALT_HOLD")
    if not robot.allow_live_actuation:
        errors.append("robot.yaml 尚未允许真实输出")
    if not robot.allow_ros_arming:
        errors.append("robot.yaml 尚未允许 ROS 解锁")
    if robot.allow_gripper_actuation:
        errors.append("连续搜寻必须关闭机械爪权限")
    if search.target_label not in autonomy.detector.class_names:
        errors.append(f"模型类别不包含 {search.target_label!r}")
    if max(search.forward_command, search.shift_command, search.turn_command, search.descent_command, search.ascent_command) > robot.command_limit:
        errors.append("连续搜寻控制量超过 robot.command_limit")
    if errors:
        print("禁止真实连续搜寻：")
        for error in dict.fromkeys(errors):
            print(f"  - {error}")
        return 2
    if not args.execute:
        print("连续搜寻配置预览通过；未初始化 ROS、未解锁、未输出运动。")
        return 0
    if not args.session_dir:
        print("真实执行必须使用 --session-dir")
        return 2
    try:
        import pygame
    except ImportError:
        print("缺少 pygame，请重新执行安装脚本。")
        return 2

    node: SearchApproachNode | None = None
    recorder: RtpMkvRecorder | None = None
    pygame_started = False
    control_opened = False
    result = 2
    detail = "启动未完成"
    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    try:
        # 复用已有节点：订阅检测、遥测和控制状态，并使用既有许可、解锁、急停服务。
        node = SearchApproachNode(capture_images=False, node_name="rov_continuous_search")
        node.wait_for_initial_data(timeout_s=10.0)
        node.wait_for_perception(timeout_s=20.0)
        error = _prearm_error(node, dataset, require_disarmed=True, allow_gripper=False)
        if error is not None:
            raise DatasetDriveError(error)

        recorder = RtpMkvRecorder(
            Path(args.session_dir).expanduser().resolve(),
            source_port=args.record_port,
            payload_type=args.payload_type,
        )
        recorder.start()
        recorder.wait_until_receiving(timeout_s=10.0, pump=lambda: node.spin(0.0))
        pygame.init()
        pygame_started = True
        screen = pygame.display.set_mode((1180, 330))
        pygame.display.set_caption("ROV Continuous Search")
        font = pygame.font.Font(None, 27)
        print("连续搜寻不会调用机械爪、群体盲抓或抓取策略。")
        print(f"T 后：触底 → 上浮 {search.clearance_m:.2f}m → 自动定时蛇形；稳定发现 {search.target_label} 后仅停车。")
        print("确认 ROV 已浸没、危险区无人、QGC 为 ALT_HOLD、QGC 遥测与视频正常，且可立即人工上锁。")
        if input(f"全部满足后完整输入 {CONFIRMATION!r}: ").strip() != CONFIRMATION:
            raise DatasetDriveError("确认词不匹配，未开启控制")

        for _ in range(5):
            node.publish(MotionCommand.neutral())
            node.spin(0.02)
        node.set_enabled(True)
        control_opened = True
        _wait_for_gateway_state(node, lambda: bool(node.status and node.status.runtime_enabled), "运行时许可")
        node.arm()
        _wait_for_gateway_state(node, lambda: bool(node.status and node.status.armed and node.status.armed_by_ros), "ROS 解锁")
        error = _active_error(node, dataset, allow_gripper=False)
        if error is not None:
            raise DatasetDriveError(error)

        mission = SemicircleSearchMission(search)
        message = "待命：点击本窗口后按 T，开始触底定高与自动搜寻"
        clock = pygame.time.Clock()
        while rclpy.ok():
            action = None
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise DatasetDriveError("控制窗口被关闭")
                if event.type in {getattr(pygame, "WINDOWFOCUSLOST", -1)} or (
                    event.type == pygame.ACTIVEEVENT and getattr(event, "gain", 1) == 0
                ):
                    if mission.state is not SearchState.PAUSED:
                        action = SearchAction.PAUSE_TOGGLE
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        raise DatasetDriveError("Esc 急停")
                    action = _event_action(pygame, event.key)
            node.spin(0.0)
            error = _active_error(node, dataset, allow_gripper=False)
            if error is not None:
                raise DatasetDriveError(error)
            now = time.monotonic()
            decision = mission.step(node.observation(now), now, action)
            message = decision.message
            node.publish(decision.motion)
            _draw(pygame, screen, font, mission, message, lane_seconds, surface_minutes)
            clock.tick(20)
    except KeyboardInterrupt:
        detail = "Ctrl+C 急停"
        result = 130
    except DatasetDriveError as exc:
        detail = str(exc)
        print(f"\n[连续搜寻中止] {detail}", flush=True)
    except Exception as exc:
        detail = f"未预见异常 {type(exc).__name__}: {exc}"
        print(f"\n[连续搜寻中止] {detail}", flush=True)
    finally:
        if node is not None and rclpy.ok() and control_opened:
            try:
                node.publish_neutral()
            except Exception:
                pass
            estop_error = node.emergency_stop()
            if estop_error is not None:
                detail += f"；急停服务未确认: {estop_error}"
                print("无法确认软件急停，立即用 QGC 上锁或物理断电。")
        if recorder is not None:
            _, recording_error = recorder.finalize()
            if recording_error is not None:
                detail += f"；录像封装: {recording_error}"
        if pygame_started:
            pygame.quit()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(f"会话目录: {Path(args.session_dir).expanduser().resolve()}；结束原因: {detail}", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
