"""水池数据集采集的键盘驾驶窗口。

本工具只发布四轴归一化运动意图。八路推进器的混控、定深和
姿态稳定仍由 ArduSub 负责。程序不自动切换模式、不自动沉底、
不加载 YOLO，也不创建机械爪客户端。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import traceback
from pathlib import Path
from typing import Callable

import rclpy
from rclpy.client import Client
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from rov_interfaces.msg import RobotTelemetry
from rov_interfaces.srv import SetArmed
from std_srvs.srv import SetBool, Trigger

from .commissioning_runtime import CommissioningNode
from .config import (
    ConfigurationError,
    DatasetCollectionConfig,
    RobotConfig,
    load_dataset_config,
    load_robot_config,
)
from .dataset_control import (
    DatasetRuntimeSnapshot,
    DatasetControlError,
    KeyCommandState,
    RecoveryController,
    active_safety_error,
    prearm_safety_error,
)
from .dataset_recording import (
    DatasetRecordingError,
    DatasetSessionLogger,
    RtpMkvRecorder,
)
from .domain import MotionCommand


DATASET_CONFIRMATION = "START DATASET ROV"
ARM_CONFIRMATION = "ARM ROV"
SOURCE = "commissioning"


class DatasetDriveError(RuntimeError):
    """实艇键盘驾驶无法安全继续。"""


def _package_config_path(name: str) -> str:
    """优先使用 ROS 安装后的配置，源码环境则回退到包目录。"""

    try:
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except ImportError:
        return str(Path(__file__).resolve().parents[1] / "config" / name)
    try:
        root = Path(get_package_share_directory("rov_competition"))
    except PackageNotFoundError:
        root = Path(__file__).resolve().parents[1]
    return str(root / "config" / name)


def build_parser() -> argparse.ArgumentParser:
    """创建默认只预览的参数解析器。"""

    parser = argparse.ArgumentParser(
        description="ROV 键盘驾驶+原始 H.264 数据集录像；默认只检查配置。"
    )
    parser.add_argument(
        "--robot-config", default=_package_config_path("robot.example.yaml")
    )
    parser.add_argument(
        "--dataset-config", default=_package_config_path("dataset.example.yaml")
    )
    parser.add_argument("--session-dir", help="当次 output/datasets/<time> 目录")
    parser.add_argument("--project-dir", default=str(Path.cwd()))
    parser.add_argument("--record-port", type=int, default=5702)
    parser.add_argument("--payload-type", type=int, default=96)
    parser.add_argument("--execute", action="store_true")
    return parser


class DatasetDriveNode(CommissioningNode):
    """在通用实艇运行时上增加许可、解锁和急停服务。"""

    def __init__(self, source: str = SOURCE) -> None:
        self.last_valid_attitude_at: float | None = None
        super().__init__("rov_dataset_drive", source)
        self.enable_client = self.create_client(SetBool, "/rov/control/set_enabled")
        self.arm_client = self.create_client(SetArmed, "/rov/control/set_armed")
        self.estop_client = self.create_client(
            Trigger, "/rov/control/emergency_stop"
        )

    def _telemetry(self, message: RobotTelemetry) -> None:
        """缓存遥测，并记住最近一次有效姿态的本机接收时间。"""

        super()._telemetry(message)
        if message.valid_attitude:
            self.last_valid_attitude_at = time.monotonic()

    def spin(self, timeout_s: float = 0.0) -> None:
        """处理 ROS 回调。"""

        rclpy.spin_once(self, timeout_sec=timeout_s)

    def _call(
        self,
        client: Client,
        request: object,
        *,
        timeout_s: float,
        neutral_while_waiting: bool = False,
    ) -> object:
        """有超时地调用服务；需要时在等待期间继续回中。"""

        service_name = str(getattr(client, "srv_name", "ROS service"))
        if not client.wait_for_service(timeout_sec=1.0):
            raise DatasetDriveError(f"服务不可用: {service_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_s
        next_neutral = 0.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self.spin(0.02)
            now = time.monotonic()
            if neutral_while_waiting and now >= next_neutral:
                self.publish(MotionCommand.neutral())
                next_neutral = now + 0.05
        if not future.done():
            raise DatasetDriveError(f"服务超时: {service_name}")
        try:
            result = future.result()
        except Exception as exc:
            raise DatasetDriveError(f"服务异常 {service_name}: {exc}") from exc
        if result is None or not bool(result.success):
            message = getattr(result, "message", "无返回内容")
            raise DatasetDriveError(f"服务拒绝 {service_name}: {message}")
        return result

    def set_enabled(self, enabled: bool) -> None:
        """开启或关闭网关运行时许可。"""

        request = SetBool.Request()
        request.data = bool(enabled)
        self._call(self.enable_client, request, timeout_s=4.0)

    def arm(self) -> None:
        """使用网关的固定确认词正常解锁。"""

        request = SetArmed.Request()
        request.arm = True
        request.confirmation = ARM_CONFIRMATION
        self._call(self.arm_client, request, timeout_s=5.0)

    def emergency_stop(self) -> str | None:
        """调用锁存急停；失败时返回人工处置说明。"""

        try:
            request = Trigger.Request()
            result = self._call(
                self.estop_client,
                request,
                timeout_s=4.0,
                neutral_while_waiting=True,
            )
            return None if bool(result.success) else str(result.message)
        except DatasetDriveError as exc:
            return str(exc)

    def wait_for_initial_data(self, timeout_s: float = 8.0) -> None:
        """在录像和解锁前等待发布器、遥测与控制状态。"""

        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin(0.05)
            if (
                self.publisher.get_subscription_count() > 0
                and self.telemetry is not None
                and self.status is not None
            ):
                return
        if self.publisher.get_subscription_count() == 0:
            raise DatasetDriveError("未发现 /rov/control/command 订阅者")
        raise DatasetDriveError("未收齐 /rov/telemetry 和 /rov/control/status")


def _runtime_snapshot(node: DatasetDriveNode) -> DatasetRuntimeSnapshot:
    """将 ROS 消息转成可离线测试的运行时事实。"""

    now = time.monotonic()
    telemetry = node.telemetry
    status = node.status
    if telemetry is None or node.telemetry_received_at is None:
        raise DatasetDriveError("遥测缺失")
    if status is None or node.status_received_at is None:
        raise DatasetDriveError("控制状态缺失")
    return DatasetRuntimeSnapshot(
        telemetry_age_s=now - node.telemetry_received_at,
        status_age_s=now - node.status_received_at,
        heartbeat_valid=bool(telemetry.valid_heartbeat),
        attitude_valid=bool(telemetry.valid_attitude),
        attitude_age_s=(
            math.inf
            if node.last_valid_attitude_at is None
            else now - node.last_valid_attitude_at
        ),
        telemetry_mode=str(telemetry.flight_mode),
        status_mode=str(status.flight_mode),
        preflight_passed=bool(status.preflight_passed),
        estop_latched=bool(status.emergency_stop_latched),
        state=str(status.state),
        allows_actuation=bool(status.configuration_allows_actuation),
        allows_arming=bool(status.configuration_allows_ros_arming),
        allows_gripper=bool(status.configuration_allows_gripper),
        runtime_enabled=bool(status.runtime_enabled),
        telemetry_armed=bool(telemetry.armed),
        status_armed=bool(status.armed),
        armed_by_ros=bool(status.armed_by_ros),
        command_source=str(status.command_source),
    )


def _prearm_error(
    node: DatasetDriveNode,
    config: DatasetCollectionConfig,
    *,
    require_disarmed: bool,
) -> str | None:
    """检查开启许可之前的状态。"""

    snapshot = _runtime_snapshot(node)
    if require_disarmed:
        return prearm_safety_error(snapshot, config, source=SOURCE)
    return None


def _active_error(
    node: DatasetDriveNode, config: DatasetCollectionConfig
) -> str | None:
    """每个控制周期检查遥测、模式、来源与解锁状态。"""

    return active_safety_error(_runtime_snapshot(node), config, source=SOURCE)


def _current_depth(node: DatasetDriveNode) -> float | None:
    """返回仅供显示/回收使用的当前深度，不作为驾驶门控。"""

    telemetry = node.telemetry
    if telemetry is None or not telemetry.valid_depth:
        return None
    depth_m = float(telemetry.depth_m)
    return depth_m if math.isfinite(depth_m) else None


def _publish_neutral_once(node: DatasetDriveNode) -> None:
    """录像安全封装期间的 ROS 泵函数。"""

    node.spin(0.0)
    node.publish(MotionCommand.neutral())


def _publish_neutral_with_health_check(
    node: DatasetDriveNode, config: DatasetCollectionConfig
) -> None:
    """封装录像时继续回中，但任一飞控故障立即打断等待。"""

    node.spin(0.0)
    error = _active_error(node, config)
    if error is not None:
        raise DatasetDriveError(error)
    node.publish(MotionCommand.neutral())


def _wait_for_gateway_state(
    node: DatasetDriveNode,
    predicate: Callable[[], bool],
    description: str,
    *,
    timeout_s: float = 5.0,
) -> None:
    """服务成功后再等待状态话题与飞控心跳证明结果。"""

    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and time.monotonic() < deadline:
        node.spin(0.05)
        if predicate():
            return
    raise DatasetDriveError(f"等待{description}超时")


def _event_key_name(pygame: object, key_code: int) -> str | None:
    """将 Pygame 键值转成纯 Python 控制键名。"""

    mapping = {
        pygame.K_w: "w",
        pygame.K_s: "s",
        pygame.K_a: "a",
        pygame.K_d: "d",
        pygame.K_1: "1",
        pygame.K_KP1: "1",
        pygame.K_2: "2",
        pygame.K_KP2: "2",
        pygame.K_UP: "up",
        pygame.K_DOWN: "down",
    }
    return mapping.get(key_code)


def _draw_window(
    pygame: object,
    screen: object,
    font: object,
    *,
    key_state: KeyCommandState,
    depth_m: float | None,
    yaw_deg: float,
    message: str,
) -> None:
    """绘制简单高对比控制面板；不显示相机画面。"""

    screen.fill((17, 24, 39))
    depth_text = "n/a" if depth_m is None else f"{depth_m:.2f} m"
    lines = [
        "ROV DATASET DRIVE - HOLD KEY TO MOVE",
        "W/S forward/back | A/D left/right | 1/2 turn",
        "UP/DOWN ascend/descend | +/- power | SPACE neutral",
        "0 finalize + recover | ESC/close emergency stop",
        f"power={key_state.strength:.2f}  held={'+'.join(sorted(key_state.held_keys)) or 'none'}",
        f"depth={depth_text} (display only)  yaw={yaw_deg:.1f} deg",
        message[:95],
    ]
    for index, line in enumerate(lines):
        color = (235, 245, 255) if index < 6 else (255, 205, 80)
        surface = font.render(line, True, color)
        screen.blit(surface, (24, 24 + index * 38))
    pygame.display.flip()


def _log_sample(
    logger: DatasetSessionLogger,
    node: DatasetDriveNode,
    key_state: KeyCommandState,
    motion: MotionCommand,
    *,
    event: str,
    message: str = "",
) -> None:
    """把当前控制与遥测写入 CSV。"""

    telemetry = node.telemetry
    status = node.status
    logger.write(
        event=event,
        keys=key_state.held_keys,
        motion=motion,
        depth_m=(float(telemetry.depth_m) if telemetry and telemetry.valid_depth else None),
        yaw_deg=(float(telemetry.yaw_deg) if telemetry and telemetry.valid_attitude else None),
        flight_mode=(str(telemetry.flight_mode) if telemetry else ""),
        armed=bool(status.armed) if status else False,
        message=message,
    )


def _recover_to_start(
    node: DatasetDriveNode,
    config: DatasetCollectionConfig,
    key_state: KeyCommandState,
    logger: DatasetSessionLogger,
    start_depth_m: float,
) -> None:
    """按 0 或单纯录像故障后，仅在遥测健康时闭环上升。"""

    controller = RecoveryController(config, start_depth_m, time.monotonic())
    period_s = 1.0 / config.publish_rate_hz
    next_publish = time.monotonic()
    while rclpy.ok():
        node.spin(0.0)
        error = _active_error(node, config)
        if error is not None:
            raise DatasetDriveError(f"回收中止: {error}")
        assert node.telemetry is not None
        now = time.monotonic()
        step = controller.step(
            depth_valid=bool(node.telemetry.valid_depth),
            depth_m=float(node.telemetry.depth_m),
            now=now,
        )
        if now >= next_publish:
            node.publish(step.motion)
            _log_sample(
                logger,
                node,
                key_state,
                step.motion,
                event="recovery",
                message=step.message,
            )
            next_publish = now + period_s
        if step.complete:
            node.publish_neutral()
            return
        time.sleep(0.005)


def _run_keyboard_loop(
    *,
    pygame: object,
    node: DatasetDriveNode,
    config: DatasetCollectionConfig,
    recorder: RtpMkvRecorder,
    logger: DatasetSessionLogger,
) -> tuple[str, str]:
    """运行按住才动的事件循环，返回退出类型和说明。"""

    key_state = KeyCommandState(config)
    screen = pygame.display.set_mode((860, 320))
    pygame.display.set_caption("ROV Dataset Drive")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()
    period_s = 1.0 / config.publish_rate_hz
    next_publish = time.monotonic()
    message = "Hold a key to move. Release/focus loss returns to neutral."

    while rclpy.ok():
        normal_finish = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "estop", "控制窗口被关闭"
            if event.type == getattr(pygame, "WINDOWFOCUSLOST", -1):
                key_state.clear()
                node.publish(MotionCommand.neutral())
                message = "Focus lost: neutral command sent"
                _log_sample(
                    logger,
                    node,
                    key_state,
                    MotionCommand.neutral(),
                    event="focus_lost",
                    message=message,
                )
                continue
            if event.type == pygame.ACTIVEEVENT and getattr(event, "gain", 1) == 0:
                key_state.clear()
                node.publish(MotionCommand.neutral())
                message = "Window inactive: neutral command sent"
                continue
            if event.type == pygame.KEYDOWN:
                key_name = _event_key_name(pygame, event.key)
                if key_name is not None:
                    key_state.press(key_name)
                    message = f"Key down: {key_name}"
                elif event.key == pygame.K_SPACE:
                    key_state.clear()
                    node.publish(MotionCommand.neutral())
                    message = "SPACE: neutral"
                elif event.key in {
                    pygame.K_PLUS,
                    pygame.K_EQUALS,
                    pygame.K_KP_PLUS,
                }:
                    message = f"Power: {key_state.adjust(1):.2f}"
                elif event.key in {pygame.K_MINUS, pygame.K_KP_MINUS}:
                    message = f"Power: {key_state.adjust(-1):.2f}"
                elif event.key in {pygame.K_0, pygame.K_KP0}:
                    key_state.clear()
                    node.publish(MotionCommand.neutral())
                    normal_finish = True
                    message = "0: finalize recording and recover"
                elif event.key == pygame.K_ESCAPE:
                    return "estop", "Esc 急停"
                _log_sample(
                    logger,
                    node,
                    key_state,
                    key_state.motion(),
                    event="key_down",
                    message=message,
                )
            elif event.type == pygame.KEYUP:
                key_name = _event_key_name(pygame, event.key)
                if key_name is not None:
                    key_state.release(key_name)
                    message = f"Key up: {key_name}"
                    # 在下一个 20 Hz 节拍前先发一次最新结果，
                    # 单轴松开后不必等待最长 50 ms。
                    node.publish(key_state.motion())
                    _log_sample(
                        logger,
                        node,
                        key_state,
                        key_state.motion(),
                        event="key_up",
                        message=message,
                    )
        if normal_finish:
            return "normal", "操作员按 0 结束采集"

        node.spin(0.0)
        error = _active_error(node, config)
        if error is not None:
            raise DatasetDriveError(error)
        if not recorder.is_stream_fresh(maximum_idle_s=3.0):
            # 录像失效时先回中；上层只在深度可用时执行回收。
            return "recording_fault", "原始视频录像停止增长"

        assert node.telemetry is not None
        now = time.monotonic()
        requested = key_state.motion()
        if now >= next_publish:
            node.publish(requested)
            _log_sample(
                logger,
                node,
                key_state,
                requested,
                event="command",
                message=message,
            )
            next_publish = now + period_s

        _draw_window(
            pygame,
            screen,
            font,
            key_state=key_state,
            depth_m=_current_depth(node),
            yaw_deg=float(node.telemetry.yaw_deg),
            message=message,
        )
        clock.tick(60)
    raise DatasetDriveError("ROS 上下文已关闭")


def _interactive_confirmation() -> None:
    """在解锁前要求操作员亲自确认现场条件。"""

    if not sys.stdin.isatty():
        raise DatasetDriveError("真实执行必须在交互终端输入确认词")
    print("\n解锁前最后确认：")
    print("  1. ROV 已完全浸没，推进器危险区无人。")
    print("  2. 飞控已由操作员切到 ALT_HOLD。")
    print("  3. QGC 画面和手动上锁功能可用，安全员可断电。")
    print("  4. 键盘工具不设置软件深度上限，操作员负责观察水池和缆线。")
    typed = input(f"\n若全部满足，完整输入 {DATASET_CONFIRMATION!r}: ").strip()
    if typed != DATASET_CONFIRMATION:
        raise DatasetDriveError("确认词不匹配，未开启控制")


def main(argv: list[str] | None = None) -> int:
    """预览配置，或执行一次有完整证据文件的数据采集。"""

    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    args = build_parser().parse_args(remove_ros_args(args=raw_args)[1:])
    try:
        robot = load_robot_config(args.robot_config)
        config = load_dataset_config(args.dataset_config)
        errors = config.readiness_errors(robot)
    except (ConfigurationError, ValueError) as exc:
        print(f"配置错误: {exc}")
        return 2

    if errors:
        print("禁止真实数据采集：")
        for error in errors:
            print(f"  - {error}")
        return 2
    if not args.execute:
        print("配置预览通过；未初始化 ROS、未启动录像、不会解锁。")
        print(
            f"指令 {config.initial_command:.2f}，最大 {config.maximum_command:.2f}；"
            "键盘采集不设置软件深度上限。"
        )
        return 0
    if not args.session_dir:
        print("真实执行必须使用 --session-dir 指定独立会话目录")
        return 2

    try:
        import pygame
    except ImportError:
        print("缺少 pygame，请重新执行安装脚本。")
        return 2

    session_directory = Path(args.session_dir).expanduser().resolve()
    logger: DatasetSessionLogger | None = None
    recorder: RtpMkvRecorder | None = None
    node: DatasetDriveNode | None = None
    pygame_started = False
    control_opened = False
    normal_completed = False
    recovery_completed = False
    outcome = "failed_before_arm"
    detail = "启动未完成"
    video_path: Path | None = None
    result_code = 2

    rclpy.init(args=[raw_args[0]], signal_handler_options=SignalHandlerOptions.NO)
    try:
        logger = DatasetSessionLogger(
            session_directory,
            project_directory=args.project_dir,
            robot_config_path=args.robot_config,
            dataset_config_path=args.dataset_config,
        )
        recorder = RtpMkvRecorder(
            session_directory,
            source_port=args.record_port,
            payload_type=args.payload_type,
        )
        node = DatasetDriveNode(robot.allowed_command_source)
        node.wait_for_initial_data()
        error = _prearm_error(node, config, require_disarmed=True)
        if error is not None:
            raise DatasetDriveError(error)

        pygame.init()
        pygame_started = True
        # 在解锁前就建立窗口，避免解锁后才发现图形环境不可用。
        probe = pygame.display.set_mode((860, 320))
        pygame.display.set_caption("ROV Dataset Drive - PREARM")
        probe.fill((25, 25, 25))
        pygame.display.flip()

        start_depth_m = _current_depth(node)
        logger.set_start_depth(start_depth_m)

        recorder.start()
        recorder.wait_until_receiving(
            timeout_s=8.0,
            pump=lambda: node.spin(0.0),
        )
        _interactive_confirmation()

        # 先把网关记住的最新意图明确刷成中位，再开许可。
        for _ in range(5):
            node.publish(MotionCommand.neutral())
            node.spin(0.02)
        node.set_enabled(True)
        control_opened = True
        _wait_for_gateway_state(
            node,
            lambda: bool(node.status and node.status.runtime_enabled),
            "运行时许可",
        )
        node.arm()
        _wait_for_gateway_state(
            node,
            lambda: bool(
                node.status
                and node.status.armed
                and node.status.armed_by_ros
                and node.status.runtime_enabled
            ),
            "ROS 解锁",
        )
        error = _active_error(node, config)
        if error is not None:
            raise DatasetDriveError(error)

        print("\n键盘采集已开始；按住才动，松开即回中。")
        exit_kind, exit_detail = _run_keyboard_loop(
            pygame=pygame,
            node=node,
            config=config,
            recorder=recorder,
            logger=logger,
        )
        detail = exit_detail
        if exit_kind == "estop":
            raise DatasetDriveError(exit_detail)

        # 正常结束的顺序是：回中 -> EOS 封装 -> 可选深度回收 -> 上锁。
        node.publish_neutral()
        video_path, recording_error = recorder.finalize(
            pump=lambda: _publish_neutral_with_health_check(node, config)
        )
        if recording_error is not None:
            detail = f"{detail}；录像异常: {recording_error}"
        health_error = _active_error(node, config)
        if health_error is not None:
            raise DatasetDriveError(f"录像封装后控制链不安全: {health_error}")
        current_depth_m = _current_depth(node)
        if start_depth_m is not None and current_depth_m is not None:
            _recover_to_start(
                node,
                config,
                KeyCommandState(config),
                logger,
                start_depth_m,
            )
            recovery_completed = True
        else:
            node.publish_neutral()
            detail = f"{detail}；无有效深度，跳过自动回收"
        disarm_error = node.request_normal_disarm(timeout_s=5.0)
        if disarm_error is not None:
            raise DatasetDriveError(f"正常上锁未确认: {disarm_error}")
        _wait_for_gateway_state(
            node,
            lambda: bool(node.status and not node.status.armed),
            "上锁",
        )
        normal_completed = True
        outcome = (
            "completed"
            if exit_kind == "normal"
            else (
                "recording_fault_recovered"
                if recovery_completed
                else "recording_fault_stopped"
            )
        )
        result_code = 0 if exit_kind == "normal" and recording_error is None else 2
    except KeyboardInterrupt:
        outcome = "emergency_stopped"
        detail = "Ctrl+C 急停"
        result_code = 130
    except (DatasetDriveError, DatasetControlError, DatasetRecordingError) as exc:
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = str(exc)
        print(f"\n采集中止: {detail}")
        result_code = 2
    except Exception as exc:
        # 图形库、文件系统或其他未预见错误也必须走同一条
        # finally 急停路径，不能让 Python traceback 绕过实艇收尾。
        outcome = "emergency_stopped" if control_opened else "failed_before_arm"
        detail = f"未预见异常 {type(exc).__name__}: {exc}"
        trace = traceback.format_exc()
        try:
            trace_path = session_directory / "logs" / "dataset_drive_traceback.log"
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_path.write_text(trace, encoding="utf-8")
        except OSError:
            pass
        print(f"\n采集中止: {detail}")
        print("完整 traceback 已尽力保存到本次会话 logs。")
        result_code = 2
    finally:
        if node is not None and rclpy.ok() and control_opened and not normal_completed:
            try:
                node.publish_neutral()
            except Exception:
                pass
            estop_error = node.emergency_stop()
            if estop_error is not None:
                detail += f"；急停服务未确认: {estop_error}"
                print("无法确认软件急停，立即用 QGC 上锁或物理断电。")
            # 急停服务会关闭运行时许可。之后不再发布命令，
            # 只等待录像器处理 EOS，避免干扰已锁存的网关。
            control_opened = False
        if recorder is not None and video_path is None:
            finalized, recording_error = recorder.finalize(
                pump=(
                    (lambda: _publish_neutral_once(node))
                    if node is not None and rclpy.ok() and control_opened
                    else None
                )
            )
            video_path = finalized
            if recording_error is not None:
                detail += f"；录像封装: {recording_error}"
        if logger is not None:
            logger.finish(outcome=outcome, detail=detail, video_path=video_path)
        if pygame_started:
            pygame.quit()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f"会话目录: {session_directory}")
    if normal_completed:
        if recovery_completed:
            print("录像已停止，ROV 已回到启动深度附近并正常上锁。")
        else:
            print("录像已停止；因无有效深度跳过回收，ROV 已正常上锁。")
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
