"""安全的视频回放入口，用于验证模型、叠加画面和状态机。

该程序永远不导入 MAVLink 执行层，也不会发送推进器或机械爪命令。它是新模型、
新阈值和新比赛录像进入真实机器人前必须通过的第一道验证门。
"""

from __future__ import annotations

import argparse
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

from .config import load_autonomy_config
from .detector import YoloDetector, draw_competition_overlay
from .domain import (
    GripperAction,
    MissionDecision,
    MissionObservation,
    MissionState,
    MotionCommand,
)
from .mission import AutonomousGraspMission
from .targets import load_target_config
from .video import OpenCvVideoSource, VideoSourceError


def _parse_source(value: str) -> str | int:
    """纯数字参数解释为摄像头编号，其余作为文件或 GStreamer 字符串。"""

    return int(value) if value.isdecimal() else value


def build_argument_parser() -> argparse.ArgumentParser:
    """创建命令行解析器，便于测试参数约束。"""

    parser = argparse.ArgumentParser(description="YOLO26 自主抓取安全回放")
    parser.add_argument("--video", required=True, help="录像路径、摄像头编号或 GStreamer 管线")
    parser.add_argument("--config", default="config/autonomy.yaml", help="自主配置 YAML")
    parser.add_argument(
        "--targets",
        default="config/grasp_targets.yaml",
        help="允许抓取类别 YAML",
    )
    parser.add_argument("--output", help="可选的标注录像输出路径，例如 output.mp4")
    parser.add_argument("--show", action="store_true", help="显示实时窗口，按 q 退出")
    parser.add_argument("--gstreamer", action="store_true", help="把 --video 作为 GStreamer 管线")
    parser.add_argument(
        "--simulate-mission",
        action="store_true",
        help="用明确的虚拟深度/航向驱动状态机；仍不连接 MAVLink",
    )
    return parser


def _create_writer(path: str, frame: Any, fps: float) -> Any:
    """按首帧尺寸创建 MP4 写入器。"""

    import cv2

    output_path = Path(path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    height, width = frame.shape[:2]
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps if fps > 0 else 20.0,
        (width, height),
    )
    if not writer.isOpened():
        raise VideoSourceError(f"无法创建输出录像: {path}")
    return writer


def main() -> None:
    """运行录像/相机回放，输出裁判可见叠加画面。"""

    args = build_argument_parser().parse_args()
    config = load_autonomy_config(args.config)
    detector = YoloDetector(config.detector)
    mission_config = replace(
        config.mission,
        # 只用于画面上明确标记的虚拟回放。真实自主仍会因 YAML
        # 中的 null 被启动门拒绝，回放不会回写配置。
        image_yaw_sign=(config.mission.image_yaw_sign or 1),
        image_vertical_sign=(config.mission.image_vertical_sign or 1),
    )
    mission = AutonomousGraspMission(
        mission_config,
        load_target_config(args.targets).graspable_labels,
    )

    import cv2

    writer = None
    frame_count = 0
    inference_started = time.monotonic()
    simulation_time = 0.0
    simulation_depth_m = 1.0
    simulation_yaw_deg = 0.0
    previous_motion = MotionCommand.neutral()
    mission_started = False
    try:
        with OpenCvVideoSource(_parse_source(args.video), gstreamer=args.gstreamer) as source:
            while True:
                ok, frame = source.read()
                if not ok:
                    break
                detections = detector.detect(frame)
                height, width = frame.shape[:2]
                frame_count += 1
                if args.simulate_mission:
                    # 虚拟数据只是为了让状态转移在录像上可观察，不是 ROV
                    # 动力学模型，更不能当作实艇验收证据。
                    simulation_time += 0.05
                    simulation_depth_m = max(
                        0.0,
                        simulation_depth_m - previous_motion.vertical * 0.5 * 0.05,
                    )
                    simulation_yaw_deg = (
                        simulation_yaw_deg + previous_motion.yaw * 720.0 * 0.05
                    ) % 360.0
                    observation = MissionObservation(
                        frame_id=frame_count,
                        detections=tuple(detections),
                        frame_width=width,
                        frame_height=height,
                        perception_valid=True,
                        depth_valid=True,
                        depth_m=simulation_depth_m,
                        attitude_valid=True,
                        yaw_deg=simulation_yaw_deg,
                    )
                    if not mission_started:
                        decision = mission.start(observation, simulation_time)
                        decision = mission.acknowledge_gripper(
                            GripperAction.OPEN,
                            True,
                            "SIMULATION accepted",
                            observation,
                            simulation_time,
                        )
                        mission_started = True
                    else:
                        decision = mission.step(observation, simulation_time)
                    if decision.gripper == GripperAction.CLOSE:
                        decision = mission.acknowledge_gripper(
                            GripperAction.CLOSE,
                            True,
                            "SIMULATION accepted",
                            observation,
                            simulation_time,
                        )
                    previous_motion = decision.motion
                else:
                    decision = MissionDecision(
                        state=MissionState.IDLE,
                        motion=MotionCommand.neutral(),
                        message="PERCEPTION-ONLY REPLAY; mission simulation disabled",
                    )
                elapsed = max(1e-6, time.monotonic() - inference_started)
                annotated = draw_competition_overlay(
                    frame,
                    detections,
                    mission_state=decision.state.value,
                    mission_message=decision.message,
                    fps=frame_count / elapsed,
                    selected_target=decision.selected_target,
                    aim_point=(
                        config.mission.grasp_aim_x_ratio,
                        config.mission.grasp_aim_y_ratio,
                    ),
                    target_area_ratio=decision.target_area_ratio,
                    grasp_area_threshold=decision.grasp_area_threshold,
                    current_depth_m=(
                        simulation_depth_m if args.simulate_mission else None
                    ),
                    scan_progress_deg=decision.scan_progress_deg,
                    simulation=True,
                )
                if args.output:
                    if writer is None:
                        writer = _create_writer(args.output, annotated, 20.0)
                    writer.write(annotated)
                if args.show:
                    cv2.imshow("ROV competition replay", annotated)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
    finally:
        if writer is not None:
            writer.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
