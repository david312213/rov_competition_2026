"""一键启动软件视频分流和 YOLO，不连接飞控网关。

QGC 直接使用 BlueOS -> 5600 默认视频；本 launch 只处理
BlueOS -> 5700 -> YOLO 5702，两条链路互不抢占。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """组合视频分流与自主感知节点，并固定关闭所有本地窗口和实机控制。"""

    # rov_stream_bridge 是普通 argparse 程序，不是 rclpy 节点。必须使用
    # ExecuteProcess；若用 launch_ros.actions.Node，ROS 会自动追加
    # ``--ros-args``，从而被 argparse 判定为未知参数。
    stream_bridge = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rov_competition",
            "rov_stream_bridge",
            "--source-port",
            LaunchConfiguration("source_port"),
            "--no-qgc",
            "--inference-host",
            LaunchConfiguration("ai_host"),
            "--inference-port",
            LaunchConfiguration("ai_port"),
            "--no-display",
        ],
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="视频分流器已经退出"))],
    )

    autonomy = Node(
        package="rov_competition",
        executable="rov_autonomy",
        name="rov_autonomy",
        output="screen",
        parameters=[
            {
                "robot_config": LaunchConfiguration("robot_config"),
                "autonomy_config": LaunchConfiguration("autonomy_config"),
                "targets_config": LaunchConfiguration("targets_config"),
                # ai_port 是纯数字文本，必须强制保留为字符串视频源。
                "video_source": ParameterValue(
                    LaunchConfiguration("ai_port"), value_type=str
                ),
                "gstreamer": True,
                "udp_mpegts": False,
                "display_window": False,
                # 禁用任务启动服务；节点只运行感知循环，不发布运动。
                "perception_only": True,
            }
        ],
        on_exit=[EmitEvent(event=Shutdown(reason="YOLO 感知节点已经退出"))],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config"),
            DeclareLaunchArgument("autonomy_config"),
            DeclareLaunchArgument("targets_config"),
            DeclareLaunchArgument("source_port", default_value="5700"),
            DeclareLaunchArgument("ai_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("ai_port", default_value="5702"),
            # 子节点启动 Python 时忽略 ~/.local，防止 CPU Torch 覆盖 CUDA 版。
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            stream_bridge,
            autonomy,
        ]
    )
