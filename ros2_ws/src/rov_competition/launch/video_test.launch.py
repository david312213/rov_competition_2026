"""一键启动原始 RTP 分流和 YOLO，只做视频测试，不连接飞控网关。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """组合视频分流与自主感知节点，并固定关闭所有本地窗口和实机控制。"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config"),
            DeclareLaunchArgument("autonomy_config"),
            DeclareLaunchArgument("targets_config"),
            DeclareLaunchArgument("source_port", default_value="5600"),
            DeclareLaunchArgument("qgc_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("qgc_port", default_value="5701"),
            DeclareLaunchArgument("ai_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("ai_port", default_value="5702"),
            # 子节点启动 Python 时忽略 ~/.local，防止 CPU Torch 覆盖 CUDA 版。
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            Node(
                package="rov_competition",
                executable="rov_stream_bridge",
                name="rov_stream_bridge",
                output="screen",
                arguments=[
                    "--source-port",
                    LaunchConfiguration("source_port"),
                    "--qgc-host",
                    LaunchConfiguration("qgc_host"),
                    "--qgc-port",
                    LaunchConfiguration("qgc_port"),
                    "--inference-host",
                    LaunchConfiguration("ai_host"),
                    "--inference-port",
                    LaunchConfiguration("ai_port"),
                    "--no-display",
                ],
            ),
            Node(
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
                    }
                ],
            ),
        ]
    )
