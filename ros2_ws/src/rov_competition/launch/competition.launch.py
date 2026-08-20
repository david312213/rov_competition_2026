"""启动真实遥测和自主感知；默认禁止任何真实执行。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """声明所有路径和安全参数，返回 ROS 2 启动描述。"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config"),
            DeclareLaunchArgument("autonomy_config"),
            DeclareLaunchArgument("targets_config"),
            DeclareLaunchArgument("video_source", default_value="0"),
            DeclareLaunchArgument("gstreamer", default_value="false"),
            DeclareLaunchArgument("udp_mpegts", default_value="false"),
            # 带框画面通过 ROS 压缩图像话题查看，避免 HighGUI 阻塞节点退出。
            DeclareLaunchArgument("display_window", default_value="false"),
            DeclareLaunchArgument("annotated_rtp_host", default_value=""),
            DeclareLaunchArgument("annotated_rtp_port", default_value="0"),
            DeclareLaunchArgument("enable_actuation", default_value="false"),
            DeclareLaunchArgument("enable_ros_arming", default_value="false"),
            DeclareLaunchArgument(
                "preflight_output_dir", default_value="output/preflight"
            ),
            # 子节点启动 Python 时忽略 ~/.local，防止 CPU Torch 覆盖 .venv CUDA 版。
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            Node(
                package="rov_competition",
                executable="rov_vehicle",
                name="rov_vehicle_gateway",
                output="screen",
                parameters=[
                    {
                        "robot_config": LaunchConfiguration("robot_config"),
                        "enable_actuation": ParameterValue(
                            LaunchConfiguration("enable_actuation"), value_type=bool
                        ),
                        "enable_ros_arming": ParameterValue(
                            LaunchConfiguration("enable_ros_arming"), value_type=bool
                        ),
                        "preflight_output_dir": LaunchConfiguration(
                            "preflight_output_dir"
                        ),
                    }
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
                        # 数字形式的视频源（例如 UDP 端口 "5702" 或摄像头
                        # 编号 "0"）也必须保留为字符串。若不显式指定类型，
                        # launch_ros 会把纯数字文本自动解析成整数，而自主节点
                        # 为了统一处理摄像头、文件和网络地址，声明的是字符串参数。
                        "video_source": ParameterValue(
                            LaunchConfiguration("video_source"), value_type=str
                        ),
                        "gstreamer": ParameterValue(
                            LaunchConfiguration("gstreamer"), value_type=bool
                        ),
                        "udp_mpegts": ParameterValue(
                            LaunchConfiguration("udp_mpegts"), value_type=bool
                        ),
                        "display_window": ParameterValue(
                            LaunchConfiguration("display_window"), value_type=bool
                        ),
                        "annotated_rtp_host": LaunchConfiguration("annotated_rtp_host"),
                        "annotated_rtp_port": ParameterValue(
                            LaunchConfiguration("annotated_rtp_port"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
