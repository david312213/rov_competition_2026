"""只启动 GPU YOLO 感知，不发布运动指令或正式任务状态。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """为搜索水池测试提供独立的结构化检测话题。"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config"),
            DeclareLaunchArgument("autonomy_config"),
            DeclareLaunchArgument("targets_config"),
            DeclareLaunchArgument("ai_port", default_value="5702"),
            SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
            Node(
                package="rov_competition",
                executable="rov_autonomy",
                name="rov_search_perception",
                output="screen",
                parameters=[
                    {
                        "robot_config": LaunchConfiguration("robot_config"),
                        "autonomy_config": LaunchConfiguration("autonomy_config"),
                        "targets_config": LaunchConfiguration("targets_config"),
                        "video_source": ParameterValue(
                            LaunchConfiguration("ai_port"), value_type=str
                        ),
                        "gstreamer": True,
                        "udp_mpegts": False,
                        "display_window": False,
                        "perception_only": True,
                    }
                ],
            ),
        ]
    )
