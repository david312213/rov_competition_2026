"""仅启动真实 MAVLink 遥测，适合人机协同赛与 ROS 加分展示。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """默认只读启动飞控网关。"""

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_config"),
            DeclareLaunchArgument("enable_actuation", default_value="false"),
            DeclareLaunchArgument("enable_ros_arming", default_value="false"),
            DeclareLaunchArgument(
                "preflight_output_dir", default_value="output/preflight"
            ),
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
        ]
    )
