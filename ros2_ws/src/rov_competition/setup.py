"""ROS 2 Python 包安装描述。"""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "rov_competition"

setup(
    name=PACKAGE_NAME,
    version="0.2.0rc1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/models", glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="ROV Team",
    maintainer_email="team@example.invalid",
    description="2026 全国水下机器人大赛 ROV 作业赛道控制与感知。",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "rov_replay = rov_competition.replay:main",
            "rov_autonomy = rov_competition.ros_nodes.autonomy_node:main",
            "rov_axis_test = rov_competition.axis_test:main",
            "rov_motion_test = rov_competition.motion_test:main",
            "rov_preflight = rov_competition.preflight_cli:main",
            "rov_stream_bridge = rov_competition.stream_bridge:main",
            "rov_telemetry_view = rov_competition.ros_nodes.telemetry_view_node:main",
            "rov_turn_test = rov_competition.turn_test:main",
            "rov_vehicle = rov_competition.ros_nodes.vehicle_gateway_node:main",
        ],
    },
)
