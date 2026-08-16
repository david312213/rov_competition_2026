cmake_minimum_required(VERSION 3.8)
project(rov_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlStatus.msg"
  "msg/MissionStatus.msg"
  "msg/NormalizedMotionCommand.msg"
  "msg/RobotTelemetry.msg"
  "msg/TargetDetection.msg"
  "msg/TargetDetectionArray.msg"
  "srv/SetArmed.srv"
  "srv/SetGripper.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
