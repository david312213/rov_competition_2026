cmake_minimum_required(VERSION 3.8)

project(ros2_topic_forwarding)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotDataMessage.msg"
)


add_executable(topic_forwarding
  src/topic_forwarding.cpp
)


ament_target_dependencies(
  topic_forwarding
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)


rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)


target_link_libraries(
  topic_forwarding
  "${cpp_typesupport_target}"
  nlohmann_json::nlohmann_json
)


install(
  TARGETS
  topic_forwarding
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

install(
 DIRECTORY
 launch
 config
 DESTINATION share/${PROJECT_NAME}
)


ament_package()
