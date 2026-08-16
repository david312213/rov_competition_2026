# ROS 控制链的当前安全状态。该消息只说明软件状态，不能代替物理急停。

builtin_interfaces/Time stamp
string state
string protocol
string command_source

bool configuration_allows_actuation
bool configuration_allows_ros_arming
bool configuration_allows_gripper
bool runtime_enabled
bool preflight_passed
bool emergency_stop_latched
bool armed_by_ros

bool armed
string flight_mode
float32 command_limit
string reason
