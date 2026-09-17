# ros2_topic_forwarding 来源说明

本目录来自比赛方在2026年9月16日提供的 `ros2_ws_src.zip`：

```text
SHA-256 1628ddcc4009af855c4b31943db8c0c58b34a58d00ba9a7b5e66f924587841cc
```

`src/topic_forwarding.cpp`、`msg/RobotDataMessage.msg` 和 `launch/forwarding.launch.py` 保持收到时的内容。`config/forwarding.yaml` 使用二队 `ddhyzx2` 的默认值 `api.bjetone.com:40197`；一队使用40198，由队伍启动脚本传入。为使其在干净的 Ubuntu 22.04 / ROS 2 Humble 环境中能够解析源码实际使用的 `<nlohmann/json.hpp>`，本工程只在 `CMakeLists.txt` 和 `package.xml` 补充了 `nlohmann_json` 构建依赖。

持续盲抓入口通过原可执行程序启动该节点，并传入 `blind_grab.local.yaml` 的 `official_ros.server_ip/server_port`。网络或该节点故障不会控制盲抓状态机。
