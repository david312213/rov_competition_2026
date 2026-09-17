# 官方 ROS 转发与旧电脑启动

本工程已纳入比赛方提供的 `ros2_topic_forwarding` ROS 2 包，并由持续盲抓入口自动管理。比赛方文档和源码只作为接口依据；盲抓状态、动作和故障策略仍以本工程配置为准。

## 已接入的数据

盲抓启动后持续发布并由官方节点转发：

- `/cmd_vel`：当前实际发送方向对应的 `geometry_msgs/Twist`，20 Hz；
- `/cmd_accel`：上述指令的时间导数，20 Hz；
- `/robot_data`：官方 `ros2_topic_forwarding/msg/RobotDataMessage`，5 Hz；
- `/imu`、`/magnetometer`、`/pressure`：仅在同一 MAVLink 链路收到真实且未过期的数据时发布。

`/robot_data` 没有有效性标志。姿态、深度、位置、速度、电池、磁场和加速度使用实际 MAVLink 数据；没有来源的舱温、舱湿、舱压和爪电流保持 0。自动运动不会伪装成 `/joy` 手柄消息。

官方 TCP 节点默认连接 `api.bjetone.com:40184`，这与收到的官方包配置一致。如果比赛平台重新分配端口，只修改 `config/blind_grab.local.yaml` 的 `official_ros.server_port`。

官方发布线程、TCP 节点和盲抓控制互相独立。ROS 初始化失败、服务器断线或官方节点退出时会记录并重试，不会取消或暂停盲抓。

比赛方说明中的“Ros视频”使用单独的 RTMP/RTSP 推流，不经过 `ros2_topic_forwarding`。本次接入解决的是官方ROS数据面板；现有QGC/YOLO UDP视频链路不会自动变成平台RTMP视频。

## 旧电脑首次更新

旧电脑要求 Ubuntu 22.04 和 ROS 2 Humble。已有工程时：

```bash
cd /home/persica/rov_competition_2026
git fetch origin
git switch codex/continuous-blind-grab
git pull --ff-only origin codex/continuous-blind-grab
./scripts/prepare_blind_grab_old_pc.sh
```

没有工程时：

```bash
cd /home/persica
git clone --branch codex/continuous-blind-grab --single-branch \
  https://github.com/david312213/rov_competition_2026.git
cd rov_competition_2026
./scripts/prepare_blind_grab_old_pc.sh
```

准备脚本会安装官方 C++ 节点需要的 `nlohmann-json3-dev`、构建整个 ROS 工作空间、保留已有本机盲抓配置，并执行官方接口离线检查。它不会连接或驱动实艇。

## 正式一键启动

启动前由操作员让艇完全入水，并完成飞控模式设置和解锁。随后只运行：

```bash
cd /home/persica/rov_competition_2026 && ./scripts/start_blind_grab.sh
```

程序会同时启动永久沉底蛇形盲抓、带框检测画面、官方 ROS 发布器和官方 TCP 转发节点。启动后立即下潜，检测框不参与控制；任务只在本终端按 `Ctrl+C` 或结束进程时关闭。

## 计分数据现场确认

启动后保留主终端，另开一个终端运行：

```bash
cd /home/persica/rov_competition_2026 && ./scripts/check_official_ros.sh --live
```

必须看到三个话题都在发布、`/topic_forwarding` 节点存在，并且到配置端口的 TCP 状态为 `ESTABLISHED`。该检查通过表示本机发布与官方 TCP 链路正常；最终平台是否入库仍以裁判平台页面显示为准。

官方转发日志保存在本次会话目录的 `official_ros_forwarder.log`。主终端会打印该目录路径。

## 只检查配置

以下命令不连接 MAVLink、不初始化 ROS，也不启动任何辅助进程：

```bash
./scripts/start_blind_grab.sh --dry-run
```

离线检查官方包是否已经正确构建：

```bash
./scripts/check_official_ros.sh --offline
```
