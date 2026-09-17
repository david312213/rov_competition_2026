# 官方 ROS 转发与旧电脑启动

本工程已纳入比赛方提供的 `ros2_topic_forwarding` ROS 2 包，并由持续盲抓入口自动管理。比赛方文档和源码只作为接口依据；盲抓状态、动作和故障策略仍以本工程配置为准。

## 已接入的数据

盲抓启动后持续发布并由官方节点转发：

| 官网面板 | 盲抓期间发送内容 | 频率与条件 |
|---|---|---|
| `cmd_vel` | 实际下潜、上潜、前进、横移和转向控制量；`linear.x/y/z` 和 `angular.z` | 20 Hz，始终发布 |
| `cmd_accel` | 相邻控制周期内 `cmd_vel` 的变化率 | 20 Hz，始终发布 |
| `robot_data` | 姿态、经纬度、深度、速度、电池、磁场模长、加速度模长和时间 | 5 Hz，始终发布；没有新鲜来源的字段为0 |
| `imu` | 姿态四元数、角速度、线加速度 | 收到真实且未过期的 MAVLink 数据时发布 |
| `magnetometer` | 三轴磁场 | 收到真实且未过期的 MAVLink 数据时发布 |
| `pressure` | 压力值 | 收到真实且未过期的 MAVLink 数据时发布 |
| `joy` | 本程序不伪造手柄数据；如果另一个真实手柄ROS节点发布 `/joy`，官方节点会转发 | 有真实 `/joy` 时才显示 |
| `Ros视频` | 源端口5700上的H.264画面，经现有视频分流同时推到官方RTMP地址 | 视频源和网络可用时持续推流 |

`/robot_data` 没有有效性标志。舱湿、舱温、舱压和爪电流当前没有可靠来源，因此保持0。官网视频是进入YOLO前的原始画面；本机查看器继续显示带框的 `/rov/annotated_image/compressed`。视频、模型或官网推流失败不改变盲抓动作。

两队端口固定记录为：

| 队伍 | ROS数据TCP | ROS视频RTMP |
|---|---|---|
| 一队 | `api.bjetone.com:40198` | `rtmp://api.bjetone.com/ros/40198` |
| 二队（`ddhyzx2`） | `api.bjetone.com:40197` | `rtmp://api.bjetone.com/ros/40197` |

二队是当前默认值。正式启动应使用对应的队伍脚本，脚本会先同步 `config/blind_grab.local.yaml`，避免旧电脑仍使用历史端口40184。

官方数据发布线程、TCP转发节点、视频推流和盲抓控制互相独立。ROS初始化失败、服务器断线、视频失败或官方节点退出时会记录并重试，不会取消或暂停盲抓。

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
cd /home/persica/rov_competition_2026
# 一队：
./scripts/start_blind_grab_team1.sh
# 二队（ddhyzx2）：
./scripts/start_blind_grab_team2.sh
```

程序会同时启动永久沉底蛇形盲抓、带框检测画面、官方 ROS 发布器和官方 TCP 转发节点。启动后立即下潜，检测框不参与控制；任务只在本终端按 `Ctrl+C` 或结束进程时关闭。

## 计分数据现场确认

启动后保留主终端，另开一个终端运行：

```bash
cd /home/persica/rov_competition_2026 && ./scripts/check_official_ros.sh --live
```

必须看到 `/cmd_vel`、`/cmd_accel`、`/robot_data` 三个固定话题都在发布、`/topic_forwarding` 节点存在，并且到所选队伍端口的 TCP 状态为 `ESTABLISHED`。`/imu`、`/magnetometer`、`/pressure` 和 `/joy` 会逐项报告当前是否有真实来源。该检查通过表示本机发布与官方 TCP 链路正常；最终平台是否入库仍以裁判平台页面显示为准。

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
