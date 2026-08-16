# ROV 赛前自主抓取候选版 `0.2.0rc1`

这是基于现有工程生成的独立候选版。它不会修改旧目录，也没有替换 Ubuntu 上正在使用的版本。

本版的目标是先把“一次单目标自主任务”做成结构清楚、默认安全、能够离线验证的工程：

> 完全浸没后启动 → 相对下潜 0.30 m → 向右扫描 360° → 无目标时估算前进 0.40 m → 再扫描 → 对准 → 接近 → 闭爪 → 上升到绝对深度 0.30 m → 正常上锁结束

重要限制：这台 ROV 目前没有 DVL 或其他水下位置来源。深度和转角可以使用真实传感器闭环；“前进 0.40 m”只能按同工况实测平均速度换算时间，日志中始终标为估算值，不能当成真实位移。

## 1. 先看结论

- 默认配置不会让推进器或机械爪动作。
- 默认配置不会自动切换模式，也不会自动解锁。
- 摄像头控制方向、水平运动、抓取阈值尚未标定时，识别和离线回放可以运行，真实自主启动会列出所有缺项并拒绝。
- Python 只发送前后、横移、升沉、偏航四种运动意图；八推进器混控仍由 ArduSub 完成。
- QGC 手柄、单轴测试和自主节点不能同时控制。`control.profile` 一次只能选择 `commissioning` 或 `autonomy`，修改后必须重启网关。
- `SetGripper` 服务成功只表示命令被网关接受，不代表没有反馈传感器的机械爪已经物理抓牢。
- `rov_replay` 永远不连接 MAVLink，画面固定显示 `SIMULATION`，不能作为实艇验收证据。

## 2. 工程入口

| 要做的事 | 文件或命令 |
|---|---|
| 调现场参数 | `ros2_ws/src/rov_competition/config/autonomy.yaml` 最前面的 `field_tuning` |
| 建立实艇配置 | 复制 `robot.example.yaml` 为根目录 `config/robot.yaml` 后填写 |
| 理解状态机 | [docs/状态机怎么写的，给hsy.md](docs/状态机怎么写的，给hsy.md) |
| 做水池标定 | [docs/参数标定，给hsy.md](docs/参数标定，给hsy.md) |
| 逐步实机测试 | [docs/实机测试，给wys.md](docs/实机测试，给wys.md) |
| 以后替换 Ubuntu 旧版 | [docs/怎么版本替换，给wys.md](docs/怎么版本替换，给wys.md) |
| 查看 ROS 接口 | [docs/ros接口，给wys.md](docs/ros接口，给wys.md) |
| 使用 Git 更新 | [docs/Git仓库使用.md](docs/Git仓库使用.md) |

## 3. Ubuntu 首次安装与构建

你们 Ubuntu 提示符里的实际用户名是小写 `persica`。Linux 路径区分大小写，因此应使用 `/home/persica`，不是 `/home/Persica`。

进入候选工程顶层后执行：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
chmod +x scripts/*.sh
./scripts/install_ubuntu_22_04.sh
```

安装脚本只支持 Ubuntu 22.04 + ROS 2 Humble。它会创建带系统 ROS 包的 `.venv`、安装依赖并构建工作空间，但不会打开实艇输出。

以后每开一个新终端，都先执行：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source ros2_ws/install/setup.bash
```

确认环境不是“看起来激活、实际串了系统 Python”：

```bash
which python
which colcon
python -c "import rclpy, yaml; print(rclpy.__file__); print(yaml.__version__)"
ros2 pkg executables rov_competition
```

应该能看到 `rov_axis_test`、`rov_turn_test`、`rov_autonomy`、`rov_vehicle` 等入口。

## 4. 先做完全离线验证

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
source .venv/bin/activate
./scripts/run_tests.sh
```

在 Ubuntu + ROS 2 Humble 上做完整的 `colcon build/test`、接口生成和命令入口检查：

```bash
./scripts/verify_ros2_humble.sh
```

然后用录像只验证模型和框：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 run rov_competition rov_replay \
  --video /你的/录像.mp4 \
  --config "$PWD/ros2_ws/src/rov_competition/config/autonomy.yaml" \
  --targets "$PWD/ros2_ws/src/rov_competition/config/grasp_targets.yaml" \
  --show
```

若还想观察虚拟深度和虚拟航向驱动的状态转换，再加 `--simulate-mission`。这仍然不连接飞控，也不证明实艇能完成动作。

## 5. 实艇测试只按这个层级升级

1. QGC 能读取 ArduSub，导出参数备份。
2. 在水中逐个核对 Motor1–Motor8、位置和推力方向。
3. QGC 手柄以最低增益验证前后、横移、升沉、偏航及断链保护。
4. ROS 网关只读遥测和预检。
5. `rov_axis_test` 从 `0.05、0.3s` 单动作开始。
6. `rov_turn_test` 固定按 `30° → 90° → 180° → 360°` 验收。
7. 标定无 DVL 的估算速度、图像控制方向和每类框面积阈值。
8. 最后才允许启动自主状态机。

不要跳级。任何一级出现方向相反、模式异常、急停/上锁失败、数据过期或断链后仍输出，都停止后续测试。

## 6. 两个新测试工具

下面的预览命令不会初始化 ROS，也不会创建发布器：

```bash
ros2 run rov_competition rov_axis_test --motion forward
ros2 run rov_competition rov_turn_test --direction right --angle 30
```

真实动作还要求 `--execute` 和确认词。例如：

```bash
ros2 run rov_competition rov_axis_test \
  --motion forward --value 0.05 --duration 0.3 \
  --config "$PWD/config/robot.yaml" \
  --execute --confirm "MOVE ROV"
```

```bash
ros2 run rov_competition rov_turn_test \
  --direction right --angle 30 --value 0.05 \
  --config "$PWD/config/robot.yaml" \
  --execute --confirm "TURN ROV"
```

工具不会自动解锁。执行前必须由人完成预检、运行时许可和正常 ROS 解锁；动作结束或 `Ctrl+C` 后，工具会连续发中位并请求正常上锁。因此下一项测试前需要重新开启许可、重新解锁。

旧的 `rov_motion_test` 入口保留用于兼容之前的命令，但新验收统一使用上面两个工具。

## 7. 自主启动的硬条件

真实自主开始前必须同时满足：

- `robot.yaml` 中 `control.profile: "autonomy"`；
- 飞控为 ArduSub，八推机架、输出和方向已实测；
- YAML、launch、运行时许可三道运动门均打开；
- 机械爪有独立授权；
- 只读预检通过；
- 操作员已将飞控设为 `ALT_HOLD`；
- 已通过 ROS 确认词正常解锁；
- 深度、姿态、心跳、网关状态和相机帧都新鲜；
- `autonomy.yaml` 两道任务权限、三项标定和两个图像方向均已填写；
- 当前没有 QGC 手柄或测试工具继续发送运动输入。

状态机不会帮人切模式或解锁。完整命令顺序见 [docs/实机测试，给wys.md](docs/实机测试，给wys.md)。

## 8. 最重要的停止条件

出现以下任一情况，立即松开控制、上锁，无法确认软件停车时物理断电：

- QGC 不能明确识别 ArduSub；
- `FRAME_CONFIG` 或 Motor1–Motor8 对应关系不确定；
- 任一轴方向与艇体坐标系相反；
- 深度、航向或相机数据跳变、过期；
- 飞控不在配置允许的模式；
- 目标框接近整屏、跳到另一个物体或只出现单帧；
- 急停、`Ctrl+C`、发布者崩溃或断链测试不能让输出停止并上锁；
- 人员、线缆、衣物或工具进入推进器危险区。

本候选版通过离线测试后，仍必须按“固定 ROV、两人值守、低功率、单轴、短时间、水中运行”的方式逐级验收。代码能启动不是正确性证据，真实方向、安全边界和通过标准必须由团队理解并记录。
