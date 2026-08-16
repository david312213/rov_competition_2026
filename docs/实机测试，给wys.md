# 实机测试：从只读到自主

## 0. 人员和机械条件

- ROV 必须在水中，不能让水下推进器长时间干转。
- 机械固定方式不能阻碍紧急断电，线缆不能进入桨叶。
- 一人负责电脑和命令；另一人只负责观察危险区、急停和断电。
- 每发一个命令前，操作员口述“预期哪个方向动、什么条件立即停”。
- QGC 保持遥测和人工上锁能力，但 ROS 测试时关闭手柄输出。

## 1. 每个终端先加载环境

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source ros2_ws/install/setup.bash
```

## 2. 创建实艇配置

只做一次复制，之后只编辑根目录这一份：

```bash
cp ros2_ws/src/rov_competition/config/robot.example.yaml config/robot.yaml
```

先保持：

```yaml
control:
  profile: "commissioning"
  allowed_flight_modes: ["MANUAL"]

safety:
  allow_live_actuation: false
  allow_ros_arming: false
  allow_gripper_actuation: false
```

根据 QGC 参数备份和实物确认 `expected_frame_config`、Motor1–Motor8、方向、failsafe、机械爪绝对输出口和深度换算。旧六推进器日志不能直接导入。

## 3. 只读网关与预检

终端 A：

```bash
ros2 launch rov_competition telemetry.launch.py \
  robot_config:="$PWD/config/robot.yaml"
```

预期日志包含 MAVLink 已连接、真实输出 `DISABLED`。保持这个终端运行，不要在同一个终端输入下一条命令。

终端 B：

```bash
ros2 topic echo /rov/telemetry
```

```bash
ros2 topic echo /rov/control/status
```

确认数据来自真实飞控，深度、航向、模式、解锁状态会随实物变化。`Ctrl+C` 只结束当前 `echo`，不会关闭终端 A。

## 4. 打开点动所需的三道门

先停止终端 A 的网关并确认飞控上锁。编辑 `config/robot.yaml`：

```yaml
control:
  profile: "commissioning"
  allowed_flight_modes: ["MANUAL"]
  expected_frame_config: 2  # 这里只是格式示例，必须以实艇为准

safety:
  allow_live_actuation: true
  allow_ros_arming: true
  allow_gripper_actuation: false
```

重新启动网关：

```bash
ros2 launch rov_competition telemetry.launch.py \
  robot_config:="$PWD/config/robot.yaml" \
  enable_actuation:=true \
  enable_ros_arming:=true
```

网关会运行只读预检。任何关键项失败都不继续。

终端 B 开启运行时许可：

```bash
ros2 service call /rov/control/set_enabled std_srvs/srv/SetBool "{data: true}"
```

在 ROV 固定、危险区清空、飞控为 `MANUAL` 后，正常解锁：

```bash
ros2 service call /rov/control/set_armed rov_interfaces/srv/SetArmed \
  "{arm: true, confirmation: 'ARM ROV'}"
```

不要使用 `21196` 强制解锁。预解锁检查失败就解决原因。

## 5. 单轴点动

第一次只做：

```bash
ros2 run rov_competition rov_axis_test \
  --motion forward --value 0.05 --duration 0.3 \
  --config "$PWD/config/robot.yaml"
```

这是预览。两人再次确认后才执行：

```bash
ros2 run rov_competition rov_axis_test \
  --motion forward --value 0.05 --duration 0.3 \
  --config "$PWD/config/robot.yaml" \
  --execute --confirm "MOVE ROV"
```

工具结束后会请求正常上锁。按“前、后、左、右、上、下”逐项测试；每项都要重新开启运行时许可和解锁，不提供一键完整动作序列。

记录表：

| 动作 | 指令/时长 | 实际方向 | 松开后停止 | 上锁成功 | 深度/航向起止 | 录像文件 | 结论 |
|---|---:|---|---|---|---|---|---|
| 前 | 0.05 / 0.3s |  |  |  |  |  |  |
| 后 | 0.05 / 0.3s |  |  |  |  |  |  |
| 左 | 0.05 / 0.3s |  |  |  |  |  |  |
| 右 | 0.05 / 0.3s |  |  |  |  |  |  |
| 上 | 0.05 / 0.3s |  |  |  |  |  |  |
| 下 | 0.05 / 0.3s |  |  |  |  |  |  |

工具记录的是深度、航向、执行时间和命令；没有位置传感器时，不把肉眼看到的水平位移冒充传感器测量。

## 6. 转向闭环

固定顺序：右转 `30° → 90° → 180° → 360°`。每一级通过方向、停止、上锁和跨零检查后才进下一级。

```bash
ros2 run rov_competition rov_turn_test \
  --direction right --angle 30 --value 0.05 \
  --config "$PWD/config/robot.yaml"
```

```bash
ros2 run rov_competition rov_turn_test \
  --direction right --angle 30 --value 0.05 \
  --config "$PWD/config/robot.yaml" \
  --execute --confirm "TURN ROV"
```

工具使用真实航向累计角度。航向跳变、实际方向相反、长期无进展、遥测过期、总超时或 `Ctrl+C` 都停止并上锁。

## 7. 切换到自主档案

只有单轴、转向、断链和参数标定全部通过后：

1. 停止网关并确认上锁。
2. 将 `robot.yaml` 改为 `control.profile: "autonomy"`。
3. 将允许模式改为 `allowed_flight_modes: ["ALT_HOLD"]`。
4. 确认 `allow_gripper_actuation: true`，并核对绝对输出口与开合 PWM。
5. 按 [参数标定，给hsy.md](参数标定，给hsy.md) 完成 `autonomy.yaml` 的所有门。

终端 A 启动网关、感知和自主节点：

```bash
ros2 launch rov_competition competition.launch.py \
  robot_config:="$PWD/config/robot.yaml" \
  autonomy_config:="$PWD/ros2_ws/src/rov_competition/config/autonomy.yaml" \
  targets_config:="$PWD/ros2_ws/src/rov_competition/config/grasp_targets.yaml" \
  video_source:=0 \
  enable_actuation:=true \
  enable_ros_arming:=true
```

终端 B 观察：

```bash
ros2 topic echo /rov/mission/status
```

确认 QGC 已设为 `ALT_HOLD`，ROV 已完全浸没，依次调用运行时许可和正常解锁，然后开始任务：

```bash
ros2 service call /rov/control/set_enabled std_srvs/srv/SetBool "{data: true}"
```

```bash
ros2 service call /rov/control/set_armed rov_interfaces/srv/SetArmed \
  "{arm: true, confirmation: 'ARM ROV'}"
```

```bash
ros2 service call /rov/mission/start std_srvs/srv/Trigger "{}"
```

若返回失败，完整保存并阅读缺项列表。不要通过改代码删除门控。

人工中止：

```bash
ros2 service call /rov/mission/abort std_srvs/srv/Trigger "{}"
```

独立网关急停：

```bash
ros2 service call /rov/control/emergency_stop std_srvs/srv/Trigger "{}"
```

## 8. 必测故障

在固定和低功率条件下分别验证：

- 自主节点 `Ctrl+C`；
- 命令发布者进程崩溃；
- 摄像头断流；
- MAVLink 链路断开；
- 模式从 `ALT_HOLD` 变化；
- 深度或航向消息停止更新；
- 机械爪服务拒绝；
- 人工任务中止和网关急停。

每一种都必须保存终端完整日志、QGC 状态和录像。无法确认推进器停止并上锁时，不调试软件，先物理断电。
