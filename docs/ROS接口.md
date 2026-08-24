# ROS 2 接口

## 1. 控制与遥测

| 名称 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/rov/control/command` | `rov_interfaces/msg/NormalizedMotionCommand` | 节点 → 网关 | 带时间戳、来源的四轴归一化运动意图 |
| `/rov/control/status` | `rov_interfaces/msg/ControlStatus` | 网关 → 全系统 | `LOCKED/READY/ACTIVE/ESTOPPED`、预检、权限、ROS 解锁、模式和拒绝原因 |
| `/rov/telemetry` | `rov_interfaces/msg/RobotTelemetry` | 网关 → 全系统 | 真实深度、姿态、电池、心跳、模式及有效性标志 |
| `/rov/control/set_enabled` | `std_srvs/srv/SetBool` | 操作员 → 网关 | 运行时控制许可；开启前要求飞控明确上锁、预检通过 |
| `/rov/control/set_armed` | `rov_interfaces/srv/SetArmed` | 操作员/任务收尾 → 网关 | 解锁要求确认词 `ARM ROV`；上锁不要求确认词 |
| `/rov/control/emergency_stop` | `std_srvs/srv/Trigger` | 任意安全层 → 网关 | 回中、尝试正常上锁、释放并锁存急停 |
| `/rov/control/reset_emergency_stop` | `std_srvs/srv/Trigger` | 操作员 → 网关 | 仅在明确上锁、心跳新鲜、预检通过时复位 |
| `/rov/control/set_gripper` | `rov_interfaces/srv/SetGripper` | 自主节点 → 网关 | 带时间戳、来源和 `OPEN/CLOSE` 枚举；执行 `robot.yaml` 当前机械爪档案，成功只表示命令序列已开始，不表示已抓牢 |

## 2. 感知与任务

| 名称 | 类型 | 说明 |
|---|---|---|
| `/rov/detections` | `rov_interfaces/msg/TargetDetectionArray` | 当前新图像的全部识别框 |
| `/rov/annotated_image/compressed` | `sensor_msgs/msg/CompressedImage` | 带全部框、锁定框、瞄准点、面积、k、深度和扫描进度的画面 |
| `/rov/mission/status` | `rov_interfaces/msg/MissionStatus` | 状态、结果、目标、面积/k、中心误差、深度、扫描角和估算前进距离 |
| `/rov/mission_state` | `std_msgs/msg/String` | 旧显示程序使用的兼容状态话题 |
| `/rov/mission/start` | `std_srvs/srv/Trigger` | 完成全部启动检查后开始任务 |
| `/rov/mission/abort` | `std_srvs/srv/Trigger` | 中止任务并请求网关急停 |

`/rov/autonomy/start` 和 `/rov/autonomy/abort` 暂时保留为旧命令兼容别名。

`start_grasp_position_test.sh` 共用上面的控制、遥测、感知和机械爪接口。
自动对准完成后，测试节点继续以唯一的 `commissioning` 来源发布 WASD
运动命令；QGC 此时只观察和保留人工上锁能力。`Enter/G → C → Y/N` 的
顺序分别对应“保存闭爪前证据 → 请求闭爪 → 人工确认实物结果”。

`start_gripper_test.sh dalian|rst` 是独立的上锁台架工具：它不启动 ROS
飞控网关、不创建运动发布器、不调用解锁服务，而是通过独立 MAVLink 端点
发送候选舵机命令并逐条等待 ACK。它仍要求 ROS 环境只是为了复用命令入口，
但不新增 ROS 话题或服务。

带框图像通过独立 ROS 话题查看：

```bash
ros2 run rqt_image_view rqt_image_view
```

在界面中选择基础话题 `/rov/annotated_image`，传输方式选择
`compressed`；不得直接订阅带 `/compressed` 后缀的传输子话题。不得在自主节点中启用
`display_window=true`；OpenCV HighGUI 会阻塞 ROS 多线程回调和节点退出。

## 3. 来源互斥

- `control.profile: commissioning` 时，网关只接受来源 `commissioning`；用于 `rov_axis_test`、`rov_turn_test` 和机械爪标定。
- `control.profile: autonomy` 时，网关只接受来源 `autonomy`；用于自主节点。
- 网关运行期间不能动态切换。切换前必须回中、上锁、停止网关，修改 YAML 后重启。

错误来源、过期时间戳、NaN/Inf 或越界值会被拒绝；活动解锁状态下的非法控制输入会升级为急停。

## 4. 查看证据

```bash
ros2 topic echo /rov/control/status
ros2 topic echo /rov/telemetry
ros2 topic echo /rov/mission/status
ros2 topic hz /rov/detections
ros2 topic hz /rov/control/command
ros2 topic hz /rov/annotated_image/compressed
```

`/rov/detections` 应接近相机实际帧率。`/rov/mission/status` 用于确认任务是否
真的处于活动状态、当前目标和深度是否有效；话题存在本身不等于自主运动已启动。

保存 rosbag、网关/自主终端完整日志、QGC 参数备份和同步录像。单独一行“节点启动成功”不能证明控制、状态机或安全条件正确。
