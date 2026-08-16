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
| `/rov/control/set_gripper` | `rov_interfaces/srv/SetGripper` | 自主节点 → 网关 | 带时间戳、来源和 `OPEN/CLOSE` 枚举；响应只说明是否接受 |

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

## 3. 来源互斥

- `control.profile: commissioning` 时，网关只接受来源 `commissioning`；用于 `rov_axis_test`、`rov_turn_test` 和兼容 `rov_motion_test`。
- `control.profile: autonomy` 时，网关只接受来源 `autonomy`；用于自主节点。
- 网关运行期间不能动态切换。切换前必须回中、上锁、停止网关，修改 YAML 后重启。

错误来源、过期时间戳、NaN/Inf 或越界值会被拒绝；活动解锁状态下的非法控制输入会升级为急停。

## 4. 查看证据

```bash
ros2 topic echo /rov/control/status
ros2 topic echo /rov/telemetry
ros2 topic echo /rov/mission/status
ros2 topic hz /rov/control/command
ros2 topic hz /rov/annotated_image/compressed
```

保存 rosbag、网关/自主终端完整日志、QGC 参数备份和同步录像。单独一行“节点启动成功”不能证明控制、状态机或安全条件正确。
