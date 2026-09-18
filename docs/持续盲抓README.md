# 永久沉底蛇形盲抓入口

入口命令是 `bash scripts/start_blind_grab.sh`，ROS安装入口是 `rov_blind_grab`。

完整动作是：

```text
首次下潜判底
→ 抓取并投筐
→ 上潜2秒
→ 蛇形前进一段
→ 下潜5秒
→ 再次抓取
→ 永久循环
```

视觉系统继续运行YOLO并显示识别框，但检测数量、位置、断帧和模型状态都不会改变运动状态。

## 1. 当前动作参数

配置模板位于 `ros2_ws/src/rov_competition/config/blind_grab.yaml`。首次使用可复制为本机配置：

```bash
cp ros2_ws/src/rov_competition/config/blind_grab.yaml config/blind_grab.local.yaml
```

### 升沉与蛇形

| 参数 | 当前值 | 行为 |
|---|---:|---|
| `vertical.initial_descent_command` | -0.8 | 启动后立即下潜 |
| `vertical.initial_bottom_stable_s` | 3秒 | 深度平台持续时间 |
| `vertical.initial_bottom_tolerance_m` | 0.05米 | 平台窗口内允许的深度波动 |
| `vertical.initial_minimum_descent_m` | 0.10米 | 压力判底前至少下降距离 |
| `vertical.initial_fallback_s` | 10秒 | 未取得有效判底结果时直接开始抓取 |
| `vertical.ascent_command` | +0.8 | 每次投放后的上潜指令 |
| `vertical.ascent_duration_s` | 2秒 | 每次固定上潜时间 |
| `vertical.repeat_descent_command` | -0.8 | 后续固定下潜指令 |
| `vertical.repeat_descent_duration_s` | 5秒 | 后续每次下潜时间 |
| `route.forward_command` | 0.23 | 蛇形前进控制量 |
| `route.step_duration_s` | 5秒 | 每次抓取之间的前进段 |
| `route.steps_per_lane` | 4 | 每带四段，共前进20秒 |
| `route.shift_command/duration_s` | 0.20 / 4.5秒 | 到带尾后的横移 |
| `route.turn_command/duration_s` | 0.20 / 11.5秒 | 横移后的转艇 |

第一条带使用负方向横移和转向，下一条带使用正方向，之后持续交替。第四个5秒前进段结束后，艇保持离底状态完成横移和转向，再下潜5秒并抓取。

### 抓取与PWM

每次底部动作固定为：

```text
张爪0.5秒
→ 前进0.23持续1秒
→ 闭爪0.5秒
→ 机械臂后仰到筐位1.8秒
→ 张爪投放2秒
→ 机械臂回垂直抓取位2秒
```

| 动作 | 飞控输出 | PWM |
|---|---:|---:|
| 张爪 | S11/AUX3 | 730 |
| 闭爪 | S11/AUX3 | 575 |
| 机械臂后仰到筐位 | S10/AUX2 | 1810 |
| 机械臂垂直向下抓取位 | S10/AUX2 | 710 |

S10的1300只作为机械臂水平参考，自动流程不发送该值。当前PWM与动作时间来自2026年9月17日实艇标定。

## 2. 状态顺序

| 状态 | 输出与转换 |
|---|---|
| `initial_descent` | 持续发送垂直 `-0.8`。压力深度下降至少0.10米后，在0.05米范围稳定3秒即判底；无深度或未稳定满条件时，第10秒直接进入抓取。 |
| `open` | 张爪730、机械臂710，运动归中，等待0.5秒。 |
| `advance` | 张爪保持730、机械臂保持710，前进0.23持续1秒。 |
| `close` | 停艇并闭爪575，等待0.5秒。 |
| `transfer` | 闭爪保持575，机械臂转到1810，等待1.8秒。 |
| `release` | 机械臂保持1810，张爪730投放，等待2秒。 |
| `return` | 张爪保持730，机械臂回710，等待2秒。 |
| `ascending` | 垂直 `+0.8` 持续2秒。 |
| `forward` | 前进0.23持续5秒。前三段结束后直接进入定时下潜；第四段后进入横移。 |
| `shift` | 按当前带方向横移0.20持续4.5秒。 |
| `turn` | 同方向转艇0.20持续11.5秒，然后切换下一条带。 |
| `repeat_descent` | 垂直 `-0.8` 持续5秒，再进入下一次抓取。 |

程序没有10次上限、批次结束、任务总时限、视觉门槛、最大深度终止或遥测失效终止状态。

## 3. 一键启动

启动前由操作员让艇完全入水，在QGC中设置模式并解锁，同时关闭其他会发送运动指令的程序。随后运行：

```bash
cd /home/persica/rov_competition_2026
# 一队，数据端口40198：
./scripts/start_blind_grab_team1.sh
# 二队ddhyzx2，数据端口40197：
./scripts/start_blind_grab_team2.sh
```

本入口不会自动切换模式或解锁。启动后不等待ROS、视频、模型、录像、查看器、官方服务器或MAVLink连接，状态机立即开始首次下潜计时。

只检查配置而不连接设备：

```bash
./scripts/start_blind_grab.sh --dry-run
```

已有独立视频和YOLO进程时：

```bash
./scripts/start_blind_grab.sh --no-helpers
```

使用 `--no-helpers` 也会关闭本入口管理的官方RTMP视频；此时需要由外部视频进程自行推流。

人工关闭使用 `Ctrl+C`。SIGTERM和终端关闭产生的SIGHUP也会结束循环，尝试发送三帧运动归中并释放控制。

## 4. 视觉与通信

默认识别类别是 `scallop`，置信度是0.18。视频链路保持为：

```text
艇端 → QGC 5600
艇端 → 软件 5700 ┬→ YOLO 5702 → /rov/detections和本机带框画面
                 ├→ 可选录像 5704
                 └→ 官方RTMP（队伍端口40198或40197）
```

`/rov/detections` 只用于终端显示框数和带框查看器。检测到0框、很多框、重复帧、断流或解析异常产生相同的运动序列。

视频分发、YOLO、查看器和可选录像由独立线程管理，启动失败或退出后持续重试。MAVLink发送失败时状态时钟继续，通信线程持续重连；恢复连接后发送当时状态的运动和舵机目标。

比赛官方数据链继续独立运行：20 Hz发布 `/cmd_vel`、`/cmd_accel`，5 Hz发布 `/robot_data`，有真实新鲜遥测时发布 `/imu`、`/magnetometer`、`/pressure`；本程序不伪造 `/joy`。视频分流同时把5700端口的原始H.264画面推到所选队伍的RTMP地址，本机查看器仍显示带框画面。官方ROS、TCP或RTMP失败只触发重试，不改变盲抓状态。

终端日志示例字段：

```text
state=repeat_descent phase=repeat_descent boxes=3 visual_control=false
bottom=pressure lane=2 segment=1/4 close_commands=8 cycles=8
```

`close_commands` 是状态机安排闭爪的次数，`cycles` 是完成整套抓取投放动作的次数，都不代表实际收获数量。

## 5. 旧电脑更新

```bash
cd /home/persica/rov_competition_2026
git fetch origin
git switch codex/continuous-blind-grab
git pull --ff-only origin codex/continuous-blind-grab
./scripts/prepare_blind_grab_old_pc.sh
```

旧的 `config/blind_grab.local.yaml` 可以保留。缺少 `vertical` 和 `route` 段时，加载器自动采用本页列出的新默认值；旧的 `trigger`、`required_boxes`、`fallback_after_s` 和 `grabs_per_batch` 不再参与控制。

臂爪单独测试：

```bash
./scripts/test_arm_gripper.sh
```

手动输入PWM试调：

```bash
./scripts/tune_arm_gripper_pwm.sh
```

官方数据链现场检查：

```bash
./scripts/check_official_ros.sh --live
```
