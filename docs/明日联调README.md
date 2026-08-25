# 0.2.0rc2 明日联调一键向导

> 本文是明天现场操作的唯一主入口。不要把六个阶段一次性无人值守运行。

## 0. 拉取、构建和检查

在 Ubuntu 22.04 的工程根目录执行：

```bash
cd /home/persica/rov_competition_2026
git pull --ff-only

source /opt/ros/humble/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1

./scripts/check_ros.sh
```

`check_ros.sh` 必须全部通过。它会重新构建 ROS 2 工作空间，因此新增的
`rov_field_setup` 和联调入口才会出现。

先只看状态，不会驱动实艇：

```bash
./scripts/start_tomorrow_test.sh status
```

无参数可打开菜单：

```bash
./scripts/start_tomorrow_test.sh
```

## 1. 恢复 QGC 和 BlueOS 默认端口

```bash
./scripts/start_tomorrow_test.sh qgc
```

向导会尝试打开 `http://192.168.2.2` 和 QGroundControl，但不会自动修改外部软件配置。

### BlueOS 只配一次

MAVLink Endpoints 保留两个 `UDP Client`：

- `192.168.2.1:14550`：QGC。
- `192.168.2.1:14551`：ROS 飞控网关。
- 删除或停用旧的 `14552`。

同一个 H.264 摄像头输入添加两个 UDP 输出：

- `udp://192.168.2.1:5600`：QGC 默认画面。
- `udp://192.168.2.1:5700`：识别和录像软件。

### QGC 只配一次

1. 删除手动添加的 `14552` Comm Link。
2. 开启 UDP 自动连接；QGC 应在 `14550` 自动收到遥测。
3. `Video Source` 选 `UDP h.264`。
4. UDP 地址/端口设为 `0.0.0.0:5600`。
5. 开启 `Low Latency Mode`。

当 QGC 同时有实时遥测和实时画面、人工上锁按钮可用时，再在终端完整输入：

```text
QGC READY
```

向导随后会验证 QGC 监听 `14550/5600`，软件端口
`14551/5700/5702/5704` 无冲突，并临时接收一个 `14551` 包和一个
`5700` 视频包。这一步不启动 ROS，不解锁。

## 2. 判断哪套机械爪可用

### 测试前的物理条件

- 飞控必须显示为上锁。
- 必须物理断开八个推进器的动力或信号。
- 机械爪夹持区无人，安全员可立即断电。

先测大连方案：

```bash
./scripts/start_tomorrow_test.sh gripper dalian
```

如果实物不动，先上锁、断电、核对接线，再测 RST：

```bash
./scripts/start_tomorrow_test.sh gripper rst
```

RST 历史档案包含 `500 us`，因此还会要求第二个危险确认词。

工具只有在下列证据全部存在时才显示激活选项：

- `result.json` 为 `outcome=passed`；
- 操作员确认实物确实开爪且闭爪；
- `commands.csv` 的每条 `COMMAND_ACK` 都是接受。

最后还必须完整输入下列其中一个，才会备份并修改 `config/robot.yaml`：

```text
ACTIVATE DALIAN GRIPPER
ACTIVATE RST GRIPPER
```

失败、中断、ACK 不完整或确认词不匹配时，`robot.yaml` 不会被改动。

## 3. 安全替换新权重

```bash
./scripts/start_tomorrow_test.sh weights
```

把你们自己训练的 `best.pt` 拖进终端，或输入完整路径。路径中可以有空格。

安装前必须通过：

- CUDA 可用，且模型能在 GPU 完成一次试推理；
- 包含 `echinus` 、`holothurian` 、`scallop` 、`starfish`；
- 复制前后 SHA-256 一致。

`.pt` 可能包含 Python pickle，只能选择自己训练或确认可信的文件。
预览验证通过后，输入：

```text
INSTALL NEW WEIGHT
```

工具会保留 `seafood_yolo26x.previous.pt`，并生成被 Git 忽略的
`config/autonomy.local.yaml`。搜索、标定和视频识别都优先使用这份现场配置。

新模型有问题时回滚：

```bash
./scripts/start_tomorrow_test.sh weights rollback
```

确认词是 `ROLLBACK WEIGHT`。权重和对应的类别/哈希配置会一起交换，
不会只回滚其中一个。

## 4. 为旧电脑应用平衡型超时

`config/robot.yaml` 和 `config/dataset.yaml` 被 Git 忽略，所以 `git pull`
不会替你更新这两份实艇配置。执行：

```bash
./scripts/start_tomorrow_test.sh timing
```

向导会显示每个旧值和新值，然后要求完整输入：

```text
APPLY BALANCED TIMEOUTS
```

它会先在 `config/` 中生成带时间的备份，再原子写入新阈值并
重新加载校验。该阶段不连接飞控，不修改 `FS_PILOT_TIMEOUT`、
GCS 失控动作或任何 ArduSub 参数。`0.5 s` 运动发布者失联停车也保持
不变。随后可用下列命令复查：

```bash
./scripts/start_tomorrow_test.sh status
```

状态中出现 `timing: OK` 才表示这台电脑的本地配置已迁移。

## 5. 先测自动搜索、对准和接近

```bash
./scripts/start_tomorrow_test.sh search
```

该阶段的流程是：

```text
输入下潜power -> 下潜至深度连续3秒稳定 -> 右转360度扫描 -> 确认目标
-> 偏航对准 -> 自动接近 -> 面积阈值停车
-> 回到启动深度 -> 上锁
```

即使第 2 步已经启用机械爪，这个入口也会在本次会话的
`resolved_robot.yaml` 中强制关闭爪子权限，因此自动接近测试不会闭爪。

## 6. 人工抓取位置标定

```bash
./scripts/start_tomorrow_test.sh calibrate
```

自动流程只到“稳定对准”，然后四轴回中，进入人工标定：

| 按键 | 功能 |
|---|---|
| `W/S` | 前进/后退 |
| `A/D` | 左移/右移 |
| `1/2` | 左转/右转 |
| `↑/↓` | 上升/下潜 |
| `+/-` | 调节人工 power |
| `Space` | 立即回中 |
| `Enter` 或 `G` | 保存当前框、深度、姿态和截图 |
| `C/O` | 闭爪/开爪 |
| `Y/N` | 记录本次抓取成功/失败 |
| `B` | 记录为明显不合格位置 |
| `R` | 回到本轮搜索深度并重新扫描 |
| `0` | 正常回收、上锁并结束 |
| `Esc`/关窗口 | 急停、上锁 |

`C` 只有在先按 `Enter/G` 保存位置后才有效。如果机械爪激活记录缺失、
证据被改动或与当前档案不匹配，仍可以保存位置，但 `C/O` 会被禁用。

## 中止与进程边界

- 每个一键模式只清理它自己启动的 ROS/视频子进程。
- 向导不会关闭 QGC。
- QGC 始终保留人工上锁能力；安全员始终保留物理断电能力。
- `Space` 是暂停/回中，`0` 是正常回收，`Esc` 是急停，三者不是同一件事。
