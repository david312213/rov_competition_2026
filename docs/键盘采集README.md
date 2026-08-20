# 键盘驾驶录像：一键使用说明

## 先说结论

是的，这是“一键启动”版本。第一次填写好安全配置后，每次只在一个
终端输入：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_collection.sh
```

它会依次启动 MAVLink 分流、QGC、ROS 飞控网关、视频分流、原始录像器
和键盘控制窗口。它不加载 YOLO、不需要权重、不控制机械爪。

“一键”不等于绕过安全：首次必须核对水池最大深度；每次实艇启动仍要
确认 `ALT_HOLD`、遥测、预检、录像和现场安全，并输入确认词。

## 一次配置

### 1. 实艇控制配置

`config/robot.yaml` 至少应使用已经通过实艇点动的配置：

```yaml
mavlink:
  connection_uri: "udpin:0.0.0.0:14551"

control:
  profile: "commissioning"
  allowed_flight_modes: ["ALT_HOLD"]
  command_limit: 0.10

safety:
  allow_live_actuation: true
  allow_ros_arming: true
  allow_gripper_actuation: false
```

不要照抄示例里的电机方向或飞控参数。这里必须使用你们已经完成水中
单轴、松键停止、急停和断链测试的实艇配置。

### 2. 核对当天水池最大深度

第一次直接运行一键脚本，它会自动生成：

```text
config/dataset.yaml
```

当前场地水深至少 `1.50 m`。为了不再把下潜限制在启动后的
`0.50 m`，仓库默认生成：

```yaml
depth_safety:
  maximum_depth_m: 1.40
  maximum_descent_from_start_m: 1.20
```

这个数值只适用于当前水池。更换场地、改变载荷或无法保证底部余量时，
必须重新测量并修改。实际下潜限制取下面两者中更浅的值：

```text
水池绝对最大深度
启动深度 + maximum_descent_from_start_m（默认 1.20 m）
```

### 3. QGC 设置

一键脚本使用固定端口：

```text
MAVLink 14550 → ROS 14551 + QGC 14552
Video   5600  → QGC 5701 + Recorder 5702
```

QGC 只设置一次：

- MAVLink UDP 监听端口：`14552`；
- 视频：`UDP H.264`，端口 `5701`；
- 开启 Low Latency；
- 删除会直接抢占 `14550` 或 `5600` 的旧自动连接。

QGC 保留作观察和人工上锁，不要同时使用 QGC 手柄与键盘抢控制权。

### 4. 编译一次

```bash
cd /home/persica/rov_competition_2026
source /opt/ros/humble/setup.bash
source .venv/bin/activate
export PYTHONNOUSERSITE=1

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..

./scripts/check_ros.sh
```

## 每次采集

### 1. 开机前安全条件

- ROV 已完全在水中，推进器附近无人；
- 一人操作键盘，一人只负责 QGC 上锁和物理断电；
- 飞控模式由操作员手动设为 `ALT_HOLD`；
- 深度数据真实变化且方向正确；
- QGC 能观察飞控并可人工上锁；
- 机械爪权限保持关闭。

### 2. 一条命令启动

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_collection.sh
```

脚本会尝试打开 QGC，并在终端逐项显示环境、端口、视频、遥测与预检
结果。按提示完成现场确认，最后输入：

```text
START DATASET ROV
```

程序才会申请控制许可和解锁。不要把确认词写进脚本或做成自动输入。

### 3. 键位

| 按键 | 动作 |
|---|---|
| `W / S` | 前进 / 后退 |
| `A / D` | 左移 / 右移 |
| `1 / 2` | 左转 / 右转 |
| `↑ / ↓` | 上升 / 下潜 |
| `+ / -` | 指令每次增加 / 减少 `0.01` |
| `Space` | 四轴立即回中，保持当前深度 |
| `0` | 正常停录像、回启动深度并上锁 |
| `Esc`、关闭窗口、`Ctrl+C` | 急停并尝试上锁 |

默认指令为 `0.05`，上限为 `0.10`。只有按住才运动，松开立即回中；
窗口失去焦点也立即回中。允许组合键，但组合向量会统一缩放，不能绕过
网关限幅。

工具不会自动沉底。必须由操作员按 `↓` 手动下潜，而且深度保护始终生效。

### 4. 正常结束一定按 0

按 `0` 后程序会执行：

```text
四轴回中 → 录像器 EOS → 验证 MKV → 仅上升回启动深度 → 上锁
```

若机器人已经比启动深度浅，不会为了追目标深度再次下潜。

`Esc`、关闭窗口或 `Ctrl+C` 是异常急停路径：程序立即回中、锁存急停并
尝试上锁，不会在深度反馈不可靠时盲目上升。

## 文件在哪里

```text
output/datasets/YYYYMMDD_HHMMSS/
├── video_raw.mkv
├── events.csv
├── session.json
└── logs/
```

- `video_raw.mkv`：相机原始 H.264 封装，不重新编码；
- `events.csv`：键盘事件、四轴指令、深度、航向和模式；
- `session.json`：Git 版本、配置哈希、深度限制、开始/结束时间与退出原因；
- `logs/`：MAVProxy、网关、预检和视频分流日志。

抽帧示例：

```bash
cd /home/persica/rov_competition_2026/output/datasets/YYYYMMDD_HHMMSS
mkdir -p frames
ffmpeg -i video_raw.mkv -vf "fps=5" -q:v 2 frames/%06d.jpg
```

## 出问题先做什么

推进器行为、方向、模式或深度任何一项异常：先按 `Space`；仍不确定就
按 `Esc`，安全员同时在 QGC 上锁，必要时执行物理断电。不要先研究终端
报错再处理正在运动的机器人。

脚本提示端口占用时查看：

```bash
ss -lunp | grep -E ':(14550|14551|14552|5600|5701|5702)\b'
```

它只会清理自己启动的后台进程，不会擅自关闭 QGC。旧测试脚本或旧
MAVProxy 仍在运行时，先回原终端安全退出，再重新执行一键命令。
