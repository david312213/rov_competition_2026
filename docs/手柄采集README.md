# 手柄驾驶录像：一键使用说明

## 先说结论

是的，这是“一键启动”版本。第一次配置好 QGroundControl（QGC）后，
每天采集只需要在一个终端输入：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_recording.sh
```

脚本会自动尝试打开 QGC，并启动：

```text
BlueOS ──→ 5600 ──→ QGC：给人和手柄操作员看
       └─→ 5700 ──→ 软件分流器 ──→ 5704 录像器
```

这个版本只负责视频分流和录像。它不启动 ROS 飞控网关、不解锁、
不发送运动命令、不加载 YOLO，也不控制机械爪。ROV 的运动、停车、
上锁和回收仍全部由 QGC 与手柄操作员负责。

## 第一次只配置一次

### 1. 确认手柄本来就能控制实艇

开始拍数据前，先单独在 QGC 中完成：

- 飞控连接正常，QGC 能看到 ArduSub 遥测；
- 手柄已校准，前后、横移、升沉和偏航方向正确；
- 松开摇杆后推进器停止；
- 人工上锁、失控保护和物理断电已经验证；
- ROV 已在水中，推进器附近无人。

本录像脚本不会替你验证这些控制安全项。

### 2. 设置艇载视频输出

BlueOS 给同一个摄像头配置两路 H.264 UDP 输出：

```text
udp://192.168.2.1:5600
udp://192.168.2.1:5700
```

若岸上有线网卡不是 `192.168.2.1`，必须改成实际地址。

### 3. 设置 QGC 视频

QGC 中设置：

```text
Video Source: UDP H.264
UDP Port:     5600（默认）
Low Latency:  开启
```

QGC 直接接收 `5600`，软件分流器独立接收 `5700`。脚本会检查两路是否
都在工作；录像故障不会占用或中断 QGC 的默认视频。

### 4. 编译一次工作空间

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

## 每次采集怎么做

### 1. 一条命令启动

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_recording.sh
```

如果 QGC 已经打开，脚本直接复用；如果没有打开，脚本会尝试自动打开。
首次没有找到 QGC 时，按屏幕提示手动打开即可。QGC 使用默认视频后，
日常启动通常不需要再输入第二条命令。

### 2. 用手柄驾驶和拍摄

看到 QGC 画面、终端显示录像开始后，再按你们已经验收过的 QGC 流程
解锁并驾驶。建议让目标出现在不同位置、尺度、角度、光照和背景下，
也要拍一些没有目标、浑水、反光和遮挡的负样本。

### 3. 正常停止

先完成机器人安全操作：

```text
松开手柄 → 确认 ROV 停稳 → 上锁或回收 → 再停止录像
```

然后回到启动脚本的终端按 `Enter`。也可以按 `Ctrl+C`；录像器会尽量
发送 EOS 并完整封装 MKV。但要特别注意：这两个操作只停止录像，
不会让机器人停车、上浮或上锁。

## 文件在哪里

每次运行会创建独立目录：

```text
output/datasets/YYYYMMDD_HHMMSS/
├── video_raw.mkv
├── session.json
└── logs/
```

异常断电或封装未完成时会保留 `video_raw.partial.mkv`。只有验证通过后
才会生成正式的 `video_raw.mkv`。

按每秒 5 帧抽图：

```bash
cd /home/persica/rov_competition_2026/output/datasets/YYYYMMDD_HHMMSS
mkdir -p frames
ffmpeg -i video_raw.mkv -vf "fps=5" -q:v 2 frames/%06d.jpg
```

抽帧后要人工删除重复、严重模糊和没有标注价值的图片。训练集、验证集
应按“不同录像片段”划分，不能把同一秒的相邻帧随机分到两边。

## 常见问题

### 提示某个视频端口没有数据

先确认 BlueOS 同一摄像头的两个输出都指向本机 `192.168.2.1`，再检查：

```bash
ss -lunp | grep -E ':(5600|5700|5704)\b'
```

正常录像时应看到：QGC 监听 `5600`、分流器监听 `5700`、录像器监听
`5704`。

### QGC 能看，录像却停止

先保证机器人安全，不要因为排查录像而继续驾驶。查看本次目录下的：

```text
logs/video_bridge.log
logs/recorder.log
```

QGC 与录像是两条独立分支；录像失败不代表 QGC 会自动停止控制。

### QGC 没有被自动打开

可以指定完整路径：

```bash
QGC_EXECUTABLE=/你的/QGroundControl \
  ./scripts/start_dataset_recording.sh
```

Flatpak 版会由脚本自动尝试使用 `org.mavlink.qgroundcontrol`。
