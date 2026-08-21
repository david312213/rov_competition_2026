# ROV 比赛工作空间

当前版本：`0.2.0rc2`。这是赛前候选版，默认不会让推进器或机械爪动作。

任务流程：

> 完全浸没 → 下潜 0.30 m → 右转扫描 360° → 无目标时估算前进 0.40 m → 对准 → 接近 → 闭爪 → 上升回收

目前没有 DVL，所以深度和转角可以闭环，水平前进距离只能按实测速度估算。机械爪尚未产生过可观察的物理动作，因此必须保持独立禁用。

## 新同学先只看这些

| 你要做什么 | 去哪里 |
|---|---|
| 第一次安装或更新 | [安装与更新](docs/安装与更新.md) |
| 连实艇、点动、标定 | [实机操作](docs/实机操作.md) |
| 用手柄驾驶并录像 | [手柄采集 README](docs/手柄采集README.md) |
| 用键盘驾驶并录像 | [键盘采集 README](docs/键盘采集README.md) |
| 把录像导出成全部或均匀取样图片 | [抽帧工具 README](docs/抽帧工具README.md) |
| 理解自主流程 | [自主任务](docs/自主任务.md) |
| 查 ROS 话题和服务 | [ROS 接口](docs/ROS接口.md) |
| 设置 QGC 与 YOLO 视频分流 | [实机操作](docs/实机操作.md) 第 7 节 |

平时不需要翻 `tests/` 和大部分 Python 源码。

## 目录怎么看

```text
rov_competition_2026/
├── README.md       # 总入口
├── config/         # 本机实艇配置，不进 Git
├── docs/           # 给人看的中文说明
├── scripts/        # 安装、检查和打包入口
├── ros2_ws/src/    # ROS 2 正式源码
└── tests/          # 自动化测试
```

`build/`、`install/`、`log/`、`.venv/`、`__pycache__/` 都是本机生成物，不是需要阅读或提交的源码。

如果要改代码，先按任务找入口，不用从头翻完整个包：

| 想改什么 | 先看哪个文件 |
|---|---|
| 自主任务步骤和状态跳转 | `mission.py` |
| 参数定义和配置校验 | `config.py` |
| YOLO 检测结果 | `detector.py` |
| 视频输入和 RTP 解码 | `video.py` |
| QGC/YOLO 原始视频分流 | `stream_bridge.py` |
| MAVLink 与飞控通信 | `vehicle.py` |
| ROS 自主节点 | `ros_nodes/autonomy_node.py` |
| ROS 飞控网关 | `ros_nodes/vehicle_gateway_node.py` |

这些文件都在 `ros2_ws/src/rov_competition/rov_competition/`。其余小文件大多是测试工具、数据结构或兼容入口，遇到具体问题再看。

## 命名约定：简单即可

- 文档用简短中文名，不再写“给某人”、日期和随意缩写。
- Python 文件、函数和 ROS 字段使用简单英文，因为工具链和报错对英文标识最稳定。
- 注释、日志和操作说明用中文，让团队能直接读懂。
- 名字只要能回答“这个东西是干什么的”，不追求过长、过度严格的形式。
- 已经对外使用的 ROS 话题、服务和命令入口不随意改名，避免现场命令失效。

## Ubuntu 首次安装

仅支持 Ubuntu 22.04 + ROS 2 Humble：

```bash
cd /home/persica/rov_competition_2026
chmod +x scripts/*.sh
./scripts/install.sh
```

每个新终端先加载环境：

```bash
cd /home/persica/rov_competition_2026
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source ros2_ws/install/setup.bash
export PYTHONNOUSERSITE=1
```

检查当前环境：

```bash
which python
python -c "import rclpy, yaml; print(rclpy.__file__); print(yaml.__version__)"
ros2 pkg executables rov_competition
```

## 检查代码

快速离线检查：

```bash
./scripts/check.sh
```

Ubuntu + ROS 2 Humble 完整检查：

```bash
./scripts/check_ros.sh
```

用录像只验证模型和框：

```bash
ros2 run rov_competition rov_replay \
  --video /你的/录像.mp4 \
  --config "$PWD/ros2_ws/src/rov_competition/config/autonomy.yaml" \
  --targets "$PWD/ros2_ws/src/rov_competition/config/targets.yaml" \
  --show
```

`rov_replay` 不连 MAVLink，画面会标记 `SIMULATION`，不能作为实艇验收证据。

## 一键查看 QGC 和 YOLO

安装和编译完成后，视频测试不再需要手动开三四个终端：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_video_test.sh
```

它会启动 `5600 → 5701(QGC) + 5702(YOLO)` 分流、YOLO，并尝试打开
QGC 和带框查看器。这个入口故意不启动飞控网关，因此不会解锁或驱动
机器人。按 `Ctrl+C` 即可一起停止视频分流和 YOLO。首次 QGC 设置和排错
方法见 [实机操作](docs/实机操作.md) 第 7 节。

## 一键键盘驾驶和数据集录像

这个入口用于重拍训练数据，不加载 YOLO、不需要权重、不控制机械爪：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_collection.sh
```

首次会生成 `config/dataset.yaml`。当前场地水深至少 `1.50 m`，
因此 `maximum_depth_m` 默认为 `1.40 m`，并允许相对启动深度继续
下潜最多 `1.40 m`。数据采集默认指令为 `0.20`，可逐级调到 `0.80`；
多轴同时按下时保留每轴功率，混控仍由 ArduSub 完成。
更换场地时必须重新测量和填写。脚本编排 MAVProxy、飞控网关、
`5600 → 5701(QGC) + 5702(原始 MKV 录像)` 和键盘窗口。
它不会自动沉底；只有录像、ALT_HOLD、深度、预检和现场确认
全部通过后才允许解锁。按键、深度保护、按 `0` 回收以及
Esc/Ctrl+C 急停的详细说明见
[键盘采集 README](docs/键盘采集README.md)。

## 手柄驾驶时只录像

如果已经用 QGC 和手柄控制 ROV，不需要 ROS 键盘控制，只运行：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_recording.sh
```

它会自动尝试打开 QGC，只进行
`5600 → 5701(QGC) + 5702(MKV 录像)`，不启动
MAVProxy、飞控网关、YOLO、机械爪或任何控制节点。开始后立即
录像，回到终端按 Enter 或 `Ctrl+C` 完整封装 MKV。详见
[手柄采集 README](docs/手柄采集README.md)。

## 一键桌面抽帧

录像完成后，不需要写 FFmpeg 命令。运行：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_frame_extractor.sh
```

把视频拖进窗口后，输出帧数填 `0` 可导出所有帧，也可填指定数量在整段
视频中均匀取样。软件显示进度、预计容量和剩余时间，
支持 JPG/PNG、选择输出磁盘、安全取消和一键打开结果目录。还可以运行
一次 `./scripts/install_frame_extractor_shortcut.sh`，之后直接从 Ubuntu
应用菜单打开。详见 [抽帧工具 README](docs/抽帧工具README.md)。

## 实艇必须逐级验收

1. QGC 连接 ArduSub，保存完整参数备份。
2. 水中逐个确认 Motor1–Motor8 的位置和方向。
3. QGC 手柄验证四轴和断链保护。
4. ROS 只读遥测和预检。
5. `rov_axis_test` 从 `0.05 / 0.3s` 单动作开始。
6. `rov_turn_test` 按 `30° → 90° → 180° → 360°` 验收。
7. 单独追线并验证机械爪；目前未通过。
8. 标定图像方向、前进估算速度和框面积阈值。
9. 最后才启动自主任务。

任意一级出现方向反了、数据过期、模式异常、急停失效或断链后仍有输出，立即停止。详细命令只在 [实机操作](docs/实机操作.md) 维护一份，避免多篇文档互相矛盾。

## 五条底线

- Python 只发送前后、横移、升沉和偏航意图；八推混控交给 ArduSub。
- 默认不自动切模式、不自动解锁、不打开真实输出。
- QGC 手柄、测试工具和自主节点不能同时取得控制权。
- 超过 `autonomy.yaml` 允许时限的旧感知数据不能驱动接近或闭爪。
- 服务返回“已接受”不等于物理动作成功；实物、日志和录像才是验收证据。
