# ROV 比赛工作空间

新增独立入口：[持续盲抓兜底](docs/持续盲抓README.md)。
一队使用 `./scripts/start_blind_grab_team1.sh`（官方端口40198），二队 `ddhyzx2` 使用 `./scripts/start_blind_grab_team2.sh`（官方端口40197）。启动后立即下潜，首次压力深度稳定3秒或10秒兜底后抓取；随后永久执行“抓取投放→上潜2秒→四段蛇形前进→定时下潜”。
视觉继续显示识别框，但框数完全不参与运动；模板已写入本艇臂爪PWM、升沉和蛇形时间，由操作员提前设置模式并解锁，手动关闭结束。
单独检查臂爪可运行 `./scripts/test_arm_gripper.sh`，按 `a/b/c/d` 分别执行张爪、闭爪、机械臂后仰和恢复。
需要手动试PWM时运行 `./scripts/tune_arm_gripper_pwm.sh`，先选 `a/b/c/d`，再输入PWM数字。
PWM已按实艇标定更新为夹爪575闭/730开、机械臂710垂直抓取/1810后仰（1300水平参考）；旧电脑拉取后运行 `./scripts/apply_corrected_pwm_direction.sh` 同步本机配置。
比赛方 `ros2_topic_forwarding` 已纳入同一工作空间并由该入口自动启动；旧电脑首次更新、一键启动和计分链路检查见[官方ROS转发与旧电脑启动](docs/官方ROS转发与旧电脑启动.md)。
如果改用 QGC 和手柄人工驾驶，二队直接运行 `./scripts/start_official_data_only_team2.sh`，一队运行 `./scripts/start_official_data_only_team1.sh`。这个入口只读14551上的MAVLink遥测并向主办方转发ROS数据，不启动盲抓、视频或YOLO，也不向飞控发送运动、舵机、解锁、模式、RC覆盖或心跳报文。

以下为原有任务入口及其说明。

新增本地策略：[半圆搜索与局部抓取](docs/半圆搜索与局部抓取.md)。
入口 `scripts/start_semicircle_collection_test.sh`；需填写现场配置，默认不能实艇执行。
使用单目标多次重试替代固定群体三次盲抓，位置仅为航向与指令时间推算。

当前版本：`0.2.0rc2`。这是赛前候选版，默认不会让推进器或机械爪动作。

任务流程：

> 完全浸没 → 下潜 0.30 m → 右转扫描 360° → 无目标时估算前进 0.40 m → 对准 → 接近 → 闭爪 → 上升回收

目前没有 DVL，所以深度和转角可以闭环，水平前进距离只能按实测速度估算。机械爪保留 `dalian`（单路 S12 渐变）和 `rst`（双路 S11+S10）两个候选档案；照片无法代替接线核对，两者默认都未标定、禁止真实输出。

## 新同学先只看这些

| 你要做什么 | 去哪里 |
|---|---|
| 明天按顺序恢复 QGC、测爪子、换权重和测逻辑 | [明日联调 README](docs/明日联调README.md) |
| 第一次安装或更新 | [安装与更新](docs/安装与更新.md) |
| 连实艇、点动、标定 | [实机操作](docs/实机操作.md) |
| 用手柄驾驶并录像 | [手柄采集 README](docs/手柄采集README.md) |
| 用键盘驾驶并录像 | [键盘采集 README](docs/键盘采集README.md) |
| 测试“搜索—对准—接近”逻辑 | [搜索接近测试 README](docs/搜索接近测试README.md) |
| 测试“触底—离底—扇贝群三次盲抓” | [群体收集测试 README](docs/群体收集测试README.md) |
| 标定和测试机械爪 | [实机操作](docs/实机操作.md) 第 9.1 节 |
| 记录成功抓取时的框位置和大小 | [实机操作](docs/实机操作.md) 第 11.4 节 |
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
| 正式自主任务步骤和状态跳转 | `mission.py` |
| 扇贝群搜索和盲抓测试 | `cluster_collection.py` |
| 触底平台和离底共用判定 | `bottom_clearance.py` |
| 参数定义和配置校验 | `config.py` |
| YOLO 检测结果 | `detector.py` |
| 视频输入和 RTP 解码 | `video.py` |
| QGC/YOLO 原始视频分流 | `stream_bridge.py` |
| MAVLink 与飞控通信 | `vehicle.py` |
| 命令时间戳、遥测新鲜度和安全门 | `safety.py` |
| ROS 自主节点 | `ros_nodes/autonomy_node.py` |
| ROS 飞控网关 | `ros_nodes/vehicle_gateway_node.py` |

这些文件都在 `ros2_ws/src/rov_competition/rov_competition/`。其余文件按用途分成三组：

- `axis_test.py`、`turn_test.py`、`search_approach*.py`：水池测试工具；
- `dataset_*.py`、`frame_extractor*.py`、`grasp_calibration.py`：数据采集、抽帧和抓取标定工具；
- `domain.py`、`commissioning*.py`、`targets.py`：多处共用的数据结构和小逻辑。

这些模块虽然名字相近，但运行权限和故障处理不同，不把它们硬塞进一个大文件。

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

## 明日联调总入口

端口、机械爪、新权重、本地超时迁移、自动接近和人工抓取标定已统一到一个
交互式向导：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_tomorrow_test.sh
```

也可以逐步运行：

```bash
./scripts/start_tomorrow_test.sh qgc
./scripts/start_tomorrow_test.sh gripper
./scripts/start_tomorrow_test.sh weights
./scripts/start_tomorrow_test.sh timing
./scripts/start_tomorrow_test.sh search
./scripts/start_tomorrow_test.sh calibrate
```

它只是“一个入口”，不会无人值守连续执行危险步骤。每次解锁、爪子
测试和权重替换都有独立的现场条件与确认词。详见
[明日联调 README](docs/明日联调README.md)。

## 一键查看 QGC 和 YOLO

安装和编译完成后，视频测试不再需要手动开三四个终端：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_video_test.sh
```

QGC 继续直接接收 BlueOS 发往默认 `5600` 的画面；脚本只处理 BlueOS
发往 `5700` 的软件副流，并复制给 `5702` 的 YOLO。它会启动 YOLO，
并尝试打开 QGC 和带框查看器。这个入口故意不启动飞控网关，因此不会解锁或驱动
机器人。按 `Ctrl+C` 即可一起停止视频分流和 YOLO。首次 QGC 设置和排错
方法见 [实机操作](docs/实机操作.md) 第 7 节。

这个入口现在只做视频与识别，不再混入控制或抓取标定。需要自动扫描、
对准后再用 WASD 精调时，使用后面的“抓取位置标定”独立入口。

## 一键键盘驾驶和数据集录像

这个入口用于重拍训练数据，不加载 YOLO、不需要权重、不控制机械爪：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_collection.sh
```

首次会自动生成 `config/dataset.yaml`。键盘采集不设置绝对或相对
软件深度上限；深度只用于界面记录，以及数据有效时按 `0` 回收。
数据采集默认指令为 `0.20`，可逐级调到 `0.80`；
多轴同时按下时保留每轴功率，混控仍由 ArduSub 完成。
键盘采集允许姿态消息短时中断 `5.0 s`，但不放宽运动发布者
`0.5 s` 失联停车看门狗。遥测消息和控制状态阈值分别为
`1.5 s` 和 `3.0 s`。旧电脑上被 Git 忽略的本地配置可用下列命令
先备份、再升级：

```bash
./scripts/start_tomorrow_test.sh timing
```

该命令不连接飞控，也不修改 ArduSub 参数。
脚本直接接收 BlueOS 发往 `14551` 的 ROS MAVLink，启动飞控网关，
并把 `5700` 软件视频副流复制到 `5704` 原始 MKV 录像器和键盘窗口。
QGC 独立使用默认 `14550` 和 `5600`，不会经过本脚本。
它不会自动沉底；只有录像、ALT_HOLD、预检和现场确认
全部通过后才允许解锁。按键、可选的按 `0` 回收以及
Esc/Ctrl+C 急停的详细说明见
[键盘采集 README](docs/键盘采集README.md)。

## 手柄驾驶时只录像

如果已经用 QGC 和手柄控制 ROV，不需要 ROS 键盘控制，只运行：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_dataset_recording.sh
```

它会自动尝试打开 QGC；QGC 直接使用默认 `5600`，脚本只将 BlueOS
发往 `5700` 的软件副流复制到 `5704` 录像器。它不启动飞控网关、
YOLO、机械爪或任何控制节点。开始后立即
录像，回到终端按 Enter 或 `Ctrl+C` 完整封装 MKV。详见
[手柄采集 README](docs/手柄采集README.md)。

## 一键搜索—接近与抓取位置标定

两种水池流程共用同一套“下潜、搜索、漏检确认和偏航对准”状态机，但稳定
对准后的行为不同：

```bash
cd /home/persica/rov_competition_2026

# 自动对准后继续自动接近；不闭爪
./scripts/start_search_approach_test.sh

# 自动对准后停车，切换到 WASD 人工精调和抓取样本记录
./scripts/start_grasp_position_test.sh
```

启动后只需输入固定下潜 power 和确认词，不再输入下潜深度。程序
持续下潜，最近 3 秒深度无明显变化后记录疑似池底；
默认先上浮 `0.10 m` 脱离池底并稳定定深，再开始扫描。
自动模式达到面积阈值后回收；人工模式用 `Enter/G` 保存闭爪前证据、`C` 闭爪、`Y/N`
记录实物成败。`0` 正常回到启动深度并上锁；`Space` 只回中/暂停，
`Esc`、关闭窗口或 `Ctrl+C` 走急停路径。完整键位和明日顺序见
[搜索接近测试 README](docs/搜索接近测试README.md)。

## 一键扇贝群体收集测试

新增流程与上面的单目标测试完全独立，不会替换原入口：

```bash
cd /home/persica/rov_competition_2026
./scripts/start_cluster_collection_test.sh
```

它使用“触底重新标定 → 上浮 `0.15m` → 扫描”的跳跃式流程；
最近 5 帧中至少 3 帧看到同一空间群中不少于 6 个 `scallop`
才锁定，对准并靠近后每群执行 3 次盲抓动作。没有下视测距时
它不是真实地形跟随，爪子 ACK 也不等于抓取成功。必须先按文档
的“只显示 → 只对准 → 靠近不下降 → 单次盲抓 → 三次盲抓”顺序验收。
详见 [群体收集测试 README](docs/群体收集测试README.md)。

机械爪还未确定是哪套接线时，必须先断开推进器并分别执行：

```bash
./scripts/start_gripper_test.sh dalian
# 上锁、断电并重新核对接线后，才可测试：
./scripts/start_gripper_test.sh rst
```

候选测试不会修改 `robot.yaml`，也不会自动把任何档案标成已标定。只有开、闭
两种实物动作都由操作员确认后，才能人工启用该档案和机械爪权限。

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
   已确认是电调待机蜂鸣时，可人工运行一次
   `./scripts/nudge_thrusters_once.sh forward`；它会自行启动网关、开许可、
   正常解锁，并在固定 `0.05 / 1s` 点动后回中上锁。它会真实驱动部分
   推进器，不能作为无人值守的周期“消音器”。
6. `rov_turn_test` 按 `30° → 90° → 180° → 360°` 验收。
7. 断开推进器动力后，先追线判断 `dalian` 或 `rst` 档案，再单独验证开闭值；目前仅代码通过，实物未通过。
8. 标定图像方向、前进估算速度和框面积阈值。
9. 最后才启动自主任务。

任意一级出现方向反了、数据过期、模式异常、急停失效或断链后仍有输出，立即停止。详细命令只在 [实机操作](docs/实机操作.md) 维护一份，避免多篇文档互相矛盾。

## 五条底线

- Python 只发送前后、横移、升沉和偏航意图；八推混控交给 ArduSub。
- 默认不自动切模式、不自动解锁、不打开真实输出。
- QGC 手柄、测试工具和自主节点不能同时取得控制权。
- 超过 `autonomy.yaml` 允许时限的旧感知数据不能驱动接近或闭爪。
- 服务返回“已接受”不等于物理动作成功；实物、日志和录像才是验收证据。
