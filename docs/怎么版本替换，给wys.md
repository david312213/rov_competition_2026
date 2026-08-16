可以替换，但不要直接覆盖。采用“旧版改名备份 → 新版解压到临时目录 → 移到原位置”的方式，随时可以退回。

先确保：

- QGC 已上锁；
- 所有 ROS 终端都已 `Ctrl+C`；
- 推进器主电断开；
- 不要复制旧版的 `.venv`、`build`、`install`、`log`。

## 第一步：把 ZIP 放到 Ubuntu

把这个文件传到 Ubuntu：

[rov_competition_2026_precompetition_rc1.zip](/Users/david/Desktop/大连2025/output/releases/rov_competition_2026_precompetition_rc1.zip)

建议放到：

```text
/home/persica/rov_release_inbox/
```

Ubuntu 执行：

```bash
mkdir -p /home/persica/rov_release_inbox
```

然后用文件管理器把 ZIP 放进去。

## 第二步：核对文件没有传坏

```bash
cd /home/persica/rov_release_inbox
sha256sum rov_competition_2026_precompetition_rc1.zip
```

必须得到：

```text
ecee1c08ad5790fabe4603353a51b7d6a3cbc55b36cec7c5c571ecd88021b9cc
```

不完全一致就停止，不要解压。

## 第三步：解压到临时目录

```bash
mkdir /home/persica/rov_rc1_staging_20260816
```

```bash
unzip /home/persica/rov_release_inbox/rov_competition_2026_precompetition_rc1.zip \
  -d /home/persica/rov_rc1_staging_20260816
```

检查版本：

```bash
cat /home/persica/rov_rc1_staging_20260816/rov_competition_2026/VERSION
```

应该只显示：

```text
0.2.0rc1
```

## 第四步：备份并替换旧工程

确认旧工程确实在这里：

```bash
ls -ld /home/persica/rov_competition_2026/rov_competition_2026
```

备份旧版：

```bash
mv /home/persica/rov_competition_2026/rov_competition_2026 \
  /home/persica/rov_competition_2026/rov_competition_2026_backup_before_rc1
```

把新版移到原位置：

```bash
mv /home/persica/rov_rc1_staging_20260816/rov_competition_2026 \
  /home/persica/rov_competition_2026/rov_competition_2026
```

检查：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
cat VERSION
ls
```

此时旧版还完整保存在：

```text
/home/persica/rov_competition_2026/rov_competition_2026_backup_before_rc1
```

暂时不要删除。

## 第五步：重新安装和构建

不要沿用旧虚拟环境，直接执行新版安装脚本：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
chmod +x scripts/*.sh
./scripts/install_ubuntu_22_04.sh
```

安装结束后，新开一个终端：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source ros2_ws/install/setup.bash
```

执行完整 Ubuntu 验证：

```bash
./scripts/verify_ros2_humble.sh
```

最后应显示：

```text
ROS 2 Humble 构建、测试、接口和命令入口检查全部通过。
```

再检查命令入口：

```bash
ros2 pkg executables rov_competition
```

应该能找到：

```text
rov_axis_test
rov_turn_test
rov_autonomy
rov_vehicle
rov_replay
rov_preflight
```

## 第六步：暂时不要复制旧实艇配置覆盖新版

先查看旧配置是否存在：

```bash
ls -l /home/persica/rov_competition_2026/rov_competition_2026_backup_before_rc1/config/robot.yaml
```

如果存在，只把它复制成“参考文件”：

```bash
cp /home/persica/rov_competition_2026/rov_competition_2026_backup_before_rc1/config/robot.yaml \
  /home/persica/rov_competition_2026/rov_competition_2026/config/robot_old_reference.yaml
```

再从新版安全模板创建真正配置：

```bash
cd /home/persica/rov_competition_2026/rov_competition_2026
cp ros2_ws/src/rov_competition/config/robot.example.yaml config/robot.yaml
```

先不要把三个 `false` 改成 `true`。把下面命令的输出发给我，我再帮你逐项把旧实艇参数迁移进新版：

```bash
diff -u config/robot.yaml config/robot_old_reference.yaml
```

如果输出很长，可以直接把两个 YAML 文件发给我。

完成到这里后，先不要启动推进器。下一步应先运行只读网关：

```bash
ros2 launch rov_competition telemetry.launch.py \
  robot_config:="$PWD/config/robot.yaml"
```

看到 `MAVLink 已连接` 且真实输出为 `DISABLED`，才说明新版本替换和只读连接基本成功。