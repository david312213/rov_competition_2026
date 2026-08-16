# Git 仓库使用说明

远程仓库：`https://github.com/david312213/rov_competition_2026`

仓库默认不保存以下本机文件：

- YOLO/PyTorch 权重；
- `config/robot.yaml` 等真实硬件配置；
- `.venv`、ROS 2 的 `build/install/log`；
- rosbag、录像、日志和发布压缩包。

这样做可以避免把 113 MB 权重、实艇参数和大量生成文件反复推送。模型和
实艇配置放好一次后，正常的 `git pull` 不会改动它们。

## 1. Ubuntu 第一次克隆

先安装并登录 GitHub 命令行：

```bash
sudo apt update
sudo apt install -y git gh
gh auth login
gh auth setup-git
```

仓库是私有仓库。登录时使用获准访问该仓库的 GitHub 账号，不要在群里发送
访问令牌或密码。

不要直接覆盖正在使用的工程，先克隆到新目录：

```bash
cd /home/persica
gh repo clone david312213/rov_competition_2026 rov_competition_2026_git
cd /home/persica/rov_competition_2026_git
git status -sb
cat VERSION
```

`VERSION` 应显示当前发布版本。随后从已验证的旧工程或备份中单独复制：

1. `config/robot.yaml`；
2. `ros2_ws/src/rov_competition/models/seafood_yolo26x.pt`。

不要复制旧工程的 `.venv`、`build`、`install` 或 `log`。

## 2. 第一次构建

```bash
cd /home/persica/rov_competition_2026_git
chmod +x scripts/*.sh
./scripts/install_ubuntu_22_04.sh
```

完成后新开终端：

```bash
cd /home/persica/rov_competition_2026_git
source /opt/ros/humble/setup.bash
source .venv/bin/activate
source ros2_ws/install/setup.bash
./scripts/verify_ros2_humble.sh
```

## 3. 以后更新代码

更新前先退出正在运行的 ROS 节点，不要在推进器已解锁时更换代码：

```bash
cd /home/persica/rov_competition_2026_git
git status -sb
git pull --ff-only
```

如果依赖或 ROS 包发生变化，再执行：

```bash
source /opt/ros/humble/setup.bash
source .venv/bin/activate
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 4. 不要这样做

- 不要使用 `sudo git pull`；
- 不要把 GitHub 令牌、密码或真实机器人参数提交到仓库；
- 不要在不理解结果时运行 `git reset --hard`；
- 不要运行 `git clean -fdx`，它会删除被 Git 忽略的权重、配置和虚拟环境；
- `git pull` 出现冲突时不要反复尝试，把 `git status` 和完整报错交给维护者。
