# 模型权重放置说明

模型权重不进入 Git 仓库。当前 `seafood_yolo26x.pt` 大约 113 MB，超过
GitHub 普通 Git 对象的单文件限制，而且权重与代码的更新频率不同。

每台需要运行识别的电脑只需准备一次权重文件，并放到：

```text
ros2_ws/src/rov_competition/models/seafood_yolo26x.pt
```

当前候选版要求的 SHA-256 是：

```text
300eb3d98ae586c1c5b26b87b3a1baf6450bf6ef367acad4fc5129622ce20a23
```

Ubuntu 上校验：

```bash
cd /home/persica/rov_competition_2026_git
sha256sum ros2_ws/src/rov_competition/models/seafood_yolo26x.pt
```

哈希不一致时不要拿它做比赛验收。`git pull` 不会删除或覆盖被忽略的本地
权重；禁止随意执行 `git clean -fdx`，该命令会删除权重、真实配置和虚拟环境。
