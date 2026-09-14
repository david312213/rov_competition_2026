# 模型权重放置说明

模型权重不进入 Git 仓库。当前 `seafood_yolo26x.pt` 大约 113 MB，超过
GitHub 普通 Git 对象的单文件限制，而且权重与代码的更新频率不同。

每台需要运行识别的电脑只需准备一次权重文件，并放到：

```text
ros2_ws/src/rov_competition/models/seafood_yolo26x.pt
```

仓库模板中的旧候选权重 SHA-256 是：

```text
300eb3d98ae586c1c5b26b87b3a1baf6450bf6ef367acad4fc5129622ce20a23
```

Ubuntu 上校验：

```bash
cd /home/persica/rov_competition_2026_git
sha256sum ros2_ws/src/rov_competition/models/seafood_yolo26x.pt
```

明天替换新权重时，不要手动修改上述哈希，运行：

```bash
./scripts/start_tomorrow_test.sh weights
```

工具会用 CUDA 试推理、核对必需类别、原子替换权重，并把真实哈希和类别写入
被 Git 忽略的 `config/autonomy.local.yaml`。现场脚本优先使用这份本地配置。
可用 `./scripts/start_tomorrow_test.sh weights rollback` 同时回滚权重和配置。

`git pull` 不会删除或覆盖被忽略的本地
权重；禁止随意执行 `git clean -fdx`，该命令会删除权重、真实配置和虚拟环境。
