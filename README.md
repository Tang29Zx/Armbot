# Armbot

## RDK ROS 2 构建

RDK 上使用脚本构建机械臂接口与控制包：

```bash
bash scripts/build-rdk-ros2.sh
```

脚本会加载 ROS 2 Humble，优先使用兼容的 Ubuntu 系统 Python 包，并以
symlink 模式构建 `action_interfaces`、`action_pkg` 和 `vla_dataset`。

## VLA episode 试采

先启动现有机械臂控制栈和带时间戳的 `/image` 相机 topic，再运行：

```bash
source ros2_ws/install/setup.bash
ros2 run vla_dataset record_episode \
  --task "抓取红色方块"
```

按 `Ctrl+C` 结束。数据默认写入 `~/vla_episodes`，新 episode 固定标记为
`unreviewed`，不能直接进入训练集。字段、风险和验收标准见
[`docs/vla-data-collection.md`](docs/vla-data-collection.md)。

需要严格追溯固件版本时，可额外传入可选参数
`--firmware-sha256 <实际烧录固件的64位SHA-256>`；省略时 manifest 记录为
`unknown`。

## VLA 在线推理

Arch/Ubuntu PC 通过 Docker 运行固定的 Ubuntu 22.04 + ROS 2 Humble bridge，
使用 Fast DDS 与 RDK 双向通信，并由独立 GPU 容器运行 OpenPI。部署、shadow 验收、
控制权切换和故障注入流程见
[`docs/vla-runtime-docker.md`](docs/vla-runtime-docker.md)。默认配置只做 shadow
推理，不发布实机命令。

## Git Hooks

默认分支的 GitHub Ruleset 为仓库管理员保留 `Always bypass`，便于管理员在修改 README、文案等低风险内容或处理紧急情况时直接推送。功能代码仍推荐通过功能分支和 Pull Request 合并。

管理员绕过远端规则后，GitHub 不会阻止直接推送，因此仓库提供本地 `pre-push` Hook 作为误操作防线。Linux、macOS 和 Git Bash 用户执行：

```bash
bash scripts/install-git-hooks.sh
```

Windows PowerShell 用户执行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/install-git-hooks.ps1
```

直接推送默认分支时，必须在交互终端输入以下完整文本：

```text
PUSH main
```

功能开发推荐使用分支和 Pull Request：

```bash
git switch -c feature/example
git push -u origin feature/example
gh pr create --fill
```

明确的非交互自动化任务可以显式绕过本地确认：

```bash
ALLOW_PROTECTED_BRANCH_PUSH=1 git push origin main
```

该环境变量只绕过普通的默认分支推送确认，不能删除默认分支。Git Hooks 不会随着 `git clone` 自动启用，克隆仓库后需要执行一次对应的安装脚本。
