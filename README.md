# Armbot

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
