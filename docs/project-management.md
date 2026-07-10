# Armbot 项目协作规范

本文档定义 Armbot 使用 GitHub Issues、Projects、分支和 Pull Request 的轻量协作流程。目标是让团队随时知道：谁在做什么、做到哪一步、被什么阻塞，以及一个任务怎样才算完成。

## 1. 核心对象怎么分工

- **Issue**：一件具体、可验收的工作，例如修复故障、实现功能或完成实验。
- **Project**：汇总 Issue 和 PR 的团队看板，用于管理状态、优先级、负责人和模块。
- **Pull Request**：提交实现结果，展示代码差异、验证结果和讨论记录。
- **Milestone**：一个版本或阶段目标，例如“SLAM MVP”或“完整演示版本”。

推荐工作链路：

```text
创建 Issue
→ 明确负责人和验收标准
→ 加入 Project
→ 创建对应分支
→ 开发和验证
→ 创建 PR，并关联 Issue
→ CI 通过和问题解决
→ Merge commit 合并
→ Issue 自动关闭
→ Project 状态变为 Done
```

## 2. 什么情况需要创建 Issue

满足任意一项时，应先创建 Issue：

- 预计耗时超过 30 分钟；
- 需要多人协作或会影响其他模块；
- 修改 ROS 2 消息、话题、服务、TCP/JSON 协议或公共接口；
- 修改硬件接口、机械臂控制、底盘控制或安全相关逻辑；
- 修复可复现 Bug；
- 进行性能测试、技术选型或对比实验；
- 需要后续验收、实机测试或文档记录；
- 当前无法立即处理，但不能遗忘。

以下低风险改动可以不单独创建 Issue：

- README 错字；
- 一行文案或明显格式问题；
- 立即完成、没有接口影响的小调整。

即使不创建 Issue，功能代码和有风险的改动仍应通过分支和 PR 合并。

## 3. Issue 类型

仓库提供三种 Issue Form。

### 3.1 Bug 报告

用于已经发生的故障、异常行为或回归问题。至少应包含：

- 可复现步骤；
- 预期行为与实际行为；
- 设备、系统、ROS 2、Python、分支或 commit 等环境信息；
- 关键日志；
- 是否影响演示或硬件安全。

### 3.2 功能任务

用于新功能、改进和有明确结果的重构。至少应包含：

- 背景和用户价值；
- 目标；
- 范围与非目标；
- 可测试的验收标准；
- 依赖、风险和验证方式。

避免只写“研究一下”“优化一下”“接入 SLAM”这类无法判断完成状态的标题。

### 3.3 实验与性能测试

用于验证技术假设、对比方案或测量性能。实验开始前应写清：

- 要回答的问题；
- 假设；
- 自变量、因变量、控制变量和对照；
- 指标与成功阈值；
- 实验环境和重复步骤；
- 预期产物和停止条件。

实验结束后，应把原始日志、配置、数据表、图表和结论补充到 Issue，必要时创建后续功能或 Bug Issue。

## 4. Issue 标题建议

标题应描述结果或问题，不要只写模块名。

推荐：

```text
[Bug] 摄像头断开后节点无法自动恢复
[Feature] 支持 SLAM 地图保存和重新加载
[Experiment] 对比两种图像分辨率下的定位精度与帧率
```

不推荐：

```text
SLAM
优化一下
测试
有问题
```

## 5. 大任务如何拆分

预计超过 2 天、涉及多个模块或多人并行的任务，应拆成父 Issue 和子 Issue。

示例：

```text
父 Issue：[EPIC] 完成 SLAM MVP

子 Issue：
- 确认传感器和里程计话题
- 接入 SLAM 节点
- 实现地图保存和加载
- 完成重定位验证
- 测量 RDK 上的 CPU、内存和帧率
- 编写启动与故障排查文档
```

父 Issue 描述整体目标和完成条件；真正分配给队员的是可独立验收的子 Issue。

## 6. Project 推荐配置

建议只建立一个总 Project：

```text
Armbot Development
```

### 6.1 Status

```text
Inbox
Ready
In progress
In review
Blocked
Done
```

含义：

- `Inbox`：刚提出，尚未整理；
- `Ready`：目标、负责人和验收标准已经明确；
- `In progress`：正在开发或实验；
- `In review`：已创建 PR，等待 CI、检查或实机验证；
- `Blocked`：被接口、硬件、数据、权限或其他任务阻塞；
- `Done`：验收完成并已合并或确认结论。

### 6.2 Priority

```text
P0 Critical
P1 High
P2 Normal
P3 Later
```

- `P0`：可能损坏硬件、阻塞全部开发或阻塞近期演示；
- `P1`：当前阶段必须完成；
- `P2`：正常计划任务；
- `P3`：有价值但暂不影响当前里程碑。

### 6.3 Module

```text
SLAM / Mapping
Perception
Navigation
Arm Control
VLA
ROS 2 / Communication
Hardware
Web / Visualization
Build / CI
Documentation
```

跨模块任务选择最主要模块，并在 Issue 中列出影响范围。

### 6.4 Size

```text
XS：半小时以内
S：半天以内
M：1～2 天
L：超过 2 天，应继续拆分
```

### 6.5 推荐视图

1. **总看板**：Board 布局，按 `Status` 分列；
2. **本周任务**：Table 布局，只显示未完成项，按 `Priority` 排序；
3. **我的任务**：过滤 `assignee:@me`；
4. **模块进度**：按 `Module` 分组；
5. **Blocked**：只显示被阻塞任务，并要求填写阻塞原因。

### 6.6 推荐自动化

- 新 Issue 或 PR 加入 Project 时，状态设为 `Inbox`；
- PR 创建后，相关任务改为 `In review`；
- Issue 关闭或 PR 合并后，状态改为 `Done`；
- 已完成一段时间的项目项自动归档。

## 7. 分支命名

一个 Issue 对应一个主要开发分支，建议包含 Issue 编号。

```text
feature/23-save-load-map
fix/31-camera-reconnect
experiment/42-resolution-benchmark
docs/18-update-rdk-guide
chore/27-update-ci
```

常用前缀：

- `feature/`：新功能；
- `fix/`：Bug 修复；
- `experiment/`：实验或性能测试；
- `docs/`：文档；
- `chore/`：构建、CI、依赖和仓库维护；
- `refactor/`：不改变外部行为的重构。

不要长期复用一个公共 `dev` 或 `test` 分支。任务合并后删除短期分支，下一个任务重新创建。

## 8. 开发流程

```bash
git switch main
git pull
git switch -c feature/23-save-load-map

# 开发、提交和本地验证
git add .
git commit -m "Add map save and load support"

git push -u origin feature/23-save-load-map
gh pr create --fill
```

提交应按有意义的开发步骤划分，不要把无关功能塞进同一个提交或 PR。

## 9. Pull Request 规范

PR 应使用仓库模板，并重点填写：

- 对应 Issue；
- 主要改动；
- 接口影响；
- 验证命令和结果；
- 是否完成 RDK、机械臂或底盘实机验证；
- 风险和回滚方式。

在 PR 描述中使用：

```text
Closes #23
```

PR 合并到默认分支后，对应 Issue 会自动关闭。

只有部分工作完成、不能关闭整个 Issue 时，改用：

```text
Related to #23
```

仓库使用 Merge commit 保留功能分支的提交和汇合历史。合并前应确保：

- Required status checks 已通过；
- 所有需要处理的 review conversation 已解决；
- 需要的实机验证已经完成，或明确记录尚未完成的原因；
- PR 没有混入无关改动。

## 10. Review 策略

不强制所有 PR 都找队友批准，但以下改动应主动请求相关成员 Review：

- ROS 2 消息、话题、服务和跨模块协议；
- 机械臂、底盘和硬件安全逻辑；
- 大范围重构；
- 数据格式、配置格式或公共 API 变化；
- 冲突较多或影响多个模块的 PR；
- 比赛或演示前的关键修复。

README、小文案和低风险维护改动可以由作者自查后合并，前提是 CI 和仓库规则允许。

## 11. Blocked 任务怎么处理

任务进入 `Blocked` 时，不要只改状态，还要在 Issue 中写明：

```text
阻塞原因：等待 #12 确定 ROS 2 消息字段
解除条件：#12 合并并发布接口说明
下一步：先完成不依赖该接口的配置读取部分
```

阻塞解除后，由负责人把状态改回 `Ready` 或 `In progress`。

## 12. 每周维护建议

每周用 15～30 分钟完成一次整理：

1. 清理 `Inbox`，补全验收标准和负责人；
2. 检查 `In progress` 是否长期没有更新；
3. 逐项确认 `Blocked` 的解除条件；
4. 把过大的任务拆分；
5. 检查当前 Milestone 仍缺哪些必要任务；
6. 关闭重复、失效或明确不做的 Issue，并写明原因。

## 13. 完成定义

一个功能或 Bug Issue 通常只有同时满足以下条件才算完成：

- 验收标准全部满足；
- 代码和必要文档已提交；
- CI 通过；
- 需要的实机验证已完成；
- 已知限制和风险有记录；
- PR 已合并；
- Issue 已关闭。

实验 Issue 的完成条件不同：即使假设被否定，只要实验可复现、数据完整、结论明确，也可以标记为 Done。
