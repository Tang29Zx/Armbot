# Armbot VLA 技术现状与接入设计

更新时间：2026-07-13  
适用分支：`feature/vla`  
代码基线：`3264a6d`  
文档状态：`v0.1 draft`

## 1. 文档目的

本文记录 Armbot 当前与 VLA（Vision-Language-Action，视觉-语言-动作模型）有关的真实代码状态、目标架构、接口约束、训练与部署路径，以及尚未解决的安全问题。

状态标记约定：

- **已实现**：仓库中已有代码；
- **已验证**：已有测试或实机证据；
- **待验证**：有实现但当前环境尚未完成构建或实机验证；
- **待实现**：只有设计或讨论结论，仓库中还没有对应代码。

本文以当前代码为准。`docs/arm-control-interface.md` 仍把部分类型化接口描述为“待新增”，但这些接口已在 `feature/vla` 分支实现，该文档状态已经滞后。

## 2. 总体架构

### 2.1 训练链路

```text
RDK X5 相机 + 机械臂状态 + 人工示教动作 + 任务文本
                          |
                          v
              同步采集并生成 episode
                          |
                          v
                 LeRobotDataset
                          |
                 上传到训练服务器
                          |
                          v
       OpenPI base checkpoint（计划使用 pi0.5）
                          |
                 LoRA / 全量微调
                          |
                          v
           Armbot checkpoint + norm stats
```

### 2.2 推理与控制链路

推荐的首版部署方式如下：

```text
RDK X5
  camera frame + /arm/state + task prompt
                    |
        openpi-client / WebSocket
                    |
                    v
WSL2（RTX 5070 Ti Laptop 12 GB）
  OpenPI policy server，默认端口 8000
                    |
              action chunk
                    |
                    v
RDK X5 VLA bridge（待实现）
  反归一化 -> 限位 -> 限速 -> 看门狗 -> ROS2 command
                    |
                    v
/arm/command -> action_pkg -> I2C bus 5 / 0x30
                    |
                    v
              STM32 + 舵机
```

模型不运行在 STM32 上。STM32 只负责确定性的底层执行与反馈；RDK 负责传感器、ROS2 和动作安全层；WSL 负责本地模型推理；训练服务器负责数据训练和 checkpoint 产出。

## 3. 当前实现清单

| 子系统 | 当前状态 | 说明 |
| --- | --- | --- |
| 机械臂类型化 ROS2 接口 | 已实现、待重新构建验证 | `ArmCommand`、`ArmState`、急停、reset、反馈和兼容接口已有代码 |
| 机械臂 I2C 控制 | 已实现、已有实机调试提交 | bus `5`，地址 `0x30`，32 字节命令与状态包 |
| 底盘 ROS2 控制 | 已实现、待本机 ROS 环境验证 | `/cmd_vel`、`/odom`、`odom -> base_link` TF，I2C 地址 `0x34` |
| RDK 视频推流 | 已实现 | USB 相机经 FFmpeg 推送 SRT/RTSP；另有 Hobot H.264 实验链路 |
| OpenPI 源码 | 已下载 | 位于同级目录 `/home/tang/projects/openpi`，不属于 Armbot Git 仓库 |
| OpenPI Python 环境 | 已完成、GPU 已验证 | JAX 0.5.3 和 PyTorch 2.7.1+cu128 均已完成 RTX 5070 Ti 实际运算 |
| 模型权重 | 未下载 | 首次启动 policy server 时按需下载 |
| ROS 2 Humble | 当前 WSL 未安装 | 因此本机还没有完成 `colcon build/test` |
| Armbot VLA policy adapter | 待实现 | 尚无相机/状态字段映射和动作解码器 |
| LeRobot 数据采集器 | 待实现 | 尚无同步 episode recorder 和转换脚本 |
| Armbot 微调配置 | 待实现 | 尚无 `TrainConfig`、数据配置和 norm stats |
| VLA ROS2 bridge | 待实现 | 尚无 policy client、action chunk 调度和模型输出安全层 |

## 4. 机械臂控制接口

### 4.1 ROS2 graph

| 方向 | 名称 | 类型 | 当前语义 |
| --- | --- | --- | --- |
| 输入 | `/arm/command` | `action_interfaces/msg/ArmCommand` | 类型化机械臂命令 |
| 输入 | `/arm/emergency_stop` | `std_msgs/msg/Bool` | `true` 锁存急停；`false` 只释放请求 |
| 输出 | `/arm/state` | `action_interfaces/msg/ArmState` | 10 Hz 状态、反馈和错误 |
| 输出 | `/joint_states` | `sensor_msgs/msg/JointState` | 配置开启时发布 5 个关节位置 |
| 服务 | `/arm/reset_error` | `std_srvs/srv/Trigger` | 清理错误和急停锁存 |
| 兼容输入 | `/command_topic` | `std_msgs/msg/String` | 旧 `ARM`、`SERVO`、`STOP` 命令 |
| 兼容输出 | `/status_topic` | `std_msgs/msg/String` | STM32 前 8 字节状态文本 |

`ArmCommand` 支持四种模式：

- `MODE_STOP=0`：发送停止请求；
- `MODE_END_EFFECTOR=1`：发送 `x/y/z/pitch/duration_sec`；
- `MODE_JOINT=2`：接口已定义，但当前代码明确拒绝执行；
- `MODE_GRIPPER=3`：`gripper_position` 使用 `[0, 1]` 规范范围。

命令使用单调递增的 `sequence_id` 关联状态并拒绝重复、过期和乱序命令。`sequence_id=0` 只作为旧字符串兼容层的特殊值，不进行去重。

### 4.2 状态机

`ArmState` 的状态为：

```text
IDLE -> MOVING -> SUCCEEDED
  |        |
  +------> ERROR
  +------> ESTOP
```

STM32 状态映射：

| 固件状态 | ROS2 状态/行为 |
| --- | --- |
| `ARM_OK__`、`SVO_OK__` | `MOVING` |
| `ARM_DONE` | 仅从 `MOVING` 进入 `SUCCEEDED` |
| `STOP_OK_`、`STM32_OK`、`ARM_RDY_` | `IDLE` |
| `NO_SOLVE` | `ERROR / 0x0020` |
| `ARM_ERR_` | `ERROR / 0x0021` |
| `BAD_CMD_` | `ERROR / 0x0022` |

节点每 100 ms 读取一次固件状态。运动命令发出后，如果在 `command_timeout_sec` 内没有成功读取固件状态，则进入 `ERROR / 0x0016`。配置文件当前将该超时设为 `3.0 s`。

### 4.3 I2C 数据协议

当前机械臂 I2C 参数：

```text
bus:             5
7-bit address:   0x30
command packet:  32 bytes
status packet:   32 bytes
```

命令包采用 little-endian：

```text
END_EFFECTOR ('A'):
  byte 0       tag = 'A'
  byte 4..7    x, float32，单位 cm
  byte 8..11   y, float32，单位 cm
  byte 12..15  z, float32，单位 cm
  byte 16..19  pitch, float32
  byte 20..23  pitch_min_deg, float32
  byte 24..27  pitch_max_deg, float32
  byte 28..31  duration_ms, uint32

SERVO ('P'):
  byte 0       tag = 'P'
  byte 4       servo id, uint8
  byte 8..11   servo raw target, float32

STOP ('S'):
  byte 0       tag = 'S'
```

状态包布局：

```text
byte 0..7    ASCII firmware status
byte 8..31   float32[6] servo raw positions，依次对应 id 1..6
```

舵机 raw 值按 `0..1000 -> 0..240°` 转换。当前关节角计算为：

```text
joint_rad = direction * (raw - 500) * (240° / 1000) + zero_offset
```

配置中的舵机映射为：

```text
joint_1_base         -> 6
joint_2_shoulder     -> 5
joint_3_elbow        -> 4
joint_4_wrist_pitch  -> 3
joint_5_wrist_roll   -> 2
gripper              -> 1
```

虽然映射已经写入 YAML，但代码仍将它视为“尚未确认”，因此 `MODE_JOINT` 当前不可用。

### 4.4 当前安全措施

已实现的措施包括：

- 拒绝 NaN、Inf、非法模式、非法时长和越界夹爪值；
- 拒绝重复、过期和乱序的非零 `sequence_id`；
- I2C 写入最多重试 3 次；
- 连续 I2C 失败进入错误状态；
- 急停请求发送 best-effort STOP 并锁存；
- 节点退出时尽力发送 STOP 并关闭 SMBus；
- 固件没有反馈时不应伪造有效状态；
- 底盘 `/cmd_vel` 超过 0.5 s 未刷新时自动停车。

### 4.5 已知控制风险

以下问题必须在 VLA 真机闭环前解决：

1. **关节模式被禁用**：VLA 若输出关节动作，当前控制层无法执行。
2. **反馈缺少合理性检查**：任何成功读取的 32 字节包都会令 `position_valid=true`，尚未验证 raw 值范围、跳变和 CRC/序号。
3. **夹爪不是真实反馈**：`ArmState.gripper_position` 当前主要保存最近一次命令值，状态解析没有把 servo id 1 的 raw 值反算为真实夹爪位置。
4. **软限位未进入实际关节执行路径**：YAML 有上下限，但 `MODE_JOINT` 尚未实现。
5. **STOP 物理语义仍需确认**：必须确认固件行为是停止轨迹、保持当前位置还是舵机卸力。

## 5. 视觉链路

### 5.1 当前主链路

`rdk_video_push` 默认从 `/dev/video0` 采集 MJPEG：

```text
/dev/video0
  -> FFmpeg
  -> libx264（默认软件编码）
  -> MPEG-TS
  -> SRT
  -> MediaMTX
  -> RTSP / WebRTC / HLS consumer
```

默认参数：

```text
resolution: 1280x720
output fps:  15
bitrate:     2500k
codec:       H.264
transport:   SRT
restart:     最多 5 次，每次间隔 3 s
```

硬件编码器名称必须通过探测脚本确认，不能根据平台名称猜测。

### 5.2 Hobot 低延迟实验链路

仓库另有 RDK ROS2 编码实验：

```text
hobot_usb_cam -> /image
  -> hobot_codec decode -> /image_nv12
  -> hobot_codec H.264 encode -> /image_h264
  -> hobot_h264_to_srt.py
  -> FFmpeg -c:v copy
  -> SRT
```

SRT `latency` 参数使用微秒；当前建议从 `200000`（约 200 ms）开始。该链路保留 H.264 码流，不进行二次编码。

### 5.3 VLA 使用约束

现有 SRT/MediaMTX 链路适合远程查看和网络实验，但不应直接作为首版闭环 VLA 的唯一视觉输入：公网中继会增加延迟、抖动和时间同步误差。

首版 VLA 推荐由 RDK 在本地取得最新相机帧，缩放并转换为 `uint8 224x224`，与同一时刻的 `/arm/state` 一起通过 `openpi-client` 发送到 WSL policy server。SRT 保留为观测和诊断旁路。

当前还缺少：

- 图像、关节状态和动作的统一时间戳；
- 帧丢失与状态过期判断；
- VLA 使用的相机字段命名；
- 前置相机与未来腕部相机的标定；
- 推理链路的端到端延迟指标。

## 6. OpenPI 与推理环境

### 6.1 当前源码状态

OpenPI 位于：

```text
/home/tang/projects/openpi
```

当前提交：

```text
15a9616a00943ada6c20a0f158e3adb39df2ccac
update output objects to support batching (#975)
```

ALOHA 与 LIBERO 子模块已经拉取。OpenPI 通过 `uv.lock` 固定依赖，并固定一个特定的 LeRobot Git commit；不能单独随意升级 LeRobot。

当前 WSL 环境：

```text
OS:       Ubuntu 22.04 / WSL2
GPU:      NVIDIA GeForce RTX 5070 Ti Laptop GPU
VRAM:     12227 MiB
driver:   591.91
uv:       0.11.28
Python:   项目要求 3.11
JAX:      0.5.3，CUDA backend
PyTorch:  2.7.1+cu128，支持 sm_120
```

12 GB 显存达到 OpenPI 文档中的推理门槛，但不适合本地微调。当前 `.venv` 已完成同步：JAX 和 PyTorch 均能识别 `cuda:0`，并已通过真实矩阵运算；模型 checkpoint 尚未下载。

OpenPI 原始锁文件会从 PyPI 解析到不支持 RTX 50 系 `sm_120` 的 PyTorch cu126。当前本地 `pyproject.toml` 已将 `torch==2.7.1` 和 `torchvision==0.22.1` 显式固定到 PyTorch 官方 cu128 索引，并重新生成 `uv.lock`。普通 `uv sync` 和 `uv run` 不会再把环境降回 cu126。

可使用以下命令复核 GPU：

```bash
cd /home/tang/projects/openpi
GIT_LFS_SKIP_SMUDGE=1 uv sync

uv run python -c "import jax; print(jax.devices())"
uv run python -c "import torch; print(torch.cuda.get_device_name()); print(torch.cuda.is_available())"
```

`GIT_LFS_SKIP_SMUDGE=1` 只阻止依赖仓库在克隆时自动下载不需要的 Git LFS 大文件，不会阻止 OpenPI 按需下载模型权重。

### 6.2 模型选择

计划优先使用 `pi05_base` 作为 Armbot 微调底座：

```text
gs://openpi-assets/checkpoints/pi05_base
```

官方 `pi05_droid`、`pi05_libero` 等 expert checkpoint 可用于验证推理链路，但它们的机器人结构、相机字段、状态维度、动作语义和归一化统计与 Armbot 不一致，不能直接下发到真机。

第一版建议先做 LoRA：数据要求和显存压力更小，也更适合早期数据量有限的自制机械臂。当前 OpenPI 配置中已有明确的 `pi0` 和 `pi0-FAST` 低显存 LoRA 示例；Armbot 的 `pi0.5` LoRA 配置仍需单独实现并验证。

### 6.3 推理服务

OpenPI policy server 默认监听 `8000`：

```bash
uv run scripts/serve_policy.py policy:checkpoint \
  --policy.config=<armbot_config> \
  --policy.dir=<armbot_checkpoint>
```

`<armbot_config>` 和 `<armbot_checkpoint>` 当前都不存在，因此以上是目标接口，不是当前可直接运行的命令。

RDK 侧只需轻量 `openpi-client`，负责组装 observation 并调用 WebSocket。WSL 网络必须确保 RDK 可以访问 policy server；应优先使用局域网直连或 WSL mirrored networking，不应把控制端口直接暴露到公网。

## 7. LeRobot 数据契约

LeRobot 在本项目中主要作为数据集格式和数据加载层，不替代 ROS2、RDK 驱动或 STM32 控制。

首版 episode 至少需要：

```text
task                         string，任务文本
observation.images.front     uint8 图像
observation.state            float32 机器人状态
action                       float32 示教动作
timestamp                    统一时钟时间戳
episode_index                episode 编号
frame_index                  episode 内帧编号
```

建议的初始采样频率为 10 Hz，与当前 `/arm/state` 发布频率一致。每个 episode 必须有清晰的开始、成功、失败和人工中止边界，不能把不同任务或复位过程拼接成一个 episode。

### 7.1 尚未冻结的状态和动作表示

当前可获得的状态候选为：

```text
[joint_1, joint_2, joint_3, joint_4, joint_5, gripper]
```

其中 5 个关节使用 rad，夹爪使用 `[0, 1]`。但夹爪目前不是真实反馈，关节 feedback 也缺少合理性校验，因此该状态还不能直接作为可信训练数据。

动作表示尚未冻结：

- 关节增量动作适合稳定的闭环控制，但必须先实现并验证 `MODE_JOINT`；
- 末端笛卡尔动作可使用现有固件 IK，但当前没有可靠的末端位姿反馈和完整 FK 标定；
- 两种动作不能混入同一个训练配置。

在确定动作表示之前不得开始大规模采集，否则数据可能无法用于训练。

### 7.2 数据同步规则

待实现 recorder 必须：

1. 以同一单调时钟记录图像、状态、实际下发动作和任务文本；
2. 拒绝 `position_valid=false` 或状态超时的样本；
3. 保存“实际执行动作”，不能只保存模型或操作者请求值；
4. 记录急停、通信错误和 episode 终止原因；
5. 数据集只保存必要传感器数据，不提交到 Git；
6. 转换后先可视化抽查，再上传训练服务器。

## 8. 微调方案

### 8.1 服务器

计划使用同一台服务器上的 `4 x 24 GB` GPU：

- LoRA：单张 24 GB 卡接近官方门槛，实际需用小 batch 验证；
- 全量微调：总显存 96 GB，使用 JAX `scripts/train.py` 和 `fsdp_devices=4` 有可行性；
- `batch_size` 必须能被设备数整除；
- 分布式显存不能简单等同于单张 96 GB，小参数、激活和运行时缓冲仍会占用单卡空间；
- 当前 PyTorch 多卡训练脚本使用 DDP，每张卡复制完整模型，不适合作为 `4 x 24 GB` 全量微调的首选路径。

服务器和 WSL 必须使用同一 OpenPI/Armbot 代码 commit。数据集和 checkpoint 使用 `rsync`、对象存储或专用数据盘传输，不进入 Git。

### 8.2 目标训练步骤

以下步骤要等 Armbot 配置实现后执行：

```bash
# 1. 计算训练集归一化统计
uv run scripts/compute_norm_stats.py --config-name <armbot_config>

# 2. LoRA 或全量训练
XLA_PYTHON_CLIENT_MEM_FRACTION=0.9 \
uv run scripts/train.py <armbot_config> \
  --exp-name=armbot_v1
```

全量训练配置需要设置：

```text
fsdp_devices = 4
batch_size % 4 == 0
```

正式长训练前必须先用少量样本运行 50～100 step，确认数据字段、loss、显存、checkpoint 保存和恢复都正常。

### 8.3 Checkpoint 部署

部署时复制完整 step 目录，包括模型参数、assets 和归一化统计，不能只复制一个参数文件：

```bash
rsync -avP \
  user@server:/path/to/checkpoints/<config>/<experiment>/<step>/ \
  /home/tang/models/armbot/<experiment>/<step>/
```

模型权重、训练数据和运行日志均已被 Armbot `.gitignore` 排除，不应提交到仓库。

## 9. VLA 动作安全层

VLA 输出不得直接写入 I2C。待实现的 VLA bridge 至少必须执行：

1. 校验 observation 新鲜度、`position_valid` 和 policy server 健康状态；
2. 校验 action shape、dtype、NaN/Inf 和反归一化结果；
3. 应用关节位置、速度、加速度和夹爪范围限制；
4. 给每条命令分配单调递增 `sequence_id`；
5. 对 action chunk 只执行短前缀并及时重规划，首版不整段开环执行；
6. policy 超时、网络断开、ROS2 状态错误时立即停止继续发动作；
7. 急停具有最高优先级，模型不能解除急停；
8. 记录 observation、原始动作、裁剪后动作、执行状态和延迟。

真机上线顺序必须为：

```text
离线数据回放
  -> shadow mode（只打印动作）
  -> 断开动力/架空检查
  -> 单步低速动作
  -> 小范围闭环
  -> 完整任务
```

## 10. 验证与验收

### 10.1 当前软件检查

根 CI 当前只覆盖 `rdk_video_push` 和 `tests/`。ROS 2 安装完成后应执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/tang/projects/Armbot/ros2_ws

rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

并检查：

```bash
ros2 interface show action_interfaces/msg/ArmCommand
ros2 interface show action_interfaces/msg/ArmState
```

### 10.2 VLA 最小验收标准

- OpenPI 在 WSL 中识别 NVIDIA GPU，并能完成官方 dummy inference；
- recorder 可生成可视化、可重放的 LeRobot episode；
- Armbot config 能计算 norm stats 并完成短训练；
- checkpoint 能从服务器复制到 WSL 并启动 policy server；
- RDK client 能构造与训练完全一致的 observation；
- shadow mode 中 action shape、范围和时序正确；
- 急停、断网、模型超时、过期状态和非法动作测试全部通过；
- 真机低速测试没有越界、突跳或持续运动；
- 端到端记录包含相机时间、状态时间、推理耗时和动作执行结果。

## 11. 推荐实施顺序

1. 完成 ROS 2 Humble 安装，构建并运行全部 ROS 测试；
2. 修复急停 reset 状态机，并把 ROS 测试加入 CI；
3. 架空确认舵机 ID、方向、零位、限位和 STOP 语义；
4. 实现并验证关节动作，或明确选择并完成笛卡尔动作闭环；
5. 下载官方模型并完成 dummy inference；
6. 冻结 observation/action 数据契约；
7. 实现 LeRobot recorder、转换和数据可视化；
8. 新增 Armbot policy adapter、数据配置和 LoRA 训练配置；
9. 在服务器完成短训练、评估和 checkpoint 回传；
10. 实现 RDK VLA bridge，按 shadow mode 到低速闭环的顺序上线。
