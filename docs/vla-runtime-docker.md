# Docker 化 VLA 在线运行

## 1. 架构与边界

目标主机可以是 Arch Linux 或 Ubuntu。主机只负责 Docker、NVIDIA 驱动和模型
checkpoint；ROS 2 Humble、Fast DDS 和电脑端 bridge 固定运行在 Ubuntu 22.04
容器中，避免主机 ROS 发行版差异进入实机控制链。

```text
RDK /image -> latest-frame 10 Hz /vla/image + /arm/state_filtered
  -> Fast DDS / Domain 29
  -> vla-bridge (ROS 2 Humble container)
  -> localhost:8000 (OpenPI websocket server container)
  -> action chunk [10, 6]
  -> /arm/command/vla
  -> RDK arm_command_mux
  -> /arm/command
  -> arm_controller -> I2C -> STM32
```

OpenPI server 与 ROS bridge 分开运行。前者拥有 GPU/JAX 环境，后者只安装轻量的
ROS、NumPy、Pillow、msgpack 和 websocket client。模型进程不会直接访问 I2C。
OpenPI 容器只把 websocket 映射到 PC 的 `127.0.0.1`；只有 bridge 使用 host
network 加入 DDS，模型端口不会暴露给局域网或 RDK。

## 2. 前置条件

- PC 与 RDK 位于允许双向 UDP/multicast 的同一局域网；
- PC 安装 Docker Engine、Docker Compose plugin 和 NVIDIA Container Toolkit；
- 不使用 Docker Desktop 或 VM 网络；ROS bridge 需要原生 Linux Docker 的
  `network_mode: host`，OpenPI 服务保持普通容器网络；
- RDK 与容器均使用 `ROS_DOMAIN_ID=29` 和 `rmw_fastrtps_cpp`；
- PC 上有包含 `pi05_armbot_lora` 配置的 OpenPI 工作树；
- PC 上有选定训练 checkpoint，例如 `fixed-pick-41-v1/9999`；
- 只能挂载训练完成且不再写入的 checkpoint，不能一边训练同一路径一边提供实机推理；
- RDK 上只能运行一套机械臂控制栈。

Docker Compose 使用 OpenPI 仓库自带的
`scripts/docker/serve_policy.Dockerfile` 构建 GPU 推理服务，不复制或分叉 OpenPI
运行时。`OPENPI_ROOT` 必须指向训练该 checkpoint 时使用的同一版 OpenPI 工作树，
其中至少应存在 `src/openpi/policies/armbot_policy.py`，且
`src/openpi/training/config.py` 注册了 `pi05_armbot_lora`；只用未适配的上游 clone
无法解释 Armbot 的单腕相机、六维状态和六维动作。

## 3. 配置 PC

在 Armbot 仓库根目录执行：

```bash
cp docker/vla-runtime/.env.example docker/vla-runtime/.env
```

编辑 `.env`，所有路径必须是 PC 上的绝对路径：

```dotenv
PC_DDS_IP=192.168.3.100
OPENPI_ROOT=/home/user/Projects/openpi
OPENPI_DATA_HOME=/home/user/.cache/openpi
OPENPI_CHECKPOINT_ROOT=/home/user/openpi_runs/checkpoints
OPENPI_EXPERIMENT_PATH=pi05_armbot_lora/fixed-pick-41-v1/9999
VLA_SHADOW=true
```

`PC_DDS_IP` 必须是 RDK 能直接访问的 PC 局域网 IPv4。容器启动脚本会验证地址，
并由 `fastdds.pc.xml.in` 生成只允许 loopback 和该地址的 Fast DDS profile。仓库中的
`config/fastdds.xml` 是 RDK 专用文件，不能原样用在 PC。

若 PC 或 RDK 启用了防火墙，只允许对端地址访问 Domain 29 使用的 UDP
`14650:14899`，并保留同网段 multicast；不要把这段 DDS 端口暴露到公网。

启动前先验证 Compose 展开结果和 GPU 容器支持：

```bash
docker compose \
  --env-file docker/vla-runtime/.env \
  -f docker/vla-runtime/compose.yml \
  config --quiet
docker run --rm --gpus all nvidia/cuda:12.2.2-cudnn8-runtime-ubuntu22.04 nvidia-smi
```

首次构建并启动：

```bash
docker compose \
  --env-file docker/vla-runtime/.env \
  -f docker/vla-runtime/compose.yml \
  up --build
```

查看日志：

```bash
docker compose \
  --env-file docker/vla-runtime/.env \
  -f docker/vla-runtime/compose.yml \
  logs -f vla-bridge openpi-server
```

停止：

```bash
docker compose \
  --env-file docker/vla-runtime/.env \
  -f docker/vla-runtime/compose.yml \
  down
```

## 4. RDK 控制栈

构建部署后的源码：

```bash
cd /home/sunrise/Armbot
bash scripts/build-rdk-ros2.sh
source ros2_ws/install/setup.bash
ros2 launch action_pkg arm_vla_control.launch.py
```

`arm_vla_control.launch.py` 与旧 Xbox launch 不能同时运行。新 launch 仍启动 Xbox，
但把它的命令改送到 `/arm/command/teleop`，由 RDK 本地 mux 在 teleop 与
`/arm/command/vla` 之间做排他选择。Xbox 的 B 急停仍直接连接
`/arm/emergency_stop`。同一 launch 还把相机原始 `/image` 的最新压缩帧限速转发到
`/vla/image`：采集器仍可在 RDK 使用完整原始帧流，PC 不会通过 DDS 接收无用的
30 FPS 积压数据。

## 5. Shadow 验收

`.env` 默认固定：

```dotenv
VLA_SHADOW=true
```

此模式只接收图像/状态并请求 OpenPI，不发布机械臂命令。日志应每秒出现一次：

```text
shadow inference ... ms first_action=[...]
```

至少确认：

1. RDK 可见且仅有一个 `/arm/state` publisher；
2. bridge 持续收到 `/vla/image`、`/arm/state_filtered` 和 `/arm/state`；
3. policy 输出固定为二维数组且末维至少为 6；
4. inference 延迟稳定小于 `max_policy_age_sec=1.5`；
5. shadow 模式始终不发布可用于获取控制权的 heartbeat；
6. shadow 运行期间 `/arm/command/vla` 没有消息。

## 6. 首次低速实机

只有 shadow 验收通过后才把 `.env` 改为：

```dotenv
VLA_SHADOW=false
```

重新创建 bridge：

```bash
docker compose \
  --env-file docker/vla-runtime/.env \
  -f docker/vla-runtime/compose.yml \
  up -d --build vla-bridge
```

实机步骤固定为：

1. Xbox 长按 `LB+RB+Y`，等待 Home 反馈验证完成；
2. 确认 Xbox teleop 为 disabled；
3. 确认药盒和相机位置与训练分布一致；
4. 在 RDK 请求 VLA 控制权：

   ```bash
   ros2 service call /arm/set_vla_enabled \
     std_srvs/srv/SetBool "{data: true}"
   ```

5. 操作者全程保留 Xbox B 急停；
6. 结束时立即归还控制权：

   ```bash
   ros2 service call /arm/set_vla_enabled \
     std_srvs/srv/SetBool "{data: false}"
   ```

VLA 获取控制权时会使 teleop 失去目标同步。归还后必须再次 Home，不能从 VLA 的
中间目标直接恢复 Xbox 控制。

首次实机默认 `action_scale=0.50`，每个 action chunk 最多执行前 4 步。模型的 held
target delta 只有在 `/arm/state` 返回匹配 `PHASE_EXECUTING/COMPLETED` 后才提交；
夹爪、腕转和笛卡尔流切换前会先发对应 END/STOP。

## 7. 失效行为

RDK mux 默认由 Xbox 控制，VLA 只能通过显式服务获取控制权。获取条件包括：

- Xbox 已 disabled；
- Home 已由 teleop 验证；
- 图像、状态、推理和 heartbeat 均新鲜；
- `position_valid=true`，无 ERROR/ESTOP/error code；
- 机械臂处于 IDLE 或 SUCCEEDED。

VLA 控制期间发生以下任一情况，mux 都会发布一次 `MODE_STOP` 并自动禁用 VLA：

- heartbeat 超过 300 ms；
- `/arm/state` 超过 500 ms；
- 位置反馈失效；
- controller 进入 ERROR/ESTOP；
- Xbox 意外重新 enabled。

必须实测以下故障注入：停止 bridge 容器、断开 PC 网线、停止 policy server，以及
人为制造陈旧图像。所有场景都应在有限时间内停止且要求重新 Home。
