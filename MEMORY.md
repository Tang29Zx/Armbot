# 项目记忆

## 项目概况

- 项目名：Armbot
- 最近更新：2026-07-14
- 技术栈：ROS 2 Humble、Python 3.10、I2C
- 构建与依赖：colcon、ament、APT、pip
- 主要目录：`ros2_ws/src`、`rdk_video_push`、`docs`、`.github/workflows`

## 长期约定

- 机械臂末端坐标单位已由用户确认使用厘米（cm）。
- 上层控制优先使用 `action_interfaces` 的类型化 ROS 2 消息；字符串 Topic 仅用于兼容和调试。
- 机械臂与底盘节点分别拥有对应的 I2C 设备，调试脚本直连 I2C 前必须停止相关 ROS 节点。
- STM32 串口舵机固件开机执行 2 秒复位，标称末端坐标为
  `(15, 0, 2) cm`，复位舵机值反解的末端 pitch 约为 `-54.48 deg`。
- 夹爪规范值固定为 `0=open`、`1=closed`；STM32 理论范围为
  `raw 200=open`、`raw 700=closed`，开机复位值 `raw 226` 接近全开。

## Xbox 手柄映射

以下映射由用户在 RDK 当前 `joy_node` 上实测，消息类型为 `sensor_msgs/msg/Joy`：

- `axes[0]`：左摇杆水平，向左 `+1.0`，向右 `-1.0`。
- `axes[1]`：左摇杆垂直，向上 `+1.0`，向下 `-1.0`。
- `axes[2]`：右摇杆水平，向左 `+1.0`，向右 `-1.0`。
- `axes[3]`：右摇杆垂直，向上 `+1.0`，向下 `-1.0`。
- `axes[4]`：RT，松开 `+1.0`，完全按下 `-1.0`，中间为连续值。
- `axes[5]`：LT，松开 `+1.0`，完全按下 `-1.0`，中间为连续值。
- `buttons[0]`：A；`buttons[1]`：B；`buttons[3]`：X；`buttons[4]`：Y。
- `buttons[6]`：LB；`buttons[7]`：RB；松开为 `0`，按下为 `1`。

遥控安全映射：A 上升沿切换使能；B 请求锁存急停；`LB+RB+X` 长按解除
错误/急停；`LB+RB+Y` 长按回零。急停、错误、Joy/ArmState 超时后必须重新
回零同步，解除锁存本身不会触发运动。

## 已验证记录

- 2026-07-14：RDK 实机证明旧版显式回零恢复存在安全缺口：固件持续返回旧
  `ARM_DONE` 且舵机 2～6 反馈为 `0`、`position_valid=false` 时，控制器仍会按
  命令时长将 Home 标记成功并允许 A 启用遥控。该行为不能作为回零验收依据，
  后续实现必须同时验证匹配命令完成与有效位置反馈。
- 2026-07-13：`action_interfaces`、`action_pkg` 可完成隔离构建；Xbox 遥控与
  控制器相关 `colcon test` 共 48 项，0 failures、0 errors、1 个仓库原有
  copyright skip。
- 2026-07-13：Shadow launch 已实际启动并发布模拟 Joy；节点只订阅
  `/joy_sim`，只发布 `/arm/teleop_command`、Shadow 急停和使能状态。左摇杆
  向上首个 10 Hz 控制周期产生 `x≈15.10 cm`、`sequence_id=1`，未连接 I2C。
- 2026-07-13：开发电脑运行 WSL2 mirrored networking，RDK X5 有线地址 `192.168.127.10`，WSL 有线镜像地址 `192.168.127.100`；两端 ping 和 SSH 端口可达。
- 2026-07-13：PC 与 RDK 的 ROS 2 Humble 统一使用 Domain 29、`rmw_fastrtps_cpp` 和有线网卡白名单；Fast DDS 配置分别位于 PC/RDK 的 `~/.config/armbot/fastdds.xml`，并由 `.bashrc` 的 `FASTRTPS_DEFAULT_PROFILES_FILE` 加载。
- 2026-07-13：Domain 29 的 Fast DDS UDP 端口范围为 `14650:14899`。WSL Hyper-V 防火墙通过 `ROS2-DDS-RDK` 仅允许 RDK `192.168.127.10` 入站；RDK UFW 也必须在 `eth0` 上允许 PC `192.168.127.100` 访问该 UDP 范围。配置后已用临时 topic 双向验证成功。
- 2026-07-13：RDK 的 Fast DDS 自定义传输必须同时保留 UDP allowlist 中的 loopback `127.0.0.1`、有线地址 `192.168.127.10`，并启用 SHM；只有 eth0 UDP 时，RDK 本机 ROS 进程无法发现同机发布者。修复后已验证 RDK 本机和 WSL 均能接收 RDK `joy_node` 发布的 `/joy`。
- 2026-07-13：RDK 已安装 `ros-humble-joy`，`sunrise` 已加入 `input` 用户组；Xbox 控制器可由 SDL 枚举并通过 `sensor_msgs/msg/Joy` 发布。运行 `joy_node` 前需确认当前登录会话的 `id` 包含 `input`。
- 2026-07-13：RDK 的 `/usr/local` 中 `setuptools 80.9.0` 会覆盖 Ubuntu 系统版
  `59.6.0`，导致 ROS 2 Humble 的 `colcon --symlink-install` 报
  `option --editable not recognized`。优先运行 `bash scripts/build-rdk-ros2.sh`；
  手工构建时需在 source ROS 后设置
  `PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH}`。若曾切换安装模式，
  只清理目标包对应的 `build/<package>` 与 `install/<package>` 后重建。
- 2026-07-13：RDK 使用独立 SSH 密钥 `~/.ssh/id_ed25519_rdk_armbot`；私钥只保存在本机，不进入仓库或项目记忆。
