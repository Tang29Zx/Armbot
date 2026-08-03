# 项目记忆

## 项目概况

- 项目名：Armbot
- 最近更新：2026-08-01
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
- 当前 I2C v3 保持固定 32 字节布局：ROS `MODE_CARTESIAN_SERVO=5`/`T` 更新
  滚动笛卡尔目标，`MODE_CARTESIAN_SERVO_END=6`/`F` 正常结束流；`H` 停指定舵机，
  `S` 全局急停。普通运动要求 ROS/STM32 版本一致，`S` 保留跨版本停止能力。

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

2026-07-16 用户明确选择取消 RDK 遥操层的全部 XYZ 矩形限位；XYZ 目标直接累计，
最终由 STM32 IK、关节角和舵机范围决定是否执行。请求 pitch 仍在遥操层限制为
`[-90, 90] deg`。`NO_IK_SOLUTION` 是非锁存目标拒绝：控制器报告
`STATE_IDLE/error_code=0x0020`；遥操回退最后成功目标并等待输入回中后自动恢复，
不需要清错或 Home。其他错误仍保持原安全恢复流程。

## 已验证记录

- 2026-08-01：RDK controller 已部署 I2C 同周期快速读重试：只对
  `EAGAIN/EREMOTEIO` 最多尝试 3 次、间隔 5 ms，`ETIMEDOUT` 不重试；耗尽后
  仍只计为一次轮询失败，原有连续失败锁止保持不变。RDK 两个 ROS 包
  构建通过，包内回归为 `113 passed、1 skipped`，控制栈保持停止。覆盖前
  备份为 `/home/sunrise/Armbot-i2c-read-retry-pre-20260801-021140.tar.gz`，
  SHA-256 为 `156f5aee6555b528e9de5ee2fc39577a492830817dceaf54b9378f0af2e7df5b`。
  这不代表内核 `lost arbitration/controller timed out` 根因已修复；STM32 续期修复
  HEX `32380e4d639b234f93acb7b2dd82ce916dca727411b02b104bde730bb137d1e8`
  仍待刷写和实机验收。

- 2026-08-01：RDK 已同步 XYZ `ARM_NOT_READY` 错误边沿修复，controller 会在
  非恢复性固件失败时立即发布 `ERROR/FAILED`；RDK 两个 ROS 包重建成功，包内回归
  为 `110 passed、1 skipped`。覆盖前备份为
  `/home/sunrise/Armbot-readiness-fix-pre-20260801-014544.tar.gz`，SHA-256 为
  `edfee6a8435de3d245a9fef1d649280447252b81d72a16a31b48fd8ba852be06`。
  STM32 空闲反馈自动重同步修复对应 HEX SHA-256 为
  `239506e74f6932035c2e5ae04f94c4d32b27c17c18a450ebf7fa994c93f5058f`；后续已刷写，
  并暴露出增量 IK/确认时间被计入 300 ms watchdog 的边界竞态。续期修复的新 HEX
  SHA-256 为 `32380e4d639b234f93acb7b2dd82ce916dca727411b02b104bde730bb137d1e8`，
  尚未刷写和实机验收。

- 2026-08-01：RDK `/home/sunrise/Armbot` 已部署夹爪和腕转 `U/G` 直接舵机流的
  ROS 接口、controller、teleop、配置与测试；9 个目标文件 checksum 与本地一致，
  `action_interfaces/action_pkg` 构建成功，安装接口包含模式 8～11，包内回归为
  `109 passed、1 skipped`。部署前备份为
  `/home/sunrise/Armbot-direct-servo-pre-20260801-005644.tar.gz`，SHA-256 为
  `2db83642d011df487be19c7fba7916c054f0e1255bb249593e577e4828c1d761`。部署期间
  控制栈保持停止；配套 STM32 HEX 尚未刷写，所以该记录不代表实机运动已验收。

- 2026-07-31：RDK `/home/sunrise/Armbot` 已部署 VLA-only 的 3 点因果中值加
  One Euro 状态滤波，以及 Xbox `0.30 s` 流 watchdog。One Euro 默认参数为
  `min_cutoff=1.0 Hz`、`beta=1.5`、导数截止 `1.0 Hz`、时间重置间隔 `0.5 s`；
  `/arm/state` 和控制安全链保持未滤波。同步的 9 个文件与本地 SHA-256 一致，
  RDK 构建通过，完整 pytest 为 `105 passed、1 skipped`，控制栈保持停止。覆盖前
  备份位于 `/home/sunrise/Armbot-one-euro-pre-20260731-232948.tar.gz`，SHA-256
  为 `9a27350dc7e2df981007996dd941ad0262c4d426798fcaa55545b2f740591b4a`。
  该记录只证明部署和离线回归，不代表真实 rosbag 延迟或 VLA 采集质量已验收。

- 2026-07-17：RDK 单控制栈下 Home/相关命令能进入固件 v3 `EXECUTING`，但随后
  多次返回 `SERVO_FEEDBACK_FAILED (error=6)`；六路 raw 多次全部为 `0`，
  `position_valid=false`。此前已验收关闭的“机械臂舵机无有效反馈”问题因此重新
  打开。RDK 启动早期另有 24 次 `Errno 121`，随后 I2C 能恢复并读取合法 v3 状态，
  所以当前 Home 的直接阻塞是 USART2 舵机反馈而不是 v2/v3 协议不匹配。Windows
  桌面源码与当前工作树关键文件哈希一致，但本次受限 Keil 构建没有生成 AXF/HEX，
  板上 v3 镜像的确切源码快照仍无法由构建产物证明；确认前停止反复 Reset/Home。

- 2026-07-17：已把 I2C v3 滚动伺服对应的 ROS 消息、controller、teleop、探针、
  配置和测试迁移到 RDK `/home/sunrise/Armbot`，保留并核对了既有取消 XYZ 限位的
  `teleop_mapping.py`。迁移前 10 个目标文件备份为
  `/home/sunrise/Armbot-qgoal-v3-pre-migration-20260717.tar.gz`，SHA-256 为
  `bee945bed8e8456b96a1aa768aa17a151653948e4e14fd81ffa8830825997eea`。
  RDK 隔离构建通过，测试为 `80 passed、1 skipped`；迁移时确认 controller、
  teleop、launch 和 joy 进程均未运行，也未启动控制栈。STM32 v3 尚未配套刷写，
  因此当前不得启动普通运动；版本不匹配会拒绝普通命令，跨版本全局 STOP 仍可用。

- 2026-07-17：ROS controller 已实现 I2C v3 和 `ArmState.command_phase`。流式 T
  必须收到匹配 `EXECUTING` 才发送下一目标；队列独立保存最新 T 与 END，Teleop
  在摇杆回中或 A 正常暂停时发送一次 END，Joy/ArmState 异常超时不发送 END。
  最后可达目标按匹配 `PHASE_EXECUTING` 记录，NO_IK/跨度拒绝回退时不会使用尚未
  安装的请求坐标。首轮速度固定为 `0.5 cm/s、5 deg/s`，watchdog 为 `0.20 s`。
  `action_interfaces/action_pkg` 构建通过，controller/teleop/mapping 定向回归
  `78 passed`；完整 `colcon test` 汇总为 `85 tests、0 errors、0 failures、
  1 skipped`。尚未部署 RDK，也未实机验收。

- 2026-07-16：RDK 再次启动第二套 `arm_xbox_control.launch.py` 后，两个同名
  controller/teleop/joy 节点同时存在，`/arm/state` 与 `/arm/command` 各有两个
  发布者/订阅者，两个 controller 竞争 I2C，实测引发 `Errno 121`、I2C 锁存错误、
  重复 sequence 和命令确认超时。单次 `/arm/state` 的成功消息不能代表整套系统
  健康。用户授权后已停止全部 launch 并清理遗留子进程，精确进程检查和 ROS 图均
  确认无机械臂控制节点；当前保持完全停止，后续只能启动一套。
- 2026-07-16：RDK 已部署取消 XYZ 遥操限位、pitch `[-90,90] deg` 的版本。
  部署前停止 enabled 遥操并发送全局 STOP；新栈启动后遥控为 disabled，机械臂
  `IDLE/position_valid/error_code=0`。运行参数已无 XYZ 限位，RDK 已安装映射的
  离线验证可从旧边界 `(20,10,25)` 继续累计到 `(21,11,26)`；尚待用户实机运动验收。
- 2026-07-16：RDK 已继续部署 `NO_IK_SOLUTION` 非锁存恢复：控制器发布
  `STATE_IDLE/error_code=0x0020`，遥操回退最后成功笛卡尔目标并等待输入回中后
  自动恢复。部署前后均为遥控 disabled、机械臂 IDLE、反馈有效且无错误；本地
  action_pkg 回归 72 passed、1 skip，尚待用户在可达域边缘做实机动态验收。
- 2026-07-15：用户曾实机确认 Xbox 遥控抖动已缓解；随后将笛卡尔遥操从原始速度
  提高到 `2×` 后再次报告抖动，原问题已从 `DEBUG_CLOSED.md` 移回 `DEBUG.md`。
  当前 10 Hz/90 ms 不变，满幅平移单步由 `0.1 cm` 增至 `0.2 cm`；输入低通只能
  缓和起停/变向或真实 Joy 噪声，不能消除恒定输入下的离散短轨迹节拍。舵机反馈
  问题仍保持用户已验收关闭。
- 2026-07-15：用户确认当前配套版本由两个仓库的最新提交共同组成：Armbot
  `29117917dc46bb0de31511a59b5e45628ff9dc1d` 与 armbot-stm32
  `6cd94ea57c5b68c0920bf8c3a1be24cb2325b936`。两个 SHA 属于不同仓库，版本发布和
  烧录记录必须成对保存；该映射本身不能证明运行中的 STM32 已烧录对应二进制。
- 2026-07-15：STM32 的 `LeArm.lib` 中 `ikine()` 固定使用负平方根肘部分支；
  `set_pitch_range()` 以 `1 deg` 步长返回第一个可行俯仰解，
  `robot_arm_coordinate_set()` 选解时不参考上一帧关节状态。RDK 只读实机采样证明，
  单调连续的遥控坐标会约每 5～6 个控制周期触发一次搜索俯仰角的 `1 deg` 阶跃，
  造成约 `1.9～2.1 deg` 的多关节目标跳变和对应肩关节反馈锯齿；这是当前移动抖动
  的直接原因，10 Hz/90 ms 微轨迹的停走节拍会进一步放大冲击。
- 2026-07-15：连续 IK 修复保持固定肘型和 I2C v2 不变，使用 `1 deg` 粗搜索加
  `0.05 deg` 边界二分，并依据上一条成功命令的四关节角选择连续候选。
  `1 deg + 120 deg/s * duration` 只作为候选优先线；用户明确选择全部候选超线时
  仍执行最大关节变化最小的解。离线实测轨迹模型把最大相邻关节变化从
  `2.121 deg` 降到 `0.486 deg`，Keil ARMCC 5.06 全量链接为 0 errors；后续用户
  实机确认抖动已缓解并接受当前效果。
- 2026-07-14：用户已实机验收 Home 后实际 y 回到机械中线，原“Home 后实际 y
  未归零”问题已关闭；当前 Home 和开机复位的底座目标均为舵机 6 `raw 500`。
- 2026-07-14：单控制栈 rosbag 证明，RT 闭合后左摇杆期间 ROS 只发布 A 命令、没有
  发布打开用的 P 命令，但舵机 1 仍从约 `raw 693` 回到约 `raw 231`。直接根因是
  开机 reset 给 1 号保存了 `raw≈226/2000 ms` 的 `MOVE_TIME_WAIT_WRITE`；后续 A
  只更新 6～3 号却广播 `MOVE_START`，从而重放 1 号残留的 reset-open 目标。ARM
  批次必须只 START 本次预写的 6、5、4、3 号；广播 START 只能用于明确覆盖了全部
  相关舵机 WAIT 目标的批次。
- 2026-07-14：独立只读固件对舵机 1～6 连续读取完整 9 字节，均为
  `00 55 55 ID 05 1C posL posH checksum`，解析结果
  `rx=OK/uart=0x00/skip=1/checksum valid`。舵机断电时每次请求仍收到单独的
  `0x00`，证明它来自板端半双工收发切换路径，而不是舵机回包。生产解析器遇到
  帧头前非 `0x55` 字节就立即失败，是当前全部反馈无效的直接软件根因；修复应在
  有界窗口内搜索帧头并校验完整帧。
- 2026-07-14 17:25：RDK 部署 v2 后，Home `wire_id=1` 从 `EXECUTING` 进入
  `FAILED/SERVO_FEEDBACK_FAILED`；ROS 映射为 `error_code=0x0024`，保持
  `position_valid=false` 和 `/arm/teleop_enabled=false`，没有把无反馈 Home
  误报为成功。状态包中的舵机 1～6 raw 当时全部为 `0`，具体失败舵机及反馈接收
  根因仍待确认。
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
