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
- VLA 数据采集 v1 使用“每个 episode 一个 rosbag2 + manifest”作为不可变事实层；
  recorder 只订阅图像、Joy、命令、原始状态和滤波状态，不打开 I2C、不发布运动命令。

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

- 2026-08-04：本机 SSH 新增 `Host rdk`，目标为 `sunrise@192.168.3.147`，固定使用
  `~/.ssh/id_ed25519_rdk_armbot`，并以 `ProxyCommand none` 绕过系统级 SOCKS 代理。
  `ssh -G rdk` 已确认解析正确；新地址 TCP/22 已可达，在线读取到的 RSA、ECDSA、
  ED25519 主机指纹与旧地址记录完全一致，确认是同一块 RDK。已安全写入新地址的
  ED25519 主机键；用户通过 `ssh-copy-id` 恢复公钥授权，`ssh rdk` 已可用。
- 2026-08-04：Windows 用户 `Tang29Zx` 的 `~/.ssh/config` 也已将 `Host rdk` 更新为
  `sunrise@192.168.3.147`，继续使用 Windows 独立密钥 `~/.ssh/id_ed25519_rdk`。原
  `authorized_keys` 文本中虽能搜索到该公钥，但 `ssh-keygen` 无法将其识别为有效行；
  保留其他有效密钥并强制追加一份格式正确的 Windows 公钥后，Windows OpenSSH
  `BatchMode` 实测返回 `windows_ssh_ok`。新地址的已核验 ED25519 主机键已写入 Windows
  `known_hosts`。
- 2026-08-04：Xbox Wireless Controller `C0:D6:D5:E7:C2:7A` 已在 RDK 配对、信任并
  连接，受控断开后可重新连接，Linux 会恢复 `/dev/input/js0`。离线修复后原 SDL
  `ros-humble-joy` 已缺失；直接恢复它会连带升级厂商镜像的 systemd、udev 和 Mesa，
  因此改用只新增两个 ROS 包且不升级系统组件的 `ros-humble-joy-linux`。独立测试中
  `joy_linux_node` 成功打开 `/dev/input/js0`，以约 19.65 Hz 发布
  `sensor_msgs/msg/Joy`，8 个轴、16 个按钮；测试没有启动 controller、teleop 或 I2C。
  `arm_xbox_control.launch.py` 与 `action_pkg/package.xml` 已切换到 `joy_linux`，RDK
  安装态软链接和 launch 解析已验证生效。
- 2026-08-04：RDK 离线修复后 `colcon` 命令缺失，`scripts/build-rdk-ros2.sh` 当前不能
  重建 workspace；现有 `--symlink-install` 使本轮 launch 修改直接生效，但后续正式
  构建仍需恢复 `python3-colcon-common-extensions`。该包模拟安装为新增 36 个包、无升级
  或删除，本轮未扩大安装范围；未解决项记录在 `DEBUG.md`。
- 2026-08-04：RDK 离线修复后的可观测验收通过：根分区为 `clean`，当前和上次启动
  无新增 ext4/I/O 错误；`std_msgs`、`sensor_msgs`、`std_srvs` 均为 `ii`，包审计与
  文件校验无异常。`config/fastdds.xml` 已加入 `PREALLOCATED_WITH_REALLOC` 并把 UDP
  allowlist 更新为 `192.168.3.147`，板端正式文件与仓库 SHA-256 一致，旧配置备份在
  RDK `~/.local/state/armbot/config-backups/fastdds-20260804-before-wifi-fix.xml`。
  1280×720 MJPEG 相机连续约 35 秒发布 `CompressedImage`，实测约 29.88 Hz，无 Fast
  CDR 崩溃。默认 calibration 文件仍缺失，不阻塞当前图像采集，但不能据此宣称相机
  已完成几何标定；机械臂实机控制链也仍需独立验收。
- 2026-08-03：`vla_dataset` 的 8 个包文件和采集规格已同步到 RDK
  `/home/sunrise/Armbot`，逐文件 SHA-256 与本地一致；README 和
  `scripts/build-rdk-ros2.sh` 也已同步，后者会随控制包一起构建 recorder。远端只构建
  `vla_dataset`，结果成功，6 项测试全部通过，安装态 `record_episode --help` 正常。
  同步和验证期间未启动 controller、teleop、Joy、相机或 rosbag。RDK 已确认
  `/dev/video0` 支持 MJPEG 1280×720@30 fps，`hobot_usb_cam` 已安装，用户属于
  `video` 组，根分区剩余约 43 GB；真实带消息 episode 尚未录制。
- 2026-08-03：用户明确说明当前实机控制链尚未完成最终验收。后续正式数据的 manifest
  仍应记录实际烧录 HEX SHA-256（可暂记 `unknown`）和 Armbot commit，且不能把当前
  控制状态描述为已验收。
- 2026-08-03：RDK 根分区 `/dev/mmcblk1p2` 存在 ext4 元数据损坏：内核记录 inode
  checksum invalid，`tune2fs` 为 `clean with errors`，错误计数 56；ROS 的
  `sensor_msgs`/`std_srvs` introspection 库和一个生成的 C 文件校验失败，
  `std_msgs` 原已 half-installed。官方 4.9.1 临时 overlay 配合 Fast DDS
  `PREALLOCATED_WITH_REALLOC` 已把 1280×720 MJPEG `/image` 验证到约 29.9 Hz，但
  正式重装被坏 inode 的 `EUCLEAN` 中断。损坏文件备份在 RDK
  `~/.local/state/armbot/package-backups/20260803-ros-msg-corruption`；必须先离线
  `fsck.ext4 -f /dev/mmcblk1p2`，再恢复三个 ROS 包并部署 DDS 配置，修复前暂停正式
  VLA 采集。
- 2026-08-03：RDK 当前根盘经 sysfs 确认为可拔出的 SD/TF 卡，根分区是
  `/dev/mmcblk1p2`。离线 fsck 前的源码与 DDS 配置只读备份位于本机
  `/home/tang/projects/rdk-recovery-20260803-1808`；排除了 `.git` 和可重建的
  `ros2_ws/build/install/log`，共 268 个文件、约 7.6 MB。
- 2026-08-03：该 RDK 镜像的 `/etc/fstab` 未配置根分区，启动后 `/` 已是 rw，
  `systemd-fsck-root.service` 因条件不满足不会检查根盘；不能依赖普通重启自动修复。
  对 `/dev/mmcblk1p2` 的安全 fsck 必须在 TF 卡作为非启动盘、未挂载时执行。
- 2026-08-03：新增独立 `vla_dataset` 被动 episode recorder。固定记录 `/image`、
  `/joy`、`/arm/command`、`/arm/state`、`/arm/state_filtered`，先原子写入
  `recording` manifest，Ctrl+C 等待 rosbag2 完成收尾后标记 `unreviewed`，异常则
  标记 `failed`。本机 ROS 2 Humble 构建和 6 项定向测试通过；真实 rosbag2 烟雾
  测试生成 sqlite3、`metadata.yaml` 和 manifest，SIGINT 正常退出。测试时没有话题
  发布者，因此消息数为 0；RDK 相机与控制话题实录仍待完成。

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

- 2026-08-05：用户要求夹爪操作不再依赖摇杆回中；映射已改为有效 RT/LT 输入
  优先于笛卡尔和腕转输入，模式切换仍先结束上一条流。夹爪接触锁存后会忽略持续
  按住的闭合输入，使摇杆无需先释放 RT 仍可移动机械臂。用户随后明确要求取消 A
  初次使能的输入中立安全门；A 现只检查输入新鲜、同步状态与机械臂状态，非零输入
  会在使能后立即运动，NO_IK/直接舵机拒绝后的自动恢复仍要求输入回中。RDK 源码、
  测试和文档已同步，隔离回归为
  `114 passed、1 skipped`；两次部署前备份分别位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-gripper-priority` 和
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-enable-active-input`。
  部署时控制栈已停止，新逻辑将在下次启动时生效，仍待实机验收。

- 2026-08-05：单栈只读证实手柄完全中立且无 `/arm/command` 时，controller 仍为
  `MOVING/PHASE_NONE/seq=1113`。根因是未下发队列命令提前覆盖公开生命周期，加上
  ACK 后缺少终态 watchdog，丢失终态会永久保留活动 wire ID；Reset 又按公开
  `STATE_MOVING` 阻止恢复。修复后队列不再改写公开状态，活动命令必须在
  `duration_sec + command_timeout_sec` 内到达终态，否则清空活动/队列并报告
  `ERR_CMD_TIMEOUT`；孤立 `MOVING` 自动回 `IDLE`，Reset 只阻止真实活动命令。
  新增 6 个回归，RDK 隔离 Domain 230 完整测试为 `120 passed、1 skipped`；源码、
  测试和契约已同步，备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-fake-moving-lifecycle`。
  生产进程尚未重启，实机连续操作验收待完成。

- 2026-08-05：新栈启动后另一种持续 `MOVING` 被证实为真实命令流：未触碰的
  `joy_linux` 扳机轴启动为 `0.0`，映射将其当作半按，teleop 以约 10 Hz 持续发布
  `MODE_GRIPPER_SERVO`，固件正常返回 `EXECUTING/error=0`。teleop 已改为分别等待
  两个扳机首次出现 `+1.0` 释放端点，在此之前将未初始化轴按中立处理；Joy 超时或
  无效后重新初始化。新增 2 个回归，RDK 隔离完整测试为
  `122 passed、1 skipped`。源码、测试和文档已同步，备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-trigger-init`；当前旧进程
  尚未重启，待安全重启及实机确认。

- 2026-08-05：扳机修复重启后，`F/wire_id=185` 在 3 秒时被主机终态 watchdog
  提前报 `0x0016`，固件约 4.8 秒后才正常 `COMPLETED`，期间下一条 T 被拒绝为
  `ARM_NOT_READY`；当前 `wire_id=367` 再现同一签名。根因是 F/G 无 duration，
  主机误用 0+3 秒，而固件允许最长 30 秒运动收敛。controller 已改为 F/G 使用
  `max_duration_sec + command_timeout_sec`，普通命令仍维持 duration 加 ACK 余量；
  新增失败回归，RDK 隔离完整测试为 `123 passed、1 skipped`。controller、测试和
  契约已同步，备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-stream-end-terminal-window`；
  17:19 已重启单栈并完成受控实机验收：启动扳机均为 `0.0` 时静置无假命令；Home
  成功；`T3～7 → F8` 与快速恢复输入 `T9 → F10 → T11 → F12` 均按序完成，两个
  F 分别约 0.387/0.405 秒到达 `COMPLETED/error=0`。最终
  `SUCCEEDED/COMPLETED/seq=12/position_valid=true`、teleop disabled，静置 5 秒无
  `/arm/command`。测试中一次误用禁用的 MODE_JOINT 被安全拒绝、无硬件运动并已
  Reset，不属于产品复现错误。

- 2026-08-05：用户 17:27 重启单栈后第一条 `seq=1/wire_id=1` 被固件拒绝为
  `ARM_NOT_READY`，同时 I2C/反馈正常且 Joy 无命令。根因是 real teleop 过去只凭
  三帧关节接近 Home 就在进程启动时自动同步，物理姿态却不能证明 STM32 规划器
  ready。已移除实机启动自动同步；每次启动必须显式 Home，并且只有匹配完成、有效
  反馈及 Home 容差内连续三帧同时满足才允许 A。输入中立门仍保持取消。旧行为失败
  回归与修复后完整测试已完成，RDK 隔离结果 `123 passed、1 skipped`。源码、测试和
  文档已部署，备份为
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-explicit-home-on-start`；部署时
  控制栈为空，`action_pkg` 已重建且安装层与源码哈希一致。未自动启动，待显式 Home
  流程实机验收。

- 2026-08-05：显式 Home 现场 `seq=602` 已完成，但关节反馈约 4 分钟后才稳定进入
  Home 容差；等待期间 A 错误显示 `run home first`。teleop 已将 Home/reset pending
  提示前置，并为固件完成后的姿态验证增加 `3 s` 期限；超时会记录实际/期望关节角、
  保持 unsynced、结束静默等待并允许再次显式 Home，不会放宽反馈安全门。修复已部署
  并重建，RDK 完整测试为 `125 passed、1 skipped`，备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-home-feedback-verification`；
  当时运行中的 PID 29741 仍是旧内存代码，需重启控制栈后生效。

- 2026-08-05：首次两次 Home 的 elbow 误差为 `5.04 deg`，仅越过原 `5.00 deg`
  阈值 `0.04 deg`，小于约 `0.24 deg/raw tick` 的反馈量化。Home 容差已放大为
  `5.25 deg` 并部署，完整测试 `126 passed、1 skipped`，备份为
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-home-tolerance-5p25`。同一现场
  后续确有一次固件 v3 包无效后回到 `READY/wire_id=0` 的独立重启事件；这种事件仍
  必须显式 Reset，不能自动清错。部署时 PID 33642 仍加载旧 `5.00 deg`，需重启生效。

- 2026-08-05：夹爪流出现 26 次连续 `STREAM_STEP_TOO_LARGE` 后进入流超时，随后
  `G(gripper)` 33 秒无终态。controller 同轮询下发排队 END 覆盖了公开失败沿，
  teleop 未能暂停并继续累计目标。修复以最后匹配 `PHASE_EXECUTING` 的直接目标
  为基线，把夹爪和腕转候选限制在 `9 deg`，并在 END 下发前显式发布拒绝快照；直接
  拒绝会回滚、只结束一次流并等待输入回中。新增 4 个回归，RDK 正式源码完整测试
  `130 passed、1 skipped`，代码、配置、测试与文档已部署；备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-direct-stream-confirmed-step`。
  生产 PID 35877/35889/35891 未重启，仍需用户重启后完成实机验收。

- 2026-08-05：用户重启确认限步参数生效后，所有夹爪 `U` 均正常
  `EXECUTING/error=0`，但 `G(gripper) seq=277/wire_id=161` 33 秒无终态，使后续
  机械臂命令被跨模式队列阻塞并最终报 `0x0016`。Joy 中立且同期无内核 I2C 错误。
  Xbox 夹爪松开/切换/拒绝现改用有界 `MODE_GRIPPER_STOP/H`，controller 同时清除
  gripper stream 并等待 H 完成后放行机械臂；腕转仍保留 G。新增直接
  `U → H → A` 回归，RDK 正式源码完整测试 `131 passed、1 skipped`，备份为
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-gripper-halt-end`。生产 PID
  47737/47749/47751 未重启，待用户重启后实机验收。

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
