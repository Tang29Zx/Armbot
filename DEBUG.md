# 当前调试问题

最近更新：2026-08-05

本文只记录当前仍未关闭的问题；已关闭问题完整归档到 `DEBUG_CLOSED.md`。
状态含义：

- **已证实**：已有代码或实机证据。
- **待确认**：现象存在，但根因尚未由实验确定。
- **已修改，待验证**：修复已落到代码或硬件，但尚未完成规定验证。
- **已验证，待用户验收**：agent 验证已通过，等待用户确认真实使用结果。
- **用户已验收，已关闭**：用户确认后立即移入 `DEBUG_CLOSED.md`，不得留在本文。
- **阻塞验收**：修复前不得把真实机械臂控制描述为完成。

## P1：控制器长期停在假 `MOVING` 状态

状态：**已验证，待用户验收**

### 现象与根因

- 2026-08-05 单栈只读检查确认：Xbox 全部摇杆和扳机均为中立，4 秒内没有新的
  `/arm/command`，但 `/arm/state` 仍保持
  `MOVING/PHASE_NONE/sequence_id=1113/error_code=0`。此前日志出现多次
  `STREAM_STEP_TOO_LARGE`、流 END 拒绝和 ACK 超时，说明这不是手柄噪声导致的
  持续运动命令。
- controller 在活动命令等待 ACK/终态时收到跨模式命令或流 END，会把新命令存入
  队列，却提前把公开状态改成该未下发命令的 `MOVING/PHASE_NONE`。该命令没有
  `wire_command_id`，因此固件不可能返回与它匹配的终态。
- 原 ACK watchdog 在收到 `ACCEPTED/EXECUTING` 后立即解除，但没有第二个终态
  deadline；如果 `COMPLETED/FAILED` 被错过或 END 失败，活动命令和跨模式队列可
  永久保留。Reset 又只检查公开 `STATE_MOVING`，导致假状态无法恢复。

### 已修改与验证

- 未下发的排队命令不再改写公开 `state/phase/sequence_id`，公开生命周期始终对应
  当前真实 `wire_command_id`。
- 新增 `duration_sec + command_timeout_sec` 终态 watchdog；超时会清除活动命令、
  流状态和队列，并报告 `ERR_CMD_TIMEOUT`。轮询会把无 pending、无活动 wire ID 的
  孤立 `MOVING` 自动协调回 `IDLE`。
- Reset 改为检查 pending/活动 wire ID；假 `MOVING` 可以清错，真实活动命令仍阻止
  Reset。
- 6 个针对性回归先在旧实现全部失败，修复后全部通过；RDK 隔离 ROS Domain 230 的
  完整 `action_pkg` 回归为 `120 passed、1 skipped`。controller、测试与接口契约已
  同步到 RDK，哈希与本地一致；覆盖前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-fake-moving-lifecycle`。
  生产控制栈尚未重启，当前 PID 5713 仍运行旧进程内存中的代码，不能把实机问题
  标记为已验收。
- 部署后只读快照显示旧进程已从假 `MOVING` 转为
  `STATE_ERROR/PHASE_FAILED/error_code=0x0001`，I2C 连续失败 5924 次，最新错误为
  `[Errno 121] Remote I/O error`；`position_valid=false`、teleop disabled。该 I2C
  故障与本次生命周期修复是两个问题，恢复总线前不得重启后直接使能运动。
- 2026-08-05 16:54 新栈启动后，I2C 已恢复且反馈有效；此时再次出现的 `MOVING`
  不是 controller 孤立状态。只读采样发现 `joy_linux` 持续上报
  `axes[4]=+1.0、axes[5]=0.0`，teleop 因而以约 10 Hz 发布
  `MODE_GRIPPER_SERVO`，固件对每个新 wire ID 正常返回 `EXECUTING/error=0`。
  根因是未触碰的绝对扳机轴启动值为 `0.0`，而映射把 `+1.0` 作为释放、`0.0`
  作为半按。
- teleop 现对两个扳机分别设置初始化门：首次真实观察到释放端点前，将该轴按
  `+1.0` 中立处理；Joy 超时或无效输入后重新初始化。新增用例先复现旧实现的假
  夹爪流，修复后定向用例通过，RDK 隔离完整回归为 `122 passed、1 skipped`。
  teleop、测试和文档已同步到 RDK，覆盖前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-trigger-init`；当前 PID
  13860 仍是同步前启动的旧进程，需安全重启后才会加载修复。
- 2026-08-05 17:05 新栈加载扳机修复后，启动假夹爪流已消失，但暴露出主机终态
  watchdog 对流 END 的时长建模错误：`F/wire_id=185` 执行 3 秒时被主机提前报
  `0x0016`，固件约 4.8 秒后才正常返回 `COMPLETED`，总收敛时间约 7.8 秒。主机
  提前清除活动 wire ID 后发送 `wire_id=186`，固件仍在完成 F，因而正确拒绝为
  `ARM_NOT_READY`；当前 `wire_id=367` 是同一签名的再次复现。
- 根因是 `F/G` 帧没有 duration 字段，旧实现把它们的期望时长当成 0，只留下 3 秒
  ACK 余量；但固件运动完成上限为 30 秒。controller 已改为 `F/G` 使用
  `max_duration_sec + command_timeout_sec`，普通命令仍使用自身 duration 加 ACK
  余量。失败回归先复现 3 秒误杀，修复后通过；RDK 隔离完整回归为
  `123 passed、1 skipped`。controller、测试和契约已同步到 RDK，覆盖前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-stream-end-terminal-window`；
  当前 PID 19838 仍为同步前旧进程，待安全重启和实机 END 验收。
- 2026-08-05 17:19 agent 完成受控实机验收：只启动一套新栈（controller PID
  26279），`joy_linux` 启动时两个未触碰扳机均为 `0.0`，静置 3 秒没有任何
  `/arm/command`，controller 保持 `IDLE/error=0`，证明扳机初始化门已实际生效。
- 当前位置附近 Home `seq=2` 正常 `SUCCEEDED`；随后发送 5 个总位移仅 `0.05 cm`
  的 T（`seq=3～7`），全部进入匹配 `EXECUTING`，F `seq=8` 在 0.387 秒后
  `COMPLETED/error=0`。再验证快速重新推杆边界 `T9 → F10 → T11 → F12`，所有
  wire 生命周期按序执行，最终 F 在约 0.405 秒完成，无 `ARM_NOT_READY`、终态超时
  或状态错配。
- 测试脚本第一次误用 `mode=2`，被 controller 按契约拒绝为 `MODE_JOINT disabled`，
  未产生硬件运动；随后显式 Reset 并使用接口定义的正确枚举继续。最终静置 5 秒无
  意外命令，状态为 `SUCCEEDED/COMPLETED/seq=12/position_valid=true/error=0`，
  teleop disabled，单栈 5 个预期进程均存活。当前日志中除此人为拒绝外仅有手柄
  force-feedback 不可用警告，不影响控制输入。
- 2026-08-05 17:27 用户重新启动单栈后，第一条命令 `seq=1/wire_id=1` 立即被固件
  拒绝为 `ARM_NOT_READY`；此时 I2C 正常、`position_valid=true`、Joy 无命令且
  teleop disabled。根因是 real teleop 启动时只凭连续三帧关节接近 Home 就自动
  `_synced=True`，但 ROS 进程重启后该物理姿态不能证明 STM32 内部滚动规划器仍
  ready，导致 A 后第一条 T 才暴露错误。
- 已移除实机启动自动同步路径。现在只有显式 Home 的匹配 `COMPLETED`、有效反馈、
  Home 容差内连续三帧三项同时满足时才设置 synced；启动时 A 明确提示
  `target is not synchronized; run home first`。扳机/摇杆不必回中的既有行为不变。
  失败回归先复现旧自动同步，修复后定向用例与 RDK 隔离完整回归
  `123 passed、1 skipped` 均通过。源码、测试和文档已部署到 RDK，覆盖前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-explicit-home-on-start`；
  部署时控制栈为空，并已重建 `action_pkg`；安装层实际加载文件与源码 SHA-256 一致。
  本轮未自动启动或产生机械臂命令，待用户按显式 Home 流程实机验证。
- 2026-08-05 17:45 现场确认 Home 手势并非未响应：夹爪、腕转和笛卡尔命令均已
  下发，`seq=602` 也已为 `SUCCEEDED/COMPLETED`，但关节反馈长时间未稳定进入 Home
  容差，直到约 4 分钟后才出现 `home feedback verified`。期间 A 因检查顺序错误一直
  误报 `run home first`，使正在执行的 Home 看起来像没有响应。修复为 Home/reset
  pending 优先提示 `home/reset operation is in progress`；固件完成后增加 `3 s`
  反馈验证期限，超时会打印 position validity、实际/期望关节角，清除 pending 并允许
  显式重试，仍保持 unsynced，绝不绕过反馈使能。修复、配置、测试和文档已部署，
  RDK 完整隔离回归为 `125 passed、1 skipped`，源码/build/install 三层哈希一致；备份
  位于 `/home/sunrise/.local/state/armbot/deploy-backups/20260805-home-feedback-verification`。
  部署前启动的 PID 29741 仍为旧内存代码且没有新参数，需安全重启控制栈后生效。
- 2026-08-05 18:00 新栈首次两次 Home 失败的唯一越界关节为 elbow：实际
  `-84.00 deg`、预期 `-89.04 deg`，误差 `5.04 deg`，仅比原 `5.00 deg` 容差多
  `0.04 deg`，且小于 Lobot 反馈约 `0.24 deg/raw tick` 的一个量化刻度。随后
  `18:00:40` 固件状态独立出现无效 v3 包，再回到 `READY/wire_id=0`，controller 正确
  锁存 `firmware restarted`；这才是该次必须 Reset 的原因，不能自动清除。Home 预置
  容差已收敛地放大到 `5.25 deg`，新增 `5.04 deg` 边界回归；RDK 完整隔离测试
  `126 passed、1 skipped`，源码/build/install 哈希一致。备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-home-tolerance-5p25`；当前 PID
  33642 仍持有旧 `5.00 deg` 参数，重启后才加载新预置。
- 2026-08-05 18:10 后续夹爪操作首先出现 26 次连续
  `FAILED/STREAM_STEP_TOO_LARGE (0x0026)`，随后固件流进入
  `STOPPING/STREAM_TIMEOUT`，最后 `G(gripper)` 等待 33 秒仍无终态。首个拒绝发生
  后，controller 在同一次轮询中立即下发排队 END，公开拒绝沿被新的 `MOVING`
  覆盖，teleop 因而没有暂停并继续累计夹爪目标，形成错误风暴。
- 主机侧已改为以最后匹配 `PHASE_EXECUTING` 的直接舵机目标为基线，夹爪/腕转每次
  最多前进 `9 deg`；未确认期间只刷新相同边界。controller 会在排队 END 前立即
  发布拒绝快照，teleop 收到后回滚、只结束一次流并暂停等待输入回中。4 个新增
  定向回归和 RDK 正式源码完整回归均已通过，最终结果为
  `130 passed、1 skipped、3 warnings`。源码、build 软链接和 install 运行时解析到的
  controller/teleop 哈希一致；部署前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-direct-stream-confirmed-step`。
  构建前后的生产进程仍为 PID 35877/35889/35891，没有 pkill、重启或运动命令；它们
  仍运行旧内存代码，需用户重启后再做 60 秒实机验收。
- 2026-08-05 18:41 用户重启新栈后复测，`gripper_stream_max_target_step=0.075` 与
  `wrist_stream_max_target_step_deg=9.0` 已由运行节点加载；夹爪 `U` 从 wire 60 起均为
  `EXECUTING/error=0`，不再出现跨度拒绝。新的唯一失败为
  `G(gripper) seq=277/wire_id=161` 在 33 秒内没有任何终态，controller 报
  `0x0016 no terminal firmware lifecycle`；期间机械臂命令因跨模式互斥只能排队，
  因而表现为夹爪动过后机械臂失控。同期 Joy 已中立，内核没有 I2C timeout、仲裁
  丢失或文件系统错误，排除上轮步长和本轮总线故障。
- Xbox 夹爪结束路径已改为 `MODE_GRIPPER_STOP/H`，仅腕转继续使用 `G`。controller
  在 H 抢占 U/G 时立即清除 gripper stream，并在 H 匹配完成后放行机械臂命令。
  3 个定向回归与 RDK 正式源码完整回归通过，结果为
  `131 passed、1 skipped、3 warnings`；源码、build 软链接和 install 运行时哈希
  一致，部署前备份位于
  `/home/sunrise/.local/state/armbot/deploy-backups/20260805-gripper-halt-end`。生产栈
  PID 47737/47749/47751 未被停止或重启，仍需用户重启后的实机验收。

### 关闭标准

- 在机械臂周围清空并重新启动单套控制栈后，静止时不再长期出现
  `MOVING/PHASE_NONE`。
- 分别验证夹爪→笛卡尔、笛卡尔→夹爪及流 END；每条活动命令最终到达终态，或在
  deadline 后进入带明确错误信息的超时状态。
- 连续实机操作至少 60 秒无假 `MOVING`，用户确认后归档。

## 当前实机快照

- 2026-07-16 19:35 只读检查发现 RDK 同时运行两套
  `arm_xbox_control.launch.py`：旧栈 PID 19974（18:24 启动）和新栈 PID 25127
  （19:31 启动），因此 `/arm/state`、`/arm/command` 和
  `/arm/teleop_enabled` 均有 2 个同名发布者，两个控制器同时访问同一 I2C 地址。
  当前单次 `/arm/state` 可读到 `STATE_SUCCEEDED/sequence_id=465/error_code=0`
  且位置有效，但该消息只能代表其中一个发布者；两套遥操均已发布 disabled，
  ROS 图整体不能视为健康单栈状态。
- 新 `NO_IK` 恢复在 19:33:37 已按设计执行：目标被拒绝、回退最后成功目标、输入
  回中后自动恢复。紧接着第二控制器因 I2C 竞争出现连续 `Errno 121`，累计到
  `error_code=0x0001`；随后两栈互相接收命令又产生 `0x0017` 重复序号和
  `0x0016` 匹配确认超时，使遥操失去同步并要求 Home。当前首要处置是停止两套
  launch 后只启动一套；在恢复单栈前不继续运动或用反复 reset/Home 掩盖竞争。
- 2026-07-16：用户授权清理后，19:31 启动的前台栈已经退出；agent 向 18:24
  启动的后台 launch PID 19974 发送 `SIGINT`，再以 `SIGTERM` 清理遗留的
  controller、teleop 和 joy 子进程。精确进程匹配无输出，ROS 图中也已无相关节点，
  `/arm/state` 已无发布者。当前控制栈为完全停止状态，尚未重新启动。
- 2026-07-15：用户此前确认 Xbox 遥控抖动已缓解，但将笛卡尔遥操提高到原始速度
  `2×` 后再次报告抖动；原条目已从 `DEBUG_CLOSED.md` 移回本文并重新打开。舵机
  反馈问题仍保持已验收关闭。下方早期反馈失败记录继续作为其他尚未关闭的命令
  生命周期、I2C 恢复和故障注入问题的历史证据。
- 2026-07-14 23:09 启动的单控制栈日志及用户补充的终端日志中，共出现至少 13 次
  `FAILED/SERVO_FEEDBACK_FAILED`（ROS `error_code=0x0024`）。首次发生于
  `wire_id=341`，失败前舵机 1 和 6 的反馈仍分别约为 `raw 511/527`，下一条命令
  又恢复为 `error=0`；说明当前“莫名进入 ERROR”的直接触发是运动期瞬态目标舵机
  回读失败，不是所有反馈持续失效。现有状态包仍不能指出失败舵机 ID。
- 同一会话在 23:24:53 起又出现持续 I2C 失联，`sequence_id=885` 不变，连续失败
  从 1 增至 11914；23:44:49 板端恢复后立即重新读到有效状态。ROS 阈值正确地在
  第 5 次失败后进入 `error_code=0x0001`，但板端仍没有自动恢复。
- 2026-07-14 21:08:53 起，RDK 在没有新运动命令、`sequence_id=1341` 保持不变时
  持续无法读取 STM32：先出现 `Errno 11/121`，随后稳定为
  `Errno 110 Connection timed out`。到 21:58 已连续失败至少 2849 次；22:00 两次
  重启 ROS 后仍从 `sequence_id=0` 立即失败。用户执行 I2C reset 后通信恢复；实时
  状态为 `SUCCEEDED/sequence_id=138/position_valid=true/error_code=0`，夹爪反馈约
  `0.996`，遥控保持禁用。
- 2026-07-14 19:56 后，板端已恢复为 I2C v2 生产固件；RDK 连续 9 个
  `/arm/state` 样本均为 `position_valid=true`，证明帧头重同步修复已使 1～6 号
  反馈恢复。当前固件产物仍未携带源码 commit/hash。
- 本次 HOME `sequence_id=1/wire_id=1` 返回
  `FAILED/SERVO_FEEDBACK_FAILED`；错误锁存为 `state=3/error_code=36`，但随后反馈
  持续有效，`/arm/teleop_enabled=false`。这表明失败发生在 HOME 运动跟踪期间的
  临时连续读失败，不是静止状态下所有舵机持续无回包。
- 2026-07-14 只读诊断固件曾对 ID 1～6 连续多轮得到
  `tx=OK/rx=OK/n=9/uart=0x00/parse=OK/skip=1`，完整帧为
  `00 55 55 ID 05 1C posL posH checksum`，所有 checksum 正确。舵机断电时每次
  请求仍收到单独的 `0x00`，证明前导零来自板端半双工收发切换路径，不是某个舵机
  回包，证明六个舵机、ID、供电、总线和 MCU UART 收发均正常。
- 2026-07-14 17:27，RDK 的 ROS 源码与本地均为
  `fix/arm-control-v2@642eeec`，工作区干净；当前固件已返回 v2 二进制生命周期，
  但固件二进制尚无可读取的源码版本号。
- 17:25:21，Home `sequence_id=1/wire_id=1` 进入 `EXECUTING`；约 102 ms 后
  返回 `FAILED/SERVO_FEEDBACK_FAILED`。当前 `/arm/state` 保持
  `STATE_ERROR`、`error_code=0x0024`、`position_valid=false`。
- 当前状态包中的舵机 1～6 raw 反馈全部为 `0`，`/status_topic=ARM_ERR_`，
  `/arm/teleop_enabled=false`。新安全门已阻止无反馈 Home 被同步为成功。
- 本地 STM32 已增加 USART2 中断源判定、FE/NE/ORE 错误恢复和最多 3 次有界
  重试；这些改动只能修复中断状态错误或偶发丢帧，尚无证据证明它们是当前持续
  无回包的根因。该固件尚未烧录，当前 RDK 快照仍来自修改前二进制。
- 11:10 的旧协议快照曾出现夹爪槽位约 `raw 498`、舵机 2～6 为 `0`；该值可能
  来自响应错配，不能作为当前 v2 固件中舵机 1 可读的证据。
- Home 配置固定为 `(x=15, y=0, z=2, pitch=-54.48 deg)`，并非从摇杆累计
  坐标生成。
- 当前节点维护的是“最后一次命令目标”，不是实测末端坐标。由于完成状态不可靠，
  维护目标已经可能与真实机械臂姿态脱节。

## P2：RDK 离线修复后缺少 colcon 构建工具

状态：**已确认**

### 现象与证据

- 2026-08-04：同步 Xbox `joy_linux` launch 修改后执行
  `bash scripts/build-rdk-ros2.sh`，脚本在调用 `colcon` 时返回
  `command not found`；`python3-colcon-common-extensions` 当前未安装。
- 现有 workspace 是 `--symlink-install`，安装态 launch 和 `package.xml` 最终解析到
  source 目录，因此本轮两个文件的修改已经生效；`ros2 launch ... --show-args` 可正常
  解析，但这不能替代一次干净构建。
- `apt-get --simulate` 显示恢复 `python3-colcon-common-extensions` 会新增 36 个构建和
  测试相关包、不会升级或删除现有包。本轮 Xbox 连接不依赖这些包，尚未扩大安装范围。

### 影响

- 当前运行态不受影响，但后续清空 build/install 或修改非软链接产物时无法重建项目。

### 关闭标准

- 恢复 `colcon` 后运行 `bash scripts/build-rdk-ros2.sh` 成功，并完成相关测试；
  `dpkg --audit` 无输出。

## P1：RDK USB 相机在 1280×720 启动时发生 Fast CDR buffer 崩溃

状态：**已验证，待用户验收**

### 现象与证据

- 2026-08-03：用户在 RDK 使用 `hobot_usb_cam`、`/dev/video0`、MJPEG、
  `1280×720@30 fps`、`usb_zero_copy=False` 启动相机；进程创建成功后立即以
  `exit code -6` 退出。
- 崩溃异常为
  `eprosima::fastcdr::exception::NotEnoughMemoryException: Not enough memory in the buffer stream`。
- 启动日志确认 `hobot_shm` 加载的是
  `/home/sunrise/.config/armbot/fastdds.xml`，并设置
  `RMW_FASTRTPS_USE_QOS_FROM_XML=1`。该自定义 profile 没有 DataWriter/DataReader 的
  `historyMemoryPolicy`，因此 Fast DDS 回退到固定大小的 `PREALLOCATED`；640×480 和
  1280×720 均可复现，RTPS 还明确报告 payload 大于 history payload 且不可扩容。
- 增加默认 `PREALLOCATED_WITH_REALLOC` 后暴露出第二个独立故障：
  `CompressedImage` 和 `SetBool` 的 introspection type-support 导出符号已发生位级
  损坏。`dpkg -V` 确认
  `libsensor_msgs__rosidl_typesupport_introspection_cpp.so`、
  `libstd_srvs__rosidl_typesupport_introspection_cpp.so` 和
  `sensor_msgs/msg/_imu_s.c` 校验失败；`ros-humble-std-msgs` 原本已处于
  `purge ok half-installed`。
- 使用官方 4.9.1 deb 解包出的只读临时 overlay，加上
  `PREALLOCATED_WITH_REALLOC` profile 后，1280×720 MJPEG 相机成功持续发布
  `sensor_msgs/msg/CompressedImage`，实测约 29.9 Hz，消息格式为 `jpeg`。
- 2026-08-03 正式重装时，ext4 对 `std_msgs` 的多个 `int64` 头文件返回
  `EUCLEAN/Bad message`。内核日志确认 `/dev/mmcblk1p2` 已累计 56 次 ext4 错误，
  多个 inode checksum invalid；`tune2fs` 显示 `clean with errors`。因此根因已从
  ROS 配置追到根文件系统损坏，挂载状态下不得继续强行修包或采集训练数据。
- 本轮已将原损坏的三个文件备份到
  `/home/sunrise/.local/state/armbot/package-backups/20260803-ros-msg-corruption`。
  重装被文件系统错误中断后，`sensor_msgs 4.9.1` 为 unpacked、
  `std_srvs 4.9.1` 已配置、`std_msgs 4.9.0` 仍为 half-installed；必须先离线 fsck，
  再恢复包管理状态。
- `/sys/block/mmcblk1/device/type` 已确认启动盘为可拔出的 SD/TF 卡；根分区固定为
  `/dev/mmcblk1p2`。断电取卡前，RDK 项目源码（排除 build/install/log 和 `.git`）及
  当前 DDS 配置已只读备份到本机
  `/home/tang/projects/rdk-recovery-20260803-1808`，共 268 个源码/配置文件、约 7.6 MB。
- 当前镜像没有可用的“下次启动自动修根分区”路径：`/etc/fstab` 未配置根分区，
  `systemd-fsck-root.service` 因 `/` 已经以读写方式挂载而保持 inactive，且没有
  `/run/initramfs/fsck-root` 记录。不得在当前挂载根分区上运行 `e2fsck`；最小安全
  修复路径是关机取出 TF 卡，在 Ubuntu Live/另一套 Linux 中离线执行 fsck。
- 2026-08-03 修复后验收尝试：旧地址 `192.168.127.10:22` 返回 `No route to host`，
  WSL 在 `192.168.127.0/24` 的邻居探测没有发现任何可达主机，连
  `192.168.127.1` 也没有 ARP 响应。当前无法读取板端 fsck、内核或 dpkg 结果，
  因而不能标记为已修复；需先恢复 RDK 启动和有线连接，或从板端提供新 IP。
- 2026-08-04 修复后验收续测：新地址 `192.168.3.147:22` 已可达，在线主机指纹与旧
  RDK 的三类已保存指纹完全一致，确认设备身份。当前 SSH 返回
  `Permission denied (publickey,password)`，说明客户端公钥尚未被板端账户授权；需先
  用 `ssh-copy-id` 恢复授权，之后才能读取离线 fsck、内核、dpkg 和相机启动结果。
- 2026-08-04 用户恢复 SSH 公钥授权后完成 agent 验证：`tune2fs` 显示根分区状态为
  `clean`，本次和上次启动的内核日志均无 ext4、checksum、I/O 或 `EUCLEAN` 错误；
  `ros-humble-std-msgs`、`ros-humble-sensor-msgs`、`ros-humble-std-srvs` 均为 `ii`，
  `dpkg --audit` 与三包 `dpkg -V` 均无输出，两个曾损坏的 ROS interface 可正常加载。
- 仓库和 RDK 的 `fastdds.xml` 已同时加入 DataWriter/DataReader
  `PREALLOCATED_WITH_REALLOC`，UDP allowlist 更新为 `192.168.3.147`，两端 SHA-256
  均为 `f34b95353d2c5be5d1bc8f89cbb866972a696a86e9efbfa98c0ea96463303707`；旧配置保存在
  `/home/sunrise/.local/state/armbot/config-backups/fastdds-20260804-before-wifi-fix.xml`。
- 使用正式配置启动 1280×720、MJPEG、30 fps 相机，`/image` 类型为
  `sensor_msgs/msg/CompressedImage`，连续约 35 秒稳定在约 29.88 Hz；日志中没有
  `NotEnoughMemory`、进程崩溃或 `exit code -6`，停止后无相机进程残留，内核也没有
  新增文件系统错误。厂商默认 calibration 文件缺失的警告仍存在，但不阻塞当前
  `/image` 采集；需要几何标定结果时另行补齐。

### 影响

- 修复前 `/image` 没有可靠发布者，`vla_dataset` 会得到缺少视觉观测的无效 episode；
  本轮相机发布链已恢复。
- 修复前根文件系统元数据和 ROS 系统包均不完整，存在生成损坏 rosbag 的风险；本轮
  可观测的文件系统与包完整性检查已恢复正常。
- 本问题的验证不代表机械臂、I2C 或 STM32 控制链已完成最终验收。

### 关闭标准

- 在根分区未挂载时对 `/dev/mmcblk1p2` 完成 `fsck.ext4 -f`，重启后确认内核没有新增
  ext4/inode checksum 错误。
- 重新安装并配置 `std_msgs`、`sensor_msgs`、`std_srvs`，要求三包均为 `ii`、
  `dpkg --audit` 和对应 `dpkg -V` 无输出。
- 部署仓库中的 `config/fastdds.xml`，保留现有 UDP allowlist 和 SHM，同时让可变长度
  payload 使用 `PREALLOCATED_WITH_REALLOC`。
- 最终命令持续运行至少 30 秒，`/image` 类型正确、频率稳定且进程无异常退出。
- 修复不得破坏 RDK 本机与 WSL 的 Domain 29 ROS 发现，也不得要求启动第二套控制栈。

## P1：右摇杆左右无法控制腕部旋转

状态：**空闲 readiness 修复已实机生效；新 watchdog 时序 HEX 待刷写与复测**

### 现象与根因

- 2026-07-31：用户实机确认当前滚动伺服基本可用，但右摇杆左右不能让夹爪绕腕部
  旋转。
- 当前 `teleop_mapping.py` 将 Xbox `axes[2]` 固定积分到笛卡尔 `pitch`；
  `ArmCommand` 和 controller 又没有腕转专用模式。虽然舵机映射已确认
  `joint_5_wrist_roll -> servo 2`，该输入路径从未向 2 号舵机生成位置命令。
- 通用 `MODE_JOINT` 仍按安全契约禁用，因此不能通过启用全部关节直控绕过该缺口。
- 2026-07-31 最新实机复测：用户确认两个摇杆控制均无响应，但 RT/LT 夹爪仍能
  正常张开闭合。RDK 只运行一套 joy/teleop/controller；`joy_node` 持有当前 Xbox
  `/dev/input/event1`，controller 持有 `/dev/i2c-5`。固件日志持续返回合法 v3
  `EXECUTING/COMPLETED`，且夹爪 raw 随 RT/LT 命令变化，排除整条 Joy、ROS、I2C
  或 STM32 链路失效。
- 同期 controller 日志只看到 1 号夹爪舵机对应的 wire 生命周期，没有观察到
  摇杆触发的 ARM/腕转动作。代码审查确认一个生命周期竞态：controller 在
  `COMPLETED` 后先刷新队列，新命令会在同一个回调中覆盖成功状态，10 Hz 状态定时器
  因而可能不发布旧命令的完成边沿。teleop 漏掉 `MODE_GRIPPER_STOP` 完成后会永久保留
  `_gripper_stop_pending_seq`，该变量会明确阻止后续笛卡尔摇杆。
- 用户随后停止旧栈并重新启动；`jstest --event /dev/input/js0` 已确认 Xbox 原始
  摇杆轴有响应，排除手柄硬件或内核输入层失效。19:20:40 的 teleop 日志明确记录
  Home 组合键已触发并发送夹爪打开命令；controller 收到固件
  `EXECUTING/wire_id=1`，但在该次进程退出前始终没有收到 `COMPLETED`。因此三阶段
  Home 卡在第一阶段，腕转归零和笛卡尔 Home 从未发送。19:20:46 按 A 被明确拒绝为
  `target is not synchronized; run home first`。当前 Home 无反应的直接原因是夹爪
  打开生命周期不完成，不是 Home 组合键映射错误。反馈约 raw 249、目标 raw 200，
  已经接近安全全开位置，但旧 Home 状态机只接受精确 `COMPLETED`，因此无法继续。
- 2026-07-31：用户进一步确认夹爪开闭和右摇杆腕转都经常“一卡一卡”。两条路径
  虽映射到不同舵机（夹爪 1、腕转 2），但共用同一旧式单舵机 `P` 协议：teleop
  以 `10 Hz` 积分目标，每条命令固定执行 `90 ms`。满幅腕转每周期增加约 `2 deg`
  （约 8 raw），夹爪每周期增加 `0.05` 规范值（约 25 raw），理想时序也会形成
  `90 ms` 运动加约 `10 ms` 停顿。
- controller 只在 10 Hz 状态轮询读到该 `P` 的 `ACCEPTED/EXECUTING` 后发送队列中
  的最新同模式目标。若确认晚一轮，中间目标会被合并，下一段可能变成约
  `4 deg/50 raw` 的位移，但执行时长仍为 `90 ms`；因此调度和 I2C 抖动会表现为
  “停一下再跳快”，而不是均匀降速。
- STM32 `HostServoSet()` 对舵机 1/2 直接发送 `MOVE_TIME_WRITE`，每个新目标都会
  清除并重建单次位置跟踪；该路径没有 `q_goal/q_cmd/q_velocity`、速度前馈或
  加速度限制。现有 25 Hz AR4 滚动伺服只覆盖 IK 舵机 6～3，因此没有修复夹爪和
  腕转的周期性重规划。
- 本次日志 `gripper contact detected; holding at feedback 0.376` 是夹爪闭合反馈
  连续三帧停滞后冻结当前位置的既有保护，会产生一次有意停止，但它只作用于夹爪，
  不能解释腕转同样卡顿。One Euro 也只发布 `/arm/state_filtered`，不参与 teleop、
  接触判断或舵机命令。
- 当前实机日志存在证据缺口：尝试读取 RDK 最新日志时，两条只读 SSH 查询持续无
  响应；随后两次带 5 秒上限的 SSH 均连接超时，三次 ping 为 100% 丢包。故当前
  不能排除 I2C/系统调度问题进一步放大卡顿，但这些附加故障不是上述固定短段节拍
  存在的前提。
- 2026-08-01：修复契约冻结为 I2C v3 新增 `U/G`：`U` 仅允许 1/2 号舵机并安装
  单通道移动参考，STM32 以 25 Hz 连续下发 40 ms 小段；`G` 平滑完成最后目标。
  旧 `P` 保持一次性语义，Home/探针不迁移；接触检测继续用 `H` 即时停止。当时为
  契约冻结阶段，RDK 仍离线，未部署或实机驱动。
- 2026-08-01：本地实现完成。STM32 新增 raw 空间单舵机移动参考、速度前馈、
  加速度/换向/限位约束、END 稳定完成、watchdog/deadline 制动以及写/反馈失效；
  ROS 新增夹爪/腕转各自的 STREAM/END 模式，controller 保证最新 `U` 先于 `G`，
  teleop 回中发一次平滑 END，只有夹爪接触继续使用即时 `H`。
- 离线证据：固件六组 host `-Werror` 测试、Cortex-M3 改动对象编译和 `cppcheck`
  通过；GNU 全工程链接为 `text=73788、data=1744、bss=5036 bytes`，生成
  `LeArm-v3-direct-servo.hex`，SHA-256
  `32380e4d639b234f93acb7b2dd82ce916dca727411b02b104bde730bb137d1e8`。
  ROS `colcon test` 为 `115 tests, 0 failures, 1 skipped`。这些只证明离线契约与
  调度算法，尚不能替代真实总线时延、夹爪接触和连续 60 秒运动验收。
- 2026-08-01：将 `U/G` 涉及的消息、controller、teleop、配置、测试和控制契约共
  9 个文件同步到 RDK `/home/sunrise/Armbot`，checksum 复核与本地一致；远端
  `scripts/build-rdk-ros2.sh` 对 `action_interfaces/action_pkg` 构建返回 `rc=0`，
  安装接口已包含模式 8～11，包内回归为 `109 passed、1 skipped`。部署前备份为
  `/home/sunrise/Armbot-direct-servo-pre-20260801-005644.tar.gz`，SHA-256 为
  `2db83642d011df487be19c7fba7916c054f0e1255bb249593e577e4828c1d761`。同步和测试期间
  controller、teleop、filter、joy 均保持停止；本轮未刷写 STM32，不能开始实机运动。
- 2026-08-01 实机失败：启动日志显示 Home 的夹爪、腕转、笛卡尔阶段分别通过
  `wire_id=1～4`，最终笛卡尔 Home 在 `1785517356.672` 返回 `COMPLETED`；第一条
  XYZ `T` 在约 2.20 秒后即以 `ARM_NOT_READY` 拒绝，此后同类请求持续失败。
  `wire_id=52/53` 的直接舵机 STREAM/END 随后仍能 `EXECUTING/COMPLETED`，且完成后
  XYZ 继续失败，结合 `robot_arm_ready` 只在启动赋值，排除了初始化失败和直接舵机
  持续占用；实际失效条件为 `rolling_servo_ready=false`。
- 根因位于本次固件反馈接线：`JointFeedbackAttemptFailed()` 在任意舵机连续三次读取
  失败后，即使滚动伺服处于空闲，也调用 `rolling_servo_feedback_failed()`；后者只要
  已初始化便执行 `rolling_servo_invalidate(..., false)`。空闲路径不发布错误事件，且
  `sync_allowed=false` 使后续恢复的四路有效反馈无法重新初始化。因此 ROS 可重新显示
  `position_valid=true`，STM32 却永久拒绝 `T`，直到再次完成 Home。日志中稍后出现
  `servo6_base_raw=0` 后又恢复到约 `502`，与该瞬态读失败触发器一致，但首次失效的
  具体舵机 ID 当前未被空闲诊断上报。
- 同时确认一个放大问题：controller 的状态发布定时器先于状态轮询；下一个 10 Hz
  请求会把刚设置的 `STATE_ERROR` 覆盖成 `MOVING`，teleop 可能看不到错误边沿并继续
  发送，所以日志出现数百个 `ARM_NOT_READY`。END 告警只是所有 `T` 都未成功打开流
  后的次生结果。修复需让空闲反馈失败保留自动重同步能力、活动运动失败仍安全锁存，
  并保证 `ARM_NOT_READY` 错误边沿在接受下一请求前送达 teleop。
- 2026-08-01 已修复：空闲滚动伺服的反馈重试耗尽现在只重置关节快照并保留
  `sync_allowed`，下一轮完整 6～3 号有效反馈会重新初始化；活动运动中的同类失败
  仍执行失效、报告 `SERVO_FEEDBACK_FAILED` 并要求 Home。controller 对 STOPPING 和
  非恢复性 FAILED 立即发布一次 `ArmState`，避免下一条 10 Hz 请求覆盖错误边沿。
  两个失败回归均先复现旧行为、修改后通过；固件六组 host 测试、Cortex-M3
  `-Werror`、`cppcheck`、ROS 定向 `98 passed` 和完整 `115 tests、0 failures、
  1 skipped` 均通过。该版 `239506e...f5058f` HEX 随后已刷写并证明空闲自动重同步
  生效，但实机又暴露 P0 条目记录的 watchdog 安装/确认竞态；它已被上方当前 HEX
  取代。
- 2026-08-01：controller 修复和对应回归已同步到 RDK，三个文件 SHA-256 与本地
  一致；`scripts/build-rdk-ros2.sh` 重建两个 ROS 包成功，RDK 包内测试为
  `110 passed、1 skipped`。同步前没有控制进程运行；备份位于
  `/home/sunrise/Armbot-readiness-fix-pre-20260801-014544.tar.gz`，SHA-256 为
  `edfee6a8435de3d245a9fef1d649280447252b81d72a16a31b48fd8ba852be06`。
  固件根因修复只存在于新 HEX，未重新刷写前 XYZ 仍会复现旧问题。

### 影响

- Xbox 无法完成腕转示教，夹爪只能开闭，不能调整抓取物绕末端轴的朝向。
- Home 当前只依次打开夹爪并恢复 6～3 号舵机的笛卡尔姿态；腕部一旦旋转，现有
  Home 也不会主动把 2 号舵机恢复到零位。

### 修复与关闭标准

- 2026-07-31：新增仅控制 `joint_5_wrist_roll` 的独立 ROS 模式，复用已存在的 I2C v3 `P`
  单舵机帧，不启用通用 `MODE_JOINT`，也不修改 STM32 协议版本。
- 右摇杆左右默认控制腕转，范围限制为 `[-90, 90] deg`；按住 RB 时同一轴保留原
  笛卡尔 pitch 调节。其他 XYZ、夹爪、急停和组合键映射不得回归。
- Home 顺序调整为“夹爪打开 -> 腕转归零 -> 笛卡尔 Home”，任一阶段失败不得继续。
- 2026-07-31：controller 在刷新队列前立即发布匹配序号的 `COMPLETED`，防止 teleop
  丢失夹爪停止完成边沿；Home 在夹爪规范反馈连续 3 帧 `<=0.10` 后发送单夹爪
  `H` 停止，只有收到该停止命令的 `COMPLETED` 才继续腕转 Home。两个失败回归已先
  复现旧行为，修改后均通过；未修改 STM32 或 I2C v3 布局。
- 本地安装态 `colcon build` 通过，action_pkg 回归为 `89 passed、1 skipped`；RDK
  源码与本地 5 个部署文件 SHA-256 全部一致，安装配置含
  `home_gripper_open_tolerance: 0.10`，RDK 直接 pytest 同样为
  `89 passed、1 skipped`。部署后 controller、teleop 和 joy 保持停止，I2C 与手柄
  设备均无占用。部署前回滚包为
  `/home/sunrise/Armbot-home-lifecycle-pre-20260731-2010.tar.gz`，SHA-256 为
  `d7742e7b9b512e7fce50dc3e2f4ffb25280836fb7ab4fa0ba00647e20d7eb3a8`。
- 本地 ROS 接口构建通过；在允许 Fast DDS 本地通信的环境中，action_pkg 回归为
  `87 passed、1 skipped`。RDK 增量部署后 `action_interfaces/action_pkg` 重建通过，
  回归为 `88 tests、0 errors、0 failures、1 skipped`；安装接口包含
  `MODE_WRIST_ROLL=7`，安装配置包含 `20 deg/s`、`[-90,90] deg` 和腕转 Home 时长。
- 同步和构建完成后确认 RDK 没有运行 controller、teleop 或 joy，未自动触发机械臂
  运动。部署前目标文件回滚包为
  `/home/sunrise/Armbot-wrist-pre-20260731-1748.tar.gz`。
- 待用户低速实机确认 Home 能完成、A 可使能、两个摇杆和腕转均恢复响应后再归档
  关闭。

## P1：RDK Python 构建环境存在损坏包和 setuptools 版本冲突

状态：**已证实，待修复**

### 现象与证据

- 2026-07-31，RDK 标准 `colcon build` 在导入 `docutils.frontend` 时先报
  `bad marshal data`；重定向字节码缓存后进一步报
  `source code string cannot contain null bytes`。系统文件
  `/usr/lib/python3/dist-packages/docutils/frontend.py` 被 `file` 识别为 `data`，
  不是正常 Python 文本。
- RDK 的 `/usr/local` setuptools `80.9.0` 优先于系统自带的 `59.6.0`，与当前
  ROS2 colcon 组合分别报 `--editable`、`--uninstall` 参数不识别。仅在本次构建
  进程中优先使用系统 setuptools 后，action_pkg 才能完成构建。
- 把整个 `/usr/lib/python3/dist-packages` 提前会继续触发系统 NumPy 源文件含空字节，
  因此不能把全局 `PYTHONPATH` 调序作为长期修复。本次部署只在 `/tmp` 遮蔽可选
  docutils 并单独暴露 setuptools 59.6.0；正常 ROS 运行环境未修改。
- 2026-07-31 状态滤波节点部署时进一步确认，仅暴露系统 `setuptools` 包仍会与
  `/usr/local` 的 `pkg_resources` 80.9.0 元数据混用，并报缺少
  `setuptools.command.bdist_wheel`。在同一 `/tmp` 兼容目录同时暴露系统版
  `pkg_resources`、`_distutils_hack` 和 59.6.0 元数据后，`action_pkg` 构建通过；
  该处理仍只是构建进程级绕过，不是系统环境修复。

### 影响与关闭标准

- 当前 action_pkg 默认运行时导入正常，但后续 RDK Python/ROS 包重建和测试可能在
  与业务代码无关的位置失败，且临时构建路径不能作为长期部署契约。
- 需要校验存储和文件系统健康状况，重新安装损坏的 docutils/NumPy 包，并固定 ROS2
  构建使用的 setuptools 兼容版本。无临时 `PYTHONPATH` 时标准 `colcon build/test`
  能实际收集并通过 action_pkg 全套测试后方可关闭。

## P1：机械臂舵机 2～6 没有有效反馈

状态：**2026-07-17 复发，已重新打开，根因待确认**

### 2026-07-17 重新打开证据

- RDK 当前只运行一套 `arm_controller_node/arm_teleop_node/joy_node`，排除双控制栈竞争。
- `/arm/state` 为 `STATE_ERROR/PHASE_FAILED/error_code=0x0019`，提示固件重启后需清错；
  controller 日志同时证明固件能返回合法 I2C v3 生命周期，当前不是 v2/v3 协议不匹配。
- 多次命令均先进入 `EXECUTING`，随后 STM32 返回
  `FAILED/SERVO_FEEDBACK_FAILED (error=6)`；ROS 记录六路 raw 多次全部为 `0`，
  `position_valid=false`。因此 Home 无法完成的直接原因是运动/回零阶段没有可信舵机反馈。
- 启动早期出现 24 次 `Errno 121`，之后 I2C 恢复并能连续读取 v3 状态；该现象说明
  RDK↔STM32 链路也有启动期不稳定，但不能解释恢复通信后 USART2 舵机反馈仍持续失败。
- Windows 桌面工程与当前工作树的 `main.c`、`stm32f1xx_it.c`、`serial_servo.h`、
  v3 协议头和 Keil 工程哈希一致；但本次 Keil 链接因 32 KiB 授权限制失败，目录中
  没有新 AXF/HEX，因此当前板上 v3 镜像的确切源码快照仍无法由构建产物证明。
- 在确认当前固件镜像和 USART2 原始回包前，停止反复 Reset/Home；安全门保持
  teleop disabled，不能用清错掩盖持续反馈失败。


### 现象

- 修复前舵机 1 槽位返回约 `raw 498`，舵机 2～6 槽位返回 `0`。
- `/arm/state.position_valid=false`，因此没有可信的 `/joint_states`。

### 已找到的软件根因

- 旧版 `Core/Src/main.c` 在同一个 `for` 循环中连续调用异步
  `serial_servo_read_position()`。ID 1 只启动请求，ID 2～6 因控制器忙而被丢弃；
  下一轮还可能把 ID 2 的响应写入 ID 1 槽位。
- 因此原先 2～6 的 `0` 是未更新的缓冲区初始值，不能作为舵机硬件无响应的证据；
  `raw 498` 也可能实际来自复位目标约为 `500` 的 ID 2。
- 2026-07-14：`armbot-stm32` 已改为单事务逐 ID 轮询，校验响应 ID 和命令，
  20 ms 无响应时标记当前 ID 无效并继续下一个。静态检查通过。
- 2026-07-14 17:25：v2 固件状态显示舵机 1～6 raw 全部为 `0`。Home 命令已通过
  协议解析和 IK，并进入 `EXECUTING`，但约 102 ms 后返回
  `FAILED/SERVO_FEEDBACK_FAILED`。这排除了本次错误是 `BAD_COMMAND`、
  `NO_IK_SOLUTION` 或 `SERVO_WRITE_FAILED`。
- 当前错误包没有携带失败舵机 ID；固件只会在被当前运动跟踪的舵机读取失败时设置
  该错误，因此能确认至少一个 Home 目标舵机反馈失败，但无法仅凭现有日志确定
  具体 ID。
- 2026-07-14 23:09 启动的单控制栈中，controller 文件日志记录了
  `wire_id=341/366/397/418/504/647/856/857/1134/1135/1206/1207`，用户补充的终端
  日志又记录了 `wire_id=1210`，共至少 13 次返回 `SERVO_FEEDBACK_FAILED`。多数失败
  前后其他命令均能读到有效位置；
  `wire_id=341` 失败时舵机 1、6 仍为约 `raw 511/527`，`wire_id=366` 失败时舵机 6
  槽位为 `0`，证明至少存在可恢复的运动期回读失败，但现有状态无法判断每次是否为
  同一 ID。该证据把“偶发一次”更新为稳定可复现问题。
- `wire_id=1208/1209/1211/1212` 的 Home 均完成，`wire_id=1210` 则在一次
  `Errno 121` 后先进入 `EXECUTING`、约 200 ms 后返回反馈失败；复位后的下一次
  Home 再次成功。`enable rejected: target is not synchronized` 出现在失败后或稳定
  样本尚未收齐时，是遥控安全门的预期结果，不是 ERROR 的触发原因。
- 2026-07-14 Git 历史复核：最初导入版 `5a6123b` 每 100 ms 在同一循环中依次
  调用 1～6 号异步读取，且完成时不校验回包 ID。第一轮只会真正发出 ID 1 请求；
  后续循环中，ID 1 槽位可能消费上一轮 ID 2 的回包。旧快照约 `raw 498` 与 ID 2
  复位值 `500` 接近，而 ID 1 复位值是 `226`，因此该快照更像错配回包，不能证明
  ID 1 当时稳定可读。
- `8957b23` 把轮询改为单一在途 ID，并增加响应 ID、命令和长度校验，20 ms 无
  有效回包就把对应槽位写为 `0`；`c963c92` 又将 v2 状态缓冲从全零初始化，并在
  运动期间也轮询反馈。两次修改都会消除旧值或错配值，但 `8957b23..c963c92`
  没有修改 USART2 接收中断本身。因此当前“1～6 全为 0”表示没有任何通过严格
  校验的回包，不足以证明是 v2 单独破坏了 ID 1 的硬件通信。

### 2026-07-14 防护性修改（根因尚未由实机证实）

- `USART2_IRQHandler()` 过去只检查 `TXE/TC/RXNE` 硬件标志，没有同时检查对应
  中断源是否启用。接收中断进入时，遗留的发送标志可能错误推进发送状态机；这是
  真实代码缺陷，但当前日志没有证明它就是本次全零的触发原因。
- 过去没有按 STM32F1 要求用 `SR -> DR` 顺序处理 `FE/NE/ORE`，错误状态可能残留，
  使后续舵机响应继续读不到。现在错误会被清除并锁存给主循环，事务会安全终止。
- 单个 20 ms 回读窗口失败不再立即判定目标舵机故障；同一 ID 最多尝试 3 次，仍
  失败才进入 `SERVO_FEEDBACK_FAILED`。如果总线始终没有 RX 字节，该重试不会解决
  问题，只会把确认失败的时间延后到约 60 ms。
- 最新日志证明上述一个“3 次读取均失败的批次”仍会把可恢复的瞬态失败直接升级为
  整机 ERROR。现在同一 ID 必须连续 3 个完整批次失败才会把该槽位置零并返回
  `SERVO_FEEDBACK_FAILED`；前两个失败批次保留最后一次有效位置，任意一次有效回读
  都会清零该 ID 的连续失败计数。持续断线仍会在第三个失败批次进入错误，未被降级
  为成功。
- 新增 `Tests/robot_arm_motion_static.c`：修改前可复现第一次失败就终止运动，修改后
  验证前两次失败保持 `ACTIVE`、第三次失败进入 `FEEDBACK_FAILED`，并验证一次有效
  反馈会重置计数、到达目标仍能完成运动。主机测试已通过。
- ROS 控制器在固件返回 `error=6` 时，会把 raw 不在 `(0, 1000]` 的舵机 ID 和全部
  6 路 raw 值附加到错误日志。对应回归测试、源码 `pytest`（61 passed，1 skipped）、
  `colcon build` 和 `colcon test`（66 tests，0 failures）均通过；不改变 32 字节
  I2C 协议。
- STM32 修改已通过 Cortex-M3 目标对象编译和所选文件 `cppcheck`；尚未进行本次
  修改后的 Keil 全量链接、烧录或机械臂实机验证。2026-07-15 从 WSL 调用 Windows
  UV4 的批量构建未产生构建日志且进程未退出，已终止该次构建；不能把它计为链接
  通过，仍需从 Windows Keil 工程完成一次可审计的全量构建。
- 新增 `Tests/serial_servo_rx_static.c`，覆盖官方位置响应帧
  `55 55 ID 05 1C posL posH checksum` 及坏帧拒绝。协议测试、改动对象
  Cortex-M3 `-Werror` 编译、Keil 工程清单中 94 个 GCC 兼容 C 对象编译和
  `cppcheck` 均通过；是否影响当前故障仍待烧录验证。`ultrasound.c` 因仓库已有的 GCC
  静态 VLA 不兼容而按既有构建边界排除，最终链接仍需 Keil 工程。

### 2026-07-14 独立诊断结论

- 第二版诊断固件对 ID 1～6 连续多轮均记录
  `tx=OK/rx=OK/n=9/uart=0x00/parse=OK/skip=1`，排除了请求未发出、UART 无有效
  回包、舵机 ID 不存在、帧校验失败和 USART2 PE/FE/NE/ORE 错误。
- 每次收到的完整 9 字节均为
  `00 55 55 ID 05 1C posL posH checksum`；位置分别稳定在 ID 1
  约 228、ID 2 约 497～498、ID 3 约 174～175、ID 4 约 130、ID 5 约 412、ID 6
  约 497～499，与机械臂当前姿态及历史复位值一致。ID 1～6 的 checksum 分别随
  位置正确变化，例如 `F9/E9/2C/58/3C/E5`，解析器已逐帧验证通过。
- `serial_servo_rx_handler()` 在等待第一帧头时遇到任意非 `0x55` 字节，会立即把
  整个事务设为 `SERIAL_SERVO_READ_DATA_ERROR`，不会跳过噪声继续搜索后续
  `55 55`。因此稳定出现的前导 `0x00` 会让生产固件在真正响应帧到达前就判定失败，
  这是当前 1～6 全部反馈为零的直接软件根因；增加超时或重复相同事务不会消除它。
- 首版诊断一次只读取 8 字节，因此前导 `0x00` 占用一个位置后，真正响应帧末尾的
  checksum 成为第 9 字节而未被首轮日志保留。第二版已扩为 9 字节，从 `55 55`
  重同步并增加 `skip` 字段；镜像通过无警告目标链接、段权限与禁止运动符号审计，
  CubeProgrammer 写后校验成功，且完整 checksum 已由实机日志验证。
- 舵机电源关闭时，每个 ID 的请求仍稳定得到
  `rx=TIMEOUT/n=1/uart=0x00/bytes=00`；打开舵机电源后立即变为完整的
  `00 + 8 字节有效响应`。这证明 `0x00` 产生在板端发送完成并切换到接收的路径，
  不是舵机协议内容；具体是总线方向切换电气瞬态还是 USART 数据寄存器残留，仍可
  单独区分，但不影响生产解析器必须容忍帧头前噪声的修复结论。
- 2026-07-14：已新增独立只读诊断固件 `Diagnostics/servo_feedback`，仅向 ID 1～6
  发送 `SERVO_POS_READ (0x1C)`，通过 USART1 输出收发状态、实际字节数、UART
  PE/FE/NE/ORE、解析阶段和原始回包。Cortex-M3 `-Werror` 链接、ELF 向量表与
  RX/RW 段权限检查、禁止运动符号审计和 `cppcheck` 均通过；镜像已刷入并由
  CubeProgrammer 写后校验，以上原始日志已完成实机采集。
- 用户确认当前硬件似乎只引出了串口，没有可直接连接的 ST-Link。源码核对显示
  USART2（PA2/PA3，115200）属于舵机总线，不能输出诊断日志；USART1
  （PA9/PA10，9600，TX/RX）已初始化且当前没有调用 `app_init()`，可作为串口诊断
  候选。必须先确认板上外接串口确实对应 USART1，再写入日志，避免占用错误端口。
- 用户插入串口后，Windows 已识别 `USB-SERIAL CH340 (COM5)`，硬件 ID 为
  `VID_1A86&PID_7523`，设备状态正常。WSL 未生成 `/dev/ttyUSB*` 或
  `/dev/ttyACM*`；`COM5` 的候选映射 `/dev/ttyS4` 当前因用户不在 `dialout` 组而
  无法访问。现有固件也没有调用 USART1 的 `app_init()` 或输出诊断日志，因此尚未
  打开串口，避免下载口 DTR/RTS 可能触发复位且无有效数据可读。
- 用户关闭串口程序后通过 `/dev/ttyS4` 仍遇到 I/O error。Windows 原生
  `System.IO.Ports.SerialPort` 已在禁用 DTR/RTS 的条件下成功打开并关闭 COM5，
  证明 CH340、Windows 驱动和 COM5 本身可用；问题位于 WSL2 的旧 `ttyS4` 映射。
  后续确认 Windows 实际已安装 `usbipd-win 5.3.0`，只是旧终端 PATH 未刷新。
- CH340 实际 BUSID 是 `5-4`，不是最初尝试的 `6-4`；设备已完成 USBIP
  `bind/attach`，WSL 识别为 `1a86:7523 QinHeng CH340` 并生成
  `/dev/ttyUSB0`。`stty` 已确认 9600/8N1 可访问，STM32CubeProgrammer 2.22.0
  也已列出该端口。旧 `/dev/ttyS4` 路径不再使用。
- 2026-07-14 18:36：用户按 BOOT/RESET 流程操作后，CubeProgrammer 能以
  115200/8E1 成功打开 `/dev/ttyUSB0`，但等待 STM32 ROM Bootloader ACK 超时；
  该次未读取、擦除或写入 Flash，也未生成备份文件。18:39 保持 BOOT0 有效并重新
  上电后，CH340 因 USB 重新枚举从 BUSID `5-4` 变为 `7-4`，重新附加 USBIP 后
  Bootloader 握手成功；CubeProgrammer 识别到 Chip ID `0x410`、128 KiB Flash，
  已只读备份完整 `0x08000000..0x0801FFFF` 到
  `armbot-stm32/backups/before-servo-diag-20260714-183949.bin`。文件大小为
  131072 字节，SHA-256 为
  `9219dc193376bdf251a54e7da728f5e9040f80e42eae238b98010433a2c93433`；向量表中的
  初始栈地址 `0x20001068` 和复位入口 `0x08000101` 均落在有效地址范围。该结果
  确认当前 CH340 接口可以访问 USART1 ROM Bootloader，先前超时不是固定接线故障。

### 剩余工作

- 2026-07-14：生产 `serial_servo_rx_handler()` 已改为在现有 20 ms 有界事务内搜索
  `55 55` 帧头；帧头前 `0x00` 或其他噪声不再终止事务，进入帧后仍严格检查长度
  和 checksum。静态测试已覆盖实机字节流 `00 + 完整位置帧` 解析成功、非法长度
  和错误 checksum 拒绝；主机测试、`cppcheck`、Cortex-M3 测试对象及相关生产对象
  `-Werror` 编译均通过。
- 2026-07-14：首次 Keil ARMCC 5.06 全量构建在 `serial_servo.h` 与
  `ps2_porting.h` 的匿名 `struct/union` 处产生 14 个 `#3092/#3093` 错误；已在
  两个头文件中仅针对 `__CC_ARM` 启用 `#pragma anon_unions`，避免全局切换
  `--gnu`。待 Keil 重新全量构建确认链接与 HEX 生成。
- 重新执行 Keil 全量构建并烧录生产固件；确认错误日志能直接给出失效舵机 ID 和
  6 路 raw 值。如果仍显示 `invalid_servo_ids=unknown`，再增加板端失败 ID/尝试计数
  的临时串口诊断。
- 实机依次执行 Reset、Home 和低速逐轴运动；Home 至少重复 10 次，并在 10 Hz 连续
  控制下运行 60 秒，确认单个可恢复批次不再锁存 ERROR。
- 通过断开一个舵机反馈或等价故障注入，确认连续 3 个失败批次仍会进入
  `SERVO_FEEDBACK_FAILED`，不能只验证“正常时不报错”。
- 如需消除而非仅容忍前导零，再用方向切换延时或读取寄存器的 A/B 实验区分电气
  瞬态与 USART 数据寄存器残留；这不是恢复反馈的前置条件。

### 关闭标准

- 静止状态下连续读取舵机 1～6，所有 raw 值均为有限且位于 `(0, 1000]`。
- `/arm/state.position_valid` 稳定为 `true`。
- Home 重复 10 次、10 Hz 连续控制 60 秒，不因单个瞬态反馈失败进入 ERROR。
- 持续反馈故障在连续 3 个失败批次后仍进入 ERROR，日志能指出无效舵机 ID。
- 断电重启后仍能重复得到相同结果。


### 历史用户验收（2026-07-15，现已因复发失效）

- 2026-07-15：用户明确确认舵机反馈问题已完成实机验收，同意关闭本条。
- 关闭日期：2026-07-15

## P0：运动完成状态可能误报

状态：**v2 实机已确认反馈失败会返回 FAILED；待完成成功运动、超时和断线故障注入验收，仍阻塞验收**

### 现象

机械臂没有到达目标，ROS 仍可能收到 `ARM_DONE` 并进入
`STATE_SUCCEEDED`。

### 证据

- 2026-07-14：STM32 已改为非阻塞轮询目标舵机反馈；20 ms 无有效响应时进入
  `FAILED/SERVO_FEEDBACK_FAILED`。
- 30 秒运动超时现在进入 `FAILED/MOTION_TIMEOUT`，不再生成完成状态。
- 串口写入会返回结果；排队写入、启动帧或发送完成超时会进入
  `FAILED/SERVO_WRITE_FAILED`。
- ROS 只把匹配 `wire_command_id` 的 `COMPLETED` 视为成功；固件失败和运动超时
  映射为当前命令错误。相关 ROS 测试已通过。
- 2026-07-14 17:25：Home `wire_id=1` 先进入 `EXECUTING`，随后固件明确返回
  `FAILED/SERVO_FEEDBACK_FAILED`；ROS 保持 `STATE_ERROR`，没有误报
  `STATE_SUCCEEDED`。
- STM32 相关源码已通过主机 GCC 语法检查、`cppcheck`、协议静态断言及
  `arm-none-eabi-gcc` Cortex-M3 目标对象编译；尚未链接、烧录和故障注入。

### 影响

- Home、摇杆运动和采集数据都可能记录不存在的成功状态。
- 遥控节点会基于错误姿态继续积分绝对坐标，存在后续跳变风险。

### 关闭标准

- 任一目标舵机读取失败时报告明确错误，绝不生成 `ARM_DONE`。
- 30 秒超时进入错误状态，而不是成功状态。
- 断开任一机械臂舵机进行故障注入时，ROS 必须保持禁用且不能同步 Home。

## P0：Home 在无有效位置反馈时仍会同步坐标

状态：**无反馈 Home 的实机安全门已验证；待反馈恢复后验证成功 Home 和连续 3 个稳定样本，仍阻塞验收**

### 现象

Home 后机械臂实际 `y` 没有归零，但遥控节点仍可能记录
“home completed; target synchronized”。

### 已修改

- Home 现在必须先收到匹配命令的完成状态，再同时满足
  `position_valid=true`、关节接近 Home，并连续稳定 3 个样本，才设置
  `_synced=true`。
- `position_valid=false` 时禁止启用 Xbox 遥控；显式 Home 仍允许发出，以便从
  无反馈状态尝试恢复。
- 自动测试已覆盖无反馈、位置超差和稳定样本不足三种拒绝路径。
- 2026-07-14 17:25：Home 因目标舵机反馈失败进入 `STATE_ERROR`，当前
  `position_valid=false` 且 `/arm/teleop_enabled=false`，没有发生目标同步。
- 2026-07-14 19:56 后：生产解析器修复刷入后，连续 9 个状态样本均为
  `position_valid=true`；但首次 HOME `wire_id=1` 仍因运动期间目标舵机反馈失败
  进入 `STATE_ERROR/error_code=36`，没有发生目标同步。需在反馈已稳定后解除错误并
  再次 HOME，区分上电首轮瞬态和可重复的运动期总线问题。

### 影响

内部维护坐标可能显示 `y=0`，真实末端却仍有横向偏移。之后的增量命令会从错误
基准继续累加。

### 关闭标准

- Home 只有在可靠的命令完成确认和有效位置反馈同时满足时才能同步。
- `position_valid=false` 时必须保持遥控禁用，并明确提示 Home 尚未确认。
- 实机连续执行三次 Home 后，反馈关节均在规定容差内。

## P0：Xbox 遥控时机械臂异常抖动

状态：**300 ms watchdog 安装/确认时序已修复并生成 HEX，待刷写与实机验证**

### 现象

- 2026-07-14：用户报告使用 Xbox 控制机械臂时出现异常抖动。
- 2026-07-14：用户进一步确认，抖动只在推动摇杆时出现，表现为整臂抖动；
  不是松杆静止时的单关节抖动。
- 2026-07-14：用户确认除夹爪开闭外，四个笛卡尔摇杆方向都会引起整臂抖动；
  夹爪单舵机控制正常。
- 2026-07-14：用户确认 Home 的单次 2 秒运动几乎不抖。Home 与 Xbox 笛卡尔
  控制使用相同 IK 和舵机 3～6，主要差异是 Home 只下发一次长轨迹，而 Xbox
  每 100 ms 下发并覆盖一次 120 ms 短轨迹。

### 已确认的软件时序

- `arm_teleop_node.py` 每 100 ms 积分并发送一个新的绝对坐标目标。
- 修改前 `teleop_config.yaml` 中每条遥控命令的执行时长为 120 ms，因此持续推动摇杆时，
  下一条命令会在上一段轨迹理论结束前约 20 ms 到达。
- 修改前 `arm_controller_node.py` 不等待上一条运动完成就转发新命令；STM32 每条命令都会
  重新执行 IK，并依次覆盖舵机 6～3 的目标轨迹。
- 摇杆已有 `0.12` 死区，但没有滞回或低通滤波。死区内漂移理论上不会发命令；
  输入在死区边缘反复穿越时仍可能产生间歇小目标。
- 2026-07-14 12:40 只读检查 RDK：`/arm/command` 只有
  `arm_teleop_node` 一个发布者，`/joy` 也只有 `joy_node` 一个发布者，排除当前
  存在多个控制节点同时抢发命令；遥控保持禁用。
- 同次检查的静止 Joy 样本中四个摇杆轴均为 `0`，两个扳机均为 `1`，没有越过
  当前死区。
- 控制器日志在 12:27:38～12:27:55 多段持续出现约 100 ms 间隔的
  `ARM_OK__`，与 10 Hz 连续命令一致；这些连续段内未出现 I2C 错误。
- 修改前 `robot_arm.c` 的 IK 路径每周期异步连发舵机 6～3；
  `serial_servo_set_position()` 返回 `void`，当 UART 状态不是
  `SERIAL_SERVO_WRITE_DATA_READY` 时会静默忽略本次调用，但上层仍无条件调用
  `robot_arm_track_target()`，可能把未发送的目标当成已发送。
- 四条命令之间只有 `HAL_Delay(1)`。当前 HAL 实现使其实际等待约 1～2 ms，
  115200 波特率下单个 10 字节控制帧理论发送时间约 0.87 ms，正常无竞争时应能
  完成但余量很小；代码没有发送结果、忙等待上限或丢帧计数，实机是否发生该丢帧
  尚无证据。
- 舵机协议已经定义 `MOVE_TIME_WAIT_WRITE` 与 `MOVE_START`，修改前驱动没有封装和
  使用，因此 IK 得到的四个关节目标不是作为一组同步启动。

现有现象和运行证据已排除静止摇杆漂移、当前多发布者竞争和对应时段 I2C 丢包
是主因。Home 对照还表明，供电、多舵机负载和单次 IK 本身不足以解释持续抖动。
历史版本还存在短轨迹重叠、关节不同步和静默丢帧；当前版本已消除这些必然缺陷，
但仍保留 10 Hz 离散短轨迹，并在每个周期重新执行不连续的 IK 搜索。当前证据见
下方实机复核。

### 2026-07-15 当前实机只读复核

- RDK 当前只运行一套 `joy_node`、`arm_teleop_node` 和 `arm_controller_node`；
  `/joy` 与 `/arm/command` 均为单发布者。RDK 源码、构建产物和本地均为
  `fix/arm-control-v2@2911791`，两个节点及 `teleop_config.yaml` 的 SHA-256 一致，
  排除本次由重复控制栈或 ROS 旧文件造成抖动。
- 实际节点参数为 `control_rate_hz=10.0`、`command_duration_sec=0.09`；静止 Joy
  四个摇杆轴最大漂移约为 `0.00066`，远小于 `deadzone=0.12`。本地 `action_pkg`
  回归为 72 tests、0 failures、0 errors、1 个既有 copyright skip，覆盖旧
  `10 Hz/120 ms` 重叠配置会在启动时被拒绝。
- 当前会话控制器日志在启动后、固件恢复通信前出现 33 次连续 I2C 读取失败；恢复
  后的运动阶段又出现 16 次孤立 `Errno 121`，每次下一周期恢复，未出现固件
  `FAILED` 或非零固件错误。这证明高频运动期间的 I2C 监听窗口竞争仍存在，但现有
  证据不支持把这些单次状态读取失败作为可见整臂抖动的直接原因。
- 第一段 y 方向连续运动的底座反馈在 `wire_id=194..291` 期间总体从 `raw 494` 单调降至
  `raw 387`，没有大幅反向跳变；该段 99 次状态采样中有 66 次已经
  `COMPLETED`、33 次仍为 `EXECUTING`，证明当前 90 ms 微轨迹多数会在下一个
  100 ms 控制周期前停止，再由下一条命令重新启动。满幅 y 输入在 Home 附近每周期
  只增加 `0.1 cm`，底座目标约变化 `1.6 raw`，还会由固件整数化为离散小步。这组
  “短轨迹完成/停止 -> 下周期重新 IK 和启动”的节拍会给每次 IK 目标变化叠加冲击；
  旧 120 ms 轨迹重叠并不是当前配置仍在发生的事实。
- 遥控禁用后的 23 个静止反馈样本中，各关节峰峰值最大约 `0.48 deg`，夹爪波动约
  `0.002`，排除静止反馈量化噪声是整臂明显抖动的主因。
- 用户已确认当前配套源码版本为 Armbot `2911791` 与 armbot-stm32 `6cd94ea`；但状态
  协议仍未携带 STM32 固件 commit/hash，因此这组仓库版本映射不能证明板上二进制
  已烧录 `6cd94ea`。这是完成根因 A/B 和 60 秒实机验收前的关键证据缺口。

### 2026-07-31 AR4 式伺服部署后只读复核

- RDK 当前只有一套 `joy_node/arm_teleop_node/arm_controller_node`，`/joy`、
  `/arm/command` 和 `/arm/state` 均只有一个发布者；当前状态为
  `STATE_SUCCEEDED/PHASE_COMPLETED/sequence_id=917`，`position_valid=true`、
  `error_code=0`。
- 本次会话 controller 日志覆盖约 116 秒运动，904 条固件状态中有 869 条
  `EXECUTING`、26 条 `COMPLETED`、9 条 `ACCEPTED`，没有 `STOPPING/FAILED`，也没有
  ROS `WARN/ERROR` 或非零固件错误。相比旧版 99 个样本中 66 次 `COMPLETED`，当前
  已不再在每个 100 ms 目标边界周期性完成并重新启动。
- controller 可见的 904 个底座反馈点中，最大相邻变化为 6 raw（约 `1.44 deg`），
  没有超过 10 raw 的跳变，超过 2 raw 的连续速度仅出现一次明显换向；该通道未显示
  大幅周期反向，但日志只携带底座和夹爪 raw，不能替代四个运动关节的同步轨迹。
- 遥控停止后的约 12 秒 `/arm/state` 实时采样保持同一序号和成功状态。五个关节反馈
  峰峰值分别约为 `0.48/0.96/0.72/0.48/0.24 deg`，夹爪规范反馈峰峰值约 `0.004`；
  量级符合 1～4 raw 的反馈量化/噪声，不能解释肉眼可见的整臂运动抖动。
- 当前 ROS 图没有任何图像话题，也没有 VLA recorder；只有机械臂、Joy 和状态话题。
  因此当前条件可用于小规模动作/本体状态试采和流程验证，但不能直接产出正式 VLA
  训练 episode。正式采集前仍需在 x/y/z/pitch/腕转的恒定输入运动中同步记录
  `/joy`、`/arm/command`、`/arm/state` 和相机时间戳，证明所有运动关节无周期反向、
  图像无明显机械振动并完成 action/observation 延迟标定。
- 2026-07-31：本地新增旁路 `arm_state_filter_node`，从原始 `/arm/state` 发布同类型
  `/arm/state_filtered`。五关节和夹爪使用 3 点因果中值加 `alpha=0.5` EMA；原始
  时间戳、命令序号、生命周期和错误字段不变。无效、非有限或 ERROR/ESTOP 输入会
  清空历史并把派生输出标为无效，controller、teleop 和安全逻辑仍只使用原始话题。
- 新滤波器 12 项算法/节点测试和完整安装态 action_pkg 回归通过，结果为
  `101 passed、1 skipped`。隔离 ROS domain 的端到端阶跃输出为
  `0 -> 0.5 -> 0.75 -> 0.875 -> 0.9375`，且保留输入时间戳。
- 2026-07-31：过滤节点、配置、两个 launch、`setup.py` 和测试已经同步到 RDK；
  7 个源码文件 SHA-256 与本地逐一一致，RDK `action_pkg` 重建通过，直接运行完整
  pytest 为 `101 passed、1 skipped`，安装态已列出 `action_pkg arm_state_filter`，
  过滤模块导入和 Xbox launch 解析通过，配置解析到源码中的
  `state_filter_config.yaml`。覆盖前备份为
  `/home/sunrise/Armbot-state-filter-pre-20260731-2046.tar.gz`，SHA-256 为
  `c2ef2f4a3511be0bd0680c09e4ce0cdaa6d901f6cf98c83592ebe6f326751831`。
  部署完成后控制栈保持停止，I2C 和手柄设备无占用；下次启动 Xbox launch 后才会
  出现 `/arm/state_filtered`，本次部署尚未替代低速实机和 VLA 采样验收。
- 2026-07-31：本地将旁路滤波的第二级从固定 `alpha=0.5` EMA 替换为 One Euro，
  保留前置 3 点因果中值。默认使用 `min_cutoff=1.0 Hz`、`beta=1.5`、导数截止
  `1.0 Hz`，并按输入消息时间戳计算 `dt`；时间不递增或间隔超过 `0.5 s` 时从
  当前样本重新初始化。尖峰、自适应响应、参数校验、reset、时间异常和节点安全状态
  共 16 项目标测试通过，`action_pkg` 完整回归为 `105 passed、1 skipped`，colcon
  构建通过。该版本尚未再次同步到 RDK，也未用真实 rosbag 量化静止降噪和运动延迟。
- 2026-07-31 23:29：One Euro 源码、配置、测试和两份控制契约，以及遥操
  `stream_watchdog_sec=0.30` 已同步到 RDK `/home/sunrise/Armbot`；9 个目标文件
  SHA-256 与本地逐一一致。覆盖前备份为
  `/home/sunrise/Armbot-one-euro-pre-20260731-232948.tar.gz`，SHA-256 为
  `9a27350dc7e2df981007996dd941ad0262c4d426798fcaa55545b2f740591b4a`。
  RDK 专用脚本重建 `action_interfaces/action_pkg` 通过，远端完整 pytest 为
  `105 passed、1 skipped`；安装态配置确认 One Euro 默认参数和 `0.30 s`
  watchdog。部署前后均未发现 controller、teleop、filter 或 joy 控制进程，本轮
  没有启动机械臂。仍需 rosbag 和低速实机量化后才能判断该滤波参数是否适合 VLA。
- 2026-07-31 21:07 左右，固件先对新目标 `wire_id=663` 报
  `ACCEPTED`，约 203 ms 后却对最后有效目标 `wire_id=662` 报
  `STOPPING/STREAM_TIMEOUT`，随后为 `FAILED/STREAM_TIMEOUT`。controller 因始终
  等不到 `663` 的匹配 `EXECUTING`，3 秒后又报 `0x0016` ACK timeout；下一条
  `wire_id=664` 因滚动伺服已失效而被 `ARM_NOT_READY` 拒绝。
- 这次超时不是同一时刻的直接 I2C timeout：RDK 内核最近一次 `lost arbitration`
  在约 23 秒前，没有与上述 203 ms 窗口对应的 `controller timed out`。源码路径
  已确认：`rolling_servo_submit()` 在收包时更新 `last_target_tick` 并启动增量 IK；
  controller 在匹配 `GOAL_INSTALLED/EXECUTING` 前不发下一条 `T`；若 IK 在
  `watchdog_ms=200` 内未完成，`rolling_servo_service()` 会取消待处理 IK并以旧的
  `active_command_id` 进入受控制动。该目标至少没有在 watchdog 内产生可见的
  `GOAL_INSTALLED`。
- 2026-07-31 约 21:38:16 的完整前序日志进一步证明问题不限于慢 IK：
  `wire_id=1802` 在 `5095.315` 报 `ACCEPTED`、在 `5095.421` 报
  `EXECUTING`，说明 IK 已成功安装；但下一条 `wire_id=1803` 在 `5095.517` 已被
  `ARM_NOT_READY` 拒绝，之后才读到旧 `wire_id=1802` 的
  `FAILED/STREAM_TIMEOUT`。RDK 内核在 21:22:53 后没有新的 I2C 错误，排除同时段
  I2C 直接触发。
- controller 的状态轮询周期和目标发布周期均为 100 ms，且只有读到匹配
  `EXECUTING` 才清除 pending 并发送队列中的下一条 `T`。因此一次
  `ACCEPTED -> EXECUTING` 已消耗约 106 ms，再叠加下一轮写入、STM32 主循环取包和
  调度抖动，就会撞上从前一条 T 收包开始计算的 200 ms watchdog。结合主循环在
  `rolling_servo_service()` 前后各处理一次命令，边界到达的 T 还可能恰好落在
  watchdog 检查之后，形成确定的时序竞态。
- 修复必须同时覆盖“候选 IK 慢”和“成功 IK 的 ACK/轮询总延迟接近 watchdog”两条
  路径，并补充真实 IK 最大耗时；单纯提高 ROS ACK timeout、移动平均或重新 Home
  只能恢复当次操作，不能消除该闭环缺陷。I2C 仲裁丢失仍是独立未关闭问题。
- 2026-07-31：已将 Xbox/RDK 和 STM32 的默认流 watchdog 同步调整为 `300 ms`，
  保持协议范围 `100..1000 ms`、显式 END、急停和错误路径不变。新增固件边界回归
  已证明默认值在 `299 ms` 不超时、`300 ms` 进入 `STOPPING_TIMEOUT`；ROS 遥操
  回归确认默认 `T.duration_sec=0.30`。该修改尚未同步到 RDK、尚未重新生成并刷写
  固件，因此当前只能标记“已修改，待实机验证”，不能关闭抖动问题。
- 2026-08-01 新固件实机日志复现更精确的边界竞态：`wire_id=107/226` 已成功返回
  `EXECUTING`，但连续两次 RDK I2C 状态读取失败使 controller 约到 T 收包后的
  `300 ms` 才看见确认。固件此时从原收包时间触发 `STREAM_TIMEOUT` 并撤销同步，
  所以下一条 `wire_id=108/227` 先显示 `ARM_NOT_READY`，随后才读回旧命令的
  `STREAM_TIMEOUT`。该顺序证明本次首因不是舵机反馈失败；I2C `121` 是暴露竞态的
  延迟来源。
- 根因是 host 按契约等待匹配 `GOAL_INSTALLED/EXECUTING` 才发送下一条 T，而固件
  将增量 IK 和确认等待都计入同一个 watchdog。修复后，成功安装 IK 会把 watchdog
  起点更新到安装时刻；收包仍立即更新起点，安装失败不会续期。新增 290 ms 安装
  延迟回归先在旧实现于 300 ms 边界失败，修改后通过，并确认安装后 `299 ms`
  不停止、`300 ms` 仍进入 `STOPPING_TIMEOUT`。
- 当前修复已通过六组固件 host `-Werror` 测试、Cortex-M3 对象编译、`cppcheck`
  和 GNU 全工程链接。新 HEX 为 `text=73788、data=1744、bss=5036 bytes`，
  SHA-256 `32380e4d639b234f93acb7b2dd82ce916dca727411b02b104bde730bb137d1e8`；
  尚未刷写，不能把离线结果记为实机恢复。

### 2026-07-15 IK 连续性复核

- STM32 使用的 `LeArm.lib` 含调试符号。对库中 `set_pitch_range()` 和 `ikine()`
  反汇编并按相同公式离线重放后确认：`ikine()` 对肘关节固定采用负平方根分支，
  当前抖动不是两套肘上/肘下解来回切换；`set_pitch_range()` 从给定俯仰角开始，
  按 `1 deg` 步长搜索并立即返回第一个可行解。
- `robot_arm_coordinate_set()` 分别从俯仰范围两端搜索，只比较候选俯仰角与请求值
  的距离；选择过程没有上一帧关节角、舵机 raw 差值或连续性代价。因此即使末端
  坐标连续，小幅移动跨过可行域边界时，选中的搜索俯仰角和多个关节目标也会阶跃。
- 第二段只读实机采样中，`/arm/command` 的 `sequence_id=366..394` 保持
  `z=9.138483 cm`、`pitch=-54.460239 deg`，`x` 平滑减至 `10.0 cm`，`y` 从
  `4.693 cm` 单调增至 `7.334 cm`；ROS 输入没有反向跳变，固件状态始终
  `error_code=0`，该会话控制器日志也没有 `WARN/ERROR`。
- 用库的同一算法重放这组命令，选中的搜索俯仰角约每 5～6 条命令下降 `1 deg`：
  `-24.460 -> -25.460 -> -26.460 -> -27.460 -> -28.460 -> -29.460 deg`。
  每次边界处最大关节目标跳变约 `1.86～2.11 deg`；例如舵机 5/4/3 目标会从
  `(260,152,131)` 跳至 `(267,160,126)`，随后形成“缓慢变化 -> 同时跳档”的锯齿。
- 实际 `/arm/state` 与预测边界相符：单调遥控期间肩关节反馈多次出现约
  `1.7～1.9 deg` 的反向/正向阶跃，例如状态序列 `366 -> 367` 为
  `2.52584 -> 2.55935 rad`，`377 -> 378` 为 `2.55516 -> 2.52584 rad`。
  因此 IK 不连续已经是可见运动抖动的直接原因，不再只是待排除假设；90 ms 微轨迹
  的停走节拍会放大这些目标阶跃。
- 离线边界扫描还发现，Home 附近多数 `0.1 cm` 小步的最大关节变化低于约
  `2.5 deg`，但高 z 可行域边缘相邻 `0.01 cm` 目标可造成约 `17.7 deg` 关节跳变。
  该结果说明连续解修复还必须处理关节限位和奇异/不可达边界，不能只给摇杆加滤波。
- 固定远坐标通常只求解并执行一次，不会连续穿过上述 1° 搜索边界，也不会每
  100 ms 停止并重启，所以“固定远坐标不抖、遥控小增量抖”与该机制一致。

### 已修改

- STM32 新增独立的 `continuous_ik_solve()`：保留 `LeArm.lib` 的固定肘型
  `ikine()`，从请求 pitch 向上下按 `1 deg` 定位可行区间，再把无解/有解边界
  二分到 `0.05 deg`；不再直接使用 `set_pitch_range()` 返回的整度首个可行解。
- 选解会比较上一条成功笛卡尔命令的四个关节角。`1 deg + 120 deg/s * duration`
  以内的候选优先按 pitch 误差、最大单关节变化和总变化排序；全部超出时按用户
  选择继续执行最大关节变化最小的解，因此该阈值不是硬安全限位。
- 上一帧参考只在舵机 6～3 全部成功启动后更新；开机复位、3～6 号关节直控、
  3～6 号 STOP 或笛卡尔串口写失败会使参考失效，无解命令不会覆盖可信参考。
- 保持 10 Hz 控制，将每段轨迹从 120 ms 改为 90 ms；启动时强制校验轨迹时长
  不超过控制周期的 90%，控制器最小时长下调到 50 ms。
- 四个 IK 关节使用 `MOVE_TIME_WAIT_WRITE` 预写，再只向本批次的 6、5、4、3 号
  分别发送 `MOVE_START`；每一帧均检查 UART 是否实际开始并在限定时间完成。不能
  广播启动，因为 1、2 号可能保留 reset 或其他路径写入的历史 WAIT 目标。
- 等待当前硬件命令确认期间不继续灌入队列，只保留最新目标，确认后再发送。
- STOP 改为逐个舵机确认串口帧已发送；任一停止帧失败时返回
  `FAILED/SERVO_WRITE_FAILED`，不再静默报告完成。
- 未增加低通滤波；现有证据未显示静止摇杆漂移，先验证时序、同步启动和可靠发送
  三项直接修复。
- host 桩测试覆盖直接可解、移动边界细化、上一帧替代解、全部候选超出优先线仍
  选择最小跳变量解，以及完全不可达输入。`sequence_id=366..394` 对应轨迹的离线
  模型把最大相邻关节目标变化从 `2.121 deg` 降至 `0.486 deg`；代表性高 z 边界
  `(14,10,24.84)->(14,10,24.85) cm` 从 `20.027 deg` 降至 `3.427 deg`。
- 新模块、测试和 `robot_arm.c` 已通过 host `-Werror`、GNU Cortex-M3 `-Werror`
  对象编译与 `cppcheck`。Windows 临时副本使用 Keil ARMCC 5.06 全量链接为
  `0 Error(s), 5 Warning(s)`；5 项均来自未修改的既有文件，生成 HEX SHA-256 为
  `716f0e899279198cbcebc3eb2be2eadb7133bfa3b90bce5c4f2fb3c5fea92a35`。
  本轮没有刷写固件、部署 RDK 或发送控制命令。

### 影响

- 不可预测的关节运动有碰撞、夹伤和舵机冲击风险。
- 当前采样已把 IK 预测边界逐帧关联到肩关节反馈；状态消息仍不携带舵机 raw，
  其他关节的目标与反馈对应关系还缺少同精度证据。

### 下一步最小复现

1. 保持低速、清空工作空间，并准备 B 急停；暂不进行 VLA 采集。
2. 用户从当前固件源码自行编译并烧录，保存源码 commit 与实际 HEX SHA-256；启动后
   先确认 I2C v2、有效关节反馈和 Home 均正常，再启用遥控。
3. 以低速逐轴恒定推动 60 秒，随后恢复当前 `10 Hz/90 ms` 参数再次验收；同步记录
   `/arm/command`、`/arm/state` 和 B 急停结果。
4. 实机连续解验收后，再对比当前离散短轨迹与连续轨迹/速度控制，单独
   量化反复启停的剩余贡献。

### 关闭标准

- 明确复现条件并用输入、命令与关节反馈证明根因。
- 修复后静止不产生运动命令，单轴持续输入时目标和各关节轨迹连续、无反向跳变。
- 低速逐轴测试与 10 Hz 连续控制各运行至少 60 秒无异常抖动，B 急停仍有效。


### 用户验收

- 2026-07-15：用户明确确认 Xbox 遥控抖动已缓解并可验收，接受当前效果。
- 当前按“已缓解”关闭；若后续再次出现影响操作的异常抖动，应从本文件移回
  `DEBUG.md`，追加新证据后重新打开，不另建重复问题。
- 关闭日期：2026-07-15


### 2026-07-15 重新打开

- 用户在将 Xbox 笛卡尔遥操提高到原始速度的 `2×` 后报告当前再次出现抖动，
  并询问能否通过滤波解决。具体是起步/变向抖动、恒定推动时周期性抖动，还是
  接近工作空间边界时抖动，尚待代表性采样确认。
- 当前配置保持 `control_rate_hz=10.0`、`command_duration_sec=0.09`；满幅平移
  每周期目标增量由原始 `0.1 cm` 增至 `0.2 cm`，俯仰由 `1 deg` 增至 `2 deg`。
  提速提交 `e5f55d5` 只修改速度倍率和文档，没有改变控制频率、轨迹时长、IK
  或 STM32 下发路径。
- 既有实测中静止摇杆漂移最大约 `0.00066`，远低于 `0.12` 死区；因此在取得
  本轮 `/joy` 证据前，不能把复发直接归因于摇杆噪声，也不能把低通滤波当作
  已确认根因修复。

### 当前判断与最小验证

- 摇杆低通滤波可以减小起步、松杆和快速变向时的速度突变，也能抑制死区外的
  高频输入噪声；代价是操控延迟和松杆后的拖尾。
- 若恒定推动摇杆时仍持续周期性抖动，一阶低通稳定后仍会恢复 `2×` 速度，
  每 `100 ms` 的 `0.2 cm/2 deg` 离散增量不会消失，因此输入滤波不能解决
  `90 ms` 微轨迹完成后再启动下一段造成的停走节拍。
- 最小 A/B：在同一安全姿态单轴恒定推动 5 秒，分别使用 `1×` 与 `2×`，同步记录
  `/joy`、`/arm/command`、`/arm/state`。若 Joy 恒定而 `2×` 才抖，优先处理
  轨迹连续性或速度上限；只有 Joy/命令增量本身高频波动时才加入输入低通。
- 在 A/B 结论前暂停继续提速和直接加入滤波；测试时清空工作空间并准备 B 急停。

### 2026-07-16 Xbox 全周期离线仿真

- 按当前 `10 Hz`、平移 `2 cm/s`、pitch `20 deg/s`、固定 `90 ms` 轨迹时长和
  `0.12` 死区，使用实际 `integrate_target()` 与固件同连杆 IK 公式重放
  “中立使能 -> 摇杆渐推 -> 满幅保持 -> 松杆 -> 反向 -> pitch -> 对角移动”。
  ROS 笛卡尔目标在恒定输入期间单调，没有自行产生正反交替。
- Home 附近满幅单周期结果为：`x/y/z` 均移动 `0.2 cm`，最大关节目标变化分别约
  `1.008/0.764/1.690 deg`；pitch 单周期请求 `2 deg`，最大关节变化约
  `4.402～4.561 deg`。对应最大舵机目标变化约为 `4/3/7/19 raw`，因此 pitch
  的关节冲击理论上最明显，z 次之。
- 固定 `90 ms` 轨迹每 `100 ms` 下发一次，理想定时下也必然形成
  `90 ms 运动 + 10 ms 停止` 的 `10 Hz` 速度脉动。满幅单轴实际运动段速度约
  `2.222 cm/s`，间隙为 `0`，平均才是配置的 `2 cm/s`；松杆不发送 ARM STOP，
  最后一段轨迹会自然完成后突然停下。直接从 `+x` 切到 `-x` 时，笛卡尔段速度
  跳变量约为 `4.444 cm/s`，且没有加速度限制。
- 遥操用真实 timer `dt` 计算目标增量，但舵机时长固定为 `90 ms`。因此一次
  `80/100/120 ms` timer 间隔分别会形成约 `10 ms` 重叠、`10 ms` 停顿和
  `30 ms` 停顿；对应运动段速度约为 `1.778/2.222/2.667 cm/s`。这会把调度抖动
  直接转换成轨迹速度纹波。
- 控制器等待固件 ACK 时只保留最新目标。若一次 `100 ms` 状态轮询/I2C 读取错过，
  中间目标可能被替换，下一次实际下发的坐标间隔可由 `0.2 cm` 放大到 `0.4 cm`，
  但仍要求在 `90 ms` 内完成，运动段速度由 `2.222` 放大到 `4.444 cm/s`；
  pitch 同理可从 `2 deg` 放大到 `4 deg`。既有实机日志中的孤立 `Errno 121`
  因此更可能表现为偶发冲击，而不是持续反向抖动。
- 对角输入未做向量归一化：满幅 `x+y` 每周期移动约 `0.283 cm`，平均合速度
  `2.828 cm/s`，比单轴高 `41%`。死区边缘 `raw=0.13` 只产生约
  `0.00227 cm/周期`，通常先被舵机整数 raw 量化吞掉，累计到 1 raw 后才动作，
  会形成低速爬行时的粘滞/跳步。
- 当前根因优先级：固定短轨迹的周期性启停 > timer/ACK 延迟造成的目标间隔放大
  > 无加速度限制的启停和反向 > pitch/z 的 IK 关节放大 > 工作空间边界处剩余
  IK 跳变 > 死区边缘与 raw 量化。输入低通只覆盖其中“启停、变向、Joy 噪声”
  部分，不能消除恒定输入下的 `90/100 ms` 节拍。
- 验证：`test_teleop_mapping.py` 为 `8 passed`；STM32
  `Tests/ik_continuity_regression.py` 仍为记录轨迹最大相邻关节变化
  `2.121 -> 0.486 deg`、高 z 边界 `20.027 -> 3.427 deg`。完整 ROS 节点测试在
  当前受限环境因 Fast DDS 无法创建网络接口而未执行，不作为代码回归失败。

### 2026-07-17 实施方案：`q_goal` 滚动关节伺服

状态：**2026-07-31 AR4 式移动参考已实现并通过离线验证，待实机复测**

- 目标是取消“每个 100 ms 笛卡尔目标都独立启动并完成一条 90 ms 轨迹”的机制。
  RDK 继续以 `10 Hz` 更新笛卡尔目标；STM32 对新目标完成连续 IK 后只原子更新
  `q_goal`，再以独立的固定周期平滑生成 `q_cmd`。VLA 的 action 采样频率与底层
  舵机控制频率不要求相同。
- STM32 需要明确维护四组状态：
  `q_goal` 为最新可达 IK 关节目标，`q_cmd` 为本周期实际发送给舵机的平滑目标，
  `q_velocity` 为当前规划速度，`q_meas` 为舵机真实反馈。允许 `q_goal` 跳变，
  但必须通过关节速度、加速度和后续可选 jerk 限制保证 `q_cmd` 连续。
- 初始时间参数建议只作为实验起点：RDK `10 Hz/100 ms`；STM32 插值内环
  `25 Hz/40 ms`；每次向舵机发送四个 `WAIT_WRITE` 和四个逐 ID `MOVE_START`，
  舵机小段时长约 `40 ms`。现有 115200 波特率下八帧纯发送时间约 `5.6 ms`，
  仍需实测完整下发时间和总线占用后才能确定是否提高到 `50 Hz`。
- 名义时间线示例：

  ```text
  0 ms      RDK 发布 x=15.2 cm
  2 ms      STM32 收到命令，返回 ACCEPTED
  2..6 ms   连续 IK 计算；旧 q_goal 继续有效
  6 ms      原子安装新 q_goal，返回 EXECUTING
  40 ms     内环按速度/加速度限制生成 q_cmd_1
  40..46 ms 下发 q_cmd_1
  46..86 ms 舵机执行第一小段
  80 ms     内环提前生成并下发 q_cmd_2
  86..126 ms 舵机无固定停顿地执行第二小段
  100 ms    RDK 发布 x=15.4 cm，更新下一份 q_goal
  ```

  上述 IK `4 ms` 仅是说明流水线的示例，不是当前实测数据。
- 内环规划采用停止距离约束的梯形速度模型：根据 `q_goal-q_cmd` 决定目标速度，
  每周期限制 `q_velocity` 的变化，再积分得到 `q_cmd`；接近目标时依据
  `sqrt(2*a_max*distance)` 提前减速。快速换向必须先减速再反向，不能把速度直接
  从正最大值切到负最大值。
- IK 连续解的参考应优先使用当前 `q_cmd` 或可信 `q_meas`，不能继续只参考上一份
  尚未实际到达的 `q_goal`。`NO_IK_SOLUTION` 只拒绝本次候选，不覆盖最后有效
  `q_goal`，也不改变当前平滑轨迹。
- 当前固件的 `continuous_ik_solve()` 是同步阻塞调用。仅增加 `q_goal` 变量不能
  保证 40 ms 内环准时运行；必须先测量 IK 最坏耗时，再选择以下一种调度方式：
  将 IK 搜索拆成每轮有界候选数的增量状态机并优先服务伺服 tick，或使用独立低优先级
  任务运行 IK。禁止在高优先级定时器 ISR 中执行完整三角函数 IK 或阻塞式串口等待。
- 建议生命周期调整为：
  `ACCEPTED` 表示命令包已接收；`EXECUTING` 表示 IK 成功且新 `q_goal` 已安装；
  持续遥操时旧目标可被更新目标取代；操作者松杆后，只有最终 `q_goal` 满足
  `q_meas` 位置容差、速度接近零并连续稳定若干样本，才报告 `COMPLETED`。ROS
  遥操层也要从“仅在 `STATE_SUCCEEDED` 记录最后成功坐标”改为记录匹配
  `EXECUTING` 的最后已接受可达目标，否则后续 `NO_IK` 可能回退过远。
- 正常命令超时建议从约 `200 ms` 开始实验：超过窗口未收到新遥操目标时，停止推进
  `q_goal` 并按加速度限制减速，将最终停止位置固化为新目标；B 急停不走平滑过程，
  立即发送全局 `MOVE_STOP`，清零规划速度并要求重新同步。
- 实施前先建立基线：记录正常区和可达边界的 IK 最大/平均耗时、八帧串口下发耗时、
  伺服 tick 最大迟到时间及 I2C NACK 数。第一阶段只做单关节离线规划和 host 测试，
  第二阶段低速空载实机，最后才恢复当前 `2 cm/s、20 deg/s` 并进行连续 60 秒验收。
- 关闭本抖动问题至少要求：恒定输入时 `q_cmd/q_meas` 无周期性归零速度和反向跳变；
  启动、松杆、换向满足已配置加速度上限；IK 慢帧不阻塞内环；命令超时能平滑停止；
  急停仍能立即生效；`NO_IK` 不破坏最后有效目标和 ROS/STM32 同步。

### 2026-07-17 实施记录

- I2C 固定 32 字节协议已升级到 v3，新增 ROS 模式 5/6 与底层 `T/F`、
  `command_phase`、`STOPPING` 和错误 8～10；v2/v3 普通运动不匹配时拒绝，`S`
  仍保留跨版本广播 STOP。
- STM32 新增纯 `joint_servo`、增量 `continuous_ik_begin/step/result`、滚动伺服
  编排和可测试八帧发送器。`25 Hz` tick 使用 `60 deg/s、240 deg/s²`，每轮 IK
  最多一次 `ikine()`，跨度硬限为 `12 deg`；八帧全部成功后才提交规划状态。
- 主循环已按 STOP、servo tick、命令、反馈、IK 的优先级接线。普通流式发送不关闭
  I2C listen；200 ms 缺失 END 和完整周期 deadline miss 都进入停止距离刹车，停止
  后丢失同步；B 路径发送单个广播 `MOVE_STOP`。
- ROS controller 只有在匹配 `EXECUTING` 后才下发下一条 T，并分别保存“最新 T”
  和 END 标志；Teleop 在摇杆回中或 A 正常暂停时发送一次 END，Joy/ArmState 超时
  不发送 END。最后可达坐标改为按匹配 `command_phase=EXECUTING` 记录。
- 首轮实机参数曾降到 `0.5 cm/s、5 deg/s`；2026-07-31 用户要求把平移调整为
  `1.5 cm/s`，pitch 仍保持 `5 deg/s`，流 watchdog 仍为 `0.20 s`。恢复
  `2 cm/s、20 deg/s` 仍必须等待当前空载验收。
- DWT/USART1 只读诊断已加入，可统计 IK、八帧发送、tick 迟到、deadline miss、
  I2C 恢复和 NACK。实际数值必须刷写后采集，文档中的 `5.6 ms` 仍只是线速估算。
- 离线验证已通过：STM32 六组 host 回归、`cppcheck`、五个改动对象 Cortex-M3
  `-Werror` 编译；ROS `colcon test` 汇总为 `85 tests、0 errors、0 failures、
  1 skipped`。连续 IK 轨迹基线仍为 `2.121 -> 0.486 deg`，高 z 边界为
  `20.027 -> 3.427 deg`。
- Keil ARMCC 5.06 已编译全部工程源文件和新增模块；链接阶段报告代码段
  `37,312 bytes`，随后被 MDK-Lite 的 32 KiB 授权上限拦截。这不是源码编译错误，
  但不能记为全量链接通过。尚未完成无限制工具链链接、HEX 记录、刷写、总线占用和
  60 秒实机验收；因此本问题保持打开，不得按离线测试结果归档。
- 2026-07-17 用户在 Windows 桌面工程再次全量构建，ARMCC 编译完成后同样在链接
  阶段返回 `L6050U`；该副本报告代码段 `36,972 bytes`、`1 fatal error`。不同工程
  快照的数值略有差异，但都超过 MDK-Lite 32 KiB 上限，进一步确认阻塞项是链接器
  授权容量而不是新增源码的编译错误。

### 2026-07-31 实机复测失败与新增根因

- 用户实机确认滚动速度控制仍不可接受。只读检查确认 RDK 仅运行一套
  `joy_node/arm_teleop_node/arm_controller_node`，遥操已启用，位置反馈有效且
  `error_code=0`，排除了重复控制栈、无反馈和协议错误。
- controller 日志中的连续操作窗口 `wire_id=55..161` 共记录 107 个
  `EXECUTING`；相邻序号的中位间隔约 `100.0 ms`、95 分位约 `105.5 ms`，
  仅两次超过 `130 ms`，最后才由 END 进入 `COMPLETED`。这证明 v3 的 `T/F`
  流式生命周期已经实际运行，ROS 没有退回“每条命令等待完成”的旧路径。
- `joint_servo_prepare()` 仍然只根据当前静态 `q_goal-q_cmd` 和停止距离生成目标
  速度，没有相邻 IK 目标的速度估计、前馈、时间戳或一帧缓冲。对每个新目标都会
  提前制动到该点；以 `a_max=240 deg/s²`、上游周期 `100 ms` 计算，关节目标步长
  不超过约 `a*T²/4=0.6 deg` 时，连续时间三角速度轨迹可在下一份目标到来前完成并
  归零。首轮低速遥操每 100 ms 只有 `0.05 cm/0.5 deg` 的笛卡尔增量，多个关节很
  容易落入该范围，因此新实现会在内环重现周期性启停。
- 该机制说明“把离散 IK 结果写入滚动 `q_goal`”本身不足以得到恒速跟踪。正式修复
  需要为连续流增加相邻关节目标的速度/时间信息，或采用至少一帧缓冲的 C1 连续
  插值；只调低加速度只能改变出现启停的阈值，不能覆盖不同姿态和不同关节尺度。
  END、异常超时和换向仍需走有界减速，不能把中间目标直接外推成无约束运动。
- 当前日志只暴露 `q_meas`，尚未暴露每个 40 ms tick 的 `q_goal/q_cmd/q_velocity`；
  修复前需增加或读取已有 USART1 诊断，并用恒定单轴输入同步记录这些状态，验证
  `q_velocity` 是否在 100 ms 边界归零，同时记录八帧总线耗时和 tick 迟到。

### 2026-07-31 AR4 式移动参考修复实施记录

- STM32 的连续 `T` 路径已从“每个静态 `q_goal` 都按停止距离制动”改为移动关节
  参考。每次成功 IK 先求旧参考在安装时刻的 `q_ref`，再以相邻成功目标的实际接收
  间隔加 `20 ms` 缓冲生成新段；默认 100 ms 输入对应 120 ms 参考段。
- 40 ms tick 使用 `qdot_ff + 4*(q_ref-q_cmd)` 的速度前馈和位置校正，并继续限制
  `|v|<=60 deg/s`、`|delta_v|<=9.6 deg/s/tick`。反向前必须出现零速 tick；
  八个 UART 帧全部成功后才提交新 `q_cmd/q_velocity`。
- `F` 会立即切到最后 `q_goal` 的终点制动。END 后无解或跨度过大的 `T` 不会让
  机械臂重新加速，只有后续 `T` 成功安装才恢复移动跟踪。watchdog、deadline、
  写失败、反馈失败和急停的既有失效/重新 Home 契约未改。
- USART1 新增最近 32 个成功 tick 的压缩轨迹。空闲时发送 `Q` 可导出
  `tick/q_goal/q_ref/q_cmd/q_velocity/q_meas`；运动期间不响应，失败 tick 不记录。
- host `-Werror` 已覆盖 0.1/0.5 deg 小步各 5 秒、80/100/105/140 ms 输入抖动、
  时间回绕、移动参考替换、零速换向、END 后无解、超时、deadline 和八帧失败；
  连续 IK 基线仍为 `2.121 -> 0.486 deg`，高 z 边界为
  `20.027 -> 3.427 deg`。`cppcheck` 的 warning/performance/portability 检查通过。
- ROS 源码未改变。重新构建后，沙箱外 Fast DDS 回归为
  `85 tests、0 errors、0 failures、1 skipped`，覆盖 v3、流式队列、最终 T
  先于 END、Home 和夹爪旧行为。
- RDK 恢复启动后完成部署复核：远端 `ArmCommand/ArmState`、controller、teleop
  和两份配置的 SHA-256 与本地逐项一致，不需要覆盖内容；重新执行
  `scripts/build-rdk-ros2.sh` 成功，远端回归为
  `81 tests、0 errors、0 failures、1 skipped`。构建和测试后未运行
  controller、teleop 或 joy，遥控保持禁用。
- 使用不受 32 KiB 限制的 `arm-none-eabi-gcc 14.3.1` 对 Keil 工程全部 C 源完成
  Cortex-M3 编译、全量链接和 HEX 生成。ELF 为
  `49,696 bytes text + 100 bytes data + 4,500 bytes bss`；当前离线 HEX
  SHA-256 为
  `f55394afe6f47f6a0c6846ebed7f716dde7f23a6f542b8c58a1951bc5d107e76`。
  为通过标准 GCC，另将超声波三点滤波的局部数组长度改为等价编译期常量，
  不改变运行行为。
- 当前状态为“已修改，待实机验证”，不是问题关闭。仍需刷写最终镜像，以
  `1.5 cm/s、5 deg/s` 对 x/y/z/pitch、松杆和正反换向各连续验证至少 60 秒，
  读取 `D/Q` 诊断确认无 100 ms 周期归零、八帧不超过 10 ms、tick 迟到不超过
  5 ms，再验证断流、END、B 急停；通过后才能恢复 `2 cm/s、20 deg/s`。

## P1：Home 坐标与固件 raw 复位姿态尚未完成闭环标定

状态：**待确认**

当前 Home 使用 IK 坐标 `(15, 0, 2, -54.48 deg)`，开机复位则直接写入六个
舵机 raw 常量。虽然底座两条路径都应得到 `raw 500`，但完整 Home 的四个机械臂
关节是否与开机 raw 复位姿态完全一致，尚未在有效反馈下验证。

关闭标准：反馈恢复后，记录两条路径的舵机 2～6 raw 值并逐项比较，在容差内后
再冻结 Home 契约。

## P2：无效反馈仍以普通数值出现在 ArmState 中

状态：**已证实**

当 raw 为 `0` 时，`ArmState.joint_position` 仍会根据零值换算出看似有效的弧度值，
只是同时设置 `position_valid=false`。当前节点不会发布伪造 `/joint_states`，但忽略
`position_valid` 的下游仍可能误用这些数值。

关闭标准：接口文档明确要求检查 `position_valid`；同时评估无效时发布 `NaN`、保留
最后一次有效值或增加逐舵机有效标志，选择一种不易误用的契约。

## P1：高频命令与 I2C 读取在固件暂停监听窗口内冲突

状态：**RDK 快速读重试已部署并通过回归，待实机复测；底层仲裁丢失仍待排查**

### 实机证据

- 2026-08-01 02:21:47～02:22:13，RDK 内核在当前单控制栈会话中连续报告数十次
  `i2c_dw_handle_tx_abort: lost arbitration`，02:22:14 紧接着报
  `controller timed out`。ROS 最后一条正常状态为 `wire_id=270`，约 1.15 s 后
  `seq=271` 返回 `Errno 110`，随后固件依次报
  `STREAM_TIMEOUT/wire_id=271` 和 `ARM_NOT_READY/wire_id=272/273`。`/dev/i2c-5`
  当时只由唯一的 `arm_controller` 进程打开，排除第二套用户态控制栈竞争。
  这次是内核级长超时，不属于新增 5 ms 快速重试所处理的瞬态
  `EAGAIN/EREMOTEIO`；其耗时必然超过流 watchdog，因此当前安全停车行为符合契约。
- 2026-08-01 当前会话约 90 秒内 controller 累计 `44` 次 I2C 读取失败，启动阶段
  先连续出现 `26` 次 `Errno 121` 才恢复。内核在 `01:51:23` 连续两次记录
  `lost arbitration` 并随后记录 `controller timed out`，在 `01:51:44`、
  `01:51:47` 和 `01:52:29～37` 又多次记录 `lost arbitration`；与 ROS 同期的
  `Errno 121/110` 一致。这确认剩余问题位于 RDK DesignWare I2C/物理链路或 STM32
  从机总线状态，不是 One Euro、Joy 或 ROS 状态解释造成。
- 本轮已修复“两个短暂丢帧恰好吃满 300 ms 安装确认窗口”的固件竞态，但不会也
  不应掩盖持续超过 watchdog 的真实总线中断。新 HEX 刷写后若仍出现单次约 1 秒
  `controller timed out`，仍会安全停车；关闭该问题前必须检查 SDA/SCL 共地、线长、
  上拉与供电，并在单控制栈下完成内核日志和 60 秒运动联合复测。
- 2026-08-01 RDK controller 已增加同周期快速读重试：只对快速返回的
  `EAGAIN/EREMOTEIO` 最多尝试 3 次、间隔 5 ms；`ETIMEDOUT` 不重试。重试后恢复
  不增加连续失败计数，但单独累计并记录 `retry_total`；三次耗尽仍按原逻辑计为一次
  轮询失败，连续失败阈值和安全锁止不变。三个新回归覆盖瞬态恢复、耗尽和长超时，
  controller `64 passed`、action_pkg 完整回归 `113 passed、1 skipped`，本地两个
  ROS 包构建通过。同日已同步至 RDK `/home/sunrise/Armbot`，RDK 构建通过，
  包内回归同样为 `113 passed、1 skipped`；部署期间控制栈保持停止。覆盖前备份为
  `/home/sunrise/Armbot-i2c-read-retry-pre-20260801-021140.tar.gz`，SHA-256 为
  `156f5aee6555b528e9de5ee2fc39577a492830817dceaf54b9378f0af2e7df5b`。这些结果只证明
  ROS 快速重试已部署，不证明底层 `lost arbitration/controller timed out` 已消失；
  实机 60 秒运动验收仍待新 HEX 刷写后完成。
- 2026-07-31 21:22:43～21:22:53，RDK 内核连续 11 次记录
  `i2c_dw_handle_tx_abort: lost arbitration`。约两秒后，controller 连续下发的
  `wire_id=1405/1406/1407` 均收到 `FW_ERROR_ARM_NOT_READY`，teleop 随即因
  `STATE_ERROR` 禁用并要求 Home。该时间关系表明这三条 `error=3` 不是三次新故障，
  而是密集 I2C 仲裁失败使 `T` 更新中断、固件流 watchdog 先行失效后的连续拒绝；
  与 21:07 左右“新命令已 ACCEPTED 但慢 IK 超时”的独立软件缺陷不同。
- 2026-07-31 20:59:29，v3 流式遥操在 `wire_id=436` 后约 1.15 秒没有新的
  controller 状态日志；teleop 先因 0.5 秒无 `/arm/state` 丢失同步，随后 controller
  的 I2C read 返回 `Errno 110 Connection timed out`。恢复读取时 STM32 已按 200 ms
  watchdog 报 `lifecycle=FAILED/error=STREAM_TIMEOUT/wire_id=437`，下一条流式请求再报
  `ARM_NOT_READY/wire_id=438`，ROS 最终保持 `STATE_ERROR/error_code=0x0021`。
- 同一时窗 RDK 内核明确记录
  `i2c_dw_handle_tx_abort: lost arbitration` 和 `controller timed out`；20:58:28～31
  也出现同一组错误。`/dev/i2c-5` 只有 controller 的一个 fd，joy 只占用
  `/dev/input/event1`，排除第二个用户态 I2C 进程竞争。旁路 `arm_state_filter` 只订阅
  ROS 话题，不打开 I2C，不是本次总线仲裁错误来源。
- 当前 `SMBus.i2c_rdwr()` 在 ROS 单线程 executor 内同步执行；一次内核级约 1 秒
  timeout 会同时阻塞 10 Hz 状态发布和后续 `T` 下发。即使 STM32 随后恢复监听，
  阻塞时间也已远超 200 ms 流式 watchdog，因此现有自动恢复只能恢复通信，不能避免
  本次受控刹停和重新 Home。RDK 内核支持 `I2C_TIMEOUT=0x0702`（10 ms 单位），但
  缩短适配器超时的兼容性和实际效果尚未验证，不能直接作为已完成修复。
- 2026-07-14 23:24:53，单控制栈在 `sequence_id=885` 不变时再次开始持续
  `Errno 121`；失败计数一直增至 11914，23:44:49 板端恢复后立即读到
  `lifecycle=EXECUTING/error=0/wire_id=858` 和有效舵机反馈。该会话中 ROS 在第 5 次
  失败后才进入 `error_code=0x0001`，证明主机阈值修复有效；约 20 分钟不能自动
  恢复则再次验证 STM32 I2C error 路径仍会锁死。
- 2026-07-14 11:08:25，`sequence_id=1635` 和 `1637` 各出现一次
  `I2C read failed: [Errno 121] Remote I/O error`。
- 最后一次失败约 105 ms 后重新读取成功，最终状态自动恢复为
  `STATE_IDLE`。
- 故障发生时 Xbox 增量命令正以约 10 Hz 连续发送。
- 2026-07-14 17:25:34，在 Home 已于 17:25:22 返回反馈失败约 12 秒后，旧节点
  会话又连续出现至少 26 次 `Errno 121`，直到 17:25:38 被 Ctrl-C 终止；17:25:48
  新会话重新打开 I2C 后恢复读取。该事件晚于 Home 的首个失败状态，不能解释
  Home 为什么先返回 `SERVO_FEEDBACK_FAILED`，但表明总线/监听恢复仍不稳定。
- 2026-07-14 20:26:15，固件最后一次记录到有效状态
  `COMPLETED/wire_id=1274`；之后没有新命令，序号保持 `1341`。21:08:53 开始持续
  I2C 失败，到 21:58 的失败计数达到 2849，期间未恢复。
- 22:00:05 和 22:00:34 两次重启 ROS 均在 `sequence_id=0`、尚未发送运动命令时
  从第一轮读取开始失败，分别出现 `Errno 110` 和 `Errno 121`。因此这次持续失联
  不能由 10 Hz 命令与短暂 I2C listen 暂停单独解释；ROS 进程重启也不能恢复。
- RDK 源码、实际加载的 `ros2_ws/build/action_pkg` 与本地均为 `9ac3380`，控制器和
  遥控节点文件哈希一致。22:00 的进程包含夹爪命令排序修复，但总线没有成功通信，
  所以这两次日志不能用于验收夹爪修复。
- 用户随后执行 I2C reset，通信立即恢复；`/arm/state` 再次稳定发布有效位置且
  `error_code=0`。这排除了永久物理断线，但不能仅凭恢复结果区分是 RDK I2C 控制器
  状态卡住，还是 STM32 I2C listen 状态被 reset 间接恢复。

### 已确认机制

- `Core/Src/main.c:187-200` 在每条 `HOST_CMD_ARM` 执行期间主动关闭
  I2C listen，完成 IK 和舵机 UART 写入后才重新开启。
- ROS 控制器同时以 10 Hz 轮询状态；读操作落在上述窗口时，STM32
  不 ACK，Linux 返回 `Errno 121`。
- `arm_controller_node.py:262-270` 虽然定义连续失败阈值为 5，但第一次
  失败就调用 `_set_error()`；下一次成功读取又将它清回 `STATE_IDLE`，
  因此上层会看到短暂的 `STATE_ERROR`。
- STM32F1 HAL 的 `I2C_ITError()` 在 LISTEN 传输发生错误时先把
  `hi2c1.State` 保持为 `HAL_I2C_STATE_LISTEN`，然后调用项目的
  `HAL_I2C_ErrorCallback()`。项目回调立即调用 `I2C1_SlaveListenStart()`，但该
  函数看到状态仍含 LISTEN 后直接返回，实际没有重新开启监听。
- 项目错误回调返回后，HAL 才关闭 `EVT/BUF/ERR` 中断。只有 AF 分支随后把状态改为
  READY 并调用 `HAL_I2C_ListenCpltCallback()`；BERR、ARLO 或 OVR 单独发生时不会
  进入这条恢复路径，结果是 HAL 软件状态仍显示 LISTEN、硬件中断却已关闭。主循环
  之后反复调用相同的 `I2C1_SlaveListenStart()` 也只会直接返回。
- HAL 的 BERR 分支还会置位 `I2C_CR1_SWRST`，该错误路径没有清除此位；当前工程只有
  `HAL_I2C_Init()` 会完成 `SWRST` 置位再清零并重建寄存器。因此 ROS 进程重启不能
  恢复，而外设/MCU reset 后先返回 `READY/wire_id=0`，与本次实机现象完全一致。
- 修改前 `comm_cmd_error` 只保存“发生过错误”，没有锁存 `hi2c1.ErrorCode`、SR1
  或恢复次数。因此现有旧日志虽然足以确认永久失联的非恢复根因，仍无法反推出最初
  触发的是 BERR、OVR 还是其他错误标志。

### 已修改

- ROS 写入增加 3 次有界重试；单次读写失败只记录警告，连续达到 5 次才进入
  `STATE_ERROR`。
- 固件状态改用独立快照缓冲，防止 I2C 主机读取期间状态头被主循环改写。
- 固件在 UART 成组发送期间仍会短暂暂停 I2C listen，因此是否消除 10 Hz 压力下
  的 `Errno 121` 必须烧录后验证。
- I2C HAL 错误回调不再在 HAL 状态仍为 LISTEN 时尝试无效的原地重监听；回调只锁存
  `hi2c1.ErrorCode`、当时的 SR1 快照并设置延后恢复标志。
- 主循环收到恢复标志后会暂停监听、丢弃未完成命令，依次执行
  `HAL_I2C_DeInit()`、`HAL_I2C_Init()` 和 `HAL_I2C_EnableListen_IT()`，从而清除
  BERR 留下的 SWRST 并重建 ACK、中断和 DMA；成功恢复次数由
  `i2c_recovery_count` 记录，恢复失败则继续重试。
- 修改已通过 Cortex-M3 目标对象 `-Werror` 编译、所选文件 `cppcheck`，反汇编确认
  恢复函数实际调用上述 3 个 HAL API。尚未烧录，因此不能把自动恢复描述为实机
  已解决；首个真实错误是 BERR、OVR 或其他标志也仍待烧录后读取诊断量确认。

### 影响

- 单次可恢复竞争会被放大成整机错误，并可能让遥控丢失坐标同步。
- 高频命令期间的通信成功率不稳定，当前不适合进行 VLA 数据采集。

### 关闭标准

- 固件在舵机 UART 发送期间仍能 ACK 状态读取，或主机对这类短暂
  NACK 执行有界重试。
- 单次读取失败只记录诊断，连续达到阈值后才进入 `STATE_ERROR`。
- 10 Hz 持续遥控压力测试期间不再出现 `Errno 121`，并通过断线故障
  注入验证真实失联仍会在限定时间内进入错误。
- 固件发生 BERR/OVR 后应在不复位整板的情况下重新初始化 I2C1、恢复 ACK 和中断；
  同时通过诊断计数确认首个触发标志及自动恢复是否成功。

## P1：STM32 在运行期间发生过一次完整复位

状态：**首次复位原因待确认；ROS 重启识别已修改，待成对部署验证，仍阻塞验收**

### 实机证据

- 2026-07-14 11:05:11.57 至 11:05:12.66，RDK 连续 12 次读取地址
  `0x30` 失败，第 5 次后进入 `STATE_ERROR`。
- 恢复通信后依次读到 `I2C_RDY_` 和 `ARM_RDY_`，当时舵机反馈 raw
  全部为 `0`。
- `I2C_RDY_` 只在 `Core/Src/main.c:272-278` 的启动初始化路径设置，
  因此这不是普通总线瞬断，而是 STM32 重新执行了 `main()`。
- 2026-07-14 12:28:23 再次先出现一次 `Errno 121`，约 108 ms 后读到
  `I2C_RDY_`，这是当天第二次可识别的完整启动。用户回忆该时段应该进行过人为
  复位或重新上电，因此本次记录不作为异常复位和整臂抖动的证据；11:05 的首次
  复位是否人为触发仍待确认。

### 待确认原因

- 如果当时有人工按下复位，该日志与人工操作一致。
- 如果没有人工复位，优先检查 STM32/舵机供电压降、公共地、`NRST`
  干扰和板端电源接触。当前固件未启用 IWDG/WWDG，也未找到主动
  `NVIC_SystemReset()` 路径。

### 影响

- 运动期间 MCU 复位会丢失当前命令状态，并重新执行开机复位动作。
- I2C v2 没有独立 boot counter，但运行中重新出现 `READY/wire_command_id=0` 时，
  ROS 会锁存固件重启错误、清除当前命令并要求重新 Home；该路径已通过单元测试。

### 关闭标准

- 在启动时读取并记录 RCC 复位原因标志，区分上电、欠压、`NRST`
  和软件复位。
- 在舵机动作和 Xbox 10 Hz 命令压力下连续运行，不再出现非预期
  `I2C_RDY_ → ARM_RDY_`。
- 故意复位 STM32 时，ROS 必须禁用遥控并要求重新 Home 同步。

## P2：固件构建产物尚未与源码版本绑定

状态：**ROS 版本已绑定；固件版本绑定待完成**

- 2026-07-14 17:27：本地与 RDK 均为 `fix/arm-control-v2@642eeec`，RDK 工作区
  干净，ROS 源文件哈希一致。
- 本地 STM32 v2 基线为 `rewrite/v2@c963c92`，其上还有 2026-07-14 的 UART
  回读修复尚未提交、链接或烧录。RDK 日志证明板上运行的是 v2 生命周期协议，
  但固件状态包和构建产物没有携带 Git commit，仍不能证明板上二进制对应哪一版
  源码。当前开发机未检测到 ST-Link，不能在本轮完成烧录验证。

关闭标准：在固件构建/烧录记录中保存源码 commit，并通过状态版本或产物哈希确认
板上二进制；不要在版本未绑定前把当前状态标记为硬件验收完成。

## P2：GitHub Actions 导入顺序检查失败

状态：**已验证，待用户验收**

### 现象与根因

- 2026-07-14：分支 `fix/arm-control-v2@f428633` 的 GitHub Actions run
  `29345116075` 在 `ROS 2 build and tests` 失败。
- 失败仅为 `test_arm_controller.py:19:1 I101`：测试模块导入名称未满足 CI 中
  `flake8-import-order` 的字典序要求；控制逻辑测试本身均已通过。
- 本机插件版本未报告该错误，说明本地与 CI 的 import-order 检查存在版本差异。

### 已修改

- 按 CI 明确给出的顺序调整 `ERR_FW_MOTION_TIMEOUT/ERR_FW_NO_SOLVE` 与
  `FW_ERROR_MOTION_TIMEOUT/FW_ERROR_NO_IK_SOLUTION`，不改变运行逻辑。
- 2026-07-14：在 `action_pkg` 包目录运行独立 flake8 测试为 1 passed；随后
  `colcon test --packages-select action_pkg` 与 `colcon test-result --verbose`
  为 65 tests、0 failures、0 errors、1 skipped。
- 2026-07-14：修复提交 `cf6b35f` 已推送；GitHub Actions run `29345779232`
  的 `Software checks` 与 `ROS 2 build and tests` 均成功，远端 CI 已恢复。

### 关闭标准

- 本地 flake8 与全部 ROS 测试通过。
- 修复提交推送后，同一分支的新 GitHub Actions run 成功。
- 用户确认 CI 已恢复后归档。

## P1：RDK X5 橙色状态灯熄灭且有线网络不可达

状态：**网络不可达再次复现，根因待确认；当前无法读取控制栈与 ROS 日志**

### 现象与证据

- 2026-07-31：用户报告此前会亮/闪烁的 RDK X5 橙色状态灯不再亮。
- 用户随后确认绿色电源灯常亮，说明开发板已有 5V 输入；故障范围收敛为系统未
  正常启动或运行中死机，而不是整板无输入电源。
- 同次只读检查确认开发机专用有线口 `eth2=192.168.127.100/24` 为 UP，路由仍
  正确指向 RDK 的 `192.168.127.10`；连续三次 ping 无响应，ARP 邻居状态为
  `FAILED`。这排除了开发机网口未启用，表明 RDK 当前连二层邻居响应都没有。
- 用户重新处理后确认指示灯和 RDK 已恢复正常；随后 SSH 登录
  `192.168.127.10` 成功，远端重新构建和 ROS 回归均通过。恢复时没有取得 Debug
  串口或上次启动日志，因此目前只能确认“已恢复”，不能把 SD 卡、欠压、异常关机
  或内核卡死中的任一项写成已确认根因。
- 2026-07-31 One Euro 部署和远端测试成功后，用户启动实机操作并报告夹爪/腕转
  卡顿；本次只读日志查询先持续无响应，随后 SSH 端口连接两次在 5 秒上限后超时，
  对 `192.168.127.10` 的三次 ping 全部丢失。当前只能确认 RDK 再次网络不可达，
  尚未取得橙灯、供电、内核或 Debug 串口证据，不能判断是整机卡死、链路掉线还是
  其他原因。
- RDK X5 官方硬件文档说明：绿色灯表示 5V 供电正常，3.1.0 及以后系统的橙色
  状态灯闪烁表示系统运行正常；供电要求为独立 `5V/5A`，不应使用电脑 USB
  接口供电：
  <https://d-robotics.github.io/rdk_doc/Quick_start/hardware_introduction/rdk_x5/#电源接口>

### 当前判断与安全排查

- 若绿色电源灯也熄灭，优先检查 5V/5A 适配器、供电 Type-C 口、线缆和插座；
  这属于整板无 5V，不是 ROS 或本项目软件问题。
- 若绿色灯常亮而橙灯完全不闪，说明有输入电源但系统未正常运行，优先检查异常
  断电后的文件系统、Micro SD 接触/损坏、启动卡住或内核崩溃。
- 在断电状态下确认 Micro SD 插紧后，只做一次重新上电。仍无橙灯时连接 Debug
  Micro-USB，以 `115200 8N1、无流控` 保存完整启动串口日志；在取得日志前不重刷
  系统，也不反复断电。
- 机械臂保持遥控禁用。RDK 恢复后先确认只有一套控制栈、I2C v3 状态有效并重新
  Home，不得直接恢复运动。

### 关闭标准

- 明确绿色电源灯状态并取得一次完整上电结果；若仍失败，保存 Debug 串口启动日志。
- RDK 恢复橙灯闪烁，`192.168.127.10` 的 ARP/ping/SSH 均恢复。
- 检查上次关机、欠压或内核异常证据，确认根因后再恢复 ROS 控制栈。

## 建议调试顺序

1. 保持 Xbox 遥控禁用。
2. 用现有 Keil 工程完成固件链接，确保 ROS 与 STM32 v2 成对部署。
3. 断开任一目标舵机做故障注入，确认只出现匹配命令的 `FAILED`。
4. 连续 Home 3 次，确认舵机 2～6 反馈有效且关节在 5° 容差内后才能启用遥控。
5. 低速逐轴与四个笛卡尔方向分别连续运行至少 60 秒，验证 10 Hz/90 ms、同步
   启动、B 急停和运行期固件复位安全门。
6. 读取 RCC 复位标志并检查供电/`NRST`，找到首次运行期复位原因。
7. 对比开机 raw 复位与 IK Home，确认底座机械零位和 y 轴方向。
8. 全部验收后再恢复 VLA 数据采集，并将关闭条目移入 `DEBUG_CLOSED.md`。

## 已确认但不是当前问题

- 机械臂末端坐标单位为厘米（cm）。
- Home 命令中的 `y` 固定为 `0`，不是由维护坐标漂移生成。
- 夹爪语义为 `0=open`、`1=closed`。
- 当前遥控已禁用；修复上述 P0/P1 问题前应保持禁用。
