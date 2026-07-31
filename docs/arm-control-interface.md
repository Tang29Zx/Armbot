# 机械臂控制接口契约

状态：`v0.4 current`

适用分支：`feature/vla`

范围：机械臂与夹爪控制

## 1. 目标与边界

本契约只声明机械臂控制层的输入、输出、硬件边界和安全行为：

```text
ROS2 caller
  -> action_pkg arm controller
  -> I2C
  -> STM32 / servo controller
  -> arm state feedback
```

上层系统不在本契约范围内，只能依赖这里定义的 ROS2 接口，不应直接拼接 I2C 数据包。

## 2. 当前接口与 I2C 协议

当前控制器实现位于
`ros2_ws/src/action_pkg/action_pkg/arm_controller_node.py`。接口分为两层：

```text
上层 ROS2 类型化接口：供遥操、VLA 采集和其他节点使用
底层 I2C v3 二进制协议：仅供 arm_controller_node 与 STM32 通信
```

上层调用方不得直接构造 I2C 包；`arm_controller_node` 是目标 I2C 地址的唯一
所有者。

### 2.1 ROS2 输入

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/arm/command` | `action_interfaces/msg/ArmCommand` | 类型化机械臂、夹爪和停止命令 |
| `/arm/emergency_stop` | `std_msgs/msg/Bool` | 锁存急停请求 |
| `/command_topic` | `std_msgs/msg/String` | 已弃用的 `ARM`、`SERVO`、`STOP` 兼容入口 |

旧字符串入口支持：

```text
ARM x y z pitch min_pitch max_pitch time
SERVO id angle
STOP
```

`CAR` 命令属于底盘，不在本机械臂接口契约内。

### 2.2 ROS2 输出

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/arm/state` | `action_interfaces/msg/ArmState` | 类型化状态、命令序号、反馈和错误码 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仅在位置反馈有效时发布 |
| `/status_topic` | `std_msgs/msg/String` | 将 v3 生命周期映射成旧文本，仅供兼容和调试 |

`/command_topic` 和 `/status_topic` 不代表 STM32 仍运行 v1。兼容命令会先转换为
`ArmCommand`，再经过与类型化命令相同的校验和安全路径；兼容状态则由 ROS 根据
v3 二进制状态生成。

### 2.3 I2C v1 与 v3

共同硬件边界：

| 项目 | 值 |
| --- | --- |
| I2C bus | `5` |
| slave address | `0x30` |
| command packet | 固定 32 字节 |
| status packet | 固定 32 字节 |
| 字节序 | little-endian |

#### 2.3.1 v1：文本状态协议（历史版本）

v1 命令没有协议版本和命令 ID。32 字节命令依靠 `byte 0` 区分类型：

```text
ARM ('A'，历史固件也接受 0xA1):
  byte 0       = tag
  byte 1..3    = unused
  byte 4..7    = x: float32
  byte 8..11   = y: float32
  byte 12..15  = z: float32
  byte 16..19  = pitch: float32
  byte 20..23  = min_pitch: float32
  byte 24..27  = max_pitch: float32
  byte 28..31  = duration_ms: uint32

SERVO ('P'):
  byte 0       = tag
  byte 4       = servo_id
  byte 8..11   = position_raw: float32
  byte 12..15  = duration_ms: uint32; 0 使用固件默认 1000 ms

STOP ('S'):
  byte 0       = tag
  remaining bytes unused
```

v1 状态包的 `byte 0..7` 是 ASCII 文本，后期版本的 `byte 8..31` 携带
servo 1..6 的 `float32 raw` 反馈。常见文本包括：

```text
I2C_RDY_  ARM_RDY_  ARM_OK__  ARM_DONE
SVO_OK__  STOP_OK_  NO_SOLVE  ARM_ERR_  BAD_CMD_
```

v1 的根本限制：

- 状态不携带命令 ID，无法判断 `ARM_OK__` 或 `ARM_DONE` 属于哪条命令；
- `ARM_OK__` 只表示固件进入处理路径，不等于 IK 成功、舵机写入成功或到达目标；
- 相同文本会被后续轮询重复读取，旧状态可能被误当成新命令确认；
- 错误只有粗粒度文本，无法区分协议错误、舵机写失败、反馈失败和运动超时；
- 夹爪没有独立停止命令，只能下发新位置或使用全局 STOP。

#### 2.3.2 v3：带滚动伺服的二进制生命周期协议（当前版本）

当前协议版本固定为 `3`。命令头统一为：

```text
byte 0       = tag: 'A' / 'T' / 'F' / 'P' / 'H' / 'S'
byte 1       = protocol_version: uint8, fixed 3
byte 2..5    = wire_command_id: uint32 little-endian
byte 6..7    = duration_ms: uint16 little-endian; motion range 1..30000,
               STOP fixed 0
```

`wire_command_id` 由 ROS 控制器为每次硬件写入单独生成；它与公共
`ArmCommand.sequence_id` 分离，由控制器维护二者映射。这样旧字符串调用者使用
`sequence_id=0` 时仍能可靠关联固件响应。

各命令 payload：

```text
ARM ('A'):
  byte 8..11   = x, float32 little-endian
  byte 12..15  = y, float32 little-endian
  byte 16..19  = z, float32 little-endian
  byte 20..23  = pitch, float32 little-endian
  byte 24..27  = min_pitch, float32 little-endian
  byte 28..31  = max_pitch, float32 little-endian

SERVO ('P'):
  byte 8       = servo_id, uint8, range 1..6
  byte 12..15  = position_raw, float32 little-endian

SERVO_HALT ('H'):
  byte 8       = servo_id, uint8, range 1..6
  duration_ms fixed 0; remaining payload bytes fixed 0

STOP ('S'):
  no payload
```

`CARTESIAN_SERVO ('T')` 与 `ARM ('A')` 使用相同的 24 字节笛卡尔 payload，
但 `duration_ms` 表示流 watchdog，当前允许 `100..1000 ms`，默认 `200 ms`。
`CARTESIAN_SERVO_END ('F')` 不携带目标且 `duration_ms=0`，只结束当前流并等待
最后安装的关节目标稳定完成。`A` 保留给 Home、探针和其他一次性运动。

`H` 只停止指定舵机当前运动，不改变其他舵机；`S` 仍是停止全部舵机的全局安全
命令。两者不得复用同一标签，避免旧固件把单舵机停止误解释为全局停止。`H` 是可
抢占命令：主机必须取消被它替代的活动 `P`，立即发送新的 `wire_command_id`，并忽略
旧 `P` 的迟到状态；若 `A` 仍在活动，主机必须先等待其生命周期结束再发送 `H`。
固件成功发出舵机 `MOVE_STOP` 后直接返回 `COMPLETED`；这表示
停止帧已发送完成，不表示已经通过位置反馈确认舵机静止。旧 v2 固件收到未知 `H`
必须返回 `FAILED/BAD_COMMAND`，不得执行全局停止。

状态包布局：

```text
byte 0       = magic: 0xA5
byte 1       = protocol_version: uint8, fixed 3
byte 2       = lifecycle: READY=0, ACCEPTED=1, EXECUTING=2,
               COMPLETED=3, FAILED=4, STOPPING=5
byte 3       = firmware_error
byte 4..7    = wire_command_id: uint32 little-endian
byte 8..31   = servo 1..6 raw position, float32 little-endian
```

`firmware_error` 稳定值：`0=NONE`、`1=BAD_PROTOCOL`、`2=BAD_COMMAND`、
`3=ARM_NOT_READY`、`4=NO_IK_SOLUTION`、`5=SERVO_WRITE_FAILED`、
`6=SERVO_FEEDBACK_FAILED`、`7=MOTION_TIMEOUT`、
`8=STREAM_STEP_TOO_LARGE`、`9=STREAM_TIMEOUT`、
`10=SERVO_DEADLINE_MISSED`。

正常 ARM/P 命令生命周期：

```text
ROS ArmCommand(sequence_id)
  -> controller 分配 wire_command_id
  -> STM32 收包并校验：ACCEPTED
  -> IK/舵机写入成功：EXECUTING
  -> 反馈确认到达：COMPLETED
  -> 任一步失败：FAILED + firmware_error
```

流式 `T/F` 生命周期：

```text
T 收包：ACCEPTED
  -> 增量 IK 成功并安装 q_goal：EXECUTING
  -> 后续 T 可替代旧 q_goal
F 收包：ACCEPTED / EXECUTING
  -> q_cmd、规划速度和 q_meas 连续稳定三轮：COMPLETED
异常缺失 F：STOPPING/STREAM_TIMEOUT
  -> 按加速度刹停后 FAILED，必须重新 Home
```

`NO_IK_SOLUTION` 与 `STREAM_STEP_TOO_LARGE` 都只拒绝当前 `T`，不覆盖最后有效
`q_goal`。控制器只有看到匹配 T 的 `EXECUTING` 才发送下一条流式目标，并先保存该
目标为“最后已安装可达坐标”。队列分别保存最新 T 与 END 标志，END 不得覆盖尚未
下发的最终坐标。

`sequence_id` 属于 ROS 调用方，负责关联 `/arm/command` 与 `/arm/state`；
`wire_command_id` 属于 RDK↔STM32 链路，只在命令真正下发硬件时分配。排队但尚未
下发的命令没有 `wire_command_id`；兼容入口的 `sequence_id=0` 仍会获得非零
`wire_command_id`，因此两种 ID 不能混用。

ROS 只有在 `wire_command_id` 与当前活动命令匹配时才接受
`ACCEPTED/EXECUTING/COMPLETED/FAILED`。ID 不匹配的有效状态只能证明 I2C 链路
存活，不能清除命令 watchdog，也不能改变当前命令结果。

`READY/wire_command_id=0` 表示固件启动后就绪。如果控制器已经见过非零 ID，却再次
收到该状态，则按固件重启处理：清除活动命令、禁用遥操，并要求清错和重新 Home。

固件状态到 ROS 状态的映射：

| 固件状态 | ROS `ArmState` |
| --- | --- |
| `READY` | `STATE_IDLE` |
| `ACCEPTED` / `EXECUTING` | `STATE_MOVING` |
| `COMPLETED` | `STATE_SUCCEEDED` |
| `FAILED/NO_IK_SOLUTION` | `STATE_IDLE + error_code=0x0020`，非锁存拒绝 |
| `FAILED/STREAM_STEP_TOO_LARGE` | `STATE_IDLE + error_code=0x0026`，非锁存拒绝 |
| `STOPPING` | `STATE_ERROR + command_phase=STOPPING`，刹停并要求 Home |
| 其他 `FAILED` | `STATE_ERROR`，按错误码执行恢复流程 |

控制器以 10 Hz 轮询状态，中间生命周期可能在两次读取之间快速经过，因此日志不保证
逐次看到 `ACCEPTED → EXECUTING → COMPLETED` 的每一项；判定依据是最新的匹配 ID
状态，而不是文本出现次数。

固件错误到 ROS 稳定错误码的映射：

| firmware_error | ROS error_code |
| --- | --- |
| `BAD_PROTOCOL=1` | `0x0018` |
| `BAD_COMMAND=2` | `0x0022` |
| `ARM_NOT_READY=3` | `0x0021` |
| `NO_IK_SOLUTION=4` | `0x0020` |
| `SERVO_WRITE_FAILED=5` | `0x0023` |
| `SERVO_FEEDBACK_FAILED=6` | `0x0024` |
| `MOTION_TIMEOUT=7` | `0x0025` |
| `STREAM_STEP_TOO_LARGE=8` | `0x0026` |
| `STREAM_TIMEOUT=9` | `0x0027` |
| `SERVO_DEADLINE_MISSED=10` | `0x0028` |

协议不匹配或仍收到 v1 文本状态时，控制器拒绝普通运动。全局 STOP 保留兼容路径：
当前固件即使收到没有有效 v3 头的 `'S'`，仍会尝试停止全部舵机，但其
`wire_command_id` 可能为 0，不能提供完整的命令关联保证。

#### 2.3.3 两版核心对比

| 能力 | v1 | v3 |
| --- | --- | --- |
| 命令包 | 32 字节，按固定偏移解析 | 32 字节，统一版本/ID/时长头 |
| 状态表达 | 8 字节 ASCII 文本 | 二进制生命周期 + 错误码 |
| 命令关联 | 无 | `wire_command_id` 精确匹配 |
| 完成判定 | `ARM_DONE` 无法确认归属 | 匹配 ID 的 `COMPLETED` |
| watchdog | 任意可读状态可能被误当确认 | 仅匹配 ID 的 ACK 清除 |
| 错误分类 | `NO_SOLVE/ARM_ERR/BAD_CMD` | 10 类稳定固件错误 |
| 位置反馈 | 后期追加，和文本状态松散组合 | 状态包固定携带 6 路 raw |
| 流式伺服 | 无 | `'T'` 更新目标、`'F'` 正常结束 |
| 夹爪停止 | 无独立命令 | `'H'` 停指定舵机 |
| 固件重启识别 | 无可靠依据 | `READY + wire_id=0` 状态机识别 |
| 混用安全性 | 可能误确认旧状态 | 版本不匹配时拒绝运动 |

对开发和 VLA 数据采集的结论：

- 新代码只使用 `/arm/command` 与 `/arm/state`，不解析 `/status_topic` 文本；
- action 记录 `ArmCommand` 及其 `sequence_id`，执行结果记录匹配的
  `ArmState.state/error_code/position_valid`；
- 同步记录真实 `joint_position` 和 `gripper_position`，不能只记录请求坐标；
- `NO_IK_SOLUTION` 代表目标被拒绝，不能把该 action 标成成功示范；
- ROS 与 STM32 必须成对部署 v3，不能只更新一端。

线上的单位固定为：`x/y/z` 使用厘米（cm），`pitch/min_pitch/max_pitch` 使用度
（deg），ROS `duration_sec` 使用秒，I2C `duration_ms` 使用毫秒，舵机位置反馈使用
原始 `raw` 值。当前字符串接口只用于联调，不作为稳定公共 API。

## 3. 稳定 ROS2 接口

当前稳定接口使用类型化消息，并由 `action_pkg` 统一拥有 I2C 总线。字符串接口已
标记为 deprecated。

### 3.1 `/arm/command`

消息类型：`action_interfaces/msg/ArmCommand`

```text
uint8 MODE_STOP=0
uint8 MODE_END_EFFECTOR=1
uint8 MODE_JOINT=2
uint8 MODE_GRIPPER=3
uint8 MODE_GRIPPER_STOP=4
uint8 MODE_CARTESIAN_SERVO=5
uint8 MODE_CARTESIAN_SERVO_END=6
uint8 MODE_WRIST_ROLL=7

std_msgs/Header header
uint8 mode
float32 x
float32 y
float32 z
float32 pitch
float32[5] joint_position
float32 gripper_position
float32 duration_sec
uint32 sequence_id
```

字段规则：

- `MODE_STOP`：忽略其他目标字段，立即请求安全停止；
- `MODE_END_EFFECTOR`：使用 `x/y/z/pitch`，其中 `x/y/z` 单位固定为厘米（cm）；
- `MODE_CARTESIAN_SERVO`：使用绝对 `x/y/z/pitch` 更新流式目标；
  `duration_sec` 是 watchdog，Xbox 默认 `0.20 s`；
- `MODE_CARTESIAN_SERVO_END`：不携带新目标，正常结束当前流并等待最后目标稳定；
- `MODE_JOINT`：使用 `joint_position`，单位 rad；
- `MODE_WRIST_ROLL`：只使用 `joint_position[4]`，单位 rad，范围由
  `joint_5_wrist_roll` 的关节限位约束；controller 将其转换为映射舵机的 `P`
  单舵机位置帧。该模式不表示通用 `MODE_JOINT` 已开放；
- `MODE_GRIPPER`：使用 `gripper_position`，规范范围 `[0, 1]`，其中
  `0` 表示完全张开、`1` 表示完全闭合；
- `MODE_GRIPPER_STOP`：忽略位置和时长，只停止映射到 `gripper` 的舵机当前运动；
  它必须抢占未完成的 `MODE_GRIPPER`，但不能替代 `MODE_STOP` 的整臂安全停止；
  若停止帧在有界重试后仍写入失败，控制节点进入独立的锁存错误
  `ERR_GRIPPER_STOP_WRITE (0x001A)`，必须执行错误复位和 Home 后才能恢复遥控；
- `duration_sec`：目标运动模式的期望执行时间，必须为有限正数并限制在配置范围内；
  `MODE_STOP`、`MODE_GRIPPER_STOP` 和 `MODE_CARTESIAN_SERVO_END` 忽略该字段；
- `sequence_id`：调用方生成的递增编号，用于关联命令和状态。

未被当前 `mode` 使用的字段必须被控制节点忽略。所有模式在写入硬件前都必须经过范围、有限值和软限位检查。

### 3.2 `/arm/emergency_stop`

消息类型：`std_msgs/msg/Bool`

- `true`：进入锁存急停状态，拒绝后续运动命令；
- `false`：不能自动解除急停，只表示调用方不再请求急停；
- 解除急停必须通过显式 reset 接口并确认硬件安全。

### 3.3 `/arm/state`

消息类型：`action_interfaces/msg/ArmState`

```text
uint8 STATE_IDLE=0
uint8 STATE_MOVING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_ERROR=3
uint8 STATE_ESTOP=4

uint8 PHASE_NONE=0
uint8 PHASE_ACCEPTED=1
uint8 PHASE_EXECUTING=2
uint8 PHASE_COMPLETED=3
uint8 PHASE_FAILED=4
uint8 PHASE_STOPPING=5

std_msgs/Header header
uint8 state
uint8 command_phase
uint32 sequence_id
float32[5] joint_position
float32 gripper_position
bool position_valid
uint16 error_code
string error_message
```

字段规则：

- `sequence_id` 对应最近接受或完成的命令；
- `command_phase` 保留匹配固件生命周期；流式调用方只把匹配
  `PHASE_EXECUTING` 的 T 坐标视为已安装目标；
- `gripper_position` 使用与命令相同的 `[0, 1]` 规范范围：`0=open`、
  `1=closed`；
- `position_valid=false` 时，调用方不得把位置数组当作真实反馈；
- `STATE_SUCCEEDED` 表示命令完成，不代表上层任务成功；
- `error_code` 是稳定机器接口，`error_message` 只用于诊断。
- `NO_IK_SOLUTION/0x0020` 是非锁存目标拒绝：状态使用 `STATE_IDLE` 并保留
  `error_code` 和失败命令的 `sequence_id`；下一条有效命令可直接执行，不需要调用
  `/arm/reset_error`。
- `STREAM_STEP_TOO_LARGE/0x0026` 与 NO_IK 一样只拒绝当前流式候选；
  `STREAM_TIMEOUT/0x0027` 和 `SERVO_DEADLINE_MISSED/0x0028` 会先发布
  `PHASE_STOPPING`，随后进入锁存错误并要求 Home。

### 3.4 `/joint_states`

消息类型：`sensor_msgs/msg/JointState`

当硬件支持位置反馈时发布以下规范名称：

```text
joint_1_base
joint_2_shoulder
joint_3_elbow
joint_4_wrist_pitch
joint_5_wrist_roll
gripper
```

机械臂关节位置单位为 rad。夹爪在 `/joint_states` 中必须使用其物理关节单位：旋转夹爪使用 rad，直线夹爪使用 m；`[0, 1]` 规范值只用于 `ArmCommand` 和 `ArmState`。消息必须按 `name` 匹配，调用方不能依赖数组的偶然顺序。

### 3.5 `/arm/reset_error`

接口类型：`std_srvs/srv/Trigger`

仅在以下条件全部满足时返回成功：

- 急停输入已经释放；
- I2C 通信正常；
- 控制器处于可恢复状态；
- 恢复过程不会产生机械臂运动。

## 4. 配置契约

以下内容必须进入可追踪配置，不允许散落在代码常量中：

```text
i2c_bus
i2c_address
command_timeout_sec
min_duration_sec
max_duration_sec
joint_names
servo_id_map
joint_zero_offsets
joint_directions
joint_lower_limits
joint_upper_limits
gripper_closed_raw
gripper_open_raw
end_effector_frame
end_effector_units
```

STM32 源码已确认串口舵机映射为：底座到腕部依次使用 id 6～2，夹爪使用
id 1。通用关节直接控制仍保持禁用；已确认的腕转专用路径仅允许
`joint_5_wrist_roll -> servo 2`，角度限制为 `[-pi/2, pi/2] rad`。其稳定验证错误码
为 `ERR_WRIST_ROLL_RANGE=0x001B` 和 `ERR_WRIST_ROLL_UNMAPPED=0x001C`。

## 5. 安全行为

控制节点必须满足：

1. 启动时不产生运动；
2. 同一时刻只有一个进程拥有目标 I2C 地址；
3. 拒绝过期、重复、非有限值、越界和未知模式命令；
4. 急停优先级高于所有普通命令，并保持锁存；
5. I2C 连续失败后进入 `STATE_ERROR`，不继续发送运动命令；
6. 正常退出时尽力发送 STOP 并关闭 SMBus；
7. 单个舵机每批最多重试 3 次；一次失败批次只保留最后有效位置，不立即锁死整机；
   同一舵机连续 3 个失败批次后必须设置 `position_valid=false` 并报告
   `SERVO_FEEDBACK_FAILED`，任一有效回读会清零连续失败计数；
8. 无法在上述有界窗口内确认反馈时，不得伪造新的关节位置；
9. 日志不得只记录“失败”，必须包含 `sequence_id`、稳定错误码、无效舵机 ID 和
   六个舵机 raw 快照。
10. `NO_IK_SOLUTION` 发生在新舵机目标下发前，只拒绝该目标并清除排队命令；
    不锁存 `STATE_ERROR`，也不要求错误复位或 Home。通信、反馈、写入、超时、协议、
    固件重启和急停错误仍按原安全恢复流程处理。

底层 STOP 的真实语义必须与 STM32 固件共同确认：是保持当前位置、停止轨迹还是舵机卸力。确认前不能把它描述为急停。

## 6. 验收标准

- [x] `ArmCommand`、`ArmState` 能独立完成 ROS2 构建和接口显示；
- [ ] 所有模式都有正常、越界、NaN、超时和 I2C 失败测试；
- [x] `sequence_id` 与 `wire_command_id` 能关联命令、完成状态和错误；
- [ ] 急停锁存及显式恢复流程已验证；
- [ ] 舵机 ID、方向、零位和软限位已在断电或架空条件下核对；
- [ ] `/joint_states` 与外部测量方向一致，或明确标记反馈无效；
- [ ] 进程异常退出不会遗留持续运动命令；
- [x] 当前 `/command_topic` 兼容层不绕过安全检查。

## 7. 当前缺口

- `MODE_JOINT` 仍保持禁用，直到关节方向、零位和限位完成独立验收；
- v3 状态包没有固件源码 commit、构建 ID 或 boot counter，板上二进制仍需通过烧录
  记录或产物哈希与源码版本绑定；
- v3 状态包只有 6 路 raw，没有逐舵机有效位和明确失败舵机 ID；当前控制器只能根据
  raw 有效范围补充诊断；
- STM32 的 I2C 错误自动恢复已修改但仍需完成链接、烧录和 BERR/OVR 故障注入，
  该事项属于传输可靠性，不改变本文定义的 v3 字节布局。

这些缺口属于机械臂控制层，应在本分支内解决。
