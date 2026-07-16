# 机械臂控制接口契约

状态：`v0.2 draft`

适用分支：`feature/control`

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

## 2. 当前接口

当前实现位于 `ros2_ws/src/action_pkg/action_pkg/i2c_controller.py`。

### 2.1 ROS2 输入

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/command_topic` | `std_msgs/msg/String` | `ARM`、`SERVO`、`STOP` 字符串命令 |

支持的命令：

```text
ARM x y z pitch min_pitch max_pitch time
SERVO id angle
STOP
```

`CAR` 命令属于底盘，不在本机械臂接口契约内。

### 2.2 ROS2 输出

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `/status_topic` | `std_msgs/msg/String` | ROS 将 v2 生命周期映射为旧 8 字节调试文本 |

### 2.3 I2C v2 边界

| 项目 | 当前值 |
| --- | --- |
| I2C bus | `5` |
| slave address | `0x30` |
| command packet | 固定 32 字节 |
| status packet | 固定 32 字节 |
| protocol version | `2` |

命令头统一为：

```text
byte 0       = tag: 'A' / 'P' / 'H' / 'S'
byte 1       = protocol_version: uint8, fixed 2
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
byte 1       = protocol_version: uint8, fixed 2
byte 2       = lifecycle: READY=0, ACCEPTED=1, EXECUTING=2,
               COMPLETED=3, FAILED=4
byte 3       = firmware_error
byte 4..7    = wire_command_id: uint32 little-endian
byte 8..31   = servo 1..6 raw position, float32 little-endian
```

`firmware_error` 稳定值：`0=NONE`、`1=BAD_PROTOCOL`、`2=BAD_COMMAND`、
`3=ARM_NOT_READY`、`4=NO_IK_SOLUTION`、`5=SERVO_WRITE_FAILED`、
`6=SERVO_FEEDBACK_FAILED`、`7=MOTION_TIMEOUT`。

ROS 只有在 `wire_command_id` 与当前命令匹配时才接受
`ACCEPTED/EXECUTING/COMPLETED/FAILED`。任何可读但 ID 不匹配的状态只能证明
I2C 链路存活，不能清除命令确认 watchdog。旧版文本状态包不能驱动普通运动；
协议版本不匹配时只保留 STOP 能力并进入错误状态。

末端坐标 `x/y/z` 的固件单位已确认为厘米（cm）；其他字段仍应以类型化接口和配置契约为准。当前字符串接口只用于联调，不作为稳定公共 API。

## 3. 稳定 ROS2 接口

稳定接口应使用类型化消息，并由 `action_pkg` 统一拥有 I2C 总线。字符串接口在迁移完成后标记为 deprecated。

### 3.1 `/arm/command`

消息类型：待新增 `action_pkg/msg/ArmCommand`

```text
uint8 MODE_STOP=0
uint8 MODE_END_EFFECTOR=1
uint8 MODE_JOINT=2
uint8 MODE_GRIPPER=3
uint8 MODE_GRIPPER_STOP=4

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
- `MODE_JOINT`：使用 `joint_position`，单位 rad；
- `MODE_GRIPPER`：使用 `gripper_position`，规范范围 `[0, 1]`，其中
  `0` 表示完全张开、`1` 表示完全闭合；
- `MODE_GRIPPER_STOP`：忽略位置和时长，只停止映射到 `gripper` 的舵机当前运动；
  它必须抢占未完成的 `MODE_GRIPPER`，但不能替代 `MODE_STOP` 的整臂安全停止；
  若停止帧在有界重试后仍写入失败，控制节点进入独立的锁存错误
  `ERR_GRIPPER_STOP_WRITE (0x001A)`，必须执行错误复位和 Home 后才能恢复遥控；
- `duration_sec`：目标运动模式的期望执行时间，必须为有限正数并限制在配置范围内；
  `MODE_STOP` 和 `MODE_GRIPPER_STOP` 忽略该字段；
- `sequence_id`：调用方生成的递增编号，用于关联命令和状态。

未被当前 `mode` 使用的字段必须被控制节点忽略。所有模式在写入硬件前都必须经过范围、有限值和软限位检查。

### 3.2 `/arm/emergency_stop`

消息类型：`std_msgs/msg/Bool`

- `true`：进入锁存急停状态，拒绝后续运动命令；
- `false`：不能自动解除急停，只表示调用方不再请求急停；
- 解除急停必须通过显式 reset 接口并确认硬件安全。

### 3.3 `/arm/state`

消息类型：待新增 `action_pkg/msg/ArmState`

```text
uint8 STATE_IDLE=0
uint8 STATE_MOVING=1
uint8 STATE_SUCCEEDED=2
uint8 STATE_ERROR=3
uint8 STATE_ESTOP=4

std_msgs/Header header
uint8 state
uint32 sequence_id
float32[5] joint_position
float32 gripper_position
bool position_valid
uint16 error_code
string error_message
```

字段规则：

- `sequence_id` 对应最近接受或完成的命令；
- `gripper_position` 使用与命令相同的 `[0, 1]` 规范范围：`0=open`、
  `1=closed`；
- `position_valid=false` 时，调用方不得把位置数组当作真实反馈；
- `STATE_SUCCEEDED` 表示命令完成，不代表上层任务成功；
- `error_code` 是稳定机器接口，`error_message` 只用于诊断。
- `NO_IK_SOLUTION/0x0020` 是非锁存目标拒绝：状态使用 `STATE_IDLE` 并保留
  `error_code` 和失败命令的 `sequence_id`；下一条有效命令可直接执行，不需要调用
  `/arm/reset_error`。

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
id 1。关节直接控制仍保持禁用，直到关节限位和实机方向完成独立验收。

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

- [ ] `ArmCommand`、`ArmState` 能独立完成 ROS2 构建和接口显示；
- [ ] 所有模式都有正常、越界、NaN、超时和 I2C 失败测试；
- [ ] `sequence_id` 能关联命令、完成状态和错误；
- [ ] 急停锁存及显式恢复流程已验证；
- [ ] 舵机 ID、方向、零位和软限位已在断电或架空条件下核对；
- [ ] `/joint_states` 与外部测量方向一致，或明确标记反馈无效；
- [ ] 进程异常退出不会遗留持续运动命令；
- [ ] 当前 `/command_topic` 兼容层不绕过安全检查。

## 7. 当前缺口

- v2 协议与真实关节反馈仍需完成固件构建、烧录和故障注入验收；
- 末端 `x/y/z` 单位已确认为厘米（cm），pitch 等剩余物理语义仍需与 STM32 固件保持一致；
- 舵机 ID 映射、软限位、急停和通信失败行为尚未完成实机验证。

这些缺口属于机械臂控制层，应在本分支内解决。
