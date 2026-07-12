# 机械臂控制接口契约

状态：`v0.1 draft`

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
| `/status_topic` | `std_msgs/msg/String` | STM32 返回的 8 字节状态文本 |

### 2.3 当前 I2C 边界

| 项目 | 当前值 |
| --- | --- |
| I2C bus | `5` |
| slave address | `0x30` |
| command packet | 固定 32 字节 |
| status packet | 固定 8 字节 |

当前命令包布局：

```text
ARM:
  byte 0       = 'A'
  byte 4..7    = x, float32 little-endian
  byte 8..11   = y, float32 little-endian
  byte 12..15  = z, float32 little-endian
  byte 16..19  = pitch, float32 little-endian
  byte 20..23  = min_pitch, float32 little-endian
  byte 24..27  = max_pitch, float32 little-endian
  byte 28..31  = time, uint32 little-endian

SERVO:
  byte 0       = 'P'
  byte 4       = servo_id, uint8, range 1..6
  byte 8..11   = angle, float32 little-endian

STOP:
  byte 0       = 'S'
```

现有代码没有确认各数值的物理单位，也没有结构化关节反馈。因此当前字符串接口只用于联调，不作为稳定公共 API。

## 3. 稳定 ROS2 接口

稳定接口应使用类型化消息，并由 `action_pkg` 统一拥有 I2C 总线。字符串接口在迁移完成后标记为 deprecated。

### 3.1 `/arm/command`

消息类型：待新增 `action_pkg/msg/ArmCommand`

```text
uint8 MODE_STOP=0
uint8 MODE_END_EFFECTOR=1
uint8 MODE_JOINT=2
uint8 MODE_GRIPPER=3

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
- `MODE_END_EFFECTOR`：使用 `x/y/z/pitch`，坐标系和单位由硬件配置固定；
- `MODE_JOINT`：使用 `joint_position`，单位 rad；
- `MODE_GRIPPER`：使用 `gripper_position`，规范范围 `[0, 1]`；
- `duration_sec`：期望执行时间，必须为有限正数并限制在配置范围内；
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
- `gripper_position` 使用与命令相同的 `[0, 1]` 规范范围；
- `position_valid=false` 时，调用方不得把位置数组当作真实反馈；
- `STATE_SUCCEEDED` 表示命令完成，不代表上层任务成功；
- `error_code` 是稳定机器接口，`error_message` 只用于诊断。

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

现有代码只证明舵机 ID 范围为 1～6，尚未验证它们与 5 个机械臂关节和夹爪的对应关系。映射确认前不得启用 `MODE_JOINT`。

## 5. 安全行为

控制节点必须满足：

1. 启动时不产生运动；
2. 同一时刻只有一个进程拥有目标 I2C 地址；
3. 拒绝过期、重复、非有限值、越界和未知模式命令；
4. 急停优先级高于所有普通命令，并保持锁存；
5. I2C 连续失败后进入 `STATE_ERROR`，不继续发送运动命令；
6. 正常退出时尽力发送 STOP 并关闭 SMBus；
7. 无法确认反馈时设置 `position_valid=false`，不伪造关节位置；
8. 日志不得只记录“失败”，必须包含 `sequence_id` 和稳定错误码。

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

- 当前只有字符串命令，没有 `ArmCommand` 和 `ArmState`；
- `/status_topic` 只有 8 字节文本，不能表达结构化执行状态；
- 没有真实关节位置反馈契约；
- 坐标、角度和时间单位尚未与 STM32 固件确认；
- 舵机 ID 映射、软限位、急停和通信失败行为尚未完成实机验证。

这些缺口属于机械臂控制层，应在本分支内解决。
