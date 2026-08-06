# Xbox 机械臂遥控契约

本文定义 Xbox 手柄到机械臂绝对目标的映射，并提供不接硬件的 Shadow 验证入口。

## 数据流

真实模式：

```text
/joy -> arm_teleop_node -> /arm/command -> arm_controller_node -> STM32
                    \-> /arm/emergency_stop
/arm/state ---------/
         \-> arm_state_filter_node -> /arm/state_filtered (VLA only)
```

Shadow 模式只订阅 `/joy_sim`，只发布 `/arm/teleop_command` 和
`/arm/teleop_emergency_stop`，不会调用真实 reset service，也不会发布
`/arm/command`。

## 输入与按键

输入类型为 `sensor_msgs/msg/Joy`。RDK 当前 Xbox 映射如下：

| 输入 | 功能 |
| --- | --- |
| 左摇杆上下 `axes[1]` | `x` 增量，向上为正 |
| 左摇杆左右 `axes[0]` | `y` 增量，向左为正 |
| 右摇杆上下 `axes[3]` | `z` 增量，向上为正 |
| 右摇杆左右 `axes[2]` | 腕部旋转增量，向左为正 |
| RB + 右摇杆左右 | `pitch` 增量，向左为正 |
| RT `axes[4]` | 逐渐闭合夹爪；松开时只停止夹爪当前运动 |
| LT `axes[5]` | 逐渐张开夹爪 |
| A `buttons[0]` | 在安全条件满足时切换遥控使能 |
| B `buttons[1]` | 按下急停；松开只释放急停请求，不解除锁存 |
| LB+RB+X | 长按 1 秒解除错误/急停锁存 |
| LB+RB+Y | 长按 1 秒回到已知绝对坐标 |

夹爪规范值固定为 `0=open`、`1=closed`。RT/LT 的按下程度按
`(1 - raw) / 2` 转换到 `[0,1]`。RT/LT 有效输入优先于摇杆，因此操作夹爪时
不要求摇杆先回中；没有夹爪输入时，XYZ 或带 RB 的 pitch 优先于腕转。同一个
控制 tick 仍只发送一种运动，模式切换前会先结束上一条流。

`joy_linux` 启动后，尚未产生输入事件的绝对扳机轴可能暂时上报 `0.0`，而本手柄
真正松开值为 `+1.0`。遥操会分别忽略每个扳机的这种未初始化值，直到实际观察到该轴
回到松开端点；Joy 超时或输入无效后会重新执行这一初始化。这样启动期 `0.0` 不会被
误当成半按并持续发送夹爪流。首次使用某个扳机前若它尚未初始化，完整按下再松开一次
即可激活其模拟量控制。

当前验收配置的满幅速度为平移 `1.5 cm/s`、俯仰 `5 deg/s`、腕转
`20 deg/s`、夹爪归一化行程 `0.5/s`。

遥操层不再对 `x/y/z` 做矩形工作空间钳制，坐标目标会持续按摇杆输入累计；
`pitch` 和腕转均限制在 `[-90, 90] deg`。XYZ 是否可达最终由 STM32 的 IK 和关节角约束
决定。无解目标会被固件拒绝，遥操回退到最后一个成功目标并停止发送；摇杆和扳机
回中后自动恢复，不需要清错或 Home。因此实机操作必须清空工作空间并保持 B 急停
可用。

## 状态与安全

- 启动不运动；即使有效关节反馈看起来位于固件复位姿态，也必须先长按
  LB+RB+Y 完成一次显式 Home。物理位置不能证明 STM32 内部运动规划器已 ready。
- 固件标称复位目标为 `(15, 0, 2) cm`，反解初始 `pitch=-54.48 deg`。
- A 使能要求：输入新鲜、坐标已同步、机械臂非 `MOVING/ERROR/ESTOP`；不再要求
  摇杆与扳机中立，因此带着有效输入使能会立即产生对应运动。
- 再按 A 只停止发送新目标，不发送 STOP，且保留坐标同步。
- 流式目标 watchdog 默认 `300 ms`。它为 10 Hz 状态轮询、匹配 `EXECUTING`
  确认和下一条目标写入保留一个控制周期余量；超过该时间仍未收到新目标时，固件
  仍按异常断流执行受控制动并要求重新 Home。
- Joy 超时也不会发送 STOP，但会关闭遥控并使坐标失去同步；重新控制前必须
  执行回零。
- 急停或锁存错误会使坐标失去同步。解除锁存后必须单独执行回零，回零成功后
  才能再次按 A。`NO_IK_SOLUTION` 是例外：只回退目标并等待控制输入回中。
- Home 固定依次打开 1 号夹爪、把 2 号腕转舵机恢复到 `0 rad`，再执行 6～3 号
  舵机的笛卡尔 Home；任一阶段失败都不会继续下一阶段。若夹爪反馈已连续 3 帧处于
  安全开度（规范位置不超过 `0.10`），但打开动作仍未报告完成，遥操会先发送单夹爪
  `MODE_GRIPPER_STOP`，确认停止后再继续腕转 Home，避免卡在机械端点。
- Home 仍在执行时，按 A 会提示 `home/reset operation is in progress`。固件报告
  笛卡尔 Home 完成后，如果关节反馈在 `3 s` 内仍未进入 Home 容差，遥操会报告
  验证失败、打印实际与期望关节角并结束本次等待；机械臂保持未同步，但允许操作者
  重新长按 LB+RB+Y。该超时不会绕过反馈直接使能。
- Home 关节容差为 `5.25 deg`：其中 `5 deg` 是几何容差，额外 `0.25 deg` 覆盖
  Lobot 反馈约 `0.24 deg/raw tick` 的量化。现场 `5.04 deg` 的边界读数因此可以通过，
  明显偏离 Home 的姿态仍会被拒绝。
- Xbox 腕转和夹爪使用 `U/G` 单舵机滚动流：ROS 保持 `10 Hz` 绝对目标，STM32
  以 `25 Hz/40 ms` 生成位置小段；默认 watchdog 为 `300 ms`。腕转回中时发送一次
  `G`，平滑完成最后目标。夹爪松开、模式切换、目标拒绝或接触检测统一使用 `H`
  立即停止 1 号舵机；等待 `H` 的匹配 `COMPLETED` 后才继续机械臂命令，避免固件
  `G(gripper)` 的反馈稳定条件长期不成立而阻塞整个控制链。旧
  `MODE_WRIST_ROLL`、`MODE_GRIPPER` 和 `P` 语义保持不变，继续服务 Home、探针及
  非流式调用。
- 直接舵机目标只相对最后一条匹配 `PHASE_EXECUTING` 的已安装目标继续累计；夹爪
  单次最多 `0.075`（当前映射为 `37.5 raw/9 deg`），腕转单次最多 `9 deg`。
  ACK 延迟时后续 10 Hz tick 只刷新同一安全边界，不继续累计到固件的 `12 deg`
  拒绝线之外。若仍收到 `STREAM_STEP_TOO_LARGE`，teleop 回退到最后确认目标、只
  发送一次对应的 `H`、暂停控制并等待输入回中。
- 真实控制 launch 同时启动只读 `arm_state_filter_node`。`/arm/state` 保持原始事实
  来源，`/arm/state_filtered` 对五关节和夹爪执行 3 点因果中值与 One Euro 自适应
  低通；默认参数为 `min_cutoff=1.0 Hz`、`beta=1.5`、导数截止频率 `1.0 Hz`，
  时间戳间隔超过 `0.5 s` 时重新初始化。该话题只供 VLA observation 和记录器使用，
  不参与遥控、安全门或完成判定。

## Shadow 模拟

构建并 source 工作区后启动：

```bash
ros2 launch action_pkg arm_teleop_shadow.launch.py
```

先发送一次 A 按下和松开：

```bash
ros2 topic pub --once /joy_sim sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0], buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
ros2 topic pub --once /joy_sim sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

然后模拟左摇杆向上 1 秒，并在另一个终端观察命令：

```bash
ros2 topic echo /arm/teleop_command
ros2 topic pub --rate 10 --times 10 /joy_sim sensor_msgs/msg/Joy \
  "{axes: [0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
ros2 topic pub --once /joy_sim sensor_msgs/msg/Joy \
  "{axes: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0], buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"
```

预期 `x` 从 `15 cm` 逐渐增加约 `1.5 cm`，真实 `/arm/command` 没有发布者。

## 真实模式

确认机械臂周围无人员和障碍物后运行：

```bash
ros2 launch action_pkg arm_xbox_control.launch.py
```

首次验证只单独推动一个轴，并通过 `/arm/state`、`/joint_states` 和
`/arm/teleop_enabled` 核对状态。每次实机控制栈启动后的顺序固定为：长按
LB+RB+Y，等待夹爪、腕转和笛卡尔 Home 全部完成，再按 A。急停或错误恢复顺序为：
松开 B、长按 LB+RB+X、长按 LB+RB+Y 等待回零完成、确认当前输入符合预期后按 A。
A 不检查摇杆或扳机是否中立，非零输入会立即运动。
