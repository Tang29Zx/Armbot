# Xbox 机械臂遥控契约

本文定义 Xbox 手柄到机械臂绝对目标的映射，并提供不接硬件的 Shadow 验证入口。

## 数据流

真实模式：

```text
/joy -> arm_teleop_node -> /arm/command -> arm_controller_node -> STM32
                    \-> /arm/emergency_stop
/arm/state ---------/
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
`(1 - raw) / 2` 转换到 `[0,1]`。XYZ 或带 RB 的 pitch 与其他运动同时输入时，
笛卡尔坐标优先；腕转和夹爪不会与同一个笛卡尔 tick 同时发送。

当前验收配置的满幅速度为平移 `1.5 cm/s`、俯仰 `5 deg/s`、腕转
`20 deg/s`、夹爪归一化行程 `0.5/s`。

遥操层不再对 `x/y/z` 做矩形工作空间钳制，坐标目标会持续按摇杆输入累计；
`pitch` 和腕转均限制在 `[-90, 90] deg`。XYZ 是否可达最终由 STM32 的 IK 和关节角约束
决定。无解目标会被固件拒绝，遥操回退到最后一个成功目标并停止发送；摇杆和扳机
回中后自动恢复，不需要清错或 Home。因此实机操作必须清空工作空间并保持 B 急停
可用。

## 状态与安全

- 启动不运动，先等待有效关节反馈并核验固件复位姿态。
- 固件标称复位目标为 `(15, 0, 2) cm`，反解初始 `pitch=-54.48 deg`。
- A 使能要求：输入新鲜、摇杆与扳机中立、坐标已同步、机械臂非
  `MOVING/ERROR/ESTOP`。
- 再按 A 只停止发送新目标，不发送 STOP，且保留坐标同步。
- Joy 超时也不会发送 STOP，但会关闭遥控并使坐标失去同步；重新控制前必须
  执行回零。
- 急停或锁存错误会使坐标失去同步。解除锁存后必须单独执行回零，回零成功后
  才能再次按 A。`NO_IK_SOLUTION` 是例外：只回退目标并等待控制输入回中。
- Home 固定依次打开 1 号夹爪、把 2 号腕转舵机恢复到 `0 rad`，再执行 6～3 号
  舵机的笛卡尔 Home；任一阶段失败都不会继续下一阶段。若夹爪反馈已连续 3 帧处于
  安全开度（规范位置不超过 `0.10`），但打开动作仍未报告完成，遥操会先发送单夹爪
  `MODE_GRIPPER_STOP`，确认停止后再继续腕转 Home，避免卡在机械端点。
- Xbox 遥控默认向 STM32 发送 `90 ms` 夹爪动作；RT 松开使用独立的单夹爪
  `MODE_GRIPPER_STOP`，不再把可能滞后的反馈位置重新下发。未填写时长的旧调用者仍
  使用兼容默认值 `1000 ms`。

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
`/arm/teleop_enabled` 核对状态。急停恢复顺序固定为：松开 B、长按
LB+RB+X、长按 LB+RB+Y 等待回零完成、摇杆中立后按 A。
