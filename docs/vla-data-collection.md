# VLA 数据采集与 LeRobot 转换规格

状态：rosbag2 采集、人工 review 和 OpenPI/LeRobot v2.1 离线转换已实现
日期：2026-08-07

## 用户目标

在不改变机械臂控制链的前提下，把相机、操作者输入、请求动作和真实状态保存为可
追溯的 episode。rosbag2 是原始事实来源；训练框架格式由独立离线转换器生成。

## 本阶段范围

新增独立的被动采集包 `vla_dataset`：

```text
/image                    sensor_msgs/msg/CompressedImage
/joy                      sensor_msgs/msg/Joy
/arm/command              action_interfaces/msg/ArmCommand
/arm/state                action_interfaces/msg/ArmState
/arm/state_filtered       action_interfaces/msg/ArmState
        -> ros2 bag record
        -> <episode>/bag/
        -> <episode>/manifest.json
```

采集器只创建 rosbag 订阅，不发布运动命令、不打开 I2C，也不启动或停止控制节点。

外部 API 只有 ROS 2 Humble `ros2bag` 包提供的 `ros2 bag record` CLI：输入为输出
目录和 topic 列表，输出为 sqlite3 bag 与 `metadata.yaml`，本项目用它保存原始 ROS
消息。最小示例和停止语义见 [ROS 2 Humble rosbag2 官方教程](https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-And-Playing-Back-Data.html)。

## 命令契约

```bash
ros2 run vla_dataset record_episode \
  --task "抓取红色方块"
```

按 `Ctrl+C` 结束录制。结束只表示文件完整关闭，不代表示范成功；新 episode 的结果
固定为 `unreviewed`，避免未经人工或质检确认的数据进入训练集。

可配置项：

- `--output-root`：episode 根目录，默认 `~/vla_episodes`；
- `--image-topic`：默认 `/image`；
- `--topic`：追加必录话题之外的 topic；
- `--firmware-sha256`：可选的实际烧录固件 SHA-256；省略时 manifest 记录为
  `unknown`。

## Manifest 契约

`manifest.json` 至少记录：

- schema 版本、episode ID、任务文本；
- UTC 开始/结束时间和持续时长；
- 固定 topic 列表及 bag 相对路径；
- `recording/unreviewed/success/failure/failed` 状态和 recorder 退出码；
- Armbot Git commit 与 dirty 标记；
- STM32 固件 SHA-256；
- 主机名和 ROS distribution。

manifest 使用临时文件加原子替换写入。启动 rosbag 前先写 `recording`，正常关闭后写
`unreviewed`；启动失败或 rosbag 异常退出写 `failed`。突然断电时遗留的
`recording` 明确代表不完整 episode。

人工检查抓取成功后使用：

```bash
ros2 run vla_dataset review_episode ~/vla_episodes/<episode> \
  --result success \
  --notes "药盒已夹紧并抬起"
```

失败示范使用 `--result failure`。review 只接受 `unreviewed`，不会覆盖已有 review；
它只原子更新 manifest，不修改 rosbag。

## OpenPI/LeRobot 转换

转换在训练机的 OpenPI Python 环境运行，不在 RDK 安装 LeRobot。OpenPI 锁定的
LeRobot 版本负责生成 parquet、图像和统计元数据，`rosbags` 只读解析 ROS 2 Humble
sqlite3 bag。示例：

```bash
cd /path/to/openpi
uv run \
  --with 'rosbags>=0.10,<0.12' \
  --with /path/to/Armbot/ros2_ws/src/vla_dataset \
  export_lerobot \
  /path/to/vla_data/raw/<episode> \
  --output /path/to/vla_data/lerobot/armbot_pi05 \
  --repo-id local/armbot_pi05 \
  --message-dir /path/to/Armbot/ros2_ws/src/action_interfaces/msg
```

`unreviewed` 数据默认拒绝。仅为了验证管线可增加 `--allow-unreviewed`，生成物不能直接
进入正式训练。输出目录必须不存在；转换先写同目录临时目录，全部成功后再原子改名，
不会覆盖已有 dataset。

批量实采数据不修改原始 manifest，而使用外部 QC 处理清单统一选择和派生：

```bash
export HF_LEROBOT_HOME=/home/tang/Projects/armbot/vla_data/lerobot

export_lerobot \
  --processing-plan /home/tang/Projects/armbot/vla_data/qc/openpi_processing_plan.json \
  --output "$HF_LEROBOT_HOME/local/armbot_pi05_fixed_pick_42" \
  --repo-id local/armbot_pi05_fixed_pick_42
```

处理清单必须覆盖全部原始 episode，并将每条标为 `success_usable`、`success_crop`、
`discard` 或 `out_of_scope`。`success_crop` 的命令序号、错误状态或损坏图像 SHA-256
剔除规则会原样复制到输出元数据；导出器只接收清单里的成功项，并要求实际 episode
集合与清单完全一致。

2026-08-07 本轮定点药盒抓取最终处理 61 条原始 episode：28 条直接使用、14 条审计
派生、16 条丢弃、3 条不属于本批次。正式数据集含 42 episodes、15,625 帧，位于
`/home/tang/Projects/armbot/vla_data/lerobot/local/armbot_pi05_fixed_pick_42`。OpenPI
锁定的 LeRobot 提交 `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5` 已在标准缓存布局下
无显式 `root` 读回 10 步动作块；详细结果见
`/home/tang/Projects/armbot/vla_data/qc/openpi_export_verification.json`。

固定数据契约为：

```text
fps = 10
observation.images.front = uint8 RGB [3, 224, 224]
observation.state = float32[6]
  [joint_1_rad..joint_5_rad, gripper_absolute]
action = float32[6]
  [delta_x_cm, delta_y_cm, delta_z_cm, delta_pitch_deg,
   delta_wrist_roll_rad, gripper_absolute]
task = manifest.task
```

图像按比例缩放并补黑边，不拉伸；原始 1280×720 JPEG 继续保留在 rosbag。动作前五维
表示下一个 100 ms 区间的保持目标变化，夹爪维度保持绝对值。转换要求普通动作具有固件
执行回执；流式目标若在 250 ms 内被同控制族的新目标连续替换，并在 1 秒内由已确认目标
或结束/停止命令收束，则按控制器的最新目标队列语义记录为 `superseded`，不会误判坏包。
转换仍会拒绝：bag/metadata 缺失、无法解释的动作回执缺失、动作区间内错误或急停、
持续无效/过期状态、过期图像、非法数值以及完全没有目标变化的 episode。孤立的
`position_valid=false` 样本只允许因果回退到最近有效状态；正常状态受 150 ms 最大年龄
限制，单点回退额外允许一个 100 ms 采样周期。连续两个无效状态仍会拒绝转换。转换
报告会记录无效消息数和实际发生回退的帧数。

默认按 rosbag 到达时间因果对齐，因为这与 RDK policy client 实际取得消息的时序一致；
相机和状态的 header-to-bag 延迟仍会写入
`meta/armbot_conversion.json`。若推理端显式按 capture time 补偿，可使用
`--clock header` 生成对照数据，但同一训练集不能混用两种时钟策略。

## 验收标准

- 空任务、非法固件哈希和重复 topic 在启动 rosbag 前被拒绝或规范化；
- 输出目录不会覆盖已有 episode；
- `Ctrl+C` 会先让 rosbag2 完成收尾，再更新 manifest；
- rosbag2 异常退出时返回非零并保留失败原因；
- 采集器源码不导入控制节点，不访问 `/dev/i2c-*`；
- 单元测试覆盖命令生成、参数验证和 manifest 状态转换；
- RDK 实机试采后，`ros2 bag info` 能看到五个必录话题和图像消息。
- LeRobot loader 可读回 224×224 图像、6 维状态和 `[action_horizon, 6]` 动作块；
- `meta/armbot_conversion.json` 的帧数、时钟策略和源延迟与输入 episode 一致。

## 风险与约束

- 正式采集前，RDK 的 `dpkg --audit` 必须无输出，内核日志不得出现新的 ext4/I/O
  错误。文件系统或 ROS 消息包校验失败时立即停止采集；rosbag 能关闭不代表落盘数据
  可信。
- `/arm/state.header.stamp` 是 RDK 发布时刻，不是每个舵机的 MCU 采样时刻；v1 先
  保存原始消息与 rosbag 接收时间，延迟由后续 QC 实测，不能宣称硬件同步。
- 相机发布端必须提供单调时间戳。仅有 SRT/H.264 推流不等于可训练的图像 topic。
- rosbag 完整不等于示范有效；`position_valid=false`、错误、急停、拒绝动作和图像
  长间隔由下一阶段 QC 拒绝。
- 两个仓库存在未提交修改时，manifest 的 `dirty=true` 只用于试采，不得进入正式
  数据集。

## 第一阶段非目标

- 本阶段不修改 STM32/I2C 协议；
- 第一阶段 recorder 不直接写 LeRobot；转换仍保持为独立只读离线步骤；
- 不自动判断任务成功；
- 不自动启动机械臂、相机或遥控节点；
- 不上传、删除或覆盖已有数据。

## 后续任务

1. 增加转换后同步视频和 state/action 曲线可视化；
2. 实现 OpenPI Armbot Inputs/Outputs、DataConfig 和 TrainConfig；
3. 计算 norm stats 并完成单 episode 过拟合测试；
4. 若实测同步抖动超标，再评估 MCU telemetry 序号、时间戳和逐舵机有效位。

## 回滚

停止采集器并移除 `vla_dataset` 包即可；控制接口、launch、I2C 和固件均未改变。
