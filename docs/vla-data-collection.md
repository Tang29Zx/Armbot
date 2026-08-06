# VLA 数据采集 v1 规格

状态：第一阶段已实现并通过本机 rosbag2 烟雾测试；RDK 实录因根文件系统损坏暂停  
日期：2026-08-03

## 用户目标

在不改变机械臂控制链的前提下，把相机、操作者输入、请求动作和真实状态保存为可
追溯的 episode。rosbag2 是原始事实来源；训练框架格式由后续离线转换器生成。

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
- `recording/unreviewed/failed` 状态和 recorder 退出码；
- Armbot Git commit 与 dirty 标记；
- STM32 固件 SHA-256；
- 主机名和 ROS distribution。

manifest 使用临时文件加原子替换写入。启动 rosbag 前先写 `recording`，正常关闭后写
`unreviewed`；启动失败或 rosbag 异常退出写 `failed`。突然断电时遗留的
`recording` 明确代表不完整 episode。

## 验收标准

- 空任务、非法固件哈希和重复 topic 在启动 rosbag 前被拒绝或规范化；
- 输出目录不会覆盖已有 episode；
- `Ctrl+C` 会先让 rosbag2 完成收尾，再更新 manifest；
- rosbag2 异常退出时返回非零并保留失败原因；
- 采集器源码不导入控制节点，不访问 `/dev/i2c-*`；
- 单元测试覆盖命令生成、参数验证和 manifest 状态转换；
- RDK 实机试采后，`ros2 bag info` 能看到五个必录话题和图像消息。

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

## 非目标

- 本阶段不修改 STM32/I2C 协议；
- 不实现 LeRobot、RLDS 等特定格式转换；
- 不自动判断任务成功；
- 不自动启动机械臂、相机或遥控节点；
- 不上传、删除或覆盖已有数据。

## 后续任务

1. 增加离线 QC：topic 覆盖、FPS、时间戳、反馈有效率、错误和 action ACK 延迟；
2. 增加显式 review 命令，把 `unreviewed` 标为 `success` 或 `failure`；
3. 根据最终 VLA 模型增加只读转换器；
4. 若实测同步抖动超标，再评估 MCU telemetry 序号、时间戳和逐舵机有效位。

## 回滚

停止采集器并移除 `vla_dataset` 包即可；控制接口、launch、I2C 和固件均未改变。
