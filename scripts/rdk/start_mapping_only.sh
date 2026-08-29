#!/bin/bash
# 8-18: 只启动建图进程（雷达+scan_filter+底盘+tf+slam）
# 由 Web "开始建图" 按钮调用：不杀 armbot_web、不启动 Web（Web 已在跑）

# 并发锁：同一时刻只允许一个脚本实例（防止按钮连点导致多套进程互踩）
# 8-29: 用 PID 文件防重——flock 的 fd 会被 nohup 子进程继承导致锁永不释放；
#       pgrep 匹配会被 $( ) 子 shell 干扰误判，PID 文件最稳
LOCKFILE=/tmp/start_mapping_only.pid
if [ -f "$LOCKFILE" ]; then
  OLDPID=$(cat "$LOCKFILE" 2>/dev/null)
  if [ -n "$OLDPID" ] && kill -0 "$OLDPID" 2>/dev/null; then
    echo "已有建图脚本在运行 (PID $OLDPID)，忽略本次"
    exit 0
  fi
  rm -f "$LOCKFILE"
fi
echo $$ > "$LOCKFILE"
trap 'rm -f "$LOCKFILE"' EXIT

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/armbot-slam/ros2_ws/install/setup.bash

LOG=/tmp/explore_run
mkdir -p $LOG

echo "=== kill 建图相关残留（保留 armbot_web）==="
ps aux | grep -E "chassis_control|ydlidar|async_slam|static_transform|scan_filter" \
  | grep -v grep | grep -vE "armbot_web|bash|start_mapping" \
  | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 3

# 自动检测雷达串口：优先按 CP210x 芯片(VID 10c4)识别，兜底用唯一 ttyUSB
LIDAR_PORT=""
for dev in /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  if udevadm info --query=property --name="$dev" 2>/dev/null | grep -q "ID_VENDOR_ID=10c4"; then
    LIDAR_PORT="$dev"; break
  fi
done
[ -z "$LIDAR_PORT" ] && LIDAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)
[ -z "$LIDAR_PORT" ] && { echo "!!! 未找到雷达串口 /dev/ttyUSB*，中止"; exit 1; }
echo "=== [1/5] 雷达 $LIDAR_PORT ==="
nohup ros2 run ydlidar ydlidar_node \
  --ros-args -p port:=$LIDAR_PORT -p frame_id:=laser -p baudrate:=115200 \
  -p singleChannel:=true -p angle_min:=-180.0 -p angle_max:=180.0 -p frequency:=5.0 \
  > $LOG/lidar.log 2>&1 < /dev/null &
sleep 6

echo "=== [2/5] 雷达已启动，建图不需要 scan_filter（导航才用双话题）==="

echo "=== [3/5] 底盘 ==="
nohup ros2 launch chassis_control chassis_control.launch.py > $LOG/chassis.log 2>&1 < /dev/null &
sleep 5

echo "=== [4/5] static tf (base_link->laser x=0.175) ==="
# 不加 static map->odom seed——与 slam 动态 map->odom 冲突
nohup ros2 run tf2_ros static_transform_publisher 0.175 0 0.18 0 0 0 base_link laser > $LOG/tf2.log 2>&1 < /dev/null &
sleep 2

echo "=== [5/5] slam ==="
nohup ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args --params-file ~/armbot-slam/ros2_ws/install/armbot_bringup/share/armbot_bringup/config/slam_toolbox_params.yaml \
  > $LOG/slam.log 2>&1 < /dev/null &
sleep 8

echo "=== 建图进程已启动（Web 保持运行）==="
echo "检查: ps aux | grep -E 'ydlidar|slam_toolbox|chassis_control|scan_filter'"
