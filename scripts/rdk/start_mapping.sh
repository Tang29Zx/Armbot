#!/bin/bash
# Armbot 手动建图全链路一键启动脚本
# 用法: bash ~/start_mapping.sh
set -e

# 8-29: 先杀残留进程（反复启动不清理会导致多雷达进程叠加→时间戳混乱→slam 丢帧→地图漂移）
bash ~/stop_all.sh > /dev/null 2>&1 || true
sleep 2

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/armbot-slam/ros2_ws/install/setup.bash

LOG=/tmp/mapping
mkdir -p $LOG

echo "=== [1/5] 底盘 ==="
nohup ros2 launch chassis_control chassis_control.launch.py > $LOG/chassis.log 2>&1 < /dev/null &
sleep 5

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

echo "=== [2/5] 官方雷达 ($LIDAR_PORT, singleChannel) ==="
nohup ros2 run ydlidar ydlidar_node \
  --ros-args \
  -p port:=$LIDAR_PORT \
  -p frame_id:=laser \
  -p baudrate:=115200 \
  -p singleChannel:=true \
  -p angle_min:=-180.0 -p angle_max:=180.0 \
  -p frequency:=5.0 \
  > $LOG/lidar.log 2>&1 < /dev/null &
sleep 6

echo "=== [3/5] 静态 tf (map->odom, base_link->laser) ==="
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > $LOG/tf1.log 2>&1 < /dev/null &
# 8-29: 曾误判雷达装反加 yaw=π，实测车头指示反向，撤销恢复 yaw=0
nohup ros2 run tf2_ros static_transform_publisher 0.175 0 0.18 0 0 0 base_link laser > $LOG/tf2.log 2>&1 < /dev/null &
sleep 2

echo "=== [4/5] slam_toolbox ==="
nohup ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file ~/armbot-slam/ros2_ws/install/armbot_bringup/share/armbot_bringup/config/slam_toolbox_params.yaml \
  > $LOG/slam.log 2>&1 < /dev/null &
sleep 8

echo "=== [5/5] Web 控制页 ==="
nohup python3 ~/armbot_web.py > $LOG/web.log 2>&1 < /dev/null &
sleep 4

echo ""
echo "=== 启动完成，验证 ==="
timeout 4 ros2 topic hz /odom 2>&1 | head -1 || true
timeout 4 ros2 topic hz /scan 2>&1 | head -1 || true
curl -s -m 3 -o /dev/null -w "Web: HTTP %{http_code}\n" http://127.0.0.1:8080/ || echo "Web: 未响应"
echo "全部就绪!"
