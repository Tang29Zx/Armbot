#!/bin/bash
# Armbot 纯雷达测试模式（无需底盘供电）
# 用法: bash ~/test_lidar.sh
# 说明: 底盘不开时无 odom->base_link tf，用静态 0 变换代替，仅用于雷达/SLAM/方向验证
set -e

bash ~/stop_all.sh > /dev/null 2>&1 || true
sleep 2

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/armbot-slam/ros2_ws/install/setup.bash

LOG=/tmp/lidar_test
mkdir -p $LOG

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

echo "=== [1/4] 官方雷达 ($LIDAR_PORT, singleChannel) ==="
nohup ros2 run ydlidar ydlidar_node \
  --ros-args \
  -p port:=$LIDAR_PORT \
  -p frame_id:=laser \
  -p baudrate:=115200 \
  -p singleChannel:=true \
  -p angle_min:=-180.0 -p angle_max:=180.0 \
  -p frequency:=8.0 \
  > $LOG/lidar.log 2>&1 < /dev/null &
sleep 6

echo "=== [2/4] 静态 tf (map->odom, odom->base_link 假, base_link->laser) ==="
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom > $LOG/tf1.log 2>&1 < /dev/null &
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link > $LOG/tf2.log 2>&1 < /dev/null &
# 8-29: 曾误判雷达装反加 yaw=π，实测车头指示反向，撤销恢复 yaw=0（参数顺序 x y z yaw pitch roll）
nohup ros2 run tf2_ros static_transform_publisher 0.175 0 0.18 0 0 0 base_link laser > $LOG/tf3.log 2>&1 < /dev/null &
sleep 2

echo "=== [3/4] slam_toolbox ==="
nohup ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file ~/armbot-slam/ros2_ws/install/armbot_bringup/share/armbot_bringup/config/slam_toolbox_params.yaml \
  > $LOG/slam.log 2>&1 < /dev/null &
sleep 8

echo "=== [4/4] Web 控制页 ==="
nohup python3 ~/armbot_web.py > $LOG/web.log 2>&1 < /dev/null &
sleep 4

echo ""
echo "=== 启动完成 ==="
timeout 4 ros2 topic hz /scan 2>&1 | head -1 || true
timeout 4 ros2 topic echo /map --once 2>&1 | grep -E 'width|height' | head -2 || echo "map 未出（等几秒后刷新 Web）"
curl -s -m 3 -o /dev/null -w "Web: HTTP %{http_code}\n" http://127.0.0.1:8080/ || echo "Web: 未响应"
echo "浏览器打开: http://192.168.3.147:8080"
