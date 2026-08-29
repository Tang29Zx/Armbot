#!/bin/bash
# ============================================================
# Armbot Web 导航启动脚本 (2026-08-29)
# 由 Web "启动导航" 按钮调用：杀残留但保留 armbot_web、不重启 Web
# 基于 start_nav.sh 裁剪（去掉 web 启动段）
# 用法: ~/start_nav_web.sh [map_verify|map_final|map_auto|map_save]
# ============================================================
MAP_NAME="${1:-map_verify}"
MAP_NAME="${MAP_NAME%.yaml}"            # 去掉可能的 .yaml 后缀
MAP_FILE="/home/sunrise/${MAP_NAME}.yaml"
LOG_DIR=/tmp/nav_run
mkdir -p $LOG_DIR

export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/armbot-slam/ros2_ws/install/setup.bash

# 自动检测雷达串口（CP210x 优先）
LIDAR_PORT=""
for dev in /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  if udevadm info --query=property --name="$dev" 2>/dev/null | grep -q "ID_VENDOR_ID=10c4"; then
    LIDAR_PORT="$dev"; break
  fi
done
[ -z "$LIDAR_PORT" ] && LIDAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | head -1)

[ -f "$MAP_FILE" ] || { echo "!!! 地图不存在: $MAP_FILE"; exit 1; }
echo "地图: $MAP_FILE | 雷达: ${LIDAR_PORT:-未找到}"

echo "=== [1/5] 杀残留（保留 armbot_web）==="
ps aux | grep -E "nav2_|map_server|localization_slam|ydlidar|chassis_control|scan_filter|robot_state_pub|map_relay|static_transform|slam_toolbox" \
  | grep -v grep | grep -vE "armbot_web|bash|start_nav|start_mapping" \
  | awk '{print $2}' | xargs -r kill -9 2>/dev/null
sleep 5

echo "=== [2/5] 单独启动底盘 ==="
nohup ros2 launch chassis_control chassis_control.launch.py > $LOG_DIR/chassis.log 2>&1 < /dev/null &
sleep 8

echo "=== [3/5] 验证 odom->base_link tf（失败重试一次）==="
check_tf() {
  timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -m1 "Translation" > /dev/null 2>&1
}
if check_tf; then
  echo "tf OK"
else
  echo "tf 未就绪，杀底盘重启..."
  ps aux | grep "[c]hassis_control_node" | awk '{print $2}' | xargs -r kill -9 2>/dev/null
  sleep 4
  nohup ros2 launch chassis_control chassis_control.launch.py > $LOG_DIR/chassis.log 2>&1 < /dev/null &
  sleep 8
  check_tf && echo "tf OK（重试成功）" || echo "⚠️ tf 仍失败，继续启动"
fi

echo "=== [4/5] 启动 nav.launch（$MAP_FILE）==="
nohup ros2 launch armbot_bringup nav.launch.py \
  map:=$MAP_FILE use_rviz:=false lidar_port:=$LIDAR_PORT \
  > $LOG_DIR/nav.log 2>&1 < /dev/null &
echo "等待 60 秒（map_server 激活 + slam 加载）..."
sleep 60

echo "=== [5/5] 验证 ==="
echo "崩溃数: $(grep -cE 'process has died|Segmentation|Mapper Error' $LOG_DIR/nav.log 2>/dev/null)"
echo "slam: $(ps aux | grep -c '[l]ocalization_slam')"
echo "Nav2核心: $(ps aux | grep -cE '[n]av2_controller|[b]t_navigator')"
echo "底盘: $(ps aux | grep -c '[c]hassis_control_node')"
echo "雷达: $(ps aux | grep -c '[y]dlidar_node')"
timeout 6 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -m1 "Translation" || echo "⚠️ map 帧未就绪（稍后设初始位姿即可）"
echo "=== 导航启动完成（Web 保持运行）==="
