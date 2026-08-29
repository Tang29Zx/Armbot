import rclpy
import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MIN_RANGE = 0.05      # 无效点阈值
# 双输出设计（8-22 15:18 定稿）：
#   /scan_filtered → costmap（重滤 1.5m：避开雷达自身干扰区/假障碍环，保证 planner 可达）
#   /scan_slam     → slam 定位（轻滤 0.9m：保留 0.9-1.5m 环境特征，保证定位不漂移）
FILTER_RANGE = 1.5    # costmap 版过滤半径（雷达自身干扰区：固定角度 0 点 + 0.1-0.7m 近距点）
SLAM_RANGE = 0.9      # slam 版过滤半径（仅滤车体/无效 0 点，保留特征给定位）
BODY_ANGLE = 45.0     # 正后方 ±45°（车体）
BODY_RANGE = 1.5      # 车体最大距离（costmap 版）
# 实测 15:15：雷达原始数据每帧 60-66 个 0.00m 无效点（左后 -180~-160 + 前方 -30~15 固定角度），
# 右后 146° 有 0.10m 极近点，全周 <1.5m 密集点 —— 雷达自身/安装结构干扰，非环境。
# 单独滤 1.5m 会让 slam 特征不足（定位漂移 1.6m），故分双话题。

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        # 发布用默认 QoS(RELIABLE)——slam 无论订阅 RELIABLE 还是 BEST_EFFORT 都能收到
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.pub_slam = self.create_publisher(LaserScan, '/scan_slam', 10)
        # 8-29 建图专用：只滤 0m 假点，保留全部真实环境点（还原 8-17 首次建图的干净雷达输入）
        self.pub_map = self.create_publisher(LaserScan, '/scan_mapping', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)
        self.n = 0

    def cb(self, msg):
        # costmap 版：滤 <1.5m
        r_cost = list(msg.ranges)
        # slam 版：滤 <0.9m
        r_slam = list(msg.ranges)
        nf = 0
        for i, r in enumerate(msg.ranges):
            # 0.0m 无效点（雷达自身）必须显式滤掉：否则进 costmap 标记车位置(0m)，
            # obstacle 层 raytrace 清除从 0.1m 开始→车位置标记永不清除→planner 起点恒为障碍！
            if r <= MIN_RANGE or r < FILTER_RANGE or (r < BODY_RANGE and abs(math.degrees(msg.angle_min + i * msg.angle_increment)) > 180.0 - BODY_ANGLE):
                r_cost[i] = float('inf')
                nf += 1
            if r <= MIN_RANGE or r < SLAM_RANGE:
                r_slam[i] = float('inf')
        m1 = msg
        m1.ranges = r_cost
        self.pub.publish(m1)
        m2 = msg
        m2.ranges = r_slam
        self.pub_slam.publish(m2)
        # 建图版：只滤 0m 假点，其余全保留（8-29）
        r_map = [float('inf') if r <= MIN_RANGE else r for r in msg.ranges]
        m3 = msg
        m3.ranges = r_map
        self.pub_map.publish(m3)
        self.n += 1
        if self.n % 40 == 0:
            print(f'[scan_filter] frames={self.n} filtered={nf}', flush=True)

def main():
    rclpy.init()
    n = ScanFilter()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
