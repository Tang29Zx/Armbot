#!/usr/bin/env python3
"""map_relay: 订阅 /map，只把 map_server 的完整静态图转发到 /map_static。
slam localization 会持续发布局部图到 /map（尺寸可能比静态图还大，如 map_save 230x98
vs slam 局部 283x157）——"更大图"过滤会选错。改为：启动延迟 2s（等 map_server 激活
发布 latched 图）后，只转发收到的第一张图并锁定，之后全部忽略。
"""
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

# TRANSIENT_LOCAL：map_server 的 /map 是 latched，新订阅者能收到
MAP_QOS = rclpy.qos.QoSProfile(depth=1, durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)

class MapRelay(Node):
    def __init__(self):
        super().__init__('map_relay')
        self.pub = self.create_publisher(OccupancyGrid, '/map_static', MAP_QOS)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.cb, MAP_QOS)
        self.locked = False
        self.n = 0
        self.t0 = time.time()

    def cb(self, msg):
        # 启动 2s 内不接收（等 map_server 激活；slam 有 6s TimerAction 延迟，先到的一定是 map_server 的图）
        if time.time() - self.t0 < 2.0:
            return
        if self.locked:
            return  # 已锁定 map_server 完整图，slam 局部图一律忽略
        w, h = msg.info.width, msg.info.height
        self.locked = True
        self.pub.publish(msg)
        self.n += 1
        print(f'[map_relay] 锁定并转发 {w}x{h}（第{self.n}次）', flush=True)

def main():
    rclpy.init()
    n = MapRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
